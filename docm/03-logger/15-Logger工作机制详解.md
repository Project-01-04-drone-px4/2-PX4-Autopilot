# Logger工作机制详解

## 📋 概述

本文档详细解析PX4 Logger的核心工作机制，包括：
- 定时轮询的实现原理
- 缓冲区架构
- 数据写入SD卡的真实流程

---

## 🔄 一、定时轮询触发机制

### 1.1 高精度定时器 + 信号量

Logger使用**高精度定时器(HRT)** + **信号量(Semaphore)**实现定时轮询：

```cpp
// src/modules/logger/logger.cpp:685
hrt_call_every(&timer_call, _log_interval, _log_interval, timer_callback, &_timer_callback_data);
```

**关键参数**：
- `_log_interval`：默认 **3500 微秒(3.5 ms)**
- 可通过参数 `SDLOG_PROFILE` 配置

### 1.2 定时器回调函数

```cpp
// src/modules/logger/logger.cpp:93
static void timer_callback(void *arg)
{
    /* Note: we are in IRQ context here (on NuttX) */
    
    Logger::timer_callback_data_s *data = (Logger::timer_callback_data_s *)arg;
    
    int semaphore_value = 0;
    px4_sem_getvalue(&data->semaphore, &semaphore_value);
    
    /* 防止信号量溢出（如果Logger主循环跟不上定时器速度） */
    bool semaphore_value_saturated = semaphore_value > 100;
    
    if (semaphore_value_saturated) {
        return;  // 不再增加信号量
    }
    
    px4_sem_post(&data->semaphore);  // 唤醒Logger主循环
}
```

**工作原理**：
1. 每隔 **3.5 ms**，定时器在**中断上下文**中触发
2. 检查信号量值，防止溢出（如果Logger处理太慢）
3. 调用 `sem_post()` 唤醒Logger主线程

### 1.3 Logger主循环等待

```cpp
// src/modules/logger/logger.cpp:920-927
/*
 * We wait on the semaphore, which periodically gets updated by a high-resolution timer.
 * The simpler alternative would be:
 *   usleep(max(300, _log_interval - elapsed_time_since_loop_start));
 * And on linux this is quite accurate as well, but under NuttX it is not accurate,
 * because usleep() has only a granularity of CONFIG_MSEC_PER_TICK (=1ms).
 */
while (px4_sem_wait(&_timer_callback_data.semaphore) != 0) {}
```

**为什么不用 `usleep()`？**
- NuttX的`usleep()`精度只有 **1 ms**（CONFIG_MSEC_PER_TICK）
- 使用HRT定时器+信号量可达到**微秒级精度**

---

## 💾 二、缓冲区架构

### 2.1 双缓冲系统

Logger使用**两个独立的环形缓冲区**：

```cpp
// src/modules/logger/log_writer_file.h:221
LogFileBuffer _buffers[(int)LogType::Count];

enum class LogType {
    Full = 0,    // 完整日志（所有主题）
    Mission,     // 任务日志（用于geotagging等）
    Count
};
```

### 2.2 缓冲区大小

```cpp
// src/modules/logger/log_writer_file.cpp:64-70
LogWriterFile::LogWriterFile(size_t buffer_size)
    : _buffers{
    // Full日志缓冲区
    {
        buffer_size,                    // 默认：12 KB (SDLOG_MODE=0)
        _min_write_chunk + 300,         // 最小：4096 + 300 = 4396 字节
        perf_alloc(PC_ELAPSED, "logger_sd_write"), 
        perf_alloc(PC_ELAPSED, "logger_sd_fsync")
    },
    // Mission日志缓冲区
    {...}
}
```

**关键常量**：
```cpp
// src/modules/logger/log_writer_file.h:170
/* 512 didn't seem to work properly, 4096 should match the FAT cluster size */
static constexpr size_t _min_write_chunk = 4096;
```

**缓冲区大小配置**：
| SDLOG_PROFILE | 缓冲区大小 | 用途 |
|---------------|-----------|------|
| 0 (默认)       | 12 KB     | 低延迟 |
| 1 (高吞吐)     | 64 KB     | 最大吞吐量 |
| 2 (低延迟)     | 8 KB      | 最低延迟 |

### 2.3 环形缓冲区结构

```cpp
// src/modules/logger/log_writer_file.h:210-218
class LogFileBuffer {
private:
    size_t _buffer_size;          // 缓冲区总大小
    uint8_t *_buffer = nullptr;   // 环形缓冲区数组
    size_t _head = 0;             // 写指针（下一个写入位置）
    size_t _count = 0;            // 缓冲区中待写入SD卡的字节数
    size_t _total_written = 0;    // 已写入SD卡的总字节数
    int _fd = -1;                 // 文件描述符
};
```

**环形缓冲区示意图**：
```
           _head
             ↓
[已写SD|空闲空间|待写SD]
        ↑
    read_ptr = (_head - _count) % _buffer_size
```

---

## 📝 三、数据流转全流程

### 3.1 主线程写入缓冲区

```cpp
// src/modules/logger/logger.cpp:749-857
while (_writer_thread_should_exit.load() == false) {
    
    /* 1. 获取缓冲区锁 */
    _writer.lock();
    
    /* 2. 遍历所有订阅的主题 */
    for (int sub_idx = 0; sub_idx < _num_subscriptions; ++sub_idx) {
        LoggerSubscription &sub = _subscriptions[sub_idx];
        
        /* 3. 检查主题是否有更新 */
        if (copy_if_updated(sub_idx, _msg_buffer + sizeof(ulog_message_data_s), try_to_subscribe)) {
            
            /* 4. 构造ULog消息头 */
            const size_t msg_size = sizeof(ulog_message_data_s) + sub.get_topic()->o_size_no_padding;
            const uint16_t write_msg_size = static_cast<uint16_t>(msg_size - ULOG_MSG_HEADER_LEN);
            const uint16_t write_msg_id = sub.msg_id;
            
            _msg_buffer[0] = (uint8_t)write_msg_size;
            _msg_buffer[1] = (uint8_t)(write_msg_size >> 8);
            _msg_buffer[2] = static_cast<uint8_t>(ULogMessageType::DATA);
            _msg_buffer[3] = (uint8_t)write_msg_id;
            _msg_buffer[4] = (uint8_t)(write_msg_id >> 8);
            
            /* 5. 写入环形缓冲区 */
            if (write_message(LogType::Full, _msg_buffer, msg_size)) {
                total_bytes += msg_size;
            }
        }
    }
    
    /* 6. 释放锁 */
    _writer.unlock();
    
    /* 7. 通知writer线程 */
    _writer.notify();
    
    /* 8. 等待下一次定时器触发 */
    while (px4_sem_wait(&_timer_callback_data.semaphore) != 0) {}
}
```

### 3.2 写入环形缓冲区

```cpp
// src/modules/logger/log_writer_file.cpp:534-561
int LogWriterFile::write(LogType type, void *ptr, size_t size, uint64_t dropout_start)
{
    if (!is_started(type)) {
        return 0;
    }
    
    // 检查缓冲区可用空间
    size_t available = _buffers[(int)type].available();
    
    if (size + dropout_size > available) {
        // 缓冲区溢出！
        return -1;
    }
    
    // 写入环形缓冲区（不检查，因为已经检查过空间）
    _buffers[(int)type].write_no_check(ptr, size);
    return 0;
}
```

**环形缓冲区写入实现**：
```cpp
// src/modules/logger/log_writer_file.cpp:598-619
void LogWriterFile::LogFileBuffer::write_no_check(void *ptr, size_t size)
{
    size_t n = _buffer_size - _head;  // 到缓冲区末尾的字节数
    
    uint8_t *buffer_c = static_cast<uint8_t *>(ptr);
    
    if (size < n) {
        /* 数据可以连续写入 */
        memcpy(&(_buffer[_head]), buffer_c, size);
        _head = (_head + size) % _buffer_size;
    } else {
        /* 数据需要分两段写入（环绕） */
        memcpy(&(_buffer[_head]), buffer_c, n);  // 写入到缓冲区末尾
        
        size_t p = size - n;  // 剩余字节
        memcpy(&(_buffer[0]), &(buffer_c[n]), p);  // 从缓冲区开头继续写
        _head = (_head + p) % _buffer_size;
    }
    
    _count += size;  // 增加待写入SD卡的字节数
}
```

### 3.3 Writer线程批量写入SD卡

**Writer线程是独立线程**，专门负责将缓冲区数据写入SD卡：

```cpp
// src/modules/logger/log_writer_file.cpp:328-494
void LogWriterFile::run()
{
    while (!_exit_thread.load()) {
        
        /* 1. 等待缓冲区有数据 */
        pthread_mutex_lock(&_mtx);
        pthread_cond_wait(&_cv, &_mtx);  // 等待主线程notify()
        
        int poll_count = 0;
        hrt_abstime last_fsync = hrt_absolute_time();
        
        while (true) {
            
            const hrt_abstime now = hrt_absolute_time();
            
            /* 2. 周期性调用fsync（确保数据持久化） */
            const bool call_fsync = ++poll_count >= 100 || 
                                    now - last_fsync > 1_s || 
                                    _want_fsync.load();
            
            if (call_fsync) {
                last_fsync = now;
                poll_count = 0;
            }
            
            /* 3. 最小写入阈值 */
            constexpr size_t min_available[(int)LogType::Count] = {
                _min_write_chunk,  // Full日志：4096字节
                1                  // Mission日志：有数据就写
            };
            
            /* 4. 检查所有缓冲区 */
            int i = (int)LogType::Count - 1;
            
            while (i >= 0) {
                LogFileBuffer &buffer = _buffers[i];
                
                // 检查缓冲区数据量是否达到阈值
                if (buffer._should_run && buffer.count() >= min_available[i]) {
                    
                    void *read_ptr;
                    bool is_part;
                    size_t available = buffer.get_read_ptr(&read_ptr, &is_part);
                    
                    /* 5. 写入SD卡 */
                    ssize_t ret = buffer.write_to_file(read_ptr, available, call_fsync);
                    
                    if (ret > 0) {
                        buffer.mark_read(ret);  // 标记已写入
                    } else {
                        buffer._had_write_error.store(true);
                    }
                    
                    // 继续检查这个缓冲区（可能还有数据）
                    continue;
                }
                
                --i;
            }
            
            // 如果所有缓冲区都处理完，跳出内层循环
            if (no_data_available) {
                break;
            }
        }
        
        pthread_mutex_unlock(&_mtx);
    }
}
```

**关键点**：
1. **批量写入阈值**：`_min_write_chunk = 4096` 字节（匹配FAT簇大小）
2. **只有当缓冲区累积 ≥ 4096 字节时才写入SD卡**
3. **Mission日志例外**：有数据就写（避免丢失重要事件）

### 3.4 写入SD卡的实际操作

```cpp
// src/modules/logger/log_writer_file.cpp:696-707
ssize_t LogWriterFile::LogFileBuffer::write_to_file(const void *buffer, size_t size, bool call_fsync) const
{
    perf_begin(_perf_write);
    ssize_t ret = ::write(_fd, buffer, size);  // POSIX write系统调用
    perf_end(_perf_write);
    
    if (call_fsync) {
        fsync();  // 强制刷新到SD卡
    }
    
    return ret;
}
```

**fsync调用时机**：
```cpp
// src/modules/logger/log_writer_file.cpp:689-694
void LogWriterFile::LogFileBuffer::fsync() const
{
    perf_begin(_perf_fsync);
    ::fsync(_fd);  // POSIX fsync系统调用
    perf_end(_perf_fsync);
}
```

**fsync触发条件**（满足任一）：
- 写入次数达到 **100次**
- 距离上次fsync超过 **1秒**
- 手动请求fsync（`_want_fsync = true`）

---

## 📊 四、完整数据流时序图

```
时间线 (每3.5ms一个周期)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[定时器中断]
    │
    ├──> timer_callback()
    │        └──> sem_post(semaphore)
    │
    ↓
[Logger主线程被唤醒]
    │
    ├──> lock()                      ← 锁定缓冲区
    │
    ├──> 遍历所有订阅的主题
    │        ├──> copy_if_updated()  ← 检查主题是否更新
    │        │        └──> orb_copy() ← 从uORB拷贝数据
    │        │
    │        └──> write_message()    ← 写入环形缓冲区
    │                 └──> write_no_check()
    │                          └──> memcpy()  ← 数据拷贝到缓冲区
    │                                   _count += size
    │
    ├──> unlock()                    ← 释放锁
    │
    ├──> notify()                    ← 通知Writer线程
    │        └──> pthread_cond_broadcast()
    │
    └──> sem_wait(semaphore)         ← 等待下一次定时器触发

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[Writer线程] (独立运行)
    │
    ├──> pthread_cond_wait()         ← 等待notify()
    │
    ├──> 检查缓冲区
    │        └──> if (_count >= 4096) {  ← 达到写入阈值
    │                  write(_fd, buffer, 4096);  ← 写SD卡
    │                  _count -= 4096;
    │                  _total_written += 4096;
    │             }
    │
    ├──> 每100次或每1秒调用一次
    │        └──> fsync(_fd);        ← 强制刷新到SD卡
    │
    └──> 循环...

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## 🎯 五、关键设计要点

### 5.1 为什么要缓冲区？

**直接写SD卡的问题**：
- SD卡写入延迟高（几毫秒到几十毫秒）
- 每次写入都触发文件系统操作（开销大）
- 小数据块写入效率低（FAT簇大小4096字节）

**缓冲区的优势**：
- **解耦采样和写入**：Logger主线程专注采样，Writer线程专注写入
- **批量写入**：累积到4096字节再写，匹配FAT簇大小
- **降低写入频率**：减少SD卡磨损

### 5.2 为什么是4096字节？

```cpp
/* 512 didn't seem to work properly, 4096 should match the FAT cluster size */
static constexpr size_t _min_write_chunk = 4096;
```

**原因**：
1. **FAT32簇大小**：通常是4096字节（4KB）
2. **对齐写入**：一次写入正好一个簇，避免跨簇写入
3. **性能优化**：512字节测试效果不佳

### 5.3 缓冲区溢出处理

**如果缓冲区满了怎么办？**

```cpp
// src/modules/logger/log_writer_file.cpp:548-551
if (size + dropout_size > available) {
    // buffer overflow
    return -1;
}
```

**Logger主线程处理**：
```cpp
// src/modules/logger/logger.cpp:775
if (write_message(LogType::Full, _msg_buffer, msg_size)) {
    total_bytes += msg_size;
}
// 如果返回-1（缓冲区满），丢弃本次数据并记录dropout
```

**dropout标记**：
```cpp
ulog_message_dropout_s dropout_msg;
dropout_msg.duration = (uint16_t)(hrt_elapsed_time(&dropout_start) / 1000);
_buffers[(int)type].write_no_check(&dropout_msg, sizeof(dropout_msg));
```

### 5.4 零丢包保证（可靠传输模式）

```cpp
// src/modules/logger/log_writer_file.cpp:498-529
if (_need_reliable_transfer) {
    int ret;
    
    do {
        size_t write_size = math::min(size, _buffers[(int)type].buffer_size());
        
        while ((ret = write(type, uptr, write_size, 0)) == -1) {
            unlock();
            notify();
            px4_usleep(3000);  // 等待3ms
            lock();
        }
        
        uptr += write_size;
        size -= write_size;
    } while (size > 0);
    
    return ret;
}
```

**可靠传输模式**：
- **阻塞等待**：如果缓冲区满，等待3ms后重试
- **保证写入**：不丢弃任何数据
- **用途**：关键日志（任务日志、事件日志）

---

## 📈 六、性能分析

### 6.1 时间开销分析

| 操作 | 时间 | 频率 | 说明 |
|------|------|------|------|
| 定时器中断 | < 10 μs | 3.5 ms | 中断上下文，极快 |
| orb_copy() | 1-5 μs | 每主题 | 内存拷贝 |
| memcpy到缓冲区 | 1-3 μs | 每主题 | 内存拷贝 |
| write()到SD卡 | 0.5-5 ms | 每4KB | 文件系统操作 |
| fsync() | 5-50 ms | 每秒或100次写入 | 强制刷新到SD卡 |

### 6.2 缓冲区利用率

**以默认配置为例**：
- 缓冲区大小：12 KB
- 写入阈值：4 KB
- 采样周期：3.5 ms

**单周期数据量估算**（ERmao优化日志）：
```
vehicle_attitude: 64 bytes
vehicle_rates_setpoint: 44 bytes
actuator_motors: 48 bytes
sensor_combined: 96 bytes
... (其他主题)
─────────────────────────────
总计：约 500-800 bytes/周期
```

**写入频率**：
```
4096 bytes / 650 bytes/周期 ≈ 6 个周期
6 × 3.5 ms = 21 ms
```

**结论**：约每 **20-30 ms** 写入一次SD卡

### 6.3 最坏情况延迟

**数据从发布到持久化的最大延迟**：
```
发布延迟: 0-3.5 ms     (等待下一次定时器触发)
缓冲延迟: 0-21 ms      (等待缓冲区累积到4KB)
写入延迟: 0.5-5 ms     (write系统调用)
fsync延迟: 0-1000 ms   (最多1秒才调用fsync)
─────────────────────────────────
总计: 0.5-1029.5 ms
```

**实际平均延迟**：约 **10-50 ms**

---

## 🔧 七、参数调优

### 7.1 SDLOG_PROFILE（日志配置文件）

| 值 | 缓冲区大小 | 日志间隔 | 用途 |
|----|-----------|---------|------|
| 0  | 12 KB     | 3.5 ms  | 默认（平衡） |
| 1  | 64 KB     | 10 ms   | 高吞吐量 |
| 2  | 8 KB      | 1 ms    | 低延迟 |

### 7.2 SDLOG_MODE（日志模式）

| 值 | 模式 | 说明 |
|----|------|------|
| 0  | while_armed | 仅解锁时记录 |
| 1  | boot_until_disarm | 启动后一直记录，直到上锁 |
| 2  | boot_until_shutdown | 启动后一直记录 |

### 7.3 调优建议

**追求低延迟**：
```
SDLOG_PROFILE = 2
SDLOG_MODE = 0
```

**追求高吞吐量**（大量数据）：
```
SDLOG_PROFILE = 1
SDLOG_MODE = 1
```

**追求零丢包**：
- 使用可靠传输模式（Mission log）
- 增大缓冲区（SDLOG_PROFILE=1）
- 使用高速SD卡（Class 10或UHS-I）

---

## 🎓 八、总结

### 回答你的问题

**Q1: Logger记录任务是如何进行的？**
- 使用**定时器中断** + **信号量**，每3.5ms唤醒一次Logger主线程
- 主线程遍历所有订阅的主题，检查是否有更新（`copy_if_updated()`）
- 有更新则拷贝数据并写入环形缓冲区

**Q2: 定时轮询触发是如何实现的？**
- `hrt_call_every()`注册高精度定时器，周期3.5ms
- 定时器中断执行`timer_callback()`，调用`sem_post()`
- Logger主线程调用`sem_wait()`等待信号量，被唤醒后执行一次采样

**Q3: 实际应该有一个大的buf吧？**
- **是的！** 有两个环形缓冲区（Full和Mission）
- Full日志缓冲区默认**12 KB**，可配置为8-64 KB
- 使用环形缓冲区（ring buffer）实现高效的读写

**Q4: 当从主题上拷贝下来的数据存满一定的buf数量，然后再调用文件系统的写入，才真正把数据写入SD卡？**
- **完全正确！** 这是批量写入优化
- 当缓冲区累积 **≥ 4096 字节**时，Writer线程才写SD卡
- 每100次写入或每1秒调用一次`fsync()`强制刷新

### 关键设计亮点

1. **双线程架构**：Logger主线程采样，Writer线程写SD卡，解耦高频采样和低频写入
2. **环形缓冲区**：高效的内存管理，避免频繁分配
3. **批量写入**：4KB对齐写入，匹配FAT簇大小，优化性能
4. **高精度定时**：使用HRT+信号量，达到微秒级精度
5. **可靠传输模式**：关键数据零丢包保证

---

**文档版本**：1.0  
**创建时间**：2025-11-02  
**相关文档**：08-Logger数据流机制详解.md

