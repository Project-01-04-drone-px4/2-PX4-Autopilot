# sensor_gyro_fifo 记录说明

## 一、问题解答

### 问题 1: `add_optional_topic` vs `add_topic` 的区别

#### 函数定义

```cpp
// logged_topics.h
bool add_topic(const char *name, uint16_t interval_ms = 0, uint8_t instance = 0)
{
    return add_topic(name, interval_ms, instance, false);  // optional = false
}

bool add_optional_topic(const char *name, uint16_t interval_ms = 0, uint8_t instance = 0)
{
    return add_topic(name, interval_ms, instance, true);   // optional = true
}
```

#### 核心区别

在 `logged_topics.cpp` 的 `add_topic()` 函数中（第 463-471 行）：

```cpp
if (optional && orb_exists(topic, instance) != 0) {
    PX4_DEBUG("Not adding non-existing optional topic %s %i", topic->o_name, instance);
    // ...省略...
    return false;  // 静默跳过，不记录
}
```

| 特性           | `add_topic()`              | `add_optional_topic()`     |
|----------------|----------------------------|----------------------------|
| **主题不存在** | 可能报警告                 | 静默跳过，不报错           |
| **适用场景**   | 必须记录的核心主题         | 可选的硬件相关主题         |
| **interval_ms** | 默认 0（全速记录）        | 默认 0（全速记录）         |

#### 为什么 ERmao 模式使用 `add_optional_topic`？

**原因**：`sensor_gyro_fifo` **不是所有硬件都支持的**。

- **支持的硬件**：带有 FIFO 功能的 IMU（如 BMI270、ICM-42688-P）
- **不支持的硬件**：老旧的 IMU（如 MPU6000、LSM9DS1）

使用 `add_optional_topic` 的好处：
- ✅ 如果硬件支持 FIFO，则正常记录
- ✅ 如果硬件不支持 FIFO，则静默跳过，不会报错
- ✅ 兼容性更好，适配不同的硬件配置

#### 对比：`add_raw_imu_gyro_fifo()` 使用 `add_topic`

```cpp
void LoggedTopics::add_raw_imu_gyro_fifo()
{
    add_topic("sensor_gyro_fifo");  // 强制记录
}
```

这个函数是专门用于 **RAW_IMU_GYRO_FIFO 模式**（bit 8）的：
- 这个模式是**专门用于记录原始 IMU FIFO 数据**
- 如果用户启用了这个模式，说明用户**知道自己的硬件支持 FIFO**
- 因此使用 `add_topic` 是合理的：如果硬件不支持，应该报错提醒用户

#### 结论

**ERmao 模式**应该使用 `add_optional_topic`，因为：
1. ERmao 模式是通用的控制链路记录模式
2. 不是所有用户的硬件都支持 FIFO
3. sensor_gyro_fifo 是锦上添花，不是必需的

**如果你确认你的硬件支持 FIFO，且想强制记录**，可以改为：
```cpp
add_topic("sensor_gyro_fifo");  // 强制记录
```

---

## 二、问题 2: 为什么实际日志中 gyro_fifo 不是全速记录？

### 原因分析

#### 1. uORB 队列深度限制 ⚠️ **主要原因**

在 `msg/SensorGyroFifo.msg` 中：

```cpp
uint8 ORB_QUEUE_LENGTH = 4  // 队列深度只有 4
```

**工作原理**：
```
IMU 驱动 → uORB 发布 → 队列 [消息1] [消息2] [消息3] [消息4] → Logger 订阅
                              ↓
                         如果 Logger 没及时读取，队列满了
                              ↓
                         新消息会覆盖旧消息（数据丢失）
```

**数据丢失场景**：

假设：
- IMU FIFO 发布频率：1600 Hz（每 0.625 ms 发布一次）
- Logger 订阅延迟：5 ms（SD 卡写入慢）

```
时间轴：
  0.0 ms: 消息1 发布 → 队列 [消息1] [ ] [ ] [ ]
  0.6 ms: 消息2 发布 → 队列 [消息1] [消息2] [ ] [ ]
  1.2 ms: 消息3 发布 → 队列 [消息1] [消息2] [消息3] [ ]
  1.8 ms: 消息4 发布 → 队列 [消息1] [消息2] [消息3] [消息4] ← 队列满
  2.4 ms: 消息5 发布 → 队列 [消息2] [消息3] [消息4] [消息5] ← 消息1 被覆盖（丢失）
  3.0 ms: 消息6 发布 → 队列 [消息3] [消息4] [消息5] [消息6] ← 消息2 被覆盖（丢失）
  ...
  5.0 ms: Logger 终于读取 → 只能读到消息5、6、7、8

  结果：消息1、2、3、4 全部丢失
```

#### 2. SD 卡写入速度限制

**sensor_gyro_fifo 的数据量**：

```cpp
struct sensor_gyro_fifo_s {
    uint64 timestamp;        // 8 字节
    uint64 timestamp_sample; // 8 字节
    uint32 device_id;        // 4 字节
    float32 dt;              // 4 字节
    float32 scale;           // 4 字节
    uint8 samples;           // 1 字节
    int16[32] x;             // 64 字节
    int16[32] y;             // 64 字节
    int16[32] z;             // 64 字节
};
// 总计：约 221 字节/消息
```

**每秒数据量计算**（假设 BMI270，1600 Hz）：

- 每条消息包含 32 个样本
- 每条消息大小：221 字节
- 实际采样率：1600 Hz（原始陀螺仪采样）
- 消息发布率：1600 / 32 = 50 Hz（每次发布 32 个样本）
- **每秒数据量**：221 字节 × 50 Hz = **11,050 字节/秒 ≈ 10.8 KB/s**

加上其他主题（vehicle_angular_velocity, sensor_combined 等）：
- **总数据量**：约 **100-200 KB/s**（取决于记录的主题数量）

**SD 卡写入速度**：
- 低速卡（Class 4）：4 MB/s（足够）
- 中速卡（Class 10）：10 MB/s（足够）
- 高速卡（UHS-I）：10-90 MB/s（足够）

**结论**：SD 卡速度通常不是瓶颈，但仍可能受影响：
- ❌ SD 卡碎片化
- ❌ 文件系统开销
- ❌ 其他高优先级任务占用 CPU

#### 3. Logger 订阅优先级

Logger 是一个普通优先级的任务，可能被更高优先级的任务抢占：
- 控制器（最高优先级）
- 传感器驱动（高优先级）
- Logger（中等优先级）← 可能被延迟

---

## 三、解决方案

### 方案 1: 增加 uORB 队列深度（推荐）✅

修改 `msg/SensorGyroFifo.msg`：

```diff
- uint8 ORB_QUEUE_LENGTH = 4
+ uint8 ORB_QUEUE_LENGTH = 16  // 或者 32
```

**优点**：
- ✅ 减少数据丢失
- ✅ 允许 Logger 更大的延迟容忍度

**缺点**：
- ❌ 增加内存占用：(16 - 4) × 221 字节 = 2.6 KB

**需要重新编译整个项目**：
```bash
make clean
make px4_sitl_default
```

---

### 方案 2: 降低记录频率

如果不需要完整的 1600 Hz 数据，可以降频记录：

```cpp
// 修改 logged_topics.cpp
add_optional_topic("sensor_gyro_fifo", 50);  // 每 50 ms 记录一次（20 Hz）
```

**优点**：
- ✅ 减少数据量
- ✅ 减少队列溢出风险

**缺点**：
- ❌ 丢失高频数据
- ❌ 无法分析高频振动（>10 Hz）

---

### 方案 3: 使用专用的 RAW_IMU_GYRO_FIFO 模式

启用 bit 8（RAW_IMU_GYRO_FIFO 模式）：

```bash
param set SDLOG_PROFILE 4352  # DEFAULT (1) + RAW_IMU_GYRO_FIFO (256) + ERMAO (4096)
```

这个模式可能有更好的优化（但仍然受限于队列深度）。

---

### 方案 4: 检查实际丢失率

使用 pyulog 分析日志，检查实际的数据丢失情况：

```python
from pyulog import ULog
import numpy as np

ulog = ULog('log.ulg')
gyro_fifo = ulog.get_dataset('sensor_gyro_fifo').data

# 检查时间戳间隔
timestamps = gyro_fifo['timestamp']
dt_actual = np.diff(timestamps) * 1e-6  # 转换为秒

# 理论间隔（假设 32 样本，1600 Hz）
dt_expected = 32 / 1600  # 0.02 秒 = 20 ms

# 统计
print(f"理论间隔: {dt_expected * 1000:.2f} ms")
print(f"实际间隔 (平均): {np.mean(dt_actual) * 1000:.2f} ms")
print(f"实际间隔 (标准差): {np.std(dt_actual) * 1000:.2f} ms")
print(f"最大间隔: {np.max(dt_actual) * 1000:.2f} ms")

# 检查丢失率
missed_count = np.sum(dt_actual > dt_expected * 1.5)
total_count = len(dt_actual)
print(f"丢失的消息: {missed_count} / {total_count} ({missed_count/total_count*100:.2f}%)")
```

---

## 四、推荐配置

### 配置 1: 完整 FIFO 记录（需要修改源码）

**适用场景**：需要完整的高频数据进行振动分析

**步骤**：
1. 修改 `msg/SensorGyroFifo.msg`，增加队列深度到 16 或 32
2. 重新编译
3. 使用 `add_topic("sensor_gyro_fifo")` 强制记录

**修改 `logged_topics.cpp`**：
```cpp
void LoggedTopics::add_ermao_topics()
{
    // ...

    // sensor_gyro_fifo - 强制全速记录（需要先修改消息定义增加队列深度）
    add_topic("sensor_gyro_fifo");  // 改为强制记录

    // ...
}
```

---

### 配置 2: 降频 FIFO 记录（不需要修改源码）✅ 推荐

**适用场景**：只需要中低频振动分析（0-100 Hz）

**修改 `logged_topics.cpp`**：
```cpp
void LoggedTopics::add_ermao_topics()
{
    // ...

    // sensor_gyro_fifo - 降频记录（减少队列溢出）
    add_optional_topic("sensor_gyro_fifo", 50);  // 每 50 ms 记录一次（20 Hz）

    // ...
}
```

**采样定理**：
- 记录频率：20 Hz
- 可分析的最高频率：10 Hz（根据奈奎斯特定理）
- 足够分析大部分机械振动问题

---

### 配置 3: 仅使用 vehicle_angular_velocity（不修改源码）✅ 最稳定

**适用场景**：主要关注控制性能，不关注原始传感器数据

**当前配置**（已经有了）：
```cpp
void LoggedTopics::add_ermao_topics()
{
    // ...

    // 使用已滤波的角速度数据（667 Hz，队列深度更大）
    add_topic("vehicle_angular_velocity");  // 已经有了

    // 不记录 sensor_gyro_fifo
    // add_optional_topic("sensor_gyro_fifo");  // 注释掉

    // ...
}
```

**优点**：
- ✅ 稳定可靠，不会丢数据
- ✅ 667 Hz 已经足够分析大部分控制问题
- ✅ 已经过滤波和偏差补偿，更适合控制分析

---

## 五、总结

### `add_optional_topic` vs `add_topic`

| 函数                  | 适用场景                   | 主题不存在时 |
|-----------------------|----------------------------|--------------|
| `add_topic()`         | 必须记录的核心主题         | 可能报警告   |
| `add_optional_topic()` | 硬件相关的可选主题         | 静默跳过     |

**ERmao 模式应该使用 `add_optional_topic`**，因为 sensor_gyro_fifo 不是所有硬件都支持。

### gyro_fifo 不是全速记录的原因

**主要原因**：uORB 队列深度只有 4，容易溢出导致数据丢失

**解决方案**：
1. ✅ **推荐**：降频记录（50 ms，20 Hz）- 不需要修改源码
2. ✅ **推荐**：只使用 vehicle_angular_velocity - 最稳定
3. ⚠️ 进阶：增加队列深度到 16-32 - 需要修改源码和重新编译

### 建议

对于你的使用场景（姿态环控制分析）：
- ✅ **vehicle_angular_velocity（667 Hz）已经足够**
- ✅ 不一定需要 sensor_gyro_fifo
- ✅ 如果需要振动分析，使用 sensor_gyro_fft（50 Hz）更合适

如果坚持要记录 sensor_gyro_fifo：
- ✅ 使用降频配置：`add_optional_topic("sensor_gyro_fifo", 50)`
- ⚠️ 或修改队列深度（需要重新编译）

---

**文档版本**：1.0
**最后更新**：2025-11-01

