# FFT 发布频率与 Logger 记录频率详解

## 一、两个关键频率

### 1.1 定义

**频率 1：FFT 发布频率**
- sensor_gyro_fft 主题的 publish() 调用频率
- 决定了有多少新的 FFT 结果产生

**频率 2：Logger 记录频率**
- logger 将 sensor_gyro_fft 写入 SD 卡的频率
- 决定了日志中有多少数据点

**关键关系：**
```
实际记录频率 = min(FFT发布频率, Logger配置频率)
```

---

## 二、FFT 发布频率分析

### 2.1 发布触发条件

从 `GyroFFT.cpp` 代码逻辑：

```cpp
// Run() 函数流程
Run() {
    _fft_updated = false;  // 重置标志
    _publish = false;      // 重置发布标志

    while (有新的 gyro_fifo 数据) {
        Update(...);  // 填充 FFT 缓冲区

        if (buffer 满了 && !_fft_updated) {
            执行 FFT
            FindPeaks(...);  // 查找峰值

            if (找到峰值) {
                UpdateOutput(...);  // 设置 _publish = true
            }

            _fft_updated = true;  // 标记已执行 FFT
        }
    }

    if (_publish) {
        Publish();  // ← 实际发布！
    }
}
```

### 2.2 发布条件总结

**sensor_gyro_fft 只在以下情况发布：**

1. ✅ FFT 缓冲区满（收集了足够样本）
2. ✅ 执行了 FFT 分析
3. ✅ 找到了峰值（freq > 0 且 SNR > 阈值）
4. ✅ 峰值通过了匹配和过滤逻辑

**不满足任一条件就不发布！**

### 2.3 FFT 发布频率计算

#### 理想情况（总是找到峰值）

```
缓冲区填充速度 = 采样率 × (1 - 重叠率) / FFT_LEN
                = 8000 × 0.25 / 256
                = 7.8125 Hz（单轴）

三个轴独立：
总体发布频率 = 7.8125 × 3 ≈ 23.4 Hz
```

#### 实际情况（并非每次都发布）

**限制因素：**

1. **SNR 阈值**（`IMU_GYRO_FFT_SNR = 5`）
   - 如果信噪比 < 5，不发布
   - Chirp 信号 SNR 通常 10-20，应该没问题

2. **频率范围**（`IMU_GYRO_FFT_MIN=2, MAX=1000`）
   - 峰值必须在范围内
   - Z 轴高频部分可能被过滤

3. **每次 Run() 只处理一个轴**（`_fft_updated` 标志）
   - X/Y/Z 轮流处理
   - 降低总体频率

**估算实际发布频率：**
```
最佳情况：~23 Hz（三轴都找到峰值）
典型情况：~15 Hz（部分峰值被过滤）
最差情况：<5 Hz（很多峰值低于 SNR 阈值）
```

---

## 三、Logger 记录频率分析

### 3.1 配置位置

**文件：** `src/modules/logger/logged_topics.cpp` 第 123 行

```cpp
add_optional_topic("sensor_gyro_fft", 20);
//                                    ↑
//                                 20 ms = 50 Hz
```

### 3.2 记录机制

```
logger 定时器（每 20ms）
  ↓
检查 sensor_gyro_fft 主题
  ↓
如果有新数据（timestamp 变化）
  ↓
写入 SD 卡
```

**关键点：**
- logger 每 20ms 检查一次
- 但只有当 sensor_gyro_fft 有新的 publish 时才记录
- 如果 FFT 发布频率 < 50 Hz，logger 会记录重复数据或跳过

### 3.3 实际记录频率

```
实际记录频率 = min(FFT 发布频率, Logger 配置频率)
            = min(~15 Hz, 50 Hz)
            = ~15 Hz
```

**也就是说：**
- Logger 配置 50 Hz，但 FFT 只发布 15 Hz
- 实际只能记录 ~15 Hz
- 其他时间 logger 检查到没有新数据，不写入

---

## 四、频率决定因素

### 4.1 FFT 发布频率由什么决定？

| 因素 | 代码位置 | 影响 | 可调 |
|------|---------|------|------|
| **FFT_LEN** | parameters.c | 越小越快 | ✅ 参数 |
| **重叠率** | GyroFFT.cpp:453 | 越小越快 | ⚠️ 改代码 |
| **采样率** | fake_imu | 越高越快 | ✅ 固定8kHz |
| **SNR 阈值** | parameters.c | 越低发布越多 | ✅ 参数 |
| **频率范围** | parameters.c | 越宽包含越多 | ✅ 参数 |
| **三轴限制** | GyroFFT.cpp:441 | 分时处理 | ⚠️ 改代码 |

**通过参数可调的：**
```bash
# 提高发布频率
param set IMU_GYRO_FFT_LEN 128  # 频率翻倍
param set IMU_GYRO_FFT_SNR 3    # 降低阈值，更多发布
```

### 4.2 Logger 记录频率由什么决定？

| 因素 | 代码位置 | 说明 | 可调 |
|------|---------|------|------|
| **Logger 配置** | logged_topics.cpp:123 | 检查间隔 | ✅ 改代码 |
| **FFT 发布频率** | gyro_fft | 数据源速率 | ✅ 见上 |

**当前配置：**
```cpp
add_optional_topic("sensor_gyro_fft", 20);
//                                    ↑
//                            20 ms = 50 Hz（检查频率）
```

**实际记录：**
```
如果 FFT 以 15 Hz 发布：
  logger 每 20ms 检查，但只有 ~67ms 才有新数据
  实际记录频率 = 15 Hz

如果 FFT 以 100 Hz 发布：
  logger 每 20ms 检查并记录新数据
  实际记录频率 = 50 Hz（受 logger 配置限制）
```

---

## 五、当前配置分析

### 5.1 fake_imu 参数

```
内部采样率: 8000 Hz
发布频率: 800 Hz（gyro_fifo）
每次样本数: 10
```

### 5.2 gyro_fft 配置

```bash
IMU_GYRO_FFT_LEN = 256
重叠率 = 75%（代码固定）
```

**FFT 执行频率：**
```
新样本需求 = 256 / 4 = 64
数据速率 = 8000 Hz
执行间隔 = 64 / 8000 = 8 ms
执行频率 = 125 Hz（理论，单轴）
```

**FFT 发布频率：**
```
考虑三轴分时：125 / 3 ≈ 41 Hz
考虑峰值过滤：实际可能 15-30 Hz
```

### 5.3 Logger 配置

```cpp
add_optional_topic("sensor_gyro_fft", 20);  // 50 Hz
```

**实际记录频率：**
```
= min(FFT发布 15-30 Hz, Logger配置 50 Hz)
≈ 15-30 Hz
```

---

## 六、优化建议

### 6.1 如果想提高 Logger 记录频率

#### 方案 A：提高 FFT 发布频率（推荐）

```bash
# 减小 FFT_LEN
param set IMU_GYRO_FFT_LEN 128

# 新的发布频率
FFT 执行：250 / 3 ≈ 83 Hz
FFT 发布：~50 Hz（考虑峰值过滤）
Logger 记录：min(50, 50) = 50 Hz ✓
```

#### 方案 B：降低 Logger 间隔

```cpp
// src/modules/logger/logged_topics.cpp
add_optional_topic("sensor_gyro_fft", 10);  // 10ms = 100 Hz
```

**但这不会增加实际数据！** 只会让 logger 检查更频繁。

### 6.2 推荐配置

#### 平衡配置（当前）
```bash
IMU_GYRO_FFT_LEN = 256
Logger 间隔 = 20 ms

预期记录频率：15-25 Hz
10 秒数据点：150-250 点
```

#### 高速配置
```bash
param set IMU_GYRO_FFT_LEN 128
Logger 间隔 = 20 ms（代码已设置）

预期记录频率：30-50 Hz
10 秒数据点：300-500 点
```

#### 超高速配置
```bash
param set IMU_GYRO_FFT_LEN 64
Logger 间隔 = 10 ms（需改代码）

预期记录频率：80-100 Hz
10 秒数据点：800-1000 点
```

---

## 七、实际测量方法

### 7.1 测量 FFT 发布频率

```bash
# 1. 启动
nsh> fake_imu start
nsh> gyro_fft start

# 2. 使用 uorb top 查看
nsh> uorb top

# 查找 sensor_gyro_fft
# 示例输出：
# sensor_gyro_fft    15.2 Hz  ← 实际发布频率
```

### 7.2 测量 Logger 记录频率

```python
from pyulog import ULog
import numpy as np

log = ULog('log.ulg')
fft = log.get_dataset('sensor_gyro_fft').data

timestamps = fft['timestamp']
dt = np.diff(timestamps) / 1000  # ms

print(f'=== Logger 记录分析 ===')
print(f'总记录数: {len(timestamps)}')
print(f'时间范围: {(timestamps[-1] - timestamps[0])/1e6:.1f} 秒')
print(f'')
print(f'记录间隔:')
print(f'  平均: {np.mean(dt):.1f} ms')
print(f'  中位数: {np.median(dt):.1f} ms')
print(f'')
print(f'实际记录频率: {1000/np.mean(dt):.1f} Hz')
```

---

## 八、频率配置对照表

### 8.1 FFT_LEN 对发布频率的影响

| FFT_LEN | FFT执行间隔 | FFT执行频率 | 预计发布频率 | 10秒数据点 |
|---------|------------|------------|-------------|-----------|
| 64 | 2 ms | 500 Hz | ~100 Hz | 1000 |
| **128** | **4 ms** | **250 Hz** | **~50 Hz** | **500** |
| **256** | **8 ms** | **125 Hz** | **~25 Hz** | **250** |
| 512 | 16 ms | 62 Hz | ~12 Hz | 120 |
| 1024 | 32 ms | 31 Hz | ~6 Hz | 60 |

### 8.2 Logger 配置对记录频率的影响

| Logger间隔 | Logger检查频率 | FFT发布=10Hz | FFT发布=25Hz | FFT发布=50Hz |
|-----------|---------------|-------------|-------------|-------------|
| 100 ms | 10 Hz | 10 Hz | 10 Hz | 10 Hz |
| 50 ms | 20 Hz | 10 Hz | 20 Hz | 20 Hz |
| **20 ms** | **50 Hz** | **10 Hz** | **25 Hz** | **50 Hz** |
| 10 ms | 100 Hz | 10 Hz | 25 Hz | 50 Hz |

**结论：**
- Logger 间隔 < FFT 发布间隔 → 记录频率 = FFT 发布频率
- Logger 间隔 > FFT 发布间隔 → 记录频率 = Logger 频率

---

## 九、当前配置的实际频率

### 9.1 配置参数

```
IMU_GYRO_FFT_LEN = 256
Logger 间隔 = 20 ms (50 Hz)
```

### 9.2 预期频率

```
FFT 执行频率：
  单轴：8 ms 间隔 = 125 Hz
  三轴分时：125 / 3 ≈ 41 Hz

FFT 发布频率：
  考虑峰值过滤：15-30 Hz（估计）
  实际测量：使用 uorb top

Logger 记录频率：
  = min(FFT发布, 50 Hz)
  ≈ 15-30 Hz

10 秒数据点数：
  ≈ 150-300 点
```

### 9.3 验证方法

```bash
# 运行测试
nsh> fake_imu start
nsh> gyro_fft start
nsh> logger start -f -t

# 运行 10 秒

# 查看实际频率
nsh> uorb top
# 查找 sensor_gyro_fft 的频率

# 示例输出：
# sensor_gyro_fft    18.3 Hz  ← 这是实际发布频率
```

---

## 十、优化建议

### 10.1 目标：提高记录数据点密度

**当前：10 秒约 150-300 点**

#### 方案 1：提高 FFT 发布频率（改参数）✅

```bash
# 使用更小的 FFT_LEN
param set IMU_GYRO_FFT_LEN 128

# 预期效果：
# FFT 发布：30-50 Hz
# Logger 记录：30-50 Hz
# 10 秒数据：300-500 点
```

#### 方案 2：降低 SNR 阈值（改参数）✅

```bash
# 让更多峰值被发布
param set IMU_GYRO_FFT_SNR 3

# 预期效果：
# 更多低 SNR 峰值被发布
# 发布频率提高 10-20%
```

#### 方案 3：提高 Logger 检查频率（改代码）

```cpp
// logged_topics.cpp
add_optional_topic("sensor_gyro_fft", 10);  // 10ms = 100 Hz
```

**但效果有限：** 如果 FFT 只发布 20 Hz，logger 检查 100 Hz 也没用。

#### 方案 4：修改重叠率（改代码）⚠️

```cpp
// GyroFFT.cpp 第 453 行
const int overlap_start = _imu_gyro_fft_len / 2;  // 50% 重叠（当前75%）
```

**效果：** FFT 执行和发布频率翻倍。

---

## 十一、推荐配置方案

### 方案 A：当前配置（平衡）✅

```bash
# 参数
IMU_GYRO_FFT_LEN = 256
IMU_GYRO_FFT_SNR = 5

# Logger（已设置）
间隔 = 20 ms

# 预期
发布：15-25 Hz
记录：15-25 Hz
10秒：150-250 点
```

**优点：** CPU 占用低，分辨率适中

### 方案 B：高速配置（推荐用于 Chirp）✅

```bash
# 修改参数
param set IMU_GYRO_FFT_LEN 128
param set IMU_GYRO_FFT_SNR 3
param save
reboot

# 预期
发布：30-50 Hz
记录：30-50 Hz
10秒：300-500 点
```

**优点：** 数据密集，Chirp 变化清晰

### 方案 C：超高速配置（需改代码）⚠️

```cpp
// 1. GyroFFT.cpp - 降低重叠率
const int overlap_start = _imu_gyro_fft_len / 2;

// 2. logged_topics.cpp - 提高检查频率
add_optional_topic("sensor_gyro_fft", 10);  // 100 Hz

// 3. 参数
param set IMU_GYRO_FFT_LEN 128
```

**预期：**
- 发布：80-100 Hz
- 记录：80-100 Hz
- 10秒：800-1000 点

---

## 十二、快速配置命令

### 推荐：使用 FFT_LEN=128

```bash
# NSH 中执行
param set IMU_GYRO_FFT_LEN 128
param set IMU_GYRO_FFT_SNR 3
param save
reboot

# 重启后
fake_imu start
gyro_fft start
logger start -f -t

# 运行 20 秒后
logger stop

# 验证实际频率
uorb top | grep gyro_fft
```

---

## 十三、频率验证脚本

### Python 完整验证

```python
#!/usr/bin/env python3
from pyulog import ULog
import numpy as np
import matplotlib.pyplot as plt

def analyze_fft_logger_frequency(log_file):
    """分析 FFT 发布频率和 Logger 记录频率"""

    log = ULog(log_file)
    fft = log.get_dataset('sensor_gyro_fft').data

    timestamps = fft['timestamp']
    time_sec = (timestamps - timestamps[0]) / 1e6
    dt = np.diff(timestamps) / 1000  # ms

    print('=' * 60)
    print('FFT 发布频率与 Logger 记录频率分析')
    print('=' * 60)
    print()

    print('【数据统计】')
    print(f'  总记录数: {len(timestamps)}')
    print(f'  时间范围: {time_sec[-1]:.1f} 秒')
    print()

    print('【记录间隔】')
    print(f'  平均间隔: {np.mean(dt):.1f} ms')
    print(f'  中位数: {np.median(dt):.1f} ms')
    print(f'  标准差: {np.std(dt):.1f} ms')
    print(f'  最小间隔: {np.min(dt):.1f} ms')
    print(f'  最大间隔: {np.max(dt):.1f} ms')
    print()

    print('【频率计算】')
    actual_freq = 1000 / np.mean(dt)
    print(f'  实际记录频率: {actual_freq:.1f} Hz')
    print(f'  Logger 配置频率: 50 Hz (20 ms)')
    print()

    if actual_freq < 45:
        print('  → FFT 发布是瓶颈（发布 < 50 Hz）')
        print(f'    建议: 减小 IMU_GYRO_FFT_LEN 到 {256 * 25 / actual_freq:.0f}')
    else:
        print('  → Logger 配置是瓶颈（检查 < FFT发布）')
        print('    建议: 减小 logger 间隔到 10 ms')
    print()

    print('【数据密度】')
    points_per_10s = actual_freq * 10
    print(f'  10秒数据点: {points_per_10s:.0f} 点')
    if points_per_10s < 200:
        print('  评价: ⚠️  稍少，建议优化')
    elif points_per_10s < 500:
        print('  评价: ✓  适中，可用')
    else:
        print('  评价: ✓✓ 优秀，数据密集')
    print()

    # 绘图
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # 记录间隔时序图
    axes[0].plot(time_sec[1:], dt, 'b.-', markersize=2)
    axes[0].axhline(np.mean(dt), color='r', linestyle='--',
                    label=f'平均 {np.mean(dt):.1f} ms')
    axes[0].set_xlabel('时间 (秒)')
    axes[0].set_ylabel('记录间隔 (ms)')
    axes[0].set_title('Logger 记录间隔时序图')
    axes[0].legend()
    axes[0].grid(True)

    # 间隔分布直方图
    axes[1].hist(dt, bins=50, edgecolor='black')
    axes[1].axvline(np.mean(dt), color='r', linestyle='--',
                    label=f'平均 {np.mean(dt):.1f} ms ({actual_freq:.1f} Hz)')
    axes[1].set_xlabel('记录间隔 (ms)')
    axes[1].set_ylabel('频次')
    axes[1].set_title('记录间隔分布')
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig('fft_logger_frequency_analysis.png', dpi=150)
    print('图表已保存: fft_logger_frequency_analysis.png')
    plt.show()

    print('=' * 60)

# 使用
if __name__ == '__main__':
    analyze_fft_logger_frequency('log_001.ulg')
```

---

## 十四、总结

### 14.1 频率链条

```
fake_imu 内部: 8000 Hz
  ↓ (FIFO batching)
gyro_fifo 发布: 800 Hz × 10 samples = 8000 Hz 数据率
  ↓ (FFT 缓冲和处理)
FFT 执行: ~125 Hz（理论，单轴）
  ↓ (三轴分时 + 峰值过滤)
FFT 发布: 15-30 Hz（实际）
  ↓ (Logger 采样)
Logger 记录: min(FFT发布, 50Hz) = 15-30 Hz
  ↓
SD 卡: .ulg 文件
```

### 14.2 决定因素

| 频率类型 | 决定因素 | 当前值 | 调整方法 |
|---------|---------|--------|---------|
| **FFT发布** | `IMU_GYRO_FFT_LEN` | 256 → ~25 Hz | 改参数 ✅ |
| **FFT发布** | 重叠率 | 75% | 改代码 ⚠️ |
| **FFT发布** | `IMU_GYRO_FFT_SNR` | 5 | 改参数 ✅ |
| **Logger记录** | logged_topics.cpp | 20ms (50Hz) | 改代码 ✅ |
| **实际记录** | min(发布, Logger) | ~25 Hz | 同时优化 |

### 14.3 快速优化

```bash
# 提高到 ~50 Hz 记录
param set IMU_GYRO_FFT_LEN 128
param set IMU_GYRO_FFT_SNR 3
param save
reboot
```

**预期：10 秒记录 500 点，足够清晰！** ✅

---

**文档版本**: v1.0
**创建日期**: 2025-10-30

