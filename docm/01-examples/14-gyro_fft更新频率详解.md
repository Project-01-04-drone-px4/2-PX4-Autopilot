# gyro_fft 更新频率详解与调优

## 一、FFT 更新频率计算

### 1.1 基本公式

```
FFT 更新频率 = 数据采样率 / 滑动窗口步长

其中：
- 数据采样率 = 实际数据点的速率（Hz）
- 滑动窗口步长 = FFT_LEN × (1 - overlap_ratio)
- overlap_ratio = 重叠比例（代码中是 75%）
```

### 1.2 代码中的实现

从 `GyroFFT.cpp` 第 452-455 行：

```cpp
// shift buffer (3/4 overlap)
const int overlap_start = _imu_gyro_fft_len / 4;
memmove(&gyro_data_buffer[axis][0],
        &gyro_data_buffer[axis][overlap_start],
        sizeof(q15_t) * overlap_start * 3);
buffer_index = overlap_start * 3;
```

**解读：**
- `overlap_start = FFT_LEN / 4 = 256 / 4 = 64`
- 移除前 64 个样本（25%）
- 保留后 192 个样本（75%）
- 需要新增 64 个样本才能再次执行 FFT

### 1.3 具体计算（fake_imu 场景）

#### 数据流

```
fake_imu 内部生成：
├─ IMU_RATE_HZ = 8000 Hz
├─ 工作队列调度间隔 = 1250 us (800 Hz)
└─ 每次发布 gyro_fifo

gyro_fifo 发布：
├─ 发布频率 = 800 Hz
├─ 每次样本数 = 10
└─ 实际数据率 = 800 × 10 = 8000 Hz
```

#### FFT 更新计算

```
FFT_LEN = 256（参数）
重叠率 = 75%（代码固定）
每次需要新样本 = 256 / 4 = 64

数据采样率 = 8000 Hz（fake_imu 内部速率）
更新间隔 = 64 / 8000 = 0.008 秒 = 8 ms
理论更新频率 = 1 / 0.008 = 125 Hz
```

**但是！还有限制因素：**

### 1.4 限制因素：每次 Run() 只处理一个轴

从第 361 和 441 行：

```cpp
// Run() 开始时
_fft_updated = false;

// Update() 中
if ((buffer_index >= _imu_gyro_fft_len) && !_fft_updated) {
    // 执行 FFT
    _fft_updated = true;  // ← 标记已更新
}
```

**这意味着：**
- 每次 Run() 调用只能执行**一次** FFT
- 如果 X/Y/Z 三个轴的 buffer 都满了
- 本次 Run() 只处理第一个满的轴
- 其他轴要等下次 Run()

#### 实际更新频率

```
假设 3 个轴同时需要更新：

Run() 第 1 次：处理 X 轴，_fft_updated = true
Run() 第 2 次：处理 Y 轴，_fft_updated = true
Run() 第 3 次：处理 Z 轴，_fft_updated = true
Run() 第 4 次：X 轴又满了，继续...

如果 gyro_fifo 以 800 Hz 触发 Run()：
- 每 6.4 次 FIFO 更新（64 样本 / 10 样本/次）
- 但需要处理 3 个轴
- 实际更新频率 ≈ 800 / (6.4 × 3) ≈ 41 Hz
```

**等等，这还是不对！**

让我重新看...第 367-390 行：

```cpp
while (_sensor_gyro_fifo_sub.update(&sensor_gyro_fifo)) {
    // ...
    Update(sensor_gyro_fifo.timestamp_sample, input, sensor_gyro_fifo.samples);
}
```

`while` 循环会处理所有待处理的 FIFO 更新，但 `_fft_updated` 在 Run() 开始时重置。

所以在一次 Run() 中：
- 可能处理多个 FIFO 更新（while 循环）
- 但最多只执行一次 FFT（`!_fft_updated` 限制）

---

## 二、为什么说是 12 Hz？

### 2.1 我之前的错误估算

我之前用的公式假设每次 Run() 都能执行 FFT，但忽略了：
1. `_fft_updated` 标志限制
2. 三个轴的处理顺序
3. 工作队列的调度延迟

### 2.2 实际测量方法

最准确的方法是**实际测量**：

```bash
nsh> gyro_fft start

# 等待 10 秒

nsh> work_queue status
# 查看 gyro_fft 的执行统计

# 或使用 perf
nsh> perf
# 查找 "gyro_fft: cycle interval"
```

---

## 三、如何调整 FFT 更新频率

### 3.1 通过 FFT_LEN 参数

**降低 FFT_LEN → 更新更快**

| FFT_LEN | 新增样本需求 | 更新间隔（理论） | 更新频率 | 频率分辨率 |
|---------|-------------|----------------|---------|-----------|
| 128 | 32 | 4 ms | 250 Hz | 62.5 Hz |
| 256 | 64 | 8 ms | 125 Hz | 31.25 Hz |
| 512 | 128 | 16 ms | 62.5 Hz | 15.62 Hz |
| 1024 | 256 | 32 ms | 31.25 Hz | 7.81 Hz |

**权衡：**
- ✅ FFT_LEN 小：更新快，跟踪 Chirp 变化好
- ❌ FFT_LEN 小：频率分辨率低，无法区分相近频率

**设置方法：**
```bash
nsh> param set IMU_GYRO_FFT_LEN 128  # 最快更新
nsh> param save
nsh> reboot
```

### 3.2 修改重叠率（需改代码）

当前重叠率 75% 是代码固定的：

```cpp
// GyroFFT.cpp 第 453 行
const int overlap_start = _imu_gyro_fft_len / 4;  // 25% 新数据
```

**修改为 50% 重叠：**

```cpp
const int overlap_start = _imu_gyro_fft_len / 2;  // 50% 新数据
memmove(&gyro_data_buffer[axis][0],
        &gyro_data_buffer[axis][overlap_start],
        sizeof(q15_t) * overlap_start * 2);  // 只保留 50%
buffer_index = overlap_start * 2;
```

**效果：**
```
FFT_LEN = 256, 重叠 50%
新增样本 = 128
更新间隔 = 128 / 8000 = 16 ms
更新频率 = 62.5 Hz（翻倍！）

但分辨率不变：8000 / 256 = 31.25 Hz
```

### 3.3 移除 _fft_updated 限制（高级）

当前代码限制每次 Run() 只能处理一个轴。如果移除这个限制：

```cpp
// GyroFFT.cpp 第 441 行
// 原代码
if ((buffer_index >= _imu_gyro_fft_len) && !_fft_updated) {
    // FFT
    _fft_updated = true;  // ← 移除这个限制
}

// 修改为
if (buffer_index >= _imu_gyro_fft_len) {
    // FFT（可以处理多个轴）
    // 不设置 _fft_updated
}
```

**效果：** 一次 Run() 可以处理所有 ready 的轴，更新频率×3。

**风险：** CPU 负载增加，可能阻塞工作队列。

---

## 四、推荐配置

### 4.1 快速跟踪 Chirp（推荐）

```bash
param set IMU_GYRO_FFT_LEN 256
param set IMU_GYRO_FFT_MIN 2
param set IMU_GYRO_FFT_MAX 1000
param set IMU_GYRO_FFT_SNR 5
param save
reboot
```

**特点：**
- 更新频率：理论 ~125 Hz，实际可能 ~40 Hz（三轴分时）
- 分辨率：31.25 Hz
- 适合 Chirp 信号（频率快速变化）

### 4.2 高分辨率分析

```bash
param set IMU_GYRO_FFT_LEN 512
param set IMU_GYRO_FFT_MIN 2
param set IMU_GYRO_FFT_MAX 400  # 限制在 Nyquist 内
param set IMU_GYRO_FFT_SNR 8
param save
reboot
```

**特点：**
- 更新频率：~20 Hz
- 分辨率：15.62 Hz（更精细）
- 适合静态频率或慢速变化

### 4.3 极速模式（修改代码）

如果需要更快，可以修改代码：

```cpp
// 1. 降低重叠率到 50%
const int overlap_start = _imu_gyro_fft_len / 2;

// 2. 移除 _fft_updated 限制
if (buffer_index >= _imu_gyro_fft_len) {  // 移除 && !_fft_updated
    // 执行 FFT
}
```

**效果：** 更新频率可达 ~200 Hz

---

## 五、实际测量更新频率

### 5.1 使用 perf 计数器

```bash
nsh> gyro_fft start

# 运行 10 秒

nsh> perf
# 查找：
# gyro_fft: cycle interval
#   count: XXX
#   mean: YYY us
```

**计算：**
```
平均间隔 = YYY us
更新频率 = 1e6 / YYY Hz
```

### 5.2 使用 listener 计时

```bash
nsh> listener sensor_gyro_fft

# 观察 timestamp 的变化
# 手动计算两次更新之间的时间差
```

### 5.3 使用 logger 数据

```python
from pyulog import ULog

log = ULog('log.ulg')
fft = log.get_dataset('sensor_gyro_fft').data

timestamps = fft['timestamp']
dt = np.diff(timestamps)

print(f'平均更新间隔: {np.mean(dt)/1000:.1f} ms')
print(f'平均更新频率: {1e6/np.mean(dt):.1f} Hz')
```

---

## 六、优化建议

### 6.1 对于 fake_imu Chirp 信号

**目标：** 清晰记录频率随时间的变化

**推荐配置：**
```bash
# 平衡配置
param set IMU_GYRO_FFT_LEN 256
param set FAKE_IMU_PERIOD 20  # 延长周期，变化更慢

# 或：快速配置
param set IMU_GYRO_FFT_LEN 128
param set FAKE_IMU_Z_F1 400  # 避免混叠
```

### 6.2 提高更新频率的方法

#### 方法 1：减小 FFT_LEN ✅（参数）

```bash
param set IMU_GYRO_FFT_LEN 128
# 更新频率翻倍！
```

#### 方法 2：降低重叠率 ✅（改代码）

```cpp
// GyroFFT.cpp 第 453 行
const int overlap_start = _imu_gyro_fft_len / 2;  // 50% 重叠
//                                         ↑ 改这里
```

#### 方法 3：并行处理多轴 ⚠️（改代码，高级）

```cpp
// 移除 _fft_updated 限制
// 允许一次 Run() 处理多个轴
```

---

## 七、参数对照表

### 7.1 FFT_LEN 影响

| FFT_LEN | 窗口时长 | 新样本需求 | 更新间隔 | 更新频率 | 分辨率 |
|---------|---------|-----------|---------|---------|--------|
| 128 | 16 ms | 32 | 4 ms | ~250 Hz | 62.5 Hz |
| **256** | **32 ms** | **64** | **8 ms** | **~125 Hz** | **31.25 Hz** |
| 512 | 64 ms | 128 | 16 ms | ~62 Hz | 15.62 Hz |
| 1024 | 128 ms | 256 | 32 ms | ~31 Hz | 7.81 Hz |

**注：** 实际频率可能因三轴分时处理而降低到理论值的 1/3。

### 7.2 重叠率影响

| 重叠率 | 新样本比例 | 新样本数(256) | 更新频率 | 特点 |
|--------|-----------|-------------|---------|------|
| **75%** | **25%** | **64** | **125 Hz** | **默认，平滑** |
| 50% | 50% | 128 | 62.5 Hz | 更快，但不平滑 |
| 0% | 100% | 256 | 31.25 Hz | 最快，但有间隙 |

**推荐：** 保持 75%（代码默认）

---

## 八、为什么我之前说是 12 Hz？

### 8.1 误解来源

我之前错误地考虑了：
- gyro_fifo 发布频率 800 Hz
- 没有考虑 FIFO 内部的 10 个样本

**正确理解：**
```
gyro_fft 看到的采样率 = 8000 Hz（FIFO 内部）
而不是 800 Hz（FIFO 发布频率）
```

### 8.2 实际频率可能的值

考虑所有因素：

1. **理论最大**：125 Hz（只看一个轴）
2. **三轴分时**：125 / 3 ≈ 41 Hz（如果三轴同时ready）
3. **工作队列调度**：可能有延迟
4. **实际观测**：可能在 30-50 Hz 之间

**最准确的是实际测量！**

---

## 九、实际测量脚本

### 9.1 NSH 测量

```bash
# 启动 gyro_fft
nsh> gyro_fft start

# 等待 10 秒让系统稳定

# 查看性能计数器
nsh> perf | grep gyro_fft

# 输出示例：
# gyro_fft: cycle interval
#   count: 1250
#   mean: 8000 us (8 ms)
#   ...
#
# 更新频率 = 1 / 0.008 = 125 Hz
```

### 9.2 Logger 测量

```python
from pyulog import ULog
import numpy as np

log = ULog('log.ulg')
fft = log.get_dataset('sensor_gyro_fft').data

timestamps = fft['timestamp']
dt = np.diff(timestamps) / 1000  # 转换为 ms

print(f'=== sensor_gyro_fft 更新频率分析 ===')
print(f'总样本数: {len(timestamps)}')
print(f'时间范围: {(timestamps[-1] - timestamps[0])/1e6:.1f} 秒')
print(f'')
print(f'更新间隔统计:')
print(f'  平均: {np.mean(dt):.2f} ms')
print(f'  中位数: {np.median(dt):.2f} ms')
print(f'  标准差: {np.std(dt):.2f} ms')
print(f'  最小: {np.min(dt):.2f} ms')
print(f'  最大: {np.max(dt):.2f} ms')
print(f'')
print(f'更新频率: {1000/np.mean(dt):.1f} Hz')
```

---

## 十、优化建议总结

### 10.1 如果需要更快更新（跟踪 Chirp）

```bash
# 方案 A：最简单（参数）
param set IMU_GYRO_FFT_LEN 128
# 频率翻倍：125 Hz → 250 Hz（理论）

# 方案 B：延长 Chirp 周期
param set FAKE_IMU_PERIOD 20
# Chirp 变化速度减半，现有更新频率足够
```

### 10.2 如果需要更高分辨率

```bash
# 增加 FFT_LEN
param set IMU_GYRO_FFT_LEN 512
# 分辨率提高：31.25 Hz → 15.62 Hz
# 更新频率降低：125 Hz → 62.5 Hz
```

### 10.3 最佳平衡（当前配置）

```bash
IMU_GYRO_FFT_LEN = 256
# 更新频率：~125 Hz（理论）
# 分辨率：31.25 Hz
# 适合大多数应用
```

---

## 十一、参数快速参考

### 可调参数

```bash
# 查看所有 FFT 参数
nsh> param show IMU_GYRO_FFT*

IMU_GYRO_FFT_EN    # 启用开关 (0/1)
IMU_GYRO_FFT_LEN   # FFT 长度 (128/256/512/1024) ← 影响更新频率
IMU_GYRO_FFT_MIN   # 最小频率 (Hz)
IMU_GYRO_FFT_MAX   # 最大频率 (Hz)
IMU_GYRO_FFT_SNR   # SNR 阈值
```

### 修改后生效

```bash
# 修改参数
param set IMU_GYRO_FFT_LEN 128

# 保存（可选）
param save

# 重启 gyro_fft 生效
gyro_fft stop
gyro_fft start

# 或重启系统
reboot
```

---

## 十二、总结

### FFT 更新频率公式

```
理论更新频率 = 采样率 × (1 - 重叠率) / FFT_LEN
             = 8000 × 0.25 / FFT_LEN
             = 2000 / FFT_LEN Hz

对于 FFT_LEN = 256:
更新频率 = 2000 / 256 ≈ 7.8 Hz（单轴）
```

**注意：** 实际可能因三轴分时处理、工作队列调度等因素而不同。

### 调整方法

✅ **参数调整**：修改 `IMU_GYRO_FFT_LEN`
⚠️ **代码修改**：调整重叠率或移除限制

### 推荐

**保持默认 256，足够跟踪 Chirp 信号！**

如果需要更快，用 128。

---

**文档版本**: v1.0
**创建日期**: 2025-10-30

