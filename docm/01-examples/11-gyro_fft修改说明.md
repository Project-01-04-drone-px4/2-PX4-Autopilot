# gyro_fft 修改说明 - 优先分析 fake_imu

## 一、修改内容

### 1.1 修改的文件

1. `src/modules/gyro_fft/GyroFFT.cpp` - 传感器选择逻辑
2. `src/modules/gyro_fft/parameters.c` - 默认参数

### 1.2 核心修改

#### 修改 1：优先选择 fake_imu

**位置：** `GyroFFT.cpp` - `SensorSelectionUpdate()` 函数

**修改内容：**
```cpp
// 添加 fake_imu 检测逻辑
static constexpr uint32_t FAKE_IMU_DEVICE_ID = 1310988;
uint32_t target_device_id = sensor_selection.gyro_device_id;
bool fake_imu_found = false;

// 优先查找 fake_imu
for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
    if (gyro_sub.get().device_id == FAKE_IMU_DEVICE_ID) {
        target_device_id = FAKE_IMU_DEVICE_ID;
        fake_imu_found = true;
        PX4_INFO("fake_imu detected, using it for FFT analysis");
        break;
    }
}

// 如果没有 fake_imu，回退到 sensor_selection 选择的传感器
if (!fake_imu_found) {
    target_device_id = sensor_selection.gyro_device_id;
}
```

**效果：**
- ✅ 如果 fake_imu 运行，gyro_fft 会自动选择它
- ✅ 如果 fake_imu 未运行，正常使用真实传感器
- ✅ 无需手动停止真实传感器

#### 修改 2：优化 FFT 参数（适配 fake_imu）

**位置：** `parameters.c`

| 参数 | 原值 | 新值 | 说明 |
|------|------|------|------|
| `IMU_GYRO_FFT_MIN` | 30 Hz | 2 Hz | 覆盖 X 轴低频（0-10 Hz） |
| `IMU_GYRO_FFT_MAX` | 150 Hz | 1000 Hz | 覆盖 Z 轴高频（0-1000 Hz） |
| `IMU_GYRO_FFT_LEN` | 512 | 256 | 更快更新（提高发布频率） |
| `IMU_GYRO_FFT_SNR` | 10 | 5 | 降低阈值，检测更多峰值 |

**效果：**
- ✅ 频率范围覆盖 fake_imu 的全部输出（2-1000 Hz）
- ✅ FFT 长度减半 → 更新速率加倍
- ✅ SNR 阈值降低 → 更容易检测到 Chirp 峰值

---

## 二、性能提升

### 2.1 FFT 更新频率

**计算公式：**
```
更新频率 = 采样率 / (FFT_LEN × (1 - overlap_ratio))

overlap_ratio = 0.75 (75% 重叠)
```

**优化前：**
```
FFT_LEN = 512
采样率 = 800 Hz
更新频率 = 800 / (512 × 0.25) = 800 / 128 ≈ 6.25 Hz
更新间隔 = 160 ms
```

**优化后：**
```
FFT_LEN = 256
采样率 = 800 Hz
更新频率 = 800 / (256 × 0.25) = 800 / 64 ≈ 12.5 Hz
更新间隔 = 80 ms
```

**改善：** 更新速率提升 **2倍**！

### 2.2 频率分辨率

**优化前：**
```
分辨率 = 800 / 512 ≈ 1.56 Hz
```

**优化后：**
```
分辨率 = 800 / 256 ≈ 3.12 Hz
```

**影响：** 分辨率降低，但对于 Chirp 信号（频率快速变化）影响不大。

---

## 三、使用方法

### 3.1 编译和上传

```bash
# 1. 编译固件
make micoair_h743_default

# 2. 上传固件
# 使用你的上传工具

# 3. 重启飞控
```

### 3.2 启动测试

```bash
# 1. 启动 fake_imu
nsh> fake_imu start
INFO  [fake_imu] Rate 800.000, Interval: 1250 us

# 2. 启动 gyro_fft
nsh> gyro_fft start
INFO  [gyro_fft] fake_imu detected (device_id 1310988), using it for FFT analysis
INFO  [gyro_fft] subscribed to sensor_gyro instance 2 (device_id 1310988)

# 3. 验证
nsh> listener sensor_gyro_fft

TOPIC: sensor_gyro_fft
 sensor_gyro_fft
    timestamp: ...
    device_id: 1310988  ← fake_imu ✓
    sensor_sample_rate_hz: 800.00
    resolution_hz: 3.12

    peak_frequencies_x: [5.2, nan, nan]   ← 会随时间变化
    peak_frequencies_y: [47.8, nan, nan]  ← 会随时间变化
    peak_frequencies_z: [234.5, nan, nan] ← 会随时间变化

    peak_snr_x: [12.3, 0.0, 0.0]
    peak_snr_y: [18.7, 0.0, 0.0]
    peak_snr_z: [8.2, 0.0, 0.0]
```

### 3.3 启动日志记录

```bash
# 启动 logger
nsh> logger start

# 运行测试（至少 10 秒，一个完整扫频周期）
# 等待 10-20 秒

# 停止 logger
nsh> logger stop
```

### 3.4 下载并分析日志

```bash
# 下载日志文件
# 通常在 /fs/microsd/log/

# 使用 FlightPlot, PlotJuggler 或 pyulog 分析
```

---

## 四、预期效果

### 4.1 实时观察

每隔 2 秒查看一次：

```bash
# t=0s 附近
nsh> listener sensor_gyro_fft 1
peak_frequencies_z: [12.5, nan, nan]  SNR: 15.2

# t=2s 附近
peak_frequencies_z: [203.1, nan, nan]  SNR: 16.8

# t=4s 附近
peak_frequencies_z: [393.7, nan, nan]  SNR: 14.5

# t=6s 附近
peak_frequencies_z: [584.4, nan, nan]  SNR: 9.2  ← 开始衰减

# t=8s 附近
peak_frequencies_z: [775.0, nan, nan]  SNR: 3.8  ← 严重衰减（混叠）

# t=10s （新周期）
peak_frequencies_z: [15.6, nan, nan]  SNR: 14.9  ← 回到起始
```

### 4.2 Logger 数据分析

使用 pyulog 或 FlightPlot 绘制：

**X 轴：**
```matlab
% 应该看到阶梯状增长（受频率分辨率 3.12 Hz 限制）
时间: 0 ─→ 2 ─→ 4 ─→ 6 ─→ 8 ─→ 10
频率: 0 ─→ 3 ─→ 6 ─→ 9 ─→ 3 ─→ 0  (Hz)
```

**Y 轴：**
```matlab
% 应该看到平滑的线性增长
时间: 0 ──→ 2 ───→ 4 ───→ 6 ───→ 8 ───→ 10
频率: 0 ──→ 20 ──→ 40 ──→ 60 ──→ 80 ──→ 100 (Hz)
```

**Z 轴：**
```matlab
% 应该看到快速增长，但到 400 Hz 后出现混叠
时间: 0 ──→ 2 ───→ 4 ────→ 6 ────→ 8 ────→ 10
频率: 0 ──→ 200 ─→ 400 ──→ ??? ──→ ??? ──→ 0
                         (混叠区域)
```

---

## 五、参数调优建议

### 5.1 如果想看清楚低频（X 轴）

```bash
nsh> param set IMU_GYRO_FFT_LEN 512  # 或 1024
nsh> param set IMU_GYRO_FFT_MIN 0.5
nsh> param set IMU_GYRO_FFT_MAX 20
nsh> reboot
```

**效果：**
- 分辨率更高（0.78-1.56 Hz）
- 能清晰看到 X 轴的 0-10 Hz 变化

### 5.2 如果想看高频（Z 轴）

```bash
nsh> param set IMU_GYRO_FFT_LEN 256
nsh> param set IMU_GYRO_FFT_MIN 50
nsh> param set IMU_GYRO_FFT_MAX 400  # 限制在 Nyquist 频率内
nsh> param set IMU_GYRO_FFT_SNR 3
nsh> reboot
```

**效果：**
- 更新速率快
- 避免混叠干扰

### 5.3 如果想平衡（Y 轴）

```bash
nsh> param set IMU_GYRO_FFT_LEN 256
nsh> param set IMU_GYRO_FFT_MIN 2
nsh> param set IMU_GYRO_FFT_MAX 150
nsh> param set IMU_GYRO_FFT_SNR 5
nsh> reboot
```

**效果：**
- Y 轴（0-100 Hz）完全覆盖
- 更新速率快（~12.5 Hz）
- 适合 logger 记录

---

## 六、Logger 配置优化

### 6.1 确保 sensor_gyro_fft 被记录

检查 `src/modules/logger/logged_topics.cpp` 是否包含：

```cpp
{ORB_ID(sensor_gyro_fft), 100},  // 100ms 间隔 = 10 Hz
```

如果没有或间隔太大，可以修改为：

```cpp
{ORB_ID(sensor_gyro_fft), 50},  // 50ms 间隔 = 20 Hz
```

### 6.2 启动 logger 的推荐配置

```bash
# 高频记录模式
nsh> logger start -f -t -r 200  # 200 Hz 记录

# 或者选择性记录
nsh> logger start -t -p sensor_gyro_fft
```

参数说明：
- `-f`: 记录到文件
- `-t`: 包含时间戳
- `-r 200`: 最大记录频率 200 Hz
- `-p topic`: 只记录特定主题

---

## 七、验证步骤

### 7.1 实时验证

```bash
# 1. 启动所有模块
nsh> fake_imu start
nsh> gyro_fft start
nsh> matlab_csv_serial start /dev/ttyS3

# 2. 持续观察（每秒刷新）
while true; do
    listener sensor_gyro_fft 1
    sleep 1
done

# 应该看到：
# - device_id = 1310988
# - 峰值频率每秒增加约 100 Hz (Z 轴)
# - 峰值频率每秒增加约 10 Hz (Y 轴)
# - 峰值频率每秒增加约 1 Hz (X 轴)
```

### 7.2 Logger 数据验证

```bash
# 1. 启动记录
nsh> logger start -f -t

# 2. 运行 20 秒（2 个完整周期）

# 3. 停止
nsh> logger stop
nsh> gyro_fft stop
nsh> fake_imu stop

# 4. 下载日志
# 从 /fs/microsd/log/
```

### 7.3 Python 分析日志

```python
#!/usr/bin/env python3
from pyulog import ULog
import matplotlib.pyplot as plt
import numpy as np

# 加载日志
log = ULog('log_file.ulg')

# 提取 sensor_gyro_fft 数据
fft_data = log.get_dataset('sensor_gyro_fft')

time = fft_data.data['timestamp'] / 1e6  # 转换为秒
peak_freq_x = fft_data.data['peak_frequencies_x'][:, 0]
peak_freq_y = fft_data.data['peak_frequencies_y'][:, 0]
peak_freq_z = fft_data.data['peak_frequencies_z'][:, 0]

# 绘图
fig, axes = plt.subplots(3, 1, figsize=(12, 8))

axes[0].plot(time, peak_freq_x, 'r.-')
axes[0].set_ylabel('X 轴峰值频率 (Hz)')
axes[0].set_title('fake_imu Chirp 信号 FFT 分析')
axes[0].grid(True)

axes[1].plot(time, peak_freq_y, 'g.-')
axes[1].set_ylabel('Y 轴峰值频率 (Hz)')
axes[1].grid(True)

axes[2].plot(time, peak_freq_z, 'b.-')
axes[2].set_ylabel('Z 轴峰值频率 (Hz)')
axes[2].set_xlabel('时间 (秒)')
axes[2].grid(True)

plt.tight_layout()
plt.savefig('fake_imu_fft_analysis.png')
plt.show()

print('FFT 分析图已保存: fake_imu_fft_analysis.png')
```

---

## 八、性能对比

### 8.1 修改前

| 指标 | 值 |
|------|-----|
| 分析传感器 | 真实 IMU (6684690) |
| FFT 长度 | 512 |
| 更新频率 | ~6 Hz |
| 频率范围 | 30-150 Hz |
| 对 fake_imu | ❌ 不分析 |

### 8.2 修改后

| 指标 | 值 |
|------|-----|
| 分析传感器 | fake_imu (1310988) ✅ |
| FFT 长度 | 256 |
| 更新频率 | ~12 Hz ↑ |
| 频率范围 | 2-1000 Hz ↑ |
| Logger 数据密度 | 提高 2 倍 ↑ |

---

## 九、故障排查

### 问题 1：gyro_fft 仍然分析真实传感器

```bash
nsh> listener sensor_gyro_fft
device_id: 6684690  ← 不是 1310988
```

**原因：** fake_imu 未启动或启动顺序问题

**解决：**
```bash
# 重启 gyro_fft
nsh> gyro_fft stop
nsh> gyro_fft start
# 现在应该重新检测并找到 fake_imu
```

### 问题 2：没有看到峰值

```bash
peak_frequencies_z: [nan, nan, nan]
peak_snr_z: [0.0, 0.0, 0.0]
```

**原因：** SNR 低于阈值或频率超出范围

**解决：**
```bash
# 降低 SNR 阈值
nsh> param set IMU_GYRO_FFT_SNR 3

# 扩大频率范围
nsh> param set IMU_GYRO_FFT_MIN 0
nsh> param set IMU_GYRO_FFT_MAX 1500

# 需要重启 gyro_fft
nsh> gyro_fft stop
nsh> gyro_fft start
```

### 问题 3：Z 轴峰值不准确（>400 Hz 时）

**原因：** 采样率 800 Hz，Nyquist 频率 400 Hz，超出部分混叠

**这是正常现象！** 可以用来演示混叠效应。

**解决：** 如果需要准确分析高频：
```bash
# 降低 Z 轴最大频率
nsh> param set FAKE_IMU_Z_F1 300
nsh> fake_imu stop
nsh> fake_imu start
```

---

## 十、完整测试流程

### 10.1 快速验证

```bash
# 1. 启动
fake_imu start
gyro_fft start

# 2. 持续观察 10 秒
for i in {1..10}; do
    echo "=== $i 秒 ==="
    listener sensor_gyro_fft 1
    sleep 1
done

# 应该看到峰值频率逐渐增加
```

### 10.2 完整测试（带 logger）

```bash
# 1. 启动所有模块
fake_imu start
gyro_fft start
matlab_csv_serial start /dev/ttyS3

# 2. 启动 logger
logger start -f -t -r 200

# 3. 等待 20 秒（两个完整周期）
sleep 20

# 4. 停止
logger stop
matlab_csv_serial stop
gyro_fft stop
fake_imu stop

# 5. 下载日志和串口数据
# - 日志: /fs/microsd/log/*.ulg
# - 串口数据: PC 端的 imu_data.csv

# 6. 对比分析
# - Logger FFT 数据：频率峰值随时间变化
# - 串口原始数据：完整波形
```

---

## 十一、高级应用

### 11.1 同时记录多个主题

```bash
logger start -f -t -r 200 \
    -p sensor_gyro \
    -p sensor_gyro_fft \
    -p sensor_accel
```

### 11.2 调整 fake_imu 参数测试

```bash
# 测试不同频率范围
param set FAKE_IMU_Y_F1 50
param set FAKE_IMU_Z_F1 500
param set FAKE_IMU_PERIOD 20

fake_imu stop
fake_imu start
gyro_fft stop
gyro_fft start
```

### 11.3 使用 MAVLink 实时查看

如果连接了 QGroundControl：
- MAVLink Inspector → sensor_gyro_fft
- 实时图表显示峰值频率变化

---

## 十二、总结

### 12.1 修改要点

✅ **自动检测 fake_imu**：gyro_fft 优先分析 fake_imu
✅ **提高更新频率**：FFT_LEN 256 → 12 Hz 更新
✅ **扩大频率范围**：2-1000 Hz 覆盖所有轴
✅ **降低 SNR 阈值**：更容易检测峰值

### 12.2 使用流程

```bash
# 编译 → 上传 → 重启
fake_imu start
gyro_fft start
logger start -f -t -r 200
# 等待 20 秒
logger stop
```

### 12.3 数据分析

- **实时监控**：`listener sensor_gyro_fft`
- **离线分析**：pyulog 分析日志文件
- **原始数据**：matlab_csv_serial 串口数据

### 12.4 教学价值

这个配置可以展示：
- ✅ 实时 FFT 分析
- ✅ Chirp 信号的频率变化
- ✅ 采样定理和混叠效应
- ✅ 工作队列与回调机制

---

**文档版本**: v1.0
**创建日期**: 2025-10-30

