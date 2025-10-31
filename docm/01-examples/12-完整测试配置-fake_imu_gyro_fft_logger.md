# 完整测试配置：fake_imu + gyro_fft + logger

## 一、系统架构

```
┌──────────────┐  sensor_gyro_fifo  ┌──────────────┐  sensor_gyro_fft  ┌──────────┐
│  fake_imu    │  ─────────────────> │  gyro_fft    │  ───────────────> │  logger  │
│  (工作队列)   │  800 Hz, 10 samples │  (工作队列)   │  ~12 Hz           │  (任务)   │
│              │                     │              │                   │          │
│ 生成 Chirp   │                     │ 实时 FFT     │                   │ 记录到   │
│ 正弦波       │                     │ 峰值检测     │                   │ SD 卡    │
└──────────────┘                     └──────────────┘                   └──────────┘
       │
       │ sensor_accel
       ↓
┌──────────────────┐  CSV via UART  ┌─────────┐
│ matlab_csv_serial│  ────────────> │   PC    │
│   (独立任务)      │  800 Hz        │ MATLAB  │
└──────────────────┘                └─────────┘
```

---

## 二、已完成的修改

### 2.1 gyro_fft 修改

**文件：** `src/modules/gyro_fft/GyroFFT.cpp`

✅ **优先选择 fake_imu**：如果 device_id 1310988 存在，自动使用它
✅ **自动订阅 gyro_fifo**：fake_imu 发布的是 gyro_fifo，包含更多样本

**文件：** `src/modules/gyro_fft/parameters.c`

✅ **启用 FFT**：`IMU_GYRO_FFT_EN = 1` （默认启用）
✅ **频率范围**：2-1000 Hz（覆盖 fake_imu 全部范围）
✅ **FFT 长度**：256（更新频率 ~12 Hz）
✅ **SNR 阈值**：5（更容易检测峰值）

### 2.2 logger 修改

**文件：** `src/modules/logger/logged_topics.cpp`

✅ **记录间隔**：50ms → **20ms** (50 Hz)
✅ **高于 FFT 更新频率**：logger 50 Hz > gyro_fft 12 Hz → 完整记录

---

## 三、fake_imu 发布 gyro_fifo 详解

### 3.1 FIFO 格式优势

```cpp
// sensor_gyro_fifo_s 结构
struct sensor_gyro_fifo_s {
    uint64_t timestamp;
    uint64_t timestamp_sample;
    uint32_t device_id;
    float dt;                    // 样本间隔
    uint8_t samples;             // 样本数（fake_imu 默认 10）
    int16_t x[32];              // X 轴数组（最多32个样本）
    int16_t y[32];              // Y 轴数组
    int16_t z[32];              // Z 轴数组
    // ...
};
```

**vs sensor_gyro_s（单样本）：**
```cpp
struct sensor_gyro_s {
    uint64_t timestamp;
    uint32_t device_id;
    float x;                     // 单个值
    float y;
    float z;
    // ...
};
```

### 3.2 fake_imu 的 FIFO 配置

从 `FakeImu.cpp` 第 69 行：

```cpp
gyro.samples = roundf(IMU_RATE_HZ / (1e6 / _sensor_interval_us));
//           = roundf(8000 / (1e6 / 1250))
//           = roundf(8000 / 800)
//           = 10
```

**每次发布包含 10 个样本！**

| 参数 | 值 |
|------|-----|
| IMU_RATE_HZ | 8000 Hz（内部生成速率） |
| 发布频率 | 800 Hz |
| 每次样本数 | 10 |
| 实际数据率 | 8000 Hz（10 × 800） |

**优势：**
- gyro_fft 一次处理 10 个样本，效率高
- 减少 uORB 通信开销
- 更准确的 FFT（更多数据点）

---

## 四、编译和配置

### 4.1 重新编译

```bash
make micoair_h743_default
```

**修改的文件：**
- `src/modules/gyro_fft/GyroFFT.cpp`
- `src/modules/gyro_fft/parameters.c`
- `src/modules/logger/logged_topics.cpp`

### 4.2 参数验证

上传固件后，在 NSH 中检查：

```bash
nsh> param show IMU_GYRO_FFT*

# 应该看到：
IMU_GYRO_FFT_EN    1        # ✓ 启用
IMU_GYRO_FFT_LEN   256      # ✓ 快速更新
IMU_GYRO_FFT_MAX   1000.0   # ✓ 高频覆盖
IMU_GYRO_FFT_MIN   2.0      # ✓ 低频覆盖
IMU_GYRO_FFT_SNR   5.0      # ✓ 低阈值
```

如果参数不对，手动设置：

```bash
nsh> param set IMU_GYRO_FFT_EN 1
nsh> param set IMU_GYRO_FFT_LEN 256
nsh> param set IMU_GYRO_FFT_MIN 2
nsh> param set IMU_GYRO_FFT_MAX 1000
nsh> param set IMU_GYRO_FFT_SNR 5
nsh> param save
nsh> reboot
```

---

## 五、完整测试流程

### 5.1 启动所有模块

```bash
# 1. 启动 fake_imu
nsh> fake_imu start
INFO  [fake_imu] Rate 800.000, Interval: 1250 us

# 2. 启动 gyro_fft（会自动选择 fake_imu）
nsh> gyro_fft start
INFO  [gyro_fft] fake_imu detected (device_id 1310988), using it for FFT analysis
INFO  [gyro_fft] subscribed to sensor_gyro_fifo instance 2 (device_id 1310988)

# 3. 验证 FFT 正在工作
nsh> listener sensor_gyro_fft 1

# 应该看到：
sensor_gyro_fft
    device_id: 1310988  ← fake_imu ✓
    sensor_sample_rate_hz: 8000.00  ← FIFO 内部采样率
    resolution_hz: 31.25  (8000/256)
    peak_frequencies_x: [6.25, nan, nan]
    peak_frequencies_y: [62.50, nan, nan]
    peak_frequencies_z: [312.50, nan, nan]
    peak_snr_x: [15.3, 0.0, 0.0]
    peak_snr_y: [18.7, 0.0, 0.0]
    peak_snr_z: [12.4, 0.0, 0.0]
```

**关键验证点：**
- ✅ `device_id: 1310988`
- ✅ `sensor_sample_rate_hz: 8000` （FIFO 内部速率）
- ✅ 峰值频率不是 nan

### 5.2 启动数据采集

```bash
# 4. 启动串口采集（可选，用于原始数据）
nsh> matlab_csv_serial start /dev/ttyS3

# 5. 启动 logger
nsh> logger start -f -t -r 200

INFO  [logger] logger started (mode=all)
```

### 5.3 运行测试

```bash
# 运行至少 20 秒（2 个完整扫频周期）
# 可以实时观察：
while true; do
    echo "=== $(date +%T) ==="
    listener sensor_gyro_fft 1 | grep "peak_frequencies_z\|peak_snr_z"
    sleep 2
done

# Ctrl+C 停止观察
```

### 5.4 停止并保存

```bash
# 停止所有模块
nsh> logger stop
nsh> matlab_csv_serial stop
nsh> gyro_fft stop
nsh> fake_imu stop

# 下载数据
# 1. Logger 日志: /fs/microsd/log/*.ulg
# 2. 串口数据: PC 端的 imu_data.csv
```

---

## 六、数据分析

### 6.1 使用 FlightPlot / PlotJuggler

```bash
# 安装 PlotJuggler
sudo apt install ros-*-plotjuggler

# 打开日志
plotjuggler /path/to/log.ulg
```

**绘制：**
1. `sensor_gyro_fft/peak_frequencies_x[0]` vs time
2. `sensor_gyro_fft/peak_frequencies_y[0]` vs time
3. `sensor_gyro_fft/peak_frequencies_z[0]` vs time

**预期效果：**
- X 轴：0 → 10 Hz 的阶梯状增长（每 10 秒循环）
- Y 轴：0 → 100 Hz 的线性增长（每 10 秒循环）
- Z 轴：0 → 400 Hz 的快速增长，然后混叠（每 10 秒循环）

### 6.2 使用 pyulog（Python）

```python
#!/usr/bin/env python3
from pyulog import ULog
import matplotlib.pyplot as plt
import numpy as np

# 加载日志
log = ULog('log_file.ulg')

# 提取数据
fft = log.get_dataset('sensor_gyro_fft').data
time = (fft['timestamp'] - fft['timestamp'][0]) / 1e6  # 相对时间（秒）

# 提取峰值频率（第一个峰值）
peak_x = fft['peak_frequencies_x'][:, 0]
peak_y = fft['peak_frequencies_y'][:, 0]
peak_z = fft['peak_frequencies_z'][:, 0]

# 提取 SNR
snr_x = fft['peak_snr_x'][:, 0]
snr_y = fft['peak_snr_y'][:, 0]
snr_z = fft['peak_snr_z'][:, 0]

# 绘图
fig, axes = plt.subplots(3, 2, figsize=(14, 10))

# X 轴
axes[0,0].plot(time, peak_x, 'r.-', markersize=3)
axes[0,0].set_ylabel('峰值频率 (Hz)')
axes[0,0].set_title('X 轴频率 (0-10 Hz Chirp)')
axes[0,0].grid(True)

axes[0,1].plot(time, snr_x, 'r.-', markersize=3)
axes[0,1].set_ylabel('SNR')
axes[0,1].set_title('X 轴 SNR')
axes[0,1].grid(True)

# Y 轴
axes[1,0].plot(time, peak_y, 'g.-', markersize=3)
axes[1,0].set_ylabel('峰值频率 (Hz)')
axes[1,0].set_title('Y 轴频率 (0-100 Hz Chirp)')
axes[1,0].grid(True)

axes[1,1].plot(time, snr_y, 'g.-', markersize=3)
axes[1,1].set_ylabel('SNR')
axes[1,1].set_title('Y 轴 SNR')
axes[1,1].grid(True)

# Z 轴
axes[2,0].plot(time, peak_z, 'b.-', markersize=3)
axes[2,0].set_xlabel('时间 (秒)')
axes[2,0].set_ylabel('峰值频率 (Hz)')
axes[2,0].set_title('Z 轴频率 (0-1000 Hz Chirp, >400Hz混叠)')
axes[2,0].axhline(400, color='r', linestyle='--', label='Nyquist 频率')
axes[2,0].legend()
axes[2,0].grid(True)

axes[2,1].plot(time, snr_z, 'b.-', markersize=3)
axes[2,1].set_xlabel('时间 (秒)')
axes[2,1].set_ylabel('SNR')
axes[2,1].set_title('Z 轴 SNR')
axes[2,1].grid(True)

plt.tight_layout()
plt.savefig('fake_imu_fft_logger_analysis.png', dpi=150)
print('图表已保存: fake_imu_fft_logger_analysis.png')
plt.show()
```

---

## 七、关键配置总结

### 7.1 fake_imu 发布的是 sensor_gyro_fifo ✅

```cpp
// FakeImu.cpp 第 67-122 行
sensor_gyro_fifo_s gyro{};
gyro.samples = 10;  // 每次 10 个样本

for (int n = 0; n < 10; n++) {
    gyro.x[n] = ...;  // 填充 10 个样本
    gyro.y[n] = ...;
    gyro.z[n] = ...;
}

_px4_gyro.updateFIFO(gyro);  // 发布 sensor_gyro_fifo
```

**参数：**
- 发布频率：800 Hz
- 每次样本数：10
- 实际数据率：8000 Hz（10 × 800）

### 7.2 gyro_fft 优先使用 gyro_fifo ✅

```cpp
// GyroFFT.cpp 第 198-217 行
// 优先查找 sensor_gyro_fifo
if (sensor_gyro_fifo_sub.get().device_id == target_device_id) {
    _sensor_gyro_fifo_sub.ChangeInstance(i);
    _sensor_gyro_fifo_sub.registerCallback();
    _gyro_fifo = true;  // ← 标记使用 FIFO
}

// 第 357-384 行：处理逻辑
if (_gyro_fifo) {
    sensor_gyro_fifo_s sensor_gyro_fifo;
    while (_sensor_gyro_fifo_sub.update(&sensor_gyro_fifo)) {
        int16_t *input[] {
            sensor_gyro_fifo.x,  // 10 个样本
            sensor_gyro_fifo.y,
            sensor_gyro_fifo.z
        };
        Update(..., input, sensor_gyro_fifo.samples);  // 处理 10 个样本
    }
}
```

**优势：**
- ✅ 每次处理 10 个样本，FFT 效率高
- ✅ 使用内部 8kHz 数据，频率分辨率更高
- ✅ 减少回调次数（800 Hz vs 8000 Hz）

### 7.3 logger 记录配置 ✅

```cpp
// logged_topics.cpp 第 123 行
add_optional_topic("sensor_gyro_fft", 20);
//                                    ↑
//                                 20ms = 50 Hz
```

**记录性能：**
- gyro_fft 更新：~12 Hz
- logger 记录：50 Hz
- **过采样率**：50/12 ≈ 4x
- **效果**：完整记录，无遗漏

---

## 八、参数说明

### 8.1 FFT 相关参数

| 参数 | 默认值 | 说明 | 推荐值 |
|------|--------|------|--------|
| `IMU_GYRO_FFT_EN` | 1 | FFT 启用开关 | 1 (启用) |
| `IMU_GYRO_FFT_LEN` | 256 | FFT 长度 | 256 (快速) |
| `IMU_GYRO_FFT_MIN` | 2 Hz | 最小检测频率 | 2 Hz |
| `IMU_GYRO_FFT_MAX` | 1000 Hz | 最大检测频率 | 1000 Hz |
| `IMU_GYRO_FFT_SNR` | 5 | 信噪比阈值 | 5 (敏感) |

### 8.2 fake_imu 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `FAKE_IMU_X_F0` | 0 Hz | X轴起始频率 |
| `FAKE_IMU_X_F1` | 10 Hz | X轴结束频率 |
| `FAKE_IMU_Y_F0` | 0 Hz | Y轴起始频率 |
| `FAKE_IMU_Y_F1` | 100 Hz | Y轴结束频率 |
| `FAKE_IMU_Z_F0` | 0 Hz | Z轴起始频率 |
| `FAKE_IMU_Z_F1` | 1000 Hz | Z轴结束频率 |
| `FAKE_IMU_PERIOD` | 10 s | 扫频周期 |

### 8.3 建议的测试配置

#### 配置 A：全频段测试（默认）
```bash
# 保持默认值
# X: 0-10 Hz
# Y: 0-100 Hz
# Z: 0-1000 Hz (会混叠)
```

#### 配置 B：避免混叠
```bash
param set FAKE_IMU_Z_F1 300  # 300 < 400 (Nyquist)
param set FAKE_IMU_PERIOD 15  # 延长周期
```

#### 配置 C：高分辨率测试
```bash
param set IMU_GYRO_FFT_LEN 512
param set FAKE_IMU_Y_F1 200
param set FAKE_IMU_Z_F1 400
```

---

## 九、预期数据质量

### 9.1 FFT 更新频率

```
FFT_LEN = 256
FIFO 样本数 = 10
发布频率 = 800 Hz

FFT 需要的样本数 = 256
收集时间 = 256 / 8000 = 0.032 秒 = 32 ms

考虑 75% 重叠：
更新间隔 = 256 / 4 / 800 = 0.08 秒 = 80 ms
更新频率 = 12.5 Hz
```

### 9.2 Logger 记录质量

```
Logger 间隔: 20 ms (50 Hz)
FFT 更新: 80 ms (12.5 Hz)

每个 FFT 更新会被记录: 80/20 = 4 次
→ 有冗余，但确保不丢失
```

### 9.3 10 秒完整周期的数据点

```
FFT 数据点: 12.5 Hz × 10 s = 125 点
Logger 记录: 50 Hz × 10 s = 500 点（包含重复）

足够绘制平滑的频率变化曲线！
```

---

## 十、故障排查

### 问题 1：gyro_fft 没有自动选择 fake_imu

```bash
nsh> listener sensor_gyro_fft
device_id: 6684690  ← 还是真实传感器
```

**检查：**
```bash
# 1. fake_imu 是否运行？
nsh> listener sensor_gyro
# 查看是否有 device_id 1310988 的实例

# 2. 重启 gyro_fft
nsh> gyro_fft stop
nsh> gyro_fft start
```

### 问题 2：FFT 没有输出峰值

```bash
peak_frequencies_z: [nan, nan, nan]
```

**原因和解决：**

1. **SNR 太高**
   ```bash
   param set IMU_GYRO_FFT_SNR 3
   ```

2. **频率超出范围**
   ```bash
   param show IMU_GYRO_FFT_MIN  # 应该 ≤ 2
   param show IMU_GYRO_FFT_MAX  # 应该 ≥ 1000
   ```

3. **FFT 未启用**
   ```bash
   param show IMU_GYRO_FFT_EN  # 应该 = 1
   ```

### 问题 3：Logger 没有记录 sensor_gyro_fft

```bash
# 检查日志中是否包含该主题
# 下载 .ulg 文件后
ulog_info log_file.ulg | grep gyro_fft
```

如果没有：
```bash
# 启动时明确指定
logger start -f -t -p sensor_gyro_fft
```

---

## 十一、完整参数清单

### 启动前检查

```bash
# === fake_imu 参数 ===
param show FAKE_IMU_X_F0     # 应该 0
param show FAKE_IMU_X_F1     # 应该 10
param show FAKE_IMU_Y_F0     # 应该 0
param show FAKE_IMU_Y_F1     # 应该 100
param show FAKE_IMU_Z_F0     # 应该 0
param show FAKE_IMU_Z_F1     # 应该 1000 (或 300 避免混叠)
param show FAKE_IMU_PERIOD   # 应该 10

# === gyro_fft 参数 ===
param show IMU_GYRO_FFT_EN   # 应该 1
param show IMU_GYRO_FFT_LEN  # 应该 256
param show IMU_GYRO_FFT_MIN  # 应该 2
param show IMU_GYRO_FFT_MAX  # 应该 1000
param show IMU_GYRO_FFT_SNR  # 应该 5

# 如果不对，重新设置并保存
param save
reboot
```

---

## 十二、总结

### 12.1 确认答案

✅ **gyro_fft 订阅的是 gyro_fifo**
- 代码优先使用 sensor_gyro_fifo（如果可用）
- fake_imu 发布的就是 gyro_fifo（每次 10 个样本）
- 所以 gyro_fft 会使用 FIFO 数据

✅ **FFT 参数已启用**
- `IMU_GYRO_FFT_EN = 1`（默认启用）
- 其他参数已优化

✅ **logger 会记录 sensor_gyro_fft**
- `logged_topics.cpp` 已配置
- 间隔优化为 20ms (50 Hz)

### 12.2 关键改进

| 改进项 | 修改内容 | 效果 |
|--------|---------|------|
| **传感器选择** | 优先 fake_imu | 自动分析测试数据 |
| **FFT 参数** | EN=1, LEN=256 | 启用且快速更新 |
| **频率范围** | 2-1000 Hz | 覆盖全部轴 |
| **Logger 间隔** | 50ms → 20ms | 记录更密集 |

### 12.3 数据流

```
fake_imu (8kHz内部)
  → sensor_gyro_fifo (800Hz, 10样本/次)
    → gyro_fft (FFT分析, ~12Hz更新)
      → sensor_gyro_fft (峰值频率)
        → logger (50Hz记录)
          → SD卡 (.ulg文件)
```

现在重新编译，一切都配置好了！🚀
