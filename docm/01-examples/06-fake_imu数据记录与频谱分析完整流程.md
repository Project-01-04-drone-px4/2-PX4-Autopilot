# 06-fake_imu 数据记录与频谱分析完整流程

## 1. 概述

本文档提供 `fake_imu` 从参数配置、数据记录、日志下载到频谱分析的**完整实战流程**。

---

## 2. 完整流程概览

```
┌────────────────────────────────────────────────────────┐
│ 步骤1: 参数配置                                        │
│   - 设置扫频范围和周期                                  │
│   - 保存参数                                           │
└────────────┬───────────────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────────────┐
│ 步骤2: 启动模块                                        │
│   - 重启飞控                                           │
│   - 启动 fake_imu                                      │
│   - 启动 gyro_fft（可选）                              │
└────────────┬───────────────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────────────┐
│ 步骤3: 启动日志记录                                    │
│   - logger on                                          │
│   - 等待扫频完成                                        │
│   - logger off                                         │
└────────────┬───────────────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────────────┐
│ 步骤4: 下载日志文件                                    │
│   - 通过 MAVLink / USB 下载                            │
│   - .ulg 格式                                          │
└────────────┬───────────────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────────────┐
│ 步骤5: 数据分析                                        │
│   - 使用 FlightPlot 可视化                             │
│   - 使用 pyulog 提取数据                               │
│   - 使用 MATLAB/Python 进行频谱分析                    │
└────────────────────────────────────────────────────────┘
```

---

## 3. 步骤1：参数配置

### 3.1 配置扫频参数

#### 场景1：测试完整频谱（默认）

```bash
# 连接飞控（USB 或串口）
nsh> param show FAKE_IMU*

# 查看默认值（如果是首次使用）
# FAKE_IMU_X_F0 = 0.0
# FAKE_IMU_X_F1 = 10.0
# FAKE_IMU_Y_F0 = 0.0
# FAKE_IMU_Y_F1 = 100.0
# FAKE_IMU_Z_F0 = 0.0
# FAKE_IMU_Z_F1 = 1000.0
# FAKE_IMU_PERIOD = 10.0

# 默认配置即可，不需要修改
```

---

#### 场景2：测试电机振动频率（100-800 Hz）

```bash
# Z 轴设置为电机常见频率范围
nsh> param set FAKE_IMU_Z_F0 100
nsh> param set FAKE_IMU_Z_F1 800
nsh> param set FAKE_IMU_PERIOD 15

# X 和 Y 轴设为 0（不扫频）
nsh> param set FAKE_IMU_X_F0 0
nsh> param set FAKE_IMU_X_F1 0
nsh> param set FAKE_IMU_Y_F0 0
nsh> param set FAKE_IMU_Y_F1 0

# 保存参数
nsh> param save
```

---

#### 场景3：高频测试（0-2000 Hz）

```bash
# 扩大 Z 轴扫频范围
nsh> param set FAKE_IMU_Z_F0 0
nsh> param set FAKE_IMU_Z_F1 2000
nsh> param set FAKE_IMU_PERIOD 20  # 慢扫，提高精度

# 保存
nsh> param save
```

---

### 3.2 重启使参数生效

```bash
nsh> reboot
```

**重要**：
- ⚠️ 参数修改后**必须重启**才能生效
- 参数在模块初始化时读取一次

---

## 4. 步骤2：启动模块

### 4.1 启动 fake_imu

```bash
# 系统重启后
nsh> fake_imu start

# 验证是否启动成功
nsh> fake_imu status
# 输出: running

# 查看数据是否发布
nsh> listener sensor_gyro_fifo
```

**预期输出**：
```
Rate 400.000, Interval: 2500 us
```

---

### 4.2 启动 gyro_fft（可选，用于频谱分析）

```bash
nsh> gyro_fft start

# 验证
nsh> gyro_fft status
```

**作用**：
- 实时进行 FFT 分析
- 检测当前振动频率
- 记录到日志中（`sensor_gyro_fft` 主题）

---

### 4.3 验证数据流

```bash
# 查看发布的主题
nsh> uorb top

# 应该看到：
# sensor_gyro_fifo    800Hz  (或根据 IMU_GYRO_RATEMAX)
# sensor_gyro         800Hz
# sensor_accel        800Hz
# sensor_gyro_fft     10Hz   (如果启动了 gyro_fft)
```

---

## 5. 步骤3：记录日志

### 5.1 启动 Logger

```bash
# 启动日志记录
nsh> logger on

# 验证 logger 状态
nsh> logger status
```

**输出示例**：
```
logger: logging started (file: /fs/microsd/log/2025-10-30/14_35_22.ulg)
```

---

### 5.2 等待扫频完成

**计算扫频时间**：
- 完整扫频时间 = `FAKE_IMU_PERIOD × 2`
- 例如：`PERIOD = 10s`，需要等待 `20秒`

```bash
# 查看当前参数
nsh> param show FAKE_IMU_PERIOD
# FAKE_IMU_PERIOD = 10.0

# 等待 20 秒让扫频完成
# 可以使用 listener 监控数据：
nsh> listener sensor_gyro_fft
```

---

### 5.3 停止日志记录

```bash
# 停止 logger
nsh> logger off

# 查看日志文件
nsh> ls /fs/microsd/log/2025-10-30/
```

---

### 5.4 停止模块（可选）

```bash
nsh> fake_imu stop
nsh> gyro_fft stop
```

---

## 6. 步骤4：下载日志文件

### 6.1 方法1：通过 QGroundControl

**步骤**：
1. 连接 QGroundControl
2. 点击左侧菜单 **Analyze Tools**
3. 选择 **Log Download**
4. 选择要下载的日志文件
5. 点击 **Download**
6. 保存到本地（如 `~/Downloads/fake_imu_test.ulg`）

---

### 6.2 方法2：通过 MAVLink Shell

```bash
# 列出日志文件
mavlink> ls /fs/microsd/log/2025-10-30/

# 下载日志
mavlink> dumpfile /fs/microsd/log/2025-10-30/14_35_22.ulg
```

**限制**：
- ⚠️ 速度慢（通过 MAVLink 传输）
- ⚠️ 大文件可能超时

---

### 6.3 方法3：直接读取 SD 卡（推荐）

**步骤**：
1. 关闭飞控电源
2. 取出 SD 卡
3. 插入电脑读卡器
4. 复制文件到电脑

**路径**：
```
SD卡/log/2025-10-30/14_35_22.ulg
```

---

## 7. 步骤5：数据分析

### 7.1 使用 FlightPlot 快速查看

**安装 FlightPlot**：
```bash
# Linux
sudo apt install openjdk-11-jre
wget https://github.com/PX4/FlightPlot/releases/download/v3.0.0/FlightPlot.jar

# 运行
java -jar FlightPlot.jar
```

**操作步骤**：
1. **File** → **Open Log File** → 选择 `.ulg` 文件
2. 选择要查看的主题：
   - `sensor_gyro_fifo` - 原始陀螺仪数据
   - `sensor_accel` - 加速度计数据
   - `sensor_gyro_fft` - FFT 分析结果（如果启用）
3. 绘制图表查看扫频效果

---

### 7.2 使用 pyulog 提取数据

#### 安装 pyulog

```bash
pip3 install pyulog
```

---

#### 提取陀螺仪数据

```python
#!/usr/bin/env python3
# extract_fake_imu_data.py

from pyulog import ULog
import numpy as np
import matplotlib.pyplot as plt

# 读取 ULog 文件
log = ULog('fake_imu_test.ulg')

# 提取 sensor_gyro_fifo 数据
gyro_fifo = log.get_dataset('sensor_gyro_fifo')

# 获取时间戳和陀螺仪数据
timestamp = gyro_fifo.data['timestamp'] / 1e6  # 转换为秒
gyro_x = gyro_fifo.data['x[0]']  # 第一个采样点
gyro_y = gyro_fifo.data['y[0]']
gyro_z = gyro_fifo.data['z[0]']

# 绘制时域图
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(timestamp, gyro_x)
plt.ylabel('Gyro X (raw)')
plt.title('Fake IMU Chirp Signal')
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(timestamp, gyro_y)
plt.ylabel('Gyro Y (raw)')
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(timestamp, gyro_z)
plt.ylabel('Gyro Z (raw)')
plt.xlabel('Time (s)')
plt.grid()

plt.tight_layout()
plt.savefig('fake_imu_timeseries.png')
plt.show()

print(f"数据点数: {len(timestamp)}")
print(f"记录时长: {timestamp[-1] - timestamp[0]:.2f} 秒")
print(f"采样率: {len(timestamp) / (timestamp[-1] - timestamp[0]):.2f} Hz")
```

---

### 7.3 FFT 频谱分析

```python
#!/usr/bin/env python3
# fft_analysis.py

from pyulog import ULog
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# 读取日志
log = ULog('fake_imu_test.ulg')
gyro_fifo = log.get_dataset('sensor_gyro_fifo')

# 提取 Z 轴数据（通常包含最宽的扫频范围）
timestamp = gyro_fifo.data['timestamp'] / 1e6
gyro_z = gyro_fifo.data['z[0]']

# 计算采样率
dt = np.mean(np.diff(timestamp))
fs = 1 / dt
print(f"采样率: {fs:.2f} Hz")

# ========== 方法1: 使用 numpy FFT ==========
N = len(gyro_z)
yf = np.fft.fft(gyro_z)
xf = np.fft.fftfreq(N, dt)[:N//2]

# 绘制频谱
plt.figure(figsize=(14, 10))

plt.subplot(3, 1, 1)
plt.plot(timestamp, gyro_z)
plt.ylabel('Gyro Z (raw)')
plt.xlabel('Time (s)')
plt.title('Time Domain Signal')
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))
plt.ylabel('Amplitude')
plt.xlabel('Frequency (Hz)')
plt.title('FFT Spectrum')
plt.grid()
plt.xlim(0, 1500)

# ========== 方法2: 使用 Spectrogram（时频图）==========
plt.subplot(3, 1, 3)
f, t, Sxx = signal.spectrogram(gyro_z, fs, nperseg=1024)
plt.pcolormesh(t, f, 10 * np.log10(Sxx), shading='gouraud', cmap='jet')
plt.ylabel('Frequency (Hz)')
plt.xlabel('Time (s)')
plt.title('Spectrogram (时频图)')
plt.colorbar(label='Power (dB)')
plt.ylim(0, 1500)

plt.tight_layout()
plt.savefig('fake_imu_fft_analysis.png', dpi=300)
plt.show()

# ========== 检测峰值频率 ==========
peaks, _ = signal.find_peaks(2.0/N * np.abs(yf[0:N//2]), height=1000)
peak_freqs = xf[peaks]
print(f"\n检测到的峰值频率:")
for freq in peak_freqs[:10]:  # 显示前10个
    print(f"  {freq:.2f} Hz")
```

**预期输出**：
- 时频图应该显示一条从低频到高频的**斜线**（扫频轨迹）
- 峰值频率应该随时间线性增加

---

### 7.4 对比 FFT 模块的检测结果

如果启用了 `gyro_fft`，可以对比实时 FFT 检测结果：

```python
#!/usr/bin/env python3
# compare_fft.py

from pyulog import ULog
import matplotlib.pyplot as plt

log = ULog('fake_imu_test.ulg')

# 提取 sensor_gyro_fft 数据
try:
    gyro_fft = log.get_dataset('sensor_gyro_fft')

    timestamp = gyro_fft.data['timestamp'] / 1e6
    peak_freq_x = gyro_fft.data['peak_frequencies_x[0]']
    peak_freq_y = gyro_fft.data['peak_frequencies_y[0]']
    peak_freq_z = gyro_fft.data['peak_frequencies_z[0]']

    # 绘制FFT模块检测到的频率
    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(timestamp, peak_freq_x)
    plt.ylabel('X Freq (Hz)')
    plt.title('gyro_fft 模块检测到的频率')
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(timestamp, peak_freq_y)
    plt.ylabel('Y Freq (Hz)')
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(timestamp, peak_freq_z)
    plt.ylabel('Z Freq (Hz)')
    plt.xlabel('Time (s)')
    plt.grid()

    plt.tight_layout()
    plt.savefig('gyro_fft_detection.png')
    plt.show()

    # 验证检测精度
    # 理论频率 vs 检测频率
    print("FFT 检测精度分析:")
    print(f"Z 轴平均检测频率: {np.mean(peak_freq_z):.2f} Hz")
    print(f"Z 轴频率范围: {np.min(peak_freq_z):.2f} - {np.max(peak_freq_z):.2f} Hz")

except:
    print("日志中没有 sensor_gyro_fft 数据")
    print("请确认 gyro_fft 模块已启动")
```

---

## 8. 常见问题与解决方案

### 8.1 Q: 日志文件太大，下载很慢？

**解决方案1：只记录特定主题**

创建自定义日志配置文件：

```bash
# 在 SD 卡上创建配置文件
nsh> echo "logger start -t -b 200 -p sensor_gyro_fifo" > /fs/microsd/etc/logging/logger_topics.txt

# 使用自定义配置
nsh> logger on -t -p sensor_gyro_fifo
```

**解决方案2：降低采样率**

```bash
# 降低 IMU_GYRO_RATEMAX
nsh> param set IMU_GYRO_RATEMAX 200
nsh> param save
nsh> reboot
```

---

### 8.2 Q: 日志中没有 sensor_gyro_fifo 数据？

**排查步骤**：

```bash
# 1. 检查 fake_imu 是否运行
nsh> fake_imu status

# 2. 检查主题是否发布
nsh> listener sensor_gyro_fifo

# 3. 检查 logger 是否包含该主题
nsh> logger status

# 4. 查看 logger 配置
nsh> cat /fs/microsd/etc/logging/logger_topics.txt
```

---

### 8.3 Q: 频谱图中看不到扫频轨迹？

**可能原因**：

1. **扫频范围太窄**
   ```bash
   # 扩大范围
   nsh> param set FAKE_IMU_Z_F1 2000
   ```

2. **周期太短**
   ```bash
   # 延长周期
   nsh> param set FAKE_IMU_PERIOD 20
   ```

3. **数据记录时间不够**
   - 确保记录时长 ≥ `PERIOD × 2`

---

### 8.4 Q: SD 卡空间不足？

```bash
# 查看 SD 卡空间
nsh> df

# 删除旧日志
nsh> rm -rf /fs/microsd/log/2025-10-29

# 或格式化（⚠️ 会丢失所有数据）
nsh> mkfatfs /dev/mmcsd0
```

---

## 9. 高级分析技巧

### 9.1 短时傅里叶变换（STFT）

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from pyulog import ULog

log = ULog('fake_imu_test.ulg')
gyro_fifo = log.get_dataset('sensor_gyro_fifo')

timestamp = gyro_fifo.data['timestamp'] / 1e6
gyro_z = gyro_fifo.data['z[0]']

# 计算采样率
fs = 1 / np.mean(np.diff(timestamp))

# STFT 参数
nperseg = 2048  # 窗口大小
noverlap = 1536  # 重叠

# 计算 STFT
f, t, Zxx = signal.stft(gyro_z, fs, nperseg=nperseg, noverlap=noverlap)

# 绘制时频图
plt.figure(figsize=(14, 8))
plt.pcolormesh(t, f, np.abs(Zxx), shading='gouraud', cmap='jet')
plt.title('STFT Magnitude - Fake IMU Chirp Signal')
plt.ylabel('Frequency (Hz)')
plt.xlabel('Time (s)')
plt.colorbar(label='Magnitude')
plt.ylim(0, 1500)
plt.savefig('stft_analysis.png', dpi=300)
plt.show()

# 验证扫频线性度
# 提取每个时间点的主频率
peak_freqs = []
for i in range(len(t)):
    spectrum = np.abs(Zxx[:, i])
    peak_idx = np.argmax(spectrum)
    peak_freqs.append(f[peak_idx])

# 线性拟合
coeffs = np.polyfit(t, peak_freqs, 1)
fitted_freq = np.polyval(coeffs, t)

plt.figure(figsize=(12, 6))
plt.plot(t, peak_freqs, 'b.', label='检测频率', markersize=2)
plt.plot(t, fitted_freq, 'r-', linewidth=2, label=f'线性拟合: {coeffs[0]:.2f} Hz/s')
plt.xlabel('Time (s)')
plt.ylabel('Frequency (Hz)')
plt.title('扫频线性度分析')
plt.legend()
plt.grid()
plt.savefig('chirp_linearity.png', dpi=300)
plt.show()

print(f"\n扫频速率: {coeffs[0]:.2f} Hz/s")
print(f"线性拟合 R²: {np.corrcoef(peak_freqs, fitted_freq)[0,1]**2:.4f}")
```

---

### 9.2 动态陷波滤波器效果分析

```python
from pyulog import ULog
import numpy as np
import matplotlib.pyplot as plt

log = ULog('fake_imu_test.ulg')

# 原始陀螺仪数据
gyro_raw = log.get_dataset('sensor_gyro_fifo')
timestamp_raw = gyro_raw.data['timestamp'] / 1e6
gyro_z_raw = gyro_raw.data['z[0]']

# 滤波后的数据（vehicle_angular_velocity）
try:
    angular_vel = log.get_dataset('vehicle_angular_velocity')
    timestamp_filt = angular_vel.data['timestamp'] / 1e6
    gyro_z_filt = angular_vel.data['xyz[2]']  # Z 轴角速度

    # 对比原始数据和滤波后数据
    plt.figure(figsize=(14, 10))

    plt.subplot(2, 1, 1)
    plt.plot(timestamp_raw, gyro_z_raw, 'b-', alpha=0.7, label='原始数据')
    plt.ylabel('Gyro Z Raw')
    plt.title('滤波器效果对比')
    plt.legend()
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(timestamp_filt, gyro_z_filt, 'r-', alpha=0.7, label='滤波后')
    plt.ylabel('Angular Velocity Z (rad/s)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.savefig('filter_comparison.png', dpi=300)
    plt.show()

    # 计算滤波效果
    # 假设振动应该被抑制
    raw_std = np.std(gyro_z_raw)
    filt_std = np.std(gyro_z_filt)
    reduction = (1 - filt_std / raw_std) * 100

    print(f"\n滤波效果分析:")
    print(f"原始数据标准差: {raw_std:.2f}")
    print(f"滤波后标准差: {filt_std:.2f}")
    print(f"振动抑制率: {reduction:.2f}%")

except Exception as e:
    print(f"无法提取滤波后数据: {e}")
```

---

### 9.3 频率跟踪精度分析

```python
from pyulog import ULog
import numpy as np
import matplotlib.pyplot as plt

log = ULog('fake_imu_test.ulg')

# 提取参数值
try:
    params = log.initial_parameters
    z_f0 = params['FAKE_IMU_Z_F0']
    z_f1 = params['FAKE_IMU_Z_F1']
    period = params['FAKE_IMU_PERIOD']
except:
    # 使用默认值
    z_f0 = 0.0
    z_f1 = 1000.0
    period = 10.0

# 提取 FFT 检测结果
try:
    gyro_fft = log.get_dataset('sensor_gyro_fft')
    timestamp = gyro_fft.data['timestamp'] / 1e6
    detected_freq = gyro_fft.data['peak_frequencies_z[0]']

    # 计算理论频率
    T = period
    t_relative = timestamp - timestamp[0]
    theoretical_freq = z_f0 + (z_f1 - z_f0) * t_relative / (2 * T)

    # 对比理论值和检测值
    plt.figure(figsize=(12, 6))
    plt.plot(t_relative, theoretical_freq, 'b-', linewidth=2, label='理论频率')
    plt.plot(timestamp - timestamp[0], detected_freq, 'r.', markersize=3, label='FFT 检测频率')
    plt.xlabel('Time (s)')
    plt.ylabel('Frequency (Hz)')
    plt.title('FFT 检测精度分析')
    plt.legend()
    plt.grid()
    plt.savefig('fft_accuracy.png', dpi=300)
    plt.show()

    # 计算误差
    # 需要插值到相同的时间点
    from scipy.interpolate import interp1d
    interp_theoretical = interp1d(t_relative, theoretical_freq,
                                   kind='linear', fill_value='extrapolate')
    theoretical_at_detection = interp_theoretical(timestamp - timestamp[0])

    error = detected_freq - theoretical_at_detection
    abs_error = np.abs(error)

    print(f"\nFFT 检测精度:")
    print(f"平均误差: {np.mean(error):.2f} Hz")
    print(f"平均绝对误差: {np.mean(abs_error):.2f} Hz")
    print(f"最大误差: {np.max(abs_error):.2f} Hz")
    print(f"误差标准差: {np.std(error):.2f} Hz")

except Exception as e:
    print(f"错误: {e}")
    print("请确认 gyro_fft 模块已启动并记录了数据")
```

---

## 10. MATLAB 分析示例

### 10.1 使用 MATLAB 读取 ULog

```matlab
%% 安装 ulog-matlab
% 下载: https://github.com/PX4/ulog_matlab
% 添加到 MATLAB 路径

%% 读取 ULog 文件
log = ulogreader('fake_imu_test.ulg');

%% 提取陀螺仪数据
gyro_fifo = log.Messages('sensor_gyro_fifo');

timestamp = double(gyro_fifo.timestamp) / 1e6;  % 转换为秒
gyro_x = double(gyro_fifo.x_0_);  % 第一个采样点
gyro_y = double(gyro_fifo.y_0_);
gyro_z = double(gyro_fifo.z_0_);

%% 时域绘图
figure('Position', [100, 100, 1200, 800]);

subplot(3, 1, 1);
plot(timestamp, gyro_x);
ylabel('Gyro X (raw)');
title('Fake IMU Chirp Signal');
grid on;

subplot(3, 1, 2);
plot(timestamp, gyro_y);
ylabel('Gyro Y (raw)');
grid on;

subplot(3, 1, 3);
plot(timestamp, gyro_z);
ylabel('Gyro Z (raw)');
xlabel('Time (s)');
grid on;

%% FFT 分析
Fs = 1 / mean(diff(timestamp));  % 采样率
N = length(gyro_z);
f = (0:N-1) * (Fs/N);  % 频率轴

Y = fft(gyro_z);
P = abs(Y/N);

figure;
plot(f(1:N/2), P(1:N/2));
xlabel('Frequency (Hz)');
ylabel('Amplitude');
title('FFT Spectrum');
xlim([0 1500]);
grid on;

%% Spectrogram（时频图）
figure;
spectrogram(gyro_z, hamming(2048), 1536, 2048, Fs, 'yaxis');
ylim([0 1.5]);  % 0-1500 Hz
title('Fake IMU Spectrogram');
colorbar;

%% 保存结果
saveas(gcf, 'fake_imu_spectrogram.png');

fprintf('采样率: %.2f Hz\n', Fs);
fprintf('数据点数: %d\n', N);
fprintf('记录时长: %.2f 秒\n', timestamp(end) - timestamp(1));
```

---

## 11. 完整测试脚本

### 11.1 自动化测试脚本（bash）

```bash
#!/bin/bash
# auto_test_fake_imu.sh

echo "========== Fake IMU 自动测试脚本 =========="

# 配置参数
echo "配置扫频参数..."
mavlink shell -c "param set FAKE_IMU_Z_F0 0"
mavlink shell -c "param set FAKE_IMU_Z_F1 1500"
mavlink shell -c "param set FAKE_IMU_PERIOD 15"
mavlink shell -c "param save"

echo "重启飞控..."
mavlink shell -c "reboot"
sleep 10

echo "启动模块..."
mavlink shell -c "fake_imu start"
mavlink shell -c "gyro_fft start"

echo "启动日志记录..."
mavlink shell -c "logger on"

echo "等待扫频完成（30秒）..."
sleep 30

echo "停止日志记录..."
mavlink shell -c "logger off"

echo "停止模块..."
mavlink shell -c "fake_imu stop"
mavlink shell -c "gyro_fft stop"

echo "下载日志..."
# 使用 mavlink ftp 下载（需要配置）
# 或提示用户手动下载

echo "完成！请从 SD 卡下载日志文件进行分析。"
```

---

## 12. 性能分析

### 12.1 CPU 占用分析

```bash
# 启动 fake_imu
nsh> fake_imu start

# 查看 CPU 占用
nsh> top

# 查看性能计数器
nsh> perf
```

**预期结果**：
- fake_imu CPU 占用：约 1-2%
- gyro_fft CPU 占用：约 3-5%

---

### 12.2 内存占用分析

```bash
# 查看内存使用
nsh> free

# 启动前
nsh> free
# 记录可用内存

# 启动后
nsh> fake_imu start
nsh> free
# 对比差异
```

---

## 13. 实际案例

### 13.1 案例1：测试 FFT 窗口大小影响

**目标**：验证不同 FFT 窗口大小对频率分辨率的影响

```bash
# 配置慢扫频
nsh> param set FAKE_IMU_Z_F0 0
nsh> param set FAKE_IMU_Z_F1 500
nsh> param set FAKE_IMU_PERIOD 30  # 慢扫

# 测试不同 FFT 窗口
nsh> param set IMU_GYRO_FFT_LEN 256
nsh> param save
nsh> reboot

# 记录日志1
nsh> fake_imu start
nsh> gyro_fft start
nsh> logger on
# ... 等待 60 秒 ...
nsh> logger off

# 修改 FFT 窗口
nsh> param set IMU_GYRO_FFT_LEN 512
nsh> param save
nsh> reboot

# 记录日志2
# ... 重复测试 ...

# 对比两次日志的频率检测精度
```

---

### 13.2 案例2：动态陷波滤波器压力测试

**目标**：测试快速扫频时陷波滤波器的跟踪能力

```bash
# 配置快速扫频
nsh> param set FAKE_IMU_Z_F0 100
nsh> param set FAKE_IMU_Z_F1 1000
nsh> param set FAKE_IMU_PERIOD 3  # 3秒周期，6秒完成扫频

# 启用动态陷波
nsh> param set IMU_GYRO_DNF_EN 2  # 启用 FFT 陷波
nsh> param save
nsh> reboot

# 测试
nsh> fake_imu start
nsh> gyro_fft start
nsh> logger on
# ... 等待 10 秒 ...
nsh> logger off

# 分析滤波器是否能跟上快速变化
```

---

## 14. 总结

### 14.1 完整流程检查清单

- [ ] 配置扫频参数（`FAKE_IMU_*`）
- [ ] 保存参数（`param save`）
- [ ] 重启飞控（`reboot`）
- [ ] 启动 fake_imu（`fake_imu start`）
- [ ] 可选：启动 gyro_fft（`gyro_fft start`）
- [ ] 启动日志记录（`logger on`）
- [ ] 等待扫频完成（`PERIOD × 2` 秒）
- [ ] 停止日志记录（`logger off`）
- [ ] 下载日志文件（QGC / SD 卡）
- [ ] 使用 Python/MATLAB 分析数据
- [ ] 绘制时频图验证扫频效果

---

### 14.2 常用参数配置

| 测试场景 | X_F0-F1 | Y_F0-F1 | Z_F0-F1 | PERIOD |
|---------|---------|---------|---------|--------|
| **默认（全频谱）** | 0-10 | 0-100 | 0-1000 | 10 |
| **电机测试** | 0-0 | 0-0 | 100-800 | 15 |
| **高频测试** | 0-0 | 0-0 | 0-2000 | 20 |
| **精细测试** | 0-0 | 0-0 | 200-400 | 30 |
| **快速扫描** | 0-10 | 0-100 | 0-1000 | 5 |

---

### 14.3 必备工具

| 工具 | 用途 | 下载链接 |
|------|------|---------|
| **pyulog** | Python ULog 解析 | `pip install pyulog` |
| **FlightPlot** | 快速可视化 | [GitHub](https://github.com/PX4/FlightPlot) |
| **QGroundControl** | 参数配置、日志下载 | [qgroundcontrol.com](https://qgroundcontrol.com) |
| **PlotJuggler** | 高级数据可视化 | [GitHub](https://github.com/facontidavide/PlotJuggler) |

---

### 14.4 相关文档

| 文档 | 说明 |
|------|------|
| `01-fake_imu传感器模拟器代码详解.md` | 代码实现原理 |
| `05-fake_imu参数配置使用指南.md` | 参数配置说明 |
| `06-动态陷波滤波器配置与实现原理.md` | 滤波器原理 |
| `07-FFT动态陷波带宽计算算法详解.md` | FFT 算法详解 |

---

**文档版本**：v1.0
**创建日期**：2025-10-30
**适用 PX4 版本**：v1.14+
**作者**：基于 fake_imu 实际测试流程整理

