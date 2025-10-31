# 使用 gyro_fft 实时分析 fake_imu 数据

## 一、sensor_gyro_fft 是什么？

`sensor_gyro_fft` 是 PX4 的实时频率分析模块，用于检测陀螺仪数据中的周期性振动。

### 1.1 主要功能

- **实时 FFT 分析**：对陀螺仪数据进行快速傅里叶变换
- **峰值检测**：识别振动的主要频率（如螺旋桨转速）
- **SNR 计算**：信噪比评估
- **多轴分析**：X/Y/Z 三轴独立分析

### 1.2 典型应用

- 检测螺旋桨不平衡（20-200 Hz）
- 识别机械共振频率
- 动态陷波滤波器（ESC RPM 滤波）
- 振动分析和诊断

### 1.3 输出数据格式

```
sensor_gyro_fft
  ├── device_id: 使用的陀螺仪设备 ID
  ├── sensor_sample_rate_hz: 采样率
  ├── resolution_hz: 频率分辨率
  ├── peak_frequencies_x[3]: X轴前3个峰值频率
  ├── peak_frequencies_y[3]: Y轴前3个峰值频率
  ├── peak_frequencies_z[3]: Z轴前3个峰值频率
  ├── peak_snr_x[3]: X轴峰值信噪比
  ├── peak_snr_y[3]: Y轴峰值信噪比
  └── peak_snr_z[3]: Z轴峰值信噪比
```

---

## 二、问题：为什么 gyro_fft 没有分析 fake_imu？

### 2.1 当前状态

从你的 `listener sensor_gyro_fft` 输出：

```
device_id: 6684690 (Type: 0x66, SPI:2)  ← 真实传感器（BMI088）
```

而 fake_imu 的 device_id 是：
```
device_id: 1310988 (Type: 0x14, SIMULATION:1)
```

**结论：gyro_fft 正在分析真实传感器，而不是 fake_imu！**

### 2.2 原因分析

`gyro_fft` 通过 `sensor_selection` 主题来决定分析哪个传感器：

```cpp
// GyroFFT.cpp line 174
if (sensor_selection.gyro_device_id != 0) {
    _selected_sensor_device_id = sensor_selection.gyro_device_id;
    // 订阅这个 device_id 的传感器
}
```

**问题：**
- `sensor_selection` 由 `sensors` 模块发布
- `sensors` 模块会自动选择"最佳"传感器
- 通常会优先选择真实硬件传感器，而不是 fake_imu

---

## 三、解决方案

### 方案 A：禁用真实传感器（推荐用于测试）

让 fake_imu 成为**唯一**的陀螺仪，系统会自动选择它。

#### 步骤：

```bash
# 1. 停止所有 IMU 相关模块
nsh> mc_att_control stop
nsh> mc_rate_control stop
nsh> sensors stop

# 2. 重新配置：只启动 fake_imu
nsh> fake_imu start

# 3. 启动 sensors 模块（会自动选择 fake_imu）
nsh> sensors start

# 4. 启动 gyro_fft
nsh> gyro_fft start

# 5. 验证
nsh> listener sensor_selection
# 应该看到 gyro_device_id: 1310988

nsh> listener sensor_gyro_fft
# 应该看到 device_id: 1310988
# 并且能看到实时的频率峰值
```

**问题：** 这会停止飞行控制，只能用于地面测试。

### 方案 B：修改传感器优先级（需要重新编译）

修改 `src/modules/sensors/voted_sensors_update.cpp`，强制选择 fake_imu。

**不推荐：** 需要改代码，影响其他功能。

### 方案 C：创建独立的 fake_imu_fft 模块（最灵活）

复制 `gyro_fft`，修改为专门分析 fake_imu。

#### 实现步骤：

1. 复制并修改订阅逻辑：
```cpp
// 不使用 sensor_selection
// 直接查找并订阅 device_id == 1310988 的传感器

for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
    uORB::SubscriptionData<sensor_gyro_fifo_s> sub{ORB_ID(sensor_gyro_fifo), i};

    if (sub.get().device_id == 1310988) {  // fake_imu
        _sensor_gyro_fifo_sub.ChangeInstance(i);
        _sensor_gyro_fifo_sub.registerCallback();
        break;
    }
}
```

2. 修改发布主题名（避免冲突）：
```cpp
uORB::Publication<sensor_gyro_fft_s> _pub{ORB_ID(fake_imu_fft)};
```

### 方案 D：临时修改 sensor_selection（最简单）

使用 `listener` 手动发布 sensor_selection。

**限制：** 每次重启后失效，且可能影响飞控。

---

## 四、推荐测试流程（方案 A）

### 4.1 完整测试脚本

```bash
# ===== 准备阶段 =====
# 停止飞控相关模块（地面测试）
nsh> mc_att_control stop
nsh> mc_rate_control stop
nsh> ekf2 stop

# 停止 sensors 模块
nsh> sensors stop

# ===== 启动 fake_imu =====
nsh> fake_imu start
INFO  [fake_imu] Rate 800.000, Interval: 1250 us

# ===== 重新启动 sensors =====
# sensors 会重新扫描传感器并选择
nsh> sensors start

# 等待 2 秒让系统稳定
nsh> sleep 2

# ===== 验证传感器选择 =====
nsh> listener sensor_selection
# 应该看到：
#   gyro_device_id: 1310988  ← 如果这是唯一的陀螺仪

# ===== 启动 gyro_fft =====
nsh> gyro_fft start

# ===== 验证 FFT 输出 =====
nsh> listener sensor_gyro_fft
# 现在应该看到 device_id: 1310988
# 并且能看到实时的频率峰值

# ===== 启动数据采集 =====
nsh> matlab_csv_serial start /dev/ttyS3

# ===== 观察结果 =====
# 你会同时看到：
# 1. 串口输出原始数据（CSV）
# 2. gyro_fft 实时显示频率峰值
```

### 4.2 预期输出

```bash
nsh> listener sensor_gyro_fft

TOPIC: sensor_gyro_fft
 sensor_gyro_fft
    timestamp: 1234567890
    device_id: 1310988  ← fake_imu ✓
    sensor_sample_rate_hz: 800.00
    resolution_hz: 3.12  (800/256)

    # X轴：0-10 Hz 扫频
    peak_frequencies_x: [5.2, nan, nan]  ← 会实时变化
    peak_snr_x: [15.3, 0.0, 0.0]

    # Y轴：0-100 Hz 扫频
    peak_frequencies_y: [47.8, nan, nan]  ← 会实时变化
    peak_snr_y: [22.1, 0.0, 0.0]

    # Z轴：0-1000 Hz 扫频
    peak_frequencies_z: [234.5, nan, nan]  ← 会实时变化
    peak_snr_z: [8.7, 0.0, 0.0]
```

**注意：峰值频率会随时间增加**（Chirp 扫频）

---

## 五、理解 FFT 输出

### 5.1 参数配置

```bash
nsh> param show IMU_GYRO_FFT*

IMU_GYRO_FFT_LEN   256    # FFT 长度
IMU_GYRO_FFT_MIN   2.0    # 最小检测频率 (Hz)
IMU_GYRO_FFT_MAX   1000.0 # 最大检测频率 (Hz)
IMU_GYRO_FFT_SNR   10.0   # 信噪比阈值
```

### 5.2 频率分辨率

```
分辨率 = 采样率 / FFT_LEN
       = 800 Hz / 256
       = 3.125 Hz

意思：能区分间隔 > 3.125 Hz 的频率
```

### 5.3 Chirp 信号的 FFT

**特点：**

fake_imu 的 Chirp 信号在时域是**瞬时频率变化**的：
```
t=0s:   频率 = 0 Hz
t=5s:   频率 = 500 Hz
t=10s:  频率 = 1000 Hz
```

**FFT 的窗口效应：**

FFT 分析的是一个**固定时间窗口**（256 个样本 ≈ 0.32 秒）：

```
时间窗口: [t, t+0.32s]

在这个窗口内，频率从 f(t) 变化到 f(t+0.32)

FFT 看到的是一个"扫过多个频率"的信号
→ 主峰值 = 窗口中间的频率
→ 峰值会比纯正弦波宽（因为频率在变化）
```

**举例：**

```
假设 t=5s，Z轴频率约 500 Hz

FFT 窗口: 5.0s ~ 5.32s
频率变化: 500 Hz → 532 Hz

FFT 结果:
  peak_frequencies_z: [516, nan, nan]  ← 窗口中心频率
  峰值会"拖尾"（能量分散到相邻频率）
```

### 5.4 观察 Chirp 的正确方法

**在 Spectrogram（时频图）中观察：**
- MATLAB 的 `spectrogram()` 最适合
- 会看到清晰的频率扫描线

**在 FFT 中观察：**
- 峰值频率会随时间变化
- 每隔几秒看一次，频率应该递增
- 直到 10 秒后重新从 0 开始

---

## 六、验证脚本

运行这个命令持续观察：

```bash
# 每 2 秒打印一次 gyro_fft
while true; do
    listener sensor_gyro_fft 1
    sleep 2
done
```

**你应该看到：**
```
# t=0s
peak_frequencies_z: [15.6, nan, nan]

# t=2s
peak_frequencies_z: [234.4, nan, nan]  ← 增加了

# t=4s
peak_frequencies_z: [453.1, nan, nan]  ← 继续增加

# t=6s
peak_frequencies_z: [671.9, nan, nan]

# t=8s
peak_frequencies_z: [890.6, nan, nan]

# t=10s (新周期开始)
peak_frequencies_z: [12.5, nan, nan]  ← 回到起始
```

---

## 七、为什么当前 FFT 分析的是真实传感器？

### 7.1 传感器选择机制

```
系统启动
  ↓
sensors 模块扫描所有 IMU
  ↓
voted_sensors_update 评估并投票
  ↓
选择"最佳"传感器（通常是优先级最高的硬件）
  ↓
发布 sensor_selection
  ↓
gyro_fft 订阅 sensor_selection
  ↓
分析选中的传感器
```

**当前状态：**
```
传感器列表:
  Instance 0: BMI088  (device_id 6684690) ← 被选中
  Instance 1: BMI270  (device_id 7798802)
  Instance 2: fake_imu (device_id 1310988) ← 未被选中
```

### 7.2 查看当前选择

```bash
nsh> listener sensor_selection

sensor_selection
  timestamp: 1234567890
  accel_device_id: 6684690  ← 真实传感器
  gyro_device_id: 6684690   ← 真实传感器
```

---

## 八、如何切换到 fake_imu（实操指南）

### 方法 1：停用真实 IMU 驱动（最简单）

```bash
# 1. 查看当前运行的 IMU 驱动
nsh> top
# 或
nsh> work_queue status

# 2. 停止真实 IMU 驱动（根据你的硬件）
nsh> bmi088 stop     # 如果有 BMI088
nsh> bmi270 stop     # 如果有 BMI270

# 3. 确认只有 fake_imu 在运行
nsh> listener sensor_gyro
# 应该只看到 1 个实例（fake_imu）

# 4. 重启 sensors 模块
nsh> sensors stop
nsh> sensors start

# 5. 验证选择
nsh> listener sensor_selection
# gyro_device_id 应该是 1310988

# 6. 启动 gyro_fft
nsh> gyro_fft start

# 7. 验证 FFT
nsh> listener sensor_gyro_fft
# device_id 应该是 1310988
```

### 方法 2：修改板级配置（永久方案）

编辑 `boards/micoair/h743/default.px4board`：

```bash
# 注释掉真实 IMU 驱动
# CONFIG_DRIVERS_IMU_BOSCH_BMI088=y
# CONFIG_DRIVERS_IMU_BOSCH_BMI270=y

# 保留 fake_imu
CONFIG_EXAMPLES_FAKE_IMU=y
```

然后重新编译：
```bash
make micoair_h743_default
```

### 方法 3：使用参数覆盖（高级）

查找并设置传感器优先级参数（如果存在）：

```bash
nsh> param show SENS_*
# 查找与传感器选择相关的参数

nsh> param show CAL_GYRO*
# 查看陀螺仪校准参数
```

---

## 九、观察 fake_imu 的 FFT 效果

### 9.1 预期结果

由于 fake_imu 是 Chirp 信号，FFT 会显示：

**Z 轴（0-1000 Hz，10秒周期）：**
```
t=0s:   peak_frequencies_z: [0, nan, nan]
t=1s:   peak_frequencies_z: [100, nan, nan]
t=2s:   peak_frequencies_z: [200, nan, nan]
t=5s:   peak_frequencies_z: [500, nan, nan]
t=10s:  peak_frequencies_z: [0, nan, nan]  (新周期)
```

**Y 轴（0-100 Hz）：**
```
t=0s:   peak_frequencies_y: [0, nan, nan]
t=5s:   peak_frequencies_y: [50, nan, nan]
t=10s:  peak_frequencies_y: [0, nan, nan]
```

**X 轴（0-10 Hz）：**
```
由于频率很低，且分辨率是 3.125 Hz，
可能只能看到 [0, 3.125, 6.25, 9.375] 这几个离散值
```

### 9.2 MATLAB 实时绘图

创建脚本监控 FFT 峰值变化：

```matlab
% monitor_fft.m
function monitor_fft(duration_sec)
    if nargin < 1
        duration_sec = 30;
    end

    % 这需要 PX4 日志或 MAVLink 实时数据
    % 或者你可以：
    % 1. 记录 gyro_fft 输出到日志
    % 2. 使用 listener sensor_gyro_fft > fft_log.txt
    % 3. 解析并绘图

    figure;
    subplot(3,1,1);
    title('X 轴峰值频率 vs 时间');
    xlabel('时间 (s)');
    ylabel('频率 (Hz)');

    subplot(3,1,2);
    title('Y 轴峰值频率 vs 时间');
    xlabel('时间 (s)');
    ylabel('频率 (Hz)');

    subplot(3,1,3);
    title('Z 轴峰值频率 vs 时间');
    xlabel('时间 (s)');
    ylabel('频率 (Hz)');
end
```

---

## 十、gyro_fft 参数调优

### 10.1 FFT 长度

```bash
nsh> param set IMU_GYRO_FFT_LEN 512  # 增加分辨率

新分辨率 = 800 / 512 = 1.56 Hz  (更精细)
但更新速率降低（需要更多样本）
```

| FFT_LEN | 分辨率 | 窗口时长 | 更新率 |
|---------|--------|----------|--------|
| 128 | 6.25 Hz | 0.16 s | ~6 Hz |
| 256 | 3.12 Hz | 0.32 s | ~3 Hz |
| 512 | 1.56 Hz | 0.64 s | ~1.5 Hz |
| 1024 | 0.78 Hz | 1.28 s | ~0.8 Hz |

**推荐：**
- 快速振动（>100 Hz）：256
- 中等频率（20-100 Hz）：512
- 低频振动（<20 Hz）：1024

### 10.2 频率范围

```bash
# 只关注低频
nsh> param set IMU_GYRO_FFT_MIN 2.0
nsh> param set IMU_GYRO_FFT_MAX 200.0

# 关注高频
nsh> param set IMU_GYRO_FFT_MIN 100.0
nsh> param set IMU_GYRO_FFT_MAX 1000.0
```

### 10.3 SNR 阈值

```bash
# 降低阈值，检测更弱的峰值
nsh> param set IMU_GYRO_FFT_SNR 5.0

# 提高阈值，只检测强峰值
nsh> param set IMU_GYRO_FFT_SNR 15.0
```

---

## 十一、实际测试结果预测

对于 fake_imu 的 Chirp 信号：

### Z 轴（0-1000 Hz）

**优势：**
- ✅ 频率覆盖广
- ✅ SNR 高（纯正弦波）
- ✅ 峰值明显

**问题：**
- ⚠️ 采样率 800 Hz → Nyquist 400 Hz
- ⚠️ 超过 400 Hz 的部分会混叠
- ⚠️ FFT 可能看到混叠频率

**预期：**
```
t=0-4s:   峰值正常增长 0-400 Hz
t=4-10s:  峰值混叠，可能看到错误频率
```

### Y 轴（0-100 Hz）

**最佳观察轴：**
- ✅ 完全在 Nyquist 频率内
- ✅ 峰值清晰准确
- ✅ 适合验证 FFT 功能

### X 轴（0-10 Hz）

**低频挑战：**
- ⚠️ 接近 FFT_MIN (2 Hz)
- ⚠️ 分辨率 3.125 Hz，可能"跳跃"
- ⚠️ 需要更长的 FFT 窗口

---

## 十二、总结

### 关键点

1. **sensor_gyro_fft** 是实时 FFT 分析模块
2. 它通过 **sensor_selection** 选择要分析的传感器
3. 默认会选择真实硬件传感器（device_id 6684690）
4. 要分析 fake_imu，需要让它成为**被选中的传感器**

### 快速测试方法

```bash
# 最简单：停用真实传感器
bmi088 stop  # 或你的 IMU 驱动
sensors stop
fake_imu start
sensors start
gyro_fft start

# 验证
listener sensor_gyro_fft
```

### fake_imu 与 FFT 的完美搭配

- **Y 轴**：最适合 FFT（0-100 Hz，无混叠）
- **Z 轴**：展示混叠效应（>400 Hz 部分）
- **X 轴**：低频检测挑战

这是一个**教学级**的 FFT 演示场景！🎓

---

## 附录：手动查看 FFT 数据

如果不想配置 gyro_fft，你也可以用 MATLAB 事后分析：

```matlab
% 你已经有的数据
plot_fake_imu_data('imu_data.csv');

% 子图3、4 就是 FFT 频谱图
% 子图5、6 是时频图（Spectrogram）

% 这和 gyro_fft 的实时分析原理相同，
% 只是一个是实时的，一个是事后的
```

**建议：** 先用 MATLAB 离线分析，理解 Chirp 信号的特性，再去配置实时 FFT。
