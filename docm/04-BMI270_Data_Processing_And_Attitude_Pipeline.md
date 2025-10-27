# BMI270 数据处理与姿态解算管道详解

## 概述

本文档详细追踪IMU数据从BMI270驱动发布后，经过缩放、旋转、滤波等处理，最终用于姿态解算和控制的完整流程，包括精确的代码文件位置和行号。

---

## 一、数据流概览

```
┌─────────────────────────────────────────────────────────────┐
│ 第1层: BMI270驱动 (硬件层)                                   │
│ 文件: src/drivers/imu/bosch/bmi270/                         │
│ 输出: sensor_gyro_fifo_s, sensor_gyro_s                     │
└─────────────────────────────────────────────────────────────┘
                        ↓ uORB
┌─────────────────────────────────────────────────────────────┐
│ 第2层: VehicleAngularVelocity (传感器处理层)                │
│ 文件: src/modules/sensors/vehicle_angular_velocity/         │
│ 操作: 订阅 → 缩放 → 滤波 → 校准 → 发布                     │
│ 输出: vehicle_angular_velocity_s                            │
└─────────────────────────────────────────────────────────────┘
                        ↓ uORB
┌─────────────────────────────────────────────────────────────┐
│ 第3层: VehicleIMU (数据集成层)                              │
│ 文件: src/modules/sensors/vehicle_imu/                      │
│ 操作: 加速度+陀螺仪集成                                     │
│ 输出: vehicle_imu_s                                         │
└─────────────────────────────────────────────────────────────┘
                        ↓ uORB
┌─────────────────────────────────────────────────────────────┐
│ 第4层: EKF2 (状态估计层)                                    │
│ 文件: src/modules/ekf2/                                     │
│ 操作: 卡尔曼滤波 → 姿态估计 → 位置估计                     │
│ 输出: vehicle_attitude_s, vehicle_local_position_s          │
└─────────────────────────────────────────────────────────────┘
                        ↓ uORB
┌─────────────────────────────────────────────────────────────┐
│ 第5层: 控制器 (飞控层)                                      │
│ 文件: src/modules/mc_rate_control/, mc_att_control/        │
│ 操作: 姿态控制 → 角速度控制 → 电机输出                     │
└─────────────────────────────────────────────────────────────┘
```

---

## 二、数据缩放与旋转（第1→2层）

### 2.1 BMI270驱动的初步处理

**文件**: `src/lib/drivers/gyroscope/PX4Gyroscope.cpp`

#### 坐标系旋转（在驱动中）

```cpp
// 行号: 137-149
void PX4Gyroscope::updateFIFO(sensor_gyro_fifo_s &sample)
{
    // 行号: 140-144
    // 旋转所有原始样本（从传感器坐标系到飞控板坐标系）
    const uint8_t N = sample.samples;
    for (int n = 0; n < N; n++) {
        rotate_3i(_rotation, sample.x[n], sample.y[n], sample.z[n]);
    }

    // 行号: 146-148
    // 设置元数据
    sample.device_id = _device_id;
    sample.scale = _scale;  // 缩放因子 (rad/s per LSB)
    sample.timestamp = hrt_absolute_time();

    // 行号: 149
    // 发布FIFO数据（原始int16样本）
    _sensor_fifo_pub.publish(sample);

    // ... 同时发布积分后的sensor_gyro_s
}
```

**坐标旋转矩阵**:
```cpp
// 旋转枚举: lib/conversion/rotation.h
enum Rotation {
    ROTATION_NONE = 0,
    ROTATION_YAW_45,
    ROTATION_YAW_90,
    ROTATION_YAW_135,
    ROTATION_YAW_180,
    ROTATION_YAW_225,
    ROTATION_YAW_270,
    ROTATION_YAW_315,
    ROTATION_ROLL_180,
    // ... 共30多种旋转
};

// 示例: ROTATION_YAW_90
// 将传感器坐标系绕Z轴旋转90度到飞控板坐标系
```

**数据格式（发布时）**:
```cpp
sensor_gyro_fifo_s {
    timestamp_sample: 123456789,     // 采样时间戳
    device_id: 0x12345678,           // 设备ID
    dt: 625,                         // 625μs 采样间隔
    scale: 0.000532632,              // 缩放因子 (rad/s per LSB)
    samples: 2,                      // 样本数
    x[2]: [125, 130],                // 原始int16数据（已旋转）
    y[2]: [-50, -48],
    z[2]: [10, 12]
}
```

### 2.2 VehicleAngularVelocity的数据订阅

**文件**: `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`

#### 订阅配置

```cpp
// 行号: 112-114 (头文件 VehicleAngularVelocity.hpp)
uORB::SubscriptionCallbackWorkItem _sensor_selection_sub{this, ORB_ID(sensor_selection)};
uORB::SubscriptionCallbackWorkItem _sensor_sub{this, ORB_ID(sensor_gyro)};
uORB::SubscriptionCallbackWorkItem _sensor_gyro_fifo_sub{this, ORB_ID(sensor_gyro_fifo)};
```

#### 数据更新入口

```cpp
// 行号: 828-846
while ((sensor_sub_updates < sensor_gyro_fifo_s::ORB_QUEUE_LENGTH)
       && _sensor_gyro_fifo_sub.update(&sensor_fifo_data)) {

    sensor_sub_updates++;

    // 行号: 831
    const float inverse_dt_s = 1e6f / sensor_fifo_data.dt;  // 采样率的倒数
    const int N = sensor_fifo_data.samples;  // 样本数量

    if ((sensor_fifo_data.dt > 0) && (N > 0) && (N <= FIFO_SIZE_MAX)) {
        Vector3f angular_velocity_uncalibrated;
        Vector3f angular_acceleration_uncalibrated;

        // 行号: 839
        int16_t *raw_data_array[] {
            sensor_fifo_data.x,
            sensor_fifo_data.y,
            sensor_fifo_data.z
        };

        // 行号: 841-847
        // 对每个轴(X/Y/Z)进行处理
        for (int axis = 0; axis < 3; axis++) {
            // 行号: 843
            // 拷贝原始int16数据到float数组
            float data[FIFO_SIZE_MAX];

            // 行号: 845-846
            // *** 关键步骤: 缩放 ***
            for (int n = 0; n < N; n++) {
                data[n] = sensor_fifo_data.scale * raw_data_array[axis][n];
            }
            // 结果: int16原始值 → float物理单位(rad/s)

            // 行号: 850
            // *** 关键步骤: 滤波 ***
            angular_velocity_uncalibrated(axis) = FilterAngularVelocity(axis, data, N);

            // 行号: 851
            // *** 关键步骤: 微分+滤波 ***
            angular_acceleration_uncalibrated(axis) = FilterAngularAcceleration(axis, inverse_dt_s, data, N);
        }

        // 行号: 856-858
        // *** 关键步骤: 校准并发布 ***
        if (!_sensor_gyro_fifo_sub.updated()) {
            if (CalibrateAndPublish(sensor_fifo_data.timestamp_sample,
                                    angular_velocity_uncalibrated,
                                    angular_acceleration_uncalibrated)) {
                perf_end(_cycle_perf);
                return;
            }
        }
    }
}
```

**数据变换总结**:
```
原始数据: int16 [-32768, 32767]
    ↓ × scale (例如: 0.000532632 rad/s per LSB)
物理单位: float (rad/s)
    ↓ 滤波处理
滤波后: float (rad/s, 噪声降低)
    ↓ 校准 (减偏置、应用旋转矩阵)
校准后: float (rad/s, 机体坐标系)
```

---

## 三、滤波器详解

### 3.1 滤波器处理顺序

**文件**: `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`

```cpp
// 行号: 725-768
float VehicleAngularVelocity::FilterAngularVelocity(int axis, float data[], int N)
{
#if !defined(CONSTRAINED_FLASH)
    // *** 滤波器1: ESC RPM动态陷波滤波器 ***
    // 行号: 730-740
    if (_dynamic_notch_filter_esc_rpm) {
        for (int esc = 0; esc < MAX_NUM_ESCS; esc++) {
            if (_esc_available[esc]) {
                for (int harmonic = 0; harmonic < _esc_rpm_harmonics; harmonic++) {
                    if (_dynamic_notch_filter_esc_rpm[harmonic][axis][esc].getNotchFreq() > 0.f) {
                        _dynamic_notch_filter_esc_rpm[harmonic][axis][esc].applyArray(data, N);
                    }
                }
            }
        }
    }

    // *** 滤波器2: FFT动态陷波滤波器 ***
    // 行号: 742-749
    if (_dynamic_notch_fft_available) {
        for (int peak = MAX_NUM_FFT_PEAKS - 1; peak >= 0; peak--) {
            if (_dynamic_notch_filter_fft[axis][peak].getNotchFreq() > 0.f) {
                _dynamic_notch_filter_fft[axis][peak].applyArray(data, N);
            }
        }
    }
#endif // !CONSTRAINED_FLASH

    // *** 滤波器3: 静态陷波滤波器0 (IMU_GYRO_NF0_FRQ) ***
    // 行号: 753-756
    if (_notch_filter0_velocity[axis].getNotchFreq() > 0.f) {
        _notch_filter0_velocity[axis].applyArray(data, N);
    }

    // *** 滤波器4: 静态陷波滤波器1 (IMU_GYRO_NF1_FRQ) ***
    // 行号: 758-761
    if (_notch_filter1_velocity[axis].getNotchFreq() > 0.f) {
        _notch_filter1_velocity[axis].applyArray(data, N);
    }

    // *** 滤波器5: 低通滤波器 (IMU_GYRO_CUTOFF) ***
    // 行号: 763-764
    _lp_filter_velocity[axis].applyArray(data, N);

    // 行号: 766-767
    // 返回最后一个滤波后的样本
    return data[N - 1];
}
```

### 3.2 滤波器类型详解

#### 滤波器1: ESC RPM动态陷波滤波器

**参数**:
- `IMU_GYRO_DNF_EN` (bit 0): 启用ESC RPM动态陷波滤波（默认: 0 = 禁用）
- `IMU_GYRO_DNF_BW`: 带宽 (默认: 15Hz)
- `IMU_GYRO_DNF_HMC`: 谐波数量 (默认: 3)
- `IMU_GYRO_DNF_MIN`: 最小频率 (默认: 25Hz)

**工作原理**:
```
订阅ESC状态 → 读取电机RPM → 计算基频和谐波 → 动态设置陷波频率

示例:
ESC1 RPM: 6000 RPM = 100 Hz
谐波: [100Hz, 200Hz, 300Hz]
为每个谐波创建陷波滤波器，带宽15Hz
```

**代码位置**:
- 更新: `VehicleAngularVelocity.cpp:536-712` (UpdateDynamicNotchEscRpm)
- 应用: `VehicleAngularVelocity.cpp:730-740`

**用途**: 消除电机振动噪声

#### 滤波器2: FFT动态陷波滤波器

**参数**:
- `IMU_GYRO_DNF_EN` (bit 1): 启用FFT动态陷波滤波（默认: 0 = 禁用）
- `IMU_GYRO_FFT_EN`: 启用板载FFT (需要额外配置)

**工作原理**:
```
实时FFT分析 → 检测振动峰值频率 → 动态设置陷波滤波器

示例:
FFT检测到峰值: [85Hz, 170Hz, 255Hz]
自动创建3个陷波滤波器
```

**代码位置**:
- 更新: `VehicleAngularVelocity.cpp:714-820` (UpdateDynamicNotchFFT)
- 应用: `VehicleAngularVelocity.cpp:742-749`

**用途**: 自动消除振动噪声（比ESC RPM更智能）

#### 滤波器3&4: 静态陷波滤波器

**参数**:
```c
// 文件: src/modules/sensors/vehicle_angular_velocity/imu_gyro_parameters.c

// 陷波滤波器0
PARAM_DEFINE_FLOAT(IMU_GYRO_NF0_FRQ, 0.0f);  // 中心频率 (默认0=禁用)
PARAM_DEFINE_FLOAT(IMU_GYRO_NF0_BW, 20.0f);  // 带宽 (默认20Hz)

// 陷波滤波器1
PARAM_DEFINE_FLOAT(IMU_GYRO_NF1_FRQ, 0.0f);  // 中心频率 (默认0=禁用)
PARAM_DEFINE_FLOAT(IMU_GYRO_NF1_BW, 20.0f);  // 带宽 (默认20Hz)
```

**工作原理**:
```
用户手动配置固定频率的陷波滤波器

示例配置:
IMU_GYRO_NF0_FRQ = 120  # 消除120Hz振动
IMU_GYRO_NF0_BW = 20    # 阻带宽度20Hz (110-130Hz)
```

**滤波器类型**: 2阶陷波滤波器 (Notch Filter / Band-Stop Filter)

**传递函数**:
```
        s² + 2ζ₁ωₙs + ωₙ²
H(s) = ─────────────────
        s² + 2ζ₂ωₙs + ωₙ²

其中:
ωₙ = 2π × f_notch  (陷波中心频率)
ζ₁ << ζ₂           (使分子快速衰减)
带宽 = 2 × ζ₂ × f_notch
```

**代码位置**:
- 配置: `VehicleAngularVelocity.cpp:175-182`
- 应用: `VehicleAngularVelocity.cpp:753-761`

**用途**: 消除已知频率的振动（如机架共振频率）

#### 滤波器5: 低通滤波器

**参数**:
```c
// 文件: imu_gyro_parameters.c:128
PARAM_DEFINE_FLOAT(IMU_GYRO_CUTOFF, 40.0f);  // 截止频率 (默认40Hz)
```

**工作原理**:
```
2阶Butterworth低通滤波器

截止频率40Hz → 40Hz以上的高频信号被衰减
```

**滤波器类型**: 2阶Butterworth低通滤波器

**频率响应**:
```
幅度(dB)
  0  |        ___________
     |       /
 -20 |      /
     |     /  <-- 40Hz截止频率
 -40 |    /
     |___/
     0   20   40   60   80  100  频率(Hz)

衰减率: -40dB/decade (2阶滤波器)
```

**代码位置**:
- 配置: `VehicleAngularVelocity.cpp:171-172`
- 应用: `VehicleAngularVelocity.cpp:763-764`

**用途**: 消除高频噪声，平滑角速度信号

### 3.3 角加速度滤波器

**文件**: `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`

```cpp
// 行号: 770-782
float VehicleAngularVelocity::FilterAngularAcceleration(int axis, float inverse_dt_s, float data[], int N)
{
    // *** 微分计算 ***
    float angular_acceleration_filtered = 0.f;

    for (int n = 0; n < N; n++) {
        // 行号: 776
        // 数值微分: (当前速度 - 上次速度) / dt
        const float angular_acceleration = (data[n] - _angular_velocity_raw_prev(axis)) * inverse_dt_s;

        // 行号: 777
        // *** Alpha滤波器 (低通) ***
        angular_acceleration_filtered = _lp_filter_acceleration[axis].update(angular_acceleration);

        // 行号: 778
        // 保存当前值用于下次微分
        _angular_velocity_raw_prev(axis) = data[n];
    }

    // 行号: 781
    return angular_acceleration_filtered;
}
```

**参数**:
```c
// 文件: imu_gyro_parameters.c:172
PARAM_DEFINE_FLOAT(IMU_DGYRO_CUTOFF, 20.0f);  // D-term截止频率 (默认20Hz)
```

**用途**:
- 计算角加速度（角速度的导数）
- 用于控制器的D项（PID控制的微分项）
- 需要额外滤波，因为微分会放大噪声

**配置**:
```cpp
// 行号: 185-192
if ((_param_imu_dgyro_cutoff.get() > 0.f)
    && (_lp_filter_acceleration[axis].setCutoffFreq(_filter_sample_rate_hz, _param_imu_dgyro_cutoff.get()))) {
    _lp_filter_acceleration[axis].reset(angular_acceleration_uncalibrated(axis));
} else {
    // disable filtering (alpha = 1, no filtering)
    _lp_filter_acceleration[axis].setAlpha(1.f);
}
```

### 3.4 滤波器链路总览

```
输入数据流:
原始数据 [125, 130] (int16)
    ↓ 缩放
物理单位 [0.0666, 0.0692] (rad/s)
    ↓ ESC RPM陷波 (可选)
消除电机振动 [0.0665, 0.0691]
    ↓ FFT陷波 (可选)
消除共振 [0.0664, 0.0690]
    ↓ 陷波0 (IMU_GYRO_NF0_FRQ)
消除已知振动 [0.0663, 0.0689]
    ↓ 陷波1 (IMU_GYRO_NF1_FRQ)
再次消除 [0.0662, 0.0688]
    ↓ 低通 (IMU_GYRO_CUTOFF=40Hz)
平滑信号 [0.0661, 0.0687]
    ↓
输出: 最后一个样本 0.0687 rad/s
```

**滤波顺序的重要性**:
1. **先陷波**: 移除特定频率的振动
2. **后低通**: 整体平滑高频噪声
3. **顺序不能颠倒**: 低通会影响陷波滤波器的效果

### 3.5 滤波器初始化配置

**文件**: `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`

```cpp
// 行号: 162-203
void VehicleAngularVelocity::ResetFilters(const hrt_abstime &time_now_us)
{
    if ((_filter_sample_rate_hz > 0) && PX4_ISFINITE(_filter_sample_rate_hz)) {

        const Vector3f angular_velocity_uncalibrated{GetResetAngularVelocity()};
        const Vector3f angular_acceleration_uncalibrated{GetResetAngularAcceleration()};

        for (int axis = 0; axis < 3; axis++) {
            // *** 低通滤波器配置 ***
            // 行号: 171-172
            _lp_filter_velocity[axis].set_cutoff_frequency(
                _filter_sample_rate_hz,      // 采样率 (例如1600Hz)
                _param_imu_gyro_cutoff.get() // 截止频率 (默认40Hz)
            );
            _lp_filter_velocity[axis].reset(angular_velocity_uncalibrated(axis));

            // *** 陷波滤波器0配置 ***
            // 行号: 175-177
            _notch_filter0_velocity[axis].setParameters(
                _filter_sample_rate_hz,           // 采样率
                _param_imu_gyro_nf0_frq.get(),   // 中心频率
                _param_imu_gyro_nf0_bw.get()     // 带宽
            );
            _notch_filter0_velocity[axis].reset();

            // *** 陷波滤波器1配置 ***
            // 行号: 180-182
            _notch_filter1_velocity[axis].setParameters(
                _filter_sample_rate_hz,
                _param_imu_gyro_nf1_frq.get(),
                _param_imu_gyro_nf1_bw.get()
            );
            _notch_filter1_velocity[axis].reset();

            // *** 角加速度低通滤波器配置 ***
            // 行号: 185-192
            if ((_param_imu_dgyro_cutoff.get() > 0.f)
                && (_lp_filter_acceleration[axis].setCutoffFreq(
                    _filter_sample_rate_hz,
                    _param_imu_dgyro_cutoff.get()))) {
                _lp_filter_acceleration[axis].reset(angular_acceleration_uncalibrated(axis));
            } else {
                // 禁用滤波 (alpha = 1)
                _lp_filter_acceleration[axis].setAlpha(1.f);
            }
        }

        // 行号: 196-197
        // 强制重置动态陷波滤波器
        UpdateDynamicNotchEscRpm(time_now_us, true);
        UpdateDynamicNotchFFT(time_now_us, true);

        _reset_filters = false;
    }
}
```

### 3.6 滤波器参数完整列表

| 参数名 | 默认值 | 单位 | 范围 | 说明 | 文件位置 |
|--------|-------|------|------|------|---------|
| `IMU_GYRO_CUTOFF` | 40.0 | Hz | 0-1000 | 角速度低通滤波截止频率 | imu_gyro_parameters.c:128 |
| `IMU_GYRO_NF0_FRQ` | 0.0 | Hz | 0-1000 | 陷波滤波器0中心频率 (0=禁用) | imu_gyro_parameters.c:53 |
| `IMU_GYRO_NF0_BW` | 20.0 | Hz | 0-100 | 陷波滤波器0带宽 | imu_gyro_parameters.c:70 |
| `IMU_GYRO_NF1_FRQ` | 0.0 | Hz | 0-1000 | 陷波滤波器1中心频率 (0=禁用) | imu_gyro_parameters.c:91 |
| `IMU_GYRO_NF1_BW` | 20.0 | Hz | 0-100 | 陷波滤波器1带宽 | imu_gyro_parameters.c:108 |
| `IMU_DGYRO_CUTOFF` | 20.0 | Hz | 0-1000 | 角加速度低通滤波截止频率 | imu_gyro_parameters.c:172 |
| `IMU_GYRO_RATEMAX` | 400 | Hz | 100-2000 | 最大发布频率（内环频率） | imu_gyro_parameters.c:150 |
| `IMU_GYRO_DNF_EN` | 0 | - | 0-3 | 动态陷波使能 (bit0=ESC, bit1=FFT) | imu_gyro_parameters.c:185 |
| `IMU_GYRO_DNF_BW` | 15.0 | Hz | 5-30 | 动态陷波带宽 | imu_gyro_parameters.c:199 |
| `IMU_GYRO_DNF_HMC` | 3 | - | 1-7 | ESC RPM谐波数量 | imu_gyro_parameters.c:210 |
| `IMU_GYRO_DNF_MIN` | 25.0 | Hz | - | 动态陷波最小频率 | imu_gyro_parameters.c:222 |

---

## 四、校准与发布

### 4.1 校准处理

**文件**: `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`

```cpp
// 行号: 921-950
bool VehicleAngularVelocity::CalibrateAndPublish(
    const hrt_abstime &timestamp_sample,
    const Vector3f &angular_velocity_uncalibrated,
    const Vector3f &angular_acceleration_uncalibrated)
{
    // 行号: 925
    // 检查发布间隔（限流）
    if (timestamp_sample >= _last_publish + _publish_interval_min_us) {

        // 行号: 928-929
        vehicle_angular_velocity_s angular_velocity;
        angular_velocity.timestamp_sample = timestamp_sample;

        // *** 关键步骤1: 校准角速度 ***
        // 行号: 932
        // Angular velocity: rotate sensor frame to board, scale raw data to SI, apply calibration, and remove in-run estimated bias
        _angular_velocity = _calibration.Correct(angular_velocity_uncalibrated) - _bias;

        // 行号: 933
        _angular_velocity.copyTo(angular_velocity.xyz);

        // *** 关键步骤2: 校准角加速度 ***
        // 行号: 936
        // Angular acceleration: rotate sensor frame to board, scale raw data to SI, apply any additional configured rotation
        _angular_acceleration = _calibration.rotation() * angular_acceleration_uncalibrated;

        // 行号: 937
        _angular_acceleration.copyTo(angular_velocity.xyz_derivative);

        // 行号: 939-940
        angular_velocity.timestamp = hrt_absolute_time();
        _vehicle_angular_velocity_pub.publish(angular_velocity);

        // 行号: 943-944
        // 更新发布时间（防止过快发布）
        _last_publish = math::constrain(_last_publish + _publish_interval_min_us,
                                        timestamp_sample - _publish_interval_min_us,
                                        timestamp_sample);

        return true;
    }

    return false;
}
```

**校准包含的操作**:
```
_calibration.Correct(angular_velocity_uncalibrated) - _bias

展开为:
1. 应用校准矩阵 (温度补偿、非线性修正)
2. 应用传感器偏置
3. 减去运行时估计的偏置 (_bias)
```

**发布的数据**:
```cpp
vehicle_angular_velocity_s {
    timestamp_sample: 123456789,        // 采样时间戳
    timestamp: 123456889,               // 发布时间戳
    xyz[3]: [0.0661, -0.0123, 0.0045], // 校准后的角速度(rad/s)
    xyz_derivative[3]: [0.12, -0.05, 0.01] // 角加速度(rad/s²)
}
```

---

## 五、姿态解算（EKF2）

### 5.1 EKF2订阅IMU数据

**文件**: `src/modules/ekf2/EKF2.cpp`

```cpp
// 行号: 746-751
if (imu_updated) {
    const hrt_abstime now = imu_sample_new.time_us;

    // 行号: 750
    // push imu data into estimator
    _ekf.setIMUData(imu_sample_new);

    // 行号: 751
    // *** 关键: 立即发布姿态 ***
    PublishAttitude(now);  // publish attitude immediately (uses quaternion from output predictor)
}
```

### 5.2 姿态预测

**文件**: `src/modules/ekf2/EKF/ekf.cpp`

```cpp
// 行号: 231-249
void Ekf::predictState(const imuSample &imu_delayed)
{
    // 行号: 233-235
    // 更新地球自转率（用于高精度）
    if (std::fabs(_gpos.latitude_rad() - _earth_rate_lat_ref_rad) > math::radians(1.0)) {
        _earth_rate_lat_ref_rad = _gpos.latitude_rad();
        _earth_rate_NED = calcEarthRateNED((float)_earth_rate_lat_ref_rad);
    }

    // *** 步骤1: 角度增量修正 ***
    // 行号: 239-240
    // 应用陀螺仪偏置修正
    const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
    Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;

    // 行号: 243
    // 减去地球自转分量（高精度导航需要）
    corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_delayed.delta_ang_dt;

    // *** 步骤2: 姿态更新 ***
    // 行号: 245
    // 将角度增量转换为四元数
    const Quatf dq(AxisAnglef{corrected_delta_ang});

    // 行号: 248
    // *** 四元数乘法更新姿态 ***
    _state.quat_nominal = (_state.quat_nominal * dq).normalized();

    // 行号: 249
    // 更新旋转矩阵（用于后续坐标变换）
    _R_to_earth = Dcmf(_state.quat_nominal);

    // ... 后续还有速度和位置的预测
}
```

**四元数更新公式**:
```
q(t+dt) = q(t) ⊗ dq

其中:
q(t): 当前姿态四元数
dq: 增量四元数，由角度增量 Δθ 构造
⊗: 四元数乘法

数学表达:
dq = [cos(|Δθ|/2), sin(|Δθ|/2) × Δθ/|Δθ|]
```

### 5.3 姿态发布

**文件**: `src/modules/ekf2/EKF2.cpp`

```cpp
// 行号: 1036-1054
void EKF2::PublishAttitude(const hrt_abstime &timestamp)
{
    // 行号: 1038
    if (_ekf.attitude_valid()) {
        // 行号: 1040-1042
        // 生成vehicle_attitude消息
        vehicle_attitude_s att;
        att.timestamp_sample = timestamp;  // IMU采样时间戳
        _ekf.getQuaternion().copyTo(att.q);  // 姿态四元数 [w, x, y, z]

        // 行号: 1044
        // 获取四元数重置信息（用于检测姿态跳变）
        _ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);

        // 行号: 1045-1046
        att.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
        _attitude_pub.publish(att);  // *** 发布姿态 ***

    } else if (_replay_mode) {
        // 回放模式下发布零时间戳的姿态
        vehicle_attitude_s att{};
        _attitude_pub.publish(att);
    }
}
```

**发布的数据结构**:
```cpp
vehicle_attitude_s {
    timestamp_sample: 123456789,      // IMU采样时间
    timestamp: 123456889,             // 发布时间
    q[4]: [0.998, 0.001, 0.002, 0.060],  // 姿态四元数 [w, x, y, z]
    delta_q_reset[4]: [1.0, 0, 0, 0], // 重置增量
    quat_reset_counter: 0              // 重置计数器
}
```

**四元数到欧拉角转换**:
```cpp
// 下游模块可以将四元数转换为欧拉角
Quatf q(att.q);
Eulerf euler(q);

float roll = euler.phi();    // 翻滚角
float pitch = euler.theta(); // 俯仰角
float yaw = euler.psi();     // 偏航角
```

### 5.4 EKF2Selector的最终发布

**文件**: `src/modules/ekf2/EKF2Selector.cpp`

```cpp
// 行号: 364-416
void EKF2Selector::PublishVehicleAttitude()
{
    // 行号: 369
    // 从选定的EKF实例订阅estimator_attitude
    vehicle_attitude_s attitude;
    if (_instance[_selected_instance].estimator_attitude_sub.update(&attitude)) {

        bool instance_change = false;

        // 行号: 372-375
        // 检测实例切换
        if (_instance[_selected_instance].estimator_attitude_sub.get_instance() != _attitude_instance_prev) {
            _attitude_instance_prev = _instance[_selected_instance].estimator_attitude_sub.get_instance();
            instance_change = true;
        }

        // 行号: 377-392
        // 计算四元数重置增量
        if (_attitude_last.timestamp != 0) {
            if (!instance_change && (attitude.quat_reset_counter == _attitude_last.quat_reset_counter + 1)) {
                // 正常增量
                ++_quat_reset_counter;
                _delta_q_reset = Quatf{attitude.delta_q_reset};
            } else if (instance_change || (attitude.quat_reset_counter != _attitude_last.quat_reset_counter)) {
                // 重置或切换，计算增量
                ++_quat_reset_counter;
                _delta_q_reset = (Quatf(attitude.q) * Quatf(_attitude_last.q).inversed()).normalized();
            }
        } else {
            _quat_reset_counter = attitude.quat_reset_counter;
            _delta_q_reset = Quatf{attitude.delta_q_reset};
        }

        bool publish = true;

        // 行号: 398-402
        // 确保时间戳单调递增，防止发布过时数据
        if ((attitude.timestamp_sample <= _attitude_last.timestamp_sample)
            || (hrt_elapsed_time(&attitude.timestamp) > 10_ms)) {
            publish = false;
        }

        // 行号: 405
        _attitude_last = attitude;

        // 行号: 407-414
        if (publish) {
            // *** 最终发布vehicle_attitude ***
            // 行号: 409-410
            attitude.quat_reset_counter = _quat_reset_counter;
            _delta_q_reset.copyTo(attitude.delta_q_reset);

            // 行号: 412-413
            attitude.timestamp = hrt_absolute_time();
            _vehicle_attitude_pub.publish(attitude);  // *** 发布到系统 ***
        }
    }
}
```

---

## 六、完整数据管道代码位置

### 6.1 数据流各阶段精确位置

| 阶段 | 模块 | 文件 | 关键函数 | 行号 | 操作 |
|------|------|------|---------|------|------|
| **采样** | BMI270驱动 | `src/drivers/imu/bosch/bmi270/BMI270.cpp` | `FIFORead()` | 732-850 | SPI读取FIFO |
| **旋转** | PX4Gyroscope | `src/lib/drivers/gyroscope/PX4Gyroscope.cpp` | `updateFIFO()` | 142-144 | 传感器坐标系→板坐标系 |
| **发布原始** | PX4Gyroscope | `src/lib/drivers/gyroscope/PX4Gyroscope.cpp` | `updateFIFO()` | 149 | 发布sensor_gyro_fifo_s |
| **订阅** | VehicleAngularVelocity | `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` | `Run()` | 828 | 订阅sensor_gyro_fifo_s |
| **缩放** | VehicleAngularVelocity | 同上 | `Run()` | 845-847 | int16 × scale → float(rad/s) |
| **滤波1** | VehicleAngularVelocity | 同上 | `FilterAngularVelocity()` | 730-740 | ESC RPM动态陷波 |
| **滤波2** | VehicleAngularVelocity | 同上 | `FilterAngularVelocity()` | 742-749 | FFT动态陷波 |
| **滤波3** | VehicleAngularVelocity | 同上 | `FilterAngularVelocity()` | 753-756 | 陷波0 (IMU_GYRO_NF0) |
| **滤波4** | VehicleAngularVelocity | 同上 | `FilterAngularVelocity()` | 758-761 | 陷波1 (IMU_GYRO_NF1) |
| **滤波5** | VehicleAngularVelocity | 同上 | `FilterAngularVelocity()` | 763-764 | 低通 (IMU_GYRO_CUTOFF) |
| **微分** | VehicleAngularVelocity | 同上 | `FilterAngularAcceleration()` | 776 | 角速度→角加速度 |
| **滤波6** | VehicleAngularVelocity | 同上 | `FilterAngularAcceleration()` | 777 | 角加速度低通 |
| **校准** | VehicleAngularVelocity | 同上 | `CalibrateAndPublish()` | 932 | 应用校准和去偏置 |
| **发布** | VehicleAngularVelocity | 同上 | `CalibrateAndPublish()` | 940 | 发布vehicle_angular_velocity |
| **EKF输入** | EKF2 | `src/modules/ekf2/EKF2.cpp` | `Run()` | 750 | setIMUData() |
| **姿态预测** | EKF | `src/modules/ekf2/EKF/ekf.cpp` | `predictState()` | 248 | 四元数更新 |
| **姿态发布** | EKF2 | `src/modules/ekf2/EKF2.cpp` | `PublishAttitude()` | 1046 | 发布estimator_attitude |
| **最终发布** | EKF2Selector | `src/modules/ekf2/EKF2Selector.cpp` | `PublishVehicleAttitude()` | 413 | 发布vehicle_attitude |

### 6.2 数据变换时间线

```
t=0.000ms: BMI270 FIFO中断触发
    ↓
t=0.050ms: BMI270::FIFORead()
    文件: BMI270.cpp:732-850
    操作: SPI读取 → 解析帧 → 填充sensor_gyro_fifo_s
    数据: x[2]={125, 130} (int16)
    ↓
t=0.080ms: PX4Gyroscope::updateFIFO()
    文件: PX4Gyroscope.cpp:137-176
    操作: rotate_3i() 旋转坐标系
    数据: x[2]={125, 130} (旋转后的int16)
    ↓
t=0.100ms: _sensor_fifo_pub.publish()
    文件: PX4Gyroscope.cpp:149
    操作: 发布sensor_gyro_fifo_s到uORB
    数据: {samples=2, scale=0.000532, x[2]={125,130}}
    ↓
t=0.120ms: VehicleAngularVelocity::Run()
    文件: VehicleAngularVelocity.cpp:828
    操作: _sensor_gyro_fifo_sub.update()
    数据: 收到sensor_gyro_fifo_s
    ↓
t=0.130ms: 缩放转换
    文件: VehicleAngularVelocity.cpp:845-847
    操作: data[n] = scale × raw_data[n]
    数据: [0.0665, 0.0692] (rad/s)
    ↓
t=0.150ms: FilterAngularVelocity()
    文件: VehicleAngularVelocity.cpp:725-768
    操作: 5级滤波器级联
    数据: [0.0665, 0.0692] → 0.0687 (最后样本)
    ↓
t=0.170ms: FilterAngularAcceleration()
    文件: VehicleAngularVelocity.cpp:770-782
    操作: 微分 + 低通滤波
    数据: 角加速度 = 0.12 rad/s²
    ↓
t=0.190ms: CalibrateAndPublish()
    文件: VehicleAngularVelocity.cpp:921-950
    操作: 校准 + 去偏置
    数据: xyz = [0.0661, -0.0123, 0.0045] (rad/s)
    ↓
t=0.200ms: _vehicle_angular_velocity_pub.publish()
    文件: VehicleAngularVelocity.cpp:940
    操作: 发布vehicle_angular_velocity_s
    ↓
t=0.220ms: EKF2::Run()
    文件: EKF2.cpp:750
    操作: _ekf.setIMUData()
    ↓
t=0.250ms: Ekf::predictState()
    文件: ekf.cpp:231-272
    操作: 四元数更新 q = q ⊗ dq
    数据: q = [0.998, 0.001, 0.002, 0.060]
    ↓
t=0.270ms: EKF2::PublishAttitude()
    文件: EKF2.cpp:1036-1054
    操作: 发布estimator_attitude
    ↓
t=0.290ms: EKF2Selector::PublishVehicleAttitude()
    文件: EKF2Selector.cpp:364-416
    操作: 发布vehicle_attitude
    ↓
t=0.300ms: 控制器订阅vehicle_attitude
    文件: src/modules/mc_att_control/
    操作: 姿态控制环使用

总延迟: ~0.3ms (从FIFO读取到姿态发布)
```

---

## 七、关键代码位置速查表

### 7.1 缩放与旋转

| 操作 | 文件 | 函数 | 行号 | 说明 |
|------|------|------|------|------|
| 坐标旋转 | `src/lib/drivers/gyroscope/PX4Gyroscope.cpp` | `updateFIFO()` | 142-144 | 传感器坐标→板坐标 |
| 设置缩放因子 | `src/drivers/imu/bosch/bmi270/BMI270.cpp` | `SetGyroScale()` | 504-510 | scale=rad(2000)/32767 |
| 数值缩放 | `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` | `Run()` | 846 | int16×scale→float |

### 7.2 滤波器

| 滤波器 | 文件 | 函数 | 行号 | 参数 | 默认值 |
|--------|------|------|------|------|--------|
| ESC RPM陷波 | `VehicleAngularVelocity.cpp` | `FilterAngularVelocity()` | 730-740 | IMU_GYRO_DNF_EN(bit0) | 0(禁用) |
| FFT动态陷波 | `VehicleAngularVelocity.cpp` | `FilterAngularVelocity()` | 742-749 | IMU_GYRO_DNF_EN(bit1) | 0(禁用) |
| 陷波滤波器0 | `VehicleAngularVelocity.cpp` | `FilterAngularVelocity()` | 753-756 | IMU_GYRO_NF0_FRQ | 0Hz(禁用) |
| 陷波滤波器1 | `VehicleAngularVelocity.cpp` | `FilterAngularVelocity()` | 758-761 | IMU_GYRO_NF1_FRQ | 0Hz(禁用) |
| 低通滤波器 | `VehicleAngularVelocity.cpp` | `FilterAngularVelocity()` | 763-764 | IMU_GYRO_CUTOFF | 40Hz |
| 角加速度低通 | `VehicleAngularVelocity.cpp` | `FilterAngularAcceleration()` | 777 | IMU_DGYRO_CUTOFF | 20Hz |

### 7.3 校准与发布

| 操作 | 文件 | 函数 | 行号 | 说明 |
|------|------|------|------|------|
| 校准角速度 | `VehicleAngularVelocity.cpp` | `CalibrateAndPublish()` | 932 | 应用校准矩阵并去偏置 |
| 校准角加速度 | `VehicleAngularVelocity.cpp` | `CalibrateAndPublish()` | 936 | 应用旋转矩阵 |
| 发布角速度 | `VehicleAngularVelocity.cpp` | `CalibrateAndPublish()` | 940 | vehicle_angular_velocity |

### 7.4 姿态解算

| 操作 | 文件 | 函数 | 行号 | 说明 |
|------|------|------|------|------|
| IMU数据输入 | `src/modules/ekf2/EKF2.cpp` | `Run()` | 750 | setIMUData() |
| 姿态预测 | `src/modules/ekf2/EKF/ekf.cpp` | `predictState()` | 248 | 四元数更新 q⊗dq |
| 获取姿态 | `src/modules/ekf2/EKF/ekf.cpp` | `getQuaternion()` | - | 返回_state.quat_nominal |
| 发布姿态(EKF) | `src/modules/ekf2/EKF2.cpp` | `PublishAttitude()` | 1046 | 发布estimator_attitude |
| 发布姿态(系统) | `src/modules/ekf2/EKF2Selector.cpp` | `PublishVehicleAttitude()` | 413 | 发布vehicle_attitude |

---

## 八、滤波器详细参数

### 8.1 低通滤波器 (Butterworth)

**类型**: 2阶Butterworth低通滤波器
**实现**: `src/lib/mathlib/math/filter/LowPassFilter2p.hpp`

**参数配置**:
```c
// 文件: imu_gyro_parameters.c:128
PARAM_DEFINE_FLOAT(IMU_GYRO_CUTOFF, 40.0f);
```

**配置代码**:
```cpp
// 文件: VehicleAngularVelocity.cpp:171-172
_lp_filter_velocity[axis].set_cutoff_frequency(
    _filter_sample_rate_hz,      // 1600 Hz (采样率)
    _param_imu_gyro_cutoff.get() // 40 Hz (截止频率)
);
```

**数学特性**:
- **截止频率**: 40Hz（-3dB点）
- **衰减率**: -40dB/decade（2阶）
- **相位延迟**: 约0.5ms @ 40Hz
- **群延迟**: 约4ms

**频率响应曲线**:
```
增益(dB)
  0  |＿＿＿＿＿＿
     |          ＼
-20  |            ＼
     |              ＼  <-- -40dB/decade
-40  |                ＼
     |                  ＼
-60  |                    ＼
     +----+----+----+----+----+----
     0   20   40   60   80  100  频率(Hz)
              ↑
           40Hz截止
```

**推荐配置**:
- **高性能**: 60-80Hz（低延迟，但噪声较大）
- **标准**: 40Hz（平衡）
- **平稳**: 20-30Hz（低噪声，但延迟增加）

### 8.2 陷波滤波器 (Notch Filter)

**类型**: 2阶陷波滤波器
**实现**: `src/lib/mathlib/math/filter/NotchFilter.hpp`

**参数配置**:
```c
// 陷波滤波器0
PARAM_DEFINE_FLOAT(IMU_GYRO_NF0_FRQ, 0.0f);  // 中心频率
PARAM_DEFINE_FLOAT(IMU_GYRO_NF0_BW, 20.0f);  // 带宽

// 陷波滤波器1
PARAM_DEFINE_FLOAT(IMU_GYRO_NF1_FRQ, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYRO_NF1_BW, 20.0f);
```

**配置代码**:
```cpp
// 文件: VehicleAngularVelocity.cpp:175-182
_notch_filter0_velocity[axis].setParameters(
    _filter_sample_rate_hz,           // 1600 Hz
    _param_imu_gyro_nf0_frq.get(),   // 例如: 120 Hz
    _param_imu_gyro_nf0_bw.get()     // 例如: 20 Hz
);
```

**数学特性**:
- **中心频率**: 用户配置（例如120Hz）
- **带宽**: 阻带宽度（例如20Hz）
- **阻带范围**: 110-130Hz（中心±带宽/2）
- **衰减**: 阻带内-30dB以上

**频率响应曲线**:
```
增益(dB)
  0  |＿＿    ＿＿
     |   ＼  /
-10  |    ＼/
     |    /＼   <-- 陷波
-20  |   /  ＼
     |  /    ＼
-30  | /      ＼
     +----+----+----+----
     90  110  130  150  频率(Hz)
          ↑    ↑
        110  130 (带宽20Hz)
         中心120Hz
```

**使用场景**:
- 消除机架共振频率
- 消除螺旋桨叶通过频率
- 消除电机振动

**配置示例**:
```bash
# 测量振动频率（通过频谱分析）
# 发现120Hz有明显振动峰

# 配置陷波滤波器
param set IMU_GYRO_NF0_FRQ 120  # 中心频率120Hz
param set IMU_GYRO_NF0_BW 20    # 带宽20Hz
```

### 8.3 动态陷波滤波器 (ESC RPM)

**参数配置**:
```c
PARAM_DEFINE_INT32(IMU_GYRO_DNF_EN, 0);      // bit0=1 启用ESC RPM
PARAM_DEFINE_FLOAT(IMU_GYRO_DNF_BW, 15.f);   // 带宽
PARAM_DEFINE_INT32(IMU_GYRO_DNF_HMC, 3);     // 谐波数量
PARAM_DEFINE_FLOAT(IMU_GYRO_DNF_MIN, 25.f);  // 最小频率
```

**工作原理**:
```cpp
// 文件: VehicleAngularVelocity.cpp:536-712
void VehicleAngularVelocity::UpdateDynamicNotchEscRpm(...)
{
    // 订阅ESC状态
    esc_status_s esc_status;
    if (_esc_status_sub.update(&esc_status)) {

        for (int esc = 0; esc < MAX_NUM_ESCS; esc++) {
            // 读取ESC RPM
            float rpm = esc_status.esc[esc].esc_rpm;
            float frequency_hz = rpm / 60.f;  // RPM → Hz

            // 为每个谐波创建陷波
            for (int harmonic = 0; harmonic < _esc_rpm_harmonics; harmonic++) {
                float notch_freq = frequency_hz * (harmonic + 1);

                if (notch_freq > _param_imu_gyro_dnf_min.get()) {
                    // 设置陷波频率
                    _dynamic_notch_filter_esc_rpm[harmonic][axis][esc].setParameters(
                        _filter_sample_rate_hz,
                        notch_freq,
                        _param_imu_gyro_dnf_bw.get()
                    );
                }
            }
        }
    }
}
```

**示例**:
```
ESC1 RPM: 6000 RPM → 100 Hz基频
谐波配置: IMU_GYRO_DNF_HMC = 3
带宽: IMU_GYRO_DNF_BW = 15 Hz

创建的陷波滤波器:
1. 100 Hz ± 7.5Hz (基频)
2. 200 Hz ± 7.5Hz (2倍谐波)
3. 300 Hz ± 7.5Hz (3倍谐波)

对4个电机 × 3个谐波 × 3个轴 = 36个陷波滤波器！
```

**代码位置**:
- 更新频率: `VehicleAngularVelocity.cpp:536-712`
- 应用滤波: `VehicleAngularVelocity.cpp:730-740`

### 8.4 动态陷波滤波器 (FFT)

**参数配置**:
```c
PARAM_DEFINE_INT32(IMU_GYRO_DNF_EN, 0);  // bit1=1 启用FFT
PARAM_DEFINE_INT32(IMU_GYRO_FFT_EN, 0);  // 启用板载FFT
```

**工作原理**:
```cpp
// 文件: VehicleAngularVelocity.cpp:714-820
void VehicleAngularVelocity::UpdateDynamicNotchFFT(...)
{
    // 订阅FFT结果
    sensor_gyro_fft_s sensor_gyro_fft;
    if (_sensor_gyro_fft_sub.update(&sensor_gyro_fft)) {

        // 读取FFT检测的峰值频率
        for (int peak = 0; peak < MAX_NUM_FFT_PEAKS; peak++) {
            float peak_freq_x = sensor_gyro_fft.peak_frequencies_x[peak];
            float peak_freq_y = sensor_gyro_fft.peak_frequencies_y[peak];
            float peak_freq_z = sensor_gyro_fft.peak_frequencies_z[peak];

            // 为每个检测到的峰值设置陷波
            if (peak_freq_x > _param_imu_gyro_dnf_min.get()) {
                _dynamic_notch_filter_fft[0][peak].setParameters(
                    _filter_sample_rate_hz,
                    peak_freq_x,
                    _param_imu_gyro_dnf_bw.get()
                );
            }
            // ... Y/Z轴类似
        }
    }
}
```

**优势**:
- 自动检测振动频率
- 无需手动配置
- 适应飞行条件变化

**代码位置**:
- 更新频率: `VehicleAngularVelocity.cpp:714-820`
- 应用滤波: `VehicleAngularVelocity.cpp:742-749`

---

## 九、姿态角发布详解

### 9.1 姿态表示形式

PX4使用**四元数**表示姿态，而非欧拉角（避免万向节死锁）。

**四元数定义**:
```
q = [q₀, q₁, q₂, q₃] = [w, x, y, z]

其中:
q₀ = w = cos(θ/2)
q₁ = x = sin(θ/2) × axis_x
q₂ = y = sin(θ/2) × axis_y
q₃ = z = sin(θ/2) × axis_z

约束: q₀² + q₁² + q₂² + q₃² = 1
```

### 9.2 姿态发布前的处理

**文件**: `src/modules/ekf2/EKF2.cpp:1036-1054`

```cpp
void EKF2::PublishAttitude(const hrt_abstime &timestamp)
{
    if (_ekf.attitude_valid()) {
        vehicle_attitude_s att;

        // *** 步骤1: 设置时间戳 ***
        // 行号: 1041
        att.timestamp_sample = timestamp;  // IMU采样时间

        // *** 步骤2: 获取四元数 ***
        // 行号: 1042
        _ekf.getQuaternion().copyTo(att.q);

        // 展开:
        // Quatf q = _ekf.getQuaternion();
        // q的来源: _state.quat_nominal (在predictState中更新)
        // att.q[0] = q(0);  // w
        // att.q[1] = q(1);  // x
        // att.q[2] = q(2);  // y
        // att.q[3] = q(3);  // z

        // *** 步骤3: 获取重置信息 ***
        // 行号: 1044
        _ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);

        // 用途: 告诉下游模块姿态是否被重置过
        // delta_q_reset: 重置时的四元数增量
        // quat_reset_counter: 重置计数器

        // *** 步骤4: 发布 ***
        // 行号: 1045-1046
        att.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
        _attitude_pub.publish(att);
    }
}
```

**发布前的EKF状态更新**:
```cpp
// 文件: src/modules/ekf2/EKF/ekf.cpp:231-249
void Ekf::predictState(const imuSample &imu_delayed)
{
    // 步骤1: 修正陀螺仪偏置
    const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
    Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;

    // 步骤2: 修正地球自转（高精度导航）
    corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_delayed.delta_ang_dt;

    // 步骤3: 构造增量四元数
    const Quatf dq(AxisAnglef{corrected_delta_ang});

    // 步骤4: 四元数乘法更新姿态
    _state.quat_nominal = (_state.quat_nominal * dq).normalized();

    // 步骤5: 更新旋转矩阵
    _R_to_earth = Dcmf(_state.quat_nominal);
}
```

### 9.3 姿态角的最终发布位置

**主发布点**: `src/modules/ekf2/EKF2Selector.cpp:413`

```cpp
// 行号: 364-416
void EKF2Selector::PublishVehicleAttitude()
{
    vehicle_attitude_s attitude;

    // 行号: 369
    // 从选定的EKF实例读取
    if (_instance[_selected_instance].estimator_attitude_sub.update(&attitude)) {

        // ... 计算重置增量 (行号: 372-392)

        bool publish = true;

        // 行号: 398-402
        // *** 检查1: 时间戳有效性 ***
        if ((attitude.timestamp_sample <= _attitude_last.timestamp_sample)
            || (hrt_elapsed_time(&attitude.timestamp) > 10_ms)) {
            publish = false;  // 过时数据，不发布
        }

        // 行号: 405
        _attitude_last = attitude;

        if (publish) {
            // 行号: 409-410
            // *** 处理1: 更新重置计数器 ***
            attitude.quat_reset_counter = _quat_reset_counter;
            _delta_q_reset.copyTo(attitude.delta_q_reset);

            // 行号: 412
            // *** 处理2: 更新时间戳为当前时间 ***
            attitude.timestamp = hrt_absolute_time();

            // 行号: 413
            // *** 最终发布 vehicle_attitude ***
            _vehicle_attitude_pub.publish(attitude);
        }
    }
}
```

**发布前的检查**:
1. **时间戳单调性**: 确保新数据比上次新
2. **数据新鲜度**: 时间戳不能超过10ms
3. **实例切换**: 处理多EKF实例切换的情况

**发布的完整数据**:
```cpp
vehicle_attitude_s {
    timestamp_sample: 123456789,      // IMU采样时间戳
    timestamp: 123456889,             // 发布时间戳

    // 姿态四元数 (NED坐标系到机体坐标系)
    q[4]: [0.9980, 0.0010, 0.0020, 0.0600],  // [w, x, y, z]

    // 转换为欧拉角:
    // roll  ≈ 0.2° (绕X轴)
    // pitch ≈ 0.4° (绕Y轴)
    // yaw   ≈ 6.9° (绕Z轴)

    // 重置信息
    delta_q_reset[4]: [1.0, 0, 0, 0], // 重置增量四元数
    quat_reset_counter: 0              // 重置计数器
}
```

---

## 十、完整信号链路图

```
[硬件] BMI270芯片
    1600Hz采样
    ↓
[驱动] src/drivers/imu/bosch/bmi270/BMI270.cpp
    FIFORead() [line 732-850]
    └─ 读取FIFO: x[2]={125, 130} (int16)
    ↓
[驱动] src/lib/drivers/gyroscope/PX4Gyroscope.cpp
    updateFIFO() [line 142-144]
    └─ 坐标旋转: rotate_3i()
    └─ 发布sensor_gyro_fifo_s [line 149]
        {scale=0.000532, x[2]={125,130}}
    ↓ uORB
[传感器层] src/modules/sensors/vehicle_angular_velocity/
    VehicleAngularVelocity.cpp

    Run() [line 828]
    └─ 订阅: _sensor_gyro_fifo_sub.update()

    [line 846]
    └─ 缩放: data[n] = 0.000532 × 125 = 0.0665 rad/s

    FilterAngularVelocity() [line 725-768]
    ├─ [line 730-740] ESC RPM陷波 (可选)
    ├─ [line 742-749] FFT陷波 (可选)
    ├─ [line 753-756] 陷波0 (IMU_GYRO_NF0_FRQ)
    ├─ [line 758-761] 陷波1 (IMU_GYRO_NF1_FRQ)
    └─ [line 763-764] 低通 (IMU_GYRO_CUTOFF=40Hz)
        → 输出: 0.0661 rad/s

    FilterAngularAcceleration() [line 770-782]
    ├─ [line 776] 微分: dω/dt
    └─ [line 777] 低通 (IMU_DGYRO_CUTOFF=20Hz)
        → 输出: 0.12 rad/s²

    CalibrateAndPublish() [line 921-950]
    ├─ [line 932] 校准: _calibration.Correct() - _bias
    ├─ [line 936] 旋转: rotation() × accel
    └─ [line 940] 发布: vehicle_angular_velocity_s
        {xyz=[0.0661,-0.0123,0.0045]}
    ↓ uORB
[估计层] src/modules/ekf2/

    EKF2.cpp
    Run() [line 750]
    └─ 输入: _ekf.setIMUData()

    ekf.cpp
    predictState() [line 231-249]
    ├─ [line 239-240] 去偏置: Δθ - bias
    ├─ [line 243] 去地球自转: Δθ - ω_earth×dt
    ├─ [line 245] 构造: dq = Quat(Δθ)
    └─ [line 248] 更新: q = q ⊗ dq
        → _state.quat_nominal = [0.998, 0.001, 0.002, 0.060]

    EKF2.cpp
    PublishAttitude() [line 1036-1054]
    ├─ [line 1042] 读取: _ekf.getQuaternion()
    ├─ [line 1044] 重置信息: get_quat_reset()
    └─ [line 1046] 发布: estimator_attitude
    ↓ uORB
[选择层] src/modules/ekf2/EKF2Selector.cpp

    PublishVehicleAttitude() [line 364-416]
    ├─ [line 369] 订阅: estimator_attitude
    ├─ [line 398-402] 验证时间戳
    ├─ [line 409-410] 处理重置计数
    └─ [line 413] *** 最终发布: vehicle_attitude ***
    ↓ uORB
[控制层] src/modules/mc_att_control/
    订阅vehicle_attitude
    用于姿态控制
```

---

## 十一、总结

### 11.1 数据处理流程

1. **硬件采样**: BMI270 1600Hz采样 → FIFO缓冲
2. **驱动处理**: SPI读取 → 坐标旋转 → 发布原始数据
3. **缩放转换**: int16 × scale → float(rad/s)
4. **5级滤波**: 动态陷波(×2) + 静态陷波(×2) + 低通
5. **微分滤波**: 角速度微分 → 角加速度 → 低通滤波
6. **校准发布**: 去偏置 → 发布vehicle_angular_velocity
7. **姿态估计**: EKF卡尔曼滤波 → 四元数更新
8. **最终发布**: vehicle_attitude (四元数姿态)

### 11.2 关键性能指标

| 指标 | 值 | 说明 |
|------|-----|------|
| 原始采样率 | 1600 Hz | BMI270硬件采样 |
| 滤波处理率 | 1600 Hz | 所有滤波器以原始速率运行 |
| 发布频率 | 400-800 Hz | 由IMU_GYRO_RATEMAX控制 |
| 滤波延迟 | <5 ms | 所有滤波器总延迟 |
| 姿态更新率 | 400-800 Hz | 与IMU_GYRO_RATEMAX同步 |
| 端到端延迟 | <10 ms | 采样到姿态发布 |

### 11.3 调优建议

**场景1: 追求低延迟（竞速）**
```bash
param set IMU_GYRO_RATEMAX 1000   # 1kHz更新
param set IMU_GYRO_CUTOFF 80      # 高截止频率
param set IMU_DGYRO_CUTOFF 40     # 高D-term截止
```

**场景2: 追求平稳（航拍）**
```bash
param set IMU_GYRO_RATEMAX 400    # 400Hz够用
param set IMU_GYRO_CUTOFF 30      # 低截止频率，更平滑
param set IMU_DGYRO_CUTOFF 15     # 低D-term截止
```

**场景3: 消除振动**
```bash
# 测量振动频率，发现120Hz振动
param set IMU_GYRO_NF0_FRQ 120    # 陷波滤波器
param set IMU_GYRO_NF0_BW 20      # 带宽

# 或启用动态陷波
param set IMU_GYRO_DNF_EN 1       # 启用ESC RPM
param set IMU_GYRO_DNF_HMC 3      # 3个谐波
```

---

**文档版本**: 1.0
**最后更新**: 2025-10-27
**涵盖模块**: BMI270驱动, VehicleAngularVelocity, EKF2, EKF2Selector

