# 18-EKF2纯IMU模式数据流与滤波策略分析
## 为什么EKF2使用原始IMU数据而不使用滤波后的数据

---

## 1. 概述

本文档详细分析EKF2在**仅有IMU传感器**时的数据订阅策略、数据处理流程，以及为什么选择使用原始IMU数据（`sensor_combined`/`vehicle_imu`）而不是滤波后的数据（`vehicle_angular_velocity`）。

### 核心问题
1. EKF2订阅的是哪些主题？
2. `sensor_combined`、`vehicle_imu`、`sensor_gyro/accel` 有什么区别？
3. 为什么不使用 `vehicle_angular_velocity` 这种已滤波的数据？
4. EKF2的滤波策略是什么？

---

## 2. EKF2的数据订阅关系

### 2.1 EKF2订阅的主题（仅IMU模式）

**文件**: `src/modules/ekf2/EKF2.hpp`

**第397-398行**: 核心订阅声明
```cpp
uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)};
```

**订阅模式**：
- **多实例模式**：订阅 `vehicle_imu`（每个IMU一个实例）
- **单实例模式**：订阅 `sensor_combined`（融合后的单一数据源）

### 2.2 EKF2不订阅的主题（仅IMU模式）

EKF2 **不订阅** 以下主题：
- ❌ `sensor_gyro` - 原始陀螺仪数据
- ❌ `sensor_accel` - 原始加速度计数据
- ❌ `sensor_gyro_fifo` - 陀螺仪FIFO数据
- ❌ `sensor_accel_fifo` - 加速度计FIFO数据
- ❌ `vehicle_angular_velocity` - 滤波后的角速度

**原因分析**：见后续章节

---

## 3. 传感器数据的层次结构

### 3.1 IMU数据的四个层次

```
┌─────────────────────────────────────────────────────────────────┐
│                    层次1: 硬件原始数据                           │
└─────────────────────────────────────────────────────────────────┘

sensor_gyro_fifo, sensor_accel_fifo
- 发布者: BMI270/BMI088驱动
- 频率: 800-1600 Hz
- 特点:
  * FIFO批量数据（2-3个样本）
  * 原始ADC计数值 × 缩放因子
  * 无任何滤波处理
  * 包含设备时间戳

sensor_gyro, sensor_accel
- 发布者: BMI270/BMI088驱动
- 频率: 800-1600 Hz
- 特点:
  * 单个样本数据
  * 原始ADC计数值 × 缩放因子
  * 无任何滤波处理
  * 包含设备时间戳

┌─────────────────────────────────────────────────────────────────┐
│                    层次2: 积分处理数据                           │
└─────────────────────────────────────────────────────────────────┘

vehicle_imu
- 发布者: VehicleIMU类（在Sensors模块内）
- 订阅: sensor_gyro, sensor_accel
- 频率: 200-265 Hz
- 处理流程:
  1. 读取原始陀螺仪和加速度计数据
  2. 应用校准参数（偏移、缩放、旋转矩阵）
  3. 时间对齐（加速度计和陀螺仪）
  4. 积分计算 delta_angle 和 delta_velocity
  5. 下采样到目标频率
- 特点:
  * 角度增量（rad）和速度增量（m/s）
  * 已校准但未滤波
  * 时间对齐
  * 包含积分时间窗口

┌─────────────────────────────────────────────────────────────────┐
│                    层次3: 融合选择数据                           │
└─────────────────────────────────────────────────────────────────┘

sensor_combined
- 发布者: Sensors类
- 订阅: vehicle_imu (所有实例)
- 频率: ~1000 Hz
- 处理流程:
  1. VotedSensorsUpdate从多个vehicle_imu中选择
  2. 投票算法选择最优传感器
  3. 故障检测和切换
- 特点:
  * 单一数据源（最优传感器）
  * 瞬时值（rad/s, m/s²）
  * 已校准但未滤波
  * 包含冗余和故障管理

┌─────────────────────────────────────────────────────────────────┐
│                    层次4: 滤波处理数据                           │
└─────────────────────────────────────────────────────────────────┘

vehicle_angular_velocity
- 发布者: VehicleAngularVelocity类
- 订阅: sensor_gyro_fifo
- 频率: 667 Hz
- 处理流程:
  1. 从FIFO数据提取高频样本
  2. 应用动态陷波滤波器（振动抑制）
  3. 应用静态陷波滤波器（已知频率）
  4. 应用低通滤波器（噪声抑制）
  5. 计算角加速度（数值微分）
- 特点:
  * 高度滤波的角速度
  * 平滑的角加速度
  * 引入相位延迟（~5-10ms）
  * 适合实时控制，不适合状态估计
```

---

## 4. 各层次数据的详细对比

### 4.1 数据结构对比

| 主题 | 数据类型 | 核心字段 | 采样方式 |
|------|---------|---------|---------|
| `sensor_gyro` | 瞬时值 | `float32[3] xyz` (rad/s) | 单次采样 |
| `sensor_accel` | 瞬时值 | `float32[3] xyz` (m/s²) | 单次采样 |
| `vehicle_imu` | 积分值 | `float32[3] delta_angle` (rad)<br>`float32[3] delta_velocity` (m/s)<br>`uint32 delta_angle_dt` (μs) | 多次采样积分 |
| `sensor_combined` | 瞬时值 | `float32[3] gyro_rad` (rad/s)<br>`float32[3] accelerometer_m_s2` (m/s²)<br>`uint32 gyro_integral_dt` (μs) | 投票选择 |
| `vehicle_angular_velocity` | 瞬时值 | `float32[3] xyz` (rad/s)<br>`float32[3] xyz_derivative` (rad/s²) | 滤波处理 |

### 4.2 关键代码位置

#### 4.2.1 VehicleIMU 积分处理

**文件**: `src/modules/sensors/vehicle_imu/VehicleIMU.cpp`

**第194-255行**: Run函数主循环
```cpp
void VehicleIMU::Run()
{
    // 读取陀螺仪和加速度计数据
    while ((_sensor_gyro_sub.updated() || _sensor_accel_sub.updated()) && ...) {

        // 更新陀螺仪
        if (UpdateGyro()) {
            updated = true;
        }

        // 更新加速度计
        while (_sensor_accel_sub.updated() && ...) {
            if (UpdateAccel()) {
                updated = true;
            }
        }

        // 发布积分后的IMU数据
        if (_intervals_configured && _accel_integrator.integral_ready() &&
            _gyro_integrator.integral_ready()) {
            Publish();
        }
    }
}
```

**第399-435行**: UpdateGyro函数
```cpp
bool VehicleIMU::UpdateGyro()
{
    sensor_gyro_s gyro;
    if (_sensor_gyro_sub.update(&gyro)) {
        // 应用校准
        const Vector3f gyro_corrected = _gyro_calibration.Correct(Vector3f{gyro.xyz});

        // 转换为角度增量
        const Vector3f delta_angle = gyro_corrected * gyro.dt;

        // 积分器累加
        _gyro_integrator.put(delta_angle, gyro.dt);

        return true;
    }
    return false;
}
```

**第282-333行**: UpdateAccel函数
```cpp
bool VehicleIMU::UpdateAccel()
{
    sensor_accel_s accel;
    if (_sensor_accel_sub.update(&accel)) {
        // 应用校准
        const Vector3f accel_corrected = _accel_calibration.Correct(Vector3f{accel.xyz});

        // 转换为速度增量
        const Vector3f delta_velocity = accel_corrected * accel.dt;

        // 积分器累加
        _accel_integrator.put(delta_velocity, accel.dt);

        return true;
    }
    return false;
}
```

**关键点**：
- VehicleIMU **订阅** `sensor_gyro` 和 `sensor_accel`
- **不订阅** FIFO版本
- 执行积分操作：∫ω(t)dt = Δθ，∫a(t)dt = Δv
- 时间窗口通常为4ms（250Hz）

#### 4.2.2 VotedSensorsUpdate 投票选择

**文件**: `src/modules/sensors/voted_sensors_update.cpp`

**第171-286行**: imuPoll函数（从vehicle_imu提取数据）
```cpp
void VotedSensorsUpdate::imuPoll(sensor_combined_s &raw)
{
    // 从多个vehicle_imu实例中读取数据
    for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
        vehicle_imu_s imu;
        if (_vehicle_imu_sub[i].update(&imu)) {
            // 存储到缓冲区
            _last_sensor_data[i].timestamp = imu.timestamp;
            _last_sensor_data[i].gyro_rad[0] = imu.delta_angle[0] / (imu.delta_angle_dt * 1e-6f);
            _last_sensor_data[i].gyro_rad[1] = imu.delta_angle[1] / (imu.delta_angle_dt * 1e-6f);
            _last_sensor_data[i].gyro_rad[2] = imu.delta_angle[2] / (imu.delta_angle_dt * 1e-6f);

            _last_sensor_data[i].accelerometer_m_s2[0] = imu.delta_velocity[0] / (imu.delta_velocity_dt * 1e-6f);
            _last_sensor_data[i].accelerometer_m_s2[1] = imu.delta_velocity[1] / (imu.delta_velocity_dt * 1e-6f);
            _last_sensor_data[i].accelerometer_m_s2[2] = imu.delta_velocity[2] / (imu.delta_velocity_dt * 1e-6f);
        }
    }

    // 投票选择最优传感器
    checkFailover(_accel, "Accel", events::px4::enums::sensor_type_t::accel);
    checkFailover(_gyro, "Gyro", events::px4::enums::sensor_type_t::gyro);

    // 将最优传感器数据写入raw
    raw.timestamp = _last_sensor_data[gyro_best_index].timestamp;
    memcpy(&raw.accelerometer_m_s2, &_last_sensor_data[accel_best_index].accelerometer_m_s2, ...);
    memcpy(&raw.gyro_rad, &_last_sensor_data[gyro_best_index].gyro_rad, ...);
}
```

**关键点**：
- VotedSensorsUpdate **订阅** `vehicle_imu`
- 将积分值转换回瞬时值（除以时间窗口）
- 执行投票算法选择最优传感器

---

## 5. 为什么EKF2不使用vehicle_angular_velocity？

### 5.1 根本原因：卡尔曼滤波器的数学要求

EKF2是一个**扩展卡尔曼滤波器（Extended Kalman Filter）**，其数学框架要求：

```
预测步骤（Prediction Step）:
  x̂ₖ₊₁|ₖ = f(x̂ₖ|ₖ, uₖ)        // 状态预测
  Pₖ₊₁|ₖ = FₖPₖ|ₖFₖᵀ + Qₖ      // 协方差预测

更新步骤（Update Step）:
  Kₖ = Pₖ|ₖ₋₁Hₖᵀ(HₖPₖ|ₖ₋₁Hₖᵀ + Rₖ)⁻¹  // 卡尔曼增益
  x̂ₖ|ₖ = x̂ₖ|ₖ₋₁ + Kₖ(zₖ - h(x̂ₖ|ₖ₋₁))   // 状态更新
  Pₖ|ₖ = (I - KₖHₖ)Pₖ|ₖ₋₁               // 协方差更新
```

**关键要求**：
1. **输入uₖ必须是未滤波的原始测量值**
2. **过程噪声Qₖ和测量噪声Rₖ必须准确建模**
3. **滤波器内部进行最优估计，不应使用外部滤波数据**

### 5.2 使用vehicle_angular_velocity的问题

#### 问题1: 噪声特性未知

`vehicle_angular_velocity` 经过复杂的滤波链：
```
原始数据
  ↓ 动态陷波滤波器（频率自适应）
  ↓ 静态陷波滤波器（固定频率）
  ↓ 低通滤波器（截止频率可配置）
vehicle_angular_velocity
```

**后果**：
- 滤波后的噪声协方差矩阵Rₖ **无法准确建模**
- 滤波器引入的相关性破坏了卡尔曼滤波器的白噪声假设
- 导致协方差估计不准确，滤波器性能下降

**文件**: `src/modules/ekf2/EKF/covariance.cpp`

**第102-150行**: 过程噪声协方差计算
```cpp
void Ekf::predictCovariance(const imuSample &imu_delayed)
{
    // 陀螺仪测量噪声方差（假设白噪声）
    const float d_ang_var = sq(dt) * sq(_params.gyro_noise);

    // 加速度计测量噪声方差（假设白噪声）
    const float d_vel_var = sq(dt) * sq(_params.accel_noise);

    // 如果输入是滤波后的数据，这些噪声模型将不再准确
    // ...
}
```

#### 问题2: 相位延迟

滤波器引入的相位延迟：
- 动态陷波滤波器：~2-5ms
- 静态陷波滤波器：~3-8ms
- 低通滤波器：~5-10ms
- **总延迟**：~10-23ms

**后果**：
- 状态预测滞后于真实状态
- 控制系统的反应延迟增加
- 高动态机动时性能下降

**文件**: `src/modules/ekf2/EKF/output_predictor.cpp`

EKF2通过输出预测器实现低延迟输出（~1.5ms），如果输入已经有10-23ms延迟，输出延迟将增加到11.5-24.5ms。

#### 问题3: 丢失高频信息

滤波器截止频率通常为30-80Hz，高频运动信息被滤除。

**影响**：
- 快速机动时跟踪性能下降
- 振动频率的有用信息丢失（用于振动监测）
- 无法检测高频故障模式

#### 问题4: 双重滤波导致过度平滑

```
原始IMU数据
  ↓ VehicleAngularVelocity滤波
vehicle_angular_velocity (已滤波)
  ↓ EKF2滤波（再次滤波）
输出姿态 (过度平滑)
```

**后果**：
- 响应迟钝
- 跟踪误差增大
- 动态性能下降

### 5.3 EKF2的内部滤波策略

EKF2 **自身就是一个最优滤波器**，不需要预先滤波的输入。

**文件**: `src/modules/ekf2/EKF/ekf.cpp`

**第158-195行**: EKF核心更新
```cpp
bool Ekf::update()
{
    if (_imu_updated) {
        const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

        // 1. 协方差预测（包含噪声建模）
        predictCovariance(imu_sample_delayed);

        // 2. 状态预测（使用原始测量值）
        predictState(imu_sample_delayed);

        // 3. 观测融合（卡尔曼更新）
        controlFusionModes(imu_sample_delayed);

        // 4. 输出修正（补偿延迟）
        _output_predictor.correctOutputStates(...);

        return true;
    }
    return false;
}
```

**EKF2的滤波机制**：
1. **预测步骤**：使用运动模型和原始IMU数据预测状态
2. **更新步骤**：融合其他传感器（GPS、磁力计等）修正状态
3. **协方差跟踪**：实时估计状态不确定性
4. **自适应调整**：根据观测质量动态调整卡尔曼增益

这是一个**理论最优的贝叶斯估计器**，前提是输入是未滤波的原始测量值。

---

## 6. sensor_combined vs vehicle_imu

### 6.1 两者的区别

| 特性 | sensor_combined | vehicle_imu |
|------|----------------|-------------|
| 数据源 | 单一（投票后） | 多实例（每个IMU一个） |
| 数据格式 | 瞬时值（rad/s, m/s²） | 积分值（rad, m/s） |
| 时间窗口 | 无明确窗口 | delta_angle_dt, delta_velocity_dt |
| 冗余管理 | 已完成（投票选择） | 未完成（需EKF2Selector） |
| 使用场景 | 单EKF实例模式 | 多EKF实例模式 |

### 6.2 为什么有两种模式？

**单实例模式**（使用sensor_combined）：
- 简单配置
- 传感器冗余由Sensors模块管理
- 单个EKF2实例

**多实例模式**（使用vehicle_imu）：
- 每个IMU一个EKF2实例
- 传感器冗余由EKF2Selector管理
- 更高的可靠性（EKF级别的冗余）

**文件**: `src/modules/ekf2/EKF2.cpp`

**第482-492行**: 模式选择
```cpp
if (!_callback_registered) {
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
    if (_multi_mode) {
        _callback_registered = _vehicle_imu_sub.registerCallback();  // 多实例模式
    } else
#endif
    {
        _callback_registered = _sensor_combined_sub.registerCallback();  // 单实例模式
    }
}
```

### 6.3 积分值 vs 瞬时值的优势

#### 使用积分值（vehicle_imu）的优势：

1. **数值稳定性**
```cpp
// 积分值
delta_theta = ∫[t, t+dt] ω(τ) dτ

// 瞬时值需要再积分
delta_theta ≈ ω(t) * dt  // 矩形积分，精度较低
```

2. **时间对齐**
```cpp
// vehicle_imu确保陀螺仪和加速度计时间对齐
struct vehicle_imu_s {
    uint64_t timestamp_sample;  // 统一的样本时间戳
    float delta_angle[3];       // 对齐的角度增量
    float delta_velocity[3];    // 对齐的速度增量
    uint32_t delta_angle_dt;    // 相同的积分窗口
    uint32_t delta_velocity_dt;
};
```

3. **精确的时间窗口信息**
- EKF需要知道精确的积分时间窗口
- vehicle_imu提供`delta_angle_dt`和`delta_velocity_dt`
- sensor_combined的时间窗口信息不明确

**文件**: `src/modules/ekf2/EKF2.cpp`

**第491-550行**: 多实例模式读取vehicle_imu
```cpp
if (_multi_mode) {
    vehicle_imu_s imu;
    imu_updated = _vehicle_imu_sub.update(&imu);

    if (imu_updated) {
        // 直接使用积分值
        imu_sample_new.delta_ang_dt = imu.delta_angle_dt * 1.e-6f;  // μs → s
        imu_sample_new.delta_ang = Vector3f{imu.delta_angle};        // 已是积分值
        imu_sample_new.delta_vel_dt = imu.delta_velocity_dt * 1.e-6f;
        imu_sample_new.delta_vel = Vector3f{imu.delta_velocity};    // 已是积分值
    }
}
```

**第554-599行**: 单实例模式读取sensor_combined
```cpp
else {
    sensor_combined_s sensor_combined;
    imu_updated = _sensor_combined_sub.update(&sensor_combined);

    if (imu_updated) {
        // 需要将瞬时值转换为积分值
        imu_sample_new.delta_ang_dt = sensor_combined.gyro_integral_dt * 1.e-6f;
        imu_sample_new.delta_ang = Vector3f{sensor_combined.gyro_rad} * imu_sample_new.delta_ang_dt;

        imu_sample_new.delta_vel_dt = sensor_combined.accelerometer_integral_dt * 1.e-6f;
        imu_sample_new.delta_vel = Vector3f{sensor_combined.accelerometer_m_s2} * imu_sample_new.delta_vel_dt;
    }
}
```

**关键差异**：
- vehicle_imu: 直接使用积分值（更准确）
- sensor_combined: 瞬时值 × 时间窗口（近似积分）

---

## 7. sensor_gyro/accel vs vehicle_imu

### 7.1 为什么VehicleIMU订阅sensor_gyro而不是sensor_gyro_fifo？

**文件**: `src/modules/sensors/vehicle_imu/VehicleIMU.cpp`

**第50-75行**: VehicleIMU构造函数
```cpp
VehicleIMU::VehicleIMU(int instance, uint8_t accel_index, uint8_t gyro_index, ...) :
    _sensor_accel_sub(ORB_ID(sensor_accel), accel_index),  // 订阅sensor_accel
    _sensor_gyro_sub(this, ORB_ID(sensor_gyro), gyro_index)  // 订阅sensor_gyro
{
    // 积分器配置
    _accel_integrator.set_reset_interval(_imu_integration_interval_us);
    _gyro_integrator.set_reset_interval(_imu_integration_interval_us);
}
```

**原因**：
1. **简化处理逻辑**
   - sensor_gyro是单个样本，处理简单
   - sensor_gyro_fifo是批量样本，需要展开处理

2. **时间对齐更容易**
   - sensor_gyro和sensor_accel都是单次采样
   - 时间戳对齐更直接

3. **积分器设计**
   - VehicleIMU的积分器设计用于处理单次样本
   - 逐个样本累加，到达目标窗口时输出

### 7.2 VehicleAngularVelocity为什么订阅sensor_gyro_fifo？

**文件**: `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`

**第67-71行**: VehicleAngularVelocity构造函数
```cpp
VehicleAngularVelocity::VehicleAngularVelocity() :
    _sensor_gyro_fifo_sub(this, ORB_ID(sensor_gyro_fifo))  // 订阅FIFO
{
    // 滤波器需要高频数据
}
```

**原因**：
1. **需要高频数据进行滤波**
   - 陷波滤波器需要采样频率 >> 陷波频率
   - sensor_gyro_fifo提供1600Hz数据
   - sensor_gyro只有800Hz

2. **批量处理效率高**
   - FIFO一次提供2-3个样本
   - 减少uORB通信开销

3. **振动频率分析**
   - FFT分析需要高频连续数据
   - FIFO数据包含时序信息

---

## 8. EKF2内部的数据处理流程

### 8.1 从原始数据到状态估计

```
┌─────────────────────────────────────────────────────────────────┐
│  步骤1: 读取原始IMU数据                                          │
└─────────────────────────────────────────────────────────────────┘

EKF2::Run() 读取 vehicle_imu 或 sensor_combined
  ↓
提取: delta_angle (rad), delta_velocity (m/s), dt (μs)

┌─────────────────────────────────────────────────────────────────┐
│  步骤2: 输出预测器实时更新（低延迟路径）                         │
└─────────────────────────────────────────────────────────────────┘

EstimatorInterface::setIMUData()
  ↓
OutputPredictor::calculateOutputStates()
  ├─ 补偿陀螺偏差: delta_angle_corrected = delta_angle - bias
  ├─ 四元数更新: q_new = q_old ⊗ dq(delta_angle_corrected)
  ├─ 速度更新: v_new = v_old + R * delta_vel_corrected + g*dt
  └─ 位置更新: p_new = p_old + v * dt

输出: _output_new.quat_nominal (实时姿态四元数)
  ↓
PublishAttitude() 立即发布
  ↓
vehicle_attitude (延迟 ~1.5ms)

┌─────────────────────────────────────────────────────────────────┐
│  步骤3: IMU数据下采样和缓冲                                      │
└─────────────────────────────────────────────────────────────────┘

ImuDownSampler::update()
  ├─ 累加多个高频样本
  ├─ 平均时间窗口内的数据
  └─ 输出下采样后的IMU样本

_imu_buffer.push()
  ├─ 存入环形缓冲区
  └─ 用于时间延迟补偿

┌─────────────────────────────────────────────────────────────────┐
│  步骤4: EKF核心更新（高精度路径）                                │
└─────────────────────────────────────────────────────────────────┘

Ekf::update()
  ├─ 获取缓冲区最旧数据: imu_delayed = _imu_buffer.get_oldest()
  │
  ├─ 协方差预测: predictCovariance(imu_delayed)
  │   ├─ 计算状态转移雅可比: F = ∂f/∂x
  │   ├─ 计算过程噪声: Q (陀螺噪声 + 加速度噪声)
  │   └─ 更新协方差: P = F*P*F' + Q
  │
  ├─ 状态预测: predictState(imu_delayed)
  │   ├─ 补偿陀螺偏差和地球自转
  │   ├─ 四元数预测: q_delayed = q_delayed ⊗ dq
  │   ├─ 速度预测: v_delayed = v_delayed + R*delta_vel + g*dt
  │   └─ 位置预测: p_delayed = p_delayed + v*dt
  │
  ├─ 观测融合: controlFusionModes(imu_delayed)
  │   ├─ 磁力计融合（如果有）
  │   ├─ GPS融合（如果有）
  │   ├─ 气压计融合（如果有）
  │   └─ 卡尔曼更新: x = x + K*(z - h(x))
  │
  └─ 输出修正: correctOutputStates()
      ├─ 计算输出状态与EKF延迟状态的误差
      ├─ 互补滤波修正输出状态
      └─ 更新偏差修正项

最终输出: 高精度、低延迟的姿态估计
```

### 8.2 关键代码位置汇总

| 功能 | 文件 | 行号 | 说明 |
|------|------|------|------|
| 订阅声明 | `EKF2.hpp` | 397-398 | vehicle_imu/sensor_combined订阅 |
| 模式选择 | `EKF2.cpp` | 482-492 | 多实例/单实例模式 |
| IMU数据读取 | `EKF2.cpp` | 491-599 | 读取并转换为imuSample |
| 输出预测器更新 | `estimator_interface.cpp` | 73-103 | setIMUData入口 |
| 实时姿态计算 | `output_predictor.cpp` | - | calculateOutputStates |
| IMU下采样 | `estimator_interface.cpp` | 89-98 | ImuDownSampler |
| 协方差预测 | `covariance.cpp` | 102+ | predictCovariance |
| 状态预测 | `ekf.cpp` | 257-394 | predictState |
| 观测融合 | `control.cpp` | - | controlFusionModes |
| 输出修正 | `output_predictor.cpp` | - | correctOutputStates |
| 姿态发布 | `EKF2.cpp` | 926-944 | PublishAttitude |

---

## 9. 卡尔曼滤波器的数学基础

### 9.1 为什么需要未滤波的数据

卡尔曼滤波器基于以下假设：

**假设1: 过程模型**
```
xₖ₊₁ = f(xₖ, uₖ) + wₖ
```
其中：
- xₖ: 状态向量
- uₖ: 控制输入（IMU测量值）
- wₖ: 过程噪声，**假设为零均值白噪声**，协方差为Qₖ

**假设2: 测量模型**
```
zₖ = h(xₖ) + vₖ
```
其中：
- zₖ: 观测值（GPS、磁力计等）
- vₖ: 测量噪声，**假设为零均值白噪声**，协方差为Rₖ

**关键点**：
- wₖ和vₖ必须是**白噪声**（不相关、零均值）
- 如果输入uₖ已经被滤波，噪声特性改变，不再是白噪声
- 滤波器性能下降，甚至发散

### 9.2 滤波后数据的噪声特性

**原始IMU噪声**（白噪声）：
```
E[w(t)] = 0
Cov[w(t), w(t+τ)] = σ² δ(τ)  // 仅在τ=0时不为零
```

**经过低通滤波后**：
```
y(t) = ∫ h(t-τ) w(τ) dτ  // h(t)是滤波器脉冲响应

Cov[y(t), y(t+τ)] ≠ 0 for τ ≠ 0  // 不再是白噪声！
```

滤波引入了时间相关性，违反了卡尔曼滤波器的白噪声假设。

### 9.3 EKF2的噪声建模

**文件**: `src/modules/ekf2/EKF/covariance.cpp`

**第102-150行**: 过程噪声协方差
```cpp
void Ekf::predictCovariance(const imuSample &imu_delayed)
{
    const float dt = _dt_ekf_avg;

    // 陀螺仪测量噪声方差（rad²）
    const float d_ang_var = sq(dt) * sq(_params.gyro_noise);

    // 加速度计测量噪声方差（(m/s)²）
    const float d_vel_var = sq(dt) * sq(_params.accel_noise);

    // 陀螺仪偏差过程噪声（rad²）
    const float d_ang_bias_sig = dt * dt * _params.gyro_bias_p_noise;

    // 加速度计偏差过程噪声（(m/s)²）
    const float d_vel_bias_sig = dt * dt * _params.accel_bias_p_noise;

    // 构造过程噪声协方差矩阵Q
    // 这些参数是根据原始IMU噪声特性调整的
    // 如果输入是滤波后的数据，这些参数将不再适用
}
```

**参数配置**（`src/modules/ekf2/EKF2.hpp`）：
```cpp
DEFINE_PARAMETER(ekf2_gyr_noise, 1.5e-2f);   // 陀螺仪噪声 rad/s
DEFINE_PARAMETER(ekf2_acc_noise, 3.5e-1f);   // 加速度计噪声 m/s²
```

这些参数是基于**原始传感器噪声规格**设定的，不适用于滤波后的数据。

---

## 10. 实时控制 vs 状态估计的不同需求

### 10.1 vehicle_angular_velocity的设计目的

**文件**: `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`

VehicleAngularVelocity的目标是为**实时控制器**提供平滑、低噪声的角速度。

**订阅者**: MulticopterRateControl（角速率控制器）

**第103-280行**: MulticopterRateControl使用
```cpp
void MulticopterRateControl::Run()
{
    vehicle_angular_velocity_s angular_velocity;
    if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
        const Vector3f rates{angular_velocity.xyz};  // 滤波后的角速度

        // PID控制器
        Vector3f torque_sp = _rate_control.update(rates, rates_sp, ...);

        // 发布力矩指令
        _vehicle_torque_setpoint_pub.publish(torque_sp);
    }
}
```

**控制器需求**：
- ✅ 低噪声（减少控制抖动）
- ✅ 平滑（避免执行器磨损）
- ✅ 抑制振动（提高稳定性）
- ⚠️ 可接受延迟（10-20ms在控制回路中仍可接受）

### 10.2 EKF2的设计目的

EKF2的目标是提供**最优状态估计**。

**订阅者**: 多种模块（位置控制、导航、数据记录）

**估计器需求**：
- ✅ 最优估计（统计意义上的最优）
- ✅ 低延迟（用于实时决策）
- ✅ 完整信息（包括高频成分）
- ✅ 正确的不确定性估计（协方差矩阵）
- ❌ 不能过度平滑（会丢失动态信息）

### 10.3 两者的哲学差异

| 特性 | vehicle_angular_velocity | EKF2 |
|------|-------------------------|------|
| 设计哲学 | 信号处理（滤波器级联） | 贝叶斯估计（概率推理） |
| 噪声处理 | 频域滤波（去除特定频率） | 时域融合（最小化方差） |
| 延迟容忍 | 可接受10-20ms | 追求最小延迟 |
| 输入要求 | 原始高频数据 | 原始未滤波数据 |
| 输出特性 | 平滑、低噪声 | 最优、低延迟 |
| 数学基础 | 线性时不变滤波器 | 非线性随机滤波器 |

---

## 11. 如果EKF2使用vehicle_angular_velocity会怎样？

### 11.1 理论分析

假设修改EKF2订阅vehicle_angular_velocity：

```cpp
// 假设的修改（不推荐！）
uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
```

**后果1: 协方差估计错误**

```cpp
// covariance.cpp
const float d_ang_var = sq(dt) * sq(_params.gyro_noise);  // 基于原始噪声

// 但实际输入的噪声已经被滤波器改变
// 实际噪声 << 参数设定的噪声
// 导致协方差矩阵P被高估
```

**影响**：
- 卡尔曼增益K过大
- 对观测数据（GPS等）过度信任
- 状态估计受观测噪声影响增大

**后果2: 动态响应下降**

```cpp
// 输入延迟10-20ms
// 输出预测器基于延迟的输入
// 输出姿态延迟增加到11.5-21.5ms
```

**影响**：
- 快速机动时跟踪性能差
- 控制回路稳定性下降
- 可能导致振荡

**后果3: 高频信息丢失**

```cpp
// 低通滤波器截止频率 ~50Hz
// 高于50Hz的运动成分被滤除
```

**影响**：
- 无法检测高频振动
- 快速姿态变化响应滞后
- 振动监测功能失效

### 11.2 实验验证（理论推演）

假设一个快速翻滚机动（300°/s）：

**使用原始数据**：
```
t=0ms:    IMU采样    ω = 300°/s
t=1ms:    EKF处理    角速度估计 = 300°/s
t=1.5ms:  姿态输出   延迟 1.5ms
```

**使用滤波数据**：
```
t=0ms:    IMU采样           ω_raw = 300°/s
t=10ms:   滤波器输出         ω_filtered = 280°/s (相位滞后)
t=11ms:   EKF处理           角速度估计 = 280°/s
t=11.5ms: 姿态输出          延迟 11.5ms

结果:
- 延迟增加10ms
- 角速度低估6.7%
- 姿态跟踪误差积累
```

---

## 12. 总结

### 12.1 核心结论

1. **EKF2订阅原始IMU数据**（sensor_combined/vehicle_imu）
   - 未经滤波处理
   - 满足卡尔曼滤波器的白噪声假设
   - 保留完整的高频信息

2. **不使用vehicle_angular_velocity的原因**
   - 噪声特性改变，协方差建模困难
   - 相位延迟导致动态性能下降
   - 高频信息丢失
   - 双重滤波导致过度平滑

3. **sensor_combined vs vehicle_imu**
   - sensor_combined: 瞬时值，单实例，投票选择
   - vehicle_imu: 积分值，多实例，更高精度

4. **VehicleIMU的作用**
   - 积分原始传感器数据
   - 时间对齐加速度计和陀螺仪
   - 提供精确的时间窗口信息

### 12.2 设计原则

**状态估计器（EKF2）的输入原则**：
- ✅ 使用未滤波的原始测量值
- ✅ 让滤波器内部进行最优估计
- ✅ 正确建模噪声协方差
- ✅ 最小化输入延迟

**实时控制器的输入原则**：
- ✅ 使用滤波后的平滑数据
- ✅ 抑制高频噪声和振动
- ✅ 可接受适度的相位延迟
- ✅ 提高控制稳定性

### 12.3 架构优势

PX4的数据流架构实现了**职责分离**：

```
原始传感器
  ↓
VehicleIMU (积分、对齐)
  ├──→ EKF2 (状态估计，使用原始数据)
  │      ↓
  │   vehicle_attitude (最优估计)
  │      ↓
  │   MulticopterAttitudeControl
  │      ↓
  │   vehicle_rates_setpoint
  │
  └──→ VehicleAngularVelocity (滤波、平滑)
         ↓
      vehicle_angular_velocity (平滑数据)
         ↓
      MulticopterRateControl
         ↓
      vehicle_torque_setpoint
```

**优点**：
- 状态估计和控制各取所需
- 模块化设计，职责清晰
- 可根据需求选择数据源
- 最大化性能和稳定性

---

## 13. 参考资料

### 13.1 相关文档
- 文档11: EKF2姿态估计算法流程详解
- 文档17: 传感器数据流关系修正说明
- 文档05: VehicleAngularVelocity滤波器详解

### 13.2 关键代码文件

| 文件 | 说明 |
|------|------|
| `src/modules/ekf2/EKF2.cpp` | EKF2主模块 |
| `src/modules/ekf2/EKF/ekf.cpp` | EKF核心算法 |
| `src/modules/ekf2/EKF/covariance.cpp` | 协方差预测 |
| `src/modules/sensors/vehicle_imu/VehicleIMU.cpp` | IMU积分处理 |
| `src/modules/sensors/voted_sensors_update.cpp` | 传感器投票 |
| `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` | 角速度滤波 |

---

**文档版本**: v1.0
**创建日期**: 2025
**适用PX4版本**: v1.14+

