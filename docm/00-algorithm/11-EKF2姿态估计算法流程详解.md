# 11-EKF2姿态估计算法流程详解
## 从IMU数据订阅到attitude发布的完整算法分析

---

## 1. 概述

本文档详细梳理PX4 EKF2模块从订阅`vehicle_imu`（或`sensor_combined`）和`vehicle_angular_velocity`数据，到最终发布`vehicle_attitude`消息的完整算法流程。文档精确到代码文件和行号。

### 核心文件列表
- `src/modules/ekf2/EKF2.cpp` - EKF2主模块实现
- `src/modules/ekf2/EKF2.hpp` - EKF2主模块头文件
- `src/modules/ekf2/EKF/estimator_interface.cpp` - 估计器接口实现
- `src/modules/ekf2/EKF/estimator_interface.h` - 估计器接口头文件
- `src/modules/ekf2/EKF/ekf.cpp` - EKF核心算法实现
- `src/modules/ekf2/EKF/ekf.h` - EKF核心算法头文件
- `src/modules/ekf2/EKF/covariance.cpp` - 协方差预测与更新
- `src/modules/ekf2/EKF/output_predictor.cpp` - 输出预测器实现
- `src/modules/ekf2/EKF/output_predictor.h` - 输出预测器头文件

---

## 2. 数据订阅与回调注册

### 2.1 订阅声明（EKF2.hpp）

**文件**: `src/modules/ekf2/EKF2.hpp`

**第383-384行**: 订阅声明
```cpp
uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)};
```

这两个订阅使用`SubscriptionCallbackWorkItem`，意味着当有新数据到达时会自动触发`Run()`函数回调。

### 2.2 回调注册（EKF2.cpp）

**文件**: `src/modules/ekf2/EKF2.cpp`

**第444-460行**: 回调注册逻辑
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

    if (!_callback_registered) {
        ScheduleDelayed(10_ms);
        return;
    }
}
```

- **多实例模式**: 订阅`vehicle_imu`
- **单实例模式**: 订阅`sensor_combined`

---

## 3. IMU数据接收与处理

### 3.1 Run()函数主循环（EKF2.cpp）

**文件**: `src/modules/ekf2/EKF2.cpp`

**第372行**: Run函数入口
```cpp
void EKF2::Run()
```

### 3.2 IMU数据读取

#### 3.2.1 多实例模式（vehicle_imu）

**第491-550行**: 从`vehicle_imu`读取IMU数据
```cpp
if (_multi_mode) {
    const unsigned last_generation = _vehicle_imu_sub.get_last_generation();
    vehicle_imu_s imu;
    imu_updated = _vehicle_imu_sub.update(&imu);  // 读取IMU数据

    if (imu_updated && (_vehicle_imu_sub.get_last_generation() != last_generation + 1)) {
        perf_count(_msg_missed_imu_perf);  // 检测数据丢失
    }

    if (imu_updated) {
        // 提取IMU数据
        imu_sample_new.time_us = imu.timestamp_sample;
        imu_sample_new.delta_ang_dt = imu.delta_angle_dt * 1.e-6f;  // 转换为秒
        imu_sample_new.delta_ang = Vector3f{imu.delta_angle};        // 角度增量(rad)
        imu_sample_new.delta_vel_dt = imu.delta_velocity_dt * 1.e-6f;
        imu_sample_new.delta_vel = Vector3f{imu.delta_velocity};    // 速度增量(m/s)

        // 处理加速度计削波标志
        if (imu.delta_velocity_clipping > 0) {
            imu_sample_new.delta_vel_clipping[0] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_X;
            imu_sample_new.delta_vel_clipping[1] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Y;
            imu_sample_new.delta_vel_clipping[2] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Z;
        }

        imu_dt = imu.delta_angle_dt;

        // 设备ID和校准计数器更新
        if ((_device_id_accel == 0) || (_device_id_gyro == 0)) {
            _device_id_accel = imu.accel_device_id;
            _device_id_gyro = imu.gyro_device_id;
            _accel_calibration_count = imu.accel_calibration_count;
            _gyro_calibration_count = imu.gyro_calibration_count;
        }
    }
}
```

#### 3.2.2 单实例模式（sensor_combined）

**第554-599行**: 从`sensor_combined`读取IMU数据
```cpp
else {
    const unsigned last_generation = _sensor_combined_sub.get_last_generation();
    sensor_combined_s sensor_combined;
    imu_updated = _sensor_combined_sub.update(&sensor_combined);

    if (imu_updated) {
        imu_sample_new.time_us = sensor_combined.timestamp;
        imu_sample_new.delta_ang_dt = sensor_combined.gyro_integral_dt * 1.e-6f;
        imu_sample_new.delta_ang = Vector3f{sensor_combined.gyro_rad} * imu_sample_new.delta_ang_dt;
        imu_sample_new.delta_vel_dt = sensor_combined.accelerometer_integral_dt * 1.e-6f;
        imu_sample_new.delta_vel = Vector3f{sensor_combined.accelerometer_m_s2} * imu_sample_new.delta_vel_dt;

        // 处理加速度计削波
        if (sensor_combined.accelerometer_clipping > 0) {
            imu_sample_new.delta_vel_clipping[0] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_X;
            imu_sample_new.delta_vel_clipping[1] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Y;
            imu_sample_new.delta_vel_clipping[2] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Z;
        }
    }
}
```

### 3.3 IMU数据推入EKF

**第628-633行**: 将IMU数据送入EKF估计器
```cpp
if (imu_updated) {
    const hrt_abstime now = imu_sample_new.time_us;

    // 将IMU数据推送到估计器
    _ekf.setIMUData(imu_sample_new);
    
    // 立即发布姿态（使用输出预测器的四元数）
    PublishAttitude(now);
}
```

**关键点**: 在调用`_ekf.setIMUData()`之后，**立即**调用`PublishAttitude(now)`发布姿态数据。

---

## 4. setIMUData函数详解

### 4.1 EstimatorInterface::setIMUData

**文件**: `src/modules/ekf2/EKF/estimator_interface.cpp`

**第73-103行**: IMU数据处理入口
```cpp
void EstimatorInterface::setIMUData(const imuSample &imu_sample)
{
    // 初始化检查
    if (!_initialised) {
        _initialised = init(imu_sample.time_us);
    }

    _time_latest_us = imu_sample.time_us;

    // 输出观测器始终运行 - 实时姿态预测
    _output_predictor.calculateOutputStates(imu_sample.time_us, 
                                           imu_sample.delta_ang, 
                                           imu_sample.delta_ang_dt, 
                                           imu_sample.delta_vel, 
                                           imu_sample.delta_vel_dt);

    // IMU数据累积和下采样，当新的下采样数据可用时推入缓冲区
    if (_imu_down_sampler.update(imu_sample)) {
        _imu_updated = true;  // 标记IMU数据已更新

        // 推送下采样后的IMU数据到环形缓冲区
        _imu_buffer.push(_imu_down_sampler.getDownSampledImuAndTriggerReset());

        // 获取缓冲区中最旧的数据
        _time_delayed_us = _imu_buffer.get_oldest().time_us;

        // 计算观测数据的最小间隔
        _min_obs_interval_us = (imu_sample.time_us - _time_delayed_us) / (_obs_buffer_length - 1);
    }

#if defined(CONFIG_EKF2_DRAG_FUSION)
    setDragData(imu_sample);
#endif
}
```

**核心功能**:
1. **输出预测器更新**: 实时计算当前时刻的姿态估计
2. **IMU下采样**: 将高频IMU数据下采样到EKF更新频率
3. **数据缓冲**: 将下采样数据存入环形缓冲区，用于时间延迟补偿

---

## 5. 输出预测器（Output Predictor）

### 5.1 calculateOutputStates函数

**文件**: `src/modules/ekf2/EKF/output_predictor.cpp`

这是**实时姿态计算**的核心函数，使用捷联惯导算法（Strapdown INS）。

**主要步骤**:

1. **角速度偏差补偿**
```cpp
// 补偿陀螺仪偏差
Vector3f delta_angle_corrected = delta_angle - _delta_angle_corr;
```

2. **四元数更新（姿态积分）**
```cpp
// 使用角增量更新四元数
const Quatf dq(AxisAnglef{delta_angle_corrected});
_output_new.quat_nominal = (_output_new.quat_nominal * dq).normalized();
_R_to_earth_now = Dcmf(_output_new.quat_nominal);  // 更新旋转矩阵
```

3. **速度更新**
```cpp
// 补偿加速度计偏差并转换到地球坐标系
Vector3f delta_vel_corrected = delta_velocity - _delta_vel_corr;
Vector3f delta_vel_NED = _R_to_earth_now * delta_vel_corrected;

// 补偿重力加速度
delta_vel_NED(2) += CONSTANTS_ONE_G * delta_velocity_dt;

// 更新速度
_output_new.vel += delta_vel_NED;
```

4. **位置更新**
```cpp
// 使用梯形积分更新位置
_output_new.pos += (_output_new.vel * delta_velocity_dt);
```

5. **存入输出缓冲区**
```cpp
_output_buffer.push(_output_new);
_output_vert_buffer.push(_output_vert_new);
```

### 5.2 getQuaternion函数

**文件**: `src/modules/ekf2/EKF/output_predictor.h`

**第96行**: 获取当前时刻的四元数
```cpp
const matrix::Quatf &getQuaternion() const { return _output_new.quat_nominal; }
```

这个四元数是**实时更新的**，代表当前最新时刻的姿态估计。

---

## 6. EKF核心更新循环

虽然姿态在`setIMUData`后立即发布，但EKF的完整更新过程在后续进行。

### 6.1 Ekf::update函数

**文件**: `src/modules/ekf2/EKF/ekf.cpp`

**第158-195行**: EKF主更新函数
```cpp
bool Ekf::update()
{
    // 滤波器初始化检查
    if (!_filter_initialised) {
        _filter_initialised = initialiseFilter();
        if (!_filter_initialised) {
            return false;
        }
    }

    // 仅当缓冲区中的IMU数据已更新时运行滤波器
    if (_imu_updated) {
        _imu_updated = false;

        // 从缓冲区获取最旧的IMU数据（时间延迟补偿）
        const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

        // 执行状态和协方差预测
        predictCovariance(imu_sample_delayed);  // 协方差预测
        predictState(imu_sample_delayed);       // 状态预测

        // 控制观测数据的融合
        controlFusionModes(imu_sample_delayed);

#if defined(CONFIG_EKF2_RANGE_FINDER)
        // 运行地形估计器
        runTerrainEstimator(imu_sample_delayed);
#endif

        // 修正输出状态
        _output_predictor.correctOutputStates(imu_sample_delayed.time_us, 
                                            getGyroBias(), 
                                            getAccelBias(),
                                            _state.quat_nominal,  // EKF估计的四元数
                                            _state.vel, 
                                            _state.pos);

        return true;
    }

    return false;
}
```

**第679行**: 在EKF2.cpp的Run函数中调用
```cpp
if (_ekf.update()) {
    perf_set_elapsed(_ecl_ekf_update_full_perf, hrt_elapsed_time(&ekf_update_start));
    // ... 发布其他估计结果
}
```

---

## 7. 状态预测（State Prediction）

### 7.1 predictState函数

**文件**: `src/modules/ekf2/EKF/ekf.cpp`

**第257-307行**: EKF状态预测
```cpp
void Ekf::predictState(const imuSample &imu_delayed)
{
    // 应用陀螺仪偏差修正
    const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
    Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;

    // 减去地球自转分量
    corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_delayed.delta_ang_dt;

    // 计算增量四元数
    const Quatf dq(AxisAnglef{corrected_delta_ang});

    // 四元数乘法更新姿态
    _state.quat_nominal = (_state.quat_nominal * dq).normalized();
    _R_to_earth = Dcmf(_state.quat_nominal);  // 更新旋转矩阵

    // 计算地球坐标系下的速度增量
    const Vector3f delta_vel_bias_scaled = getAccelBias() * imu_delayed.delta_vel_dt;
    const Vector3f corrected_delta_vel = imu_delayed.delta_vel - delta_vel_bias_scaled;
    const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

    // 保存上一次的速度用于梯形积分
    const Vector3f vel_last = _state.vel;

    // 使用当前姿态计算速度增量
    _state.vel += corrected_delta_vel_ef;

    // 补偿重力加速度
    _state.vel(2) += CONSTANTS_ONE_G * imu_delayed.delta_vel_dt;

    // 通过梯形积分预测位置
    _state.pos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;

    constrainStates();  // 约束状态

    // 计算平均滤波器更新时间
    float input = 0.5f * (imu_delayed.delta_vel_dt + imu_delayed.delta_ang_dt);
    const float filter_update_s = 1e-6f * _params.filter_update_interval_us;
    input = math::constrain(input, 0.5f * filter_update_s, 2.f * filter_update_s);
    _dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;

    // 计算原始角速度向量（用于其他代码）
    if (imu_delayed.delta_ang_dt > 0.25f * _dt_ekf_avg) {
        _ang_rate_delayed_raw = imu_delayed.delta_ang / imu_delayed.delta_ang_dt;
    }
}
```

**关键点**:
- 这里的`_state.quat_nominal`是EKF估计的延迟四元数（对应缓冲区中最旧的IMU数据）
- 考虑了陀螺仪偏差、地球自转等因素
- 姿态更新使用四元数乘法，保证了数值稳定性

---

## 8. 协方差预测（Covariance Prediction）

### 8.1 predictCovariance函数

**文件**: `src/modules/ekf2/EKF/covariance.cpp`

**第102行**: 协方差预测入口
```cpp
void Ekf::predictCovariance(const imuSample &imu_delayed)
{
    // 使用平均更新间隔以减少小时间步长导致的累积误差
    const float dt = _dt_ekf_avg;
    const float dt_inv = 1.f / dt;

    // 陀螺仪偏差过程噪声（rad/s^2 -> rad）
    const float d_ang_bias_sig = dt * dt * math::constrain(_params.gyro_bias_p_noise, 0.0f, 1.0f);

    // 加速度计偏差过程噪声（m/s^3 -> m/s）
    const float d_vel_bias_sig = dt * dt * math::constrain(_params.accel_bias_p_noise, 0.0f, 1.0f);
    
    // ... 更多协方差预测代码
}
```

**主要步骤**:
1. 计算状态转移矩阵雅可比
2. 计算过程噪声协方差矩阵
3. 应用离散时间协方差预测方程：`P = F*P*F' + Q`
4. 更新24x24状态协方差矩阵

协方差矩阵包含以下状态的不确定性：
- 四元数（姿态） - 索引0-3
- 速度 - 索引4-6
- 位置 - 索引7-9
- 陀螺仪偏差 - 索引10-12
- 加速度计偏差 - 索引13-15
- 磁场 - 索引16-21
- 风速 - 索引22-23

---

## 9. 观测融合（Measurement Fusion）

### 9.1 controlFusionModes函数

**文件**: `src/modules/ekf2/EKF/control.cpp`

这个函数控制各种传感器观测数据的融合，包括：
- 磁力计（用于航向估计）
- GPS位置和速度
- 气压计高度
- 光流
- 外部视觉等

融合过程使用扩展卡尔曼滤波器（EKF）的测量更新方程：
```
K = P*H' / (H*P*H' + R)  // 卡尔曼增益
x = x + K*(z - h(x))     // 状态更新
P = (I - K*H)*P          // 协方差更新
```

虽然这些融合会影响状态估计，但**姿态的实时输出主要来自输出预测器**。

---

## 10. 输出修正（Output Correction）

### 10.1 correctOutputStates函数

**文件**: `src/modules/ekf2/EKF/output_predictor.cpp`

**在Ekf::update的最后调用**（ekf.cpp第188行）:
```cpp
_output_predictor.correctOutputStates(imu_sample_delayed.time_us, 
                                     getGyroBias(), 
                                     getAccelBias(),
                                     _state.quat_nominal,  // EKF延迟状态
                                     _state.vel, 
                                     _state.pos);
```

**主要功能**:
1. 计算输出预测器状态与EKF延迟状态之间的差异
2. 应用互补滤波器修正输出状态，使其跟踪EKF状态
3. 更新偏差修正项（`_delta_angle_corr`、`_delta_vel_corr`）

这确保了实时输出姿态与EKF融合后的最优估计保持一致。

---

## 11. 姿态发布（Attitude Publication）

### 11.1 PublishAttitude函数

**文件**: `src/modules/ekf2/EKF2.cpp`

**第926-944行**: 姿态发布函数
```cpp
void EKF2::PublishAttitude(const hrt_abstime &timestamp)
{
    if (_ekf.attitude_valid()) {
        // 生成飞行器姿态四元数数据
        vehicle_attitude_s att;
        att.timestamp_sample = timestamp;
        
        // 从EKF获取四元数（实际是从output_predictor获取）
        _ekf.getQuaternion().copyTo(att.q);

        // 获取四元数重置信息
        _ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);
        
        att.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
        
        // 发布姿态消息
        _attitude_pub.publish(att);

    } else if (_replay_mode) {
        // 回放模式下发布零时间戳的姿态
        vehicle_attitude_s att{};
        _attitude_pub.publish(att);
    }
}
```

### 11.2 getQuaternion追踪

**文件**: `src/modules/ekf2/EKF/estimator_interface.h`

EstimatorInterface类中的getQuaternion方法返回输出预测器的四元数：
```cpp
const matrix::Quatf &getQuaternion() const { 
    return _output_predictor.getQuaternion(); 
}
```

**文件**: `src/modules/ekf2/EKF/output_predictor.h`

**第96行**: 输出预测器返回实时四元数
```cpp
const matrix::Quatf &getQuaternion() const { 
    return _output_new.quat_nominal; 
}
```

---

## 12. 完整数据流图

```
┌─────────────────────────────────────────────────────────────────┐
│                    IMU传感器数据源                                │
│        vehicle_imu (多实例) / sensor_combined (单实例)            │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  EKF2::Run()  (EKF2.cpp:372)                                    │
│  - 订阅回调触发                                                   │
│  - 读取IMU数据 (492-599行)                                       │
│  - 提取delta_ang, delta_vel                                     │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  _ekf.setIMUData(imu_sample_new)  (EKF2.cpp:632)               │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  EstimatorInterface::setIMUData()  (estimator_interface.cpp:73) │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ 1. 输出预测器实时更新 (83行)                               │  │
│  │    _output_predictor.calculateOutputStates()              │  │
│  │    ├─ 补偿陀螺偏差                                         │  │
│  │    ├─ 四元数更新: q = q * dq(delta_ang)                   │  │
│  │    ├─ 速度更新: v += R_earth * delta_vel + g*dt          │  │
│  │    └─ 位置更新: p += v * dt                               │  │
│  │                                                              │  │
│  │ 2. IMU数据下采样和缓冲 (86-98行)                           │  │
│  │    ├─ _imu_down_sampler.update()                          │  │
│  │    ├─ _imu_buffer.push()                                  │  │
│  │    └─ _imu_updated = true                                 │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  PublishAttitude(now)  (EKF2.cpp:633)                          │
│  立即发布姿态 - 使用输出预测器的实时四元数                        │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  EKF2::PublishAttitude()  (EKF2.cpp:926)                       │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ 1. 检查姿态有效性: _ekf.attitude_valid()                  │  │
│  │ 2. 获取四元数: _ekf.getQuaternion().copyTo(att.q)        │  │
│  │    └─> OutputPredictor::getQuaternion()                  │  │
│  │        └─> _output_new.quat_nominal (实时四元数)          │  │
│  │ 3. 获取重置信息: _ekf.get_quat_reset()                   │  │
│  │ 4. 发布: _attitude_pub.publish(att)                      │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
           ┌─────────────────────┐
           │  vehicle_attitude   │  <-- 其他模块订阅此话题
           │   (uORB消息)        │
           └─────────────────────┘

═══════════════════════════════════════════════════════════════════
                  EKF主更新循环（异步，在后台运行）
═══════════════════════════════════════════════════════════════════

┌─────────────────────────────────────────────────────────────────┐
│  _ekf.update()  (EKF2.cpp:679, ekf.cpp:158)                    │
│  - 仅当_imu_updated=true时执行                                   │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  1. 获取缓冲区最旧IMU数据 (ekf.cpp:174)                          │
│     imu_sample_delayed = _imu_buffer.get_oldest()               │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  2. predictCovariance(imu_sample_delayed)  (ekf.cpp:177)       │
│     协方差预测: P = F*P*F' + Q  (covariance.cpp:102)            │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  3. predictState(imu_sample_delayed)  (ekf.cpp:178, 257)       │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ - 补偿陀螺偏差和地球自转                                    │  │
│  │ - 四元数更新: q = q * dq                                   │  │
│  │ - 速度预测: v += R * delta_vel + g*dt                      │  │
│  │ - 位置预测: p += (v_old + v_new) * dt / 2                 │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  4. controlFusionModes(imu_sample_delayed)  (ekf.cpp:181)      │
│     融合各种传感器观测数据：                                      │
│     - 磁力计 (航向)                                              │
│     - GPS (位置/速度)                                            │
│     - 气压计 (高度)                                              │
│     - 光流、视觉等                                               │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  5. correctOutputStates()  (ekf.cpp:188)                       │
│     修正输出预测器状态，使其跟踪EKF最优估计                       │
│     - 计算输出状态与EKF延迟状态的差异                             │
│     - 应用互补滤波器修正                                         │
│     - 更新偏差修正项                                             │
└─────────────────────────────────────────────────────────────────┘
```

---

## 13. 关键算法细节

### 13.1 四元数姿态更新

**数学公式**:
```
q_new = q_old ⊗ dq
```

其中：
- `q_old`: 上一时刻的四元数
- `dq`: 增量四元数，由角度增量计算得到
- `⊗`: 四元数乘法

**角度增量到增量四元数的转换**:
```cpp
const Quatf dq(AxisAnglef{corrected_delta_ang});
```

这使用轴-角表示法构造增量四元数，相当于：
```
dq = [cos(|θ|/2), sin(|θ|/2) * θ/|θ|]
```

### 13.2 时间延迟补偿策略

EKF2使用**双时间线**架构：

1. **实时时间线**（Output Predictor）
   - 使用最新的IMU数据
   - 实时更新姿态、速度、位置
   - 延迟低（~1ms）
   - 精度较低（无传感器融合）

2. **延迟时间线**（EKF Core）
   - 使用缓冲区中最旧的IMU数据
   - 融合多种传感器观测数据
   - 延迟高（~50-200ms，取决于传感器延迟）
   - 精度高（多传感器融合）

3. **互补滤波修正**
   - 计算两条时间线的状态差异
   - 使用互补滤波器逐渐修正实时输出
   - 保证实时输出最终收敛到EKF最优估计

### 13.3 陀螺仪偏差补偿

**在输出预测器中**（实时）:
```cpp
Vector3f delta_angle_corrected = delta_angle - _delta_angle_corr;
```

**在EKF状态预测中**（延迟）:
```cpp
const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;
```

陀螺仪偏差由EKF在线估计（状态索引10-12），并通过互补滤波器传递给输出预测器。

### 13.4 重力补偿

在速度更新时必须补偿重力加速度：
```cpp
// 地球坐标系（NED）
_state.vel(2) += CONSTANTS_ONE_G * imu_delayed.delta_vel_dt;
```

其中：
- `CONSTANTS_ONE_G = 9.80665 m/s²`
- 仅在Z轴（向下为正）添加重力

---

## 14. 性能与时序分析

### 14.1 执行频率

- **IMU采样频率**: 通常1000Hz（1ms周期）
- **输出预测器更新**: 每次IMU数据到达时更新（1000Hz）
- **EKF下采样频率**: 可配置，默认~250Hz（4ms周期）
- **姿态发布频率**: 与IMU频率相同（1000Hz）
- **EKF完整更新**: 与下采样频率相同（~250Hz）

### 14.2 计算复杂度

| 组件 | 复杂度 | 频率 | 说明 |
|------|--------|------|------|
| 输出预测器 | O(1) | 1000Hz | 轻量级四元数和向量运算 |
| 状态预测 | O(n²) | 250Hz | n=24状态维度 |
| 协方差预测 | O(n³) | 250Hz | 24x24矩阵运算 |
| 测量更新 | O(n²m) | 变化 | m=测量维度 |

### 14.3 延迟分析

- **传感器到IMU数据**: ~0.5ms
- **IMU数据到姿态发布**: ~1ms（主要是计算时间）
- **总延迟**: **约1.5ms**（非常低）

EKF融合延迟虽然较高（50-200ms），但通过输出预测器和互补滤波器，**不影响姿态输出的实时性**。

---

## 15. 代码调用链总结

### 15.1 姿态发布快速路径（实时）

```
IMU数据到达
  ↓
EKF2::Run() (EKF2.cpp:372)
  ↓
读取IMU数据 (EKF2.cpp:492-599)
  ↓
_ekf.setIMUData(imu_sample_new) (EKF2.cpp:632)
  ↓
EstimatorInterface::setIMUData() (estimator_interface.cpp:73)
  ↓
_output_predictor.calculateOutputStates() (estimator_interface.cpp:83)
  ├─ 补偿陀螺偏差
  ├─ 四元数更新
  ├─ 速度更新
  └─ 位置更新
  ↓
PublishAttitude(now) (EKF2.cpp:633)
  ↓
EKF2::PublishAttitude() (EKF2.cpp:926)
  ↓
_ekf.getQuaternion() → OutputPredictor::getQuaternion() (output_predictor.h:96)
  ↓
_attitude_pub.publish(att) (EKF2.cpp:936)
  ↓
vehicle_attitude消息发布 ✓
```

**总延迟**: ~1.5ms

### 15.2 EKF完整更新路径（后台）

```
IMU数据缓冲区更新
  ↓
_ekf.update() (EKF2.cpp:679, ekf.cpp:158)
  ↓
imu_sample_delayed = _imu_buffer.get_oldest() (ekf.cpp:174)
  ↓
predictCovariance(imu_sample_delayed) (ekf.cpp:177, covariance.cpp:102)
  ├─ 计算状态转移雅可比矩阵
  ├─ 计算过程噪声协方差
  └─ 更新协方差矩阵: P = F*P*F' + Q
  ↓
predictState(imu_sample_delayed) (ekf.cpp:178, ekf.cpp:257)
  ├─ 补偿陀螺偏差和地球自转
  ├─ 四元数预测
  ├─ 速度预测
  └─ 位置预测
  ↓
controlFusionModes(imu_sample_delayed) (ekf.cpp:181)
  ├─ 磁力计融合（航向）
  ├─ GPS融合（位置/速度）
  ├─ 气压计融合（高度）
  └─ 其他传感器融合
  ↓
_output_predictor.correctOutputStates() (ekf.cpp:188)
  ├─ 计算输出状态与EKF状态的差异
  ├─ 应用互补滤波器修正
  └─ 更新偏差修正项
```

**更新频率**: ~250Hz（每4ms）

---

## 16. 初始化流程

### 16.1 姿态初始化

**文件**: `src/modules/ekf2/EKF/ekf.cpp`

**第235-255行**: 初始姿态估计
```cpp
bool Ekf::initialiseTilt()
{
    const float accel_norm = _accel_lpf.getState().norm();
    const float gyro_norm = _gyro_lpf.getState().norm();

    // 检查加速度计和陀螺仪是否处于静止状态
    if (accel_norm < 0.8f * CONSTANTS_ONE_G ||
        accel_norm > 1.2f * CONSTANTS_ONE_G ||
        gyro_norm > math::radians(15.0f)) {
        return false;
    }

    // 从加速度向量获取初始横滚和俯仰估计（假设飞行器静止）
    const Vector3f gravity_in_body = _accel_lpf.getState().normalized();
    const float pitch = asinf(gravity_in_body(0));
    const float roll = atan2f(-gravity_in_body(1), -gravity_in_body(2));

    // 初始偏航角设为0
    _state.quat_nominal = Quatf{Eulerf{roll, pitch, 0.0f}};
    _R_to_earth = Dcmf(_state.quat_nominal);

    return true;
}
```

**初始化条件**:
1. 加速度模值接近1g（0.8g ~ 1.2g）
2. 角速度小于15°/s
3. 满足上述条件时，利用重力向量估计初始横滚和俯仰
4. 初始偏航角设为0（需要磁力计或GPS航向初始化）

### 16.2 协方差初始化

**文件**: `src/modules/ekf2/EKF/covariance.cpp`

**第53-100行**: 初始协方差设置
```cpp
void Ekf::initialiseCovariance()
{
    P.zero();

    const float dt = _dt_ekf_avg;

    resetQuatCov();  // 四元数协方差

    // 速度不确定性
    P(4,4) = sq(fmaxf(_params.gps_vel_noise, 0.01f));
    P(5,5) = P(4,4);
    P(6,6) = sq(1.5f) * P(4,4);

    // 位置不确定性
    P(7,7) = sq(fmaxf(_params.gps_pos_noise, 0.01f));
    P(8,8) = P(7,7);
    P(9,9) = sq(fmaxf(_params.baro_noise, 0.01f));

    // 陀螺仪偏差不确定性
    _prev_delta_ang_bias_var(0) = P(10,10) = sq(_params.switch_on_gyro_bias * dt);
    _prev_delta_ang_bias_var(1) = P(11,11) = P(10,10);
    _prev_delta_ang_bias_var(2) = P(12,12) = P(10,10);

    // 加速度计偏差不确定性
    _prev_dvel_bias_var(0) = P(13,13) = sq(_params.switch_on_accel_bias * dt);
    _prev_dvel_bias_var(1) = P(14,14) = P(13,13);
    _prev_dvel_bias_var(2) = P(15,15) = P(13,13);

    resetMagCov();  // 磁场协方差

    // 风速不确定性
    P(22,22) = sq(_params.initial_wind_uncertainty);
    P(23,23) = P(22,22);
}
```

---

## 17. 相关参数配置

### 17.1 EKF2主要参数

| 参数名 | 默认值 | 说明 | 代码位置 |
|--------|--------|------|----------|
| EKF2_PREDICT_US | 10000 | EKF预测间隔(μs)，对应100Hz | EKF2.hpp:458 |
| EKF2_GYR_NOISE | 1.5e-2 | 陀螺仪过程噪声(rad/s) | EKF2.hpp:473 |
| EKF2_ACC_NOISE | 3.5e-1 | 加速度计过程噪声(m/s²) | EKF2.hpp:475 |
| EKF2_GYR_B_NOISE | 1.0e-3 | 陀螺仪偏差过程噪声(rad/s²) | EKF2.hpp:479 |
| EKF2_ACC_B_NOISE | 1.0e-2 | 加速度计偏差过程噪声(m/s³) | EKF2.hpp:481 |
| EKF2_GBIAS_INIT | 0.1 | 初始陀螺仪偏差不确定性(rad/s) | EKF2.hpp:676 |
| EKF2_ABIAS_INIT | 0.2 | 初始加速度计偏差不确定性(m/s²) | EKF2.hpp:678 |
| EKF2_ANGERR_INIT | 0.1 | 初始姿态误差(rad) | EKF2.hpp:680 |
| EKF2_TAU_VEL | 0.25 | 速度互补滤波器时间常数(s) | EKF2.hpp:671 |
| EKF2_TAU_POS | 0.25 | 位置互补滤波器时间常数(s) | EKF2.hpp:673 |

### 17.2 参数调整建议

**高动态环境**（竞速无人机）:
- 增大`EKF2_GYR_NOISE`和`EKF2_ACC_NOISE`
- 减小`EKF2_TAU_VEL`和`EKF2_TAU_POS`（更快的修正）

**低噪声环境**（固定翼巡航）:
- 减小`EKF2_GYR_NOISE`和`EKF2_ACC_NOISE`
- 增大`EKF2_TAU_VEL`和`EKF2_TAU_POS`（更平滑的输出）

---

## 18. 常见问题与调试

### 18.1 姿态漂移

**可能原因**:
1. 陀螺仪偏差估计不准确
2. 磁力计干扰导致航向漂移
3. IMU校准不良

**调试方法**:
```cpp
// 查看陀螺仪偏差估计
Vector3f gyro_bias = _ekf.getGyroBias();

// 查看加速度计偏差估计
Vector3f accel_bias = _ekf.getAccelBias();

// 查看协方差
float gyro_bias_var = _ekf.getGyroBiasVariance();
```

### 18.2 姿态振荡

**可能原因**:
1. 互补滤波器时间常数过小
2. IMU噪声过大
3. 过程噪声参数设置不当

**解决方案**:
- 增大`EKF2_TAU_VEL`和`EKF2_TAU_POS`
- 检查IMU硬件和隔振
- 适当增大`EKF2_GYR_NOISE`和`EKF2_ACC_NOISE`

### 18.3 姿态延迟

**症状**: 姿态响应滞后于实际运动

**检查点**:
1. IMU数据频率是否足够高
2. EKF更新频率（`EKF2_PREDICT_US`）
3. CPU负载是否过高

---

## 19. 总结

### 19.1 关键特性

1. **低延迟**: 通过输出预测器实现~1.5ms的姿态输出延迟
2. **高精度**: 通过EKF融合多传感器数据提高估计精度
3. **时间同步**: 双时间线架构和互补滤波器保证实时性和准确性
4. **鲁棒性**: 处理传感器延迟、丢帧、噪声等异常情况

### 19.2 算法亮点

1. **Output Predictor设计**: 解决了EKF固有的延迟问题
2. **互补滤波修正**: 平衡了实时性和精确性
3. **IMU下采样**: 降低计算负担同时保持高频输出
4. **四元数表示**: 避免万向节锁，数值稳定

### 19.3 适用场景

- 多旋翼无人机姿态控制
- 固定翼飞行器导航
- 地面机器人定位
- 任何需要低延迟高精度姿态估计的系统

---

## 20. 参考资料

### 20.1 相关论文

1. A. Khosravian et al., "Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements"
   - 互补滤波器时间延迟补偿的理论基础

2. PX4 ECL Library Documentation
   - `src/modules/ekf2/EKF/documentation/`

### 20.2 代码文档

- **输出预测器**: `src/modules/ekf2/EKF/documentation/Output Predictor.pdf`
- **过程和观测模型**: `src/modules/ekf2/EKF/documentation/Process and Observation Models.pdf`

### 20.3 重要代码文件

| 文件 | 行数 | 说明 |
|------|------|------|
| `EKF2.cpp` | 2826 | EKF2主模块 |
| `ekf.cpp` | ~600 | EKF核心算法 |
| `covariance.cpp` | ~600 | 协方差预测与更新 |
| `estimator_interface.cpp` | ~400 | 估计器接口 |
| `output_predictor.cpp` | ~400 | 输出预测器 |

---

**文档版本**: v1.0  
**创建日期**: 2025  
**适用PX4版本**: v1.14+  
**维护者**: PX4开发团队

---


