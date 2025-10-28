# 15-uORB主题中心数据流图
## PX4 多旋翼 IMU 到电机输出的完整 uORB 主题网络

---

## 概述

本文档以 **uORB 主题（消息）** 为中心，详细梳理 PX4 多旋翼从 IMU 到电机输出的完整数据流。与文档 09 不同的是，这里将每个 uORB 主题作为核心节点，展示其发布者和订阅者模块。

**文档视角**：
- 文档 09：以**模块**为中心（模块之间的数据流）
- 本文档：以 **uORB 主题**为中心（主题的生产者和消费者）

---

## 一、完整主题网络拓扑图

### 1.1 主题流向总览

```
                        ┌──────────────────────┐
                        │  sensor_gyro_fifo    │
                        │  (原始 FIFO 数据)     │
                        │  发布: IMU 驱动       │
                        │  频率: 800 Hz         │
                        └──────┬───────────────┘
                               │
                ┌──────────────┼──────────────┐
                │              │              │
                ↓              ↓              ↓
     ┌──────────────┐  ┌─────────────┐  ┌──────────────┐
     │ vehicle_imu  │  │ vehicle_    │  │sensor_       │
     │              │  │ angular_    │  │combined      │
     │ 发布: Vehicle│  │ velocity    │  │              │
     │   IMU        │  │             │  │发布: Sensors │
     │ 频率: 265 Hz │  │发布: Vehicle│  │频率: 265 Hz  │
     └──────┬───────┘  │  Angular    │  └──────┬───────┘
            │          │  Velocity   │         │
            │          │频率: 667 Hz │         │
            │          └──────┬──────┘         │
            │                 │                │
            └────────┐        │        ┌───────┘
                     ↓        │        ↓
              ┌──────────────────────────┐
              │   vehicle_attitude       │
              │   (姿态估计)              │
              │   发布: EKF2             │
              │   频率: 193 Hz           │
              └──────────┬───────────────┘
                         │
                         ↓
              ┌──────────────────────────┐
              │ vehicle_rates_setpoint   │
              │ (角速率设定值)            │
              │ 发布: mc_att_control     │
              │ 频率: 193 Hz             │
              └──────────┬───────────────┘
                         │
                         ↓ (mc_rate_control订阅)
              ┌──────────────────────────┐
              │vehicle_torque_setpoint   │
              │vehicle_thrust_setpoint   │
              │ 发布: mc_rate_control    │
              │ 频率: 667 Hz             │
              └──────────┬───────────────┘
                         │
                         ↓
              ┌──────────────────────────┐
              │   actuator_motors        │
              │   (电机控制输出)          │
              │   发布: control_allocator│
              │   频率: 667 Hz           │
              └──────────┬───────────────┘
                         │
                         ↓
              ┌──────────────────────────┐
              │   电机 PWM/DShot 输出    │
              └──────────────────────────┘
```

---

## 二、主题详细信息

### 主题 1：sensor_gyro_fifo

#### 2.1.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `sensor_gyro_fifo` |
| **消息类型** | `sensor_gyro_fifo_s` |
| **发布频率** | 800 Hz (BMI088), 666.7 Hz (BMI270) |
| **实例数量** | 多实例（每个陀螺仪一个） |
| **工作队列** | wq:SPI2 |

#### 2.1.2 发布者

| 模块名称 | 文件位置 | 实例 | 频率 | 发布函数 |
|---------|---------|------|------|---------|
| **bmi088_gyro** | `src/drivers/imu/bosch/bmi088/BMI088_Gyroscope.cpp` | Instance 0 | 666.7 Hz | `_px4_gyro.updateFIFO()` ~450行 |
| **bmi088_accel** | `src/drivers/imu/bosch/bmi088/BMI088_Accelerometer.cpp` | Instance 0 | 800 Hz | `_px4_accel.updateFIFO()` |
| **bmi270** | `src/drivers/imu/bosch/bmi270/BMI270.cpp` | Instance 1 | 800 Hz | `_px4_gyro.updateFIFO()` 844行 |

**关键代码**：
```cpp
// BMI270.cpp:732-850
bool BMI270::FIFORead(const hrt_abstime &timestamp_sample, uint16_t fifo_bytes)
{
    sensor_gyro_fifo_s gyro_buffer{};
    gyro_buffer.timestamp_sample = timestamp_sample;
    gyro_buffer.dt = FIFO_SAMPLE_DT;  // 625 μs (1600 Hz)
    gyro_buffer.samples = samples_count;

    // 填充 FIFO 数据
    for (int i = 0; i < samples; i++) {
        gyro_buffer.x[i] = raw_data_x;
        gyro_buffer.y[i] = raw_data_y;
        gyro_buffer.z[i] = raw_data_z;
    }

    // 发布
    _px4_gyro.updateFIFO(gyro_buffer);  // → sensor_gyro_fifo
}
```

#### 2.1.3 订阅者

| 模块名称 | 文件位置 | 订阅实例 | 用途 | 订阅函数 |
|---------|---------|---------|------|---------|
| **VehicleIMU** | `src/modules/sensors/vehicle_imu/VehicleIMU.cpp` | 根据 device_id | IMU 数据积分 | `_sensor_gyro_fifo_sub.update()` 169行 |
| **VehicleAngularVelocity** | `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` | 根据传感器选择 | 角速度滤波 | `_sensor_gyro_fifo_sub.update()` 828行 |

**订阅代码示例**：
```cpp
// VehicleAngularVelocity.cpp:828
sensor_gyro_fifo_s sensor_fifo_data;
while (_sensor_gyro_fifo_sub.update(&sensor_fifo_data)) {
    // 处理 FIFO 数据
    for (int axis = 0; axis < 3; axis++) {
        float filtered = FilterAngularVelocity(axis, data, N);
    }
}
```

#### 2.1.4 数据结构

```cpp
struct sensor_gyro_fifo_s {
    uint64_t timestamp;           // 微秒
    uint64_t timestamp_sample;    // 第一个样本的时间戳
    uint32_t device_id;           // 设备 ID
    float dt;                     // 样本间隔 (500 μs 或 625 μs)
    float scale;                  // 缩放因子 (rad/s per LSB)
    uint8_t samples;              // FIFO 中的样本数量 (2-3)
    int16_t x[32];                // 原始 X 轴数据
    int16_t y[32];                // 原始 Y 轴数据
    int16_t z[32];                // 原始 Z 轴数据
};
```

---

### 主题 2：vehicle_imu

#### 2.2.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `vehicle_imu` |
| **消息类型** | `vehicle_imu_s` |
| **发布频率** | 265 Hz (Instance 0), 214 Hz (Instance 1) |
| **实例数量** | 多实例（每个 IMU 一个） |
| **工作队列** | wq:INS0, wq:INS1 |

#### 2.2.2 发布者

| 模块名称 | 文件位置 | 实例 | 频率 | 发布函数 |
|---------|---------|------|------|---------|
| **VehicleIMU** | `src/modules/sensors/vehicle_imu/VehicleIMU.cpp` | Instance 0 (INS0) | 265 Hz | `_vehicle_imu_pub.publish()` ~240行 |
| **VehicleIMU** | `src/modules/sensors/vehicle_imu/VehicleIMU.cpp` | Instance 1 (INS1) | 214 Hz | `_vehicle_imu_pub.publish()` ~240行 |

**关键代码**：
```cpp
// VehicleIMU.cpp:169-600
void VehicleIMU::Run()
{
    sensor_gyro_fifo_s sensor_gyro_fifo;
    if (_sensor_gyro_fifo_sub.update(&sensor_gyro_fifo)) {

        // 积分计算 delta_angle 和 delta_velocity
        for (int n = 0; n < samples; n++) {
            _imu_down_sampler.update(
                Vector3f{gyro_data} * dt,    // delta_angle
                Vector3f{accel_data} * dt    // delta_velocity
            );
        }

        // 发布 vehicle_imu
        vehicle_imu_s imu;
        imu.timestamp = now;
        imu.timestamp_sample = timestamp_sample;
        imu.delta_angle = _imu_down_sampler.delta_angle();
        imu.delta_velocity = _imu_down_sampler.delta_velocity();
        imu.delta_angle_dt = dt_sum;
        imu.delta_velocity_dt = dt_sum;

        _vehicle_imu_pub.publish(imu);
    }
}
```

#### 2.2.3 订阅者

| 模块名称 | 文件位置 | 订阅实例 | 用途 | 订阅函数 |
|---------|---------|---------|------|---------|
| **EKF2** | `src/modules/ekf2/EKF2.cpp` | Instance 0 或 1 | 状态估计 | `_vehicle_imu_sub.update()` ~550行 |
| **Sensors** | `src/modules/sensors/sensors.cpp` | All instances | 传感器融合 | `_vehicle_imu_sub[i].update()` |
| **Logger** | `src/modules/logger/` | All instances | 日志记录 | - |

**订阅代码示例**：
```cpp
// EKF2.cpp:435-864
void EKF2::Run()
{
    vehicle_imu_s imu;
    if (_vehicle_imu_sub.update(&imu)) {

        // 更新 IMU 样本到 EKF 缓冲区
        UpdateIMUSample(imu);

        // 调用 EKF 核心算法
        if (_ekf.update()) {
            PublishAttitude(now);           // → vehicle_attitude
            PublishLocalPosition(now);      // → vehicle_local_position
        }
    }
}
```

#### 2.2.4 数据结构

```cpp
struct vehicle_imu_s {
    uint64_t timestamp;           // 微秒
    uint64_t timestamp_sample;    // 样本时间戳
    uint32_t accel_device_id;     // 加速度计设备 ID
    uint32_t gyro_device_id;      // 陀螺仪设备 ID
    float delta_angle[3];         // 角度增量 (rad)
    float delta_velocity[3];      // 速度增量 (m/s)
    uint32_t delta_angle_dt;      // 角度积分时间 (μs)
    uint32_t delta_velocity_dt;   // 速度积分时间 (μs)
    uint8_t delta_velocity_clipping; // 削波标志
    uint8_t calibration_count;    // 校准计数
};
```

---

### 主题 3：sensor_combined

#### 2.3.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `sensor_combined` |
| **消息类型** | `sensor_combined_s` |
| **发布频率** | ~1000 Hz (与主 IMU 同步) |
| **实例数量** | 单实例 |
| **工作队列** | wq:nav_and_controllers |

#### 2.3.2 发布者

| 模块名称 | 文件位置 | 频率 | 发布函数 |
|---------|---------|------|---------|
| **Sensors** | `src/modules/sensors/sensors.cpp` | ~1000 Hz | `_sensor_pub.publish()` 601行 |

**关键代码**：
```cpp
// sensors.cpp:596-603
void Sensors::Run()
{
    // 调用投票传感器更新
    _voted_sensors_update.sensorsPoll(_sensor_combined);

    // 发布 sensor_combined
    if (_sensor_combined.timestamp != _sensor_combined_prev_timestamp) {
        _voted_sensors_update.setRelativeTimestamps(_sensor_combined);
        _sensor_pub.publish(_sensor_combined);  // ← 发布点
        _sensor_combined_prev_timestamp = _sensor_combined.timestamp;
    }
}
```

#### 2.3.3 订阅者

| 模块名称 | 文件位置 | 用途 | 订阅方式 |
|---------|---------|------|---------|
| **EKF2** (单实例模式) | `src/modules/ekf2/EKF2.cpp` | 状态估计（备用） | `_sensor_combined_sub.update()` |
| **LocalPositionEstimator** | `src/modules/local_position_estimator/` | 本地位置估计 | 轮询订阅 |
| **AttitudeEstimatorQ** | `src/modules/attitude_estimator_q/` | 姿态估计（备用） | 轮询订阅 |
| **Logger** | `src/modules/logger/` | 日志记录 | - |
| **MSP OSD** | `src/drivers/osd/msp_osd/` | OSD 显示 | - |

**订阅代码示例**：
```cpp
// EKF2.cpp (单实例模式)
sensor_combined_s sensor_combined;
if (_sensor_combined_sub.update(&sensor_combined)) {

    imuSample imu_sample_new;
    imu_sample_new.time_us = sensor_combined.timestamp;
    imu_sample_new.delta_ang = Vector3f(sensor_combined.gyro_rad) * dt;
    imu_sample_new.delta_vel = Vector3f(sensor_combined.accelerometer_m_s2) * dt;

    _ekf.setIMUData(imu_sample_new);
}
```

#### 2.3.4 数据结构

```cpp
struct sensor_combined_s {
    uint64_t timestamp;               // 微秒

    // 陀螺仪数据
    float gyro_rad[3];                // 角速度 (rad/s) - 机体坐标系
    uint32_t gyro_integral_dt;        // 陀螺仪积分时间 (μs)
    uint32_t gyro_calibration_count;  // 陀螺仪校准计数

    // 加速度计数据
    float accelerometer_m_s2[3];     // 加速度 (m/s²) - 机体坐标系
    uint32_t accelerometer_integral_dt;    // 加速度计积分时间 (μs)
    uint32_t accel_calibration_count;      // 加速度计校准计数

    // 削波标志
    uint8_t accelerometer_clipping;   // 加速度计削波标志
    uint8_t gyro_clipping;            // 陀螺仪削波标志

    // 相对时间戳（用于同步）
    int32_t accelerometer_timestamp_relative;
};
```

**说明**：
- `sensor_combined` 是融合后的单一 IMU 数据源
- 由 Sensors 模块通过投票算法从多个 `vehicle_imu` 实例中选择最优的
- 主要用于单 EKF 实例配置（较旧的配置方式）
- 现代 PX4 更倾向于使用多 `vehicle_imu` 实例 + EKF2Selector

---

### 主题 4：vehicle_angular_velocity

#### 2.4.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `vehicle_angular_velocity` |
| **消息类型** | `vehicle_angular_velocity_s` |
| **发布频率** | 667 Hz |
| **实例数量** | 单实例（根据传感器选择） |
| **工作队列** | wq:rate_ctrl |

#### 2.4.2 发布者

| 模块名称 | 文件位置 | 频率 | 发布函数 |
|---------|---------|------|---------|
| **VehicleAngularVelocity** | `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` | 667 Hz | `_vehicle_angular_velocity_pub.publish()` 940行 |

**关键代码**：
```cpp
// VehicleAngularVelocity.cpp:784-919
void VehicleAngularVelocity::Run()
{
    sensor_gyro_fifo_s sensor_fifo_data;
    while (_sensor_gyro_fifo_sub.update(&sensor_fifo_data)) {

        // 应用滤波器链
        for (int axis = 0; axis < 3; axis++) {
            // 动态陷波 + 静态陷波 + 低通滤波
            angular_velocity(axis) = FilterAngularVelocity(axis, data, N);
            angular_acceleration(axis) = FilterAngularAcceleration(axis, dt, data, N);
        }

        // 校准并发布
        CalibrateAndPublish(timestamp, angular_velocity, angular_acceleration);
    }
}

// VehicleAngularVelocity.cpp:921-950
void VehicleAngularVelocity::CalibrateAndPublish(...)
{
    vehicle_angular_velocity_s angular_velocity_out;
    angular_velocity_out.timestamp = timestamp;
    angular_velocity_out.timestamp_sample = timestamp_sample;
    angular_velocity_out.xyz[0] = angular_velocity(0);
    angular_velocity_out.xyz[1] = angular_velocity(1);
    angular_velocity_out.xyz[2] = angular_velocity(2);
    angular_velocity_out.xyz_derivative[0] = angular_acceleration(0);
    angular_velocity_out.xyz_derivative[1] = angular_acceleration(1);
    angular_velocity_out.xyz_derivative[2] = angular_acceleration(2);

    _vehicle_angular_velocity_pub.publish(angular_velocity_out);
}
```

#### 2.4.3 订阅者

| 模块名称 | 文件位置 | 用途 | 订阅函数 |
|---------|---------|------|---------|
| **mc_rate_control** | `src/modules/mc_rate_control/MulticopterRateControl.cpp` | 角速率控制 | `_vehicle_angular_velocity_sub.update()` 126行 |
| **EKF2** | `src/modules/ekf2/EKF2.cpp` | 状态估计（辅助） | `_vehicle_angular_velocity_sub.update()` |
| **fw_rate_control** | `src/modules/fw_rate_control/` | 固定翼角速率控制 | - |
| **Logger** | `src/modules/logger/` | 日志记录 | - |

**订阅代码示例**：
```cpp
// MulticopterRateControl.cpp:103-280
void MulticopterRateControl::Run()
{
    vehicle_angular_velocity_s angular_velocity;
    if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

        const Vector3f rates{angular_velocity.xyz};           // 当前角速度
        const Vector3f angular_accel{angular_velocity.xyz_derivative};  // 角加速度

        // 获取角速率设定值
        vehicle_rates_setpoint_s rates_setpoint;
        _vehicle_rates_setpoint_sub.update(&rates_setpoint);

        // PID 控制器
        Vector3f torque_setpoint = _rate_control.update(
            rates, _rates_setpoint, angular_accel, dt, _landed
        );

        // 发布
        _vehicle_torque_setpoint_pub.publish(torque_sp);
    }
}
```

#### 2.4.4 数据结构

```cpp
struct vehicle_angular_velocity_s {
    uint64_t timestamp;           // 微秒
    uint64_t timestamp_sample;    // 样本时间戳
    uint32_t device_id;           // 设备 ID
    float xyz[3];                 // 角速度 (rad/s) - Roll, Pitch, Yaw
    float xyz_derivative[3];      // 角加速度 (rad/s²)
};
```

---

### 主题 5：vehicle_attitude

#### 2.5.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `vehicle_attitude` |
| **消息类型** | `vehicle_attitude_s` |
| **发布频率** | 193 Hz |
| **实例数量** | 单实例（由 EKF2Selector 选择） |
| **工作队列** | wq:INS0 或 wq:INS1 |

#### 2.5.2 发布者

| 模块名称 | 文件位置 | 频率 | 发布函数 |
|---------|---------|------|---------|
| **EKF2** | `src/modules/ekf2/EKF2.cpp` | 193 Hz (Instance 0), 200 Hz (Instance 1) | `PublishAttitude()` ~650行 |
| **EKF2Selector** | `src/modules/ekf2/EKF2Selector.cpp` | 290 Hz | 复制选中实例的姿态 |

**关键代码**：
```cpp
// EKF2.cpp:435-864
void EKF2::Run()
{
    vehicle_imu_s imu;
    if (_vehicle_imu_sub.update(&imu)) {

        UpdateIMUSample(imu);

        if (_ekf.update()) {  // 调用 EKF 核心算法

            // 发布姿态
            PublishAttitude(now);  // ← 发布点
            PublishLocalPosition(now);
            PublishGlobalPosition(now);
        }
    }
}

// EKF2.cpp:~650 (伪代码)
void EKF2::PublishAttitude(hrt_abstime now)
{
    vehicle_attitude_s attitude;
    attitude.timestamp = now;

    // 从 EKF 获取姿态四元数
    _ekf.get_quat().copyTo(attitude.q);

    _attitude_pub.publish(attitude);
}
```

#### 2.5.3 订阅者

| 模块名称 | 文件位置 | 用途 | 订阅函数 |
|---------|---------|------|---------|
| **mc_att_control** | `src/modules/mc_att_control/mc_att_control_main.cpp` | 姿态控制 | `_vehicle_attitude_sub.update()` 243行 |
| **fw_att_control** | `src/modules/fw_att_control/` | 固定翼姿态控制 | - |
| **vtol_att_control** | `src/modules/vtol_att_control/` | VTOL 姿态控制 | - |
| **Logger** | `src/modules/logger/` | 日志记录 | - |
| **MAVLink** | `src/modules/mavlink/` | 遥测发送 | - |

**订阅代码示例**：
```cpp
// mc_att_control_main.cpp:205-400
void MulticopterAttitudeControl::Run()
{
    vehicle_attitude_s v_att;
    if (_vehicle_attitude_sub.update(&v_att)) {

        const Quatf q{v_att.q};  // 当前姿态四元数

        // 获取姿态设定值
        vehicle_attitude_setpoint_s att_sp;
        _vehicle_attitude_setpoint_sub.copy(&att_sp);

        _attitude_control.setAttitudeSetpoint(Quatf(att_sp.q_d), att_sp.yaw_sp_move_rate);

        // PID 控制器（输出角速率设定值）
        Vector3f rates_sp = _attitude_control.update(q);

        // 发布角速率设定值
        vehicle_rates_setpoint_s rates_setpoint;
        rates_setpoint.roll = rates_sp(0);
        rates_setpoint.pitch = rates_sp(1);
        rates_setpoint.yaw = rates_sp(2);
        _vehicle_rates_setpoint_pub.publish(rates_setpoint);
    }
}
```

#### 2.5.4 数据结构

```cpp
struct vehicle_attitude_s {
    uint64_t timestamp;           // 微秒
    uint64_t timestamp_sample;    // 样本时间戳
    float q[4];                   // 姿态四元数 [w, x, y, z]
    float delta_q_reset[4];       // 四元数重置增量
    uint8_t quat_reset_counter;   // 四元数重置计数器
};
```

---

### 主题 6：vehicle_rates_setpoint

#### 2.6.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `vehicle_rates_setpoint` |
| **消息类型** | `vehicle_rates_setpoint_s` |
| **发布频率** | 193 Hz |
| **实例数量** | 单实例 |
| **工作队列** | wq:nav_and_controllers |

#### 2.6.2 发布者

| 模块名称 | 文件位置 | 频率 | 发布函数 |
|---------|---------|------|---------|
| **mc_att_control** | `src/modules/mc_att_control/mc_att_control_main.cpp` | 193 Hz | `_vehicle_rates_setpoint_pub.publish()` ~350行 |
| **fw_att_control** | `src/modules/fw_att_control/` | 变化 | - |
| **vtol_att_control** | `src/modules/vtol_att_control/` | 变化 | - |

**关键代码**：
```cpp
// mc_att_control_main.cpp:205-400
void MulticopterAttitudeControl::Run()
{
    vehicle_attitude_s v_att;
    if (_vehicle_attitude_sub.update(&v_att)) {

        const Quatf q{v_att.q};

        // 姿态 PID 控制器
        Vector3f rates_sp = _attitude_control.update(q);

        // 发布角速率设定值
        vehicle_rates_setpoint_s rates_setpoint;
        rates_setpoint.timestamp = hrt_absolute_time();
        rates_setpoint.roll = rates_sp(0);
        rates_setpoint.pitch = rates_sp(1);
        rates_setpoint.yaw = rates_sp(2);
        rates_setpoint.thrust_body[0] = thrust_body(0);
        rates_setpoint.thrust_body[1] = thrust_body(1);
        rates_setpoint.thrust_body[2] = thrust_body(2);

        _vehicle_rates_setpoint_pub.publish(rates_setpoint);  // ← 发布点
    }
}
```

#### 2.6.3 订阅者

| 模块名称 | 文件位置 | 用途 | 订阅函数 |
|---------|---------|------|---------|
| **mc_rate_control** | `src/modules/mc_rate_control/MulticopterRateControl.cpp` | 角速率控制 | `_vehicle_rates_setpoint_sub.update()` 179行 |
| **fw_rate_control** | `src/modules/fw_rate_control/` | 固定翼角速率控制 | - |
| **Logger** | `src/modules/logger/` | 日志记录 | - |

**订阅代码**：见主题 4 (vehicle_angular_velocity) 的订阅代码示例。

#### 2.6.4 数据结构

```cpp
struct vehicle_rates_setpoint_s {
    uint64_t timestamp;       // 微秒
    float roll;               // Roll 角速率设定值 (rad/s)
    float pitch;              // Pitch 角速率设定值 (rad/s)
    float yaw;                // Yaw 角速率设定值 (rad/s)
    float thrust_body[3];     // 推力设定值 (机体坐标系, N)
};
```

---

### 主题 7：vehicle_torque_setpoint

#### 2.7.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `vehicle_torque_setpoint` |
| **消息类型** | `vehicle_torque_setpoint_s` |
| **发布频率** | 667 Hz |
| **实例数量** | 单实例 |
| **工作队列** | wq:rate_ctrl |

#### 2.7.2 发布者

| 模块名称 | 文件位置 | 频率 | 发布函数 |
|---------|---------|------|---------|
| **mc_rate_control** | `src/modules/mc_rate_control/MulticopterRateControl.cpp` | 667 Hz | `_vehicle_torque_setpoint_pub.publish()` ~250行 |
| **fw_rate_control** | `src/modules/fw_rate_control/` | 变化 | - |

**关键代码**：
```cpp
// MulticopterRateControl.cpp:103-280
void MulticopterRateControl::Run()
{
    vehicle_angular_velocity_s angular_velocity;
    if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

        const Vector3f rates{angular_velocity.xyz};
        const Vector3f angular_accel{angular_velocity.xyz_derivative};

        // 获取角速率设定值
        _vehicle_rates_setpoint_sub.update(&rates_setpoint);

        // PID 控制器
        Vector3f torque_setpoint = _rate_control.update(
            rates, _rates_setpoint, angular_accel, dt, _landed
        );

        // 发布力矩设定值
        vehicle_torque_setpoint_s torque_sp;
        torque_sp.timestamp = hrt_absolute_time();
        torque_sp.timestamp_sample = angular_velocity.timestamp_sample;
        torque_sp.xyz[0] = torque_setpoint(0);
        torque_sp.xyz[1] = torque_setpoint(1);
        torque_sp.xyz[2] = torque_setpoint(2);

        _vehicle_torque_setpoint_pub.publish(torque_sp);  // ← 发布点
    }
}
```

#### 2.7.3 订阅者

| 模块名称 | 文件位置 | 用途 | 订阅函数 |
|---------|---------|------|---------|
| **control_allocator** | `src/modules/control_allocator/ControlAllocator.cpp` | 混控分配 | `_vehicle_torque_setpoint_sub.update()` 360行 |
| **Logger** | `src/modules/logger/` | 日志记录 | - |

**订阅代码示例**：
```cpp
// ControlAllocator.cpp:303-461
void ControlAllocator::Run()
{
    // 订阅力矩设定值
    vehicle_torque_setpoint_s torque_sp;
    if (_vehicle_torque_setpoint_sub.update(&torque_sp)) {
        _torque_sp = Vector3f(torque_sp.xyz);
    }

    // 订阅推力设定值
    vehicle_thrust_setpoint_s thrust_sp;
    if (_vehicle_thrust_setpoint_sub.update(&thrust_sp)) {
        _thrust_sp = Vector3f(thrust_sp.xyz);
    }

    // 组装控制向量
    matrix::Vector<float, NUM_AXES> c;
    c(0) = _torque_sp(0);  // Roll
    c(1) = _torque_sp(1);  // Pitch
    c(2) = _torque_sp(2);  // Yaw
    c(3) = _thrust_sp(0);  // Thrust X
    c(4) = _thrust_sp(1);  // Thrust Y
    c(5) = _thrust_sp(2);  // Thrust Z

    _control_allocation->setControlSetpoint(c);
    _control_allocation->allocate();

    publish_actuator_controls();  // → actuator_motors
}
```

#### 2.7.4 数据结构

```cpp
struct vehicle_torque_setpoint_s {
    uint64_t timestamp;           // 微秒
    uint64_t timestamp_sample;    // 样本时间戳
    float xyz[3];                 // 力矩设定值 (Nm) - Roll, Pitch, Yaw
};
```

---

### 主题 8：vehicle_thrust_setpoint

#### 2.8.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `vehicle_thrust_setpoint` |
| **消息类型** | `vehicle_thrust_setpoint_s` |
| **发布频率** | 667 Hz |
| **实例数量** | 单实例 |
| **工作队列** | wq:rate_ctrl |

#### 2.8.2 发布者

| 模块名称 | 文件位置 | 频率 | 发布函数 |
|---------|---------|------|---------|
| **mc_rate_control** | `src/modules/mc_rate_control/MulticopterRateControl.cpp` | 667 Hz | `_vehicle_thrust_setpoint_pub.publish()` ~260行 |

**关键代码**：
```cpp
// MulticopterRateControl.cpp:220-280
// 在 mc_rate_control 中，推力通常来自姿态设定值
vehicle_thrust_setpoint_s thrust_sp;
thrust_sp.timestamp = hrt_absolute_time();
thrust_sp.timestamp_sample = angular_velocity.timestamp_sample;
thrust_sp.xyz[0] = thrust_body(0);
thrust_sp.xyz[1] = thrust_body(1);
thrust_sp.xyz[2] = thrust_body(2);  // 主要是 Z 轴推力

_vehicle_thrust_setpoint_pub.publish(thrust_sp);
```

#### 2.8.3 订阅者

| 模块名称 | 文件位置 | 用途 | 订阅函数 |
|---------|---------|------|---------|
| **control_allocator** | `src/modules/control_allocator/ControlAllocator.cpp` | 混控分配 | `_vehicle_thrust_setpoint_sub.update()` 393行 |
| **Logger** | `src/modules/logger/` | 日志记录 | - |

**订阅代码**：见主题 7 的订阅代码示例。

#### 2.8.4 数据结构

```cpp
struct vehicle_thrust_setpoint_s {
    uint64_t timestamp;           // 微秒
    uint64_t timestamp_sample;    // 样本时间戳
    float xyz[3];                 // 推力设定值 (N) - X, Y, Z (机体坐标系)
};
```

---

### 主题 9：actuator_motors

#### 2.9.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `actuator_motors` |
| **消息类型** | `actuator_motors_s` |
| **发布频率** | 667 Hz |
| **实例数量** | 单实例 |
| **工作队列** | wq:rate_ctrl |

#### 2.9.2 发布者

| 模块名称 | 文件位置 | 频率 | 发布函数 |
|---------|---------|------|---------|
| **control_allocator** | `src/modules/control_allocator/ControlAllocator.cpp` | 667 Hz | `publish_actuator_controls()` 446行 |

**关键代码**：
```cpp
// ControlAllocator.cpp:303-461
void ControlAllocator::Run()
{
    // 1. 订阅力矩和推力
    // (见主题 7)

    // 2. 混控分配
    _control_allocation->allocate();

    // 3. 发布电机输出
    publish_actuator_controls();
}

// ControlAllocator.cpp:446 (伪代码)
void ControlAllocator::publish_actuator_controls()
{
    actuator_motors_s motors;
    motors.timestamp = hrt_absolute_time();
    motors.timestamp_sample = _timestamp_sample;

    // 获取混控器输出（归一化 [-1, 1]）
    for (int i = 0; i < num_motors; i++) {
        motors.control[i] = _control_allocation->getActuatorOutput(i);
    }

    motors.reversible_flags = _control_allocation->getReversibleFlags();

    _actuator_motors_pub.publish(motors);  // ← 发布点
}
```

#### 2.9.3 订阅者

| 模块名称 | 文件位置 | 用途 | 订阅函数 |
|---------|---------|------|---------|
| **dshot** | `src/drivers/dshot/DShot.cpp` | DShot 电调输出 | `_actuator_motors_sub.update()` |
| **pwm_out** | `src/drivers/pwm_out/PWMOut.cpp` | PWM 电调输出 | `_actuator_motors_sub.update()` |
| **uavcan** | `src/drivers/uavcan/` | UAVCAN 电调输出 | - |
| **Logger** | `src/modules/logger/` | 日志记录 | - |

**订阅代码示例**：
```cpp
// DShot.cpp (伪代码)
void DShot::Run()
{
    actuator_motors_s motors;
    if (_actuator_motors_sub.update(&motors)) {

        for (int i = 0; i < num_motors; i++) {
            // 归一化输出 [-1, 1] → DShot 值 [0, 2047]
            float normalized = motors.control[i];  // -1 到 1
            uint16_t dshot_value = (normalized + 1.0f) * 0.5f * 2047;

            // 发送 DShot 信号到电机
            send_dshot_command(i, dshot_value);
        }
    }
}
```

#### 2.9.4 数据结构

```cpp
struct actuator_motors_s {
    uint64_t timestamp;           // 微秒
    uint64_t timestamp_sample;    // 样本时间戳
    float control[16];            // 电机控制输出（归一化 [-1, 1]）
    uint32_t reversible_flags;    // 可反转标志（位掩码）
};
```

---

## 三、主题订阅关系矩阵

### 3.1 发布-订阅矩阵

| uORB 主题 | 发布者 | 订阅者 | 实例 | 频率 |
|----------|--------|--------|------|------|
| **sensor_gyro_fifo** | bmi088, bmi270 | VehicleIMU, VehicleAngularVelocity | 多 | 800 Hz |
| **sensor_accel_fifo** | bmi088, bmi270 | VehicleIMU, VehicleAcceleration | 多 | 800 Hz |
| **vehicle_imu** | VehicleIMU | EKF2, Sensors, Logger | 多 | 265 Hz |
| **sensor_combined** | Sensors | EKF2 (备用), Logger | 单 | 1000 Hz |
| **vehicle_angular_velocity** | VehicleAngularVelocity | mc_rate_control, EKF2, Logger | 单 | 667 Hz |
| **vehicle_acceleration** | VehicleAcceleration | EKF2, Logger | 单 | 200 Hz |
| **vehicle_attitude** | EKF2 → EKF2Selector | mc_att_control, fw_att_control, Logger | 单 | 193 Hz |
| **vehicle_local_position** | EKF2 → EKF2Selector | mc_pos_control, Navigator, Logger | 单 | 193 Hz |
| **vehicle_attitude_setpoint** | mc_pos_control, Manual | mc_att_control, Logger | 单 | 97 Hz |
| **vehicle_rates_setpoint** | mc_att_control | mc_rate_control, Logger | 单 | 193 Hz |
| **vehicle_torque_setpoint** | mc_rate_control | control_allocator, Logger | 单 | 667 Hz |
| **vehicle_thrust_setpoint** | mc_rate_control | control_allocator, Logger | 单 | 667 Hz |
| **actuator_motors** | control_allocator | dshot, pwm_out, uavcan, Logger | 单 | 667 Hz |
| **actuator_servos** | control_allocator | pwm_out, uavcan, Logger | 单 | 667 Hz |

---

### 3.2 频率层级分组

#### 最高频率层 (800-1000 Hz)
```
sensor_gyro_fifo (800 Hz)
    ↓
sensor_combined (1000 Hz) ← Sensors 模块融合
```

#### 高频率层 (667 Hz) - 内环控制
```
vehicle_angular_velocity (667 Hz)
    ↓
vehicle_torque_setpoint (667 Hz)
vehicle_thrust_setpoint (667 Hz)
    ↓
actuator_motors (667 Hz)
```

#### 中频率层 (193-265 Hz) - 外环控制
```
vehicle_imu (265 Hz)
    ↓
vehicle_attitude (193 Hz)
    ↓
vehicle_rates_setpoint (193 Hz)
```

#### 低频率层 (50-97 Hz) - 位置控制
```
vehicle_local_position (193 Hz)
    ↓
vehicle_attitude_setpoint (97 Hz)
```

---

## 四、完整数据流路径

### 4.1 快速路径（角速率控制）

```
┌────────────────────────────────────────────────────────────────┐
│  快速路径（内环）: ~1 ms 延迟                                   │
└────────────────────────────────────────────────────────────────┘

sensor_gyro_fifo (800 Hz, wq:SPI2)
    │ 发布: bmi088, bmi270
    │ 订阅: VehicleAngularVelocity
    ↓
vehicle_angular_velocity (667 Hz, wq:rate_ctrl)
    │ 发布: VehicleAngularVelocity
    │ 订阅: mc_rate_control
    ↓
                    ┌──────────────────────┐
                    │ mc_rate_control      │
                    │ (同时订阅)            │
                    │ - vehicle_angular_   │
                    │   velocity (实际值)   │
                    │ - vehicle_rates_     │
                    │   setpoint (设定值)   │
                    └──────────┬───────────┘
                               ↓
vehicle_torque_setpoint (667 Hz, wq:rate_ctrl)
vehicle_thrust_setpoint (667 Hz, wq:rate_ctrl)
    │ 发布: mc_rate_control
    │ 订阅: control_allocator
    ↓
actuator_motors (667 Hz, wq:rate_ctrl)
    │ 发布: control_allocator
    │ 订阅: dshot, pwm_out
    ↓
电机 PWM/DShot 输出

总延迟: ~300-500 μs
```

---

### 4.2 完整路径（位置控制）

```
┌────────────────────────────────────────────────────────────────┐
│  完整路径（多环）: ~10-15 ms 延迟                               │
└────────────────────────────────────────────────────────────────┘

sensor_gyro_fifo (800 Hz)
sensor_accel_fifo (800 Hz)
    │ 发布: IMU 驱动
    │ 订阅: VehicleIMU
    ↓
vehicle_imu (265 Hz, wq:INS0/INS1)
    │ 发布: VehicleIMU
    │ 订阅: EKF2, Sensors
    ↓
           ┌──────────────────┐
           │ EKF2             │
           │ (融合多传感器)    │
           │ - vehicle_imu    │
           │ - sensor_gps     │
           │ - sensor_mag     │
           │ - sensor_baro    │
           └─────────┬────────┘
                     ↓
vehicle_attitude (193 Hz, wq:INS0 → EKF2Selector)
vehicle_local_position (193 Hz)
    │ 发布: EKF2 → EKF2Selector
    │ 订阅: mc_att_control, mc_pos_control
    ↓
vehicle_attitude_setpoint (97 Hz, wq:nav_and_ctrl)
    │ 发布: mc_pos_control
    │ 订阅: mc_att_control
    ↓
           ┌──────────────────┐
           │ mc_att_control   │
           │ (姿态外环)        │
           │ - vehicle_       │
           │   attitude       │
           │ - vehicle_       │
           │   attitude_      │
           │   setpoint       │
           └─────────┬────────┘
                     ↓
vehicle_rates_setpoint (193 Hz, wq:nav_and_ctrl)
    │ 发布: mc_att_control
    │ 订阅: mc_rate_control
    ↓
    (接快速路径)

总延迟: ~10-15 ms
```

---

### 4.3 sensor_combined 备用路径

```
┌────────────────────────────────────────────────────────────────┐
│  备用路径（单 EKF 模式）: 仅用于兼容性                           │
└────────────────────────────────────────────────────────────────┘

vehicle_imu (Instance 0, 1, 2, 3)
    │ 发布: VehicleIMU
    │ 订阅: Sensors (投票模块)
    ↓
           ┌──────────────────┐
           │ Sensors          │
           │ (投票算法)        │
           │ - 选择最优 IMU    │
           │ - 融合多传感器    │
           └─────────┬────────┘
                     ↓
sensor_combined (1000 Hz, wq:nav_and_ctrl)
    │ 发布: Sensors
    │ 订阅: EKF2 (单实例模式), LocalPositionEstimator
    ↓
EKF2 (单实例模式)
    ↓
vehicle_attitude
    ↓
    (接完整路径)

说明:
- 现代 PX4 倾向于使用多 vehicle_imu + EKF2Selector
- sensor_combined 主要用于向后兼容
- 日志记录和 OSD 显示仍可能使用此主题
```

---

## 五、工作队列与主题对应

### 5.1 wq:SPI2（传感器数据采集）

**主题**：
- `sensor_gyro_fifo` ← bmi088, bmi270
- `sensor_accel_fifo` ← bmi088, bmi270
- `sensor_mag` ← 磁力计驱动

**特点**：
- 最高优先级
- 800 Hz 采样
- 硬件中断触发

---

### 5.2 wq:INS0 / wq:INS1（惯性导航）

**主题**：
- `vehicle_imu` ← VehicleIMU (Instance 0/1)
- `vehicle_attitude` ← EKF2 (Instance 0/1)
- `vehicle_local_position` ← EKF2 (Instance 0/1)
- `vehicle_global_position` ← EKF2 (Instance 0/1)

**特点**：
- 高优先级
- 193-265 Hz
- IMU 数据处理和状态估计

---

### 5.3 wq:rate_ctrl（角速率控制）

**主题**：
- `vehicle_angular_velocity` ← VehicleAngularVelocity
- `vehicle_torque_setpoint` ← mc_rate_control
- `vehicle_thrust_setpoint` ← mc_rate_control
- `actuator_motors` ← control_allocator

**特点**：
- 最高优先级控制回路
- 667 Hz 同步
- 低延迟（< 2 ms）
- 内环控制

---

### 5.4 wq:nav_and_controllers（导航和外环控制）

**主题**：
- `sensor_combined` ← Sensors
- `vehicle_attitude_setpoint` ← mc_pos_control
- `vehicle_rates_setpoint` ← mc_att_control
- `vehicle_acceleration` ← VehicleAcceleration
- `vehicle_magnetometer` ← VehicleMagnetometer

**特点**：
- 中等优先级
- 50-290 Hz
- 外环控制和传感器融合

---

### 5.5 wq:hp_default（高优先级默认）

**主题**：
- (电机输出订阅 `actuator_motors`)
- FFT 分析

**特点**：
- 执行器输出
- 频谱分析

---

## 六、关键主题性能分析

### 6.1 延迟敏感主题（< 2 ms）

| 主题 | 频率 | 延迟要求 | 工作队列 |
|------|------|---------|---------|
| `vehicle_angular_velocity` | 667 Hz | < 0.5 ms | wq:rate_ctrl |
| `vehicle_torque_setpoint` | 667 Hz | < 0.5 ms | wq:rate_ctrl |
| `actuator_motors` | 667 Hz | < 1 ms | wq:rate_ctrl |

---

### 6.2 高带宽主题（> 500 Hz）

| 主题 | 频率 | 带宽 | 数据量 |
|------|------|------|--------|
| `sensor_gyro_fifo` | 800 Hz | ~200 Hz | ~200 bytes/msg |
| `sensor_combined` | 1000 Hz | ~300 Hz | ~100 bytes/msg |
| `vehicle_angular_velocity` | 667 Hz | ~150 Hz | ~50 bytes/msg |

---

### 6.3 多实例主题

| 主题 | 最大实例数 | 实例选择 |
|------|-----------|---------|
| `sensor_gyro_fifo` | 4 | device_id |
| `sensor_accel_fifo` | 4 | device_id |
| `vehicle_imu` | 4 | IMU instance |
| `sensor_mag` | 4 | device_id |
| `sensor_baro` | 4 | device_id |

---

## 七、主题使用场景

### 7.1 实时控制回路

**使用主题**：
```
vehicle_angular_velocity  → mc_rate_control
vehicle_rates_setpoint    → mc_rate_control
vehicle_torque_setpoint   → control_allocator
actuator_motors           → dshot/pwm_out
```

**要求**：
- 低延迟（< 2 ms）
- 高频率（667 Hz）
- 确定性调度

---

### 7.2 状态估计

**使用主题**：
```
sensor_gyro_fifo    → VehicleIMU
vehicle_imu         → EKF2
sensor_gps          → EKF2
sensor_mag          → EKF2
sensor_baro         → EKF2
    ↓
vehicle_attitude
vehicle_local_position
```

**要求**：
- 数据融合
- 时间同步
- 多传感器冗余

---

### 7.3 日志记录

**使用主题**：几乎所有主题

**订阅者**：Logger 模块

**要求**：
- 不影响实时性
- 完整数据记录
- 时间戳同步

---

### 7.4 遥测发送

**使用主题**：
```
vehicle_attitude
vehicle_local_position
vehicle_global_position
battery_status
sensor_combined (可选)
```

**订阅者**：MAVLink 模块

**要求**：
- 周期性发送（1-50 Hz）
- 数据压缩
- 优先级管理

---

## 八、主题设计模式

### 8.1 单发布者-多订阅者

**示例**：`vehicle_attitude`

```
发布者: EKF2 (单个)
    ↓
订阅者:
    - mc_att_control (控制)
    - Logger (日志)
    - MAVLink (遥测)
    - OSD (显示)
```

**优点**：
- 数据一致性
- 简单清晰
- 性能可预测

---

### 8.2 多发布者-单订阅者（实例选择）

**示例**：`vehicle_imu` → EKF2

```
发布者:
    - VehicleIMU Instance 0
    - VehicleIMU Instance 1
    - VehicleIMU Instance 2
    - VehicleIMU Instance 3
    ↓
订阅者: EKF2 (根据配置选择实例)
```

**优点**：
- 传感器冗余
- 故障隔离
- 灵活配置

---

### 8.3 级联发布-订阅

**示例**：vehicle_imu → EKF2 → mc_att_control

```
sensor_gyro_fifo
    ↓ (VehicleIMU)
vehicle_imu
    ↓ (EKF2)
vehicle_attitude
    ↓ (mc_att_control)
vehicle_rates_setpoint
```

**特点**：
- 串级控制
- 数据流清晰
- 频率递减

---

### 8.4 汇聚模式

**示例**：control_allocator

```
vehicle_torque_setpoint   ──┐
                            ├─→ control_allocator
vehicle_thrust_setpoint   ──┘
    ↓
actuator_motors
```

**特点**：
- 多输入单输出
- 数据同步
- 混控计算

---

## 九、调试和监控

### 9.1 查看主题列表

```bash
# 列出所有主题
uorb top

# 查看特定主题
uorb top vehicle_angular_velocity

# 监听主题数据
listener vehicle_angular_velocity

# 查看主题发布频率
listener vehicle_angular_velocity -r
```

---

### 9.2 查看主题订阅关系

```bash
# 查看所有订阅者
uorb status

# 查看特定主题的订阅者
uorb status vehicle_attitude
```

---

### 9.3 性能分析

```bash
# 查看性能统计
perf

# 查看工作队列性能
work_queue status

# 查看模块性能
top
```

---

## 十、总结

### 10.1 主题层级结构

```
传感器原始数据层:
    sensor_gyro_fifo, sensor_accel_fifo
        ↓
处理数据层:
    vehicle_imu, sensor_combined, vehicle_angular_velocity
        ↓
估计器输出层:
    vehicle_attitude, vehicle_local_position
        ↓
控制设定值层:
    vehicle_attitude_setpoint, vehicle_rates_setpoint
        ↓
执行器输入层:
    vehicle_torque_setpoint, vehicle_thrust_setpoint
        ↓
执行器输出层:
    actuator_motors, actuator_servos
```

---

### 10.2 关键主题对比

| 主题 | sensor_combined | vehicle_imu | vehicle_angular_velocity |
|------|----------------|-------------|-------------------------|
| **用途** | 融合 IMU 数据 | 单个 IMU 数据 | 滤波后角速度 |
| **发布者** | Sensors | VehicleIMU | VehicleAngularVelocity |
| **频率** | 1000 Hz | 265 Hz | 667 Hz |
| **实例** | 单 | 多 | 单 |
| **主要订阅者** | EKF2 (备用) | EKF2 | mc_rate_control |
| **使用场景** | 单 EKF 模式 | 多 EKF 模式 | 实时控制 |

---

### 10.3 数据流优化要点

1. **快速路径优化**
   - vehicle_angular_velocity → actuator_motors
   - 全程在 wq:rate_ctrl
   - 延迟 < 1 ms

2. **多传感器冗余**
   - 多个 sensor_gyro_fifo 实例
   - 多个 vehicle_imu 实例
   - EKF2Selector 选择最优

3. **频率分层**
   - 内环 667 Hz
   - 外环 193 Hz
   - 位置环 97 Hz

4. **工作队列隔离**
   - 传感器采集：wq:SPI2
   - 状态估计：wq:INS0/1
   - 控制回路：wq:rate_ctrl, wq:nav_and_ctrl

---

### 10.4 与文档 09 的关系

| 文档 | 视角 | 核心元素 | 关注点 |
|------|------|---------|--------|
| **文档 09** | 模块中心 | 模块 → 模块 | 代码位置、函数调用 |
| **文档 15** | 主题中心 | 主题 ← 发布者/订阅者 | 数据流、发布订阅关系 |

**互补性**：
- 文档 09：告诉你"谁在哪里做什么"
- 文档 15：告诉你"数据从哪来到哪去"

---

## 参考资料

### 相关文档

- 文档 09：IMU 到电机输出完整数据流分析（模块视角）
- 文档 14：sensor_combined 发布者与数据流详解
- 文档 05：VehicleAngularVelocity 滤波器详解

### 相关文件

| 文件 | 说明 |
|------|------|
| `msg/sensor_gyro_fifo.msg` | sensor_gyro_fifo 消息定义 |
| `msg/vehicle_imu.msg` | vehicle_imu 消息定义 |
| `msg/sensor_combined.msg` | sensor_combined 消息定义 |
| `msg/vehicle_angular_velocity.msg` | vehicle_angular_velocity 消息定义 |
| `msg/vehicle_attitude.msg` | vehicle_attitude 消息定义 |

---

**文档完成** ✓

