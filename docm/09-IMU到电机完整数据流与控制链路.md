# IMU 到电机输出完整数据流分析

## 概述

本文档详细梳理 PX4 多旋翼从 IMU 传感器采集到电机输出的完整数据流路径，包括每个环节所在的工作队列、模块名称、代码位置和关键函数。

**基于 work_queue status 输出分析**

---

## 一、完整数据流概览

### 1.1 数据流图

```
┌────────────────────────────────────────────────────────────────┐
│ [1] IMU 硬件驱动                  工作队列: wq:SPI2            │
│     bmi088_gyro / bmi270                                        │
│     频率: 800 Hz / 666.7 Hz                                    │
├────────────────────────────────────────────────────────────────┤
│ 发布: sensor_gyro_fifo (原始 FIFO 数据)                        │
└────────────────────────────────────────────────────────────────┘
                       ↓
          ┌────────────┴────────────┐
          ↓                         ↓
┌─────────────────────┐   ┌─────────────────────┐
│ [2a] VehicleIMU     │   │ [2b] VehicleAngular │
│  wq:INS0/INS1       │   │      Velocity       │
│  265/214 Hz         │   │  wq:rate_ctrl       │
│                     │   │  667 Hz             │
│ 发布: vehicle_imu   │   │ 发布: vehicle_      │
│                     │   │   angular_velocity  │
└─────────────────────┘   └─────────────────────┘
          ↓                         ↓
┌─────────────────────┐             │
│ [3] EKF2            │             │
│  wq:INS0/INS1       │             │
│  193/200 Hz         │             │
│                     │◄────────────┘
│ 发布: vehicle_      │  (EKF也订阅angular_velocity)
│   attitude,         │
│   local_position    │
└─────────────────────┘
          ↓
┌─────────────────────┐
│ [4] mc_att_control  │
│  wq:nav_and_ctrl    │
│  193 Hz             │
│                     │
│ 发布: vehicle_rates_│
│       setpoint      │
└─────────────────────┘
          ↓
┌─────────────────────┐
│ [5] mc_rate_control │
│  wq:rate_ctrl       │
│  667 Hz             │
│                     │
│ 发布: vehicle_      │
│   torque_setpoint,  │
│   thrust_setpoint   │
└─────────────────────┘
          ↓
┌─────────────────────┐
│ [6] control_        │
│     allocator       │
│  wq:rate_ctrl       │
│  667 Hz             │
│                     │
│ 发布: actuator_     │
│       motors        │
└─────────────────────┘
          ↓
┌─────────────────────┐
│ [7] PWM 输出驱动    │
│  wq:hp_default      │
│  (pwm_out/dshot)    │
│                     │
│ 输出: 电机 PWM/DShot│
└─────────────────────┘
```

---

## 二、详细数据流分析

### [1] IMU 硬件驱动层

#### 工作队列：`wq:SPI2`

**包含的任务**：
```
|__ 2) wq:SPI2
|   |__ 1) bmi088_accel    800.0 Hz  1250 us
|   |__ 2) bmi088_gyro     666.7 Hz  1500 us
|   \__ 3) bmi270          800.0 Hz  1250 us
```

#### 模块 1：bmi088_gyro

**文件位置**：
```
src/drivers/imu/bosch/bmi088/BMI088_Gyroscope.cpp
```

**关键函数**：
| 函数 | 行号 | 作用 |
|------|------|------|
| `RunImpl()` | ~200 | 主循环函数 |
| `FIFORead()` | ~400 | 读取 FIFO 数据 |
| `ProcessGyro()` | ~300 | 处理陀螺仪数据 |
| `_px4_gyro.updateFIFO()` | ~450 | 发布数据 |

**发布主题**：
- `sensor_gyro_fifo` (Instance 0)
  - dt = 500 μs (2000 Hz)
  - samples = 3

**数据流**：
```cpp
// 伪代码流程
RunImpl() {
    // 1. 读取 FIFO 字节数
    uint16_t fifo_bytes = FIFOReadCount();

    // 2. 读取 FIFO 原始数据
    FIFORead(timestamp, fifo_bytes) {
        // 读取 int16 数据
        ProcessGyro(&gyro_buffer, fifo_data);

        // 3. 发布到 uORB
        _px4_gyro.updateFIFO(gyro_buffer);  // → sensor_gyro_fifo
    }
}
```

---

#### 模块 2：bmi270

**文件位置**：
```
src/drivers/imu/bosch/bmi270/BMI270.cpp
```

**关键函数**：
| 函数 | 行号 | 作用 |
|------|------|------|
| `RunImpl()` | 255-475 | 主循环函数 |
| `FIFORead()` | 732-850 | 读取和处理 FIFO |
| `ProcessGyro()` | 697-711 | 处理陀螺仪帧 |
| `ProcessAccel()` | 714-729 | 处理加速度计帧 |

**发布主题**：
- `sensor_gyro_fifo` (Instance 1)
  - dt = 625 μs (1600 Hz，修改后 312.5 μs / 3200 Hz)
  - samples = 2

**核心代码**：
```cpp
// BMI270.cpp:732-850
bool BMI270::FIFORead(const hrt_abstime &timestamp_sample, uint16_t fifo_bytes)
{
    sensor_gyro_fifo_s gyro_buffer{};
    gyro_buffer.timestamp_sample = timestamp_sample;
    gyro_buffer.dt = FIFO_SAMPLE_DT;  // ← dt 来源

    // 解析 FIFO 数据
    while (fifo_buffer_index < fifo_bytes) {
        switch (data_buffer[fifo_buffer_index]) {
            case sensor_gyro_frame:
                ProcessGyro(&gyro_buffer, &data);  // 填充数据
                break;
            // ... 其他帧类型
        }
    }

    // 发布
    _px4_gyro.updateFIFO(gyro_buffer);  // 第 844 行
}
```

---

### [2a] VehicleIMU 模块

#### 工作队列：`wq:INS0` 和 `wq:INS1`

```
|__ 5) wq:INS0
|   |__ 2) vehicle_imu      265.1 Hz  3772 us

|__ 6) wq:INS1
|   |__ 2) vehicle_imu      213.8 Hz  4678 us
```

**说明**：两个实例对应两个 IMU（BMI088 和 BMI270）。

---

**文件位置**：
```
src/modules/sensors/vehicle_imu/VehicleIMU.cpp
src/modules/sensors/vehicle_imu/VehicleIMU.hpp
```

**关键函数**：
| 函数 | 行号 | 作用 |
|------|------|------|
| `Run()` | 169-600 | 主循环 |
| `UpdateIntegratorConfiguration()` | ~350 | 配置积分器 |
| `ProcessIMU()` | ~450 | 处理 IMU 数据 |

**订阅主题**：
- `sensor_gyro_fifo` (根据 device_id 选择实例)
- `sensor_accel_fifo`

**发布主题**：
- `vehicle_imu`
- `vehicle_imu_status`

**核心代码**：
```cpp
// VehicleIMU.cpp:169-600
void VehicleIMU::Run()
{
    // 1. 处理 FIFO 数据
    if (_sensor_gyro_fifo_sub.update(&sensor_gyro_fifo)) {

        // 2. 积分计算 delta_angle 和 delta_velocity
        for (int n = 0; n < samples; n++) {
            _imu_down_sampler.update(
                Vector3f{gyro_data} * dt,
                Vector3f{accel_data} * dt
            );
        }

        // 3. 发布 vehicle_imu
        vehicle_imu_s imu;
        imu.delta_angle = _imu_down_sampler.delta_angle();
        imu.delta_velocity = _imu_down_sampler.delta_velocity();
        _vehicle_imu_pub.publish(imu);
    }
}
```

**数据转换**：
```
sensor_gyro_fifo (int16[] × N samples)
    ↓ 积分
vehicle_imu (delta_angle, delta_velocity)
    ↓
发送给 EKF2
```

---

### [2b] VehicleAngularVelocity 模块

#### 工作队列：`wq:rate_ctrl`

```
|__ 1) wq:rate_ctrl
|   \__ 3) vehicle_angular_velocity  666.8 Hz  1500 us
```

**文件位置**：
```
src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp
src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.hpp
```

**关键函数**：
| 函数 | 行号 | 作用 |
|------|------|------|
| `Run()` | 784-919 | **主循环（已详细文档 05）** |
| `FilterAngularVelocity()` | 725-768 | 应用滤波器链 |
| `FilterAngularAcceleration()` | 770-782 | 计算角加速度 |
| `CalibrateAndPublish()` | 921-950 | 校准并发布 |
| `UpdateDynamicNotchEscRpm()` | 569-664 | 更新 ESC RPM 陷波 |
| `UpdateDynamicNotchFFT()` | 666-723 | 更新 FFT 陷波 |

**订阅主题**：
- `sensor_gyro_fifo` (根据 sensor_selection 选择)

**发布主题**：
- `vehicle_angular_velocity`

**核心代码**：
```cpp
// VehicleAngularVelocity.cpp:784-919
void VehicleAngularVelocity::Run()
{
    // 1. 更新滤波器参数
    ParametersUpdate();
    SensorSelectionUpdate(time_now_us);
    UpdateDynamicNotchEscRpm(time_now_us);  // 第 820 行
    UpdateDynamicNotchFFT(time_now_us);     // 第 821 行

    // 2. 处理 FIFO 数据
    while (_sensor_gyro_fifo_sub.update(&sensor_fifo_data)) {
        for (int axis = 0; axis < 3; axis++) {
            // 3. 应用滤波器链（动态陷波+静态陷波+低通）
            angular_velocity(axis) = FilterAngularVelocity(axis, data, N);  // 第 850 行
            angular_acceleration(axis) = FilterAngularAcceleration(axis, dt, data, N);  // 第 851 行
        }

        // 4. 校准并发布
        CalibrateAndPublish(timestamp, angular_velocity, angular_acceleration);  // 第 856 行
    }
}
```

**数据转换**：
```
sensor_gyro_fifo (原始数据)
    ↓ 动态陷波滤波 (ESC RPM, FFT)
    ↓ 静态陷波滤波
    ↓ 低通滤波
    ↓ 校准 (零偏、缩放、旋转)
vehicle_angular_velocity (角速度 rad/s, 角加速度 rad/s²)
```

---

### [3] EKF2 估计器

#### 工作队列：`wq:INS0` 和 `wq:INS1`

```
|__ 5) wq:INS0
|   |__ 1) ekf2            193.1 Hz  5180 us

|__ 6) wq:INS1
|   |__ 1) ekf2            200.1 Hz  4997 us
```

**说明**：两个 EKF2 实例分别处理两个 IMU。

---

**文件位置**：
```
src/modules/ekf2/EKF2.cpp              (主模块)
src/modules/ekf2/EKF/ekf.cpp           (核心算法)
src/modules/ekf2/EKF2Selector.cpp      (选择器)
```

**关键函数**：

#### EKF2.cpp（主模块）
| 函数 | 行号 | 作用 |
|------|------|------|
| `Run()` | 435-864 | 主循环 |
| `UpdateIMUSample()` | ~600 | 更新 IMU 样本 |
| `_ekf.update()` | 804 | 调用 EKF 核心算法 |
| `PublishLocalPosition()` | ~700 | 发布本地位置 |
| `PublishGlobalPosition()` | ~720 | 发布全局位置 |
| `PublishAttitude()` | ~650 | 发布姿态 |

#### ekf.cpp（核心算法）
| 函数 | 行号 | 作用 |
|------|------|------|
| `update()` | 137-196 | EKF 主更新函数 |
| `predictState()` | ~184 | 状态预测 |
| `predictCovariance()` | ~183 | 协方差预测 |
| `controlFusionModes()` | ~187 | 控制融合模式 |

---

**订阅主题**：
- `vehicle_imu` (Instance 0 或 1)
- `vehicle_angular_velocity`
- `sensor_baro`
- `sensor_gps`
- `sensor_mag`
- 等等

**发布主题**：
- `vehicle_attitude` ⭐
- `vehicle_local_position`
- `vehicle_global_position`
- `estimator_sensor_bias`
- `estimator_status`

---

**核心代码**：
```cpp
// EKF2.cpp:435-864
void EKF2::Run()
{
    // 1. 订阅 vehicle_imu（由 VehicleIMU 发布）
    vehicle_imu_s imu;
    if (_vehicle_imu_sub.update(&imu)) {

        // 2. 更新 IMU 样本到 EKF 缓冲区
        UpdateIMUSample(imu);

        // 3. 调用 EKF 核心算法
        if (_ekf.update()) {  // 第 804 行

            // 4. 发布估计结果
            PublishAttitude(now);           // → vehicle_attitude ⭐
            PublishLocalPosition(now);      // → vehicle_local_position
            PublishGlobalPosition(now);     // → vehicle_global_position
            PublishSensorBias(now);         // → estimator_sensor_bias
        }
    }
}
```

```cpp
// ekf.cpp:137-196
bool Ekf::update()
{
    if (_imu_updated) {
        // 1. 获取延迟补偿后的 IMU 数据
        const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

        // 2. 状态和协方差预测
        predictCovariance(imu_sample_delayed);   // 第 183 行
        predictState(imu_sample_delayed);        // 第 184 行

        // 3. 控制融合模式（GPS/MAG/BARO 等）
        controlFusionModes(imu_sample_delayed);  // 第 187 行

        // 4. 输出预测器校正
        _output_predictor.correctOutputStates(...);  // 第 189 行

        return true;
    }
    return false;
}
```

**数据转换**：
```
vehicle_imu (delta_angle, delta_velocity)
    ↓ EKF 预测
    ↓ 传感器融合 (GPS, MAG, BARO, etc)
    ↓ 状态估计
vehicle_attitude (四元数 q, 欧拉角)
vehicle_local_position (位置, 速度)
```

---

#### EKF2Selector（选择器）

**工作队列**：`wq:nav_and_controllers`

```
|__ 4) wq:nav_and_controllers
|   |__ 1) ekf2_selector    290.3 Hz  3445 us
```

**文件位置**：
```
src/modules/ekf2/EKF2Selector.cpp
```

**关键函数**：
| 函数 | 行号 | 作用 |
|------|------|------|
| `Run()` | 700-735 | 主循环 |
| `SelectInstance()` | ~500 | 选择主 EKF 实例 |

**作用**：
- 从多个 EKF2 实例中选择最优的
- 发布 `estimator_selector_status`
- 复制选中实例的数据到系统主题

---

### [4] 姿态控制器 (mc_att_control)

#### 工作队列：`wq:nav_and_controllers`

```
|__ 4) wq:nav_and_controllers
|   |__ 4) mc_att_control    193.1 Hz  5177 us
```

**文件位置**：
```
src/modules/mc_att_control/mc_att_control_main.cpp
src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
```

**关键函数**：
| 函数 | 行号 | 作用 |
|------|------|------|
| `Run()` | 205-400 | 主循环 |
| `generate_attitude_setpoint()` | 137-202 | 从摇杆生成姿态设定值 |
| `_attitude_control.update()` | 342 | PID 姿态控制 |

**订阅主题**：
- `vehicle_attitude` ⭐ (从 EKF2)
- `vehicle_attitude_setpoint` (从导航/手动输入)

**发布主题**：
- `vehicle_rates_setpoint` ⭐ (角速率设定值)

---

**核心代码**：
```cpp
// mc_att_control_main.cpp:205-400
void MulticopterAttitudeControl::Run()
{
    // 1. 订阅姿态估计（从 EKF2）
    vehicle_attitude_s v_att;
    if (_vehicle_attitude_sub.update(&v_att)) {  // 第 243 行

        const Quatf q{v_att.q};  // 当前姿态四元数

        // 2. 获取姿态设定值
        vehicle_attitude_setpoint_s att_sp;
        _vehicle_attitude_setpoint_sub.copy(&att_sp);

        _attitude_control.setAttitudeSetpoint(Quatf(att_sp.q_d), ...);

        // 3. 姿态 PID 控制器（输出角速率设定值）
        Vector3f rates_sp = _attitude_control.update(q);  // 第 342 行

        // 4. 发布角速率设定值
        vehicle_rates_setpoint_s rates_setpoint;
        rates_setpoint.roll = rates_sp(0);
        rates_setpoint.pitch = rates_sp(1);
        rates_setpoint.yaw = rates_sp(2);
        _vehicle_rates_setpoint_pub.publish(rates_setpoint);  // → 发送给 mc_rate_control
    }
}
```

**数据转换**：
```
vehicle_attitude (当前姿态四元数)
    +
vehicle_attitude_setpoint (期望姿态四元数)
    ↓ PID 控制器
vehicle_rates_setpoint (角速率设定值 rad/s)
```

---

### [5] 角速率控制器 (mc_rate_control)

#### 工作队列：`wq:rate_ctrl`

```
|__ 1) wq:rate_ctrl
|   |__ 2) mc_rate_control    666.8 Hz  1500 us
```

**文件位置**：
```
src/modules/mc_rate_control/MulticopterRateControl.cpp
src/modules/mc_rate_control/RateControl/RateControl.cpp
```

**关键函数**：
| 函数 | 行号 | 作用 |
|------|------|------|
| `Run()` | 103-280 | 主循环 |
| `_rate_control.update()` | 220 | PID 角速率控制 |

**订阅主题**：
- `vehicle_angular_velocity` ⭐ (从 VehicleAngularVelocity)
- `vehicle_rates_setpoint` (从 mc_att_control)

**发布主题**：
- `vehicle_torque_setpoint` ⭐
- `vehicle_thrust_setpoint`

---

**核心代码**：
```cpp
// MulticopterRateControl.cpp:103-280
void MulticopterRateControl::Run()
{
    // 1. 订阅当前角速度（从 VehicleAngularVelocity）
    vehicle_angular_velocity_s angular_velocity;
    if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {  // 第 126 行

        const Vector3f rates{angular_velocity.xyz};  // 当前角速度
        const Vector3f angular_accel{angular_velocity.xyz_derivative};  // 角加速度

        // 2. 获取角速率设定值（从 mc_att_control）
        vehicle_rates_setpoint_s rates_setpoint;
        _vehicle_rates_setpoint_sub.update(&rates_setpoint);  // 第 179 行

        _rates_setpoint(0) = rates_setpoint.roll;
        _rates_setpoint(1) = rates_setpoint.pitch;
        _rates_setpoint(2) = rates_setpoint.yaw;

        // 3. PID 角速率控制器（输出力矩）
        Vector3f torque_setpoint = _rate_control.update(
            rates,              // 当前角速度
            _rates_setpoint,    // 期望角速度
            angular_accel,      // 角加速度（前馈）
            dt,
            _landed
        );  // 第 220 行

        // 4. 发布力矩设定值
        vehicle_torque_setpoint_s torque_sp;
        torque_sp.xyz[0] = torque_setpoint(0);
        torque_sp.xyz[1] = torque_setpoint(1);
        torque_sp.xyz[2] = torque_setpoint(2);
        _vehicle_torque_setpoint_pub.publish(torque_sp);  // → 发送给 control_allocator
    }
}
```

**数据转换**：
```
vehicle_angular_velocity (当前角速度)
    +
vehicle_rates_setpoint (期望角速度)
    ↓ PID 控制器
vehicle_torque_setpoint (力矩 Nm)
vehicle_thrust_setpoint (推力 N)
```

---

### [6] 控制分配器 (control_allocator)

#### 工作队列：`wq:rate_ctrl`

```
|__ 1) wq:rate_ctrl
|   |__ 1) control_allocator    666.8 Hz  1500 us
```

**文件位置**：
```
src/modules/control_allocator/ControlAllocator.cpp
src/modules/control_allocator/ControlAllocation/ControlAllocation.cpp
```

**关键函数**：
| 函数 | 行号 | 作用 |
|------|------|------|
| `Run()` | 303-461 | 主循环 |
| `allocate()` | 432 | 混控矩阵计算 |
| `publish_actuator_controls()` | 446 | 发布执行器控制 |

**订阅主题**：
- `vehicle_torque_setpoint` ⭐ (从 mc_rate_control)
- `vehicle_thrust_setpoint`

**发布主题**：
- `actuator_motors` ⭐
- `actuator_servos`
- `control_allocator_status`

---

**核心代码**：
```cpp
// ControlAllocator.cpp:303-461
void ControlAllocator::Run()
{
    // 1. 订阅力矩和推力设定值
    vehicle_torque_setpoint_s torque_sp;
    vehicle_thrust_setpoint_s thrust_sp;

    if (_vehicle_torque_setpoint_sub.update(&torque_sp)) {  // 第 360 行
        _torque_sp = Vector3f(torque_sp.xyz);
    }

    if (_vehicle_thrust_setpoint_sub.update(&thrust_sp)) {  // 第 393 行
        _thrust_sp = Vector3f(thrust_sp.xyz);
    }

    // 2. 设置控制向量
    matrix::Vector<float, NUM_AXES> c;
    c(0) = _torque_sp(0);  // Roll 力矩
    c(1) = _torque_sp(1);  // Pitch 力矩
    c(2) = _torque_sp(2);  // Yaw 力矩
    c(3) = _thrust_sp(0);  // X 推力
    c(4) = _thrust_sp(1);  // Y 推力
    c(5) = _thrust_sp(2);  // Z 推力

    _control_allocation->setControlSetpoint(c);  // 第 429 行

    // 3. 混控分配（力矩/推力 → 电机输出）
    _control_allocation->allocate();  // 第 432 行

    // 4. 发布执行器控制
    publish_actuator_controls();  // 第 446 行 → actuator_motors
}
```

**混控矩阵**：
```
[电机1输出]   [效率矩阵]   [Roll 力矩  ]
[电机2输出] = [6×N 矩阵] × [Pitch 力矩 ]
[电机3输出]                [Yaw 力矩   ]
[电机4输出]                [X 推力     ]
                          [Y 推力     ]
                          [Z 推力     ]

例如四旋翼 X 构型：
motor1 = +roll +pitch -yaw +thrust
motor2 = -roll +pitch +yaw +thrust
motor3 = -roll -pitch -yaw +thrust
motor4 = +roll -pitch +yaw +thrust
```

**数据转换**：
```
vehicle_torque_setpoint (3轴力矩)
    +
vehicle_thrust_setpoint (3轴推力)
    ↓ 混控矩阵
actuator_motors (N 个电机的归一化输出 [-1, 1])
```

---

### [7] PWM/DShot 输出驱动

#### 工作队列：`wq:hp_default`

```
|__ 7) wq:hp_default
|   |__ 3) dshot         3.3 Hz  299988 us (300000 us)
|   \__ 7) pwm_out      3.3 Hz  299988 us (300000 us)
```

**文件位置**：
```
src/drivers/dshot/DShot.cpp
src/drivers/pwm_out/PWMOut.cpp
```

**订阅主题**：
- `actuator_motors`
- `actuator_armed`

**输出**：
- 硬件 PWM 波形
- 或 DShot 数字信号

**核心功能**：
```cpp
// 伪代码
void DShot::Run()
{
    actuator_motors_s motors;
    if (_actuator_motors_sub.update(&motors)) {

        for (int i = 0; i < num_motors; i++) {
            // 归一化输出 [-1, 1] → DShot 值 [0, 2047]
            uint16_t dshot_value = (motors.control[i] + 1.0f) * 0.5f * 2047;

            // 发送 DShot 信号到电机
            send_dshot_command(i, dshot_value);
        }
    }
}
```

---

## 三、完整数据流时序图

### 3.1 时间序列（基于 667 Hz 控制回路）

```
时间: 0 μs            1500 μs          3000 μs          4500 μs
      │                │                │                │
SPI2: [bmi088读FIFO]    [bmi088读FIFO]    [bmi088读FIFO]
      │ 500μs          │                │
      └─[发布sensor_gyro_fifo]
                       │
INS0:                  [vehicle_imu处理]
                       │ 200μs
                       └─[发布vehicle_imu]
                                        │
INS0:                                   [ekf2更新]
                                        │ 300μs
                                        └─[发布vehicle_attitude]
      │                                               │
rate: [VehicleAngularVelocity]          [VehicleAngularVelocity]
      │ 100μs                            │ 100μs
      └─[发布vehicle_angular_velocity]   └─[发布vehicle_angular_velocity]
                       │                               │
nav:                   [mc_att_control]                [mc_att_control]
                       │ 150μs                          │
                       └─[发布rates_setpoint]
      │                               │                │
rate: [mc_rate_control]               [mc_rate_control]
      │ 80μs                           │ 80μs
      └─[发布torque/thrust_setpoint]   └─[发布setpoint]
      │                               │                │
rate: [control_allocator]             [control_allocator]
      │ 100μs                          │ 100μs
      └─[发布actuator_motors]          └─[发布actuator_motors]
      │                                               │
hp:   [dshot/pwm_out]                                 [dshot/pwm_out]
      └─[输出PWM到电机]                               └─[输出PWM]
```

---

### 3.2 频率层级

```
最高频率: wq:SPI2
  └─ IMU 驱动: 800 Hz (BMI088/BMI270)

高频: wq:rate_ctrl
  ├─ VehicleAngularVelocity: 667 Hz  ← 滤波器
  ├─ mc_rate_control: 667 Hz          ← 内环控制
  └─ control_allocator: 667 Hz        ← 混控

中频: wq:INS0/INS1
  ├─ vehicle_imu: 265/214 Hz          ← IMU 处理
  └─ ekf2: 193/200 Hz                 ← 状态估计

中频: wq:nav_and_controllers
  ├─ ekf2_selector: 290 Hz            ← 选择器
  └─ mc_att_control: 193 Hz           ← 外环控制

低频: wq:hp_default
  ├─ gyro_fft: 332 Hz                 ← FFT 分析
  └─ dshot/pwm_out: 3.3 Hz            ← PWM 更新很慢（实际电机更新更快）
```

**说明**：
- **内环（角速率）**：667 Hz - 快速响应
- **外环（姿态）**：193 Hz - 中速响应
- **估计器**：193-265 Hz - 状态估计

---

## 四、关键代码位置汇总

### 4.1 IMU 数据链

| 模块 | 文件 | 关键函数 | 行号 | 工作队列 | 频率 |
|------|------|---------|------|---------|------|
| **BMI088 陀螺** | `src/drivers/imu/bosch/bmi088/BMI088_Gyroscope.cpp` | `RunImpl()` | ~200 | wq:SPI2 | 667 Hz |
|  | | `FIFORead()` | ~400 | | |
|  | | `_px4_gyro.updateFIFO()` | ~450 | | |
| **BMI270** | `src/drivers/imu/bosch/bmi270/BMI270.cpp` | `RunImpl()` | 255 | wq:SPI2 | 800 Hz |
|  | | `FIFORead()` | 732 | | |
|  | | `ProcessGyro()` | 697 | | |
| **VehicleIMU** | `src/modules/sensors/vehicle_imu/VehicleIMU.cpp` | `Run()` | 169 | wq:INS0/1 | 265/214 Hz |
| **VehicleAngularVelocity** | `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` | `Run()` | 784 | wq:rate_ctrl | 667 Hz |
|  | | `FilterAngularVelocity()` | 725 | | |
|  | | `UpdateDynamicNotchFFT()` | 666 | | |

---

### 4.2 估计器链

| 模块 | 文件 | 关键函数 | 行号 | 工作队列 | 频率 |
|------|------|---------|------|---------|------|
| **EKF2** | `src/modules/ekf2/EKF2.cpp` | `Run()` | 435 | wq:INS0/1 | 193/200 Hz |
|  | | `UpdateIMUSample()` | ~600 | | |
|  | | `_ekf.update()` | 804 | | |
|  | | `PublishAttitude()` | ~650 | | |
| **EKF 核心** | `src/modules/ekf2/EKF/ekf.cpp` | `update()` | 137 | | |
|  | | `predictState()` | 184 | | |
|  | | `predictCovariance()` | 183 | | |
|  | | `controlFusionModes()` | 187 | | |
| **EKF2Selector** | `src/modules/ekf2/EKF2Selector.cpp` | `Run()` | 700 | wq:nav_and_ctrl | 290 Hz |

---

### 4.3 控制器链

| 模块 | 文件 | 关键函数 | 行号 | 工作队列 | 频率 |
|------|------|---------|------|---------|------|
| **姿态控制** | `src/modules/mc_att_control/mc_att_control_main.cpp` | `Run()` | 205 | wq:nav_and_ctrl | 193 Hz |
|  | | `update()` (PID) | 342 | | |
| **角速率控制** | `src/modules/mc_rate_control/MulticopterRateControl.cpp` | `Run()` | 103 | wq:rate_ctrl | 667 Hz |
|  | | `_rate_control.update()` | 220 | | |
| **PID 核心** | `src/modules/mc_rate_control/RateControl/RateControl.cpp` | `update()` | ~80 | | |

---

### 4.4 混控和输出链

| 模块 | 文件 | 关键函数 | 行号 | 工作队列 | 频率 |
|------|------|---------|------|---------|------|
| **混控分配** | `src/modules/control_allocator/ControlAllocator.cpp` | `Run()` | 303 | wq:rate_ctrl | 667 Hz |
|  | | `allocate()` | 432 | | |
|  | | `publish_actuator_controls()` | 446 | | |
| **DShot 输出** | `src/drivers/dshot/DShot.cpp` | `Run()` | ~300 | wq:hp_default | 3.3 Hz⚠️ |
| **PWM 输出** | `src/drivers/pwm_out/PWMOut.cpp` | `Run()` | ~200 | wq:hp_default | 3.3 Hz⚠️ |

**注意**：PWM/DShot 的显示频率 (3.3 Hz) 不准确，实际电机更新频率由混控器触发（~667 Hz）。

---

## 五、工作队列优先级和调度

### 5.1 工作队列分类

根据您的 `work_queue status` 输出：

| 工作队列 | 优先级 | 用途 | 典型频率 |
|---------|--------|------|----------|
| **wq:rate_ctrl** | 最高 | 角速率控制（内环） | 667 Hz |
| **wq:SPI2** | 很高 | 传感器数据采集 | 667-800 Hz |
| **wq:INS0/INS1** | 高 | 状态估计 | 193-265 Hz |
| **wq:nav_and_controllers** | 中 | 导航和姿态控制（外环） | 50-290 Hz |
| **wq:hp_default** | 中低 | 高优先级任务 | 3-332 Hz |
| **wq:lp_default** | 低 | 低优先级任务 | 2-50 Hz |

---

### 5.2 关键路径分析

#### 最快路径（用于角速率控制）

```
IMU → VehicleAngularVelocity → mc_rate_control → control_allocator → 电机
     (667 Hz)                  (667 Hz)          (667 Hz)

总延迟: ~300 μs (最快，低延迟)
```

#### 完整路径（用于位置控制）

```
IMU → VehicleIMU → EKF2 → mc_att_control → mc_rate_control → allocator → 电机
     (265 Hz)    (193 Hz)  (193 Hz)        (667 Hz)         (667 Hz)

总延迟: ~5-10 ms (包含所有环节)
```

---

## 六、主题订阅关系图

### 6.1 主题流向

```
sensor_gyro_fifo (Instance 0: BMI088, Instance 1: BMI270)
    ↓                           ↓
vehicle_imu (Instance 0)    vehicle_angular_velocity
    ↓                           ↓
vehicle_attitude ←──────────────┘ (EKF2 也订阅)
    ↓
vehicle_rates_setpoint
    ↓
vehicle_angular_velocity (实际值)
    ↓
vehicle_torque_setpoint
vehicle_thrust_setpoint
    ↓
actuator_motors
    ↓
电机 PWM/DShot
```

---

### 6.2 订阅者数量分析

从您的 uORB 输出：
```
vehicle_imu             Instance 0:  4 订阅者
vehicle_imu             Instance 1:  4 订阅者
vehicle_angular_velocity:           ? 订阅者（需查看）
vehicle_attitude:                   ? 订阅者
vehicle_rates_setpoint:              4 订阅者
vehicle_torque_setpoint:             3 订阅者
```

**4 个订阅者的含义**（以 vehicle_imu 为例）：
1. EKF2 实例
2. 日志模块（logger）
3. MAVLink（遥测）
4. 可能的调试/监控模块

---

## 七、延迟分析

### 7.1 端到端延迟

**从 IMU 采样到电机输出的总延迟**：

| 环节 | 处理时间 | 频率 | 最大延迟 |
|------|---------|------|---------|
| [1] IMU FIFO 读取 | 50 μs | 800 Hz | 1250 μs |
| [2a] VehicleIMU 处理 | 50 μs | 265 Hz | 3772 μs |
| [2b] VehicleAngularVelocity 滤波 | 100 μs | 667 Hz | 1500 μs |
| [3] EKF2 估计 | 300 μs | 193 Hz | 5180 μs |
| [4] mc_att_control | 150 μs | 193 Hz | 5177 μs |
| [5] mc_rate_control | 80 μs | 667 Hz | 1500 μs |
| [6] control_allocator | 100 μs | 667 Hz | 1500 μs |
| [7] DShot 输出 | 50 μs | 实际很快 | 500 μs |

**最快路径延迟**（VehicleAngularVelocity → 电机）：
```
100 + 80 + 100 + 50 = 330 μs
+ 调度抖动 ~500 μs
= 约 0.8-1 ms
```

**完整路径延迟**（IMU → EKF2 → 电机）：
```
1250 + 3772 + 5180 + 5177 + 1500 + 1500 + 500
= 约 18-20 ms (包含最坏情况)

实际典型延迟: 8-12 ms
```

---

### 7.2 控制回路带宽

根据频率计算控制带宽：

| 控制环 | 频率 | 理论带宽 | 实际带宽 |
|--------|------|---------|----------|
| **角速率环** | 667 Hz | ~200 Hz | ~150 Hz |
| **姿态环** | 193 Hz | ~60 Hz | ~40 Hz |
| **速度环** | 97 Hz | ~30 Hz | ~20 Hz |
| **位置环** | 97 Hz | ~30 Hz | ~15 Hz |

**带宽 ≈ 频率 / 3**（经验值，考虑延迟和相位裕度）

---

## 八、工作队列详细配置

### 8.1 wq:rate_ctrl（最高优先级）

```
|__ 1) wq:rate_ctrl
|   |__ 1) control_allocator         666.8 Hz  1500 us
|   |__ 2) mc_rate_control           666.8 Hz  1500 us
|   \__ 3) vehicle_angular_velocity  666.8 Hz  1500 us
```

**特点**：
- 最高优先级工作队列
- 667 Hz 同步运行
- 角速率控制内环
- 低延迟要求（< 2 ms）

**配置位置**：
```cpp
// VehicleAngularVelocity.cpp:47
ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)

// MulticopterRateControl.cpp
WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)

// ControlAllocator.cpp
ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
```

---

### 8.2 wq:INS0 和 wq:INS1

```
|__ 5) wq:INS0
|   |__ 1) ekf2          193.1 Hz  5180 us
|   \__ 2) vehicle_imu   265.1 Hz  3772 us

|__ 6) wq:INS1
|   |__ 1) ekf2          200.1 Hz  4997 us
|   \__ 2) vehicle_imu   213.8 Hz  4678 us
```

**特点**：
- 两个独立的惯性导航队列
- 每个队列处理一个 IMU
- 支持冗余和故障切换

**为什么有两个**：
- INS0 处理 IMU Instance 0 (BMI088)
- INS1 处理 IMU Instance 1 (BMI270)
- 两个 EKF2 实例独立运行
- EKF2Selector 选择最优结果

---

### 8.3 wq:nav_and_controllers

```
|__ 4) wq:nav_and_controllers
|   |__ 1) ekf2_selector              290.3 Hz  3445 us
|   |__ 4) mc_att_control             193.1 Hz  5177 us
|   |__ 7) sensors                    193.1 Hz  5179 us
|   |__ 8) vehicle_acceleration       200.1 Hz  4997 us
|   |__11) vehicle_magnetometer        47.1 Hz 21240 us
```

**特点**：
- 导航和外环控制
- 中等优先级
- 193 Hz 姿态控制
- 其他传感器融合任务

---

## 九、uORB 主题数据结构

### 9.1 关键主题

| 主题名 | 发布者 | 订阅者 | 频率 | 队列 | 数据结构 |
|--------|--------|--------|------|------|----------|
| `sensor_gyro_fifo` | IMU 驱动 | VehicleIMU, VehicleAngularVelocity | 800 Hz | 4 | int16[32] x/y/z, dt, scale |
| `vehicle_imu` | VehicleIMU | EKF2 | 265 Hz | 8 | delta_angle[3], delta_velocity[3] |
| `vehicle_angular_velocity` | VehicleAngularVelocity | mc_rate_control, EKF2 | 667 Hz | ? | xyz[3], xyz_derivative[3] |
| `vehicle_attitude` | EKF2 | mc_att_control | 193 Hz | ? | q[4], rollspeed, pitchspeed, yawspeed |
| `vehicle_rates_setpoint` | mc_att_control | mc_rate_control | 193 Hz | 4 | roll, pitch, yaw, thrust_body[3] |
| `vehicle_torque_setpoint` | mc_rate_control | control_allocator | 667 Hz | 3 | xyz[3] |
| `actuator_motors` | control_allocator | dshot/pwm_out | 667 Hz | ? | control[N] |

---

### 9.2 数据格式示例

#### sensor_gyro_fifo
```cpp
struct sensor_gyro_fifo_s {
    uint64_t timestamp;
    uint64_t timestamp_sample;
    uint32_t device_id;
    float32 dt;           // 500 μs (BMI088), 625 μs (BMI270)
    float32 scale;        // 0.00106 rad/s per LSB
    uint8 samples;        // 3 (BMI088), 2 (BMI270)
    int16[32] x;          // 原始数据
    int16[32] y;
    int16[32] z;
};
```

#### vehicle_angular_velocity
```cpp
struct vehicle_angular_velocity_s {
    uint64_t timestamp;
    uint64_t timestamp_sample;
    float32 xyz[3];             // 角速度 rad/s
    float32 xyz_derivative[3];  // 角加速度 rad/s²
};
```

#### vehicle_attitude
```cpp
struct vehicle_attitude_s {
    uint64_t timestamp;
    uint64_t timestamp_sample;
    float32 q[4];         // 姿态四元数 [w, x, y, z]
    float32 delta_q_reset[4];
    uint8 quat_reset_counter;
    // 角速度也包含在内（来自 EKF2 估计）
};
```

---

## 十、完整代码调用栈

### 10.1 从 IMU 中断到发布

```cpp
// [硬件中断]
GPIO 中断（BMI270 DRDY）
    ↓
px4_arch_gpiosetevent() callback
    ↓
BMI270::DataReadyInterruptCallback()  // BMI270.cpp:590
    ↓
BMI270::DataReady()                   // BMI270.cpp:597
    ↓
ScheduleNow()  // 触发 RunImpl
    ↓
BMI270::RunImpl()                     // BMI270.cpp:255
    ↓ (STATE::FIFO_READ)
BMI270::FIFORead()                    // BMI270.cpp:732
    ↓
BMI270::ProcessGyro()                 // BMI270.cpp:697
    ↓
_px4_gyro.updateFIFO(gyro_buffer)    // BMI270.cpp:844
    ↓
PX4Gyroscope::updateFIFO()           // PX4Gyroscope.cpp:137
    ↓
_sensor_fifo_pub.publish(sample)     // PX4Gyroscope.cpp:149
    ↓
[uORB] sensor_gyro_fifo 主题发布
```

---

### 10.2 从 sensor_gyro_fifo 到 vehicle_angular_velocity

```cpp
// [uORB 回调触发]
sensor_gyro_fifo 发布
    ↓
VehicleAngularVelocity::Run() 被触发  // VehicleAngularVelocity.cpp:784
    ↓
_sensor_gyro_fifo_sub.update(&sensor_fifo_data)  // 第 828 行
    ↓
for (int axis = 0; axis < 3; axis++) {
    FilterAngularVelocity(axis, data, N);  // 第 850 行
        ↓
    UpdateDynamicNotchEscRpm()  // 第 820 行（如果启用）
    UpdateDynamicNotchFFT()     // 第 821 行（如果启用）
        ↓
    应用陷波和低通滤波器
}
    ↓
CalibrateAndPublish()                  // 第 856 行
    ↓
_vehicle_angular_velocity_pub.publish()  // VehicleAngularVelocity.cpp:940
    ↓
[uORB] vehicle_angular_velocity 主题发布
```

---

### 10.3 从 vehicle_imu 到 vehicle_attitude

```cpp
// [uORB 回调触发]
vehicle_imu 发布
    ↓
EKF2::Run() 被触发                    // EKF2.cpp:435
    ↓
_vehicle_imu_sub.update(&imu)         // 第 ~550 行
    ↓
UpdateIMUSample(imu)                  // 更新 IMU 样本到缓冲区
    ↓
_ekf.update()                         // EKF2.cpp:804
    ↓
Ekf::update()                         // ekf.cpp:137
    ↓
predictState(imu_sample_delayed)      // ekf.cpp:184
predictCovariance(imu_sample_delayed) // ekf.cpp:183
controlFusionModes(imu_sample_delayed)// ekf.cpp:187
    ↓
PublishAttitude(now)                  // EKF2.cpp:~650
    ↓
_vehicle_attitude_pub.publish(attitude)
    ↓
[uORB] vehicle_attitude 主题发布
```

---

### 10.4 从 vehicle_attitude 到 vehicle_rates_setpoint

```cpp
// [uORB 回调触发]
vehicle_attitude 发布
    ↓
MulticopterAttitudeControl::Run()     // mc_att_control_main.cpp:205
    ↓
_vehicle_attitude_sub.update(&v_att)  // 第 243 行
    ↓
const Quatf q{v_att.q};  // 提取姿态四元数
    ↓
_attitude_control.update(q)           // 第 342 行
    ↓ (AttitudeControl.cpp)
计算姿态误差
PID 控制器计算
    ↓ 返回
Vector3f rates_sp  // 角速率设定值
    ↓
vehicle_rates_setpoint.roll = rates_sp(0);
vehicle_rates_setpoint.pitch = rates_sp(1);
vehicle_rates_setpoint.yaw = rates_sp(2);
    ↓
_vehicle_rates_setpoint_pub.publish()
    ↓
[uORB] vehicle_rates_setpoint 主题发布
```

---

### 10.5 从 vehicle_angular_velocity 到 vehicle_torque_setpoint

```cpp
// [uORB 回调触发]
vehicle_angular_velocity 发布
    ↓
MulticopterRateControl::Run()         // MulticopterRateControl.cpp:103
    ↓
_vehicle_angular_velocity_sub.update(&angular_velocity)  // 第 126 行
    ↓
const Vector3f rates{angular_velocity.xyz};  // 当前角速度
    ↓
_vehicle_rates_setpoint_sub.update(&rates_setpoint)  // 第 179 行（获取设定值）
    ↓
Vector3f torque_setpoint = _rate_control.update(
    rates,              // 当前值
    _rates_setpoint,    // 设定值
    angular_accel,      // 前馈
    dt,
    _landed
);  // 第 220 行
    ↓
RateControl::update()  // RateControl.cpp
    计算 PID (P + I + D + FF)
    ↓ 返回
torque_setpoint  // Nm
    ↓
vehicle_torque_setpoint.xyz[0] = torque_setpoint(0);
vehicle_torque_setpoint.xyz[1] = torque_setpoint(1);
vehicle_torque_setpoint.xyz[2] = torque_setpoint(2);
    ↓
_vehicle_torque_setpoint_pub.publish()
    ↓
[uORB] vehicle_torque_setpoint 主题发布
```

---

### 10.6 从 torque_setpoint 到 actuator_motors

```cpp
// [uORB 回调触发]
vehicle_torque_setpoint 发布
    ↓
ControlAllocator::Run()               // ControlAllocator.cpp:303
    ↓
_vehicle_torque_setpoint_sub.update(&torque_sp)  // 第 360 行
_vehicle_thrust_setpoint_sub.update(&thrust_sp)  // 第 393 行
    ↓
// 组装控制向量
c(0) = _torque_sp(0);  // Roll
c(1) = _torque_sp(1);  // Pitch
c(2) = _torque_sp(2);  // Yaw
c(3) = _thrust_sp(0);  // Thrust X
c(4) = _thrust_sp(1);  // Thrust Y
c(5) = _thrust_sp(2);  // Thrust Z
    ↓
_control_allocation->setControlSetpoint(c);  // 第 429 行
    ↓
_control_allocation->allocate();      // 第 432 行
    ↓ (ControlAllocation.cpp)
混控矩阵计算：
    actuator_sp = pseudo_inverse(效率矩阵) × 控制向量
    或：sequential_desaturation()
    ↓
clipActuatorSetpoint()  // 限制到 [-1, 1]
    ↓
publish_actuator_controls()           // 第 446 行
    ↓
actuator_motors_s motors;
for (int i = 0; i < num_motors; i++) {
    motors.control[i] = actuator_sp[i];
}
_actuator_motors_pub.publish(motors);
    ↓
[uORB] actuator_motors 主题发布
```

---

## 十一、频率匹配和同步

### 11.1 为什么 rate_ctrl 队列都是 667 Hz？

**触发源**：`vehicle_angular_velocity` 以 667 Hz 发布

```
VehicleAngularVelocity (667 Hz) 发布
    ↓ 触发回调
mc_rate_control (订阅 vehicle_angular_velocity)
    ↓ 同步运行 667 Hz
    ↓ 发布 torque_setpoint
control_allocator (订阅 torque_setpoint)
    ↓ 同步运行 667 Hz
```

**工作队列的作用**：
- 确保这些任务在同一线程按顺序执行
- 避免竞态条件
- 保证低延迟（无上下文切换）

---

### 11.2 频率不匹配的处理

**问题**：mc_att_control (193 Hz) vs mc_rate_control (667 Hz)

**解决**：

```
mc_att_control (193 Hz):
    发布 vehicle_rates_setpoint
    ↓ 保持最新值
mc_rate_control (667 Hz):
    每次都读取最新的 rates_setpoint
    即使 rates_setpoint 没更新，也用上次的值
    ↓ 结果
    rates_setpoint 缓慢变化（193 Hz）
    torque_setpoint 快速更新（667 Hz）
```

这就是**串级 PID 控制**的经典模式：
- 外环慢（姿态）
- 内环快（角速率）

---

## 十二、性能优化要点

### 12.1 关键路径优化

**优先级 1：wq:rate_ctrl**
- VehicleAngularVelocity
- mc_rate_control
- control_allocator

**优化原则**：
- 最小化处理时间（< 300 μs 总计）
- 避免阻塞操作
- 使用 FIFO 批量处理

---

### 12.2 调度策略

| 队列 | 策略 | 说明 |
|------|------|------|
| wq:rate_ctrl | 回调触发 | 由 vehicle_angular_velocity 发布触发 |
| wq:INS0/1 | 回调触发 | 由 sensor_gyro_fifo 发布触发 |
| wq:SPI2 | 中断 + 定时 | GPIO 中断或定时器 |
| wq:nav_and_ctrl | 回调触发 | 由 vehicle_attitude 等触发 |

---

## 十三、调试和验证

### 13.1 查看完整数据流

```bash
# 1. 查看工作队列状态
work_queue status

# 2. 查看传感器模块
sensors status

# 3. 查看 IMU 数据
listener sensor_gyro_fifo 0  # BMI088
listener sensor_gyro_fifo 1  # BMI270

# 4. 查看角速度
listener vehicle_angular_velocity

# 5. 查看姿态
listener vehicle_attitude

# 6. 查看角速率设定
listener vehicle_rates_setpoint

# 7. 查看力矩输出
listener vehicle_torque_setpoint

# 8. 查看电机输出
listener actuator_motors
```

---

### 13.2 性能分析

```bash
# 查看各模块性能
perf

# 查看 CPU 占用
top

# 查看实时频率
uorb top vehicle_angular_velocity
uorb top vehicle_attitude
uorb top actuator_motors
```

---

### 13.3 延迟测量

通过时间戳计算延迟：

```bash
# 监听并比较时间戳
listener vehicle_angular_velocity -n 1
listener vehicle_torque_setpoint -n 1

# 时间差 = torque.timestamp - angular_velocity.timestamp
# 典型值应 < 2 ms
```

---

## 十四、总结

### 14.1 数据流总结表

| 步骤 | 模块 | 工作队列 | 频率 | 输入 | 输出 | 延迟 |
|------|------|---------|------|------|------|------|
| 1 | bmi088/bmi270 | wq:SPI2 | 800 Hz | FIFO 中断 | sensor_gyro_fifo | ~50 μs |
| 2a | vehicle_imu | wq:INS0/1 | 265 Hz | sensor_gyro_fifo | vehicle_imu | ~50 μs |
| 2b | vehicle_angular_velocity | wq:rate_ctrl | 667 Hz | sensor_gyro_fifo | vehicle_angular_velocity | ~100 μs |
| 3 | ekf2 | wq:INS0/1 | 193 Hz | vehicle_imu | vehicle_attitude | ~300 μs |
| 4 | mc_att_control | wq:nav_and_ctrl | 193 Hz | vehicle_attitude | vehicle_rates_setpoint | ~150 μs |
| 5 | mc_rate_control | wq:rate_ctrl | 667 Hz | vehicle_angular_velocity | vehicle_torque_setpoint | ~80 μs |
| 6 | control_allocator | wq:rate_ctrl | 667 Hz | torque/thrust_setpoint | actuator_motors | ~100 μs |
| 7 | dshot/pwm_out | wq:hp_default | 实时 | actuator_motors | 电机 PWM | ~50 μs |

---

### 14.2 关键文件索引

| 文件路径 | 模块 | 主要函数 |
|---------|------|----------|
| `src/drivers/imu/bosch/bmi270/BMI270.cpp` | BMI270 驱动 | RunImpl (255), FIFORead (732) |
| `src/modules/sensors/vehicle_imu/VehicleIMU.cpp` | IMU 处理 | Run (169) |
| `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` | 角速度滤波 | Run (784), FilterAngularVelocity (725) |
| `src/modules/ekf2/EKF2.cpp` | EKF2 主模块 | Run (435) |
| `src/modules/ekf2/EKF/ekf.cpp` | EKF 核心 | update (137), predictState (184) |
| `src/modules/mc_att_control/mc_att_control_main.cpp` | 姿态控制 | Run (205) |
| `src/modules/mc_rate_control/MulticopterRateControl.cpp` | 角速率控制 | Run (103) |
| `src/modules/control_allocator/ControlAllocator.cpp` | 混控分配 | Run (303), allocate (432) |

---

### 14.3 频率层级

```
800 Hz   - IMU 硬件采样
667 Hz   - 角速率控制回路（内环）⭐ 最关键
265 Hz   - IMU 数据处理
193 Hz   - 姿态控制回路（外环）
193 Hz   - EKF2 状态估计
97 Hz    - 位置/速度控制
```

---

### 14.4 性能要点

1. **rate_ctrl 队列是最关键的**
   - 667 Hz 高频运行
   - 低延迟（~300 μs）
   - 三个模块紧密协作

2. **双 IMU 冗余**
   - 两个 INS 队列独立处理
   - EKF2Selector 选择最优

3. **串级控制结构**
   - 外环慢（姿态 193 Hz）
   - 内环快（角速率 667 Hz）
   - 保证稳定性和响应性

4. **数据流优化**
   - 使用 FIFO 批量处理
   - 回调机制减少延迟
   - 工作队列避免竞态

---

## 十五、常见问题

### Q1: 为什么 VehicleAngularVelocity 在 rate_ctrl 而不是 INS 队列？

**A**: 因为它直接服务于角速率控制：

```
vehicle_angular_velocity (667 Hz)
    ↓ 立即触发（同一队列）
mc_rate_control (667 Hz)
    ↓ 无延迟
control_allocator (667 Hz)
```

如果在 INS 队列，会有跨队列调度延迟。

---

### Q2: 为什么有两个角速度数据源？

**A**: 两个用途不同：

| 数据源 | 频率 | 用途 | 订阅者 |
|--------|------|------|--------|
| `vehicle_imu` | 265 Hz | **状态估计**（EKF2） | ekf2 |
| `vehicle_angular_velocity` | 667 Hz | **控制回路**（快速） | mc_rate_control |

EKF2 不需要 667 Hz 的高速数据，265 Hz 足够且节省 CPU。

---

### Q3: EKF2 是否使用 vehicle_angular_velocity？

**A**: 是的，但作为辅助：

- 主要输入：`vehicle_imu` (delta_angle, delta_velocity)
- 辅助输入：`vehicle_angular_velocity` (用于偏差估计和验证)

---

### Q4: 如何查看 sensors 模块状态？

**A**:

```bash
sensors status

# 这会输出所有子模块的状态：
# - VehicleAcceleration
# - VehicleAngularVelocity  ← 您要的
# - VehicleMagnetometer
# - VehicleAirData
# - VehicleGPSPosition
# - VehicleOpticalFlow
```

您可以运行这个命令查看完整的滤波器统计！
