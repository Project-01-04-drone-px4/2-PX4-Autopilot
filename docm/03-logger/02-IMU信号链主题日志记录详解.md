# IMU 信号链主题日志记录详解
## Logger 模块对 IMU 相关主题的记录频率、发布者和订阅者分析

---

## 1. 概述

本文档详细分析 PX4 Logger 模块如何记录 IMU（惯性测量单元）信号链中的各个主题，包括记录频率、发布者、订阅者以及在信号链中的作用。

### 1.1 IMU 信号链概览

```
硬件传感器 (BMI088, BMI270, ICM42688P 等)
    ↓
[驱动层] sensor_gyro_fifo, sensor_accel_fifo
    ↓
[VehicleIMU] vehicle_imu
    ↓
[VehicleAngularVelocity] vehicle_angular_velocity
    ↓
[Sensors] sensor_combined
    ↓
[EKF2] vehicle_attitude, vehicle_local_position
    ↓
[控制器] actuator_motors, actuator_servos
```

---

## 2. IMU 信号链主题详细分析

### 2.1 sensor_gyro - 原始陀螺仪数据

#### 2.1.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `sensor_gyro` |
| **消息类型** | `sensor_gyro_s` |
| **发布频率** | ~1000-8000 Hz（取决于传感器） |
| **实例数量** | 最多 4 个（多 IMU 冗余） |
| **队列长度** | 8 samples |
| **消息定义** | `msg/sensor_gyro.msg` |

#### 2.1.2 Logger 记录配置

**在 DEFAULT 模式下**：

```cpp
// logged_topics.cpp:201
add_optional_topic_multi("sensor_gyro", 1000, 4);
```

| 配置项 | 值 |
|--------|-----|
| **记录间隔** | 1000 ms (1 Hz) |
| **最大实例** | 4 个 |
| **是否可选** | 是（optional） |
| **实际记录频率** | ~1 Hz |

**说明**：
- ⚠️ DEFAULT 模式下记录频率**非常低**（1 Hz），仅用于基本监控
- 如需高速率数据，应使用 `THERMAL_CALIBRATION` 或 `SENSOR_COMPARISON` 模式

**在 THERMAL_CALIBRATION 模式下**：

```cpp
// logged_topics.cpp:327
add_topic_multi("sensor_gyro", 100, 4);
```

| 配置项 | 值 |
|--------|-----|
| **记录间隔** | 100 ms (10 Hz) |
| **最大实例** | 4 个 |
| **实际记录频率** | ~10 Hz |

#### 2.1.3 发布者

| 类名 | 文件位置 | 频率 | 发布方式 |
|------|---------|------|---------|
| **PX4Gyroscope** | `src/lib/drivers/gyroscope/PX4Gyroscope.cpp` | 1000-8000 Hz | `_sensor_pub.publish()` |

**发布流程**：

```
传感器驱动 (例如 BMI088)
    ↓ 读取硬件寄存器
PX4Gyroscope::update()
    ↓ 应用校准、旋转
_sensor_pub.publish(sensor_gyro)
```

**关键代码**：`src/lib/drivers/gyroscope/PX4Gyroscope.cpp`

```cpp
void PX4Gyroscope::update(const hrt_abstime &timestamp_sample,
                          float x, float y, float z)
{
	sensor_gyro_s report;
	report.timestamp_sample = timestamp_sample;
	report.device_id = _device_id;

	// 应用旋转和缩放
	rotate_3f(_rotation, x, y, z);
	report.x = x * _scale;
	report.y = y * _scale;
	report.z = z * _scale;

	report.timestamp = hrt_absolute_time();
	_sensor_pub.publish(report);
}
```

#### 2.1.4 订阅者

| 类名 | 文件位置 | 用途 |
|------|---------|------|
| **VehicleIMU** | `src/modules/sensors/vehicle_imu/VehicleIMU.cpp` | IMU 数据集成 |
| **GyroFFT** | `src/modules/gyro_fft/GyroFFT.cpp` | FFT 振动分析 |
| **Logger** | `src/modules/logger/` | 日志记录 |

#### 2.1.5 在信号链中的作用

- **原始传感器数据源**
- 包含未融合的陀螺仪测量值
- 用于传感器健康监控、校准和故障检测
- 为上层 VehicleIMU 模块提供输入

#### 2.1.6 数据结构

```cpp
struct sensor_gyro_s {
    uint64_t timestamp;           // 时间戳 (μs)
    uint64_t timestamp_sample;    // 采样时间戳
    uint32_t device_id;           // 设备 ID
    float x;                      // 角速度 X 轴 (rad/s)
    float y;                      // 角速度 Y 轴 (rad/s)
    float z;                      // 角速度 Z 轴 (rad/s)
    float temperature;            // 温度 (°C)
    uint32_t error_count;         // 错误计数
    uint8_t clip_counter[3];      // 削波计数器
};
```

---

### 2.2 sensor_accel - 原始加速度计数据

#### 2.2.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `sensor_accel` |
| **消息类型** | `sensor_accel_s` |
| **发布频率** | ~1000-8000 Hz |
| **实例数量** | 最多 4 个 |
| **队列长度** | 8 samples |

#### 2.2.2 Logger 记录配置

**在 DEFAULT 模式下**：

```cpp
// logged_topics.cpp:197
add_optional_topic_multi("sensor_accel", 1000, 4);
```

| 配置项 | 值 |
|--------|-----|
| **记录间隔** | 1000 ms (1 Hz) |
| **最大实例** | 4 个 |
| **实际记录频率** | ~1 Hz |

**在 THERMAL_CALIBRATION 模式下**：

```cpp
// logged_topics.cpp:325
add_topic_multi("sensor_accel", 100, 4);
```

| 配置项 | 值 |
|--------|-----|
| **记录间隔** | 100 ms (10 Hz) |
| **最大实例** | 4 个 |
| **实际记录频率** | ~10 Hz |

#### 2.2.3 发布者

| 类名 | 文件位置 | 频率 |
|------|---------|------|
| **PX4Accelerometer** | `src/lib/drivers/accelerometer/PX4Accelerometer.cpp` | 1000-8000 Hz |

#### 2.2.4 订阅者

| 类名 | 文件位置 | 用途 |
|------|---------|------|
| **VehicleIMU** | `src/modules/sensors/vehicle_imu/VehicleIMU.cpp` | IMU 数据集成 |
| **VehicleAcceleration** | `src/modules/sensors/vehicle_acceleration/` | 加速度处理 |
| **Logger** | `src/modules/logger/` | 日志记录 |

#### 2.2.5 在信号链中的作用

- **原始加速度数据源**
- 包含线性加速度和重力加速度
- 用于姿态估计和位置估计
- 振动监测和故障检测

---

### 2.3 vehicle_imu - 集成 IMU 数据

#### 2.3.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `vehicle_imu` |
| **消息类型** | `vehicle_imu_s` |
| **发布频率** | ~250 Hz（取决于 IMU_INTEG_RATE 参数） |
| **实例数量** | 最多 4 个（对应每个 IMU） |
| **工作队列** | `wq:INS0`, `wq:INS1`, `wq:INS2`, `wq:INS3` |

#### 2.3.2 Logger 记录配置

**在 DEFAULT 模式下**：

```cpp
// logged_topics.cpp:205
add_topic_multi("vehicle_imu", 500, 4);
```

| 配置项 | 值 |
|--------|-----|
| **记录间隔** | 500 ms (2 Hz) |
| **最大实例** | 4 个 |
| **实际记录频率** | ~2 Hz |

**说明**：
- 记录的是**积分后的数据**（delta_angle, delta_velocity）
- 相比原始传感器数据，已经过校准和降噪处理

#### 2.3.3 发布者

| 类名 | 文件位置 | 频率 | 发布函数 |
|------|---------|------|---------|
| **VehicleIMU** | `src/modules/sensors/vehicle_imu/VehicleIMU.cpp:539-640` | ~250 Hz | `_vehicle_imu_pub.publish()` |

**发布流程**：

```
sensor_accel + sensor_gyro
    ↓
VehicleIMU::UpdateAccel() + UpdateGyro()
    ↓ 积分 (Integration)
VehicleIMU::Publish()
    ↓ 应用校准 (Calibration)
_vehicle_imu_pub.publish(vehicle_imu)
```

**关键代码**：

```cpp
// VehicleIMU.cpp:539-640
bool VehicleIMU::Publish()
{
	vehicle_imu_s imu;
	Vector3f delta_angle;
	Vector3f delta_velocity;

	// 从积分器获取数据
	if (_accel_integrator.reset(delta_velocity, imu.delta_velocity_dt)
	    && _gyro_integrator.reset(delta_angle, imu.delta_angle_dt)) {

		// 应用校准
		const Vector3f angular_velocity =
		    _gyro_calibration.Correct(delta_angle / gyro_dt_s);
		const Vector3f acceleration =
		    _accel_calibration.Correct(delta_velocity / accel_dt_s);

		// 填充消息
		imu.timestamp = now;
		imu.timestamp_sample = timestamp_sample;
		imu.delta_angle = angular_velocity * gyro_dt_s;
		imu.delta_velocity = acceleration * accel_dt_s;
		imu.delta_angle_dt = gyro_dt_us;
		imu.delta_velocity_dt = accel_dt_us;
		imu.accel_device_id = _accel_device_id;
		imu.gyro_device_id = _gyro_device_id;

		_vehicle_imu_pub.publish(imu);
		return true;
	}
	return false;
}
```

#### 2.3.4 订阅者

| 类名 | 文件位置 | 用途 |
|------|---------|------|
| **EKF2** | `src/modules/ekf2/EKF2.cpp:435-864` | 状态估计（多实例模式） |
| **Sensors** | `src/modules/sensors/sensors.cpp` | 传感器融合 |
| **Logger** | `src/modules/logger/` | 日志记录 |

**EKF2 订阅代码**：

```cpp
// EKF2.cpp:435-864
void EKF2::Run()
{
	vehicle_imu_s imu;
	if (_vehicle_imu_sub.update(&imu)) {
		// 将 IMU 样本送入 EKF
		imuSample imu_sample_new;
		imu_sample_new.time_us = imu.timestamp_sample;
		imu_sample_new.delta_ang = Vector3f(imu.delta_angle);
		imu_sample_new.delta_vel = Vector3f(imu.delta_velocity);
		imu_sample_new.delta_ang_dt = imu.delta_angle_dt * 1e-6f;
		imu_sample_new.delta_vel_dt = imu.delta_velocity_dt * 1e-6f;

		_ekf.setIMUData(imu_sample_new);

		if (_ekf.update()) {
			PublishAttitude(now);
			PublishLocalPosition(now);
			// ...
		}
	}
}
```

#### 2.3.5 在信号链中的作用

- **IMU 数据积分和预处理**
- 将高速原始传感器数据降采样到控制频率
- 应用校准参数（bias, scale, rotation）
- 为 EKF2 提供时间对齐的 IMU 数据
- 支持多 IMU 冗余

#### 2.3.6 数据结构

```cpp
struct vehicle_imu_s {
    uint64_t timestamp;           // 时间戳 (μs)
    uint64_t timestamp_sample;    // 采样时间戳
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

**关键参数**：
- `delta_angle`: 在 `delta_angle_dt` 时间内的角度变化量（积分）
- `delta_velocity`: 在 `delta_velocity_dt` 时间内的速度变化量（积分）

---

### 2.4 vehicle_angular_velocity - 滤波后角速度

#### 2.4.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `vehicle_angular_velocity` |
| **消息类型** | `vehicle_angular_velocity_s` |
| **发布频率** | ~667 Hz（1.5 ms 周期） |
| **实例数量** | 单实例（选择最优传感器） |
| **工作队列** | `wq:rate_ctrl` |

#### 2.4.2 Logger 记录配置

**在 DEFAULT 模式下**：

```cpp
// logged_topics.cpp:135
add_topic("vehicle_angular_velocity", 20);
```

| 配置项 | 值 |
|--------|-----|
| **记录间隔** | 20 ms (50 Hz) |
| **实际记录频率** | ~50 Hz |

**在 HIGH_RATE 模式下**：

```cpp
// logged_topics.cpp:276
add_topic("vehicle_angular_velocity");  // 全速率
```

| 配置项 | 值 |
|--------|-----|
| **记录间隔** | 0 (全速) |
| **实际记录频率** | ~667 Hz |

**在 SYSTEM_IDENTIFICATION 模式下**：

```cpp
// logged_topics.cpp:363
add_topic("vehicle_angular_velocity");  // 全速率
```

| 配置项 | 值 |
|--------|-----|
| **记录间隔** | 0 (全速) |
| **实际记录频率** | ~667 Hz |

#### 2.4.3 发布者

| 类名 | 文件位置 | 频率 | 发布函数 |
|------|---------|------|---------|
| **VehicleAngularVelocity** | `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp:921-950` | ~667 Hz | `_vehicle_angular_velocity_pub.publish()` |

**发布流程**：

```
sensor_gyro_fifo (原始 FIFO 数据)
    ↓
VehicleAngularVelocity::Run()
    ↓ 应用滤波器链
      - 动态陷波滤波器 (ESC RPM)
      - 静态陷波滤波器
      - 低通滤波器
    ↓
CalibrateAndPublish()
    ↓ 应用校准、去偏置
_vehicle_angular_velocity_pub.publish()
```

**关键代码**：

```cpp
// VehicleAngularVelocity.cpp:784-919
void VehicleAngularVelocity::Run()
{
	// 更新动态陷波滤波器
	UpdateDynamicNotchEscRpm(time_now_us);
	UpdateDynamicNotchFFT(time_now_us);

	sensor_gyro_fifo_s sensor_fifo_data;
	while (_sensor_gyro_fifo_sub.update(&sensor_fifo_data)) {

		for (int axis = 0; axis < 3; axis++) {
			// 应用滤波器链
			angular_velocity(axis) = FilterAngularVelocity(
			    axis, gyro_data, N);
			angular_acceleration(axis) = FilterAngularAcceleration(
			    axis, dt, gyro_data, N);
		}

		// 校准并发布
		CalibrateAndPublish(timestamp_sample,
		                    angular_velocity,
		                    angular_acceleration);
	}
}

// VehicleAngularVelocity.cpp:921-950
bool VehicleAngularVelocity::CalibrateAndPublish(...)
{
	vehicle_angular_velocity_s angular_velocity_out;
	angular_velocity_out.timestamp_sample = timestamp_sample;

	// 应用校准和去偏置
	_angular_velocity = _calibration.Correct(angular_velocity_uncalibrated) - _bias;
	_angular_velocity.copyTo(angular_velocity_out.xyz);

	// 角加速度
	_angular_acceleration = _calibration.rotation() * angular_acceleration_uncalibrated;
	_angular_acceleration.copyTo(angular_velocity_out.xyz_derivative);

	angular_velocity_out.timestamp = hrt_absolute_time();
	_vehicle_angular_velocity_pub.publish(angular_velocity_out);

	return true;
}
```

#### 2.4.4 订阅者

| 类名 | 文件位置 | 用途 |
|------|---------|------|
| **MulticopterRateControl** | `src/modules/mc_rate_control/MulticopterRateControl.cpp:126` | 多旋翼角速率控制 |
| **FixedwingRateControl** | `src/modules/fw_rate_control/` | 固定翼角速率控制 |
| **Logger** | `src/modules/logger/` | 日志记录 |

**多旋翼控制器订阅代码**：

```cpp
// MulticopterRateControl.cpp:126-250
void MulticopterRateControl::Run()
{
	vehicle_angular_velocity_s angular_velocity;
	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// 获取角速度
		Vector3f rates(angular_velocity.xyz);

		// 角速率控制 PID
		Vector3f rates_setpoint = ...;
		Vector3f rate_error = rates_setpoint - rates;

		// 计算输出
		_control_output = _rate_control.update(rate_error, dt);

		// 发布到控制分配器
		vehicle_torque_setpoint_s torque_setpoint;
		_control_output.copyTo(torque_setpoint.xyz);
		_vehicle_torque_setpoint_pub.publish(torque_setpoint);
	}
}
```

#### 2.4.5 在信号链中的作用

- **高质量角速度数据源**
- 应用多级滤波降低振动噪声
- 动态陷波滤波器抑制电机振动
- 为控制器提供干净的反馈信号
- **关键性能节点**：滤波延迟直接影响控制性能

#### 2.4.6 滤波器配置

| 滤波器类型 | 参数 | 默认值 | 说明 |
|-----------|------|--------|------|
| 动态陷波 (ESC RPM) | `IMU_GYRO_DNF_EN` | 1 | 基于 ESC 转速 |
| 动态陷波 (FFT) | `IMU_GYRO_FFT_EN` | 0 | 基于 FFT 分析 |
| 静态陷波 | `IMU_GYRO_NF0_FRQ` | 0 (禁用) | 固定频率陷波 |
| 低通滤波 | `IMU_GYRO_CUTOFF` | 80 Hz | 截止频率 |

---

### 2.5 sensor_combined - 融合传感器数据

#### 2.5.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `sensor_combined` |
| **消息类型** | `sensor_combined_s` |
| **发布频率** | ~1000 Hz（与主 IMU 同步） |
| **实例数量** | 单实例 |
| **工作队列** | `wq:nav_and_controllers` |

#### 2.5.2 Logger 记录配置

**在 DEFAULT 模式下**：

```cpp
// logged_topics.cpp:121
add_topic("sensor_combined");
```

| 配置项 | 值 |
|--------|-----|
| **记录间隔** | 0 (全速) |
| **实际记录频率** | ~1000 Hz |

**在所有高速模式下均为全速记录**

#### 2.5.3 发布者

| 类名 | 文件位置 | 频率 | 发布函数 |
|------|---------|------|---------|
| **Sensors** | `src/modules/sensors/sensors.cpp:618-623` | ~1000 Hz | `_sensor_pub.publish()` |

**发布流程**：

```
vehicle_imu (多实例)
    ↓
Sensors::Run()
    ↓
VotedSensorsUpdate::sensorsPoll()
    ↓ 多 IMU 投票选择
    ↓ 数据一致性检查
setRelativeTimestamps()
    ↓
_sensor_pub.publish(sensor_combined)
```

**关键代码**：

```cpp
// sensors.cpp:596-623
void Sensors::Run()
{
	// 调用投票传感器更新
	_voted_sensors_update.sensorsPoll(_sensor_combined);

	// 发布 sensor_combined
	if (_sensor_combined.timestamp != _sensor_combined_prev_timestamp) {
		_voted_sensors_update.setRelativeTimestamps(_sensor_combined);
		_sensor_pub.publish(_sensor_combined);  // ← 发布
		_sensor_combined_prev_timestamp = _sensor_combined.timestamp;
	}

	ScheduleDelayed(10_ms);
}
```

#### 2.5.4 订阅者

| 类名 | 文件位置 | 用途 |
|------|---------|------|
| **EKF2** (单实例模式) | `src/modules/ekf2/EKF2.cpp` | 状态估计（备用） |
| **LocalPositionEstimator** | `src/modules/local_position_estimator/` | 本地位置估计 |
| **AttitudeEstimatorQ** | `src/modules/attitude_estimator_q/` | 姿态估计（备用） |
| **Logger** | `src/modules/logger/` | 日志记录 |

#### 2.5.5 在信号链中的作用

- **多 IMU 融合**：从多个 IMU 中选择最优数据
- **故障检测**：检测传感器异常并切换
- **数据投票**：确保数据一致性
- **为 EKF2 单实例模式提供输入**（传统模式）

#### 2.5.6 数据结构

```cpp
struct sensor_combined_s {
    uint64_t timestamp;                // 时间戳 (μs)

    // 陀螺仪数据
    float gyro_rad[3];                 // 角速度 (rad/s)
    uint32_t gyro_integral_dt;         // 陀螺仪积分时间 (μs)

    // 加速度计数据
    float accelerometer_m_s2[3];      // 加速度 (m/s²)
    uint32_t accelerometer_integral_dt; // 加速度计积分时间 (μs)

    // 削波标志
    uint8_t accelerometer_clipping;    // 加速度计削波
    uint8_t gyro_clipping;             // 陀螺仪削波

    // 校准计数
    uint8_t accel_calibration_count;
    uint8_t gyro_calibration_count;
};
```

---

### 2.6 vehicle_imu_status - IMU 状态信息

#### 2.6.1 基本信息

| 属性 | 值 |
|------|-----|
| **主题名称** | `vehicle_imu_status` |
| **消息类型** | `vehicle_imu_status_s` |
| **发布频率** | ~10 Hz (100 ms 间隔) |
| **实例数量** | 最多 4 个 |

#### 2.6.2 Logger 记录配置

**在 DEFAULT 模式下**：

```cpp
// logged_topics.cpp:206
add_topic_multi("vehicle_imu_status", 1000, 4);
```

| 配置项 | 值 |
|--------|-----|
| **记录间隔** | 1000 ms (1 Hz) |
| **最大实例** | 4 个 |

#### 2.6.3 在信号链中的作用

- **IMU 健康监控**
- **振动指标**：均方根振动幅度
- **削波检测**：传感器饱和统计
- **温度监控**
- **校准状态**

---

## 3. 不同 Logger 模式下的 IMU 记录对比

### 3.1 各模式记录频率对比表

| 主题名称 | DEFAULT | THERMAL_CAL | SYSTEM_ID | HIGH_RATE | SENSOR_CMP |
|---------|---------|-------------|-----------|-----------|-----------|
| `sensor_gyro` | 1 Hz | **10 Hz** | 1 Hz | 1 Hz | **10 Hz** |
| `sensor_accel` | 1 Hz | **10 Hz** | 1 Hz | 1 Hz | **10 Hz** |
| `vehicle_imu` | 2 Hz | 2 Hz | 2 Hz | 2 Hz | 2 Hz |
| `vehicle_angular_velocity` | 50 Hz | 50 Hz | **~667 Hz** | **~667 Hz** | 50 Hz |
| `sensor_combined` | **~1000 Hz** | **~1000 Hz** | **~1000 Hz** | **~1000 Hz** | **~1000 Hz** |
| `vehicle_attitude` | 50 Hz | 50 Hz | 50 Hz | **全速** | 50 Hz |
| `actuator_motors` | 100 Hz | 100 Hz | **全速** | **全速** | 100 Hz |

### 3.2 模式选择建议

| 应用场景 | 推荐模式 | 原因 |
|---------|---------|------|
| **日常飞行** | DEFAULT | 文件小，包含基本信息 |
| **IMU 热校准** | THERMAL_CALIBRATION | 高频原始传感器数据 |
| **系统辨识** | SYSTEM_IDENTIFICATION | 高频控制和 IMU 数据 |
| **PID 调参** | HIGH_RATE | 高频姿态和控制数据 |
| **传感器对比** | SENSOR_COMPARISON | 多传感器低频对比 |
| **振动分析** | HIGH_RATE_SENSORS | 高频传感器数据 |

### 3.3 数据量估算

**1 分钟飞行的日志大小估算**：

| 模式 | 估算大小 | 主要贡献 |
|------|---------|---------|
| DEFAULT | ~10 MB | sensor_combined (1000 Hz) |
| SYSTEM_IDENTIFICATION | ~30 MB | + 高频 vehicle_angular_velocity |
| HIGH_RATE | ~40 MB | + 高频姿态和执行器 |
| THERMAL_CALIBRATION | ~15 MB | + 原始传感器 10 Hz |
| RAW_IMU_GYRO_FIFO | ~100 MB | sensor_gyro_fifo (8000 Hz) |

---

## 4. IMU 信号链完整数据流图

### 4.1 完整信号链

```
┌─────────────────────────────────────────────────────────────────┐
│                         硬件传感器层                              │
│  BMI088, BMI270, ICM42688P 等                                   │
│  - 采样率: 1000-8000 Hz                                         │
│  - 输出: 原始 ADC 数据                                           │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│                        驱动层 (Drivers)                          │
│  PX4Gyroscope, PX4Accelerometer                                 │
│  - 应用: scale, rotation, offset                                │
│  - 发布: sensor_gyro (1000-8000 Hz)                             │
│  - 发布: sensor_accel (1000-8000 Hz)                            │
│  ┌─────────────────────────────────────┐                        │
│  │ Logger: DEFAULT 模式记录 1 Hz       │                        │
│  └─────────────────────────────────────┘                        │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│                   VehicleIMU (wq:INS0-3)                         │
│  - 订阅: sensor_gyro, sensor_accel                              │
│  - 处理: 积分 → delta_angle, delta_velocity                     │
│  - 应用: 校准 (Calibration)                                     │
│  - 发布: vehicle_imu (~250 Hz)                                  │
│  ┌─────────────────────────────────────┐                        │
│  │ Logger: DEFAULT 模式记录 2 Hz       │                        │
│  └─────────────────────────────────────┘                        │
└─────┬───────────────────────────────────────────────────────────┘
      │
      ├─────────────────┐
      │                 │
      ▼                 ▼
┌──────────────┐  ┌──────────────────────────────────────────────┐
│   Sensors    │  │  VehicleAngularVelocity (wq:rate_ctrl)       │
│   (多IMU融合) │  │  - 订阅: sensor_gyro_fifo                    │
│              │  │  - 处理: 动态陷波 + 静态陷波 + 低通滤波       │
│              │  │  - 应用: 校准, 去偏置                        │
│              │  │  - 发布: vehicle_angular_velocity (~667 Hz)  │
│              │  │  ┌────────────────────────────────────────┐  │
│              │  │  │ Logger: DEFAULT 50 Hz                  │  │
│              │  │  │         SYSTEM_ID/HIGH_RATE 全速       │  │
│              │  │  └────────────────────────────────────────┘  │
└──────┬───────┘  └─────────────────┬────────────────────────────┘
       │                            │
       ▼                            │
┌─────────────────────────────────┐ │
│  sensor_combined (~1000 Hz)     │ │
│  ┌────────────────────────────┐ │ │
│  │ Logger: 所有模式全速记录   │ │ │
│  └────────────────────────────┘ │ │
└──────┬──────────────────────────┘ │
       │                            │
       ▼                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                         EKF2 (wq:nav_and_controllers)            │
│  - 订阅: sensor_combined (单实例) 或 vehicle_imu (多实例)        │
│  - 处理: 扩展卡尔曼滤波                                           │
│  - 发布: vehicle_attitude, vehicle_local_position              │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│                      控制器 (Controllers)                         │
│  MulticopterRateControl, MulticopterAttitudeControl             │
│  - 订阅: vehicle_angular_velocity, vehicle_attitude             │
│  - 订阅: vehicle_attitude_setpoint                              │
│  - 发布: vehicle_torque_setpoint, vehicle_thrust_setpoint       │
│  ┌─────────────────────────────────────┐                        │
│  │ Logger: SYSTEM_ID 模式全速记录      │                        │
│  └─────────────────────────────────────┘                        │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│                   控制分配器 (Control Allocator)                  │
│  - 订阅: vehicle_torque_setpoint, vehicle_thrust_setpoint      │
│  - 发布: actuator_motors, actuator_servos                      │
│  ┌─────────────────────────────────────┐                        │
│  │ Logger: DEFAULT 100 Hz              │                        │
│  │         SYSTEM_ID/HIGH_RATE 全速    │                        │
│  └─────────────────────────────────────┘                        │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│                       执行器/电机                                 │
│  ESC, Servos                                                    │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 关键延迟节点

| 节点 | 典型延迟 | 影响 |
|------|---------|------|
| 传感器 → 驱动 | < 0.1 ms | 硬件中断延迟 |
| 驱动 → VehicleIMU | ~1 ms | 积分周期 (IMU_INTEG_RATE) |
| VehicleIMU → VehicleAngularVelocity | < 0.5 ms | 滤波延迟 |
| VehicleAngularVelocity → 控制器 | < 0.5 ms | 控制环延迟 |
| **总延迟** | **~2-3 ms** | 从传感器到控制输出 |

---

## 5. 实用工具和分析方法

### 5.1 查看当前 IMU 配置

```bash
# 查看 IMU 相关参数
param show IMU_*

# 关键参数
IMU_INTEG_RATE     # 积分频率 (Hz)
IMU_GYRO_CUTOFF    # 陀螺仪低通截止频率 (Hz)
IMU_GYRO_DNF_EN    # 动态陷波使能
IMU_GYRO_FFT_EN    # FFT 陷波使能
```

### 5.2 实时监控 IMU 数据

```bash
# 监听主题
listener vehicle_angular_velocity
listener sensor_combined
listener vehicle_imu

# 查看发布频率
uorb top
```

### 5.3 日志分析 - pyulog

```python
from pyulog import ULog

# 加载日志
ulog = ULog('log_file.ulg')

# 获取 IMU 数据
imu_data = ulog.get_dataset('vehicle_angular_velocity')
print(f"采样数: {len(imu_data.data['timestamp'])}")
print(f"平均频率: {1e6 / np.mean(np.diff(imu_data.data['timestamp']))} Hz")

# 绘制角速度
import matplotlib.pyplot as plt
time = (imu_data.data['timestamp'] - imu_data.data['timestamp'][0]) * 1e-6
plt.plot(time, imu_data.data['xyz'][:, 0], label='Roll rate')
plt.plot(time, imu_data.data['xyz'][:, 1], label='Pitch rate')
plt.plot(time, imu_data.data['xyz'][:, 2], label='Yaw rate')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Angular velocity (rad/s)')
plt.show()
```

### 5.4 验证数据完整性

```bash
# 使用 ulog_info 检查日志
ulog_info log_file.ulg

# 检查特定主题
ulog_info log_file.ulg -m sensor_combined

# 输出示例
# sensor_combined:
#   num messages: 60000
#   frequency: 1000.2 Hz
#   duration: 59.98 s
```

---

## 6. 总结

### 6.1 关键要点

1. **sensor_gyro/sensor_accel**：
   - 原始传感器数据，DEFAULT 模式仅 1 Hz
   - 需要高频数据应使用 THERMAL_CALIBRATION (10 Hz)

2. **vehicle_imu**：
   - 积分和校准后的 IMU 数据
   - 所有模式下固定 2 Hz（低频监控）

3. **vehicle_angular_velocity**：
   - 滤波后的高质量角速度
   - DEFAULT 50 Hz，SYSTEM_ID/HIGH_RATE 全速 (~667 Hz)
   - **控制器的直接输入**

4. **sensor_combined**：
   - 多 IMU 融合数据
   - 所有模式下全速 (~1000 Hz)
   - EKF2 单实例模式输入

### 6.2 模式切换推荐

```bash
# 系统辨识实验
param set SDLOG_PROFILE 9  # DEFAULT + SYSTEM_ID
# 记录: vehicle_angular_velocity, actuator_motors 全速

# PID 调参
param set SDLOG_PROFILE 17  # DEFAULT + HIGH_RATE
# 记录: 姿态、角速率、执行器全速

# IMU 校准
param set SDLOG_PROFILE 5  # DEFAULT + THERMAL_CAL
# 记录: sensor_gyro, sensor_accel 10 Hz
```

---

**文档版本**: v1.0
**最后更新**: 2025-11-01
**适用 PX4 版本**: v1.14+
**参考文档**:
- `logged_topics.cpp`
- `VehicleIMU.cpp`
- `VehicleAngularVelocity.cpp`
- `sensors.cpp`

