# PX4 MTF01/Optical Flow Velocity Pipeline

## 摘要

本文梳理当前 PX4 在收到 MTF01 或同类光流数据后，如何从原始光流积分生成飞机 body 系水平速度，并进一步经过传感器安装 yaw 校正、机体角速度补偿、EKF 姿态与航向转换，最终发布世界系 NED 速度。

当前代码树中没有直接命名为 `MTF01` 或 `mtf01` 的专用驱动。实际链路通常以通用光流输入进入 PX4，例如：

- MAVLink `OPTICAL_FLOW_RAD` / `HIL_OPTICAL_FLOW`
- 串口或 SPI 光流驱动发布的 `sensor_optical_flow`
- 例如 `thoneflow`、`px4flow`、`pmw3901`、`paa3905` 等驱动

后续主链路统一为：

```text
MTF01 / 光流传感器
  -> sensor_optical_flow
  -> VehicleOpticalFlow
  -> vehicle_optical_flow
  -> EKF2::UpdateOpticalFlow
  -> Ekf::controlOpticalFlowFusion
  -> Ekf::updateOptFlow / fuseOptFlow
  -> vehicle_local_position.vx/vy/vz
  -> vehicle_odometry.velocity
```

## 1. 光流数据入口

### 1.1 MAVLink 光流入口

如果 MTF01 是通过 MAVLink `OPTICAL_FLOW_RAD` 输入，入口位于：

```text
src/modules/mavlink/mavlink_receiver.cpp
MavlinkReceiver::handle_message_optical_flow_rad()
```

核心处理：

```cpp
sensor_optical_flow.pixel_flow[0] = flow.integrated_x;
sensor_optical_flow.pixel_flow[1] = flow.integrated_y;
sensor_optical_flow.integration_timespan_us = flow.integration_time_us;
sensor_optical_flow.quality = flow.quality;
```

如果 MAVLink 消息中带积分陀螺：

```cpp
integrated_gyro.copyTo(sensor_optical_flow.delta_angle);
sensor_optical_flow.delta_angle_available = true;
```

如果消息中带距离：

```cpp
sensor_optical_flow.distance_m = flow.distance;
sensor_optical_flow.distance_available = true;
```

最终发布：

```cpp
_sensor_optical_flow_pub.publish(sensor_optical_flow);
```

### 1.2 ThoneFlow 类串口驱动入口

如果使用类似 `thoneflow` 的串口光流驱动，解析入口位于：

```text
src/drivers/optical_flow/thoneflow/thoneflow_parser.cpp
```

串口包中的 `delta_x`、`delta_y` 被转换为弧度：

```cpp
flow->pixel_flow[0] = static_cast<float>(delta_x) * (3.52e-3f);
flow->pixel_flow[1] = static_cast<float>(delta_y) * (3.52e-3f);
```

驱动发布 `sensor_optical_flow` 的位置：

```text
src/drivers/optical_flow/thoneflow/thoneflow.cpp
```

关键代码：

```cpp
report.integration_timespan_us = 10526;
rotate_3f(_rotation, report.pixel_flow[0], report.pixel_flow[1], zeroval);
_sensor_optical_flow_pub.publish(report);
```

## 2. sensor_optical_flow 到 vehicle_optical_flow

PX4 的 `sensors` 模块会创建 `VehicleOpticalFlow`，订阅 `sensor_optical_flow`，并输出 `vehicle_optical_flow`。

入口：

```text
src/modules/sensors/vehicle_optical_flow/VehicleOpticalFlow.cpp
VehicleOpticalFlow::Run()
```

### 2.1 光流积分累计

`VehicleOpticalFlow` 会累计传感器输入的光流积分：

```cpp
_flow_integral(0) += sensor_optical_flow.pixel_flow[0];
_flow_integral(1) += sensor_optical_flow.pixel_flow[1];
_integration_timespan_us += sensor_optical_flow.integration_timespan_us;
```

### 2.2 陀螺角增量来源

如果 `sensor_optical_flow` 自带 `delta_angle`，PX4 直接使用：

```cpp
if (sensor_optical_flow.delta_angle_available && Vector3f(sensor_optical_flow.delta_angle).isAllFinite()) {
	_delta_angle += _flow_rotation * Vector3f{sensor_optical_flow.delta_angle};
	_delta_angle_available = true;
}
```

否则，PX4 会从 `sensor_gyro` 中取与光流积分时间段同步的陀螺数据，并积分得到 `_delta_angle`。

### 2.3 距离来源

距离优先来自光流消息：

```cpp
if (sensor_optical_flow.distance_available && PX4_ISFINITE(sensor_optical_flow.distance_m)) {
	_distance_sum += sensor_optical_flow.distance_m;
}
```

如果光流消息没有距离，则使用向下的 `distance_sensor`：

```cpp
if (_range_buffer.peak_first_older_than(sensor_optical_flow.timestamp_sample, &range_sample)) {
	_distance_sum += range_sample.data;
}
```

## 3. 传感器安装 yaw/旋转校正

如果用户说的“yaw 校准”是指光流传感器相对飞机 body 的安装方向校正，那么实际发生在 `VehicleOpticalFlow` 中，使用参数：

```text
SENS_FLOW_ROT
```

对应代码：

```cpp
rotate_3f((enum Rotation)_param_sens_flow_rot.get(),
          vehicle_optical_flow.pixel_flow[0],
          vehicle_optical_flow.pixel_flow[1],
          zeroval);
```

这一步将光流测量从传感器安装坐标旋转到 PX4 约定的机体系。它校正的是传感器安装朝向，不是 EKF 的航向估计。

处理完成后发布：

```cpp
_vehicle_optical_flow_pub.publish(vehicle_optical_flow);
```

发布的话题为：

```text
vehicle_optical_flow
```

## 4. 机体系水平速度的生成

PX4 中有两个地方会根据光流计算 body 系速度：

- `VehicleOpticalFlow` 中计算 `vehicle_optical_flow_vel`，主要用于 logging/debug。
- EKF 内部计算 `_flow_vel_body` 和 `_flow_vel_ne`，用于日志，以及在特定情况下 reset EKF 水平速度；真正的导航速度来自 EKF 状态融合结果。

### 4.1 VehicleOpticalFlow 中的 body 速度

位置：

```text
src/modules/sensors/vehicle_optical_flow/VehicleOpticalFlow.cpp
```

PX4 会先将光流符号转换为 EKF 使用的 LOS rate 约定：

```cpp
const Vector2f flow_xy_rad{-vehicle_optical_flow.pixel_flow[0],
                           -vehicle_optical_flow.pixel_flow[1]};

const Vector3f gyro_xyz{-vehicle_optical_flow.delta_angle[0],
                        -vehicle_optical_flow.delta_angle[1],
                        -vehicle_optical_flow.delta_angle[2]};
```

然后去除机体角运动造成的图像运动：

```cpp
const Vector2f flow_compensated_XY_rad = flow_xy_rad - gyro_xyz.xy();
```

再根据距离和积分时间换算为 body 系水平速度：

```cpp
vel_optflow_body(0) = - range * flow_compensated_XY_rad(1) / flow_dt;
vel_optflow_body(1) =   range * flow_compensated_XY_rad(0) / flow_dt;
vel_optflow_body(2) = 0.f;
```

公式整理为：

```text
flow_rate_x = compensated_flow_x / dt
flow_rate_y = compensated_flow_y / dt

v_body_x = -range * flow_rate_y
v_body_y =  range * flow_rate_x
```

`VehicleOpticalFlow` 发布的调试话题：

```text
vehicle_optical_flow_vel
```

其中字段定义位于：

```text
msg/VehicleOpticalFlowVel.msg
```

核心字段：

```text
vel_body
vel_ne
flow_uncompensated_integral
flow_compensated_integral
gyro_rate
gyro_rate_integral
```

### 4.2 EKF 中的 body 速度

EKF2 订阅 `vehicle_optical_flow`，并生成内部 `flowSample`：

```text
src/modules/ekf2/EKF2.cpp
EKF2::UpdateOpticalFlow()
```

关键代码：

```cpp
flowSample flow {
	.time_us = optical_flow.timestamp_sample,
	.flow_xy_rad = Vector2f{-optical_flow.pixel_flow[0], -optical_flow.pixel_flow[1]},
	.gyro_xyz = Vector3f{-optical_flow.delta_angle[0], -optical_flow.delta_angle[1], -optical_flow.delta_angle[2]},
	.dt = 1e-6f * (float)optical_flow.integration_timespan_us,
	.quality = optical_flow.quality,
};
```

然后写入 EKF：

```cpp
_ekf.setOpticalFlowData(flow);
```

`flowSample` 的定义位于：

```text
src/modules/ekf2/EKF/common.h
```

其中：

```cpp
Vector2f flow_xy_rad; // image delta angle, rad
Vector3f gyro_xyz;    // gyro delta angle, rad
float dt;             // integration time, sec
uint8_t quality;
```

## 5. EKF 中的角速度补偿与姿态修正

EKF 光流控制入口：

```text
src/modules/ekf2/EKF/optical_flow_control.cpp
Ekf::controlOpticalFlowFusion()
```

### 5.1 角速度补偿

EKF 首先调用：

```cpp
const bool is_body_rate_comp_available = calcOptFlowBodyRateComp();
```

位置：

```text
src/modules/ekf2/EKF/optflow_fusion.cpp
Ekf::calcOptFlowBodyRateComp()
```

如果光流传感器自带 gyro，EKF 会将光流 gyro 与 EKF 自身 IMU 积分作比较，估计 `_flow_gyro_bias` 并扣除：

```cpp
const Vector3f reference_body_rate(_imu_del_ang_of * (1.0f / _delta_time_of));
const Vector3f measured_body_rate(_flow_sample_delayed.gyro_xyz * (1.0f / _flow_sample_delayed.dt));

_flow_gyro_bias = _flow_gyro_bias * 0.99f
	+ matrix::constrain(measured_body_rate - reference_body_rate, -0.1f, 0.1f) * 0.01f;

_flow_sample_delayed.gyro_xyz -= (_flow_gyro_bias * _flow_sample_delayed.dt);
```

如果光流不带 gyro，则 EKF 使用自身 IMU 积分补齐：

```cpp
_flow_sample_delayed.gyro_xyz =
	-_imu_del_ang_of / _delta_time_of * _flow_sample_delayed.dt;
```

随后得到补偿后的光流积分：

```cpp
_flow_compensated_XY_rad =
	_flow_sample_delayed.flow_xy_rad - _flow_sample_delayed.gyro_xyz.xy();
```

### 5.2 姿态倾斜对距离的修正

EKF 不直接把距离传感器读数当作光流尺度，而是通过姿态矩阵 `_R_to_earth`、地形估计和传感器安装位置计算光流相机视场中心距离：

```cpp
const float height_above_gnd_est =
	math::max(_terrain_vpos - _state.pos(2) - pos_offset_earth(2),
	          fmaxf(_params.rng_gnd_clearance, 0.01f));

return height_above_gnd_est / _R_to_earth(2, 2);
```

位置：

```text
src/modules/ekf2/EKF/optflow_fusion.cpp
Ekf::predictFlowRange()
```

这里 `_R_to_earth(2, 2)` 包含 roll/pitch 倾斜影响。飞机倾斜越大，光流视场中心距离与垂直高度差别越大。

## 6. EKF 中由光流得到 body 与 local NE 速度

位置：

```text
src/modules/ekf2/EKF/optflow_fusion.cpp
Ekf::updateOptFlow()
```

先计算 LOS rate：

```cpp
const Vector2f opt_flow_rate = _flow_compensated_XY_rad / _flow_sample_delayed.dt;
```

再计算 body 系速度：

```cpp
_flow_vel_body(0) = -opt_flow_rate(1) * range;
_flow_vel_body(1) =  opt_flow_rate(0) * range;
```

再通过 EKF 当前姿态矩阵转到 local NED 的 NE 分量：

```cpp
_flow_vel_ne = Vector2f(_R_to_earth *
	Vector3f(_flow_vel_body(0), _flow_vel_body(1), 0.f));
```

需要注意：

- `_flow_vel_body` 是由补偿光流和 range 推导出的机体系水平速度。
- `_flow_vel_ne` 是用 EKF 当前完整姿态旋到 local NED 后的水平速度。
- 这里的 `_R_to_earth` 包含 roll、pitch、yaw。
- 这个速度主要用于日志和必要时 reset 估计器水平速度，不是最终直接发布给控制器的速度。

## 7. yaw 校准与姿态来源

需要区分两个概念：

### 7.1 光流安装 yaw/旋转校正

这一步由 `SENS_FLOW_ROT` 完成，发生在 `VehicleOpticalFlow`，作用是把传感器测量旋到飞机 body frame。

### 7.2 EKF 航向 yaw alignment

EKF 的航向估计不是由光流本身完成，而是来自：

- 磁力计 yaw
- GPS yaw
- external vision yaw
- GPS/IMU GSF yaw
- 其他可用航向源

EKF 发布 local position 时会带出当前 heading：

```cpp
lpos.heading = Eulerf(_ekf.getQuaternion()).psi();
lpos.heading_good_for_control = _ekf.isYawFinalAlignComplete();
```

如果 yaw 未对齐，body 速度旋到 NED 的方向也会不准。PX4 使用 `heading_good_for_control` 表明当前 heading 是否已经适合控制。

## 8. 光流如何进入 EKF 融合

`Ekf::updateOptFlow()` 并不是简单把光流速度当成最终速度，而是构造 LOS rate 观测和创新：

```cpp
aid_src.observation[0] = opt_flow_rate(0);
aid_src.observation[1] = opt_flow_rate(1);

aid_src.innovation[0] =  (vel_body(1) / range) - aid_src.observation[0];
aid_src.innovation[1] = (-vel_body(0) / range) - aid_src.observation[1];
```

其中 `vel_body` 来自 EKF 当前状态预测：

```cpp
const Vector2f vel_body = predictFlowVelBody();
```

随后 `fuseOptFlow()` 对两个轴依次进行创新一致性检查和 Kalman update：

```cpp
setEstimatorAidStatusTestRatio(_aid_src_optical_flow, math::max(_params.flow_innov_gate, 1.f));
measurementUpdate(Kfusion, innovation_variance, innovation);
```

如果 EKF 刚开始使用光流，并且没有其他水平辅助源，PX4 会用 `_flow_vel_ne` reset 水平速度：

```cpp
resetHorizontalVelocityTo(_flow_vel_ne, calcOptFlowMeasVar(_flow_sample_delayed));
```

之后速度状态由 EKF 持续融合光流 LOS rate、IMU、range/terrain、GPS/EV 等信息共同估计。

## 9. 最终世界系速度发布

真正用于控制和系统输出的世界系速度来自 EKF 状态：

```text
src/modules/ekf2/EKF2.cpp
EKF2::PublishLocalPosition()
```

关键代码：

```cpp
const Vector3f velocity{_ekf.getVelocity()};
lpos.vx = velocity(0);
lpos.vy = velocity(1);
lpos.vz = velocity(2);
_local_position_pub.publish(lpos);
```

发布话题：

```text
vehicle_local_position
```

其中 `vx/vy/vz` 是 local NED 世界系速度。

同时，`vehicle_odometry` 也发布 NED 速度：

```text
src/modules/ekf2/EKF2.cpp
EKF2::PublishOdometry()
```

关键代码：

```cpp
odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
_ekf.getVelocity().copyTo(odom.velocity);
_odometry_pub.publish(odom);
```

如果启用多 EKF 实例，`EKF2Selector` 会将选中的：

```text
estimator_local_position -> vehicle_local_position
estimator_odometry       -> vehicle_odometry
```

## 10. 关键结论

1. MTF01/光流数据最终统一进入 `sensor_optical_flow`。
2. `VehicleOpticalFlow` 负责累计光流、同步 gyro/range、应用 `SENS_FLOW_ROT` 安装方向校正，并发布 `vehicle_optical_flow`。
3. 机体系速度计算的核心公式是：

```text
v_body_x = -range * compensated_flow_y / dt
v_body_y =  range * compensated_flow_x / dt
```

4. 光流角速度补偿为：

```text
compensated_flow_xy = flow_xy - gyro_xy
```

5. body 到 world/local NED 的转换使用 EKF 当前姿态矩阵：

```text
vel_ne = R_to_earth * [v_body_x, v_body_y, 0]
```

6. `SENS_FLOW_ROT` 只校正光流传感器安装旋转；EKF 的 yaw alignment 来自磁力计、GPS yaw、EV yaw 或 GPS/IMU GSF yaw 等航向源。
7. 光流速度不是直接作为最终速度发布，而是作为 LOS rate 观测进入 EKF 融合。
8. 最终发布给控制器和系统的是 EKF 状态速度：

```text
vehicle_local_position.vx/vy/vz
vehicle_odometry.velocity
```

## 关键代码索引

```text
src/modules/mavlink/mavlink_receiver.cpp
  MavlinkReceiver::handle_message_optical_flow_rad()

src/drivers/optical_flow/thoneflow/thoneflow_parser.cpp
  thoneflow_parse()

src/drivers/optical_flow/thoneflow/thoneflow.cpp
  ThoneFlow driver publishes sensor_optical_flow

src/modules/sensors/vehicle_optical_flow/VehicleOpticalFlow.cpp
  VehicleOpticalFlow::Run()
  SENS_FLOW_ROT correction
  vehicle_optical_flow_vel logging velocity

src/modules/ekf2/EKF2.cpp
  EKF2::UpdateOpticalFlow()
  EKF2::PublishLocalPosition()
  EKF2::PublishOdometry()

src/modules/ekf2/EKF/optical_flow_control.cpp
  Ekf::controlOpticalFlowFusion()

src/modules/ekf2/EKF/optflow_fusion.cpp
  Ekf::calcOptFlowBodyRateComp()
  Ekf::updateOptFlow()
  Ekf::fuseOptFlow()
  Ekf::predictFlowRange()
  Ekf::predictFlowVelBody()

src/modules/ekf2/EKF2Selector.cpp
  estimator_local_position -> vehicle_local_position
  estimator_odometry -> vehicle_odometry

msg/VehicleOpticalFlow.msg
msg/VehicleOpticalFlowVel.msg
msg/SensorOpticalFlow.msg
```
