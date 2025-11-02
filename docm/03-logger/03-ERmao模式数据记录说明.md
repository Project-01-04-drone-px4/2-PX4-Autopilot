# ERmao 模式数据记录说明

## 一、概述

ERmao 模式是专门为记录完整的双闭环（姿态环 + 角速度环）控制链路数据而设计的日志模式。本模式记录从传感器原始数据到执行器输出的完整信号链路，用于控制性能分析、系统辨识和故障诊断。

## 二、数据记录架构

### 2.1 数据流向

```
传感器层 → 预处理层 → 状态估计层 → 外环控制（姿态环）→ 内环控制（角速度环）→ 执行器层
```

### 2.2 双闭环结构

```
                    ┌─────────────────────────────────────────────────┐
                    │              外环（姿态环）                        │
                    │                                                 │
位置控制器 ─────→ vehicle_attitude_setpoint ─────→ 姿态控制器 ─────→ vehicle_rates_setpoint
或遥控器           (四元数 + 欧拉角)                  ↑                (roll/pitch/yaw, rad/s)
                                                     │                         │
                                           vehicle_attitude                   │
                                        (四元数 + 欧拉角, rad)                 │
                                                     ↑                         ↓
                                                     │              ┌──────────────────────┐
                                            状态估计器 ←────        │    内环（角速度环）    │
                                                          │        │                      │
                                                          │        └──────────────────────┘
                                                          │                 ↓
                                                          │        角速度控制器
                                                          │                 ↓
sensor_gyro_fifo ──→ vehicle_angular_velocity ──────────┘        actuator_motors
  (原始数据)         (滤波后角速度, rad/s)                          (电机输出)
```

## 三、记录的 uORB 主题详解

### 3.1 原始传感器数据层

#### sensor_gyro_fifo - 原始陀螺仪 FIFO 数据

**数据格式**：
```c
struct sensor_gyro_fifo_s {
    uint64 timestamp;          // 消息发布时间戳 (μs)
    uint64 timestamp_sample;   // 第一个采样的时间戳 (μs)
    uint32 device_id;          // 传感器唯一ID
    float32 dt;                // 采样间隔 (μs)
    float32 scale;             // 缩放因子（将 int16 转换为 rad/s）
    uint8 samples;             // 有效样本数（0-32）
    int16[32] x;               // X 轴原始数据（需乘以 scale）
    int16[32] y;               // Y 轴原始数据（需乘以 scale）
    int16[32] z;               // Z 轴原始数据（需乘以 scale）
};
```

**采样率**：1000-8000 Hz（取决于 IMU 型号，BMI270 为 1600 Hz）

**单位转换**：
```python
# 将原始 int16 数据转换为物理单位 (rad/s)
gyro_x_rad_s = sensor_gyro_fifo.x * sensor_gyro_fifo.scale
gyro_y_rad_s = sensor_gyro_fifo.y * sensor_gyro_fifo.scale
gyro_z_rad_s = sensor_gyro_fifo.z * sensor_gyro_fifo.scale
```

**用途**：
- 最原始的角速度数据，用于信号链路分析
- 振动诊断（配合 sensor_gyro_fft）
- 滤波器设计与验证

#### sensor_gyro_fft - 陀螺仪 FFT 频谱数据

**采样率**：50 Hz（频谱更新率）

**用途**：
- 振动频谱分析
- 检测机械共振、电机不平衡、桨叶损坏等问题
- 识别振动频率和幅值

---

### 3.2 预处理后的 IMU 数据层

#### vehicle_angular_velocity - 滤波后的角速度（内环实际值）

**数据格式**：
```c
struct vehicle_angular_velocity_s {
    uint64 timestamp;           // 时间戳 (μs)
    uint64 timestamp_sample;    // 采样时间戳 (μs)
    float32[3] xyz;             // 角速度 [roll, pitch, yaw] (rad/s)
    float32[3] xyz_derivative;  // 角加速度 [roll, pitch, yaw] (rad/s²)
};
```

**采样率**：~667 Hz

**特性**：
- 已应用陀螺仪偏差补偿（零偏校正）
- 已应用低通滤波（抑制高频噪声）
- 已完成坐标系转换（传感器坐标系 → 机体坐标系）

**单位**：rad/s（与 `vehicle_rates_setpoint` 统一）

**用途**：
- 角速度闭环控制的实际反馈值
- 与 `vehicle_rates_setpoint` 对比分析内环跟踪性能

#### vehicle_imu - 积分后的 IMU 数据

**采样率**：~250 Hz

**用途**：
- 姿态解算的输入数据
- 包含角度增量和速度增量

#### sensor_combined - 多传感器融合数据

**采样率**：~1000 Hz

**用途**：
- EKF 输入
- 状态估计

---

### 3.3 状态估计层（外环实际值）

#### vehicle_attitude - 姿态估计结果

**数据格式**：
```c
struct vehicle_attitude_s {
    uint64 timestamp;           // 时间戳 (μs)
    uint64 timestamp_sample;    // 采样时间戳 (μs)
    float32[4] q;               // 姿态四元数 [w, x, y, z] (Hamilton 约定)
    float32[4] delta_q_reset;   // 四元数重置变化量
    uint8 quat_reset_counter;   // 四元数重置计数器

    // 欧拉角（从四元数转换而来，便于分析）
    float32 roll;               // 横滚角 (rad)
    float32 pitch;              // 俯仰角 (rad)
    float32 yaw;                // 偏航角 (rad)
};
```

**采样率**：~250 Hz

**坐标系**：
- 四元数 `q` 表示从 FRD 机体坐标系到 NED 地球坐标系的旋转
- 欧拉角按照 ZYX 欧拉角序列（Yaw-Pitch-Roll）

**用途**：
- 外环（姿态环）的实际反馈值
- 与 `vehicle_attitude_setpoint` 对比分析外环跟踪性能

---

### 3.4 控制设定值层

#### vehicle_attitude_setpoint - 姿态期望值（外环设定值）

**数据格式**（已扩展）：
```c
struct vehicle_attitude_setpoint_s {
    uint64 timestamp;              // 时间戳 (μs)
    float32 yaw_sp_move_rate;      // 偏航速率命令 (rad/s)
    float32[4] q_d;                // 期望姿态四元数 [w, x, y, z]

    // ✨ 新增：欧拉角（从四元数转换而来，便于日志分析）
    float32 roll_body;             // 横滚角期望值 (rad)
    float32 pitch_body;            // 俯仰角期望值 (rad)
    float32 yaw_body;              // 偏航角期望值 (rad)

    float32[3] thrust_body;        // 归一化推力 [-1, 1]
};
```

**采样率**：全速率（跟随控制器更新，通常 ~250 Hz）

**来源**：
- **位置控制模式**：由位置控制器根据位置误差计算
- **姿态控制模式**：由遥控器摇杆输入转换
- **外部指令**：通过 MAVLink 接收（OFFBOARD 模式）

**用途**：
- 外环（姿态环）的期望值
- 与 `vehicle_attitude` 对比可分析姿态跟踪误差

**重要修改**：
- **原先**：只包含四元数 `q_d`
- **现在**：增加了欧拉角字段 `roll_body`, `pitch_body`, `yaw_body`
- **优势**：方便直接对比期望值与实际值，无需后处理时再转换四元数

#### vehicle_rates_setpoint - 角速度期望值（内环设定值）

**数据格式**：
```c
struct vehicle_rates_setpoint_s {
    uint64 timestamp;           // 时间戳 (μs)

    // 机体坐标系下的角速度设定值 (FRD 坐标系)
    float32 roll;               // 横滚角速度期望值 (rad/s)
    float32 pitch;              // 俯仰角速度期望值 (rad/s)
    float32 yaw;                // 偏航角速度期望值 (rad/s)

    float32[3] thrust_body;     // 归一化推力 [-1, 1]
    bool reset_integral;        // 积分项重置标志
};
```

**采样率**：全速率（跟随控制器更新，通常 ~250 Hz）

**来源**：
- 由姿态控制器根据姿态误差计算（PID 或其他控制算法）
- 公式（以 roll 为例）：
  ```
  roll_rate_sp = Kp * (roll_sp - roll_actual) + Ki * ∫(roll_sp - roll_actual)dt
  ```

**用途**：
- 内环（角速度环）的期望值
- 与 `vehicle_angular_velocity` 对比可分析角速度跟踪误差

---

### 3.5 执行器输出层

#### actuator_motors - 电机 PWM/DShot 输出

**数据格式**：
```c
struct actuator_motors_s {
    uint64 timestamp;           // 时间戳 (μs)
    uint64 timestamp_sample;    // 采样时间戳 (μs)
    bool reversible_flags;      // 可逆标志
    float32[16] control;        // 归一化电机输出 [-1, 1]
};
```

**采样率**：~250 Hz

**用途**：
- 分析控制指令到执行器的映射关系
- 评估混控矩阵的效果
- 检测电机饱和、输出限幅等问题

---

## 四、维度统一性分析

### 4.1 角速度维度统一性 ✓

| 主题                        | 字段              | 原始单位     | 统一后单位 | 转换方式                    |
|-----------------------------|-------------------|--------------|------------|-----------------------------|
| `sensor_gyro_fifo`          | `x/y/z` (int16)   | 原始计数值   | rad/s      | `raw * scale`               |
| `vehicle_angular_velocity`  | `xyz` (float32)   | rad/s        | rad/s      | 无需转换（已统一）          |
| `vehicle_rates_setpoint`    | `roll/pitch/yaw`  | rad/s        | rad/s      | 无需转换（已统一）          |

**结论**：所有角速度数据在后处理时统一为 `rad/s`，可直接对比分析。

### 4.2 姿态角度维度统一性 ✓

| 主题                        | 字段                         | 单位  | 说明                           |
|-----------------------------|------------------------------|-------|--------------------------------|
| `vehicle_attitude`          | `roll/pitch/yaw` (float32)   | rad   | 实际姿态（从四元数转换）       |
| `vehicle_attitude_setpoint` | `roll_body/pitch_body/yaw_body` (float32) | rad   | 期望姿态（从四元数转换）✨ 新增 |

**结论**：姿态期望值和实际值均包含欧拉角，单位统一为弧度（rad），可直接对比分析。

---

## 五、数据分析指南

### 5.1 内环（角速度环）性能分析

**分析目标**：评估角速度控制器的跟踪性能

**对比数据**：
- 期望值：`vehicle_rates_setpoint` 的 `roll/pitch/yaw` (rad/s)
- 实际值：`vehicle_angular_velocity` 的 `xyz[0]/xyz[1]/xyz[2]` (rad/s)

**分析指标**：
```python
# 计算角速度跟踪误差
error_roll  = rates_setpoint.roll  - angular_velocity.xyz[0]
error_pitch = rates_setpoint.pitch - angular_velocity.xyz[1]
error_yaw   = rates_setpoint.yaw   - angular_velocity.xyz[2]

# 评估指标
mean_error = np.mean(error_roll)       # 平均误差（稳态误差）
std_error  = np.std(error_roll)        # 标准差（跟踪精度）
max_error  = np.max(np.abs(error_roll))# 最大误差（瞬态性能）
```

**典型问题诊断**：
- **平均误差大**：可能存在零偏、积分饱和
- **标准差大**：控制器增益不足、响应慢
- **最大误差大**：瞬态响应不足、超调过大

### 5.2 外环（姿态环）性能分析

**分析目标**：评估姿态控制器的跟踪性能

**对比数据**：
- 期望值：`vehicle_attitude_setpoint` 的 `roll_body/pitch_body/yaw_body` (rad)
- 实际值：`vehicle_attitude` 的 `roll/pitch/yaw` (rad)

**分析指标**：
```python
# 计算姿态跟踪误差
error_roll  = attitude_setpoint.roll_body  - attitude.roll
error_pitch = attitude_setpoint.pitch_body - attitude.pitch
error_yaw   = wrap_pi(attitude_setpoint.yaw_body - attitude.yaw)  # 注意偏航角环绕

# 评估指标
settling_time = calculate_settling_time(error_roll, threshold=0.05)  # 建立时间
overshoot     = calculate_overshoot(error_roll)                      # 超调量
rise_time     = calculate_rise_time(error_roll)                      # 上升时间
```

**典型问题诊断**：
- **建立时间长**：外环增益不足、阻尼过大
- **超调量大**：外环增益过大、阻尼不足
- **振荡**：内外环耦合、参数不匹配

### 5.3 振动分析

**分析目标**：识别机械振动源

**数据来源**：
- `sensor_gyro_fifo`：原始高频数据
- `sensor_gyro_fft`：频谱数据

**分析方法**：
```python
# 方法1：直接使用 FFT 主题
dominant_freq = sensor_gyro_fft.peak_frequencies_x[0]  # 主导频率
dominant_amp  = sensor_gyro_fft.peak_magnitudes_x[0]   # 主导幅值

# 方法2：自行计算 FFT
gyro_data = sensor_gyro_fifo.x * sensor_gyro_fifo.scale
fft_result = np.fft.fft(gyro_data)
freq_axis  = np.fft.fftfreq(len(gyro_data), d=sensor_gyro_fifo.dt * 1e-6)
```

**常见振动频率**：
- **电机振动**：通常在 100-300 Hz
- **桨叶振动**：取决于电机转速（RPM / 60 * 桨叶数）
- **机架共振**：通常在 50-150 Hz

### 5.4 信号链路延迟分析

**分析目标**：评估端到端延迟

**关键时间戳**：
- `sensor_gyro_fifo.timestamp_sample`：传感器采样时刻
- `vehicle_angular_velocity.timestamp_sample`：滤波后数据时刻
- `vehicle_rates_setpoint.timestamp`：控制指令生成时刻
- `actuator_motors.timestamp`：电机输出时刻

**延迟计算**：
```python
# 传感器到状态估计延迟
delay_sensing = vehicle_angular_velocity.timestamp - sensor_gyro_fifo.timestamp_sample

# 控制计算延迟
delay_control = actuator_motors.timestamp - vehicle_rates_setpoint.timestamp

# 端到端延迟
delay_total = actuator_motors.timestamp - sensor_gyro_fifo.timestamp_sample
```

---

## 六、推荐分析工具

### 6.1 PX4 官方工具

- **Flight Review**：在线日志分析平台 (https://logs.px4.io/)
- **pyulog**：Python 日志解析库

### 6.2 自定义 Python 脚本

```python
from pyulog import ULog

# 加载日志文件
ulog = ULog('log.ulg')

# 提取数据
attitude_sp = ulog.get_dataset('vehicle_attitude_setpoint').data
attitude    = ulog.get_dataset('vehicle_attitude').data
rates_sp    = ulog.get_dataset('vehicle_rates_setpoint').data
angular_vel = ulog.get_dataset('vehicle_angular_velocity').data

# 时间对齐（使用最近邻插值）
import pandas as pd
df = pd.DataFrame({
    'time': attitude_sp['timestamp'],
    'roll_sp': attitude_sp['roll_body'],      # ✨ 新增字段
    'roll_actual': np.interp(attitude_sp['timestamp'],
                              attitude['timestamp'],
                              attitude['roll'])
})

# 计算误差
df['roll_error'] = df['roll_sp'] - df['roll_actual']

# 绘图分析
import matplotlib.pyplot as plt
plt.plot(df['time'], df['roll_sp'], label='Setpoint')
plt.plot(df['time'], df['roll_actual'], label='Actual')
plt.plot(df['time'], df['roll_error'], label='Error')
plt.legend()
plt.show()
```

---

## 七、修改总结

### 7.1 消息定义修改

**文件**：`msg/versioned/VehicleAttitudeSetpoint.msg`

**修改内容**：
- 增加 `MESSAGE_VERSION` 从 1 → 2
- 新增字段：
  ```c
  float32 roll_body    # Roll angle setpoint (rad)
  float32 pitch_body   # Pitch angle setpoint (rad)
  float32 yaw_body     # Yaw angle setpoint (rad)
  ```

**影响**：
- 所有生成 `vehicle_attitude_setpoint` 的控制器需要填充欧拉角字段
- 日志文件中可直接读取欧拉角，无需后处理转换

### 7.2 控制器代码修改

已修改的控制器（共 7 个）：

1. **多旋翼姿态控制器**
   文件：`src/modules/mc_att_control/mc_att_control_main.cpp`

2. **固定翼姿态控制器**
   文件：`src/modules/fw_att_control/FixedwingAttitudeControl.cpp`

3. **多旋翼位置控制器**
   文件：`src/modules/mc_pos_control/PositionControl/ControlMath.cpp`

4. **航天器位置控制器**
   文件：`src/modules/spacecraft/SpacecraftPositionControl/PositionControl/ControlMath.cpp`

5. **航天器姿态控制器**
   文件：`src/modules/spacecraft/SpacecraftAttitudeControl/SpacecraftAttitudeControl.cpp`

6. **水下无人机姿态控制器**
   文件：`src/modules/uuv_att_control/uuv_att_control.cpp`

7. **固定翼横向纵向控制器**
   文件：`src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp`

8. **MAVLink 接收器**
   文件：`src/modules/mavlink/mavlink_receiver.cpp`

**统一转换方法**：
```cpp
// 从四元数转换为欧拉角
const Eulerf euler_sp(q_sp);
attitude_setpoint.roll_body  = euler_sp.phi();
attitude_setpoint.pitch_body = euler_sp.theta();
attitude_setpoint.yaw_body   = euler_sp.psi();
```

### 7.3 日志配置修改

**文件**：`src/modules/logger/logged_topics.cpp`

**修改内容**：
- 优化 `add_ermao_topics()` 函数注释
- 添加详细的数据流向说明
- 添加维度统一性说明
- 添加数据分析建议

---

## 八、编译与测试

### 8.1 编译步骤

```bash
cd /home/gg/01-code/01-px4/01-fork-gg/2-PX4-Autopilot
make clean
make px4_sitl_default  # 或者你的目标平台
```

### 8.2 测试验证

1. **启动仿真**：
   ```bash
   make px4_sitl gazebo
   ```

2. **启用 ERmao 日志模式**：
   ```bash
   # 通过 MAVLink 控制台或 QGC 参数设置
   param set SDLOG_PROFILE 4096  # ERmao 模式对应的位掩码
   ```

3. **执行飞行测试**：
   - 切换到手动模式（Manual）或姿态模式（Stabilized）
   - 执行打杆操作，测试姿态响应

4. **检查日志**：
   ```bash
   # 下载日志文件
   # 上传到 https://logs.px4.io/ 或使用 pyulog 分析

   # 验证新字段
   python3 << EOF
   from pyulog import ULog
   ulog = ULog('log.ulg')
   att_sp = ulog.get_dataset('vehicle_attitude_setpoint').data
   print("New fields:", att_sp.keys())
   # 应该包含 'roll_body', 'pitch_body', 'yaw_body'
   EOF
   ```

---

## 九、常见问题

### Q1：为什么不直接记录欧拉角，还要保留四元数？

**A**：四元数和欧拉角各有优势：
- **四元数**：无奇异性（万向锁），控制算法内部使用
- **欧拉角**：直观易懂，便于人类分析和可视化

因此两者都保留，控制器内部使用四元数，日志分析使用欧拉角。

### Q2：sensor_gyro_fifo 的 scale 字段是什么？

**A**：`scale` 是将 int16 原始数据转换为物理单位（rad/s）的缩放因子。

转换公式：
```
gyro_rad_s = raw_int16 * scale
```

对于 BMI270，scale 通常为 0.00053263 (对应 ±2000 °/s 量程)。

### Q3：如何判断角速度的期望值和实际值是否在同一维度？

**A**：通过数据分析验证：

```python
from pyulog import ULog
import numpy as np

ulog = ULog('log.ulg')
rates_sp = ulog.get_dataset('vehicle_rates_setpoint').data
angular_vel = ulog.get_dataset('vehicle_angular_velocity').data

# 检查数值范围（应该都在 ±10 rad/s 以内，常规飞行）
print("Rates setpoint range:", np.min(rates_sp['roll']), np.max(rates_sp['roll']))
print("Angular velocity range:", np.min(angular_vel['xyz'][:, 0]), np.max(angular_vel['xyz'][:, 0]))

# 如果数值范围相近（同一数量级），说明单位一致
```

### Q4：如何在日志中快速定位控制性能问题？

**A**：按照以下步骤：

1. **先看外环**：对比 `vehicle_attitude_setpoint` 和 `vehicle_attitude`
   - 如果外环误差大 → 调整姿态控制器参数

2. **再看内环**：对比 `vehicle_rates_setpoint` 和 `vehicle_angular_velocity`
   - 如果内环误差大 → 调整角速度控制器参数

3. **检查振动**：查看 `sensor_gyro_fft`
   - 如果振动过大 → 检查机械结构、滤波器设置

---

## 十、参考资料

- [PX4 开发指南 - 日志系统](https://docs.px4.io/main/en/dev_log/logging.html)
- [PX4 开发指南 - uORB 消息](https://docs.px4.io/main/en/middleware/uorb.html)
- [四元数与欧拉角转换](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
- [PX4 姿态控制架构](https://docs.px4.io/main/en/flight_stack/controller_diagrams.html)

---

**文档版本**：1.0
**最后更新**：2025-11-01
**作者**：ERmao 模式开发团队

