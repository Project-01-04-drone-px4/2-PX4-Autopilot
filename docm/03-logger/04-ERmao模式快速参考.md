# ERmao 模式快速参考卡片

## 一、维度统一性速查表

### 角速度数据 ✓ 已统一为 rad/s

| uORB 主题                   | 字段名            | 原始单位      | 统一单位 | 转换公式                    | 备注                    |
|-----------------------------|-------------------|---------------|----------|-----------------------------|-------------------------|
| `sensor_gyro_fifo`          | `x/y/z` (int16)   | 原始计数值    | rad/s    | `gyro = raw * scale`        | 需手动缩放              |
| `vehicle_angular_velocity`  | `xyz[0/1/2]`      | rad/s         | rad/s    | 无需转换                    | 已滤波，已偏差补偿      |
| `vehicle_rates_setpoint`    | `roll/pitch/yaw`  | rad/s         | rad/s    | 无需转换                    | 控制器输出              |

### 姿态角度数据 ✓ 已统一为 rad

| uORB 主题                   | 字段名                         | 单位  | 说明                        |
|-----------------------------|--------------------------------|-------|-----------------------------|
| `vehicle_attitude`          | `roll/pitch/yaw`               | rad   | 实际姿态（从四元数转换）    |
| `vehicle_attitude_setpoint` | `roll_body/pitch_body/yaw_body`| rad   | 期望姿态（✨ 新增字段）     |

---

## 二、数据对比速查

### 内环（角速度环）对比

```python
# 期望值（来自姿态控制器）
setpoint_roll  = vehicle_rates_setpoint.roll      # rad/s
setpoint_pitch = vehicle_rates_setpoint.pitch     # rad/s
setpoint_yaw   = vehicle_rates_setpoint.yaw       # rad/s

# 实际值（来自陀螺仪滤波）
actual_roll  = vehicle_angular_velocity.xyz[0]    # rad/s
actual_pitch = vehicle_angular_velocity.xyz[1]    # rad/s
actual_yaw   = vehicle_angular_velocity.xyz[2]    # rad/s

# 跟踪误差
error_roll  = setpoint_roll  - actual_roll
error_pitch = setpoint_pitch - actual_pitch
error_yaw   = setpoint_yaw   - actual_yaw
```

### 外环（姿态环）对比

```python
# 期望值（来自位置控制器或遥控器）
setpoint_roll  = vehicle_attitude_setpoint.roll_body   # rad ✨ 新增
setpoint_pitch = vehicle_attitude_setpoint.pitch_body  # rad ✨ 新增
setpoint_yaw   = vehicle_attitude_setpoint.yaw_body    # rad ✨ 新增

# 实际值（来自状态估计器）
actual_roll  = vehicle_attitude.roll                   # rad
actual_pitch = vehicle_attitude.pitch                  # rad
actual_yaw   = vehicle_attitude.yaw                    # rad

# 跟踪误差（注意偏航角环绕）
error_roll  = setpoint_roll  - actual_roll
error_pitch = setpoint_pitch - actual_pitch
error_yaw   = wrap_pi(setpoint_yaw - actual_yaw)  # 处理 ±π 环绕
```

---

## 三、pyulog 数据提取模板

```python
from pyulog import ULog
import numpy as np
import matplotlib.pyplot as plt

# 加载日志
ulog = ULog('log.ulg')

# ============ 提取数据 ============

# 1. 内环数据
rates_sp    = ulog.get_dataset('vehicle_rates_setpoint').data
angular_vel = ulog.get_dataset('vehicle_angular_velocity').data

# 2. 外环数据
attitude_sp = ulog.get_dataset('vehicle_attitude_setpoint').data
attitude    = ulog.get_dataset('vehicle_attitude').data

# 3. 原始传感器数据
gyro_fifo   = ulog.get_dataset('sensor_gyro_fifo').data

# 4. 执行器输出
actuators   = ulog.get_dataset('actuator_motors').data

# ============ 时间对齐 ============

import pandas as pd

# 以 rates_setpoint 为基准时间轴
time_base = rates_sp['timestamp'] * 1e-6  # 转换为秒

df = pd.DataFrame({
    'time': time_base,

    # 内环数据
    'rate_sp_roll':  rates_sp['roll'],
    'rate_act_roll': np.interp(time_base, angular_vel['timestamp'] * 1e-6, angular_vel['xyz'][:, 0]),

    # 外环数据
    'att_sp_roll':  np.interp(time_base, attitude_sp['timestamp'] * 1e-6, attitude_sp['roll_body']),  # ✨ 新字段
    'att_act_roll': np.interp(time_base, attitude['timestamp'] * 1e-6, attitude['roll']),
})

# 计算误差
df['rate_error_roll'] = df['rate_sp_roll'] - df['rate_act_roll']
df['att_error_roll']  = df['att_sp_roll']  - df['att_act_roll']

# ============ 绘图分析 ============

fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

# 子图1：姿态期望 vs 实际
axes[0].plot(df['time'], np.rad2deg(df['att_sp_roll']),  label='Attitude Setpoint', linewidth=2)
axes[0].plot(df['time'], np.rad2deg(df['att_act_roll']), label='Attitude Actual', linewidth=1)
axes[0].set_ylabel('Roll Angle (deg)')
axes[0].legend()
axes[0].grid(True)
axes[0].set_title('Outer Loop: Attitude Tracking')

# 子图2：姿态误差
axes[1].plot(df['time'], np.rad2deg(df['att_error_roll']), color='red', linewidth=1)
axes[1].axhline(0, color='black', linestyle='--', linewidth=0.5)
axes[1].set_ylabel('Roll Error (deg)')
axes[1].grid(True)

# 子图3：角速度期望 vs 实际
axes[2].plot(df['time'], np.rad2deg(df['rate_sp_roll']),  label='Rate Setpoint', linewidth=2)
axes[2].plot(df['time'], np.rad2deg(df['rate_act_roll']), label='Rate Actual', linewidth=1)
axes[2].set_ylabel('Roll Rate (deg/s)')
axes[2].legend()
axes[2].grid(True)
axes[2].set_title('Inner Loop: Angular Velocity Tracking')

# 子图4：角速度误差
axes[3].plot(df['time'], np.rad2deg(df['rate_error_roll']), color='red', linewidth=1)
axes[3].axhline(0, color='black', linestyle='--', linewidth=0.5)
axes[3].set_ylabel('Roll Rate Error (deg/s)')
axes[3].set_xlabel('Time (s)')
axes[3].grid(True)

plt.tight_layout()
plt.show()

# ============ 性能指标计算 ============

# 内环性能
print("======== Inner Loop Performance ========")
print(f"Rate Error (mean):  {np.rad2deg(np.mean(df['rate_error_roll'])):.3f} deg/s")
print(f"Rate Error (std):   {np.rad2deg(np.std(df['rate_error_roll'])):.3f} deg/s")
print(f"Rate Error (max):   {np.rad2deg(np.max(np.abs(df['rate_error_roll']))):.3f} deg/s")

# 外环性能
print("\n======== Outer Loop Performance ========")
print(f"Attitude Error (mean):  {np.rad2deg(np.mean(df['att_error_roll'])):.3f} deg")
print(f"Attitude Error (std):   {np.rad2deg(np.std(df['att_error_roll'])):.3f} deg")
print(f"Attitude Error (max):   {np.rad2deg(np.max(np.abs(df['att_error_roll']))):.3f} deg")
```

---

## 四、sensor_gyro_fifo 缩放公式

```python
# 提取原始 FIFO 数据
gyro_fifo = ulog.get_dataset('sensor_gyro_fifo').data

# 获取第一条消息的缩放因子（通常是常数）
scale = gyro_fifo['scale'][0]

# 将 int16 原始数据转换为 rad/s
gyro_x_rad_s = []
for i in range(len(gyro_fifo['timestamp'])):
    samples = gyro_fifo['samples'][i]  # 有效样本数
    for j in range(samples):
        gyro_x_rad_s.append(gyro_fifo['x'][i][j] * scale)

# 或者使用 numpy（更高效）
all_x = []
for i in range(len(gyro_fifo['timestamp'])):
    samples = gyro_fifo['samples'][i]
    all_x.extend(gyro_fifo['x'][i][:samples] * gyro_fifo['scale'][i])

gyro_x_rad_s = np.array(all_x)
```

---

## 五、常见单位转换

| 从       | 到       | 公式                    |
|----------|----------|-------------------------|
| rad      | deg      | `deg = rad * 180 / π`   |
| deg      | rad      | `rad = deg * π / 180`   |
| rad/s    | deg/s    | `deg_s = rad_s * 180/π` |
| deg/s    | rad/s    | `rad_s = deg_s * π/180` |
| μs       | s        | `s = μs * 1e-6`         |
| s        | μs       | `μs = s * 1e6`          |

---

## 六、启用 ERmao 日志模式

### 方法1：通过 QGroundControl 参数

1. 连接飞控
2. 打开参数设置（Vehicle Setup → Parameters）
3. 搜索 `SDLOG_PROFILE`
4. 设置为 `4096`（ERmao 模式的位掩码）
5. 重启飞控

### 方法2：通过 MAVLink 控制台

```bash
# 在 QGC 的 MAVLink 控制台或 PX4 Shell 中执行
param set SDLOG_PROFILE 4096
param save
reboot
```

### 方法3：组合多个日志模式（位掩码）

```bash
# 同时启用 DEFAULT (1) + DEBUG (64) + ERMAO (4096)
param set SDLOG_PROFILE 4161  # 1 + 64 + 4096 = 4161
```

---

## 七、记录的主题列表（ERmao 模式）

| 层级               | 主题名称                      | 采样率    | 用途                          |
|--------------------|-------------------------------|-----------|-------------------------------|
| **传感器层**       | `sensor_gyro_fifo`            | 1600 Hz   | 原始陀螺仪数据（需缩放）      |
|                    | `sensor_gyro_fft`             | 50 Hz     | 频谱分析                      |
| **预处理层**       | `vehicle_angular_velocity`    | 667 Hz    | 滤波后角速度（内环实际值）    |
|                    | `vehicle_imu`                 | 250 Hz    | 积分 IMU 数据                 |
|                    | `sensor_combined`             | 1000 Hz   | 融合传感器数据                |
| **状态估计层**     | `vehicle_attitude`            | 250 Hz    | 姿态估计（外环实际值）        |
| **控制层**         | `vehicle_attitude_setpoint`   | 全速率    | 姿态期望值（外环设定值）✨    |
|                    | `vehicle_rates_setpoint`      | 全速率    | 角速度期望值（内环设定值）    |
| **执行器层**       | `actuator_motors`             | 250 Hz    | 电机输出                      |

✨ 新增欧拉角字段：`roll_body`, `pitch_body`, `yaw_body`

---

## 八、典型问题诊断决策树

```
飞行器姿态不稳定
    │
    ├─ 查看姿态跟踪误差 (vehicle_attitude_setpoint vs vehicle_attitude)
    │   │
    │   ├─ 误差大 → 外环问题
    │   │   ├─ 误差持续大 → 增加姿态控制器 P 增益
    │   │   ├─ 误差振荡    → 减小姿态控制器 P 增益
    │   │   └─ 响应慢      → 增加姿态控制器增益
    │   │
    │   └─ 误差小 → 继续检查内环
    │
    └─ 查看角速度跟踪误差 (vehicle_rates_setpoint vs vehicle_angular_velocity)
        │
        ├─ 误差大 → 内环问题
        │   ├─ 误差持续大 → 增加角速度控制器 P 增益
        │   ├─ 误差振荡    → 减小角速度控制器 D 增益
        │   └─ 响应慢      → 增加角速度控制器增益
        │
        └─ 误差小 → 检查振动
            │
            └─ 查看 sensor_gyro_fft
                ├─ 100-300 Hz 振动 → 电机/桨叶问题
                ├─ 50-150 Hz 振动  → 机架共振
                └─ 高频噪声        → 调整滤波器
```

---

## 九、快速检查清单

### 编译前检查

- [ ] 消息定义已修改 (`VehicleAttitudeSetpoint.msg`)
- [ ] 所有控制器已更新（8 个文件）
- [ ] 日志配置已优化 (`logged_topics.cpp`)

### 编译命令

```bash
cd /home/gg/01-code/01-px4/01-fork-gg/2-PX4-Autopilot
make clean
make px4_sitl_default  # SITL 仿真
# 或
make px4_fmu-v6x       # 真机硬件（根据你的飞控型号）
```

### 日志验证

```bash
# 检查日志中是否包含新字段
ulog_info log.ulg | grep vehicle_attitude_setpoint
# 应该显示 roll_body, pitch_body, yaw_body 字段

# 快速查看数据
ulog_params log.ulg
ulog_messages log.ulg
```

---

**版本**：1.0
**最后更新**：2025-11-01

