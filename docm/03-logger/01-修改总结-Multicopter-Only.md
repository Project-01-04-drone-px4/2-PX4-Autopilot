# 修改总结 - 多旋翼专用版本

## 一、修改范围

本次修改**仅针对多旋翼（Multicopter）**，不涉及固定翼、VTOL、水下无人机等其他机型。

## 二、修改的文件列表（共 6 个）

### 1. 消息定义

**文件**: `msg/versioned/VehicleAttitudeSetpoint.msg`

**修改内容**:
```diff
+ uint32 MESSAGE_VERSION = 2  // 版本号从 1 升级到 2
+
+ // 新增欧拉角字段（从四元数转换）
+ float32 roll_body     # Roll angle setpoint (rad)
+ float32 pitch_body    # Pitch angle setpoint (rad)
+ float32 yaw_body      # Yaw angle setpoint (rad)
```

---

### 2. 多旋翼姿态控制器

**文件**: `src/modules/mc_att_control/mc_att_control_main.cpp`

**修改位置**: 第 193-207 行（`generate_attitude_setpoint` 函数）

**修改内容**:
```cpp
// 从四元数转换为欧拉角
const Eulerf euler_sp(q_sp);
attitude_setpoint.roll_body  = euler_sp.phi();
attitude_setpoint.pitch_body = euler_sp.theta();
attitude_setpoint.yaw_body   = euler_sp.psi();
```

---

### 3. 多旋翼位置控制器

**文件**: `src/modules/mc_pos_control/PositionControl/ControlMath.cpp`

**修改位置**: 第 111-119 行（`thrustToAttitude` 函数）

**修改内容**:
```cpp
// 从四元数转换为欧拉角
const Eulerf euler_sp(q_sp);
att_sp.roll_body  = euler_sp.phi();
att_sp.pitch_body = euler_sp.theta();
att_sp.yaw_body   = euler_sp.psi();
```

---

### 4. 日志配置 - 主文件

**文件**: `src/modules/logger/logged_topics.cpp`

**修改内容**:
- 新增 `add_ermao_topics()` 函数（第 602-700 行）
- 添加详细的英文注释，说明记录的每个主题的用途
- 包含数据流向、维度统一性说明、数据分析建议

---

### 5. 日志配置 - 头文件

**文件**: `src/modules/logger/logged_topics.h`

**修改内容**:
```cpp
enum class SDLogProfileMask : int32_t {
    // ... 其他模式 ...
    ERMAO = 1 << 12  // 新增 ERmao 模式（位 12）
};

// 新增函数声明
void add_ermao_topics();
```

---

### 6. 日志配置 - 参数定义

**文件**: `src/modules/logger/module.yaml`

**修改内容**:
- 参数 `SDLOG_PROFILE` 新增 bit 12: "ERmao mode (full rate IMU signal chain)"
- 参数最大值从 4095 增加到 8191

---

## 三、记录的数据主题

### 完整的双闭环控制数据

```
传感器层 → 预处理层 → 状态估计层 → 外环（姿态环） → 内环（角速度环） → 执行器层
```

| 层级           | 主题名称                      | 采样率  | 字段                          | 单位   |
|---------------|-------------------------------|---------|-------------------------------|--------|
| 传感器层       | `sensor_gyro_fifo`            | 1600 Hz | `x/y/z * scale`               | rad/s  |
| 传感器层       | `sensor_gyro_fft`             | 50 Hz   | 频谱数据                      | -      |
| 预处理层       | `vehicle_angular_velocity`    | 667 Hz  | `xyz[0/1/2]` (内环实际值)     | rad/s  |
| 预处理层       | `vehicle_imu`                 | 250 Hz  | 积分 IMU 数据                 | -      |
| 预处理层       | `sensor_combined`             | 1000 Hz | 融合传感器数据                | -      |
| 状态估计层     | `vehicle_attitude`            | 250 Hz  | `roll/pitch/yaw` (外环实际值) | rad    |
| 控制层         | `vehicle_attitude_setpoint`   | 全速率  | `roll_body/pitch_body/yaw_body` (外环设定值) ✨ | rad |
| 控制层         | `vehicle_rates_setpoint`      | 全速率  | `roll/pitch/yaw` (内环设定值) | rad/s  |
| 执行器层       | `actuator_motors`             | 250 Hz  | 电机输出 `[-1, 1]`            | -      |

✨ = 本次新增的欧拉角字段

---

## 四、维度统一性验证

### ✅ 角速度维度统一

所有角速度数据统一为 **rad/s**：
- `sensor_gyro_fifo` 的 `x/y/z * scale` → **rad/s**
- `vehicle_angular_velocity` 的 `xyz[0/1/2]` → **rad/s**
- `vehicle_rates_setpoint` 的 `roll/pitch/yaw` → **rad/s**

### ✅ 姿态角度维度统一

所有姿态角度数据统一为 **rad**：
- `vehicle_attitude` 的 `roll/pitch/yaw` → **rad** (实际值，已有)
- `vehicle_attitude_setpoint` 的 `roll_body/pitch_body/yaw_body` → **rad** (期望值，✨ 新增)

---

## 五、编译与测试

### 编译

```bash
cd /home/gg/01-code/01-px4/01-fork-gg/2-PX4-Autopilot
make clean
make px4_sitl_default  # SITL 仿真
# 或
make px4_fmu-v6x       # 真机硬件（根据你的飞控型号）
```

### 启用 ERmao 日志模式

通过 QGroundControl 参数设置或 MAVLink 控制台：

```bash
param set SDLOG_PROFILE 4096  # 单独启用 ERmao 模式
# 或
param set SDLOG_PROFILE 4097  # 启用 DEFAULT (1) + ERMAO (4096)
param save
reboot
```

### 验证日志

```python
from pyulog import ULog

ulog = ULog('log.ulg')
att_sp = ulog.get_dataset('vehicle_attitude_setpoint').data

# 检查新字段
print("Available fields:", att_sp.dtype.names)
# 应该包含: 'roll_body', 'pitch_body', 'yaw_body'

# 验证数据
print("Roll setpoint sample:", att_sp['roll_body'][0])
```

---

## 六、数据分析示例

### 内环（角速度环）性能分析

```python
import numpy as np
from pyulog import ULog

ulog = ULog('log.ulg')
rates_sp = ulog.get_dataset('vehicle_rates_setpoint').data
angular_vel = ulog.get_dataset('vehicle_angular_velocity').data

# 时间对齐（简单插值）
time_base = rates_sp['timestamp']
rate_actual = np.interp(time_base, angular_vel['timestamp'], angular_vel['xyz'][:, 0])

# 计算跟踪误差
error = rates_sp['roll'] - rate_actual

# 性能指标
print(f"内环角速度跟踪误差:")
print(f"  平均误差: {np.rad2deg(np.mean(error)):.3f} deg/s")
print(f"  标准差:   {np.rad2deg(np.std(error)):.3f} deg/s")
print(f"  最大误差: {np.rad2deg(np.max(np.abs(error))):.3f} deg/s")
```

### 外环（姿态环）性能分析

```python
attitude_sp = ulog.get_dataset('vehicle_attitude_setpoint').data
attitude = ulog.get_dataset('vehicle_attitude').data

# 时间对齐
time_base = attitude_sp['timestamp']
att_actual = np.interp(time_base, attitude['timestamp'], attitude['roll'])

# 计算跟踪误差
error = attitude_sp['roll_body'] - att_actual  # ✨ 使用新增的欧拉角字段

# 性能指标
print(f"外环姿态跟踪误差:")
print(f"  平均误差: {np.rad2deg(np.mean(error)):.3f} deg")
print(f"  标准差:   {np.rad2deg(np.std(error)):.3f} deg")
print(f"  最大误差: {np.rad2deg(np.max(np.abs(error))):.3f} deg")
```

---

## 七、关键优势

### ✅ 不再需要后处理时转换四元数

**之前**:
```python
# 需要手动转换四元数到欧拉角
from scipy.spatial.transform import Rotation
q = attitude_sp['q_d']  # 四元数 [w, x, y, z]
euler = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_euler('xyz')
roll_sp = euler[0]  # 繁琐！
```

**现在**:
```python
# 直接读取欧拉角
roll_sp = attitude_sp['roll_body']  # 简单！
```

### ✅ 完整的双闭环控制数据

- **外环（姿态环）**: 期望值 vs 实际值 ✓
- **内环（角速度环）**: 期望值 vs 实际值 ✓
- **原始传感器数据**: FIFO + FFT ✓
- **执行器输出**: 电机指令 ✓

### ✅ 维度统一，直接对比

- 所有角速度: **rad/s**
- 所有姿态角: **rad**
- 无需单位转换

---

## 八、不受影响的机型

以下机型的控制器**未被修改**，不受本次改动影响：

- ❌ 固定翼 (Fixed-Wing)
- ❌ VTOL (垂直起降)
- ❌ 水下无人机 (UUV)
- ❌ 航天器 (Spacecraft)
- ❌ 地面车辆 (Rover)

这些机型的 `vehicle_attitude_setpoint` 中的欧拉角字段将为 0 或未定义值，但不影响正常飞行，因为控制器内部使用的是四元数 `q_d`。

---

## 九、注意事项

### 1. 消息版本升级

`VehicleAttitudeSetpoint` 的消息版本从 1 升级到 2，确保所有工具链支持新版本。

### 2. 日志文件大小

ERmao 模式记录全速率数据，日志文件会更大。建议：
- 使用高速 SD 卡（Class 10 或 UHS-I）
- 定期清理日志文件
- 仅在需要详细分析时启用

### 3. 编译依赖

如果编译失败，尝试：
```bash
make distclean
make px4_sitl_default
```

---

**修改完成时间**: 2025-11-01
**适用机型**: 仅多旋翼 (Multicopter)
**修改文件数**: 6 个核心文件
**新增字段**: 3 个欧拉角字段 (roll_body, pitch_body, yaw_body)

