# Logger 日志记录模式切换指南
## 如何在不同的 Logger Profile 之间切换

---

## 1. 概述

PX4 的 Logger 模块支持多种日志记录配置文件（Profile），通过参数 `SDLOG_PROFILE` 来控制。这个参数采用**位掩码（Bitmask）**机制，允许同时启用多个配置文件。

### 1.1 核心信息

| 项目 | 信息 |
|------|------|
| **参数名称** | `SDLOG_PROFILE` |
| **参数类型** | 位掩码（Bitmask）整数 |
| **默认值** | `1` (仅 DEFAULT 模式) |
| **取值范围** | `0 - 4095` |
| **重启要求** | 需要重启才能生效 |
| **配置文件** | `src/modules/logger/module.yaml` |

---

## 2. 所有可用的日志配置文件

### 2.1 配置文件列表

| Bit 位 | 配置名称 | 枚举值 | 十进制值 | 用途 |
|--------|---------|--------|---------|------|
| **0** | `DEFAULT` | `1 << 0` | `1` | 默认日志集（常规分析） |
| **1** | `ESTIMATOR_REPLAY` | `1 << 1` | `2` | EKF2 回放数据（全速估计器主题） |
| **2** | `THERMAL_CALIBRATION` | `1 << 2` | `4` | 热校准（高频 IMU 和气压计数据） |
| **3** | `SYSTEM_IDENTIFICATION` | `1 << 3` | `8` | **系统辨识**（高频执行器和 IMU） |
| **4** | `HIGH_RATE` | `1 << 4` | `16` | 高速率（快速机动分析） |
| **5** | `DEBUG_TOPICS` | `1 << 5` | `32` | 调试主题（debug_* 消息） |
| **6** | `SENSOR_COMPARISON` | `1 << 6` | `64` | 传感器比较（低速率原始传感器） |
| **7** | `VISION_AND_AVOIDANCE` | `1 << 7` | `128` | 计算机视觉和避障 |
| **8** | `RAW_IMU_GYRO_FIFO` | `1 << 8` | `256` | 原始 FIFO 陀螺仪高速率 |
| **9** | `RAW_IMU_ACCEL_FIFO` | `1 << 9` | `512` | 原始 FIFO 加速度计高速率 |
| **10** | `MAVLINK_TUNNEL` | `1 << 10` | `1024` | Mavlink 隧道消息 |
| **11** | `HIGH_RATE_SENSORS` | `1 << 11` | `2048` | 高速率传感器 |

### 2.2 枚举定义

**文件**: `src/modules/logger/logged_topics.h:46-59`

```cpp
enum class SDLogProfileMask : int32_t {
	DEFAULT =               1 << 0,   // 1
	ESTIMATOR_REPLAY =      1 << 1,   // 2
	THERMAL_CALIBRATION =   1 << 2,   // 4
	SYSTEM_IDENTIFICATION = 1 << 3,   // 8
	HIGH_RATE =             1 << 4,   // 16
	DEBUG_TOPICS =          1 << 5,   // 32
	SENSOR_COMPARISON =     1 << 6,   // 64
	VISION_AND_AVOIDANCE =  1 << 7,   // 128
	RAW_IMU_GYRO_FIFO =     1 << 8,   // 256
	RAW_IMU_ACCEL_FIFO =    1 << 9,   // 512
	MAVLINK_TUNNEL =        1 << 10,  // 1024
	HIGH_RATE_SENSORS =     1 << 11   // 2048
};
```

---

## 3. 如何切换日志配置模式

### 3.1 方法一：通过 QGroundControl (QGC)

#### 步骤：

1. **连接飞控到 QGC**
2. **打开参数设置**：
   - 点击左侧菜单 → `Vehicle Setup` → `Parameters`
3. **搜索参数**：
   - 在搜索框输入 `SDLOG_PROFILE`
4. **设置参数值**：
   - 根据下方的计算器选择你需要的模式
5. **重启飞控**：
   - 点击 `Reboot Vehicle` 使参数生效

### 3.2 方法二：通过 MAVLink 控制台

```bash
# 查看当前配置
param show SDLOG_PROFILE

# 设置为系统辨识模式（单独）
param set SDLOG_PROFILE 8

# 保存参数
param save

# 重启飞控
reboot
```

### 3.3 方法三：通过 PX4 Shell (nsh)

```bash
# 连接到飞控串口或通过 MAVLink shell

# 查看当前值
param show SDLOG_PROFILE

# 设置新值
param set SDLOG_PROFILE 8

# 保存
param save

# 重启
reboot
```

### 3.4 方法四：编辑配置文件（SITL 仿真）

在 SITL 仿真中，可以编辑参数文件：

```bash
# 文件路径示例
build/px4_sitl_default/etc/init.d-posix/rcS

# 添加如下行
param set SDLOG_PROFILE 8
```

---

## 4. 常用配置场景及计算

### 4.1 单独使用某个模式

#### 示例 1：仅使用 DEFAULT 模式
```
SDLOG_PROFILE = 1
```

#### 示例 2：仅使用 SYSTEM_IDENTIFICATION 模式
```
SDLOG_PROFILE = 8
```
**记录内容**（参考 `logged_topics.cpp:359-367`）：
- `sensor_combined` - 全速率
- `vehicle_angular_velocity` - 全速率
- `vehicle_torque_setpoint` - 全速率
- `vehicle_acceleration` - 全速率
- `actuator_motors` - 全速率

#### 示例 3：仅使用 HIGH_RATE 模式
```
SDLOG_PROFILE = 16
```
**记录内容**（参考 `logged_topics.cpp:270-287`）：
- `manual_control_setpoint` - 全速率
- `rate_ctrl_status` - 20ms 间隔
- `sensor_combined` - 全速率
- `vehicle_angular_velocity` - 全速率
- `vehicle_attitude` - 全速率
- `vehicle_attitude_setpoint` - 全速率
- `vehicle_rates_setpoint` - 全速率
- `esc_status` - 5ms 间隔
- `actuator_motors` - 全速率
- `actuator_outputs_debug` - 全速率
- `actuator_servos` - 全速率
- `vehicle_thrust_setpoint` - 全速率
- `vehicle_torque_setpoint` - 全速率

### 4.2 组合多个模式

位掩码允许**同时启用多个配置文件**。需要将对应的十进制值相加。

#### 示例 1：DEFAULT + SYSTEM_IDENTIFICATION
```
计算: 1 (DEFAULT) + 8 (SYSTEM_IDENTIFICATION) = 9
SDLOG_PROFILE = 9
```

#### 示例 2：DEFAULT + HIGH_RATE + DEBUG_TOPICS
```
计算: 1 (DEFAULT) + 16 (HIGH_RATE) + 32 (DEBUG) = 49
SDLOG_PROFILE = 49
```

#### 示例 3：所有与 IMU 相关的高速模式
```
计算: 1 (DEFAULT) + 8 (SYSTEM_ID) + 16 (HIGH_RATE) + 256 (GYRO_FIFO) + 512 (ACCEL_FIFO)
    = 793
SDLOG_PROFILE = 793
```

#### 示例 4：EKF2 完整回放模式
```
计算: 1 (DEFAULT) + 2 (ESTIMATOR_REPLAY) = 3
SDLOG_PROFILE = 3
```

### 4.3 常用组合速查表

| 应用场景 | 组合 | 计算值 |
|---------|------|--------|
| **常规飞行日志** | DEFAULT | `1` |
| **系统辨识** | DEFAULT + SYSTEM_IDENTIFICATION | `9` |
| **PID 调参** | DEFAULT + HIGH_RATE | `17` |
| **IMU 分析** | DEFAULT + THERMAL_CALIBRATION | `5` |
| **EKF 调试** | DEFAULT + ESTIMATOR_REPLAY | `3` |
| **传感器故障分析** | DEFAULT + SENSOR_COMPARISON | `65` |
| **完整调试** | DEFAULT + HIGH_RATE + DEBUG | `49` |
| **最大 IMU 数据** | DEFAULT + GYRO_FIFO + ACCEL_FIFO | `769` |

---

## 5. 位掩码计算器

### 5.1 如何手动计算

**步骤**：
1. 从上表找到你需要的配置项
2. 将对应的十进制值相加
3. 设置 `SDLOG_PROFILE` 为计算结果

### 5.2 Python 计算脚本

```python
#!/usr/bin/env python3
"""
SDLOG_PROFILE 计算器
"""

profiles = {
    'DEFAULT':               1 << 0,   # 1
    'ESTIMATOR_REPLAY':      1 << 1,   # 2
    'THERMAL_CALIBRATION':   1 << 2,   # 4
    'SYSTEM_IDENTIFICATION': 1 << 3,   # 8
    'HIGH_RATE':             1 << 4,   # 16
    'DEBUG_TOPICS':          1 << 5,   # 32
    'SENSOR_COMPARISON':     1 << 6,   # 64
    'VISION_AND_AVOIDANCE':  1 << 7,   # 128
    'RAW_IMU_GYRO_FIFO':     1 << 8,   # 256
    'RAW_IMU_ACCEL_FIFO':    1 << 9,   # 512
    'MAVLINK_TUNNEL':        1 << 10,  # 1024
    'HIGH_RATE_SENSORS':     1 << 11   # 2048
}

# 示例：计算 DEFAULT + SYSTEM_IDENTIFICATION
selected = ['DEFAULT', 'SYSTEM_IDENTIFICATION']
value = sum(profiles[name] for name in selected)
print(f"SDLOG_PROFILE = {value}")

# 输出: SDLOG_PROFILE = 9
```

### 5.3 反向解析（已知值查询包含的模式）

```python
def parse_profile(value):
    """解析 SDLOG_PROFILE 值，返回启用的模式列表"""
    enabled = []
    for name, mask in profiles.items():
        if value & mask:
            enabled.append(name)
    return enabled

# 示例
value = 49
enabled = parse_profile(value)
print(f"SDLOG_PROFILE = {value} 包含: {enabled}")
# 输出: ['DEFAULT', 'HIGH_RATE', 'DEBUG_TOPICS']
```

---

## 6. 各模式的详细记录主题

### 6.1 DEFAULT 模式

**函数**: `add_default_topics()` (`logged_topics.cpp:46-268`)

**主要记录的主题**（部分）：

| 主题名称 | 记录间隔 (ms) | 实例数 |
|---------|--------------|--------|
| `sensor_gyro` | 1000 | 4 |
| `sensor_accel` | 1000 | 4 |
| `sensor_combined` | 全速率 | 1 |
| `vehicle_imu` | 500 | 4 |
| `vehicle_angular_velocity` | 20 | 1 |
| `vehicle_attitude` | 50 | 1 |
| `vehicle_local_position` | 100 | 1 |
| `battery_status` | 200 | 3 |
| `actuator_motors` | 100 | 1 |

**用途**：
- 常规飞行日志分析
- 基本故障诊断
- 日常飞行记录

### 6.2 SYSTEM_IDENTIFICATION 模式

**函数**: `add_system_identification_topics()` (`logged_topics.cpp:359-367`)

**记录的主题**：

```cpp
void LoggedTopics::add_system_identification_topics()
{
	// for system id need to log imu and controls at full rate
	add_topic("sensor_combined");              // 全速率
	add_topic("vehicle_angular_velocity");     // 全速率
	add_topic("vehicle_torque_setpoint");      // 全速率
	add_topic("vehicle_acceleration");         // 全速率
	add_topic("actuator_motors");              // 全速率
}
```

| 主题名称 | 记录间隔 | 典型频率 |
|---------|---------|---------|
| `sensor_combined` | 0 (全速) | ~1000 Hz |
| `vehicle_angular_velocity` | 0 (全速) | ~667 Hz |
| `vehicle_torque_setpoint` | 0 (全速) | ~250 Hz |
| `vehicle_acceleration` | 0 (全速) | ~250 Hz |
| `actuator_motors` | 0 (全速) | ~250 Hz |

**用途**：
- **系统辨识**（System Identification）
- 动态模型参数估计
- 控制系统建模
- 机体惯量、阻尼系数识别

**警告**：
- ⚠️ **数据量大**：全速率记录会产生非常大的日志文件
- ⚠️ **带宽需求高**：SD 卡写入速度需要足够快
- ⚠️ **建议飞行时间**：限制在 1-2 分钟内

### 6.3 HIGH_RATE 模式

**函数**: `add_high_rate_topics()` (`logged_topics.cpp:270-287`)

**记录的主题**：

| 主题名称 | 记录间隔 (ms) | 典型频率 |
|---------|--------------|---------|
| `manual_control_setpoint` | 0 (全速) | ~50 Hz |
| `rate_ctrl_status` | 20 | 50 Hz |
| `sensor_combined` | 0 (全速) | ~1000 Hz |
| `vehicle_angular_velocity` | 0 (全速) | ~667 Hz |
| `vehicle_attitude` | 0 (全速) | ~250 Hz |
| `vehicle_attitude_setpoint` | 0 (全速) | ~250 Hz |
| `vehicle_rates_setpoint` | 0 (全速) | ~500 Hz |
| `esc_status` | 5 | 200 Hz |
| `actuator_motors` | 0 (全速) | ~250 Hz |
| `actuator_servos` | 0 (全速) | ~250 Hz |
| `vehicle_thrust_setpoint` | 0 (全速) | ~250 Hz |
| `vehicle_torque_setpoint` | 0 (全速) | ~250 Hz |

**用途**：
- 快速机动分析（如竞速、特技飞行）
- PID 参数调优
- 控制环路性能分析
- 电机响应分析

### 6.4 ESTIMATOR_REPLAY 模式

**函数**: `add_estimator_replay_topics()` (`logged_topics.cpp:302-321`)

**记录的主题**（部分）：

| 主题名称 | 记录间隔 | 说明 |
|---------|---------|------|
| `ekf2_timestamps` | 0 (全速) | EKF2 时间戳 |
| `airspeed` | 0 (全速) | 空速数据 |
| `sensor_combined` | 0 (全速) | 组合传感器 |
| `vehicle_gps_position` | 0 (全速) | GPS 位置 |
| `vehicle_magnetometer` | 0 (全速) | 磁力计 |
| `vehicle_status` | 0 (全速) | 飞机状态 |
| `distance_sensor` | 0 (全速) | 距离传感器 |

**用途**：
- EKF2 离线回放和调试
- 状态估计器性能分析
- 传感器融合算法验证

### 6.5 DEBUG_TOPICS 模式

**函数**: `add_debug_topics()` (`logged_topics.cpp:289-300`)

**记录的主题**：

```cpp
void LoggedTopics::add_debug_topics()
{
	add_topic("debug_array");
	add_topic("debug_key_value");
	add_topic("debug_value");
	add_topic("debug_vect");
	add_topic_multi("satellite_info", 1000, 2);
	add_topic("mag_worker_data");
	add_topic("sensor_preflight_mag", 500);
	add_topic("actuator_test", 500);
	add_topic("neural_control", 50);
}
```

**用途**：
- 自定义代码调试
- 使用 `debug_*` 消息输出中间变量
- 算法开发和验证

---

## 7. 实际切换示例

### 7.1 场景 1：从默认模式切换到系统辨识模式

**目标**：进行系统辨识实验

**步骤**：

```bash
# 1. 连接飞控
# 2. 打开 MAVLink 控制台

# 3. 查看当前配置
param show SDLOG_PROFILE
# 输出: SDLOG_PROFILE = 1 (DEFAULT)

# 4. 切换到系统辨识模式（保留 DEFAULT）
param set SDLOG_PROFILE 9
# 计算: 1 (DEFAULT) + 8 (SYSTEM_IDENTIFICATION) = 9

# 5. 保存并重启
param save
reboot
```

**飞行后验证**：
```bash
# 下载日志后，检查是否包含高速数据
# 使用 pyulog 工具
ulog_info log_file.ulg | grep -E "(sensor_combined|vehicle_angular_velocity|actuator_motors)"
```

### 7.2 场景 2：调试 PID 参数

**目标**：调整姿态控制器 PID

**步骤**：

```bash
# 切换到 DEFAULT + HIGH_RATE
param set SDLOG_PROFILE 17
# 计算: 1 + 16 = 17

param save
reboot
```

**分析日志**：
- 查看 `vehicle_attitude_setpoint` vs `vehicle_attitude` 跟踪性能
- 分析 `vehicle_rates_setpoint` vs `vehicle_angular_velocity` 响应
- 检查 `actuator_motors` 输出饱和情况

### 7.3 场景 3：IMU 传感器标定

**目标**：进行 IMU 热校准

**步骤**：

```bash
# 切换到 THERMAL_CALIBRATION 模式
param set SDLOG_PROFILE 4
# 或组合 DEFAULT: 1 + 4 = 5

param save
reboot
```

**记录数据**：
- `sensor_accel` - 100ms 间隔，4 个实例
- `sensor_gyro` - 100ms 间隔，4 个实例
- `sensor_baro` - 100ms 间隔，4 个实例
- `sensor_mag` - 100ms 间隔，4 个实例

### 7.4 场景 4：恢复到默认模式

```bash
# 恢复默认配置
param set SDLOG_PROFILE 1

param save
reboot
```

---

## 8. 注意事项和最佳实践

### 8.1 ⚠️ 警告

1. **存储空间**：
   - 高速率模式会快速填满 SD 卡
   - 建议使用高速 SD 卡（Class 10 或 UHS-I）
   - 监控可用空间

2. **带宽限制**：
   - 同时启用多个高速模式可能导致数据丢失
   - 建议分开测试

3. **重启要求**：
   - **必须重启飞控**才能使新配置生效
   - 修改后立即重启

4. **FIFO 模式性能影响**：
   - `RAW_IMU_GYRO_FIFO` 和 `RAW_IMU_ACCEL_FIFO` 会显著增加 CPU 负载
   - 自动提高 logger 任务优先级（见 `logger.cpp:533-536`）

### 8.2 ✅ 最佳实践

1. **分阶段测试**：
   - 先用 DEFAULT 模式验证基本功能
   - 再启用专用模式进行特定测试

2. **日志文件管理**：
   - 定期清理旧日志
   - 使用有意义的飞行记录

3. **参数备份**：
   - 修改前备份当前参数：`param export`
   - 恢复：`param import`

4. **验证配置**：
   ```bash
   # 重启后验证
   param show SDLOG_PROFILE

   # 查看 logger 状态
   logger status
   ```

5. **监控性能**：
   ```bash
   # 检查 logger 缓冲区使用情况
   logger status

   # 输出示例
   # log buffer: 16/512 KB
   # rate: 5.2 KB/s
   # message gaps: 0
   ```

---

## 9. 代码实现原理

### 9.1 参数读取

**文件**: `src/modules/logger/logger.cpp:494-502`

```cpp
bool Logger::initialize_topics()
{
	// get the logging profile
	SDLogProfileMask sdlog_profile = (SDLogProfileMask)_param_sdlog_profile.get();

	if ((int32_t)sdlog_profile == 0) {
		PX4_WARN("No logging profile selected. Using default set");
		sdlog_profile = SDLogProfileMask::DEFAULT;
	}

	LoggedTopics logged_topics;
	// ...
}
```

### 9.2 配置文件应用

**文件**: `src/modules/logger/logged_topics.cpp:545-596`

```cpp
void LoggedTopics::initialize_configured_topics(SDLogProfileMask profile)
{
	// load appropriate topics for profile
	// the order matters: if several profiles add the same topic,
	// the logging rate of the last one will be used

	if (profile & SDLogProfileMask::DEFAULT) {
		add_default_topics();
	}

	if (profile & SDLogProfileMask::ESTIMATOR_REPLAY) {
		add_estimator_replay_topics();
	}

	if (profile & SDLogProfileMask::THERMAL_CALIBRATION) {
		add_thermal_calibration_topics();
	}

	if (profile & SDLogProfileMask::SYSTEM_IDENTIFICATION) {
		add_system_identification_topics();
	}

	if (profile & SDLogProfileMask::HIGH_RATE) {
		add_high_rate_topics();
	}

	// ... 其他模式
}
```

**说明**：
- 使用位运算 `&` 检查每个位
- 按顺序添加主题
- **后添加的配置会覆盖之前的记录频率**

### 9.3 主题覆盖机制

如果多个配置文件包含相同主题但频率不同，**最后添加的频率会生效**。

**示例**：
```cpp
// DEFAULT 模式
add_topic("sensor_combined");  // 全速率

// SYSTEM_IDENTIFICATION 模式（后执行）
add_topic("sensor_combined");  // 全速率（覆盖）
```

---

## 10. 故障排查

### 10.1 参数设置不生效

**问题**：修改 `SDLOG_PROFILE` 后日志内容没变化

**解决方案**：
```bash
# 1. 确认参数已保存
param show SDLOG_PROFILE

# 2. 必须重启
reboot

# 3. 重启后再次确认
param show SDLOG_PROFILE

# 4. 查看 logger 启动信息
dmesg | grep logger
```

### 10.2 日志数据丢失

**问题**：高速率模式下出现数据间隙（message gaps）

**诊断**：
```bash
logger status

# 检查输出
# message gaps: 150  ← 如果大于 0 表示有丢失
```

**解决方案**：
1. 使用更快的 SD 卡
2. 减少启用的配置文件数量
3. 检查 CPU 负载：`top`

### 10.3 SD 卡空间不足

**问题**：高速率模式快速填满存储

**解决方案**：
```bash
# 1. 清理旧日志
cd /fs/microsd/log
rm -rf 2024-*

# 2. 设置自动清理参数
param set SDLOG_DIRS_MAX 10

# 3. 缩短测试时间
# 系统辨识通常 1-2 分钟足够
```

---

## 11. 相关参考

### 11.1 源代码文件

| 文件 | 说明 |
|------|------|
| `src/modules/logger/logged_topics.h` | 配置文件枚举定义 |
| `src/modules/logger/logged_topics.cpp` | 各模式主题列表 |
| `src/modules/logger/logger.cpp` | Logger 主逻辑 |
| `src/modules/logger/module.yaml` | 参数定义 |

### 11.2 相关参数

| 参数 | 说明 |
|------|------|
| `SDLOG_PROFILE` | 日志配置文件（位掩码） |
| `SDLOG_MODE` | 日志触发模式（何时开始/停止） |
| `SDLOG_DIRS_MAX` | 最多保留的日志目录数 |
| `SDLOG_MISSION` | 任务日志模式 |

### 11.3 相关命令

```bash
# Logger 模块命令
logger start              # 启动 logger
logger stop               # 停止 logger
logger status             # 查看状态
logger on                 # 开始记录
logger off                # 停止记录

# 参数命令
param show SDLOG_PROFILE  # 查看当前值
param set SDLOG_PROFILE 9 # 设置新值
param save                # 保存参数
param reset SDLOG_PROFILE # 恢复默认值
```

---

## 12. 总结

### 12.1 快速参考

**从 DEFAULT 切换到 SYSTEM_IDENTIFICATION**：

```bash
param set SDLOG_PROFILE 9
param save
reboot
```

**恢复默认**：
```bash
param set SDLOG_PROFILE 1
param save
reboot
```

### 12.2 关键要点

1. ✅ `SDLOG_PROFILE` 是**位掩码**，可以组合多个模式
2. ✅ 修改后**必须重启**才能生效
3. ✅ 高速率模式会产生**大量数据**，注意存储空间
4. ✅ 使用**高速 SD 卡**避免数据丢失
5. ✅ 通过 `logger status` 监控性能

---

**文档版本**: v1.0
**最后更新**: 2025-11-01
**适用 PX4 版本**: v1.14+
**作者**: PX4 开发文档

