# ERmao 日志模式使用说明
## 完整 IMU 信号链全速率记录模式

---

## 1. 概述

**ERmao 模式**是专门为 IMU 信号链完整分析而设计的日志记录配置文件。该模式以**全速率**记录从原始传感器到电机输出的完整控制链路数据。

### 1.1 核心特性

| 特性 | 说明 |
|------|------|
| **Bit 位** | 12 |
| **十进制值** | 4096 |
| **记录频率** | 全速率（无降采样） |
| **数据量** | 非常大（~100-200 MB/分钟） |
| **用途** | IMU 信号链分析、滤波器调试、控制性能分析 |

---

## 2. 记录的主题列表

### 2.1 完整主题清单

| 序号 | 主题名称 | 典型发布频率 | 说明 |
|------|---------|-------------|------|
| 1 | `sensor_gyro_fifo` | 1000-8000 Hz | 原始陀螺仪 FIFO 数据（可选） |
| 2 | `sensor_gyro_fft` | 10 Hz | 陀螺仪 FFT 频谱数据（可选） |
| 3 | `vehicle_angular_velocity` | ~667 Hz | 滤波后的角速度 |
| 4 | `vehicle_imu` | ~250 Hz | 积分后的 IMU 数据 |
| 5 | `sensor_combined` | ~1000 Hz | 融合传感器数据 |
| 6 | `vehicle_attitude` | ~250 Hz | 姿态估计（四元数） |
| 7 | `vehicle_attitude_setpoint` | ~250 Hz | 姿态设定值 |
| 8 | `vehicle_rates_setpoint` | ~250 Hz | 角速率设定值 |
| 9 | `actuator_motors` | ~250 Hz | 电机输出 |

### 2.2 信号链覆盖范围

```
[传感器硬件] → sensor_gyro_fifo (FIFO 原始数据)
    ↓
[驱动层] → (sensor_gyro 未记录，频率过高)
    ↓
[VehicleIMU] → vehicle_imu (积分数据)
    ↓
[VehicleAngularVelocity] → vehicle_angular_velocity (滤波数据)
    ↓
[Sensors] → sensor_combined (融合数据)
    ↓
[EKF2] → vehicle_attitude (姿态估计)
    ↓
[姿态控制器] → vehicle_attitude_setpoint (姿态指令)
    ↓
[角速率控制器] → vehicle_rates_setpoint (角速率指令)
    ↓
[控制分配器] → actuator_motors (电机输出)
    ↓
[电机/ESC]
```

**覆盖范围**：从原始 FIFO 到电机输出的**完整信号链**

---

## 3. 如何启用 ERmao 模式

### 3.1 方法 1：仅使用 ERmao 模式

```bash
# 连接飞控后
param set SDLOG_PROFILE 4096
param save
reboot
```

**计算**：
```
ERmao = 1 << 12 = 4096
```

### 3.2 方法 2：ERmao + DEFAULT（推荐）

```bash
param set SDLOG_PROFILE 4097
param save
reboot
```

**计算**：
```
DEFAULT (1) + ERmao (4096) = 4097
```

**说明**：组合 DEFAULT 模式可以确保记录其他必要的系统信息（如 GPS、电池等）。

### 3.3 方法 3：通过 QGroundControl

1. 打开 QGC → `Vehicle Setup` → `Parameters`
2. 搜索 `SDLOG_PROFILE`
3. 勾选 **Bit 12: ERmao mode**
4. （可选）同时勾选 **Bit 0: Default set**
5. 点击保存并重启飞控

### 3.4 验证模式已启用

```bash
# 重启后检查
param show SDLOG_PROFILE

# 应该显示
SDLOG_PROFILE = 4096  # 或 4097（如果包含 DEFAULT）

# 查看 logger 状态
logger status
```

---

## 4. 数据分析应用场景

### 4.1 场景 1：陀螺仪滤波器分析

**目标**：分析 VehicleAngularVelocity 模块的滤波效果

**数据对比**：
```python
import numpy as np
from pyulog import ULog

ulog = ULog('log.ulg')

# 原始 FIFO 数据
gyro_fifo = ulog.get_dataset('sensor_gyro_fifo')
raw_data = gyro_fifo.data['x']  # X 轴原始数据

# 滤波后数据
angular_vel = ulog.get_dataset('vehicle_angular_velocity')
filtered_data = angular_vel.data['xyz'][:, 0]  # X 轴滤波数据

# FFT 分析
from scipy import signal
f_raw, Pxx_raw = signal.welch(raw_data, fs=8000)
f_filt, Pxx_filt = signal.welch(filtered_data, fs=667)

# 绘制频谱对比
import matplotlib.pyplot as plt
plt.semilogy(f_raw, Pxx_raw, label='Raw')
plt.semilogy(f_filt, Pxx_filt, label='Filtered')
plt.xlabel('Frequency (Hz)')
plt.ylabel('PSD')
plt.legend()
plt.title('Gyro Filter Effectiveness')
plt.show()
```

**分析内容**：
- 动态陷波滤波器是否有效抑制电机振动
- 低通滤波器截止频率是否合适
- 是否引入过多相位延迟

### 4.2 场景 2：姿态控制器性能分析

**目标**：评估姿态控制器跟踪性能

**数据对比**：
```python
# 姿态设定值 vs 实际姿态
attitude_sp = ulog.get_dataset('vehicle_attitude_setpoint')
attitude = ulog.get_dataset('vehicle_attitude')

# 转换四元数到欧拉角
from pyulog.px4 import PX4ULog
euler_sp = px4ulog.quaternion_to_euler(attitude_sp.data['q_d'])
euler_act = px4ulog.quaternion_to_euler(attitude.data['q'])

# 计算跟踪误差
roll_error = euler_sp[:, 0] - euler_act[:, 0]
pitch_error = euler_sp[:, 1] - euler_act[:, 1]

# 绘制
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(time, np.degrees(euler_sp[:, 0]), label='Setpoint')
plt.plot(time, np.degrees(euler_act[:, 0]), label='Actual')
plt.ylabel('Roll (deg)')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time, np.degrees(roll_error))
plt.ylabel('Roll Error (deg)')
plt.xlabel('Time (s)')
plt.show()
```

**分析指标**：
- **跟踪延迟**：设定值到实际响应的时间延迟
- **稳态误差**：长时间的跟踪偏差
- **超调量**：阶跃响应的过冲

### 4.3 场景 3：角速率控制器调参

**目标**：优化 PID 参数

**数据对比**：
```python
# 角速率设定值 vs 实际角速率
rates_sp = ulog.get_dataset('vehicle_rates_setpoint')
angular_vel = ulog.get_dataset('vehicle_angular_velocity')

roll_rate_sp = rates_sp.data['roll']
roll_rate_act = angular_vel.data['xyz'][:, 0]

# 计算性能指标
rate_error = roll_rate_sp - roll_rate_act
rmse = np.sqrt(np.mean(rate_error**2))
max_error = np.max(np.abs(rate_error))

print(f"RMSE: {rmse:.3f} rad/s")
print(f"Max error: {max_error:.3f} rad/s")

# 阶跃响应分析（找到阶跃时刻）
step_idx = np.where(np.abs(np.diff(roll_rate_sp)) > 0.5)[0]
if len(step_idx) > 0:
    idx = step_idx[0]
    response = roll_rate_act[idx:idx+200]  # 200 个采样点

    # 计算上升时间、超调量
    rise_time = calculate_rise_time(response)
    overshoot = calculate_overshoot(response)

    print(f"Rise time: {rise_time:.3f} s")
    print(f"Overshoot: {overshoot:.1f}%")
```

**PID 调整建议**：
- `RMSE` 大 → 增大 P 增益（`MC_ROLLRATE_P`）
- 有稳态误差 → 增大 I 增益（`MC_ROLLRATE_I`）
- 振荡/超调大 → 减小 P/I，增加 D（`MC_ROLLRATE_D`）

### 4.4 场景 4：电机响应延迟分析

**目标**：测量从控制指令到实际输出的延迟

```python
rates_sp = ulog.get_dataset('vehicle_rates_setpoint')
motors = ulog.get_dataset('actuator_motors')
angular_vel = ulog.get_dataset('vehicle_angular_velocity')

# 计算互相关，找延迟
from scipy.signal import correlate

correlation = correlate(motors.data['control'][0],
                        angular_vel.data['xyz'][:, 0],
                        mode='full')
delay_samples = np.argmax(correlation) - len(angular_vel.data['xyz'][:, 0])
delay_ms = delay_samples / 667 * 1000  # 假设 667 Hz

print(f"电机响应延迟: {delay_ms:.1f} ms")
```

### 4.5 场景 5：sensor_gyro_fft 振动频谱分析

**目标**：识别振动源

```python
gyro_fft = ulog.get_dataset('sensor_gyro_fft')

# 提取 FFT 数据
freq = gyro_fft.data['freq']  # 频率轴
peak_freq_x = gyro_fft.data['peak_frequencies_x']
peak_freq_y = gyro_fft.data['peak_frequencies_y']
peak_freq_z = gyro_fft.data['peak_frequencies_z']

# 绘制时频图
plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(time, peak_freq_x)
plt.ylabel('Peak Freq X (Hz)')
plt.title('Vibration Frequency Tracking')

plt.subplot(3, 1, 2)
plt.plot(time, peak_freq_y)
plt.ylabel('Peak Freq Y (Hz)')

plt.subplot(3, 1, 3)
plt.plot(time, peak_freq_z)
plt.ylabel('Peak Freq Z (Hz)')
plt.xlabel('Time (s)')
plt.show()
```

**振动源识别**：
- **~100-200 Hz**：电机基频（取决于 KV 值和转速）
- **~200-400 Hz**：电机 2 倍频
- **~50-80 Hz**：机架共振
- **不规则宽频**：螺旋桨不平衡

---

## 5. 注意事项

### 5.1 ⚠️ 数据量警告

**预估数据量**（1 分钟飞行）：

| 主题 | 频率 | 数据大小/分钟 |
|------|------|--------------|
| `sensor_gyro_fifo` | 8000 Hz | ~60 MB |
| `vehicle_angular_velocity` | 667 Hz | ~8 MB |
| `sensor_combined` | 1000 Hz | ~12 MB |
| `vehicle_imu` | 250 Hz | ~3 MB |
| `vehicle_attitude` | 250 Hz | ~3 MB |
| 其他主题 | - | ~10 MB |
| **总计** | - | **~100 MB/分钟** |

**建议**：
- ✅ 使用高速 SD 卡（UHS-I 或更快）
- ✅ 限制飞行时间（1-3 分钟）
- ✅ 定期清理日志文件
- ✅ 监控 SD 卡可用空间

### 5.2 ⚠️ CPU 负载

启用 ERmao 模式会增加 logger 任务的 CPU 使用率。

**监控方法**：
```bash
# 检查 logger 性能
logger status

# 输出示例
# log buffer: 48/512 KB
# rate: 120.5 KB/s
# message gaps: 0  ← 应该为 0

# 检查系统负载
top
```

**如果出现 `message gaps > 0`**：
- 减少启用的配置文件数量
- 使用更快的 SD 卡
- 缩短飞行时间

### 5.3 ⚠️ 与其他模式组合

**推荐组合**：
```bash
# ERmao + DEFAULT（推荐）
param set SDLOG_PROFILE 4097  # 4096 + 1

# 不推荐同时启用
# ERmao + RAW_IMU_GYRO_FIFO + RAW_IMU_ACCEL_FIFO
# 数据量太大，可能导致丢失
```

---

## 6. 快速参考

### 6.1 启用/禁用命令

```bash
# 启用 ERmao 模式
param set SDLOG_PROFILE 4097
param save
reboot

# 恢复默认
param set SDLOG_PROFILE 1
param save
reboot

# 查看当前设置
param show SDLOG_PROFILE
```

### 6.2 日志下载与分析

```bash
# 下载日志（通过 MAVLink）
# 使用 QGC 或 mavlink_logdownload.py

# 转换为 CSV（使用 pyulog）
ulog2csv log_file.ulg

# Python 分析
from pyulog import ULog
ulog = ULog('log_file.ulg')

# 列出所有主题
print(ulog.data_list)

# 提取数据
angular_vel = ulog.get_dataset('vehicle_angular_velocity')
print(angular_vel.data.keys())
```

### 6.3 参数对照表

| 十进制值 | 二进制表示 | 包含的模式 |
|---------|-----------|-----------|
| `4096` | `1000000000000` | 仅 ERmao |
| `4097` | `1000000000001` | ERmao + DEFAULT |
| `4104` | `1000000001000` | ERmao + SYSTEM_ID |
| `4112` | `1000000010000` | ERmao + HIGH_RATE |

---

## 7. 故障排查

### 7.1 模式未生效

**症状**：修改参数后，日志中没有记录对应主题

**解决方案**：
```bash
# 1. 确认参数已保存
param show SDLOG_PROFILE

# 2. 必须重启
reboot

# 3. 查看 logger 启动信息
dmesg | grep logger

# 4. 检查主题是否存在
listener vehicle_angular_velocity
```

### 7.2 数据丢失（message gaps）

**症状**：`logger status` 显示 `message gaps > 0`

**解决方案**：
1. 检查 SD 卡速度：使用 Class 10 或 UHS-I
2. 减少记录时间
3. 单独使用 ERmao，不组合其他高速模式
4. 检查 CPU 负载：`top`

### 7.3 某些主题未记录

**症状**：日志中缺少某些主题

**可能原因**：
- `sensor_gyro_fifo`、`sensor_gyro_fft` 是**可选主题**（optional）
- 如果传感器不支持或模块未启动，不会记录

**检查方法**：
```bash
# 检查主题是否发布
uorb top

# 手动监听
listener sensor_gyro_fifo
listener sensor_gyro_fft

# 如果提示 "not found"，说明该主题未发布
```

---

## 8. 源码参考

### 8.1 修改的文件

| 文件 | 修改内容 |
|------|---------|
| `src/modules/logger/logged_topics.h` | 添加 `ERMAO` 枚举和函数声明 |
| `src/modules/logger/logged_topics.cpp` | 添加 `add_ermao_topics()` 函数实现 |
| `src/modules/logger/module.yaml` | 添加 Bit 12 参数描述 |

### 8.2 关键代码

```cpp
// logged_topics.cpp:602-630
void LoggedTopics::add_ermao_topics()
{
	// ERmao 模式：记录完整的 IMU 和控制信号链数据（全速率）

	add_optional_topic("sensor_gyro_fifo");      // 原始 FIFO
	add_optional_topic("sensor_gyro_fft");       // FFT 频谱
	add_topic("vehicle_angular_velocity");       // 滤波角速度
	add_topic("vehicle_imu");                    // 积分 IMU
	add_topic("sensor_combined");                // 融合数据
	add_topic("vehicle_attitude");               // 姿态估计
	add_topic("vehicle_attitude_setpoint");      // 姿态设定值
	add_topic("vehicle_rates_setpoint");         // 角速率设定值
	add_topic("actuator_motors");                // 电机输出
}
```

---

## 9. 总结

### 9.1 ERmao 模式的优势

✅ **完整性**：覆盖从传感器到执行器的完整信号链
✅ **高保真**：全速率记录，无降采样失真
✅ **灵活性**：可与其他模式组合使用
✅ **针对性**：专为 IMU 信号链分析设计

### 9.2 适用场景

- 🔧 滤波器参数调优
- 🔧 PID 参数调试
- 🔧 振动分析和抑制
- 🔧 控制延迟测量
- 🔧 信号链故障排查
- 🔧 算法验证和性能测试

### 9.3 最佳实践

1. **短时间测试**：1-3 分钟足够，避免过大日志
2. **组合 DEFAULT**：确保记录系统状态信息
3. **使用快速 SD 卡**：UHS-I 或更快
4. **飞行前验证**：检查 `logger status`，确保无 gaps
5. **及时分析**：下载后立即分析，避免混淆

---

**文档版本**: v1.0
**创建日期**: 2025-11-01
**适用 PX4 版本**: v1.14+
**作者**: ERmao 模式开发团队

