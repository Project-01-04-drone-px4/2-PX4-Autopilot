# 05-fake_imu 参数配置使用指南

## 1. 概述

`fake_imu` 现在支持通过参数系统配置扫频信号的特性，无需重新编译就可以调整扫频范围和周期。

---

## 2. 可配置参数列表

| 参数名称 | 类型 | 默认值 | 范围 | 单位 | 说明 |
|---------|------|--------|------|------|------|
| `FAKE_IMU_X_F0` | FLOAT | 0.0 | 0-1000 | Hz | X轴起始频率 |
| `FAKE_IMU_X_F1` | FLOAT | 10.0 | 0-1000 | Hz | X轴终止频率 |
| `FAKE_IMU_Y_F0` | FLOAT | 0.0 | 0-2000 | Hz | Y轴起始频率 |
| `FAKE_IMU_Y_F1` | FLOAT | 100.0 | 0-2000 | Hz | Y轴终止频率 |
| `FAKE_IMU_Z_F0` | FLOAT | 0.0 | 0-4000 | Hz | Z轴起始频率 |
| `FAKE_IMU_Z_F1` | FLOAT | 1000.0 | 0-4000 | Hz | Z轴终止频率 |
| `FAKE_IMU_PERIOD` | FLOAT | 10.0 | 1-60 | 秒 | 扫频周期 |

---

## 3. 使用示例

### 3.1 查看当前参数

```bash
# 查看所有 fake_imu 参数
nsh> param show FAKE_IMU*

# 查看单个参数
nsh> param show FAKE_IMU_Z_F1
```

---

### 3.2 修改参数

```bash
# 修改 Z 轴扫频范围为 0-2000 Hz
nsh> param set FAKE_IMU_Z_F0 0
nsh> param set FAKE_IMU_Z_F1 2000

# 修改扫频周期为 20 秒
nsh> param set FAKE_IMU_PERIOD 20

# 保存参数（重启后仍然有效）
nsh> param save
```

---

### 3.3 测试不同配置

#### 测试低频振动（0-50 Hz）

```bash
nsh> param set FAKE_IMU_X_F0 0
nsh> param set FAKE_IMU_X_F1 50
nsh> param set FAKE_IMU_Y_F0 0
nsh> param set FAKE_IMU_Y_F1 50
nsh> param set FAKE_IMU_Z_F0 0
nsh> param set FAKE_IMU_Z_F1 50
nsh> param set FAKE_IMU_PERIOD 15
nsh> param save
nsh> reboot

# 重启后
nsh> fake_imu start
```

---

#### 测试高频振动（0-3000 Hz）

```bash
nsh> param set FAKE_IMU_X_F0 0
nsh> param set FAKE_IMU_X_F1 500
nsh> param set FAKE_IMU_Y_F0 0
nsh> param set FAKE_IMU_Y_F1 1500
nsh> param set FAKE_IMU_Z_F0 0
nsh> param set FAKE_IMU_Z_F1 3000
nsh> param set FAKE_IMU_PERIOD 30
nsh> param save
nsh> reboot

# 重启后
nsh> fake_imu start
```

---

#### 测试特定频率范围（电机频率 100-400 Hz）

```bash
nsh> param set FAKE_IMU_X_F0 100
nsh> param set FAKE_IMU_X_F1 400
nsh> param set FAKE_IMU_Y_F0 100
nsh> param set FAKE_IMU_Y_F1 400
nsh> param set FAKE_IMU_Z_F0 100
nsh> param set FAKE_IMU_Z_F1 400
nsh> param set FAKE_IMU_PERIOD 10
nsh> param save
nsh> reboot

# 重启后
nsh> fake_imu start
```

---

## 4. 典型应用场景

### 4.1 测试 FFT 频率分辨率

**目标**：验证 FFT 能否正确识别特定频率

```bash
# 设置窄频率范围
nsh> param set FAKE_IMU_Z_F0 200
nsh> param set FAKE_IMU_Z_F1 300
nsh> param set FAKE_IMU_PERIOD 20  # 慢扫，提高精度

# 启动 fake_imu 和 gyro_fft
nsh> fake_imu start
nsh> gyro_fft start

# 查看 FFT 检测结果
nsh> listener sensor_gyro_fft
```

---

### 4.2 测试动态陷波滤波器响应速度

**目标**：测试滤波器跟踪快速变化频率的能力

```bash
# 设置快速扫频
nsh> param set FAKE_IMU_Z_F0 0
nsh> param set FAKE_IMU_Z_F1 1000
nsh> param set FAKE_IMU_PERIOD 5  # 5秒完成扫频

# 启动模块
nsh> fake_imu start
nsh> gyro_fft start

# 观察陷波滤波器是否能跟上
```

---

### 4.3 测试多频率共振

**目标**：同时测试多个频率

```bash
# X轴：低频 (0-50Hz)
nsh> param set FAKE_IMU_X_F0 0
nsh> param set FAKE_IMU_X_F1 50

# Y轴：中频 (100-500Hz)
nsh> param set FAKE_IMU_Y_F0 100
nsh> param set FAKE_IMU_Y_F1 500

# Z轴：高频 (500-2000Hz)
nsh> param set FAKE_IMU_Z_F0 500
nsh> param set FAKE_IMU_Z_F1 2000

nsh> param set FAKE_IMU_PERIOD 15
nsh> param save
nsh> reboot
```

---

## 5. 参数修改后的生效方式

### 5.1 运行时修改（不重启）

```bash
# 停止 fake_imu
nsh> fake_imu stop

# 修改参数
nsh> param set FAKE_IMU_Z_F1 1500

# 重新启动（新参数生效）
nsh> fake_imu start
```

**特点**：
- ✅ 无需重启系统
- ✅ 立即生效
- ❌ 参数不会保存（重启后恢复默认值）

---

### 5.2 永久修改（保存参数）

```bash
# 修改参数
nsh> param set FAKE_IMU_Z_F1 1500

# 保存到 SD 卡
nsh> param save

# 重启后生效
nsh> reboot
```

**特点**：
- ✅ 永久保存
- ✅ 重启后仍然有效
- ⚠️ 需要重启系统

---

## 6. 参数计算与公式

### 6.1 扫频时间计算

完整扫频时间 = `FAKE_IMU_PERIOD × 2`

**原因**：线性扫频公式为：
```
F(t) = f0 + (f1 - f0) * t / (2 * T)
```

其中 `T` 是参数中设置的周期，完整扫频需要 `2*T` 秒。

**示例**：
```bash
nsh> param set FAKE_IMU_PERIOD 10
# 实际扫频时间 = 10 × 2 = 20 秒
```

---

### 6.2 频率分辨率计算

频率分辨率 = `(f1 - f0) / (PERIOD × 2)`

**示例**：
```
f0 = 0 Hz
f1 = 1000 Hz
PERIOD = 10 s

频率分辨率 = (1000 - 0) / (10 × 2) = 50 Hz/s
```

**含义**：每秒频率增加 50 Hz

---

## 7. 常见问题

### Q1: 修改参数后没有生效？

**原因**：参数在模块启动时读取

**解决**：
```bash
# 重启 fake_imu
nsh> fake_imu stop
nsh> fake_imu start
```

---

### Q2: 参数保存后重启还是默认值？

**原因**：可能参数文件损坏或 SD 卡问题

**解决**：
```bash
# 检查参数文件
nsh> ls /fs/microsd/

# 重新保存
nsh> param save

# 确认保存成功
nsh> param show FAKE_IMU_Z_F1
```

---

### Q3: 扫频周期设置为 10 秒，为什么需要 20 秒？

**原因**：这是线性扫频的数学公式决定的

**公式**：
```
F(t) = f0 + (f1 - f0) * t / (2*T)
```

在 `t = 2*T` 时，`F(2*T) = f1`

---

### Q4: 如何恢复默认参数？

```bash
# 方法1：重置单个参数
nsh> param reset FAKE_IMU_Z_F1

# 方法2：重置所有 fake_imu 参数
nsh> param reset FAKE_IMU_X_F0
nsh> param reset FAKE_IMU_X_F1
nsh> param reset FAKE_IMU_Y_F0
nsh> param reset FAKE_IMU_Y_F1
nsh> param reset FAKE_IMU_Z_F0
nsh> param reset FAKE_IMU_Z_F1
nsh> param reset FAKE_IMU_PERIOD
nsh> param save
nsh> reboot
```

---

## 8. 参数调优建议

### 8.1 FFT 测试

```bash
# 推荐配置
FAKE_IMU_Z_F0 = 0
FAKE_IMU_Z_F1 = 1000  # 覆盖大多数电机频率
FAKE_IMU_PERIOD = 15  # 慢扫，提高精度
```

---

### 8.2 陷波滤波器测试

```bash
# 快速响应测试
FAKE_IMU_Z_F0 = 0
FAKE_IMU_Z_F1 = 2000
FAKE_IMU_PERIOD = 5   # 快扫，测试跟踪能力

# 稳定性测试
FAKE_IMU_Z_F0 = 100
FAKE_IMU_Z_F1 = 400
FAKE_IMU_PERIOD = 30  # 慢扫，测试长时间稳定性
```

---

### 8.3 多轴独立测试

```bash
# 只测试 Z 轴，其他轴设为 0
FAKE_IMU_X_F0 = 0
FAKE_IMU_X_F1 = 0    # 不扫频
FAKE_IMU_Y_F0 = 0
FAKE_IMU_Y_F1 = 0    # 不扫频
FAKE_IMU_Z_F0 = 0
FAKE_IMU_Z_F1 = 1000 # 只有 Z 轴扫频
```

---

## 9. QGroundControl 中配置

### 9.1 通过 QGC 修改参数

1. 连接飞控
2. 打开 **Vehicle Setup** → **Parameters**
3. 搜索 `FAKE_IMU`
4. 修改参数值
5. 点击 **Save** 或 **Apply**
6. 重启飞控

---

### 9.2 参数组说明

所有 fake_imu 参数都在 **Fake IMU** 组中，方便查找和管理。

---

## 10. 实际测试流程示例

### 完整的测试流程

```bash
# ========== 步骤1: 连接飞控 ==========
# 通过 USB 或串口连接

# ========== 步骤2: 配置参数 ==========
nsh> param set FAKE_IMU_Z_F0 0
nsh> param set FAKE_IMU_Z_F1 1500
nsh> param set FAKE_IMU_PERIOD 15
nsh> param save

# ========== 步骤3: 重启系统 ==========
nsh> reboot

# ========== 步骤4: 启动模块 ==========
nsh> fake_imu start
nsh> gyro_fft start

# ========== 步骤5: 监控数据 ==========
nsh> listener sensor_gyro_fft

# ========== 步骤6: 记录日志 ==========
nsh> logger on

# ========== 步骤7: 等待扫频完成 ==========
# 等待 2 × PERIOD 秒 (15 × 2 = 30 秒)

# ========== 步骤8: 停止并分析 ==========
nsh> fake_imu stop
nsh> gyro_fft stop
nsh> logger off

# 下载日志文件进行分析
```

---

## 11. 总结

### 11.1 核心要点

| 要点 | 说明 |
|------|------|
| **参数数量** | 7 个（3轴 × 2频率 + 1周期） |
| **默认配置** | X: 0-10Hz, Y: 0-100Hz, Z: 0-1000Hz, Period: 10s |
| **修改方式** | `param set` + `fake_imu restart` 或 `reboot` |
| **保存方式** | `param save` |
| **参数位置** | SD 卡：`/fs/microsd/params` |

---

### 11.2 优势

1. ✅ **灵活配置**：无需重新编译
2. ✅ **快速测试**：不同场景快速切换
3. ✅ **参数持久化**：配置保存到 SD 卡
4. ✅ **QGC 支持**：图形界面配置
5. ✅ **独立调节**：每个轴独立配置

---

### 11.3 相关文档

| 文档 | 说明 |
|------|------|
| `01-fake_imu传感器模拟器代码详解.md` | fake_imu 的完整分析 |
| `04-PX4Gyroscope参数系统与采样率配置详解.md` | 参数系统原理 |
| `20-PX4参数系统存储机制与配置流程详解.md` | 参数存储机制 |

---

**文档版本**：v1.0  
**创建日期**：2025-10-30  
**适用 PX4 版本**：v1.14+  
**作者**：基于 fake_imu 参数化改进整理

