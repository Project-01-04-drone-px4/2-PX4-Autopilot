# ERmao 优化日志系统 - 实施检查清单

## 📋 总体进度

- [x] **阶段 1**: 创建精简 msg 定义（8 个）
- [x] **阶段 2**: 创建数据转换模块（2 个）
- [x] **阶段 3**: 更新日志配置
- [x] **阶段 4**: 创建文档（4 个）
- [ ] **阶段 5**: 手动配置（3 个文件）← **当前阶段**
- [ ] **阶段 6**: 编译测试
- [ ] **阶段 7**: 仿真验证
- [ ] **阶段 8**: 真机测试

---

## ✅ 已完成的工作

### 阶段 1-4：自动创建的文件（13 个）

| # | 文件路径 | 状态 | 大小 | 说明 |
|---|---------|------|------|------|
| 1 | `msg/LogGyroFifo.msg` | ✅ | ~800 B | 精简 gyro FIFO |
| 2 | `msg/LogAngularVelocity.msg` | ✅ | ~600 B | 精简角速度 |
| 3 | `msg/LogVehicleImu.msg` | ✅ | ~700 B | 精简 IMU |
| 4 | `msg/LogSensorCombined.msg` | ✅ | ~600 B | 精简融合传感器 |
| 5 | `msg/LogAttitude.msg` | ✅ | ~600 B | 精简姿态 |
| 6 | `msg/LogAttitudeSetpoint.msg` | ✅ | ~650 B | 精简姿态设定值 |
| 7 | `msg/LogRatesSetpoint.msg` | ✅ | ~600 B | 精简角速度设定值 |
| 8 | `msg/LogActuatorMotors.msg` | ✅ | ~650 B | 精简电机输出 |
| 9 | `src/modules/logger/ERmaoLogPublisher.hpp` | ✅ | ~3.5 KB | 转换模块头文件 |
| 10 | `src/modules/logger/ERmaoLogPublisher.cpp` | ✅ | ~8 KB | 转换模块实现 |
| 11 | `docm/ERmao优化日志实施指南.md` | ✅ | ~25 KB | 完整实施指南 |
| 12 | `docm/ERmao优化日志快速参考.md` | ✅ | ~10 KB | 快速参考 |
| 13 | `docm/ERmao优化日志系统总结.md` | ✅ | ~18 KB | 系统总结 |

**总计**：13 个文件，~69.25 KB

---

## ⚠️ 待完成的手动配置

### 任务 1: 创建参数定义文件

**文件路径**：`src/modules/logger/params.c`

**操作**：创建新文件，内容如下：

```c
/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file params.c
 * Logger module parameters
 */

/**
 * Enable ERmao optimized logging
 *
 * When enabled, the ERmaoLogPublisher module converts full-size uORB topics
 * to compact log-optimized versions. This enables full-rate logging without
 * data loss by:
 *  - Removing redundant fields (device IDs, calibration counters, etc.)
 *  - Using larger uORB queue depths (16-32 vs 4-8)
 *  - Reducing message size by 30-64%
 *  - Maintaining full sampling rate for signal chain analysis
 *
 * Total data rate reduced from ~160 KB/s to ~90 KB/s (-45%)
 * Memory overhead: +6 KB for larger queues
 *
 * @boolean
 * @reboot_required true
 * @group Logger
 */
PARAM_DEFINE_INT32(ERMAO_LOG_ENABLE, 1);
```

**检查**：
- [ ] 文件创建成功
- [ ] 参数名称正确：`ERMAO_LOG_ENABLE`
- [ ] 默认值为 1（启用）

---

### 任务 2: 修改 CMakeLists.txt

**文件路径**：`src/modules/logger/CMakeLists.txt`

**操作**：在 `SRCS` 部分添加两行

**查找**：
```cmake
px4_add_module(
    MODULE modules__logger
    MAIN logger
    COMPILE_FLAGS
    SRCS
        logger.cpp
        logged_topics.cpp
        log_writer.cpp
        log_writer_file.cpp
        log_writer_mavlink.cpp
        messages.cpp
```

**修改为**：
```cmake
px4_add_module(
    MODULE modules__logger
    MAIN logger
    COMPILE_FLAGS
    SRCS
        logger.cpp
        logged_topics.cpp
        log_writer.cpp
        log_writer_file.cpp
        log_writer_mavlink.cpp
        messages.cpp
        ERmaoLogPublisher.cpp  # ← 添加这一行
        params.c               # ← 添加这一行
```

**检查**：
- [ ] 找到正确的 CMakeLists.txt 文件
- [ ] 在 SRCS 列表中添加了 `ERmaoLogPublisher.cpp`
- [ ] 在 SRCS 列表中添加了 `params.c`
- [ ] 保持正确的缩进（使用空格或 tab，与其他行一致）

---

### 任务 3: 添加启动脚本

**文件路径**：查找启动脚本（通常是以下之一）
- `ROMFS/px4fmu_common/init.d/rcS`
- `ROMFS/px4fmu_common/init.d/rc.logging`
- 或相关的启动脚本

**操作**：在合适位置添加启动命令

**添加位置**：在 logger 启动之前，或在 "Start system logging" 相关部分

**添加内容**：
```bash
#
# Start ERmao log publisher for optimized logging
#
if param compare ERMAO_LOG_ENABLE 1
then
    ermao_log_publisher start
    if [ $? -eq 0 ]
    then
        echo "ERmao log publisher started"
    else
        echo "ERmao log publisher failed to start"
    fi
fi
```

**检查**：
- [ ] 找到正确的启动脚本
- [ ] 添加了启动命令
- [ ] 启动命令在 logger 之前或同一部分
- [ ] 使用 tab 缩进（与其他行一致）

---

## 🔨 编译与测试

### 步骤 1: 清理旧构建

```bash
cd /home/gg/01-code/01-px4/01-fork-gg/2-PX4-Autopilot
make clean
```

**检查**：
- [ ] 执行成功
- [ ] `build/` 目录被清空

---

### 步骤 2: 编译 SITL

```bash
make px4_sitl_default 2>&1 | tee build.log
```

**预期输出**：
```
[1/445] ...
[2/445] ...
...
[445/445] Linking CXX shared library ...

Build succeeded
```

**检查**：
- [ ] 编译成功，无错误
- [ ] 如果有错误，查看 `build.log`
- [ ] 确认新的 msg 文件被编译（查找 `LogGyroFifo` 等）

**常见错误**：
- ❌ `找不到 ERmaoLogPublisher.cpp` → 检查 CMakeLists.txt
- ❌ `找不到 log_gyro_fifo.h` → 运行 `make clean` 后重试
- ❌ `语法错误` → 检查 params.c 语法

---

### 步骤 3: 启动仿真

```bash
make px4_sitl gazebo
```

**检查**：
- [ ] Gazebo 成功启动
- [ ] 无人机模型加载
- [ ] MAVLink 控制台可用

---

### 步骤 4: 验证模块运行

在 MAVLink 控制台中执行：

```bash
# 检查参数
param show ERMAO_LOG_ENABLE
```

**预期输出**：
```
ERMAO_LOG_ENABLE: 1
```

**检查**：
- [ ] 参数存在
- [ ] 参数值为 1

```bash
# 检查模块状态
ermao_log_publisher status
```

**预期输出**：
```
Running
ERmao logging enabled: YES
```

**检查**：
- [ ] 模块正在运行
- [ ] 日志启用状态为 YES

---

### 步骤 5: 验证主题发布

```bash
# 检查精简主题是否发布
uorb top log_angular_velocity
```

**预期输出**：
```
update rate: ~667 Hz
```

**检查**：
- [ ] 主题存在
- [ ] 更新频率正确（~667 Hz）

```bash
uorb top log_attitude
```

**预期输出**：
```
update rate: ~250 Hz
```

**检查**：
- [ ] 主题存在
- [ ] 更新频率正确（~250 Hz）

```bash
uorb top log_gyro_fifo
```

**预期输出**（如果硬件支持）：
```
update rate: ~50 Hz
```

**检查**：
- [ ] 主题存在（或显示 "topic not found" 如果硬件不支持）
- [ ] 如果存在，更新频率正确（~50 Hz）

---

### 步骤 6: 配置日志参数

```bash
# 启用 ERmao 日志模式
param set SDLOG_PROFILE 4097
# 4097 = DEFAULT (1) + ERMAO (4096)

# 保存参数
param save

# 重启（如果需要）
reboot
```

**检查**：
- [ ] 参数设置成功
- [ ] 参数已保存
- [ ] 重启后参数保持

---

### 步骤 7: 飞行测试并检查日志

飞行 1 分钟后：

```bash
# 在开发机上，不是在 MAVLink 控制台
python3 << 'EOF'
from pyulog import ULog
import numpy as np

# 加载最新日志
ulog = ULog('最新的日志文件.ulg')

# 检查精简主题
topics = [msg.name for msg in ulog.data_list]
log_topics = sorted([t for t in topics if t.startswith('log_')])

print("✓ Found compact log topics:")
for topic in log_topics:
    data = ulog.get_dataset(topic).data
    print(f"  {topic}: {len(data['timestamp'])} samples")

# 检查数据完整性
log_rates_sp = ulog.get_dataset('log_rates_setpoint').data
samples = len(log_rates_sp['timestamp'])
expected = 250 * 60  # 250 Hz × 60 s
print(f"\n✓ Data completeness:")
print(f"  Samples: {samples}")
print(f"  Expected: {expected}")
print(f"  Completeness: {samples/expected*100:.1f}%")

# 检查数据丢失
dt = np.diff(log_rates_sp['timestamp'] * 1e-6)
max_interval = np.max(dt)
print(f"\n✓ Data loss check:")
print(f"  Max interval: {max_interval*1000:.2f} ms")
print(f"  Expected: ~4 ms")
if max_interval < 0.01:
    print("  Result: NO DATA LOSS ✓")
else:
    print("  Result: DATA LOSS DETECTED ✗")
EOF
```

**检查**：
- [ ] 所有 log_* 主题都存在
- [ ] 样本数量符合预期（~15000 for 1 min @ 250 Hz）
- [ ] 完整性 > 95%
- [ ] 无数据丢失（最大间隔 < 10 ms）

---

## 🐛 故障排查

### 问题 1: 编译失败 - 找不到头文件

**症状**：
```
fatal error: uORB/topics/log_gyro_fifo.h: No such file or directory
```

**解决**：
```bash
make clean
make px4_sitl_default
```

---

### 问题 2: ERmao log publisher 未启动

**症状**：
```bash
ermao_log_publisher status
# 输出: not running
```

**排查步骤**：

1. 检查参数：
```bash
param show ERMAO_LOG_ENABLE
```
- 如果不存在 → params.c 未正确添加，重新编译
- 如果为 0 → 设置为 1：`param set ERMAO_LOG_ENABLE 1`

2. 检查启动脚本：
```bash
# 查看启动日志
dmesg | grep ermao
```

3. 手动启动：
```bash
ermao_log_publisher start
```

---

### 问题 3: 日志中没有 log_* 主题

**症状**：pyulog 找不到 `log_angular_velocity` 等主题

**排查步骤**：

1. 确认模块运行：
```bash
ermao_log_publisher status
```

2. 确认主题发布：
```bash
uorb top log_angular_velocity
```

3. 确认日志模式：
```bash
param show SDLOG_PROFILE
# 应该包含 bit 12（值 >= 4096）
```

4. 重启系统：
```bash
reboot
```

---

### 问题 4: 数据仍然丢失

**症状**：最大时间间隔 > 10 ms

**排查步骤**：

1. 检查 SD 卡速度（使用 Class 10 或更快）

2. 增加队列深度：编辑 msg 文件，增加 `ORB_QUEUE_LENGTH`
```
uint8 ORB_QUEUE_LENGTH = 64  # 从 32 增加到 64
```

3. 检查系统负载：
```bash
top
# CPU 使用率应该 < 80%
```

4. 检查 ERmaoLogPublisher 频率：
编辑 `ERmaoLogPublisher.cpp`，降低频率：
```cpp
ScheduleOnInterval(4_ms);  # 从 2ms 改为 4ms（降低到 250 Hz）
```

---

## ✅ 最终检查清单

### 配置完成度

- [ ] `src/modules/logger/params.c` 已创建
- [ ] `src/modules/logger/CMakeLists.txt` 已修改
- [ ] 启动脚本已修改
- [ ] 编译成功无错误
- [ ] 仿真成功启动
- [ ] ERmao log publisher 运行中
- [ ] 所有 log_* 主题正常发布
- [ ] 参数 `ERMAO_LOG_ENABLE = 1`
- [ ] 参数 `SDLOG_PROFILE` 包含 bit 12
- [ ] 日志文件包含所有 log_* 主题
- [ ] 数据完整性 > 95%
- [ ] 无数据丢失

### 性能验证

- [ ] 日志大小合理（1分钟 ~5.5 MB）
- [ ] CPU 使用率正常（< 80%）
- [ ] SD 卡写入速度正常
- [ ] 内存使用正常

---

## 📞 获取帮助

如果遇到问题，请参考：

1. **完整实施指南**：`docm/ERmao优化日志实施指南.md`
2. **快速参考**：`docm/ERmao优化日志快速参考.md`
3. **系统总结**：`docm/ERmao优化日志系统总结.md`
4. **数据丢失分析**：`docm/sensor_gyro_fifo记录说明.md`

---

**检查清单版本**：1.0
**最后更新**：2025-11-01
**下一步**：完成上述 3 个手动配置任务，然后编译测试

