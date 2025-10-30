# MicoAir H743 固件烧录故障排查与解决方案

## ❌ 问题描述

### 症状
1. JLink烧录时报错：`Failed to open file`
2. QGC无法连接飞控
3. 烧录过程显示完成，但实际固件未写入

### 错误日志
```
J-Link>loadfile ../../../build/micoair_h743_default/micoair_h743_default.elf
'loadfile': Performing implicit reset & halt of MCU.
Downloading file [../../../build/micoair_h743_default/micoair_h743_default.elf]...
Failed to open file.
```

---

## 🔍 原因分析

### 问题1：文件路径错误
- **相对路径问题**：在不同目录执行命令，相对路径会失效
- **JLink解析限制**：JLink对相对路径的解析可能不同于shell

### 问题2：文件类型选择错误
- **ELF vs BIN**：从非起始地址（0x08020000）烧录时，应使用BIN文件
- **地址指定**：BIN文件需要明确指定烧录地址

### 问题3：Bootloader保护配置
MicoAir H743的Flash布局：
```
0x08000000 - 0x0801FFFF  [128KB]  Bootloader区域（保护）
0x08020000 - 0x081FFFFF  [1.87MB] PX4固件区域（可擦写）
```

---

## ✅ 解决方案

### 方案1：使用修复后的JLink脚本（推荐）

已修复的 `download_firmware.jlink`：

```jlink
// 连接配置
si SWD
speed 4000

// 连接到目标
connect

// 只擦除固件区域（保护Bootloader）
erase 0x08020000 0x081FFFFF

// 下载BIN文件（使用绝对路径和明确地址）
loadbin /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot/build/micoair_h743_default/micoair_h743_default.bin 0x08020000

// 验证下载
verifybin /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot/build/micoair_h743_default/micoair_h743_default.bin 0x08020000

// 复位并运行
r
g

// 退出
qc
```

**使用方法**：
```bash
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot/boards/micoair/h743/debug
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile download_firmware.jlink
```

### 方案2：使用命令行直接烧录

```bash
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot

# 方法A：使用JLink命令行
JLinkExe -device STM32H743VI -if SWD -speed 4000 \
         -CommanderScript - << EOF
connect
erase 0x08020000 0x081FFFFF
loadbin build/micoair_h743_default/micoair_h743_default.bin 0x08020000
verifybin build/micoair_h743_default/micoair_h743_default.bin 0x08020000
r
g
qc
EOF

# 方法B：使用make上传（如果支持）
make micoair_h743_default upload
```

### 方案3：使用JFlash GUI工具

1. 打开JFlash
2. 创建新项目，选择 STM32H743VI
3. File → Open data file → 选择 `build/micoair_h743_default/micoair_h743_default.bin`
4. 设置起始地址为 **0x08020000**
5. Target → Connect
6. Target → Erase Chip (或手动选择扇区)
7. Target → Program & Verify

---

## 📋 烧录验证步骤

### 步骤1：烧录前检查
```bash
# 检查固件文件是否存在
ls -lh build/micoair_h743_default/micoair_h743_default.bin

# 检查文件大小（应该约1.9MB）
# 不能超过1.87MB（Flash固件区大小）
```

### 步骤2：执行烧录
```bash
cd boards/micoair/h743/debug
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile download_firmware.jlink
```

### 步骤3：查看烧录日志
✅ **成功标志**：
```
J-Link: Flash download: Total time needed: X.XXXs
Downloading file [...]...
O.K.
Verifying...
O.K.
```

❌ **失败标志**：
```
Failed to open file
Error while programming flash
Verify failed
```

### 步骤4：验证固件运行

#### 方法A：串口检查
```bash
# 连接串口（MAVLink端口）
screen /dev/ttyACM0 115200

# 应该看到PX4启动信息
# 如果没有，说明固件未正确运行
```

#### 方法B：LED指示
- **正常启动**：LED按一定模式闪烁
- **启动失败**：LED不亮或异常闪烁

#### 方法C：QGC连接
- 打开QGroundControl
- 选择正确的串口
- 应该能自动连接

---

## 🐛 常见问题排查

### Q1: 烧录后QGC无法连接

**可能原因**：
1. ❌ 固件未成功烧录（查看JLink日志）
2. ❌ 串口驱动问题（`lsusb` 检查是否识别）
3. ❌ 波特率设置错误（默认57600或115200）
4. ❌ MAVLink配置问题

**解决方法**：
```bash
# 1. 检查USB设备
lsusb | grep -i "STM\|PX4"

# 2. 检查串口设备
ls /dev/ttyACM*

# 3. 使用串口工具测试
screen /dev/ttyACM0 57600
# 输入回车，应该看到 "nsh>" 提示符

# 4. 重新烧录完整固件（包括bootloader）
# 使用 restore_bootloader.jlink 恢复
```

### Q2: JLink报错 "Cannot connect to target"

**可能原因**：
1. ❌ 硬件连接问题
2. ❌ 供电不足
3. ❌ SWD接口被禁用

**解决方法**：
```bash
# 1. 检查硬件连接
JLinkExe
> connect
# 如果失败，检查SWDIO, SWCLK, GND, VTref连接

# 2. 尝试降低速度
speed 100
connect

# 3. 尝试硬件复位
# 按住板子上的RESET按钮，然后连接
```

### Q3: 烧录地址是否正确？

**验证方法**：
```bash
# 查看ELF文件的加载地址
arm-none-eabi-readelf -l build/micoair_h743_default/micoair_h743_default.elf | grep LOAD
```

**预期输出**：
```
LOAD  0x010000 0x08020000 0x08020000 ...
```

如果起始地址是 `0x08020000`，说明地址正确（保护了bootloader）。

### Q4: 如何恢复Bootloader？

如果不小心擦除了bootloader：

```bash
cd boards/micoair/h743/debug

# 使用恢复脚本
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile restore_bootloader.jlink

# 或手动执行
JLinkExe -device STM32H743VI -if SWD -speed 4000
> connect
> erase  # 全片擦除
> loadbin bootloader.bin 0x08000000  # 烧录bootloader
> loadbin firmware.bin 0x08020000    # 烧录固件
> r
> g
> qc
```

---

## 📊 不同烧录方式对比

| 方式 | 优点 | 缺点 | 推荐场景 |
|------|------|------|---------|
| **JLink脚本** | 自动化、可重复 | 需要配置脚本 | ✅ 日常开发 |
| **JFlash GUI** | 直观、可视化 | 手动操作 | 调试烧录问题 |
| **make upload** | 最简单 | 依赖工具链配置 | 快速烧录 |
| **GDB调试** | 可同时调试 | 速度慢 | 开发调试 |

---

## 🔧 脚本文件说明

### `download_firmware.jlink`
- **用途**：只烧录固件，保护Bootloader
- **擦除范围**：0x08020000 - 0x081FFFFF
- **适用场景**：日常开发更新

### `restore_bootloader.jlink`
- **用途**：恢复Bootloader和固件
- **擦除范围**：全片擦除（0x08000000 - 0x081FFFFF）
- **适用场景**：首次烧录、Bootloader损坏

### `erase_all.jlink`
- **用途**：完全擦除Flash
- **擦除范围**：全片
- **适用场景**：清除所有配置、恢复出厂

---

## ⚡ 快速修复命令

### 问题：无法连接

```bash
# 重新烧录完整固件
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot/boards/micoair/h743/debug
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile restore_bootloader.jlink
```

### 问题：QGC无法连接

```bash
# 1. 检查串口
ls -l /dev/ttyACM0

# 2. 设置权限
sudo chmod 666 /dev/ttyACM0

# 3. 重启飞控
# 拔插USB或按复位键
```

### 问题：烧录卡在某个阶段

```bash
# 终止JLink
Ctrl+C

# 手动复位芯片
JLinkExe
> connect
> r  # reset
> qc
```

---

## 📝 完整烧录流程（推荐）

```bash
#!/bin/bash
# 完整烧录流程

# 1. 编译固件
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot
make micoair_h743_default

# 2. 检查文件
ls -lh build/micoair_h743_default/micoair_h743_default.bin

# 3. 连接JLink并烧录
cd boards/micoair/h743/debug
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile download_firmware.jlink

# 4. 验证
echo "请检查："
echo "1. JLink日志中是否有 'O.K.'"
echo "2. 串口是否出现：ls /dev/ttyACM0"
echo "3. QGC是否能连接"

# 5. 串口测试
echo "测试串口连接..."
screen /dev/ttyACM0 57600
```

---

## 🎯 总结

### 问题根源
1. ✅ **路径问题**：JLink脚本使用相对路径导致找不到文件
2. ✅ **文件类型**：应使用BIN文件而不是ELF文件
3. ✅ **地址指定**：BIN文件必须明确指定烧录地址

### 解决方法
1. ✅ 使用**绝对路径**
2. ✅ 使用 `loadbin` 命令而不是 `loadfile`
3. ✅ 明确指定烧录地址 **0x08020000**
4. ✅ 添加 `verifybin` 验证步骤

### 最佳实践
- 📌 使用修复后的JLink脚本
- 📌 每次烧录后验证
- 📌 保持Bootloader区域受保护
- 📌 记录每次烧录日志

---

**最后更新**: 2025年10月29日
**适用固件**: PX4 v1.17+
**硬件平台**: MicoAir H743

