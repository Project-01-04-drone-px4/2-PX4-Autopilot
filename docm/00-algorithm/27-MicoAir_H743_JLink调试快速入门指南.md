# 🚀 MicoAir H743 JLink调试快速开始

## ✅ 已完成的配置

所有配置已自动完成！您可以直接开始使用：

- ✅ gg_imu_logger模块已添加到编译配置
- ✅ JLink下载脚本已创建
- ✅ Ozone调试配置已创建
- ✅ 板级参数已配置

---

## 📦 第一步：编译固件

```bash
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot

# 编译（第一次约5分钟）
make micoair_h743_default
```

**编译成功后会生成**：
- `build/micoair_h743_default/micoair_h743_default.elf` ← JLink使用此文件
- `build/micoair_h743_default/micoair_h743_default.bin`
- `build/micoair_h743_default/micoair_h743_default.px4`

---

## 🔌 第二步：连接JLink

**硬件连接**：
```
JLink          MicoAir H743
─────────────────────────────
VTref    →     3.3V
GND      →     GND
SWDIO    →     SWDIO
SWCLK    →     SWCLK
```

**测试连接**：
```bash
JLinkExe -device STM32H743VI -if SWD -speed 4000

J-Link> connect
Device "STM32H743VI" selected.
Connecting to target via SWD
Found SW-DP with ID 0x6BA02477
Found SW-DP with ID 0x6BA02477
Cortex-M7 identified.
J-Link> q  # 退出
```

---

## 📥 第三步：下载固件（两种方式）

### 方式A：JLink命令行（最快，10秒）

```bash
cd boards/micoair/h743/debug

# 下载固件（自动保护Bootloader）
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile download_firmware.jlink
```

**输出示例**：
```
Connecting to J-Link via USB...
Downloading file [build/micoair_h743_default/micoair_h743_default.elf]...
Comparing flash   [100%] Done.
Erasing flash     [100%] Done.
Programming flash [100%] Done.
Verifying flash   [100%] Done.
J-Link: Flash download: Total time needed: 8.723s
Reset delay: 0 ms
Reset type NORMAL: Reboot via AIRCR.SYSRESETREQ.
```

### 方式B：Ozone调试（推荐开发）

```bash
cd boards/micoair/h743/debug

# 启动Ozone
ozone micoair_h743.jdebug

# 在Ozone界面中：
# 1. 点击 "Download & Reset Program" 按钮（或按F5）
# 2. 程序自动下载并运行
# 3. 可以设置断点进行调试
```

---

## ✨ 重要说明

### ⚠️ Bootloader保护

**Flash地址布局**：
```
0x08000000 ┌────────────────────┐
           │ Bootloader (32KB)  │ ← JLink不会擦除这里！
0x08008000 ├────────────────────┤
           │ Reserved (96KB)    │ ← JLink不会擦除这里！
0x08020000 ├────────────────────┤
           │ PX4 Firmware       │ ← JLink只擦除从这里开始
           │ (1920KB)           │
0x081FFFFF └────────────────────┘
```

**为什么安全？**
- JLink脚本配置为只擦除`0x08020000`以后的区域
- Bootloader区域（`0x08000000 - 0x0801FFFF`）**完全不受影响**
- 您可以放心使用JLink下载，不会破坏Bootloader

### 验证Bootloader完整性

```bash
JLinkExe -device STM32H743VI -if SWD -speed 4000

J-Link> connect
J-Link> mem8 0x08000000 16

# 正常输出（Bootloader存在）：
08000000 = 20 01 00 20 09 00 00 08 ...

# 被擦除（需要恢复）：
08000000 = FF FF FF FF FF FF FF FF ...
```

---

## 🐛 调试模块

### 在Ozone中设置断点

1. 打开源文件：`src/modules/gg_imu_logger/gg_imu_logger.cpp`
2. 在`Run()`函数中点击行号设置断点
3. Download & Reset Program (F5)
4. 程序会在断点处停止

**观察变量**：
```
View → Watched Data
  添加：
  - _log_fd
  - _log_write_count
  - imu_data.timestamp
```

### 实时监控uORB消息

```bash
# 在NSH中（通过USB串口或QGC MAVLink Console）
nsh> listener vehicle_imu
```

---

## 📊 开发效率对比

| 方法 | 编译时间 | 下载时间 | 总时间 | 调试能力 |
|------|---------|---------|--------|---------|
| **QGC上传** | 5分钟 | 2-3分钟 | ~8分钟 | ❌ 无 |
| **JLink命令** | 5分钟 | **10秒** | ~5分钟 | ❌ 无 |
| **JLink增量** | **30秒** | **10秒** | **40秒** | ❌ 无 |
| **Ozone调试** | 5分钟 | **10秒** | ~5分钟 | ✅ **完整调试** |

**推荐**：开发时使用Ozone，测试时偶尔用QGC验证。

---

## 🆘 常见问题

### Q1: 使用JLink下载会破坏Bootloader吗？

**答**：不会！提供的JLink脚本已配置为只擦除固件区域（0x08020000开始）。

### Q2: 如果Bootloader被破坏了怎么办？

**答**：使用恢复脚本：
```bash
cd boards/micoair/h743/debug
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile restore_bootloader.jlink
```

### Q3: 为什么固件从0x08020000开始而不是0x08008000？

**答**：STM32H7的Flash扇区大小为128KB。为了对齐扇区边界：
- Sector 0 (0x08000000-0x0801FFFF): 保留给Bootloader
- Sector 1 (0x08020000-0x0803FFFF): 固件开始

### Q4: 能直接擦除全片Flash重新烧录吗？

**答**：可以，但会失去Bootloader：
```bash
# 这会擦除全部Flash（包括Bootloader）
JLinkExe -device STM32H743VI -if SWD -speed 4000
J-Link> erase  # 擦除全部
J-Link> loadfile xxx.elf  # 重新烧录
```

之后需要用`restore_bootloader.jlink`恢复Bootloader。

---

## 📚 详细文档

- **完整教程**: `docm/24-PX4模块编译配置与JLink调试烧录指南.md`
- **模块开发**: `docm/22-PX4自定义IMU日志模块开发详解.md`
- **启动分析**: `docm/23-PX4_dmesg输出解析与源码追踪.md`
- **rcS机制**: `docm/21-PX4_rcS启动脚本机制详解.md`

---

## 🎯 开始使用

现在就可以开始了！

```bash
# 1. 编译
make micoair_h743_default

# 2. 连接JLink

# 3. 下载
cd boards/micoair/h743/debug
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile download_firmware.jlink

# 4. 验证（通过串口）
screen /dev/ttyACM0 57600
nsh> gg_imu_logger status
```

**祝开发顺利！** 🎉

