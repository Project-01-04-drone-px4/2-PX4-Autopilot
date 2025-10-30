# MicoAir H743 开发调试快速指南

## 🚀 快速入门（3步完成）

### 步骤1: 开启模块编译 ✅

模块已自动添加到配置文件中！位置：
```
boards/micoair/h743/default.px4board
最后一行: CONFIG_MODULES_GG_IMU_LOGGER=y
```

### 步骤2: 编译固件

```bash
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot

# 首次编译（完整）
make micoair_h743_default

# 后续修改代码后（增量编译，很快）
make micoair_h743_default
```

### 步骤3: JLink下载

```bash
cd boards/micoair/h743/debug

# 使用JLink命令行下载
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile download_firmware.jlink
```

**完成！** 固件已下载，Bootloader完好无损！

---

## 📋 方法对比

### 方法A: JLink命令行（最快）

```bash
# 优点：最快速，适合频繁下载
cd boards/micoair/h743/debug
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile download_firmware.jlink

# 耗时：约10秒
```

### 方法B: Ozone图形界面（推荐调试）

```bash
# 优点：可以设置断点、单步执行、查看变量
cd boards/micoair/h743/debug
ozone micoair_h743.jdebug

# 在Ozone中：
# 1. File → Open → 选择 micoair_h743.jdebug
# 2. Download & Reset Program (F5)
# 3. 设置断点开始调试
```

### 方法C: QGC上传（最稳妥）

```bash
# 优点：最稳定，适合最终测试和发布
# 缺点：较慢，无法调试

make micoair_h743_default
# 生成：build/micoair_h743_default/micoair_h743_default.px4

# 在QGC中上传
```

---

## ⚠️ 重要提示

### Bootloader保护

**MicoAir H743的Flash布局**：
```
0x08000000 ┌─────────────────────────────────┐
           │  Bootloader (32KB)              │  ← 不要擦除！
0x08008000 ├─────────────────────────────────┤
           │  Reserved (96KB)                │  ← 不要擦除！
0x08020000 ├─────────────────────────────────┤
           │  PX4 Firmware (1920KB)          │  ← 只擦除这里
0x081FFFFF └─────────────────────────────────┘
```

**JLink会自动保护Bootloader区域**（如果使用提供的脚本）

### 验证Bootloader是否完整

```bash
JLinkExe -device STM32H743VI -if SWD -speed 4000

J-Link> connect
J-Link> mem8 0x08000000 16

# 正常输出（示例）：
# 08000000 = 20 01 00 20 09 00 00 08 ...
#           └─ Stack  └─ Reset Handler

# 被擦除的输出：
# 08000000 = FF FF FF FF FF FF FF FF ...
```

---

## 🔧 故障排查

### 问题1: JLink无法连接

**解决方法**：
```bash
# 1. 检查硬件连接
# 2. 降低连接速度
JLinkExe -device STM32H743VI -if SWD -speed 1000

# 3. 使用Connect Under Reset
```

### 问题2: 下载后程序不运行

**原因**: 地址配置错误

**检查**：
```bash
# 查看链接脚本
cat ../nuttx-config/scripts/script.ld | grep "ORIGIN"

# 必须看到：flash (rx)  : ORIGIN = 0x08020000
```

### 问题3: Bootloader被擦除

**恢复**：
```bash
# 使用恢复脚本
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile restore_bootloader.jlink

# 注意：需要先准备bootloader.bin文件
```

---

## 📝 开发流程推荐

```
修改代码
  ↓
make micoair_h743_default (30秒)
  ↓
JLink下载 (10秒)
  ↓
Ozone调试/测试
  ↓
发现问题，继续修改...
```

**总时间**：每次迭代约40秒！比QGC上传快10倍！

---

## 📚 参考文档

详细说明请查看：
- `docm/24-PX4模块编译配置与JLink调试烧录指南.md` - 完整教程
- `docm/22-PX4自定义IMU日志模块开发详解.md` - 模块开发
- `docm/23-PX4_dmesg输出解析与源码追踪.md` - 启动日志分析

