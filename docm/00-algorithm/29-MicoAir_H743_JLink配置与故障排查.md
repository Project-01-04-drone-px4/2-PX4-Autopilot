# MicoAir H743 JLink调试快速指南

## 1. 快速开始

### 1.1 开启模块编译

```bash
# 方法1: 直接修改配置文件（最简单）
echo "CONFIG_MODULES_GG_IMU_LOGGER=y" >> boards/micoair/h743/default.px4board

# 方法2: 使用menuconfig
make micoair_h743_default menuconfig
# 导航到: PX4 Configuration → modules → [*] gg_imu_logger
```

### 1.2 编译固件

```bash
# 清理并编译
make clean
make micoair_h743_default

# 增量编译（修改代码后）
make micoair_h743_default
```

### 1.3 使用JLink下载

```bash
cd boards/micoair/h743/debug

# 下载固件（保护Bootloader）
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile download_firmware.jlink
```

## 2. 重要地址说明

**MicoAir H743 Flash布局**:
```
0x08000000 - 0x08007FFF  (32KB)   Bootloader
0x08008000 - 0x0801FFFF  (96KB)   Reserved
0x08020000 - 0x081FFFFF  (1920KB) PX4 Firmware  ← ELF加载地址
```

**⚠️ 关键点**:
- 固件起始地址：**0x08020000**（不是0x08000000或0x08008000！）
- Ozone下载时必须设置这个地址
- JLink脚本擦除范围：0x08020000 - 0x081FFFFF

## 3. Ozone配置

1. **设备**: STM32H743VI
2. **接口**: SWD, 4000 kHz
3. **程序文件**: `build/micoair_h743_default/micoair_h743_default.elf`
4. **下载地址**: **0x08020000**
5. **擦除选项**: Erase Affected Ranges Only（不要Erase All！）

## 4. 验证Bootloader未被破坏

```bash
# 使用JLink Commander
JLinkExe -device STM32H743VI -if SWD -speed 4000

J-Link> connect
J-Link> mem8 0x08000000 16

# 应该看到有效的向量表：
# 08000000 = 20 01 00 20  (Stack Pointer)
# 08000004 = XX XX XX 08  (Reset Handler, 地址在0x08000000范围内)

# 如果全是 FF FF FF FF，说明Bootloader被擦除了！
```

## 5. 故障排查

### JLink无法连接
```bash
# 降低速度重试
JLinkExe -device STM32H743VI -if SWD -speed 1000

# 或使用硬件复位连接
# 按住NRST → JLink connect → 释放NRST
```

### 程序下载后不运行
```bash
# 检查链接脚本地址
grep "ORIGIN" ../nuttx-config/scripts/script.ld
# 必须是: ORIGIN = 0x08020000

# 在Ozone中检查向量表地址
# Debug → Registers → SCB.VTOR
# 应该是: 0x08020000
```

### Bootloader被破坏
```bash
# 使用restore_bootloader.jlink恢复
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile restore_bootloader.jlink
```

## 6. 常用命令

```bash
# 编译
make micoair_h743_default

# 下载（保护Bootloader）
cd boards/micoair/h743/debug
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile download_firmware.jlink

# Ozone调试
ozone micoair_h743.jdebug
```

## 7. 参考文档

详细说明请查看：
- `docm/24-PX4模块编译配置与JLink调试烧录指南.md`
- `docm/22-PX4自定义IMU日志模块开发详解.md`

