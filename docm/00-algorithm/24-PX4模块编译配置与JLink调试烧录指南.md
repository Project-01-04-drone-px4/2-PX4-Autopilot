# 24-PX4模块编译配置与JLink调试烧录指南
## MicoAir H743 开发调试完整流程

---

## 1. 概述

本文档详细说明如何：
1. 使用menuconfig开启自定义模块（以gg_imu_logger为例）
2. 使用JLink/Ozone直接调试下载ELF文件
3. 配置内存映射，保护Bootloader不被擦除
4. 优化开发调试流程

### 1.1 为什么要使用JLink调试下载

**传统方式（通过QGC升级）的问题**：
- ❌ 需要先编译.bin文件
- ❌ 需要连接QGC
- ❌ 上传固件较慢
- ❌ 无法实时调试
- ❌ 需要保留Bootloader

**JLink/Ozone方式的优势**：
- ✅ 直接下载ELF文件（包含调试符号）
- ✅ 可以实时单步调试
- ✅ 快速烧录（几秒钟）
- ✅ 可以设置断点、观察变量
- ✅ 可以配置保护Bootloader区域

---

## 2. 开启模块编译

### 2.1 方法1: 直接修改.px4board配置文件（推荐）

**文件**: `boards/micoair/h743/default.px4board`

在文件末尾添加gg_imu_logger模块配置：

```bash
# 编辑配置文件
vim boards/micoair/h743/default.px4board

# 在文件末尾添加
CONFIG_MODULES_GG_IMU_LOGGER=y
```

**完整示例**：
```kconfig
# ... 前面的配置保持不变 ...
CONFIG_SYSTEMCMDS_VER=y
CONFIG_SYSTEMCMDS_WORK_QUEUE=y

# 添加gg_imu_logger模块
CONFIG_MODULES_GG_IMU_LOGGER=y
```

**编译**：
```bash
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot

# 清理旧的构建
make clean

# 编译（会自动应用新配置）
make micoair_h743_default
```

---

### 2.2 方法2: 使用menuconfig图形界面

#### 步骤1: 进入menuconfig

```bash
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot

# 启动menuconfig
make micoair_h743_default menuconfig
```

#### 步骤2: 导航到模块配置

```
使用方向键和回车键导航：

Main Menu
  └─> PX4 Configuration
      └─> modules
          └─> [*] gg_imu_logger  ← 按空格键选中（显示为[*]）
```

**菜单路径**:
```
Main Menu
  → PX4 Configuration
    → modules
      → gg_imu_logger
```

#### 步骤3: 保存并退出

1. 按 `Esc` 键两次返回上级菜单
2. 继续按 `Esc` 直到出现保存提示
3. 选择 `Yes` 保存配置
4. 退出menuconfig

#### 步骤4: 编译固件

```bash
# menuconfig保存后直接编译
make micoair_h743_default

# 检查模块是否被编译
ls build/micoair_h743_default/src/modules/gg_imu_logger/
```

---

### 2.3 验证模块已编译

```bash
# 检查编译输出
ls -lh build/micoair_h743_default/src/modules/gg_imu_logger/gg_imu_logger.o

# 检查符号表中是否包含模块
arm-none-eabi-nm build/micoair_h743_default/micoair_h743_default.elf | grep gg_imu_logger

# 输出示例:
# 0800a2b0 T gg_imu_logger_main
# 0800a2c4 t GgImuLogger::Run()
# ...
```

---

## 3. MicoAir H743 Flash内存布局

### 3.1 默认内存布局

**STM32H743VI Flash规格**:
- 总容量: 2MB (0x200000)
- 起始地址: 0x08000000
- 结束地址: 0x081FFFFF
- 扇区大小: 128KB (STM32H7使用Bank/Sector结构)

**PX4默认布局**:
```
┌──────────────────────────────────────────────────────────┐
│ 0x08000000 - 0x08007FFF (32KB)   Bootloader             │
│                                   - 引导加载程序          │
│                                   - USB DFU功能          │
├──────────────────────────────────────────────────────────┤
│ 0x08008000 - 0x0801FFFF (96KB)   Reserved/Padding       │
│                                   - 保留区域              │
├──────────────────────────────────────────────────────────┤
│ 0x08020000 - 0x081FFFFF (1920KB) PX4 Firmware          │
│                                    - 固件代码             │
│                                    - 数据和常量           │
│                                    - ROMFS文件系统        │
└──────────────────────────────────────────────────────────┘
```

### 3.2 查看链接脚本确认

**Bootloader链接脚本**: `boards/micoair/h743/nuttx-config/scripts/bootloader_script.ld`

```ld
MEMORY
{
    /* Bootloader占用前32KB */
    flash (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
    sram (rwx)  : ORIGIN = 0x20010000, LENGTH = 245K
}
```

**固件链接脚本**: `boards/micoair/h743/nuttx-config/scripts/script.ld`

```ld
MEMORY
{
    /* 固件从0x08020000开始，避开Bootloader和保留区 */
    flash (rx)  : ORIGIN = 0x08020000, LENGTH = 1920K

    /* 其他内存区域 */
    itcm (rwx)  : ORIGIN = 0x00000000, LENGTH = 64K
    dtcm (rwx)  : ORIGIN = 0x20000000, LENGTH = 128K
    sram (rwx)  : ORIGIN = 0x24000000, LENGTH = 512K
    sram4(rwx)  : ORIGIN = 0x38000000, LENGTH = 64K
    bbram (rwx) : ORIGIN = 0x38800000, LENGTH = 4K
}
```

**关键点**:
- Bootloader: `0x08000000 - 0x08007FFF` (32KB)
- Reserved: `0x08008000 - 0x0801FFFF` (96KB，对齐到128KB扇区边界）
- Firmware: `0x08020000 - 0x081FFFFF` (1920KB)
- 固件**不会**覆盖Bootloader区域

**为什么有96KB保留区？**
- STM32H7的Flash扇区大小为**128KB**
- Bootloader只占用32KB，但为了方便擦除管理，整个Sector 0（128KB）都保留给Bootloader
- 这样固件从Sector 1（0x08020000）开始，擦除时不会影响Bootloader

---

## 4. 使用JLink/Ozone调试烧录

### 4.1 JLink连接配置

**硬件连接**:
```
JLink          MicoAir H743
──────────────────────────
VTref    ───>  3.3V
GND      ───>  GND
SWDIO    ───>  SWDIO
SWCLK    ───>  SWCLK
RESET    ───>  NRST (可选)
```

### 4.2 Ozone配置文件

创建Ozone项目配置文件: `micoair_h743.jdebug`

```json
{
  "Device": {
    "Name": "STM32H743VI",
    "Manufacturer": "STMicroelectronics",
    "Core": "Cortex-M7",
    "FlashSize": 2097152,
    "RAMSize": 1048576
  },
  "Interface": {
    "Type": "SWD",
    "Speed": 4000
  },
  "Memory": {
    "Flash": {
      "Base": "0x08000000",
      "Size": "0x200000"
    },
    "RAM": {
      "Base": "0x24000000",
      "Size": "0x80000"
    }
  },
  "FlashDownload": {
    "SkipErase": false,
    "SkipProgram": false,
    "SkipVerify": false,
    "EraseType": "Sector",
    "ProgramFile": "build/micoair_h743_default/micoair_h743_default.elf",
    "AddressRanges": [
      {
        "Start": "0x08020000",
        "End": "0x081FFFFF"
      }
    ]
  }
}
```

**关键配置说明**:

1. **EraseType: "Sector"** - 只擦除需要的扇区
2. **AddressRanges: Start = 0x08008000** - **不擦除Bootloader区域**
3. **ProgramFile**: 指向ELF文件（包含调试符号）

### 4.3 保护Bootloader的配置

#### 方法1: 在Ozone中配置（推荐）

**步骤**:
1. 打开Ozone
2. File → Open → 选择你的.jdebug配置文件
3. Download → Project Settings

**配置界面**:
```
┌─────────────────────────────────────────────────────────┐
│ Flash Download                                          │
├─────────────────────────────────────────────────────────┤
│ [√] Erase All Flash Before Download    ← 取消勾选！     │
│ [ ] Skip Flash Programming                              │
│ [√] Verify Download                                     │
│                                                          │
│ Flash Address Ranges:                                   │
│ ┌───────────────────────────────────────────────────┐  │
│ │ Start: 0x08020000    ← 设置为固件起始地址！！！    │  │
│ │ End:   0x081FFFFF    ← 设置为Flash结束地址         │  │
│ └───────────────────────────────────────────────────┘  │
│                                                          │
│ [ ] Reset and Halt After Download                       │
│ [√] Start Application After Download                    │
└─────────────────────────────────────────────────────────┘
```

**重要提示**:
- ❌ **不要勾选** "Erase All Flash Before Download"
- ✅ **必须设置** Start Address = 0x08020000（固件实际起始地址）
- ✅ 这样只会擦除固件区域，Bootloader和保留区保持不变

#### 方法2: 使用JLink命令脚本

创建下载脚本: `download_firmware.jlink`

```jlink
// JLink脚本 - 保护Bootloader下载固件

// 连接到目标
si SWD
speed 4000
connect

// 只擦除固件区域（从0x08020000开始）
// STM32H7的扇区大小为128KB
// Sector 0: 0x08000000 - 0x0801FFFF (Bootloader+保留区，不擦除)
// Sector 1: 0x08020000 - 0x0803FFFF (固件开始，擦除)
// ... 其他扇区

// 擦除Sector 1到最后（保留Sector 0）
erase 0x08020000 0x081FFFFF

// 下载ELF文件
loadfile build/micoair_h743_default/micoair_h743_default.elf

// 复位并运行
r
g

// 退出
exit
```

**使用脚本下载**:
```bash
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile download_firmware.jlink
```

---

### 4.4 Ozone实时调试

#### 启动Ozone调试会话

```bash
# 方法1: 直接启动
Ozone micoair_h743.jdebug

# 方法2: 命令行
ozone --projectfile=micoair_h743.jdebug --download --go
```

#### Ozone调试界面配置

**1. 项目设置**:
```
File → New Project
  Device: STM32H743VI
  Target Interface: SWD
  Speed: 4000 kHz
  Program File: build/micoair_h743_default/micoair_h743_default.elf
```

**2. 下载设置**:
```
Download → Project Settings
  Base Address: 0x08020000  ← 非常重要！必须与链接脚本一致！
  [ ] Erase All Flash
  [√] Erase Affected Ranges Only
```

**3. 调试设置**:
```
Debug → Debug Settings
  Reset Type: Normal
  Reset After Load: Yes
  Run After Attach: No (停在main函数)
```

#### 设置断点示例

```cpp
// 在gg_imu_logger模块设置断点
// 文件: src/modules/gg_imu_logger/gg_imu_logger.cpp

void GgImuLogger::Run()
{
    // 设置断点在这里 ← 在Ozone中点击行号
    if (should_exit()) {
        ScheduleClear();
        CloseLogFile();
        exit_and_cleanup();
        return;
    }

    // ... 其他代码
}
```

---

## 5. 完整开发调试流程

### 5.1 初次配置（已完成！✅）

**配置已自动完成**，您可以直接使用：

```bash
# 1. 模块配置已添加到：
#    boards/micoair/h743/default.px4board (最后一行)

# 2. JLink脚本已创建：
#    boards/micoair/h743/debug/download_firmware.jlink
#    boards/micoair/h743/debug/restore_bootloader.jlink

# 3. Ozone配置已创建：
#    boards/micoair/h743/debug/micoair_h743.jdebug

# 4. 快速参考文档：
#    boards/micoair/h743/debug/README_JLink.md
#    boards/micoair/h743/debug/使用说明.md
```

**现在可以直接编译**：
```bash
make micoair_h743_default
```

### 5.2 日常开发流程

```bash
# 1. 修改代码
vim src/modules/gg_imu_logger/gg_imu_logger.cpp

# 2. 快速编译（只编译修改的部分）
make micoair_h743_default

# 3. 使用Ozone下载并调试
# 点击 Download & Reset Program (F5)
# 或使用命令行：
ozone --projectfile=micoair_h743.jdebug --download --go

# 4. 实时调试
# - 设置断点
# - 单步执行
# - 观察变量
# - 查看调用栈
```

**时间对比**:
| 方式 | 编译 | 下载 | 总时间 |
|------|------|------|--------|
| QGC升级 | 5分钟 | 2-3分钟 | ~8分钟 |
| JLink直接下载 | 5分钟 | **10秒** | **~5分钟** |
| JLink增量编译 | **30秒** | **10秒** | **~40秒** ✅ |

---

## 6. Bootloader保护验证

### 6.1 验证Bootloader未被破坏

#### 方法1: 读取Bootloader区域

```bash
# 使用JLink Commander读取Bootloader区域
JLinkExe -device STM32H743VI -if SWD -speed 4000

J-Link> connect
J-Link> mem8 0x08000000 0x100  # 读取前256字节

# 输出应该包含有效的ARM Cortex-M向量表
# 0x08000000: Stack Pointer初始值
# 0x08000004: Reset Handler地址
```

#### 方法2: 在Ozone中查看

```
Debug → Memory
  Address: 0x08000000
  Size: 0x8000 (32KB)
```

**有效的Bootloader特征**:
```
0x08000000: 20 01 00 20  ← Stack Pointer (0x20010020)
0x08000004: 09 00 00 08  ← Reset Handler (0x08000009)
0x08000008: XX XX XX XX  ← NMI Handler
...
```

如果这些值是`0xFF FF FF FF`，说明被擦除了！

### 6.2 测试Bootloader功能

```bash
# 1. 拔掉JLink
# 2. 按住BOOT按钮（如果有）上电
# 3. 连接USB
# 4. 使用QGC检测到Bootloader

# 应该看到:
# USB设备: PX4 BL FMU v6.x
# 可以通过QGC上传固件
```

---

## 7. 故障排查

### 7.1 Bootloader被擦除了怎么办？

**症状**:
- 使用QGC无法识别飞控
- USB不再枚举为PX4设备
- 只能通过JLink下载

**恢复方法**:

#### 步骤1: 下载Bootloader固件

```bash
# Bootloader位置
ls boards/micoair/h743/extras/*bootloader*.bin

# 或从官方下载
# https://github.com/PX4/PX4-Autopilot/releases
```

#### 步骤2: 使用JLink恢复Bootloader

创建恢复脚本: `restore_bootloader.jlink`

```jlink
// 恢复Bootloader脚本

si SWD
speed 4000
connect

// 擦除Bootloader区域
erase 0x08000000 0x08007FFF

// 下载Bootloader
loadbin boards/micoair/h743/extras/micoair_h743_bootloader.bin 0x08000000

// 验证
verifybin boards/micoair/h743/extras/micoair_h743_bootloader.bin 0x08000000

// 复位
r

exit
```

```bash
# 执行恢复
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile restore_bootloader.jlink
```

#### 步骤3: 验证恢复

```bash
# 1. 断开JLink
# 2. 重新上电
# 3. 连接USB
# 4. 使用QGC应该能检测到Bootloader
```

---

### 7.2 JLink无法连接

**症状**:
```
Cannot connect to target.
```

**解决方法**:

```bash
# 1. 检查硬件连接
# 2. 检查供电（3.3V）
# 3. 尝试降低速度
JLinkExe -device STM32H743VI -if SWD -speed 1000

# 4. 使用硬件复位
# 按住NRST，点击Connect，然后释放NRST

# 5. 如果还不行，尝试Connect Under Reset
J-Link> connect
  Device: STM32H743VI
  Interface: SWD
  Speed: 4000 kHz
  [√] Connect Under Reset
```

---

### 7.3 下载后程序不运行

**症状**:
- 下载成功
- 但程序不执行或立即崩溃

**可能原因和解决方法**:

1. **起始地址错误**:
```bash
# 检查链接脚本
grep "ORIGIN" boards/micoair/h743/nuttx-config/scripts/script.ld

# 确认 flash ORIGIN = 0x08008000
```

2. **向量表未重定位**:
```cpp
// 在main函数开始检查
printf("Vector Table: 0x%08x\n", SCB->VTOR);
// 应该输出: 0x08008000
```

3. **时钟配置问题**:
```bash
# 重新编译，确保使用正确的时钟配置
make clean
make micoair_h743_default
```

---

## 8. 高级技巧

### 8.1 使用RAM调试（最快）

对于快速迭代，可以直接加载到RAM：

```jlink
// RAM调试脚本
si SWD
speed 4000
connect

// 加载到SRAM
loadfile build/micoair_h743_default/micoair_h743_default.elf,0x24000000

// 设置PC和SP
wreg R13 0x24080000  // Stack Pointer
wreg PC 0x24000001   // Program Counter (Thumb模式+1)

g
exit
```

**优点**:
- ⚡ 超快（无需擦除Flash）
- ⚡ 无限次数（不损耗Flash）

**缺点**:
- ❌ 断电丢失
- ❌ 需要修改链接脚本

---

### 8.2 条件断点

在Ozone中设置条件断点：

```cpp
// 只在特定条件下停止
void GgImuLogger::LogImuData(const vehicle_imu_s &imu_data, uint8_t instance)
{
    // 断点条件: instance == 1
    if (_log_fd < 0) {
        return;
    }

    // ... 记录数据
}
```

**Ozone设置**:
```
1. 右键断点 → Breakpoint Properties
2. Condition: instance == 1
3. 只有当instance为1时才会停止
```

---

### 8.3 实时表达式监视

在Ozone中监视变量：

```
View → Watched Data
  添加表达式:
  - _log_fd
  - _log_write_count
  - _param_imu_instance.get()
  - imu_data.timestamp
```

---

## 9. 总结

### 9.1 推荐工作流程

**开发阶段**:
```
修改代码 → 增量编译(30秒) → JLink下载(10秒) → Ozone调试
```

**测试阶段**:
```
完整编译 → 生成.px4固件 → QGC上传 → 飞行测试
```

### 9.2 关键配置检查清单

- [√] `.px4board`文件添加了模块配置
- [√] 链接脚本起始地址为`0x08020000`（**非常重要！**）
- [√] Ozone配置不擦除全部Flash
- [√] Ozone下载地址范围从`0x08020000`开始
- [√] Bootloader区域（0x08000000-0x0801FFFF，128KB）保持完整
- [√] JLink连接稳定（速度4000 kHz）

### 9.3 常用命令速查

```bash
# 开启模块
echo "CONFIG_MODULES_GG_IMU_LOGGER=y" >> boards/micoair/h743/default.px4board

# 编译
make micoair_h743_default

# JLink下载
JLinkExe -device STM32H743VI -if SWD -speed 4000 -CommandFile download.jlink

# Ozone调试
ozone --projectfile=micoair_h743.jdebug --download --go

# 验证Bootloader
JLinkExe -device STM32H743VI -if SWD -speed 4000
J-Link> mem8 0x08000000 0x100
```

---

**文档版本**: v1.0
**创建日期**: 2025-10-29
**适用平台**: MicoAir H743
**工具**: JLink/Ozone, PX4 v1.14+
**作者**: GG

