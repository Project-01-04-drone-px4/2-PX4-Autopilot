# 23-PX4 dmesg输出解析与源码追踪
## 系统启动日志完整分析

---

## 1. 概述

`dmesg`命令用于显示PX4系统启动时的控制台消息缓冲区内容。本文档逐行分析一个实际的dmesg输出，并追踪每一行输出的源码位置，帮助理解PX4系统的启动流程和各模块的初始化顺序。

### 1.1 dmesg命令实现

**命令位置**: `src/systemcmds/dmesg/dmesg.cpp`

**核心功能**:
```cpp
int dmesg_main(int argc, char *argv[])
{
    bool follow = false;
    // 解析参数...

    px4_console_buffer_print(follow);  // ← 打印控制台缓冲区
    return 0;
}
```

**关键点**:
- `px4_console_buffer_print()`: 从控制台缓冲区读取并打印消息
- 缓冲区在`BOARD_ENABLE_CONSOLE_BUFFER`启用时生效
- 所有使用`PX4_INFO`、`PX4_ERR`等宏的输出都会被捕获

### 1.2 控制台缓冲区机制

**实现位置**: `platforms/common/px4_platform_common/console_buffer.cpp`

所有启动时的日志都通过以下宏输出：
- `PX4_INFO()`: 信息日志
- `PX4_WARN()`: 警告日志
- `PX4_ERR()`: 错误日志
- `PX4_INFO_RAW()`: 原始信息（无前缀）

---

## 2. dmesg输出逐行解析

以下是实际输出的完整分析：

```
nsh> dmesg
```

### 2.1 系统版本信息 (ver all命令输出)

#### 行1-2: 硬件架构
```
HW arch: MICOAIR_H743
```

**源码位置**: `src/systemcmds/ver/ver.cpp`
**行号**: 164
**函数**: `ver_main()`

```cpp
if (show_all || !strncmp(argv[1], sz_ver_hw_str, sizeof(sz_ver_hw_str))) {
    PX4_INFO_RAW("HW arch: %s\n", px4_board_name());  // ← 这里
    ret = 0;
}
```

**数据来源**:
- `px4_board_name()`: 定义在`version/version.h`
- 实际值来自`boards/micoair/h743/src/board_config.h`中的`BOARD_NAME`宏

**作用**: 标识硬件板型号

---

#### 行3-5: PX4固件信息
```
PX4 git-hash: 3816636561ddd734fbc26cce6efb3049ca99a653
PX4 version: 1.14.0 80 (17694848)
PX4 git-branch: gf-file
```

**源码位置**: `src/systemcmds/ver/ver.cpp`
**行号**: 206-225

```cpp
if (show_all || !strncmp(argv[1], sz_ver_git_str, sizeof(sz_ver_git_str))) {
    PX4_INFO_RAW("PX4 git-hash: %s\n", px4_firmware_version_string());  // ← 行3

    unsigned fwver = px4_firmware_version();
    unsigned major = (fwver >> (8 * 3)) & 0xFF;
    unsigned minor = (fwver >> (8 * 2)) & 0xFF;
    unsigned patch = (fwver >> (8 * 1)) & 0xFF;
    unsigned type = (fwver >> (8 * 0)) & 0xFF;

    PX4_INFO_RAW("PX4 version: %u.%u.%u %x (%u)\n", major, minor, patch, type, fwver);  // ← 行4

    const char *git_branch = px4_firmware_git_branch();
    if (git_branch && git_branch[0]) {
        PX4_INFO_RAW("PX4 git-branch: %s\n", git_branch);  // ← 行5
    }
}
```

**数据来源**:
- `px4_firmware_version_string()`: 来自`build/micoair_h743_default/src/lib/version/git_version.h`（编译时生成）
- `px4_firmware_version()`: 来自`CMakeLists.txt`中的版本定义
- `px4_firmware_git_branch()`: 来自git分支信息

**版本号解析**:
```
1.14.0 80 (17694848)
│  │  │  │   └─────── 原始版本号（十进制）
│  │  │  └─────────── 版本类型（80 = 0x50）
│  │  └────────────── patch版本
│  └───────────────── minor版本
└──────────────────── major版本
```

---

#### 行6-8: 操作系统信息
```
OS: NuttX
OS version: Release 11.0.0 (184549631)
OS git-hash: d50ffd354495ce5d30814dba41d2eb70a32b8fa3
```

**源码位置**: `src/systemcmds/ver/ver.cpp`
**行号**: 239-257

```cpp
fwver = px4_os_version();
major = (fwver >> (8 * 3)) & 0xFF;
minor = (fwver >> (8 * 2)) & 0xFF;
patch = (fwver >> (8 * 1)) & 0xFF;
type = (fwver >> (8 * 0)) & 0xFF;
PX4_INFO_RAW("OS: %s\n", px4_os_name());  // ← 行6

if (type == 255) {
    PX4_INFO_RAW("OS version: Release %u.%u.%u (%u)\n", major, minor, patch, fwver);  // ← 行7
}

const char *os_git_hash = px4_os_version_string();
if (os_git_hash) {
    PX4_INFO_RAW("OS git-hash: %s\n", os_git_hash);  // ← 行8
}
```

**数据来源**:
- 来自NuttX RTOS的版本信息
- 定义在`platforms/nuttx/NuttX/nuttx/`子模块中

---

#### 行9-10: 构建信息
```
Build datetime: Oct 28 2025 09:55:21
Build uri: localhost
```

**源码位置**: `src/systemcmds/ver/ver.cpp`
**行号**: 264-272

```cpp
if (show_all || !strncmp(argv[1], sz_ver_bdate_str, sizeof(sz_ver_bdate_str))) {
    PX4_INFO_RAW("Build datetime: %s %s\n", __DATE__, __TIME__);  // ← 行9
    ret = 0;
}

if (show_all || !strncmp(argv[1], sz_ver_buri_str, sizeof(sz_ver_buri_str))) {
    PX4_INFO_RAW("Build uri: %s\n", px4_build_uri());  // ← 行10
    ret = 0;
}
```

**数据来源**:
- `__DATE__`和`__TIME__`: GCC内置宏，编译时间戳
- `px4_build_uri()`: 构建主机名，来自环境变量或默认"localhost"

---

#### 行11: 构建变体
```
Build variant: default
```

**源码位置**: `src/systemcmds/ver/ver.cpp`
**行号**: 276

```cpp
if (show_all) {
    PX4_INFO_RAW("Build variant: %s\n", px4_board_target_label());  // ← 行11
}
```

**数据来源**:
- `px4_board_target_label()`: 来自板级配置，通常是"default"或其他变体名

---

#### 行12: 编译工具链
```
Toolchain: GNU GCC, 9.3.1 20200408 (release)
```

**源码位置**: `src/systemcmds/ver/ver.cpp`
**行号**: 280

```cpp
if (show_all || !strncmp(argv[1], sz_ver_gcc_str, sizeof(sz_ver_gcc_str))) {
    PX4_INFO_RAW("Toolchain: %s, %s\n", px4_toolchain_name(), px4_toolchain_version());  // ← 行12
    ret = 0;
}
```

**数据来源**:
- `px4_toolchain_name()`: "GNU GCC"
- `px4_toolchain_version()`: 来自`__VERSION__`宏

---

#### 行13: PX4 GUID
```
PX4GUID: 000600000000373635363233511300310048
```

**源码位置**: `src/systemcmds/ver/ver.cpp`
**行号**: 286-290

```cpp
if (show_all || !strncmp(argv[1], px4_guid_str, sizeof(px4_guid_str))) {
    char px4guid_fmt_buffer[PX4_GUID_FORMAT_SIZE];

    board_get_px4_guid_formated(px4guid_fmt_buffer, sizeof(px4guid_fmt_buffer));
    PX4_INFO_RAW("PX4GUID: %s\n", px4guid_fmt_buffer);  // ← 行13
    ret = 0;
}
```

**数据来源**:
- `board_get_px4_guid_formated()`: 从STM32的96位唯一ID生成
- 实现位置: `platforms/nuttx/src/px4/stm/stm32_common/board_hw_info/`

**GUID格式**:
```
0006 0000 0000 3736 3536 3233 5113 0031 0048
│    │    │    └──────┴──────┴─────┴────┴───── STM32 UID[95:0]
│    │    └────────────────────────────────── 保留字节
│    └─────────────────────────────────────── USB VID (0x0000)
└──────────────────────────────────────────── USB PID (0x0006)
```

---

#### 行14: MCU信息
```
MCU: STM32H7[4|5]xxx, rev. V
```

**源码位置**: `src/systemcmds/ver/ver.cpp`
**行号**: 295-315

```cpp
if (show_all || !strncmp(argv[1], mcu_ver_str, sizeof(mcu_ver_str))) {
    char rev = ' ';
    const char *revstr = nullptr;
    const char *errata = nullptr;

    int chip_version = board_mcu_version(&rev, &revstr, &errata);

    if (chip_version < 0) {
        PX4_INFO_RAW("UNKNOWN MCU\n");
    } else {
        PX4_INFO_RAW("MCU: %s, rev. %c\n", revstr, rev);  // ← 行14

        if (errata != nullptr) {
            printf("\nWARNING   WARNING   WARNING!\n"
                   "Revision %c has a silicon errata:\n"
                   "%s", rev, errata);
        }
    }
    ret = 0;
}
```

**数据来源**:
- `board_mcu_version()`: 读取STM32的DBGMCU_IDCODE寄存器
- 实现位置: `platforms/nuttx/src/px4/stm/stm32h7/board_hw_info/board_mcu_version.c`

**MCU版本识别**:
```
DBGMCU_IDCODE[15:0] = DEV_ID
- 0x450: STM32H742, STM32H743/753, STM32H750
- 0x480: STM32H7A3/7B3, STM32H7B0

DBGMCU_IDCODE[31:16] = REV_ID
- 0x1000: Rev. A
- 0x2000: Rev. B
- 0x2001: Rev. Z (V)
```

---

### 2.2 启动脚本输出（rcS执行）

启动脚本`/etc/init.d/rcS`被执行，产生以下输出：

#### 行15-17: MTD校准数据加载失败
```
ERROR [bsondump] open '/fs/mtd_caldata' failed (2)
New /fs/mtd_caldata size is:
ERROR [bsondump] open '/fs/mtd_caldata' failed (2)
```

**源码位置**: `ROMFS/px4fmu_common/init.d/rcS`
**行号**: 132-137

```bash
# Check if /fs/mtd_params is a valid BSON file
if ! bsondump docsize /fs/mtd_caldata
then
    echo "New /fs/mtd_caldata size is:"
    bsondump docsize /fs/mtd_caldata
fi
```

**原因分析**:
- MicoAir H743没有配置MTD_CALDATA分区
- `bsondump`命令尝试打开`/fs/mtd_caldata`失败（errno=2: No such file or directory）
- 这是正常的，因为该板没有出厂校准数据存储

**bsondump命令**: `src/systemcmds/bsondump/bsondump.cpp`

---

#### 行18-20: 参数系统初始化
```
INFO  [param] selected parameter default file /fs/microsd/params
INFO  [param] importing from '/fs/microsd/params'
INFO  [parameters] BSON document size 996 bytes, decoded 996 bytes (INT32:22, FLOAT:24)
```

**源码位置**:

**行18**: `ROMFS/px4fmu_common/init.d/rcS` 行148
```bash
param select $PARAM_FILE  # $PARAM_FILE = /fs/microsd/params
```

实际代码: `src/lib/parameters/parameters.cpp`
```cpp
int param_select(const char *filename)
{
    PX4_INFO("selected parameter default file %s", filename);  // ← 行18
    // ...
}
```

**行19-20**: `ROMFS/px4fmu_common/init.d/rcS` 行149
```bash
if ! param import
then
    echo "ERROR [init] param import failed"
    # ...
fi
```

实际代码: `src/lib/parameters/parameters.cpp`
```cpp
int param_import(const char *filename)
{
    PX4_INFO("importing from '%s'", filename);  // ← 行19

    // 读取BSON文件...

    PX4_INFO("BSON document size %d bytes, decoded %d bytes (INT32:%d, FLOAT:%d)",
             bson_size, decoded_size, num_int32, num_float);  // ← 行20
}
```

**参数统计**:
- BSON文件大小: 996字节
- 解码大小: 996字节
- INT32参数: 22个
- FLOAT参数: 24个

---

#### 行21: 参数备份文件选择
```
INFO  [param] selected parameter backup file /fs/microsd/parameters_backup.bson
```

**源码位置**: `ROMFS/px4fmu_common/init.d/rcS`
**行号**: 182-184

```bash
if [ $STORAGE_AVAILABLE = yes ]
then
    param select-backup $PARAM_BACKUP_FILE
fi
```

实际代码: `src/lib/parameters/parameters.cpp`
```cpp
int param_select_backup(const char *filename)
{
    PX4_INFO("selected parameter backup file %s", filename);  // ← 行21
    // ...
}
```

---

#### 行22-24: 板级配置脚本加载
```
Board architecture defaults: /etc/init.d/rc.board_arch_defaults
Board defaults: /etc/init.d/rc.board_defaults
Loading airframe: /etc/init.d/airframes/4001_quad_x
```

**源码位置**: `ROMFS/px4fmu_common/init.d/rcS`

**行22**: 行201-206
```bash
set BOARD_ARCH_RC_DEFAULTS ${R}etc/init.d/rc.board_arch_defaults
if [ -f $BOARD_ARCH_RC_DEFAULTS ]
then
    echo "Board architecture defaults: ${BOARD_ARCH_RC_DEFAULTS}"  // ← 行22
    . $BOARD_ARCH_RC_DEFAULTS
fi
```

**行23**: 行212-217
```bash
set BOARD_RC_DEFAULTS ${R}etc/init.d/rc.board_defaults
if [ -f $BOARD_RC_DEFAULTS ]
then
    echo "Board defaults: ${BOARD_RC_DEFAULTS}"  // ← 行23
    . $BOARD_RC_DEFAULTS
fi
```

**行24**: 来自`ROMFS/px4fmu_common/init.d/rc.autostart`
```bash
# rc.autostart根据SYS_AUTOSTART参数加载对应机架配置
# SYS_AUTOSTART=4001 → airframes/4001_quad_x
echo "Loading airframe: /etc/init.d/airframes/4001_quad_x"  // ← 行24
. /etc/init.d/airframes/4001_quad_x
```

---

#### 行25: Dataman启动
```
INFO  [dataman] data manager file '/fs/microsd/dataman' size is 68528 bytes
```

**源码位置**: `ROMFS/px4fmu_common/init.d/rcS`
**行号**: 274-283

```bash
if param compare -s SYS_DM_BACKEND 1
then
    dataman start -r
else
    if param compare SYS_DM_BACKEND 0
    then
        dataman start
    fi
fi
```

实际代码: `src/modules/dataman/dataman.cpp`
```cpp
int Dataman::start()
{
    // ...
    PX4_INFO("data manager file '%s' size is %d bytes",
             _data_file, file_size);  // ← 行25
}
```

**Dataman作用**: 存储任务航点、地理围栏等数据

---

### 2.3 传感器初始化

#### 行26-27: 板级传感器配置
```
Board sensors: /etc/init.d/rc.board_sensors
```

**源码位置**: `ROMFS/px4fmu_common/init.d/rcS`
**行号**: 345-350

```bash
set BOARD_RC_SENSORS ${R}etc/init.d/rc.board_sensors
if [ -f $BOARD_RC_SENSORS ]
then
    echo "Board sensors: ${BOARD_RC_SENSORS}"  // ← 行26
    . $BOARD_RC_SENSORS
fi
```

**rc.board_sensors内容**: `boards/micoair/h743/init/rc.board_sensors`
```bash
#!/bin/sh
# Board sensors configuration

# BMI088 (SPI2)
bmi088_accel start -s -R 6
bmi088_gyro start -s -R 6

# BMI270 (SPI2)
bmi270 start -s

# DPS310 (I2C2)
dps310 start -I

# IST8310 (I2C2)
ist8310 start -I -R 2
```

---

#### 行27-31: 具体传感器启动
```
bmi088_accel #0 on SPI bus 2 rotation 6
bmi088_gyro #0 on SPI bus 2 rotation 6
bmi270 #0 on SPI bus 2
dps310 #0 on I2C bus 2 address 0x76
ist8310 #0 on I2C bus 2 address 0xE rotation 2
```

这些是各个传感器驱动的启动消息。

**源码位置示例**: `src/drivers/imu/bosch/bmi088/BMI088_Accelerometer.cpp`

```cpp
void BMI088_Accelerometer::Run()
{
    if (_drdy_gpio != 0) {
        // GPIO驱动
    }

    // 读取数据...

    // 首次运行时打印
    if (_first_run) {
        PX4_INFO("bmi088_accel #%d on SPI bus %d rotation %d",
                 instance(), bus_id(), rotation());  // ← 行27
        _first_run = false;
    }
}
```

**传感器配置说明**:
- `#0`: 实例编号
- `SPI bus 2`: SPI2总线
- `I2C bus 2`: I2C2总线
- `rotation 6`: 旋转矩阵（6 = ROTATION_YAW_180）
- `address 0x76`: I2C设备地址

---

### 2.4 核心模块启动

#### 行32: EKF2启动
```
ekf2 [606:237]
```

**源码位置**: `ROMFS/px4fmu_common/init.d/rcS`
**行号**: 371-374

```bash
if param compare -s EKF2_EN 1
then
    ekf2 start &
fi
```

实际代码: `src/modules/ekf2/EKF2.cpp`
```cpp
int EKF2::task_spawn(int argc, char *argv[])
{
    // ...
    EKF2 *instance = new EKF2(...);

    // 启动成功后打印任务信息
    // [606:237] = [PID:优先级]
}
```

**输出格式**: `ekf2 [PID:Priority]`
- PID 606: 进程ID
- Priority 237: 任务优先级

---

#### 行33: DShot DMA分配
```
INFO  [arch_dshot] Allocated DMA UP Timer Index 0
```

**源码位置**: `platforms/nuttx/src/px4/stm/stm32h7/dshot/dshot.c`

```cpp
int up_dshot_init(uint32_t num_channels, const uint32_t *output_mask, bool no_dma_mask)
{
    // DMA通道分配...

    PX4_INFO("Allocated DMA UP Timer Index %d", timer_index);  // ← 行33
}
```

**作用**: 为DShot电调协议分配DMA资源

---

#### 行34-36: GPS和MSP OSD启动
```
Starting Main GPS on /dev/ttyS2
Starting MSP OSD on /dev/ttyS1
INFO  [msp_osd] MSP OSD running on /dev/ttyS1
```

**源码位置**: 来自`ROMFS/px4fmu_common/init.d/rc.serial`（自动生成）

```bash
# 由Tools/serial/generate_config.py生成
echo "Starting Main GPS on /dev/ttyS2"
gps start -d /dev/ttyS2

echo "Starting MSP OSD on /dev/ttyS1"
msp_osd start -d /dev/ttyS1
```

实际代码: `src/drivers/osd/msp_osd/msp_osd.cpp`
```cpp
int MspOsd::task_spawn(int argc, char *argv[])
{
    // ...
    PX4_INFO("MSP OSD running on %s", device_path);  // ← 行36
}
```

---

#### 行37-39: RC输入和MAVLink
```
Starting RC Input Driver on /dev/ttyS4
Starting MAVLink on /dev/ttyS0
INFO  [mavlink] mode: Normal, data rate: 1200 B/s on /dev/ttyS0 @ 57600B
```

**源码位置**: `ROMFS/px4fmu_common/init.d/rc.serial`

```bash
echo "Starting RC Input Driver on /dev/ttyS4"
rc_input start

echo "Starting MAVLink on /dev/ttyS0"
mavlink start -d /dev/ttyS0 -b 57600 -m normal -r 1200
```

实际代码: `src/modules/mavlink/mavlink_main.cpp`
```cpp
int Mavlink::task_spawn(int argc, char *argv[])
{
    // ...
    PX4_INFO("mode: %s, data rate: %d B/s on %s @ %dB",
             mode_str, data_rate, device, baudrate);  // ← 行39
}
```

**MAVLink配置**:
- mode: Normal（标准模式）
- data rate: 1200字节/秒
- device: /dev/ttyS0
- baudrate: 57600波特率

---

#### 行40-41: CDC/ACM自动启动
```
INFO  [cdcacm_autostart] Starting CDC/ACM autostart
```

**源码位置**: `ROMFS/px4fmu_common/init.d/rcS`
**行号**: 514-522

```bash
if param greater -s SYS_USB_AUTO -1
then
    if ! cdcacm_autostart start
    then
        sercon
        echo "Starting MAVLink on /dev/ttyACM0"
        mavlink start -d /dev/ttyACM0
    fi
fi
```

实际代码: `src/drivers/cdcacm_autostart/cdcacm_autostart.cpp`
```cpp
int CdcAcmAutostart::task_spawn(int argc, char *argv[])
{
    PX4_INFO("Starting CDC/ACM autostart");  // ← 行40
    // ...
}
```

---

#### 行42-43: Board extras和Logger
```
Board extras: /etc/init.d/rc.board_extras
INFO  [logger] logger started (mode=all)
```

**源码位置**:

**行42**: `ROMFS/px4fmu_common/init.d/rcS` 行588-593
```bash
set BOARD_RC_EXTRAS ${R}etc/init.d/rc.board_extras
if [ -f $BOARD_RC_EXTRAS ]
then
    echo "Board extras: ${BOARD_RC_EXTRAS}"  // ← 行42
    . $BOARD_RC_EXTRAS
fi
```

**行43**: `ROMFS/px4fmu_common/init.d/rc.logging`
```bash
logger start
```

实际代码: `src/modules/logger/logger.cpp` 行574
```cpp
void Logger::run()
{
    PX4_INFO("logger started (mode=%s)", configured_backend_mode());  // ← 行43
}
```

---

#### 行44: UAVCAN启动
```
INFO  [uavcan] Node ID 1, bitrate 1000000
```

**源码位置**: `ROMFS/px4fmu_common/init.d/rcS`
**行号**: 634-646

```bash
if param greater -s UAVCAN_ENABLE 0
then
    if ! uavcan start
    then
        tune_control play error
    fi
fi
```

实际代码: `src/drivers/uavcan/uavcan_main.cpp`
```cpp
int UavcanNode::start(uavcan::NodeID node_id, uint32_t bitrate)
{
    // ...
    PX4_INFO("Node ID %d, bitrate %d", node_id, bitrate);  // ← 行44
}
```

---

### 2.5 NSH Shell启动

#### 行45-46: NSH提示符
```
NuttShell (NSH) NuttX-11.0.0
nsh>
```

**源码位置**: `platforms/nuttx/NuttX/apps/nshlib/nsh_console.c`

```c
int nsh_consolemain(int argc, char *argv[])
{
    // ...
    printf("\nNuttShell (NSH) NuttX-%s\n", CONFIG_VERSION_STRING);  // ← 行45
    printf("nsh> ");  // ← 行46 (提示符)
}
```

---

### 2.6 USB连接后的输出

#### 行47-48: USB串口注册
```
sercon: Registering CDC/ACM serial driver
sercon: Successfully registered the CDC/ACM serial driver
```

**源码位置**: `src/drivers/cdcacm_autostart/cdcacm_autostart.cpp`

当USB插入后，CDC/ACM设备被检测到，系统注册`/dev/ttyACM0`设备。

---

#### 行49-50: USB上的MAVLink启动
```
INFO  [cdcacm_autostart] Starting mavlink on /dev/ttyACM0 (SYS_USB_AUTO=2)
INFO  [mavlink] mode: Onboard, data rate: 100000 B/s on /dev/ttyACM0 @ 2000000B
```

**源码位置**: `src/drivers/cdcacm_autostart/cdcacm_autostart.cpp`

```cpp
void CdcAcmAutostart::Run()
{
    if (access("/dev/ttyACM0", F_OK) == 0) {
        PX4_INFO("Starting mavlink on /dev/ttyACM0 (SYS_USB_AUTO=%d)",
                 param_sys_usb_auto);  // ← 行49

        // 启动MAVLink...
    }
}
```

**USB MAVLink配置**:
- mode: Onboard（板载模式，用于QGC连接）
- data rate: 100000字节/秒（100KB/s）
- baudrate: 2000000（2Mbps，USB全速）

---

### 2.7 健康检查和警告

#### 行51-53: 姿态失败警告
```
WARN  [health_and_arming_checks] Preflight Fail: Attitude failure (roll)
WARN  [health_and_arming_checks] Preflight Fail: Attitude failure (roll)
WARN  [health_and_arming_checks] Preflight Fail: Attitude failure (roll)
```

**源码位置**: `src/modules/commander/HealthAndArmingChecks/checks/attitudeCheck.cpp`

```cpp
void AttitudeChecks::checkAndReport(const Context &context, Report &reporter)
{
    if (roll_error > max_roll_error) {
        reporter.healthFailure(NavModes::All, health_component_t::system,
                               events::ID("check_attitude_roll"),
                               events::Log::Warning,
                               "Attitude failure (roll)");  // ← 行51-53
    }
}
```

**原因**:
- 飞控静止放置，姿态估计可能未完全收敛
- 或者飞控放置角度超出允许范围
- 这是正常的启动时警告，姿态稳定后会消失

---

#### 行54-57: MAVLink消息丢失错误
```
ERROR [mavlink] vehicle_command_ack lost, generation 11 -> 14
ERROR [mavlink] vehicle_command_ack lost, generation 17 -> 20
ERROR [mavlink] vehicle_command_ack lost, generation 7 -> 25
ERROR [mavlink] vehicle_command_ack lost, generation 24 -> 27
```

**源码位置**: `src/modules/mavlink/mavlink_messages.cpp`

```cpp
void StreamVehicleCommandAck::send()
{
    if (_vehicle_command_ack_sub.updated()) {
        vehicle_command_ack_s ack;

        const unsigned last_generation = _vehicle_command_ack_sub.get_last_generation();

        if (_vehicle_command_ack_sub.copy(&ack)) {
            const unsigned current_generation = _vehicle_command_ack_sub.get_last_generation();

            if (current_generation != last_generation + 1) {
                PX4_ERR("vehicle_command_ack lost, generation %d -> %d",
                        last_generation, current_generation);  // ← 行54-57
            }
        }
    }
}
```

**原因**:
- QGC连接时发送了大量命令
- MAVLink缓冲区来不及处理所有消息
- 部分command_ack消息被丢弃
- 这在连接初期是正常的，不影响功能

---

#### 行58: 再次出现姿态警告
```
WARN  [health_and_arming_checks] Preflight Fail: Attitude failure (roll)
```

同行51-53的重复警告。

---

#### 行59: MAVLink Shell启动
```
INFO  [mavlink] Starting mavlink shell
```

**源码位置**: `src/modules/mavlink/mavlink_main.cpp`

```cpp
void Mavlink::start_mavlink_shell()
{
    PX4_INFO("Starting mavlink shell");  // ← 行59
    // 创建MAVLink shell实例
    // 允许通过QGC的MAVLink Console访问NSH
}
```

**作用**: 启动MAVLink shell，允许通过QGC控制台执行NSH命令。

---

## 3. 启动顺序总结

### 3.1 启动流程图

```
┌────────────────────────────────────────────────────────────────┐
│  1. Bootloader                                                 │
│     - 硬件初始化                                               │
│     - 加载固件到RAM                                            │
└────────────────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────────────────┐
│  2. NuttX内核启动                                              │
│     - 硬件时钟配置                                             │
│     - 内存初始化                                               │
│     - 中断向量表配置                                           │
└────────────────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────────────────┐
│  3. PX4主任务启动                                              │
│     - board_app_initialize()                                   │
│     - 初始化work queue系统                                     │
└────────────────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────────────────┐
│  4. NSH Shell启动                                              │
│     - 打印版本信息 (ver all)                                   │
│     - 执行启动脚本                                             │
└────────────────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────────────────┐
│  5. rcS启动脚本执行                                            │
│     - 挂载SD卡                                                 │
│     - 加载参数                                                 │
│     - 执行板级配置                                             │
│     - 加载机架配置                                             │
└────────────────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────────────────┐
│  6. 传感器驱动启动                                             │
│     - BMI088 (加速度+陀螺仪)                                   │
│     - BMI270 (IMU)                                             │
│     - DPS310 (气压计)                                          │
│     - IST8310 (磁力计)                                         │
└────────────────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────────────────┐
│  7. 核心模块启动                                               │
│     - EKF2 (状态估计)                                          │
│     - Commander (系统管理)                                     │
│     - Navigator (导航)                                         │
└────────────────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────────────────┐
│  8. 通信和IO启动                                               │
│     - MAVLink (遥测链路)                                       │
│     - GPS                                                      │
│     - RC Input                                                 │
│     - DShot/PWM Output                                         │
└────────────────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────────────────┐
│  9. 可选模块启动                                               │
│     - Logger                                                   │
│     - UAVCAN                                                   │
│     - MSP OSD                                                  │
└────────────────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────────────────┐
│  10. USB连接检测                                               │
│     - CDC/ACM设备注册                                          │
│     - USB MAVLink启动                                          │
│     - MAVLink Shell启动                                        │
└────────────────────────────────────────────────────────────────┘
                          ↓
┌────────────────────────────────────────────────────────────────┐
│  11. 系统就绪                                                  │
│     - 健康检查                                                 │
│     - 等待解锁命令                                             │
└────────────────────────────────────────────────────────────────┘
```

### 3.2 时间轴估计

假设从上电到系统完全启动的时间约3-5秒：

| 时间(ms) | 阶段 | 说明 |
|---------|------|------|
| 0 | 上电 | 硬件复位 |
| 0-100 | Bootloader | 硬件检测、固件加载 |
| 100-500 | NuttX启动 | 内核初始化 |
| 500-1000 | rcS执行开始 | 挂载SD卡、加载参数 |
| 1000-1500 | 传感器初始化 | IMU、气压计、磁力计 |
| 1500-2500 | 核心模块启动 | EKF2、Commander |
| 2500-3000 | 通信启动 | MAVLink、GPS |
| 3000-3500 | Logger启动 | 开始记录日志 |
| 3500+ | USB检测 | QGC连接（如果插入USB） |

---

## 4. 关键源码位置总结

### 4.1 版本信息打印

| 功能 | 源文件 | 函数/行号 |
|------|--------|----------|
| **ver命令实现** | `src/systemcmds/ver/ver.cpp` | `ver_main()` |
| HW arch | 同上 | 行164 |
| PX4 git-hash | 同上 | 行206 |
| PX4 version | 同上 | 行214-218 |
| OS信息 | 同上 | 行244-257 |
| Build时间 | 同上 | 行264 |
| Toolchain | 同上 | 行280 |
| PX4GUID | 同上 | 行286-290 |
| MCU版本 | 同上 | 行295-315 |

### 4.2 启动脚本

| 功能 | 源文件 | 行号 |
|------|--------|------|
| **主启动脚本** | `ROMFS/px4fmu_common/init.d/rcS` | 全文 |
| 参数加载 | 同上 | 148-179 |
| 板级配置 | 同上 | 199-229 |
| 传感器启动 | 同上 | 343-366 |
| EKF2启动 | 同上 | 371-384 |
| MAVLink启动 | 同上 | 508-522 |
| Logger启动 | `ROMFS/px4fmu_common/init.d/rc.logging` | - |

### 4.3 模块初始化

| 模块 | 源文件 | 打印位置 |
|------|--------|---------|
| **参数系统** | `src/lib/parameters/parameters.cpp` | `param_import()` |
| **Dataman** | `src/modules/dataman/dataman.cpp` | `start()` |
| **Logger** | `src/modules/logger/logger.cpp` | `run()` 行574 |
| **MAVLink** | `src/modules/mavlink/mavlink_main.cpp` | `task_spawn()` |
| **EKF2** | `src/modules/ekf2/EKF2.cpp` | `task_spawn()` |
| **传感器** | `src/drivers/imu/bosch/bmi088/` | 各驱动的`Run()` |

### 4.4 健康检查

| 功能 | 源文件 | 说明 |
|------|--------|------|
| **姿态检查** | `src/modules/commander/HealthAndArmingChecks/checks/attitudeCheck.cpp` | 检查姿态误差 |
| **解锁检查** | `src/modules/commander/HealthAndArmingChecks/` | 各种检查器 |

---

## 5. dmesg使用技巧

### 5.1 实时监控启动过程

```bash
# 终端1: 持续显示新消息
nsh> dmesg -f

# 终端2: 重启系统
nsh> reboot
```

### 5.2 保存启动日志

```bash
# 将dmesg输出保存到文件
nsh> dmesg > /fs/microsd/bootlog.txt

# 查看
nsh> cat /fs/microsd/bootlog.txt
```

### 5.3 过滤特定模块

```bash
# 只看ERROR
nsh> dmesg | grep ERROR

# 只看特定模块
nsh> dmesg | grep mavlink
nsh> dmesg | grep ekf2
```

### 5.4 在PC上分析

```bash
# 通过MAVLink shell获取dmesg
# 在QGC的MAVLink Console中:
dmesg

# 或通过串口
screen /dev/ttyACM0 57600
nsh> dmesg
```

---

## 6. 常见启动问题分析

### 6.1 参数加载失败

**症状**:
```
ERROR [param] param import failed
```

**原因**:
- SD卡未插入或损坏
- `/fs/microsd/params`文件损坏
- 文件系统损坏

**解决方法**:
```bash
# 检查SD卡
nsh> df
nsh> ls /fs/microsd/

# 重置参数
nsh> param reset_all
nsh> param save
```

### 6.2 传感器启动失败

**症状**:
```
ERROR [bmi088_accel] probe failed
```

**原因**:
- 硬件连接问题
- SPI/I2C总线故障
- 传感器损坏

**解决方法**:
```bash
# 检查传感器列表
nsh> sensors status

# 手动启动传感器查看详细错误
nsh> bmi088_accel start -s -R 6
```

### 6.3 EKF2启动失败

**症状**:
```
ERROR [ekf2] sensor data missing
```

**原因**:
- IMU数据未准备好
- 传感器初始化失败

**解决方法**:
```bash
# 检查IMU数据
nsh> listener vehicle_imu

# 查看EKF2状态
nsh> ekf2 status
```

### 6.4 MAVLink连接问题

**症状**:
```
ERROR [mavlink] Failed to open /dev/ttyS0
```

**原因**:
- 串口设备不存在
- 串口被其他程序占用
- 板级配置错误

**解决方法**:
```bash
# 检查串口设备
nsh> ls /dev/tty*

# 查看MAVLink状态
nsh> mavlink status
```

---

## 7. 调试技巧

### 7.1 添加自定义打印

在rcS脚本中添加调试信息：

```bash
# 在rcS的任何位置添加
echo "DEBUG: About to start sensors"
. ${R}etc/init.d/rc.sensors
echo "DEBUG: Sensors started"
```

### 7.2 使用PX4_INFO宏

在C++代码中添加调试打印：

```cpp
#include <px4_platform_common/log.h>

void my_function()
{
    PX4_INFO("Debug: entering my_function");
    // ...
    PX4_INFO("Debug: variable value = %d", value);
}
```

### 7.3 性能分析

```bash
# 查看任务CPU使用率
nsh> top

# 查看work queue状态
nsh> work_queue status

# 查看性能计数器
nsh> perf
```

---

## 8. 总结

### 8.1 dmesg的作用

1. **系统诊断**: 快速了解系统启动状态
2. **错误排查**: 定位启动失败的原因
3. **性能分析**: 了解启动时间和模块加载顺序
4. **开发调试**: 验证代码修改效果

### 8.2 关键输出理解

- **版本信息**: 确认固件版本和构建配置
- **参数加载**: 确认配置是否正确
- **传感器初始化**: 确认硬件连接状态
- **模块启动**: 确认系统功能完整性
- **健康检查**: 确认系统是否准备好飞行

### 8.3 最佳实践

1. ✅ 每次修改后检查dmesg输出
2. ✅ 保存启动日志用于对比
3. ✅ 关注ERROR和WARN消息
4. ✅ 了解正常的启动序列
5. ✅ 使用dmesg -f实时监控

---

**文档版本**: v1.0
**创建日期**: 2025-10-29
**适用PX4版本**: v1.14+
**作者**: GG
**测试平台**: MicoAir H743

