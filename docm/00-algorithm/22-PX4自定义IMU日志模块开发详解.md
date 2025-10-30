# 22-PX4自定义IMU日志模块开发详解
## 从零开发vehicle_imu日志记录模块

---

## 1. 概述

本文档详细记录了如何在PX4中从零创建一个自定义模块`gg_imu_logger`，该模块实现以下功能：
- ✅ 订阅`vehicle_imu` uORB主题（支持单例或多实例）
- ✅ 将IMU数据记录到独立的ulog文件
- ✅ 通过参数控制模块的启动和配置
- ✅ 运行在独立的work queue中
- ✅ 支持通过QGC远程下载日志文件

### 1.1 需求分析

**核心需求**：
1. 获取vehicle_imu单例数据（实例1或实例2或全部）
2. 记录原始IMU数据到独立日志文件
3. 参数`GG_START`控制是否自动启动
4. 可配置记录频率和实例选择
5. 无需拔卡即可通过QGC下载日志

**技术要点**：
- uORB多实例订阅
- Work Queue调度
- ulog文件格式
- 参数系统集成
- 启动脚本配置

---

## 2. 模块创建步骤

### 2.1 创建模块目录结构

```bash
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot
mkdir -p src/modules/gg_imu_logger
```

**目录结构**：
```
src/modules/gg_imu_logger/
├── gg_imu_logger.hpp       # 模块头文件
├── gg_imu_logger.cpp       # 模块实现
├── CMakeLists.txt          # 构建配置
├── Kconfig                 # Kconfig配置
└── module.yaml             # 参数定义
```

### 2.2 模块头文件 (gg_imu_logger.hpp)

**关键要点**：
1. 继承自`ModuleBase`、`ModuleParams`和`ScheduledWorkItem`
2. 定义参数宏`DEFINE_PARAMETERS`
3. 使用`uORB::SubscriptionMultiArray`订阅多实例

```cpp
class GgImuLogger : public ModuleBase<GgImuLogger>,
                     public ModuleParams,
                     public px4::ScheduledWorkItem
{
public:
    GgImuLogger();
    ~GgImuLogger() override;

    // ModuleBase接口
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    int print_status() override;

    bool init();

private:
    void Run() override;  // Work Queue调度函数

    // 参数定义
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::GG_IMU_INSTANCE>) _param_imu_instance,
        (ParamInt<px4::params::GG_LOG_RATE>) _param_log_rate
    )

    // uORB订阅（支持最多3个实例）
    uORB::SubscriptionMultiArray<vehicle_imu_s, 3> _vehicle_imu_subs{ORB_ID::vehicle_imu};

    // 日志文件
    int _log_fd{-1};
    char _log_filename[64]{};
};
```

**设计说明**：
- `ScheduledWorkItem`: 周期性任务调度
- `SubscriptionMultiArray<vehicle_imu_s, 3>`: 支持订阅最多3个IMU实例
- `_log_fd`: 日志文件描述符

### 2.3 模块实现 (gg_imu_logger.cpp)

#### 2.3.1 构造函数和初始化

```cpp
GgImuLogger::GgImuLogger() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)  // 使用lp_default work queue
{
}

bool GgImuLogger::init()
{
    ParametersUpdate(true);

    // 打开日志文件
    if (!OpenLogFile()) {
        PX4_ERR("Failed to open log file");
        return false;
    }

    // 根据配置的频率调度运行
    int rate_hz = _param_log_rate.get();
    if (rate_hz <= 0) {
        rate_hz = 100; // 默认100Hz
    }

    uint32_t interval_us = 1000000 / rate_hz;
    ScheduleOnInterval(interval_us);  // 设置调度间隔

    return true;
}
```

**关键点**：
- 使用现有的`lp_default` work queue（低优先级默认队列）
- `ScheduleOnInterval`: 设置周期性调度（微秒）

#### 2.3.2 Run函数（周期执行）

```cpp
void GgImuLogger::Run()
{
    if (should_exit()) {
        ScheduleClear();
        CloseLogFile();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);

    // 检查参数更新
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        ParametersUpdate();
    }

    // 获取IMU数据并记录
    const int imu_instance = _param_imu_instance.get();

    if (imu_instance == 0) {
        // 记录所有实例
        for (uint8_t i = 0; i < _vehicle_imu_subs.size(); i++) {
            vehicle_imu_s imu_data;
            if (_vehicle_imu_subs[i].update(&imu_data)) {
                LogImuData(imu_data, i);
            }
        }
    } else {
        // 记录指定实例
        uint8_t instance_index = imu_instance - 1;
        if (instance_index < _vehicle_imu_subs.size()) {
            vehicle_imu_s imu_data;
            if (_vehicle_imu_subs[instance_index].update(&imu_data)) {
                LogImuData(imu_data, instance_index);
            }
        }
    }

    perf_end(_loop_perf);
}
```

**工作流程**：
1. 检查退出标志
2. 检查参数更新
3. 根据配置读取指定IMU实例或全部实例
4. 记录数据到日志文件

#### 2.3.3 日志文件管理

**打开日志文件**：
```cpp
bool GgImuLogger::OpenLogFile()
{
    // 确保日志目录存在
    const char *log_dir = "/fs/microsd/log_gg_imu";
    mkdir(log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

    // 生成文件名（带时间戳）
    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);

    const char *instance_str = "";
    int imu_instance = _param_imu_instance.get();
    if (imu_instance == 1) {
        instance_str = "_imu1";
    } else if (imu_instance == 2) {
        instance_str = "_imu2";
    } else {
        instance_str = "_all";
    }

    snprintf(_log_filename, sizeof(_log_filename),
             "%s/%04d%02d%02d_%02d%02d%02d%s_%03d.ulg",
             log_dir,
             timeinfo->tm_year + 1900,
             timeinfo->tm_mon + 1,
             timeinfo->tm_mday,
             timeinfo->tm_hour,
             timeinfo->tm_min,
             timeinfo->tm_sec,
             instance_str,
             _log_sequence);

    // 打开文件
    _log_fd = ::open(_log_filename, O_CREAT | O_WRONLY | O_TRUNC, PX4_O_MODE_666);

    if (_log_fd < 0) {
        PX4_ERR("Failed to open log file: %s (errno=%d)", _log_filename, errno);
        return false;
    }

    // 写入ulog文件头
    if (!WriteULogHeader()) {
        ::close(_log_fd);
        _log_fd = -1;
        return false;
    }

    PX4_INFO("Log file opened: %s", _log_filename);
    return true;
}
```

**文件命名规则**：
```
格式: /fs/microsd/log_gg_imu/YYYYMMDD_HHMMSS_[imu1|imu2|all]_XXX.ulg

示例:
- /fs/microsd/log_gg_imu/20251029_143022_all_000.ulg
- /fs/microsd/log_gg_imu/20251029_143022_imu1_000.ulg
```

**写入ulog文件头**：
```cpp
bool GgImuLogger::WriteULogHeader()
{
    // ULog文件魔术字节
    const uint8_t magic[] = {'U', 'L', 'o', 'g', 0x01, 0x12, 0x35};
    if (::write(_log_fd, magic, sizeof(magic)) != sizeof(magic)) {
        return false;
    }

    // 写入时间戳
    uint64_t timestamp = hrt_absolute_time();
    if (::write(_log_fd, &timestamp, sizeof(timestamp)) != sizeof(timestamp)) {
        return false;
    }

    // 写入格式定义
    const char *format_msg = "F\x00\x00vehicle_imu uint64_t timestamp;uint64_t timestamp_sample;"
                             "uint32_t accel_device_id;uint32_t gyro_device_id;"
                             "float[3] delta_angle;float[3] delta_velocity;"
                             "uint16_t delta_angle_dt;uint16_t delta_velocity_dt;"
                             "uint8_t delta_velocity_clipping;uint8_t delta_angle_clipping;";

    uint16_t msg_size = strlen(format_msg + 3);
    format_msg[1] = msg_size & 0xFF;
    format_msg[2] = (msg_size >> 8) & 0xFF;

    if (::write(_log_fd, format_msg, msg_size + 3) != (ssize_t)(msg_size + 3)) {
        return false;
    }

    return true;
}
```

**ulog格式说明**：
- 魔术字节: `ULog\x01\x12\x35`
- 时间戳: 8字节uint64_t
- 格式定义: 'F' + size + 消息格式字符串

#### 2.3.4 记录数据

```cpp
void GgImuLogger::LogImuData(const vehicle_imu_s &imu_data, uint8_t instance)
{
    if (_log_fd < 0) {
        return;
    }

    perf_begin(_write_perf);

    // 准备数据缓冲区
    struct __attribute__((packed)) {
        uint8_t msg_type;
        uint16_t msg_size;
        uint16_t msg_id;
        vehicle_imu_s data;
    } log_msg;

    log_msg.msg_type = ULOG_MSG_TYPE_DATA;  // 'D'
    log_msg.msg_size = sizeof(vehicle_imu_s);
    log_msg.msg_id = instance;
    log_msg.data = imu_data;

    // 写入数据
    ssize_t written = ::write(_log_fd, &log_msg, sizeof(log_msg));

    if (written == sizeof(log_msg)) {
        _log_write_count++;
    } else {
        PX4_ERR("Write failed: %d (errno=%d)", (int)written, errno);
    }

    perf_end(_write_perf);
}
```

### 2.4 CMakeLists.txt

```cmake
px4_add_module(
    MODULE modules__gg_imu_logger
    MAIN gg_imu_logger
    COMPILE_FLAGS
    SRCS
        gg_imu_logger.cpp
    DEPENDS
        px4_work_queue
)
```

**说明**：
- `MODULE`: 模块名称（用于构建系统）
- `MAIN`: 可执行命令名
- `DEPENDS`: 依赖项

### 2.5 Kconfig

```kconfig
menuconfig MODULES_GG_IMU_LOGGER
    bool "gg_imu_logger"
    default n
    ---help---
        Enable support for GG IMU Logger module
```

**说明**：
- `menuconfig`: 在编译配置菜单中显示
- `default n`: 默认不启用（需要手动配置）

### 2.6 module.yaml（参数定义）

```yaml
module_name: GG IMU Logger

parameters:
    - group: GG IMU Logger
      definitions:

        GG_START:
            description:
                short: Enable GG IMU Logger
                long: |
                    Enables the GG IMU Logger module to start automatically at boot.

                    0: Disabled (manual start)
                    1: Enabled (auto start)
            type: int32
            default: 0
            min: 0
            max: 1

        GG_IMU_INSTANCE:
            description:
                short: IMU instance to log
                long: |
                    Select which vehicle_imu instance to log:

                    0: Log all instances
                    1: Log only instance 1
                    2: Log only instance 2
            type: int32
            default: 0
            min: 0
            max: 2

        GG_LOG_RATE:
            description:
                short: Logging rate in Hz
                long: |
                    Rate at which IMU data is logged to file.
            type: int32
            default: 100
            min: 1
            max: 1000
            unit: Hz
```

**参数说明**：
- `GG_START`: 控制模块是否自动启动
- `GG_IMU_INSTANCE`: 选择记录哪个IMU实例
- `GG_LOG_RATE`: 记录频率

---

## 3. 集成到构建系统

### 3.1 添加到板级配置

**文件**: `boards/micoair/h743/init/rc.board_defaults`

```bash
# GG IMU Logger parameters
param set-default GG_START 0
param set-default GG_IMU_INSTANCE 0
param set-default GG_LOG_RATE 100
```

### 3.2 添加到启动脚本

**文件**: `ROMFS/px4fmu_common/init.d/rcS`

在`payload_deliverer start`之后添加：

```bash
#
# GG IMU Logger - 自动启动（如果启用）
#
if param compare -s GG_START 1
then
    gg_imu_logger start
fi
```

**启动逻辑**：
- 检查`GG_START`参数
- 如果为1，自动启动模块
- 如果为0，需要手动启动

---

## 4. 编译和烧录

### 4.1 编译固件

```bash
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot

# 清理旧的构建
make clean

# 编译MicoAir H743
make micoair_h743_default

# 或使用其他板子
# make px4_fmu-v5_default
```

### 4.2 烧录固件

**方法1: USB烧录**
```bash
make micoair_h743_default upload
```

**方法2: 通过QGC烧录**
1. 打开QGroundControl
2. Vehicle Setup → Firmware
3. 选择Custom firmware file
4. 选择`build/micoair_h743_default/micoair_h743_default.px4`

### 4.3 验证编译

```bash
# 检查模块是否被编译
ls -lh build/micoair_h743_default/src/modules/gg_imu_logger/

# 检查固件大小
ls -lh build/micoair_h743_default/micoair_h743_default.bin
```

---

## 5. 使用方法

### 5.1 通过NSH Shell使用

**连接飞控串口**：
```bash
# 波特率: 57600
screen /dev/ttyACM0 57600

# 或使用minicom
minicom -D /dev/ttyACM0 -b 57600
```

**手动启动模块**：
```bash
nsh> gg_imu_logger start
[gg_imu_logger] Log file opened: /fs/microsd/log_gg_imu/20251029_143022_all_000.ulg
```

**查看模块状态**：
```bash
nsh> gg_imu_logger status
Running
Log file: /fs/microsd/log_gg_imu/20251029_143022_all_000.ulg
Entries written: 15234
IMU instance: 0 (0=all, 1=imu1, 2=imu2)
Log rate: 100 Hz

Performance counters:
gg_imu_logger: cycle: 1523 events, 15234 elapsed, 10us avg, 8us min, 25us max
gg_imu_logger: interval: 1523 events, 15234 elapsed, 10000us avg
gg_imu_logger: write: 1523 events, 5234 elapsed, 3us avg
```

**停止模块**：
```bash
nsh> gg_imu_logger stop
[gg_imu_logger] Log file closed: /fs/microsd/log_gg_imu/20251029_143022_all_000.ulg (wrote 15234 entries)
```

### 5.2 通过QGC配置参数

**步骤**：
1. 打开QGroundControl
2. Vehicle Setup → Parameters
3. 搜索"GG"
4. 配置参数：
   - `GG_START = 1` (自动启动)
   - `GG_IMU_INSTANCE = 1` (只记录IMU1)
   - `GG_LOG_RATE = 200` (200Hz记录频率)
5. 点击"Save"
6. 重启飞控

**参数配置界面**：
```
┌─────────────────────────────────────────┐
│ GG IMU Logger                           │
├─────────────────────────────────────────┤
│ GG_START             [0]  ▼  0/1       │
│ GG_IMU_INSTANCE      [0]  ▼  0/1/2     │
│ GG_LOG_RATE          [100] ▼  1-1000   │
└─────────────────────────────────────────┘
```

### 5.3 自动启动配置

**永久启用自动启动**：
```bash
# 方法1: 通过NSH
nsh> param set GG_START 1
nsh> param save
nsh> reboot

# 方法2: 通过QGC (如上所述)
```

**临时启动（不保存参数）**：
```bash
nsh> gg_imu_logger start
```

---

## 6. 通过QGC下载日志文件

### 6.1 使用QGC日志下载功能

**步骤**：
1. 打开QGroundControl
2. 连接飞控（USB或遥测）
3. 点击顶部工具栏 **Analyze** 图标
4. 选择 **Log Download**
5. 查看可用日志列表
6. 选择要下载的日志文件
7. 点击 **Download** 按钮
8. 选择保存位置

**QGC日志下载界面**：
```
┌───────────────────────────────────────────────────────────┐
│ Log Download                                              │
├───────────────────────────────────────────────────────────┤
│ ┌─────────────────────────────────────────────────────┐  │
│ │ Available Logs                                      │  │
│ ├─────────────────────────────────────────────────────┤  │
│ │ □ 2025-10-29 14:30:22  log_001.ulg     2.5 MB      │  │
│ │ □ 2025-10-29 14:25:10  log_000.ulg     1.8 MB      │  │
│ │ □ 2025-10-29 14:20:05  log_002.ulg     3.2 MB      │  │
│ │                                                     │  │
│ │ Custom logs (log_gg_imu):                          │  │
│ │ □ 2025-10-29 14:30:22  *_all_000.ulg   5.2 MB     │  │
│ │ □ 2025-10-29 14:25:10  *_imu1_000.ulg  2.8 MB     │  │
│ └─────────────────────────────────────────────────────┘  │
│                                                           │
│ [Refresh]  [Download Selected]  [Download All]           │
└───────────────────────────────────────────────────────────┘
```

**注意**：
- QGC会扫描`/fs/microsd/log/`目录（标准日志）
- 自定义日志在`/fs/microsd/log_gg_imu/`，可能不会自动显示
- 需要通过文件浏览器下载（见下节）

### 6.2 使用MAVLink FTP下载

PX4支持通过MAVLink FTP协议访问SD卡文件系统。

**QGC文件管理器**（如果支持）：
1. Analyze → File Manager
2. 浏览到 `/log_gg_imu/`
3. 选择文件并下载

**使用pymavlink脚本下载**：
```python
#!/usr/bin/env python3
from pymavlink import mavutil
import sys

# 连接到飞控
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# 等待心跳
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

# 列出日志文件
master.mav.file_request_list_send(
    master.target_system,
    master.target_component,
    0,  # offset
    "/log_gg_imu/".encode('utf-8')
)

# 接收文件列表
while True:
    msg = master.recv_match(type=['FILE_ENTRY'], blocking=True, timeout=5)
    if msg:
        print(f"File: {msg.name} (size: {msg.size} bytes)")
    else:
        break
```

### 6.3 使用download_logs.py脚本

PX4提供了专门的日志下载脚本：

**位置**: `Tools/log_encryption/download_logs.py`

**使用方法**：
```bash
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot/Tools/log_encryption

# USB连接下载
python3 download_logs.py /dev/ttyACM0 --baudrate 57600

# UDP连接下载（SITL或遥测）
python3 download_logs.py udp:0.0.0.0:14550
```

**修改脚本支持自定义路径**：

编辑`download_logs.py`，在`main()`函数中添加：

```python
# 默认下载 /log/ 目录
LOG_DIR = "/fs/microsd/log/"

# 添加自定义目录
CUSTOM_LOG_DIRS = [
    "/fs/microsd/log_gg_imu/",
]

for log_dir in CUSTOM_LOG_DIRS:
    print(f"Downloading logs from {log_dir}...")
    # 下载逻辑
```

### 6.4 直接通过NSH复制到电脑

**方法1: 通过串口传输（较慢）**
```bash
# 在NSH中
nsh> cat /fs/microsd/log_gg_imu/20251029_143022_all_000.ulg > /dev/ttyACM0

# 在PC上接收
cat /dev/ttyACM0 > log_file.ulg
```

**方法2: 通过SD卡读卡器（推荐）**
1. 关闭飞控
2. 取出SD卡
3. 插入电脑读卡器
4. 复制`/log_gg_imu/`目录中的文件

**方法3: 使用NSH的dumpfile命令**
```bash
# 在NSH中以十六进制显示文件
nsh> dumpfile /fs/microsd/log_gg_imu/20251029_143022_all_000.ulg
```

---

## 7. 日志文件分析

### 7.1 查看日志文件内容

**使用ulog_info**：
```bash
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot/Tools

# 查看日志信息
python3 -m pyulog.ulog_info /path/to/20251029_143022_all_000.ulg

# 输出示例
ULog version: 1
Timestamp: 2025-10-29 14:30:22
Duration: 152.3 s
Data messages:
  vehicle_imu
    timestamp
    timestamp_sample
    delta_angle[3]
    delta_velocity[3]
    ...
```

**使用ulog2csv转换为CSV**：
```bash
# 转换为CSV
python3 -m pyulog.ulog2csv /path/to/20251029_143022_all_000.ulg

# 生成CSV文件
# vehicle_imu_0.csv (实例0)
# vehicle_imu_1.csv (实例1)
```

### 7.2 用Python分析

```python
#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from pyulog import ULog

# 读取ulog文件
ulog = ULog('/path/to/20251029_143022_all_000.ulg')

# 提取vehicle_imu数据
imu_data = ulog.get_dataset('vehicle_imu')

# 转换为DataFrame
df = pd.DataFrame({
    'timestamp': imu_data.data['timestamp'],
    'delta_angle_x': imu_data.data['delta_angle[0]'],
    'delta_angle_y': imu_data.data['delta_angle[1]'],
    'delta_angle_z': imu_data.data['delta_angle[2]'],
    'delta_velocity_x': imu_data.data['delta_velocity[0]'],
    'delta_velocity_y': imu_data.data['delta_velocity[1]'],
    'delta_velocity_z': imu_data.data['delta_velocity[2]'],
})

# 绘图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

# 角速度积分
ax1.plot(df['timestamp'], df['delta_angle_x'], label='X')
ax1.plot(df['timestamp'], df['delta_angle_y'], label='Y')
ax1.plot(df['timestamp'], df['delta_angle_z'], label='Z')
ax1.set_ylabel('Delta Angle (rad)')
ax1.set_title('Gyroscope Delta Angle')
ax1.legend()
ax1.grid(True)

# 加速度积分
ax2.plot(df['timestamp'], df['delta_velocity_x'], label='X')
ax2.plot(df['timestamp'], df['delta_velocity_y'], label='Y')
ax2.plot(df['timestamp'], df['delta_velocity_z'], label='Z')
ax2.set_xlabel('Time (us)')
ax2.set_ylabel('Delta Velocity (m/s)')
ax2.set_title('Accelerometer Delta Velocity')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.savefig('imu_analysis.png', dpi=150)
plt.show()

print(f"Total samples: {len(df)}")
print(f"Duration: {(df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]) / 1e6:.2f} s")
print(f"Average rate: {len(df) / ((df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]) / 1e6):.2f} Hz")
```

### 7.3 上传到Flight Review

```bash
# 虽然是自定义日志，但仍然可以上传到Flight Review进行可视化
# 访问: https://logs.px4.io/
# 上传: 20251029_143022_all_000.ulg
```

---

## 8. 故障排查

### 8.1 模块启动失败

**现象**：
```bash
nsh> gg_imu_logger start
ERROR [gg_imu_logger] Failed to open log file
```

**原因**：
- SD卡未挂载
- SD卡空间不足
- 目录权限问题

**解决方法**：
```bash
# 检查SD卡挂载
nsh> df
Filesystem      Size  Used  Avail  Use%  Mounted on
/dev/mmcsd0    32GB  2GB   30GB   6%    /fs/microsd

# 检查目录
nsh> ls -l /fs/microsd/log_gg_imu/

# 手动创建目录
nsh> mkdir /fs/microsd/log_gg_imu

# 检查磁盘空间
nsh> free
```

### 8.2 参数不生效

**现象**：
```bash
nsh> param set GG_START 1
nsh> reboot
# 重启后模块没有自动启动
```

**原因**：
- 参数未保存
- rcS脚本配置错误

**解决方法**：
```bash
# 确保保存参数
nsh> param set GG_START 1
nsh> param save
nsh> param show GG_START  # 确认参数值

# 检查启动脚本
nsh> cat /etc/init.d/rcS | grep gg_imu_logger
```

### 8.3 日志写入速度慢

**现象**：
- 性能计数器显示写入时间过长
- 日志丢失数据

**原因**：
- SD卡速度慢
- 记录频率过高
- Work queue优先级低

**解决方法**：
```bash
# 降低记录频率
nsh> param set GG_LOG_RATE 50
nsh> param save

# 使用更快的SD卡（Class 10或UHS-I）

# 修改work queue（需要重新编译）
# 将lp_default改为rate_ctrl
```

### 8.4 编译错误

**现象**：
```
error: 'GG_START' was not declared in this scope
```

**原因**：
- 参数定义未被正确识别
- 需要重新生成参数代码

**解决方法**：
```bash
# 清理并重新编译
make clean
make micoair_h743_default

# 确保module.yaml在正确位置
ls src/modules/gg_imu_logger/module.yaml
```

---

## 9. 高级功能扩展

### 9.1 添加数据过滤

在`LogImuData()`函数中添加：

```cpp
void GgImuLogger::LogImuData(const vehicle_imu_s &imu_data, uint8_t instance)
{
    // 添加数据过滤逻辑
    bool should_log = true;

    // 示例: 只记录高g值数据
    float accel_mag = sqrtf(
        imu_data.delta_velocity[0] * imu_data.delta_velocity[0] +
        imu_data.delta_velocity[1] * imu_data.delta_velocity[1] +
        imu_data.delta_velocity[2] * imu_data.delta_velocity[2]
    );

    if (accel_mag < 0.5f) {  // 小于0.5 m/s的积分不记录
        should_log = false;
    }

    if (!should_log) {
        return;
    }

    // 原有记录逻辑
    // ...
}
```

### 9.2 添加实时数据发布

除了记录到文件，还可以发布到uORB：

```cpp
class GgImuLogger : public ... {
private:
    uORB::Publication<vehicle_imu_s> _imu_logged_pub{ORB_ID(vehicle_imu_logged)};
};

void GgImuLogger::LogImuData(const vehicle_imu_s &imu_data, uint8_t instance)
{
    // 记录到文件
    // ...

    // 同时发布到uORB
    vehicle_imu_s logged_imu = imu_data;
    logged_imu.timestamp = hrt_absolute_time();
    _imu_logged_pub.publish(logged_imu);
}
```

### 9.3 添加触发条件

只在特定条件下记录：

```cpp
private:
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    bool _armed{false};
};

void GgImuLogger::Run()
{
    // 更新解锁状态
    vehicle_status_s vehicle_status;
    if (_vehicle_status_sub.update(&vehicle_status)) {
        _armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
    }

    // 只在解锁时记录
    if (!_armed) {
        return;
    }

    // 原有记录逻辑
    // ...
}
```

---

## 10. 关键源码位置总结

### 10.1 模块源码

| 文件 | 路径 | 说明 |
|------|------|------|
| **模块头文件** | `src/modules/gg_imu_logger/gg_imu_logger.hpp` | 类定义 |
| **模块实现** | `src/modules/gg_imu_logger/gg_imu_logger.cpp` | 功能实现 |
| **构建配置** | `src/modules/gg_imu_logger/CMakeLists.txt` | CMake配置 |
| **Kconfig配置** | `src/modules/gg_imu_logger/Kconfig` | 编译选项 |
| **参数定义** | `src/modules/gg_imu_logger/module.yaml` | 参数定义 |

### 10.2 配置文件

| 文件 | 路径 | 说明 |
|------|------|------|
| **板级参数** | `boards/micoair/h743/init/rc.board_defaults` | 默认参数值 |
| **启动脚本** | `ROMFS/px4fmu_common/init.d/rcS` | 自动启动逻辑 |

### 10.3 参考代码

| 功能 | 参考文件 | 说明 |
|------|---------|------|
| **Work Queue示例** | `src/examples/work_item/WorkItemExample.cpp` | work queue用法 |
| **uORB多实例订阅** | `src/modules/ekf2/EKF2.cpp` | 多实例订阅示例 |
| **日志系统** | `src/modules/logger/logger.cpp` | 标准日志实现 |
| **参数系统** | `src/lib/parameters/` | 参数系统实现 |

---

## 11. 总结

### 11.1 开发流程总结

```
1. 创建模块目录
   ↓
2. 编写.hpp和.cpp
   ↓
3. 创建CMakeLists.txt和Kconfig
   ↓
4. 定义参数(module.yaml)
   ↓
5. 添加到板级配置
   ↓
6. 添加到启动脚本
   ↓
7. 编译和烧录
   ↓
8. 测试和调试
```

### 11.2 关键技术点

1. **ModuleBase框架**: 提供标准模块接口
2. **ScheduledWorkItem**: 周期性任务调度
3. **uORB订阅**: 数据订阅机制
4. **参数系统**: 运行时配置
5. **ulog格式**: 日志文件格式
6. **MAVLink FTP**: 远程文件访问

### 11.3 最佳实践

1. ✅ 使用现有的work queue而非创建新的
2. ✅ 参数使用module.yaml定义
3. ✅ 添加性能计数器监控
4. ✅ 实现status命令查看运行状态
5. ✅ 处理文件操作错误
6. ✅ 支持通过参数动态配置
7. ✅ 提供清晰的日志输出

### 11.4 常见问题

**Q: 如何选择work queue?**
A: 根据任务优先级和频率选择：
- 高频高优先级：`INS0`
- 中频中优先级：`hp_default`
- 低频低优先级：`lp_default`

**Q: 如何知道有多少个IMU实例?**
A: 通过`listener vehicle_imu`命令查看

**Q: 日志文件可以在飞行中读取吗?**
A: 不建议，可能影响写入性能。使用uORB发布代替。

**Q: 如何减少日志文件大小?**
A: 降低记录频率、添加过滤条件、只记录需要的字段

---

**文档版本**: v1.0
**创建日期**: 2025-10-29
**适用PX4版本**: v1.14+
**作者**: GG
**测试平台**: MicoAir H743

