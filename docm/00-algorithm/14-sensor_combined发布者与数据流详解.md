# 14-sensor_combined发布者与数据流详解
## Sensors模块数据采集、融合与发布全流程分析

---

## 1. 概述

`sensor_combined` 是PX4系统中最重要的传感器数据话题之一，它包含了融合后的IMU（惯性测量单元）数据，为EKF2等估计器提供基础数据源。本文档详细分析该消息的发布者、数据来源、处理流程和发布机制。

### 1.1 核心信息摘要

| 项目 | 信息 |
|------|------|
| **发布者模块** | Sensors |
| **Publisher声明** | `src/modules/sensors/sensors.hpp:163` |
| **发布调用位置** | `src/modules/sensors/sensors.cpp:621` |
| **发布函数** | `Sensors::Run()` |
| **发布频率** | ~1000Hz（与IMU频率同步） |
| **数据内容** | 融合后的陀螺仪、加速度计数据 |
| **主要订阅者** | EKF2（单实例模式） |

---

## 2. Sensors模块架构

### 2.1 模块文件结构

```
src/modules/sensors/
├── sensors.cpp                      # 主模块实现
├── sensors.hpp                      # 主模块头文件
├── voted_sensors_update.cpp         # 传感器投票与融合
├── voted_sensors_update.h           # 投票更新头文件
├── vehicle_imu/                     # IMU数据处理
│   ├── VehicleIMU.cpp
│   └── VehicleIMU.hpp
├── vehicle_acceleration/            # 加速度处理
├── vehicle_angular_velocity/        # 角速度处理
├── vehicle_air_data/                # 气压数据处理
├── vehicle_gps_position/            # GPS数据处理
├── vehicle_magnetometer/            # 磁力计数据处理
└── vehicle_optical_flow/            # 光流数据处理
```

### 2.2 模块类定义

**文件**: `src/modules/sensors/sensors.hpp`

**第1行起**: 类声明和包含文件
```cpp
/**
 * @file sensors.hpp
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */
```

---

## 3. Publisher声明与初始化

### 3.1 Publisher声明

**文件**: `src/modules/sensors/sensors.hpp`

**第163行**: Publisher成员变量声明
```cpp
uORB::Publication<sensor_combined_s> _sensor_pub{ORB_ID(sensor_combined)};
```

**说明**:
- 使用 `uORB::Publication` 模板类
- 消息类型: `sensor_combined_s`
- ORB ID: `sensor_combined`
- 成员变量名: `_sensor_pub`

### 3.2 Publisher初始化

**文件**: `src/modules/sensors/sensors.cpp`

**第46-90行**: 构造函数中初始化
```cpp
Sensors::Sensors(bool hil_enabled) :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
    _hil_enabled(hil_enabled),
    _loop_perf(perf_alloc(PC_ELAPSED, "sensors")),
    _voted_sensors_update(hil_enabled, _vehicle_imu_sub)
{
    _sensor_pub.advertise();  // 第53行：广播Publisher
    
    // ... 其他初始化代码
    
    _sensor_combined.accelerometer_timestamp_relative = sensor_combined_s::RELATIVE_TIMESTAMP_INVALID;
    
    parameters_update();
    
    InitializeVehicleIMU();
}
```

**第53行**: `_sensor_pub.advertise()` 
- 向uORB系统注册此Publisher
- 允许其他模块发现并订阅此话题

---

## 4. 数据采集与融合流程

### 4.1 数据流程总览

```
┌─────────────────────────────────────────────────────────────────┐
│                      硬件传感器层                                  │
│  IMU1   IMU2   IMU3   (多个冗余IMU传感器)                         │
└────┬─────┬──────┬───────────────────────────────────────────────┘
     │     │      │
     ▼     ▼      ▼
┌─────────────────────────────────────────────────────────────────┐
│                      驱动层                                       │
│  sensor_accel, sensor_gyro (原始传感器数据)                       │
│  - 发布频率: 1000Hz+                                              │
│  - 多实例数据 (instance 0, 1, 2, ...)                            │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│              VehicleIMU (vehicle_imu/)                           │
│  - 订阅 sensor_accel, sensor_gyro                                │
│  - 校准补偿（bias, scale, rotation）                             │
│  - 数据降噪和滤波                                                 │
│  - 发布 vehicle_imu                                              │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│          VotedSensorsUpdate::sensorsPoll()                       │
│  (voted_sensors_update.cpp:399)                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ 1. imuPoll(raw)  - 读取多个IMU数据                        │  │
│  │    ├─ 订阅 vehicle_imu (多实例)                           │  │
│  │    ├─ 选择主IMU和备份IMU                                  │  │
│  │    └─ 数据投票与故障检测                                  │  │
│  │                                                              │  │
│  │ 2. calcAccelInconsistency()  - 加速度计一致性检查         │  │
│  │    └─ 检测各IMU间的数据差异                               │  │
│  │                                                              │  │
│  │ 3. calcGyroInconsistency()  - 陀螺仪一致性检查            │  │
│  │    └─ 检测各IMU间的数据差异                               │  │
│  │                                                              │  │
│  │ 4. 填充 sensor_combined_s 结构                            │  │
│  │    ├─ gyro_rad[3]              (角速度)                   │  │
│  │    ├─ accelerometer_m_s2[3]    (加速度)                   │  │
│  │    ├─ gyro_integral_dt          (积分时间)                │  │
│  │    ├─ accelerometer_integral_dt (积分时间)                │  │
│  │    ├─ timestamp                 (时间戳)                  │  │
│  │    ├─ accel_calibration_count   (校准计数)               │  │
│  │    └─ gyro_calibration_count    (校准计数)               │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│         Sensors::Run() - 发布逻辑                                │
│         (sensors.cpp:517)                                       │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ if (timestamp changed) {                                    │  │
│  │     setRelativeTimestamps()  - 设置相对时间戳             │  │
│  │     _sensor_pub.publish(_sensor_combined)  (621行)        │  │
│  │ }                                                           │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     ▼
           ┌─────────────────────┐
           │  sensor_combined    │
           │    (uORB话题)        │
           └─────────┬───────────┘
                     │
                     ▼
           ┌─────────────────────┐
           │   订阅者模块         │
           │  - EKF2 (单实例)     │
           │  - 姿态控制器        │
           │  - 日志记录          │
           └─────────────────────┘
```

### 4.2 数据采集详细流程

#### 4.2.1 VehicleIMU数据处理

**文件**: `src/modules/sensors/vehicle_imu/VehicleIMU.cpp`

VehicleIMU模块负责：
1. 订阅原始传感器数据（sensor_accel, sensor_gyro）
2. 应用校准参数（偏差、比例、旋转矩阵）
3. 温度补偿
4. 数据滤波
5. 发布处理后的vehicle_imu数据

**数据处理公式**:
```
calibrated_data = R * (raw_data - bias) * scale + thermal_offset
```

其中：
- `R`: 旋转矩阵（传感器坐标系到机体坐标系）
- `bias`: 零偏
- `scale`: 比例因子
- `thermal_offset`: 温度补偿偏移

#### 4.2.2 传感器投票机制

**文件**: `src/modules/sensors/voted_sensors_update.cpp`

**第399行**: `sensorsPoll`函数入口
```cpp
void VotedSensorsUpdate::sensorsPoll(sensor_combined_s &raw)
{
    imuPoll(raw);
    
    calcAccelInconsistency();
    calcGyroInconsistency();
}
```

**IMU投票流程**:

1. **读取所有可用IMU数据**
   - 从vehicle_imu多实例话题读取数据
   - 最多支持4个IMU

2. **数据有效性检查**
   - 检查时间戳
   - 检查数据范围
   - 检查更新频率

3. **投票选择主IMU**
   - 基于数据质量评分
   - 考虑历史可靠性
   - 故障检测与隔离

4. **一致性检查**
   - 计算各IMU间的数据差异
   - 设置不一致性标志
   - 触发健康检查

---

## 5. 发布机制详解

### 5.1 Run函数主循环

**文件**: `src/modules/sensors/sensors.cpp`

**第517-635行**: Run函数完整流程
```cpp
void Sensors::Run()
{
    // 退出检查
    if (should_exit()) {
        // clear all registered callbacks
        for (auto &sub : _vehicle_imu_sub) {
            sub.unregisterCallback();
        }
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);

    // 配置更新检查（每100ms）
    const hrt_abstime time_now_us = hrt_absolute_time();
    
    if (time_now_us - _last_config_update > 100_ms) {
        
        // 检测新的传感器实例
        const int n_accel = orb_group_count(ORB_ID(sensor_accel));
        const int n_gyro  = orb_group_count(ORB_ID(sensor_gyro));
        
        if ((n_accel != _n_accel) || (n_gyro != _n_gyro)) {
            _n_accel = n_accel;
            _n_gyro = n_gyro;
            
            parameters_update();
        }
        
        // 初始化传感器
        _voted_sensors_update.initializeSensors();
        InitializeVehicleIMU();
        
        _last_config_update = hrt_absolute_time();
        
    } else {
        // 参数更新检查
        if (_parameter_update_sub.updated()) {
            parameter_update_s pupdate;
            _parameter_update_sub.copy(&pupdate);
            
            parameters_update();
            updateParams();
        }
    }
    
    // ========== 核心数据采集 ==========
    // 第616行：调用投票传感器更新
    _voted_sensors_update.sensorsPoll(_sensor_combined);
    
    // 第618-623行：发布sensor_combined
    if (_sensor_combined.timestamp != _sensor_combined_prev_timestamp) {
        
        // 设置相对时间戳
        _voted_sensors_update.setRelativeTimestamps(_sensor_combined);
        
        // ★★★ 第621行：发布sensor_combined消息 ★★★
        _sensor_pub.publish(_sensor_combined);
        
        // 记录时间戳，避免重复发布
        _sensor_combined_prev_timestamp = _sensor_combined.timestamp;
    }
    
#if defined(CONFIG_SENSORS_VEHICLE_AIRSPEED)
    // 空速传感器处理
    adc_poll();
    diff_pres_poll();
#endif
    
    // 备份调度作为看门狗超时
    ScheduleDelayed(10_ms);
    
    perf_end(_loop_perf);
}
```

### 5.2 发布条件

**关键检查**（第618行）:
```cpp
if (_sensor_combined.timestamp != _sensor_combined_prev_timestamp)
```

**说明**:
- 仅当时间戳更新时才发布
- 避免发布重复数据
- 确保数据是新鲜的

**时间戳来源**:
- 由`sensorsPoll()`函数填充
- 来自主IMU的时间戳
- 通常为`hrt_absolute_time()`

### 5.3 相对时间戳设置

**第620行**:
```cpp
_voted_sensors_update.setRelativeTimestamps(_sensor_combined);
```

**功能**:
- 设置加速度计相对时间戳
- 用于时间同步
- 便于订阅者对齐数据

---

## 6. sensor_combined消息结构

### 6.1 消息定义

**文件**: `msg/sensor_combined.msg`

```msg
uint64 timestamp                    # 时间戳 (微秒)

# 陀螺仪数据
float32[3] gyro_rad                 # 角速度 (rad/s) - 机体坐标系
uint32 gyro_integral_dt             # 陀螺仪积分时间 (微秒)
uint32 gyro_calibration_count       # 陀螺仪校准计数

# 加速度计数据
float32[3] accelerometer_m_s2       # 加速度 (m/s²) - 机体坐标系
uint32 accelerometer_integral_dt    # 加速度计积分时间 (微秒)
uint32 accel_calibration_count      # 加速度计校准计数

# 削波标志
uint8 accelerometer_clipping        # 加速度计削波标志 (位掩码)
uint8 gyro_clipping                 # 陀螺仪削波标志 (位掩码)

# 相对时间戳
uint32 accelerometer_timestamp_relative  # 加速度计相对时间戳 (0.1ms单位)

# 保留字段
uint8[3] _padding0
```

### 6.2 数据坐标系

**机体坐标系（FRD）**:
- **X轴**: 前 (Forward)
- **Y轴**: 右 (Right)
- **Z轴**: 下 (Down)

**陀螺仪数据 (gyro_rad)**:
```
gyro_rad[0] = roll_rate   (绕X轴旋转角速度)
gyro_rad[1] = pitch_rate  (绕Y轴旋转角速度)
gyro_rad[2] = yaw_rate    (绕Z轴旋转角速度)
```

**加速度计数据 (accelerometer_m_s2)**:
```
accelerometer_m_s2[0] = accel_x  (X轴加速度)
accelerometer_m_s2[1] = accel_y  (Y轴加速度)
accelerometer_m_s2[2] = accel_z  (Z轴加速度，悬停时约为+9.8m/s²)
```

### 6.3 积分时间

**gyro_integral_dt** 和 **accelerometer_integral_dt**:
- 单位: 微秒 (μs)
- 表示自上次采样以来的时间间隔
- 用于计算增量角度和增量速度：
  ```
  delta_angle = gyro_rad * (gyro_integral_dt / 1e6)
  delta_velocity = accelerometer_m_s2 * (accelerometer_integral_dt / 1e6)
  ```

### 6.4 削波标志

**accelerometer_clipping** 和 **gyro_clipping**:
- 位掩码，表示哪个轴发生了削波（饱和）
- 位0: X轴
- 位1: Y轴
- 位2: Z轴

示例：
```cpp
if (sensor_combined.accelerometer_clipping & 0x01) {
    // X轴加速度计削波
}
```

---

## 7. 调度与工作队列

### 7.1 工作队列配置

**第48行**: 构造函数中指定工作队列
```cpp
ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
```

**工作队列**: `nav_and_controllers`
- 优先级: 中等
- 用于导航和控制相关任务
- 与EKF2、姿态控制器等运行在相同队列

### 7.2 调度机制

**触发方式**:
1. **回调触发**: 订阅vehicle_imu时使用回调
2. **定时触发**: 第632行的备份调度
   ```cpp
   ScheduleDelayed(10_ms);  // 10ms超时保护
   ```

**实际执行频率**:
- 主要由IMU数据到达频率决定
- 通常为~1000Hz
- 10ms超时作为故障保护

---

## 8. 传感器投票与冗余

### 8.1 多IMU支持

Sensors模块支持最多**4个IMU**同时运行：

```cpp
// sensors.hpp
static constexpr uint8_t MAX_SENSOR_COUNT = 4;
```

### 8.2 投票算法

**优先级评分因素**:
1. **数据有效性**: 时间戳、范围检查
2. **更新频率**: 是否按时更新
3. **一致性**: 与其他IMU的数据对比
4. **历史可靠性**: 过去的故障记录
5. **校准状态**: 是否已校准

### 8.3 故障检测

**检测项目**:
- 数据卡住（时间戳不更新）
- 数据跳变（突变检测）
- 传感器不一致（多IMU差异过大）
- 校准丢失（校准计数变化）

**故障处理**:
- 降低故障传感器优先级
- 自动切换到备份传感器
- 记录健康状态
- 触发警告事件

---

## 9. 与EKF2的接口

### 9.1 EKF2订阅

**文件**: `src/modules/ekf2/EKF2.hpp`

**第383行**: EKF2订阅sensor_combined
```cpp
uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
```

### 9.2 数据使用

**文件**: `src/modules/ekf2/EKF2.cpp`

**第554-599行**: EKF2读取sensor_combined
```cpp
const unsigned last_generation = _sensor_combined_sub.get_last_generation();
sensor_combined_s sensor_combined;
imu_updated = _sensor_combined_sub.update(&sensor_combined);

if (imu_updated) {
    imu_sample_new.time_us = sensor_combined.timestamp;
    imu_sample_new.delta_ang_dt = sensor_combined.gyro_integral_dt * 1.e-6f;
    imu_sample_new.delta_ang = Vector3f{sensor_combined.gyro_rad} * imu_sample_new.delta_ang_dt;
    imu_sample_new.delta_vel_dt = sensor_combined.accelerometer_integral_dt * 1.e-6f;
    imu_sample_new.delta_vel = Vector3f{sensor_combined.accelerometer_m_s2} * imu_sample_new.delta_vel_dt;
    
    // 处理削波标志
    if (sensor_combined.accelerometer_clipping > 0) {
        imu_sample_new.delta_vel_clipping[0] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_X;
        imu_sample_new.delta_vel_clipping[1] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Y;
        imu_sample_new.delta_vel_clipping[2] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Z;
    }
}
```

### 9.3 单实例模式 vs 多实例模式

| 模式 | 数据源 | 使用场景 |
|------|--------|----------|
| **单实例** | `sensor_combined` | 标准配置，一个EKF实例 |
| **多实例** | `vehicle_imu` | 冗余配置，多个EKF实例，每个使用不同IMU |

**单实例模式**:
- EKF2订阅 `sensor_combined`
- 已经过投票和融合
- CPU开销较小

**多实例模式**:
- 每个EKF2实例订阅一个`vehicle_imu`
- 独立运行多个EKF
- 通过EKF2Selector选择最优估计
- CPU开销较大，但冗余性更好

---

## 10. 性能分析

### 10.1 执行时间

**性能计数器**（第50行）:
```cpp
_loop_perf(perf_alloc(PC_ELAPSED, "sensors"))
```

**典型执行时间**:
- **正常情况**: 50-100μs
- **配置更新**: 200-500μs
- **传感器初始化**: 1-5ms

### 10.2 发布频率

| 项目 | 频率 | 说明 |
|------|------|------|
| IMU采样频率 | 1000-8000Hz | 取决于硬件 |
| vehicle_imu发布 | 1000Hz | 降采样后 |
| sensor_combined发布 | ~1000Hz | 与主IMU同步 |

### 10.3 CPU负载

**典型负载**（基于pixhawk硬件）:
- Sensors模块: ~5-10% CPU
- 单次Run调用: ~0.05-0.1ms
- 每秒调用次数: ~1000次

---

## 11. 启动与配置

### 11.1 模块启动

**启动脚本位置**: `ROMFS/px4fmu_common/init.d/rcS`

**启动命令**:
```bash
sensors start
```

**可选参数**:
```bash
sensors start -h    # HIL（硬件在环）模式
```

### 11.2 相关参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `SENS_BOARD_ROT` | 0 | 板载传感器旋转 |
| `SENS_BOARD_X_OFF` | 0.0 | X轴偏移 (m) |
| `SENS_BOARD_Y_OFF` | 0.0 | Y轴偏移 (m) |
| `SENS_BOARD_Z_OFF` | 0.0 | Z轴偏移 (m) |
| `IMU_ACCEL_CUTOFF` | 30.0 | 加速度计截止频率 (Hz) |
| `IMU_GYRO_CUTOFF` | 30.0 | 陀螺仪截止频率 (Hz) |
| `IMU_GYRO_RATEMAX` | 400.0 | 陀螺仪最大角速度 (°/s) |

### 11.3 校准流程

**加速度计校准**:
```bash
commander calibrate accel
```

**陀螺仪校准**:
```bash
commander calibrate gyro
```

**磁力计校准**:
```bash
commander calibrate mag
```

---

## 12. 调试与诊断

### 12.1 查看发布状态

**查看sensor_combined话题**:
```bash
listener sensor_combined
```

**输出示例**:
```
TOPIC: sensor_combined
    timestamp: 1234567890
    gyro_rad[0]: 0.001234
    gyro_rad[1]: -0.000567
    gyro_rad[2]: 0.000123
    accelerometer_m_s2[0]: 0.05
    accelerometer_m_s2[1]: -0.03
    accelerometer_m_s2[2]: 9.81
    gyro_integral_dt: 1000
    accelerometer_integral_dt: 1000
```

### 12.2 检查传感器健康

**查看传感器状态**:
```bash
sensors status
```

**输出信息**:
- 活动传感器数量
- 主IMU选择
- 传感器一致性
- 性能统计

### 12.3 常见问题

#### 问题1: sensor_combined不发布

**可能原因**:
1. Sensors模块未启动
2. IMU驱动未加载
3. 传感器硬件故障

**诊断步骤**:
```bash
# 检查模块状态
ps | grep sensors

# 检查原始传感器数据
listener sensor_accel
listener sensor_gyro

# 重启模块
sensors stop
sensors start
```

#### 问题2: 数据更新频率低

**可能原因**:
1. CPU负载过高
2. IMU驱动问题
3. 中断配置错误

**诊断步骤**:
```bash
# 检查CPU负载
top

# 检查性能计数器
perf
```

#### 问题3: 传感器数据异常

**可能原因**:
1. 校准参数错误
2. 传感器安装方向错误
3. 硬件故障或干扰

**解决方法**:
- 重新校准传感器
- 检查`SENS_BOARD_ROT`参数
- 检查硬件连接和隔振

---

## 13. 代码调用链总结

### 13.1 完整调用链

```
系统启动
  ↓
rcS脚本: sensors start
  ↓
Sensors::task_spawn() (sensors.cpp:637)
  ↓
Sensors::Sensors() 构造函数 (sensors.cpp:46)
  ├─ _sensor_pub.advertise() (第53行)
  ├─ parameters_update() (第87行)
  └─ InitializeVehicleIMU() (第89行)
  ↓
工作队列调度
  ↓
Sensors::Run() (sensors.cpp:517) ← 主循环，~1000Hz
  ↓
  ├─ 配置更新检查 (每100ms)
  │   ├─ orb_group_count() - 检测新传感器
  │   ├─ _voted_sensors_update.initializeSensors()
  │   └─ InitializeVehicleIMU()
  │
  ├─ 参数更新检查
  │   └─ parameters_update()
  │
  ├─ _voted_sensors_update.sensorsPoll(_sensor_combined) (第616行)
  │   ↓
  │   VotedSensorsUpdate::sensorsPoll() (voted_sensors_update.cpp:399)
  │     ├─ imuPoll(raw)
  │     │   ├─ 订阅vehicle_imu数据
  │     │   ├─ 传感器投票选择
  │     │   └─ 填充sensor_combined结构
  │     │
  │     ├─ calcAccelInconsistency()
  │     │   └─ 计算加速度计一致性
  │     │
  │     └─ calcGyroInconsistency()
  │         └─ 计算陀螺仪一致性
  │
  ├─ 时间戳检查 (第618行)
  │   if (_sensor_combined.timestamp != _sensor_combined_prev_timestamp)
  │
  ├─ _voted_sensors_update.setRelativeTimestamps() (第620行)
  │   └─ 设置加速度计相对时间戳
  │
  └─ _sensor_pub.publish(_sensor_combined) (第621行) ★★★
      ↓
      uORB发布系统
      ↓
      订阅者接收数据
        ├─ EKF2::Run()
        ├─ 姿态控制器
        └─ 日志系统
```

### 13.2 关键函数位置总结

| 功能 | 文件 | 函数 | 行号 |
|------|------|------|------|
| **Publisher声明** | sensors.hpp | - | 163 |
| **Publisher初始化** | sensors.cpp | Sensors() | 53 |
| **主循环入口** | sensors.cpp | Run() | 517 |
| **数据采集** | sensors.cpp | Run() | 616 |
| **传感器投票** | voted_sensors_update.cpp | sensorsPoll() | 399 |
| **时间戳检查** | sensors.cpp | Run() | 618 |
| **相对时间戳** | sensors.cpp | Run() | 620 |
| **发布调用** | sensors.cpp | Run() | **621** |

---

## 14. 与其他传感器数据流对比

### 14.1 传感器数据流对比表

| 传感器类型 | 原始数据 | 处理模块 | 融合数据 | 发布者 | 订阅者 |
|-----------|---------|---------|---------|--------|--------|
| **IMU** | sensor_accel<br>sensor_gyro | VehicleIMU | sensor_combined | Sensors | EKF2 (单实例) |
| **IMU** | sensor_accel<br>sensor_gyro | VehicleIMU | vehicle_imu | VehicleIMU | EKF2 (多实例) |
| **加速度** | sensor_accel | VehicleAcceleration | vehicle_acceleration | VehicleAcceleration | 控制器 |
| **角速度** | sensor_gyro | VehicleAngularVelocity | vehicle_angular_velocity | VehicleAngularVelocity | 控制器 |
| **磁力计** | sensor_mag | VehicleMagnetometer | vehicle_magnetometer | VehicleMagnetometer | EKF2 |
| **气压计** | sensor_baro | VehicleAirData | vehicle_air_data | VehicleAirData | EKF2 |
| **GPS** | sensor_gps | VehicleGPSPosition | vehicle_gps_position | VehicleGPSPosition | EKF2 |

### 14.2 数据流架构图

```
┌──────────────────────────────────────────────────────────────┐
│                        硬件驱动层                              │
│  sensor_accel, sensor_gyro, sensor_mag, sensor_baro, ...     │
└──────────┬───────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────┐
│                    Sensors模块协调层                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │
│  │ VehicleIMU   │  │VehicleAirData│  │VehicleGPS... │       │
│  │ (校准/滤波)  │  │ (校准/滤波)  │  │ (融合/滤波)  │       │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘       │
│         │                  │                  │               │
│         ▼                  ▼                  ▼               │
│  vehicle_imu      vehicle_air_data   vehicle_gps_position    │
│         │                  │                  │               │
│         └─────────┬────────┴──────────────────┘               │
│                   ▼                                           │
│         VotedSensorsUpdate                                    │
│         (投票 + 融合)                                         │
│                   │                                           │
│                   ▼                                           │
│         sensor_combined ← Sensors::Run():621发布             │
└──────────┬────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────┐
│                      估计与控制层                              │
│  ┌─────────┐  ┌─────────────┐  ┌──────────────┐            │
│  │  EKF2   │  │姿态控制器   │  │位置控制器    │            │
│  └─────────┘  └─────────────┘  └──────────────┘            │
└──────────────────────────────────────────────────────────────┘
```

---

## 15. 总结

### 15.1 关键要点

1. **发布者**: Sensors模块（`src/modules/sensors/`）
2. **发布位置**: `sensors.cpp:621` - `_sensor_pub.publish(_sensor_combined)`
3. **发布频率**: ~1000Hz，与主IMU同步
4. **数据来源**: 多IMU投票融合后的结果
5. **主要订阅者**: EKF2（单实例模式）

### 15.2 数据质量保证

Sensors模块通过以下机制保证数据质量：

1. **多传感器冗余**: 最多支持4个IMU
2. **投票算法**: 自动选择最优传感器
3. **故障检测**: 实时监控传感器健康
4. **校准补偿**: 应用校准参数修正数据
5. **一致性检查**: 检测传感器间差异
6. **滤波处理**: 降噪和平滑处理

### 15.3 性能特点

- **低延迟**: 数据从传感器到发布 < 1ms
- **高频率**: 1000Hz发布频率
- **高可靠**: 冗余设计和故障隔离
- **低开销**: CPU占用 ~5-10%

### 15.4 适用场景

sensor_combined主要用于：
- 单EKF实例配置（标准配置）
- 需要融合后IMU数据的应用
- 对实时性要求高的控制回路
- 日志记录和回放

---

## 16. 参考资料

### 16.1 相关文件

| 文件 | 行数 | 说明 |
|------|------|------|
| `sensors.cpp` | 800+ | 主模块实现 |
| `sensors.hpp` | 260+ | 主模块头文件 |
| `voted_sensors_update.cpp` | 600+ | 投票与融合 |
| `VehicleIMU.cpp` | 600+ | IMU数据处理 |

### 16.2 相关话题

- `sensor_accel` - 原始加速度计数据
- `sensor_gyro` - 原始陀螺仪数据
- `vehicle_imu` - 处理后的IMU数据
- `sensor_combined` - 融合后的传感器数据（本文档重点）
- `vehicle_acceleration` - 机体加速度
- `vehicle_angular_velocity` - 机体角速度

### 16.3 相关模块

- **Sensors** - 传感器融合模块（本文档）
- **EKF2** - 扩展卡尔曼滤波器（订阅者）
- **VehicleIMU** - IMU数据处理
- **驱动层** - 各传感器硬件驱动

---

**文档版本**: v1.0  
**创建日期**: 2025  
**适用PX4版本**: v1.14+  
**关联文档**: 11-EKF2姿态估计算法流程详解.md

---

