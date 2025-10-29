# PX4 工作队列架构与启动机制深度分析

## 概述

本文档深入解析 PX4 工作队列（Work Queue）的完整架构，包括启动流程、任务分配机制、以及为什么某些驱动会被分配到特定的工作队列。

**关键问题解答**：
1. ✅ 工作队列是如何被启动的？
2. ✅ 如何确定某个任务属于哪个工作队列？
3. ✅ 为什么 BMI088 和 BMI270 都属于 wq:SPI2？

---

## 一、工作队列架构总览

### 1.1 什么是工作队列？

工作队列（Work Queue）是 PX4 的任务调度机制，用于：
- **串行化执行**：同一工作队列中的任务按顺序执行，避免竞态条件
- **优先级管理**：不同工作队列有不同的优先级
- **资源隔离**：相关任务组织在一起（如同一 SPI 总线的驱动）
- **低延迟保证**：高优先级队列（如 rate_ctrl）确保控制回路的实时性

### 1.2 工作队列层次结构

```
┌─────────────────────────────────────────────────────────────┐
│                    wq:manager (管理器)                      │
│  - 系统启动时创建                                           │
│  - 负责创建和管理所有工作队列                               │
│  - 优先级: SCHED_PRIORITY_MAX (最高)                       │
└─────────────────────────────────────────────────────────────┘
                           │
                           ├─ 创建各个工作队列
                           │
        ┌──────────────────┼──────────────────┬──────────────┐
        │                  │                  │              │
┌───────▼─────┐  ┌─────────▼────┐  ┌─────────▼────┐  ┌──────▼─────┐
│ wq:rate_ctrl│  │   wq:SPI2    │  │   wq:INS0    │  │ wq:nav_and │
│  (最高优先) │  │  (总线驱动)  │  │  (状态估计)  │  │  _ctrl     │
│  优先级: 0  │  │  优先级: -3  │  │  优先级: -14 │  │ 优先级: -13│
└─────────────┘  └──────────────┘  └──────────────┘  └────────────┘
```

**优先级说明**：
- 优先级值越小（越接近 0），实际优先级越高
- `relative_priority` 是相对于 `SCHED_PRIORITY_MAX` 的偏移
- 实际优先级 = `SCHED_PRIORITY_MAX + relative_priority`

---

## 二、工作队列启动流程

### 2.1 系统初始化时序

```
系统启动
    ↓
┌──────────────────────────────────────────────────┐
│ 1. px4_platform_init()                           │
│    platforms/posix/src/px4/common/px4_init.cpp   │
│    第 47 行                                       │
└──────────────────────────────────────────────────┘
    ↓
┌──────────────────────────────────────────────────┐
│ 2. WorkQueueManagerStart()                       │
│    platforms/common/px4_work_queue/              │
│    WorkQueueManager.cpp:372                      │
│                                                  │
│    创建 wq:manager 任务                          │
│    - 优先级: SCHED_PRIORITY_MAX                  │
│    - 堆栈: 1280 字节                             │
└──────────────────────────────────────────────────┘
    ↓
┌──────────────────────────────────────────────────┐
│ 3. WorkQueueManagerRun()                         │
│    WorkQueueManager.cpp:258                      │
│                                                  │
│    进入主循环，等待工作队列创建请求              │
└──────────────────────────────────────────────────┘
    ↓
┌──────────────────────────────────────────────────┐
│ 4. 各模块启动时...                               │
│                                                  │
│    模块初始化 → WorkQueueFindOrCreate()         │
│                    ↓                             │
│    _wq_manager_create_queue->push(&new_wq)      │
└──────────────────────────────────────────────────┘
    ↓
┌──────────────────────────────────────────────────┐
│ 5. wq:manager 创建新线程                         │
│                                                  │
│    px4_task_spawn_cmd(wq->name, ...)            │
│         ↓                                        │
│    WorkQueueRunner(wq_config)                   │
│         ↓                                        │
│    WorkQueue::Run()  // 工作队列主循环          │
└──────────────────────────────────────────────────┘
```

---

### 2.2 关键代码分析

#### （1）系统初始化入口

```cpp
// platforms/posix/src/px4/common/px4_init.cpp:47-75
int px4_platform_init(void)
{
    hrt_init();

    // ⭐ 启动工作队列管理器
    px4::WorkQueueManagerStart();  // 第 51 行

    param_init();
    uorb_start();
    px4_log_initialize();

    return PX4_OK;
}
```

---

#### （2）启动工作队列管理器

```cpp
// platforms/common/px4_work_queue/WorkQueueManager.cpp:372-409
int WorkQueueManagerStart()
{
    if (_wq_manager_should_exit.load() && !_wq_manager_running.load()) {

        _wq_manager_should_exit.store(false);

        // ⭐ 创建 wq:manager 任务
        int task_id = px4_task_spawn_cmd("wq:manager",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_MAX,  // 最高优先级
                                         PX4_STACK_ADJUSTED(1280),
                                         (px4_main_t)&WorkQueueManagerRun,
                                         nullptr);

        if (task_id < 0) {
            PX4_ERR("task start failed (%i)", task_id);
            return -errno;
        }

        // 等待管理器初始化完成
        int max_tries = 1000;
        while (!_wq_manager_running.load() && --max_tries > 0) {
            px4_usleep(1000);
        }

        return PX4_OK;
    }

    return PX4_ERROR;
}
```

---

#### （3）工作队列管理器主循环

```cpp
// platforms/common/px4_work_queue/WorkQueueManager.cpp:258-368
static int WorkQueueManagerRun(int, char **)
{
    // 初始化工作队列列表和创建队列
    _wq_manager_wqs_list = new BlockingList<WorkQueue *>();
    _wq_manager_create_queue = new BlockingQueue<const wq_config_t *, 1>();
    _wq_manager_running.store(true);

    // ⭐ 主循环：等待工作队列创建请求
    while (!_wq_manager_should_exit.load()) {

        // 从队列中取出工作队列配置请求
        const wq_config_t *wq = _wq_manager_create_queue->pop();  // 第 266 行

        if (wq != nullptr) {
            // ⭐ 创建新的工作队列

            // 1. 计算堆栈大小
            const size_t stacksize = math::max(PTHREAD_STACK_MIN,
                                               PX4_STACK_ADJUSTED(wq->stacksize));

            // 2. 计算优先级
            int sched_priority = sched_get_priority_max(SCHED_FIFO)
                               + wq->relative_priority;  // 第 283 行

            // 3. 创建线程
            int pid = px4_task_spawn_cmd(wq->name,
                                         SCHED_FIFO,
                                         sched_priority,
                                         stacksize,
                                         WorkQueueRunner,
                                         (char *const *)arg);

            if (pid > 0) {
                PX4_DEBUG("starting: %s, priority: %d, stack: %zu bytes",
                          wq->name, sched_priority, stacksize);
            }
        }
    }

    _wq_manager_running.store(false);
    return 0;
}
```

---

#### （4）工作队列线程入口

```cpp
// platforms/common/px4_work_queue/WorkQueueManager.cpp:229-244
static void * WorkQueueRunner(void *context)
{
    wq_config_t *config = static_cast<wq_config_t *>(context);

    // ⭐ 创建 WorkQueue 对象
    WorkQueue wq(*config);

    // 添加到工作队列列表（供查找使用）
    _wq_manager_wqs_list->add(&wq);

    // ⭐ 进入工作队列主循环
    wq.Run();

    // 退出时从列表移除
    _wq_manager_wqs_list->remove(&wq);

    return nullptr;
}
```

---

### 2.3 工作队列配置

所有工作队列的配置定义在：

```cpp
// platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp:50-101

namespace px4
{

// ⭐ 工作队列配置结构
struct wq_config_t {
    const char *name;              // 工作队列名称
    uint16_t stacksize;            // 堆栈大小
    int8_t relative_priority;      // 相对优先级（相对于 SCHED_PRIORITY_MAX）
};

namespace wq_configurations
{
    // 配置值从 KConfig 读取（可在编译时配置）

    // ⭐ 角速率控制队列（最高优先级）
    static constexpr wq_config_t rate_ctrl{
        "wq:rate_ctrl",
        CONFIG_WQ_RATE_CTRL_STACKSIZE,      // 默认 3150 字节
        (int8_t)CONFIG_WQ_RATE_CTRL_PRIORITY  // 默认 0（最高）
    };

    // ⭐ SPI 总线工作队列
    static constexpr wq_config_t SPI0{"wq:SPI0", CONFIG_WQ_SPI_STACKSIZE, (int8_t)CONFIG_WQ_SPI0_PRIORITY};  // -1
    static constexpr wq_config_t SPI1{"wq:SPI1", CONFIG_WQ_SPI_STACKSIZE, (int8_t)CONFIG_WQ_SPI1_PRIORITY};  // -2
    static constexpr wq_config_t SPI2{"wq:SPI2", CONFIG_WQ_SPI_STACKSIZE, (int8_t)CONFIG_WQ_SPI2_PRIORITY};  // -3
    static constexpr wq_config_t SPI3{"wq:SPI3", CONFIG_WQ_SPI_STACKSIZE, (int8_t)CONFIG_WQ_SPI3_PRIORITY};  // -4
    // ... SPI4, SPI5, SPI6

    // ⭐ I2C 总线工作队列
    static constexpr wq_config_t I2C0{"wq:I2C0", CONFIG_WQ_I2C_STACKSIZE, (int8_t)CONFIG_WQ_I2C0_PRIORITY};  // -8
    static constexpr wq_config_t I2C1{"wq:I2C1", CONFIG_WQ_I2C_STACKSIZE, (int8_t)CONFIG_WQ_I2C1_PRIORITY};  // -9
    // ... I2C2, I2C3, I2C4

    // ⭐ 导航和控制队列
    static constexpr wq_config_t nav_and_controllers{
        "wq:nav_and_controllers",
        CONFIG_WQ_NAV_AND_CONTROLLERS_STACKSIZE,  // 默认 2240 字节
        (int8_t)CONFIG_WQ_NAV_AND_CONTROLLERS_PRIORITY  // 默认 -13
    };

    // ⭐ 惯性导航系统队列（用于 EKF2 和 VehicleIMU）
    static constexpr wq_config_t INS0{"wq:INS0", CONFIG_WQ_INS_STACKSIZE, (int8_t)CONFIG_WQ_INS0_PRIORITY};  // -14
    static constexpr wq_config_t INS1{"wq:INS1", CONFIG_WQ_INS_STACKSIZE, (int8_t)CONFIG_WQ_INS1_PRIORITY};  // -15
    static constexpr wq_config_t INS2{"wq:INS2", CONFIG_WQ_INS_STACKSIZE, (int8_t)CONFIG_WQ_INS2_PRIORITY};  // -16
    static constexpr wq_config_t INS3{"wq:INS3", CONFIG_WQ_INS_STACKSIZE, (int8_t)CONFIG_WQ_INS3_PRIORITY};  // -17

    // ⭐ 高优先级默认队列
    static constexpr wq_config_t hp_default{
        "wq:hp_default",
        CONFIG_WQ_HP_DEFAULT_STACKSIZE,  // 默认 2800 字节
        (int8_t)CONFIG_WQ_HP_DEFAULT_PRIORITY  // 默认 -18
    };

    // ⭐ 低优先级默认队列
    static constexpr wq_config_t lp_default{
        "wq:lp_default",
        CONFIG_WQ_LP_DEFAULT_STACKSIZE,  // 默认 1800 字节
        (int8_t)CONFIG_WQ_LP_DEFAULT_PRIORITY  // 默认 -50
    };

} // namespace wq_configurations

} // namespace px4
```

---

### 2.4 优先级配置文件

工作队列的优先级和堆栈大小可以通过 KConfig 配置：

```
platforms/common/px4_work_queue/Kconfig
```

**示例**：
```kconfig
config WQ_RATE_CTRL_PRIORITY
    int "Relative priority for wq:rate_ctrl"
    default 0              # 最高优先级
    range -255 0
    help
      Sets the relative priority for the rate_ctrl work queue.

config WQ_SPI2_PRIORITY
    int "Relative priority for wq:SPI2"
    default -3             # 中等优先级
    range -255 0
    help
      Sets the relative priority for the SPI2 work queue.
```

---

## 三、任务如何分配到工作队列

### 3.1 两种任务类型

#### 类型 1：SPI/I2C 设备驱动（自动分配）

**继承关系**：
```
设备驱动类
    ↓ 继承
I2CSPIDriver<T>
    ↓ 继承
I2CSPIDriverBase
    ↓ 继承
ScheduledWorkItem
    ↓ 继承
WorkItem
```

**工作队列自动分配机制**：
```cpp
// platforms/common/include/px4_platform_common/i2c_spi_buses.h:282-287

class I2CSPIDriverBase : public px4::ScheduledWorkItem, public I2CSPIInstance
{
public:
    // ⭐ 构造函数：传入工作队列配置
    I2CSPIDriverBase(const I2CSPIDriverConfig &config)
        : ScheduledWorkItem(config.module_name, config.wq_config),  // 第 286 行
          I2CSPIInstance(config)
    {}

    // ... 其他成员
};
```

**关键问题：`config.wq_config` 从哪里来？**

答案：通过 `device_bus_to_wq()` 函数自动映射！

---

#### 类型 2：控制器模块（手动指定）

**继承关系**：
```
控制器类
    ↓ 继承
ScheduledWorkItem
    ↓ 继承
WorkItem
```

**工作队列手动指定**：
```cpp
// 示例：VehicleAngularVelocity 模块

// src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.hpp:65
class VehicleAngularVelocity : public ModuleParams, public px4::ScheduledWorkItem
{
    // ...
};

// src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp:45-50
VehicleAngularVelocity::VehicleAngularVelocity() :
    ModuleParams(nullptr),
    // ⭐ 在构造函数中显式指定工作队列
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)  // 第 47 行
{
    _vehicle_angular_velocity_pub.advertise();
}
```

---

### 3.2 设备总线到工作队列的自动映射

#### 核心函数：`device_bus_to_wq()`

```cpp
// platforms/common/px4_work_queue/WorkQueueManager.cpp:123-165

const wq_config_t &
device_bus_to_wq(uint32_t device_id_int)
{
    // ⭐ 解析设备 ID
    union device::Device::DeviceId device_id;
    device_id.devid = device_id_int;

    // 提取总线类型和总线号
    const device::Device::DeviceBusType bus_type = device_id.devid_s.bus_type;
    const uint8_t bus = device_id.devid_s.bus;

    // ⭐ I2C 设备映射
    if (bus_type == device::Device::DeviceBusType_I2C) {
        switch (bus) {
            case 0: return wq_configurations::I2C0;
            case 1: return wq_configurations::I2C1;
            case 2: return wq_configurations::I2C2;
            case 3: return wq_configurations::I2C3;
            case 4: return wq_configurations::I2C4;
        }
    }

    // ⭐ SPI 设备映射
    else if (bus_type == device::Device::DeviceBusType_SPI) {
        switch (bus) {
            case 0: return wq_configurations::SPI0;
            case 1: return wq_configurations::SPI1;
            case 2: return wq_configurations::SPI2;  // ← BMI088 和 BMI270 映射到这里！
            case 3: return wq_configurations::SPI3;
            case 4: return wq_configurations::SPI4;
            case 5: return wq_configurations::SPI5;
            case 6: return wq_configurations::SPI6;
        }
    }

    // 默认：使用高优先级默认队列
    return wq_configurations::hp_default;
}
```

---

### 3.3 设备驱动初始化流程（以 BMI270 为例）

#### 完整调用栈

```
1. bmi270 start -s
        ↓
2. bmi270_main.cpp::main()
        ↓
3. I2CSPIDriver::module_start()
        ↓
4. BMI270::instantiate()
        ↓
5. new BMI270(config)
        ↓
6. I2CSPIDriverBase::I2CSPIDriverBase(config)
        ↓  调用 device_bus_to_wq(device_id)
        ↓  返回 wq_configurations::SPI2
        ↓
7. ScheduledWorkItem::ScheduledWorkItem(name, wq_config)
        ↓
8. WorkQueueFindOrCreate(wq_config)
        ↓  如果 wq:SPI2 不存在，推送创建请求
        ↓
9. wq:manager 收到请求，创建 wq:SPI2 线程
        ↓
10. BMI270::init()
        ↓
11. BMI270::Reset()
        ↓
12. ScheduleNow()  ← 触发首次运行
        ↓
13. wq:SPI2 调用 BMI270::Run()
        ↓
14. BMI270::RunImpl()
```

---

#### 关键代码示例：BMI270 驱动

```cpp
// src/drivers/imu/bosch/bmi270/BMI270.hpp:56
class BMI270 : public device::SPI, public I2CSPIDriver<BMI270>
{
    // ⭐ 继承自 I2CSPIDriver，获得工作队列机制
};

// src/drivers/imu/bosch/bmi270/BMI270.cpp:101-113
BMI270::BMI270(const I2CSPIDriverConfig &config) :
    SPI(config),                   // SPI 设备初始化
    I2CSPIDriver(config),          // ⭐ 工作队列配置传递到这里
    _drdy_gpio(config.drdy_gpio),
    _px4_accel(get_device_id(), config.rotation),
    _px4_gyro(get_device_id(), config.rotation)
{
    // 构造函数中，I2CSPIDriver 会：
    // 1. 调用 device_bus_to_wq(device_id)
    // 2. 返回 wq_configurations::SPI2（因为设备在 SPI2 总线上）
    // 3. 如果 wq:SPI2 不存在，创建它

    ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

// src/drivers/imu/bosch/bmi270/BMI270.cpp:125-138
int BMI270::init()
{
    int ret = SPI::init();
    if (ret != PX4_OK) {
        DEVICE_DEBUG("SPI::init failed (%i)", ret);
        return ret;
    }

    return Reset() ? 0 : -1;
}

bool BMI270::Reset()
{
    _state = STATE::RESET;
    DataReadyInterruptDisable();
    ScheduleClear();
    ScheduleNow();  // ⭐ 触发首次运行，wq:SPI2 会调用 Run()
    return true;
}

// src/drivers/imu/bosch/bmi270/BMI270.cpp:255-475
void BMI270::RunImpl()
{
    // ⭐ 这个函数在 wq:SPI2 线程中运行
    const hrt_abstime now = hrt_absolute_time();

    switch (_state) {
        case STATE::RESET:
            RegisterWrite(Register::CMD, CMD::softreset);
            _reset_timestamp = now;
            _state = STATE::WAIT_FOR_RESET;
            ScheduleDelayed(1_ms);  // 1ms 后再次运行
            break;

        case STATE::FIFO_READ:
            // 读取 FIFO 数据
            FIFORead(timestamp_sample, fifo_count);
            // 发布 sensor_gyro_fifo
            _px4_gyro.updateFIFO(gyro_buffer);
            break;

        // ... 其他状态
    }
}
```

---

## 四、为什么 BMI088 和 BMI270 都在 wq:SPI2？

### 4.1 硬件连接

```
┌──────────────────────────────────────┐
│   飞控板（例如 MicoAir H743）        │
│                                      │
│  ┌─────────────────────────────────┐ │
│  │        STM32H743 MCU            │ │
│  │                                 │ │
│  │  SPI1 ────┐                     │ │
│  │           │                     │ │
│  │  SPI2 ────┼─────────────────┐   │ │
│  │           │                 │   │ │
│  │           │                 │   │ │
│  └───────────┼─────────────────┼───┘ │
│              │                 │     │
│         ┌────▼────┐      ┌─────▼────┐│
│         │ BMI088  │      │ BMI270   ││
│         │ 陀螺+加速│      │ 陀螺+加速││
│         │         │      │          ││
│         └─────────┘      └──────────┘│
└──────────────────────────────────────┘
```

**关键点**：
- BMI088 和 BMI270 都物理连接到 **SPI2 总线**
- SPI 总线是共享的，同一时间只能有一个设备通信
- 因此它们**必须**在同一个工作队列中按顺序访问

---

### 4.2 设备 ID 结构

```cpp
// lib/drivers/device/Device.h

union DeviceId {
    struct {
        uint8_t  bus_type : 3;    // I2C=1, SPI=2, UAVCAN=3, etc
        uint8_t  bus : 5;         // 总线号：0-31
        uint8_t  address : 8;     // I2C地址 或 SPI片选
        uint16_t devtype : 16;    // 设备类型（如 DRV_GYR_DEVTYPE_BMI088）
    } devid_s;
    uint32_t devid;               // 完整的设备ID
};
```

**BMI088 设备 ID 示例**：
```
bus_type = 2 (SPI)
bus = 2      (SPI2)
address = 1  (片选CS1)
devtype = DRV_GYR_DEVTYPE_BMI088
```

**BMI270 设备 ID 示例**：
```
bus_type = 2 (SPI)
bus = 2      (SPI2)
address = 2  (片选CS2)
devtype = DRV_GYR_DEVTYPE_BMI270
```

**关键**：`bus = 2`，因此 `device_bus_to_wq()` 返回 `wq_configurations::SPI2`

---

### 4.3 工作队列分配流程

```
BMI088 启动
    ↓
device_id.bus = 2  →  device_bus_to_wq()  →  wq:SPI2 ──┐
                                                        │
BMI270 启动                                             │
    ↓                                                   │
device_id.bus = 2  →  device_bus_to_wq()  →  wq:SPI2 ──┤
                                                        │
                                                        ↓
                                ┌───────────────────────────────┐
                                │     wq:SPI2 工作队列          │
                                │                               │
                                │  ┌────────────────────────┐   │
                                │  │ 1) bmi088_accel        │   │
                                │  │    800.0 Hz            │   │
                                │  ├────────────────────────┤   │
                                │  │ 2) bmi088_gyro         │   │
                                │  │    666.7 Hz            │   │
                                │  ├────────────────────────┤   │
                                │  │ 3) bmi270              │   │
                                │  │    800.0 Hz            │   │
                                │  └────────────────────────┘   │
                                │                               │
                                │  ⭐ 串行执行，避免总线冲突    │
                                └───────────────────────────────┘
```

---

### 4.4 为什么必须在同一工作队列？

#### 问题：如果 BMI088 和 BMI270 在不同工作队列会怎样？

```
假设：
- BMI088 在 wq:SPI2（线程A）
- BMI270 在 wq:SPI3（线程B）

时间轴：
t=0    线程A: BMI088开始SPI传输 ───────────┐
                                         │ 同时访问！
t=5us  线程B: BMI270开始SPI传输 ───────────┤
                                         │
结果：  SPI总线冲突！数据损坏！             │
        ═════════════════════════════════╧════
```

#### 解决方案：同一工作队列串行执行

```
wq:SPI2（单线程）：

t=0     BMI088: SPI传输  [====================]
                                              ↓ 完成
t=20us  BMI270: SPI传输                      [====================]
                                                                  ↓ 完成
t=40us  BMI088: SPI传输                                          [====================]

✅ 同一时刻只有一个设备访问SPI总线
✅ 数据完整性得到保证
```

---

## 五、各类模块的工作队列分配总结

### 5.1 自动分配（SPI/I2C 设备驱动）

| 设备示例 | 总线类型 | 总线号 | 工作队列 | 优先级 |
|---------|---------|-------|---------|--------|
| BMI088 加速度计 | SPI | 2 | wq:SPI2 | -3 |
| BMI088 陀螺仪 | SPI | 2 | wq:SPI2 | -3 |
| BMI270 | SPI | 2 | wq:SPI2 | -3 |
| ICM42688P | SPI | 1 | wq:SPI1 | -2 |
| BMM150 磁力计 | I2C | 1 | wq:I2C1 | -9 |
| MS5611 气压计 | I2C | 0 | wq:I2C0 | -8 |

**规则**：
- SPI 设备 → `wq:SPI{总线号}`
- I2C 设备 → `wq:I2C{总线号}`
- 同一总线的所有设备在同一工作队列

---

### 5.2 手动指定（控制器和传感器处理模块）

| 模块 | 工作队列 | 指定位置 | 优先级 | 原因 |
|-----|---------|---------|-------|------|
| **VehicleAngularVelocity** | wq:rate_ctrl | 构造函数 | 0 | 为角速率控制提供实时数据 |
| **mc_rate_control** | wq:rate_ctrl | 构造函数 | 0 | 角速率控制内环（最高优先级） |
| **control_allocator** | wq:rate_ctrl | 构造函数 | 0 | 混控计算（内环一部分） |
| **VehicleIMU** | wq:INS{0-3} | 构造函数 | -14 ~ -17 | IMU 数据处理，每个实例对应一个 IMU |
| **ekf2** | wq:INS{0-3} | 构造函数 | -14 ~ -17 | 状态估计，每个实例对应一个 IMU |
| **mc_att_control** | wq:nav_and_ctrl | 构造函数 | -13 | 姿态控制外环 |
| **gyro_fft** | wq:hp_default | 构造函数 | -18 | FFT 频谱分析 |

---

### 5.3 代码示例：不同模块的工作队列指定

#### （1）VehicleAngularVelocity - 指定 rate_ctrl

```cpp
// src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp:45-50

VehicleAngularVelocity::VehicleAngularVelocity() :
    ModuleParams(nullptr),
    // ⭐ 显式指定 rate_ctrl 工作队列
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
    _vehicle_angular_velocity_pub.advertise();
}
```

**原因**：
- 需要以 667 Hz 的高频运行
- 直接服务于角速率控制内环
- 必须与 `mc_rate_control` 和 `control_allocator` 在同一队列

---

#### （2）mc_rate_control - 指定 rate_ctrl

```cpp
// src/modules/mc_rate_control/MulticopterRateControl.cpp:66-72

MulticopterRateControl::MulticopterRateControl(bool vtol) :
    ModuleParams(nullptr),
    // ⭐ 显式指定 rate_ctrl 工作队列
    WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
    _vehicle_angular_velocity_sub(this, ORB_ID(vehicle_angular_velocity)),
    _vtol(vtol)
{
}
```

**原因**：
- 角速率控制内环，需要最低延迟
- 订阅 `vehicle_angular_velocity`，回调触发时立即运行
- 与发布者在同一队列，避免跨队列调度延迟

---

#### （3）VehicleIMU - 指定 INS 队列

```cpp
// src/modules/sensors/vehicle_imu/VehicleIMU.cpp:64-70

VehicleIMU::VehicleIMU(uint8_t instance, uint8_t accel_index, uint8_t gyro_index) :
    ModuleParams(nullptr),
    // ⭐ 根据 IMU 实例号选择工作队列
    ScheduledWorkItem(MODULE_NAME, px4::ins_instance_to_wq(instance)),
    _instance(instance),
    _accel_calibration(accel_index),
    _gyro_calibration(gyro_index)
{
    // ...
}
```

```cpp
// platforms/common/px4_work_queue/WorkQueueManager.cpp:212-227

const wq_config_t &ins_instance_to_wq(uint8_t instance)
{
    switch (instance) {
        case 0: return wq_configurations::INS0;
        case 1: return wq_configurations::INS1;
        case 2: return wq_configurations::INS2;
        case 3: return wq_configurations::INS3;
    }

    PX4_WARN("no INS%d wq configuration, using INS0", instance);
    return wq_configurations::INS0;
}
```

**原因**：
- 每个 IMU 实例有独立的工作队列
- 支持多 IMU 冗余
- IMU0 → wq:INS0，IMU1 → wq:INS1
- EKF2 实例也在相应的 INS 队列中

---

#### （4）ekf2 - 指定 INS 队列

```cpp
// src/modules/ekf2/EKF2.cpp:67-79

EKF2::EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode) :
    ModuleParams(nullptr),
    // ⭐ EKF2 实例使用对应的 INS 工作队列
    ScheduledWorkItem(MODULE_NAME, config),
    _replay_mode(replay_mode && !multi_mode),
    _multi_mode(multi_mode),
    _instance(multi_mode ? px4_atomic_fetch_add(&_multi_instance_count, 1) : 0),
    _attitude_pub(multi_mode ? ORB_ID(estimator_attitude) : ORB_ID(vehicle_attitude))
{
    // ...
}
```

**原因**：
- EKF2 与 VehicleIMU 在同一工作队列
- 订阅 `vehicle_imu`，触发时立即运行
- 避免跨队列延迟

---

## 六、工作队列调度机制

### 6.1 调度方式

#### （1）定时调度

```cpp
// 固定间隔运行
ScheduleOnInterval(1500, 0);  // 每 1500 μs 运行一次（667 Hz）
```

#### （2）延迟调度

```cpp
// 延迟一段时间后运行
ScheduleDelayed(10_ms);  // 10 ms 后运行一次
```

#### （3）立即调度

```cpp
// 立即运行（插入到工作队列队首）
ScheduleNow();
```

#### （4）回调调度（最常用）

```cpp
// 订阅 uORB 主题，数据到达时触发回调
uORB::SubscriptionCallbackWorkItem _sensor_gyro_fifo_sub{this, ORB_ID(sensor_gyro_fifo)};

// 数据到达 → 自动调用 Run()
```

---

### 6.2 wq:rate_ctrl 的同步机制

```
┌─────────────────────────────────────────────────────────┐
│                 wq:rate_ctrl (667 Hz)                   │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  时刻 0 μs:  sensor_gyro_fifo 发布                      │
│              ↓                                          │
│  时刻 5 μs:  VehicleAngularVelocity::Run() 被触发       │
│              - 滤波处理（100 μs）                        │
│              - 发布 vehicle_angular_velocity            │
│              ↓                                          │
│  时刻 105 μs: mc_rate_control::Run() 被触发             │
│              - PID 计算（80 μs）                         │
│              - 发布 vehicle_torque_setpoint             │
│              ↓                                          │
│  时刻 185 μs: control_allocator::Run() 被触发           │
│              - 混控计算（100 μs）                        │
│              - 发布 actuator_motors                     │
│              ↓                                          │
│  时刻 285 μs: 完成！                                     │
│                                                         │
│  ✅ 总延迟：285 μs（< 1500 μs 周期）                     │
│  ✅ 无跨线程切换开销                                     │
│  ✅ 串行执行，无竞态条件                                 │
└─────────────────────────────────────────────────────────┘
```

**关键优势**：
1. **低延迟**：同一线程执行，无上下文切换
2. **可预测**：串行执行，无竞态
3. **高频**：667 Hz 稳定运行

---

## 七、调试和查看工作队列

### 7.1 查看所有工作队列

```bash
work_queue status
```

**输出示例**：
```
|__ 0) wq:manager
|__ 1) wq:rate_ctrl
|   |__ 1) control_allocator         666.8 Hz  1500 us
|   |__ 2) mc_rate_control           666.8 Hz  1500 us
|   \__ 3) vehicle_angular_velocity  666.8 Hz  1500 us
|__ 2) wq:SPI2
|   |__ 1) bmi088_accel              800.0 Hz  1250 us
|   |__ 2) bmi088_gyro               666.7 Hz  1500 us
|   \__ 3) bmi270                    800.0 Hz  1250 us
|__ 4) wq:nav_and_controllers
|   |__ 4) mc_att_control            193.1 Hz  5177 us
|__ 5) wq:INS0
|   |__ 1) ekf2                      193.1 Hz  5180 us
|   \__ 2) vehicle_imu               265.1 Hz  3772 us
|__ 6) wq:INS1
|   |__ 1) ekf2                      200.1 Hz  4997 us
|   \__ 2) vehicle_imu               213.8 Hz  4678 us
```

---

### 7.2 查看线程优先级

```bash
top
```

**输出示例**：
```
PID   PRI  USED  STACK  NAME
  1   255    0%   3648  wq:manager         ← 最高优先级
  2   255    5%   3150  wq:rate_ctrl       ← 最高优先级
  3   252    2%   2392  wq:SPI2            ← 高优先级
  4   242    3%   2240  wq:nav_and_ctrl
  5   241    4%   6000  wq:INS0
```

**优先级值**：
- 数值越大，优先级越高
- 255 = `SCHED_PRIORITY_MAX`
- wq:rate_ctrl = 255 + 0 = 255
- wq:SPI2 = 255 + (-3) = 252

---

### 7.3 查看模块所属工作队列

每个模块启动时会打印所属工作队列：

```bash
dmesg
```

**输出示例**：
```
[    0.234] INFO  [wq:manager] starting: wq:SPI2, priority: 252, stack: 2392 bytes
[    0.256] INFO  [wq:manager] starting: wq:rate_ctrl, priority: 255, stack: 3150 bytes
[    0.342] INFO  [bmi270] found BMI270 on SPI2
[    0.358] INFO  [bmi088] found BMI088 on SPI2
```

---

## 八、常见问题解答

### Q1: 为什么不是每个模块都有独立的线程？

**A**: 性能和资源考虑：

**如果每个模块独立线程**：
- 100+ 个模块 = 100+ 个线程
- 每个线程需要 2-6KB 堆栈
- 上下文切换开销巨大
- 优先级调度复杂

**使用工作队列**：
- 10-15 个工作队列线程
- 相关模块共享线程
- 串行执行，无竞态
- 优先级管理简单

---

### Q2: 如果一个模块在工作队列中执行时间过长会怎样？

**A**: 会阻塞同一工作队列中的其他模块！

**示例**：
```
wq:rate_ctrl 中有 3 个任务：
- VehicleAngularVelocity: 100 μs
- mc_rate_control: 80 μs
- control_allocator: 100 μs

如果 VehicleAngularVelocity 突然执行 1000 μs：
    ↓
mc_rate_control 和 control_allocator 被延迟 900 μs
    ↓
控制回路性能下降！
```

**因此**：
- 工作队列中的任务必须尽快完成
- 避免阻塞操作（如长时间等待）
- 使用性能计数器监控执行时间

---

### Q3: 为什么 EKF2 和 VehicleIMU 在同一工作队列？

**A**: 为了减少延迟和同步：

```
如果在不同工作队列：
    VehicleIMU (wq:INS0) 发布 vehicle_imu
        ↓  跨队列延迟 100-500 μs
    EKF2 (wq:nav) 订阅到数据

如果在同一工作队列：
    VehicleIMU (wq:INS0) 发布 vehicle_imu
        ↓  立即触发（同一线程）
    EKF2 (wq:INS0) 订阅到数据

延迟从 ~500 μs 减少到 ~10 μs
```

---

### Q4: 可以在运行时更改工作队列配置吗？

**A**: 不可以。工作队列配置是编译时确定的：

- KConfig 配置在编译时固定
- 优先级和堆栈大小不能动态改变
- 模块所属的工作队列在构造函数中确定

**如果要修改**：
1. 修改 `platforms/common/px4_work_queue/Kconfig`
2. 或修改模块构造函数中的工作队列指定
3. 重新编译

---

### Q5: 如何为自定义模块选择工作队列？

**A**: 根据模块特性选择：

| 模块类型 | 推荐工作队列 | 原因 |
|---------|-------------|------|
| **SPI/I2C 驱动** | 自动分配（wq:SPI{N}/I2C{N}） | 避免总线冲突 |
| **高频控制器（> 500 Hz）** | wq:rate_ctrl | 最高优先级，低延迟 |
| **姿态/导航控制（50-200 Hz）** | wq:nav_and_controllers | 中等优先级 |
| **传感器融合/估计器** | wq:INS{N} | 与 IMU 处理同步 |
| **非实时任务（< 50 Hz）** | wq:hp_default 或 wq:lp_default | 低优先级 |

**代码示例**：
```cpp
// 自定义控制器模块
class MyController : public px4::ScheduledWorkItem
{
public:
    MyController() :
        ScheduledWorkItem("my_controller", px4::wq_configurations::rate_ctrl)  // ← 选择 rate_ctrl
    {}

    void Run() override {
        // 控制逻辑
    }
};
```

---

## 九、总结

### 9.1 核心要点

1. **工作队列启动**：
   - 系统启动时通过 `WorkQueueManagerStart()` 创建管理器
   - 管理器按需创建工作队列线程
   - 每个工作队列是一个独立的 FIFO 调度线程

2. **任务分配机制**：
   - **SPI/I2C 驱动**：通过 `device_bus_to_wq()` 自动映射到总线对应的工作队列
   - **控制器模块**：在构造函数中显式指定工作队列
   - 同一总线的设备必须在同一工作队列，避免总线冲突

3. **BMI088 和 BMI270 同属 wq:SPI2**：
   - 因为它们都连接在 SPI2 总线上
   - `device_id.bus = 2` → `device_bus_to_wq()` → `wq:SPI2`
   - 串行执行，保证 SPI 总线的独占访问

4. **工作队列优势**：
   - 减少线程数量，节省资源
   - 相关任务串行执行，避免竞态
   - 优先级管理清晰
   - 低延迟（同一队列的任务无上下文切换）

---

### 9.2 架构图

```
┌─────────────────────────────────────────────────────────────┐
│                  PX4 工作队列架构                            │
└─────────────────────────────────────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┬────────────┐
        │                  │                  │            │
┌───────▼─────┐  ┌─────────▼────┐  ┌─────────▼────┐  ┌────▼──────┐
│ wq:rate_ctrl│  │   wq:SPI2    │  │   wq:INS0    │  │ wq:nav_and│
│  优先级: 0  │  │  优先级: -3  │  │  优先级: -14 │  │  _ctrl    │
│             │  │              │  │              │  │ 优先级:-13│
│ ┌─────────┐ │  │ ┌──────────┐ │  │ ┌──────────┐ │  │ ┌────────┐│
│ │Vehicle  │ │  │ │ bmi088   │ │  │ │ ekf2     │ │  │ │mc_att_ ││
│ │Angular  │ │  │ │ _accel   │ │  │ │          │ │  │ │control ││
│ │Velocity │ │  │ ├──────────┤ │  │ ├──────────┤ │  │ └────────┘│
│ ├─────────┤ │  │ │ bmi088   │ │  │ │ vehicle_ │ │  │           │
│ │mc_rate_ │ │  │ │ _gyro    │ │  │ │ imu      │ │  │           │
│ │control  │ │  │ ├──────────┤ │  │ └──────────┘ │  │           │
│ ├─────────┤ │  │ │ bmi270   │ │  │              │  │           │
│ │control_ │ │  │ └──────────┘ │  │              │  │           │
│ │allocator│ │  │              │  │              │  │           │
│ └─────────┘ │  │              │  │              │  │           │
│             │  │              │  │              │  │           │
│  串行执行   │  │  串行执行    │  │  串行执行    │  │  串行执行 │
│  667 Hz     │  │  800 Hz      │  │  265 Hz      │  │  193 Hz   │
└─────────────┘  └──────────────┘  └──────────────┘  └───────────┘

⭐ 每个工作队列是一个独立的线程
⭐ 同一工作队列中的任务串行执行（FIFO）
⭐ 不同工作队列之间并发执行
```

---

### 9.3 关键文件索引

| 功能 | 文件路径 | 关键函数/行号 |
|-----|---------|--------------|
| **工作队列管理器启动** | `platforms/posix/src/px4/common/px4_init.cpp` | `px4_platform_init()` (47行) |
| **工作队列管理器主循环** | `platforms/common/px4_work_queue/WorkQueueManager.cpp` | `WorkQueueManagerRun()` (258行) |
| **工作队列配置定义** | `platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp` | `wq_configurations` (50-101行) |
| **总线到工作队列映射** | `platforms/common/px4_work_queue/WorkQueueManager.cpp` | `device_bus_to_wq()` (123行) |
| **I2CSPIDriver 基类** | `platforms/common/include/px4_platform_common/i2c_spi_buses.h` | `I2CSPIDriverBase` (282行) |
| **ScheduledWorkItem 基类** | `platforms/common/include/px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp` | `ScheduledWorkItem` (43行) |
| **BMI270 驱动** | `src/drivers/imu/bosch/bmi270/BMI270.cpp` | 构造函数 (101行) |
| **VehicleAngularVelocity** | `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` | 构造函数 (45行) |

---

### 9.4 延伸阅读

- **文档 09**：IMU 到电机完整数据流分析
- **工作队列 API**：`platforms/common/include/px4_platform_common/px4_work_queue/`
- **调度机制**：`platforms/common/px4_work_queue/WorkQueue.cpp`

---

**最后更新**：2025-10-28
**作者**：PX4 架构分析

