# PX4 示例程序架构分析

本文档分析 PX4 中三种不同的示例程序实现方式：`fake_imu`、`hello` 和 `matlab_csv_serial`。

---

## 一、总览对比表

| 特性 | fake_imu | hello | matlab_csv_serial |
|------|----------|-------|-------------------|
| **基础架构** | ModuleBase + ScheduledWorkItem | 独立Task | 独立Task |
| **任务调度方式** | 工作队列(Work Queue) | 独立线程 | 独立线程 |
| **继承关系** | 继承 ModuleBase, ModuleParams, ScheduledWorkItem | 无继承，使用 AppState | 无继承 |
| **入口函数** | fake_imu_main() | hello_main() | matlab_csv_serial_main() |
| **执行函数** | Run() | PX4_MAIN + HelloExample::main() | matlab_csv_serial_thread_main() |
| **状态管理** | ModuleBase 管理 | AppState 类 | 全局变量 |
| **编程语言** | C++ | C++ | C |
| **复杂度** | 高（框架化） | 中等 | 低（简单） |

---

## 二、详细架构分析

### 2.1 fake_imu - 基于工作队列的架构

#### 核心特点
✅ **运行在工作队列中，不创建独立的任务线程**

#### 类定义
```cpp
class FakeImu : public ModuleBase<FakeImu>,
                public ModuleParams,
                public px4::ScheduledWorkItem
{
public:
    FakeImu();
    bool init();
    void Run() override;  // 由工作队列定时调用

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
};
```

#### 构造函数
```cpp
FakeImu::FakeImu() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),  // 注册到高优先级工作队列
    _px4_accel(1310988),
    _px4_gyro(1310988)
{
    _sensor_interval_us = roundf(1.e6f / _px4_gyro.get_max_rate_hz());
}
```

#### 初始化和调度
```cpp
bool FakeImu::init()
{
    ScheduleOnInterval(_sensor_interval_us);  // 设置定时调度间隔
    return true;
}
```

#### 任务生成
```cpp
int FakeImu::task_spawn(int argc, char *argv[])
{
    FakeImu *instance = new FakeImu();
    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;  // 关键！标记为工作队列任务
        if (instance->init()) {
            return PX4_OK;
        }
    }
    return PX4_ERROR;
}
```

#### 执行逻辑
```cpp
void FakeImu::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // 生成IMU数据
    sensor_gyro_fifo_s gyro{};
    // ... 数据处理 ...
    _px4_gyro.updateFIFO(gyro);
}
```

#### CMakeLists.txt
```cmake
px4_add_module(
    MODULE modules__fake_imu
    MAIN fake_imu                    # 暴露 fake_imu_main 函数到NSH
    SRCS
        FakeImu.cpp
        FakeImu.hpp
    DEPENDS
        drivers_accelerometer
        drivers_gyroscope
        px4_work_queue               # 依赖工作队列
)
```

#### 工作流程
```
nsh> fake_imu start
  ↓
fake_imu_main(argc, argv)
  ↓
FakeImu::main(argc, argv)  [ModuleBase提供]
  ↓
FakeImu::task_spawn()
  ↓
new FakeImu() → 注册到工作队列
  ↓
init() → ScheduleOnInterval()
  ↓
[工作队列定时调用] → Run() → Run() → Run() ...
```

---

### 2.2 hello - 基于独立Task的架构（面向对象）

#### 核心特点
✅ **使用 px4_task_spawn_cmd() 创建独立任务**
✅ **面向对象设计，使用 AppState 管理状态**

#### 文件结构
- `hello_main.cpp` - PX4_MAIN入口，实际执行逻辑
- `hello_start.cpp` - hello_main命令处理，创建任务
- `hello_example.cpp` - HelloExample类实现
- `hello_example.h` - HelloExample类定义

#### 类定义
```cpp
class HelloExample
{
public:
    static px4::AppState appState;  // 静态状态管理对象
    int main();
};
```

#### 命令入口（hello_start.cpp）
```cpp
extern "C" __EXPORT int hello_main(int argc, char *argv[])
{
    if (!strcmp(argv[1], "start")) {
        if (HelloExample::appState.isRunning()) {
            PX4_INFO("already running\n");
            return 0;
        }

        // 创建独立任务
        daemon_task = px4_task_spawn_cmd(
            "hello",                    // 任务名
            SCHED_DEFAULT,              // 调度策略
            SCHED_PRIORITY_MAX - 5,     // 优先级
            2000,                       // 栈大小
            PX4_MAIN,                   // 任务函数（实际是hello_main.cpp中的函数）
            (argv) ? (char *const *)&argv[2] : nullptr
        );
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        HelloExample::appState.requestExit();
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        // 检查状态
    }
}
```

#### 任务执行函数（hello_main.cpp）
```cpp
int PX4_MAIN(int argc, char **argv)
{
    px4::init(argc, argv, "hello");

    printf("hello\n");
    HelloExample hello;
    hello.main();  // 调用实际工作函数

    printf("goodbye\n");
    return 0;
}
```

#### 实际工作逻辑（hello_example.cpp）
```cpp
px4::AppState HelloExample::appState;

int HelloExample::main()
{
    appState.setRunning(true);

    int i = 0;
    while (!appState.exitRequested() && i < 5) {
        px4_sleep(2);
        printf("  Doing work...\n");
        ++i;
    }

    appState.setRunning(false);
    return 0;
}
```

#### CMakeLists.txt
```cmake
px4_add_module(
    MODULE examples__hello
    MAIN hello                       # 暴露 hello_main 函数到NSH
    SRCS
        hello_main.cpp
        hello_start.cpp
        hello_example.cpp
    DEPENDS
)
```

#### 工作流程
```
nsh> hello start
  ↓
hello_main(argc, argv)  [hello_start.cpp]
  ↓
px4_task_spawn_cmd("hello", ..., PX4_MAIN, ...)
  ↓
[创建新线程]
  ↓
PX4_MAIN()  [hello_main.cpp]
  ↓
HelloExample hello;
hello.main()
  ↓
[独立线程运行，每2秒执行一次工作]
```

---

### 2.3 matlab_csv_serial - 基于独立Task的架构（过程式）

#### 核心特点
✅ **使用 px4_task_spawn_cmd() 创建独立任务**
✅ **纯C实现，过程式编程**
✅ **使用全局变量管理状态**

#### 全局变量
```c
static bool thread_should_exit = false;  // 退出标志
static bool thread_running = false;      // 运行状态标志
static int daemon_task;                  // 任务句柄
```

#### 函数声明
```c
__EXPORT int matlab_csv_serial_main(int argc, char *argv[]);
int matlab_csv_serial_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
```

#### 命令入口
```c
int matlab_csv_serial_main(int argc, char *argv[])
{
    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("already running\n");
            exit(0);
        }

        thread_should_exit = false;

        // 创建独立任务
        daemon_task = px4_task_spawn_cmd(
            "matlab_csv_serial",                  // 任务名
            SCHED_DEFAULT,                        // 调度策略
            SCHED_PRIORITY_MAX - 5,               // 优先级
            2000,                                 // 栈大小
            matlab_csv_serial_thread_main,        // 任务函数（直接是工作函数）
            (argv) ? (char *const *)&argv[2] : (char *const *)NULL
        );
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("running");
        } else {
            warnx("stopped");
        }
        exit(0);
    }
}
```

#### 任务执行函数
```c
int matlab_csv_serial_thread_main(int argc, char *argv[])
{
    // 参数解析
    const char *uart_name = argv[1];

    // 打开串口
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    // 配置串口
    struct termios uart_config;
    // ... 串口配置 ...

    // 订阅uORB主题
    int accel0_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
    int accel1_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 1);
    int gyro0_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 0);
    int gyro1_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 1);

    thread_running = true;

    // 主循环
    while (!thread_should_exit) {
        struct pollfd fds[] = {
            { .fd = accel0_sub, .events = POLLIN }
        };

        int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 500);

        if (ret > 0 && (fds[0].revents & POLLIN)) {
            // 读取传感器数据
            orb_copy(ORB_ID(sensor_accel), accel0_sub, &accel0);
            // ... 读取其他传感器 ...

            // 写入串口（CSV格式）
            dprintf(serial_fd, "%"PRId64",%d,%d,%d,%d,%d,%d\n",
                    accel0.timestamp, (int)accel0.x, (int)accel0.y, (int)accel0.z,
                    (int)accel1.x, (int)accel1.y, (int)accel1.z);
        }
    }

    thread_running = false;
    return 0;
}
```

#### CMakeLists.txt
```cmake
px4_add_module(
    MODULE examples__matlab_csv_serial
    MAIN matlab_csv_serial           # 暴露 matlab_csv_serial_main 函数到NSH
    SRCS
        matlab_csv_serial.c
    DEPENDS
)
```

#### 工作流程
```
nsh> matlab_csv_serial start /dev/ttyS1
  ↓
matlab_csv_serial_main(argc, argv)
  ↓
px4_task_spawn_cmd("matlab_csv_serial", ..., matlab_csv_serial_thread_main, ...)
  ↓
[创建新线程]
  ↓
matlab_csv_serial_thread_main()
  ↓
打开串口 → 订阅uORB主题 → 进入主循环
  ↓
while (!thread_should_exit) {
    poll等待传感器数据 → 读取数据 → 写入串口
}
```

---

## 三、关键差异分析

### 3.1 任务创建方式

#### fake_imu - 工作队列
- **不创建独立线程/任务**
- 在现有工作队列中调度执行
- 通过 `ScheduledWorkItem` 基类实现
- `task_id = task_id_is_work_queue` 标记为工作队列
- 优点：轻量级，资源占用少，适合周期性任务
- 缺点：不能执行阻塞操作（如长时间等待、sleep等）

#### hello & matlab_csv_serial - 独立任务
- **使用 `px4_task_spawn_cmd()` 创建独立线程**
- 拥有自己的栈空间和调度优先级
- 可以执行阻塞操作
- 优点：灵活性高，可以使用sleep、poll等阻塞调用
- 缺点：资源占用相对较大

### 3.2 start 命令的本质区别

#### fake_imu 的 start
```cpp
FakeImu::main(argc, argv)          // ModuleBase 提供的统一接口
  ↓
解析 "start" 命令
  ↓
FakeImu::task_spawn()
  ↓
new FakeImu()                      // 创建对象
  ↓
instance->init()
  ↓
ScheduleOnInterval()               // 注册到工作队列
  ↓
返回，main函数结束                 // 没有创建新任务！
  ↓
[之后] wq定时调用 Run()            // 由工作队列调度
```

**关键点：**
- `start` 命令执行完就返回了
- 没有创建新的任务线程
- 只是把对象注册到了工作队列
- `_task_id = task_id_is_work_queue` 是特殊值，表示使用工作队列

#### hello 的 start
```cpp
hello_main("start")
  ↓
检查未运行
  ↓
daemon_task = px4_task_spawn_cmd(..., PX4_MAIN, ...)  // 创建新任务
  ↓
返回
  ↓
[新任务独立运行]
PX4_MAIN() → HelloExample::main()  // 在新任务中执行
```

**关键点：**
- `start` 命令会创建一个新的任务线程
- 新任务在独立的上下文中运行
- 可以使用 `px4_sleep()` 等阻塞函数
- 任务有自己的栈和优先级

#### matlab_csv_serial 的 start
```cpp
matlab_csv_serial_main("start")
  ↓
检查未运行
  ↓
daemon_task = px4_task_spawn_cmd(..., matlab_csv_serial_thread_main, ...)
  ↓
返回
  ↓
[新任务独立运行]
matlab_csv_serial_thread_main()    // 在新任务中执行
```

**关键点：**
- 与 hello 类似，创建新任务
- 直接传入工作函数，更简洁
- 可以使用 `poll()` 等待数据

### 3.3 状态管理方式

| 方式 | fake_imu | hello | matlab_csv_serial |
|------|----------|-------|-------------------|
| **状态管理** | ModuleBase 内部管理 | AppState 类 | 全局变量 |
| **退出检查** | `should_exit()` | `appState.exitRequested()` | `thread_should_exit` |
| **运行状态** | ModuleBase 管理 | `appState.isRunning()` | `thread_running` |
| **线程安全** | ✅ 框架保证 | ✅ AppState 封装 | ⚠️ 需要注意 |

### 3.4 适用场景

#### fake_imu (工作队列方式)
**适合：**
- 周期性的传感器数据生成/处理
- 高频率执行（如1kHz, 8kHz）
- 不需要阻塞等待
- 需要精确的定时控制

**不适合：**
- 需要长时间等待（如串口读取）
- 需要阻塞操作
- 不规则的事件驱动

#### hello (独立任务 - 面向对象)
**适合：**
- 需要复杂状态管理
- 周期性但频率不高的任务
- 需要与其他类交互
- 代码复用性要求高

**不适合：**
- 简单的功能实现（过度设计）
- 高频率执行（开销较大）

#### matlab_csv_serial (独立任务 - 过程式)
**适合：**
- 简单的串口通信
- 事件驱动（poll/select）
- 快速原型开发
- 一次性工具程序

**不适合：**
- 复杂的状态管理
- 需要代码复用

---

## 四、CMakeLists.txt 统一点

尽管实现方式不同，但所有三个模块的 CMakeLists.txt 都使用相同的模板：

```cmake
px4_add_module(
    MODULE <module_name>
    MAIN <command_name>        # 这会暴露 <command_name>_main 函数
    SRCS
        <source_files>
    DEPENDS
        <dependencies>
)
```

**关键参数 `MAIN`：**
- 生成的入口函数名 = `MAIN参数值_main`
- fake_imu: `MAIN fake_imu` → `fake_imu_main()`
- hello: `MAIN hello` → `hello_main()`
- matlab_csv_serial: `MAIN matlab_csv_serial` → `matlab_csv_serial_main()`

**这个 main 函数：**
1. 会被注册到 NSH 命令系统
2. 在 NSH 中输入命令名就会调用这个函数
3. 函数内部决定如何处理 start/stop/status 等参数

---

## 五、如何选择实现方式

### 决策树

```
需要阻塞操作？（串口读取、长时间等待）
├─ 是 → 使用独立任务方式
│       ├─ 代码简单？
│       │   ├─ 是 → matlab_csv_serial 方式（过程式C）
│       │   └─ 否 → hello 方式（面向对象C++）
│       └─
└─ 否 → 需要高频执行？（>100Hz）
        ├─ 是 → fake_imu 方式（工作队列）
        └─ 否 → 两者都可
                ├─ 需要框架支持 → fake_imu 方式
                └─ 简单实现 → 独立任务方式
```

### 推荐实践

| 场景 | 推荐方式 | 理由 |
|------|---------|------|
| **驱动开发** | fake_imu (工作队列) | 高性能，低延迟，框架完整 |
| **控制器** | fake_imu (工作队列) | 实时性要求高，定时精确 |
| **数据记录** | matlab_csv_serial (独立任务) | 需要阻塞IO操作 |
| **通信接口** | matlab_csv_serial (独立任务) | 事件驱动，poll等待 |
| **简单测试** | hello (独立任务) | 灵活，易于调试 |
| **复杂应用** | hello (独立任务) | 面向对象，代码组织好 |

---

## 六、代码模板

### 6.1 工作队列模板（类似 fake_imu）

**MyModule.hpp:**
```cpp
#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class MyModule : public ModuleBase<MyModule>,
                 public ModuleParams,
                 public px4::ScheduledWorkItem
{
public:
    MyModule();
    ~MyModule() override = default;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;  // 工作队列会定时调用这个函数

    // 你的成员变量
};
```

**MyModule.cpp:**
```cpp
#include "MyModule.hpp"

MyModule::MyModule() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
    // 初始化
}

bool MyModule::init()
{
    ScheduleOnInterval(10000);  // 每10ms执行一次
    return true;
}

void MyModule::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // 你的周期性工作
}

int MyModule::task_spawn(int argc, char *argv[])
{
    MyModule *instance = new MyModule();
    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;
        if (instance->init()) {
            return PX4_OK;
        }
    }
    delete instance;
    _object.store(nullptr);
    _task_id = -1;
    return PX4_ERROR;
}

int MyModule::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int MyModule::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }
    PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
你的模块描述
)DESCR_STR");
    PRINT_MODULE_USAGE_NAME("my_module", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int my_module_main(int argc, char *argv[])
{
    return MyModule::main(argc, argv);
}
```

**CMakeLists.txt:**
```cmake
px4_add_module(
    MODULE modules__my_module
    MAIN my_module
    SRCS
        MyModule.cpp
        MyModule.hpp
    DEPENDS
        px4_work_queue
)
```

### 6.2 独立任务模板（类似 matlab_csv_serial）

**my_task.c:**
```c
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/log.h>
#include <unistd.h>

__EXPORT int my_task_main(int argc, char *argv[]);

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;

int my_task_thread_main(int argc, char *argv[]);

int my_task_main(int argc, char *argv[])
{
    if (argc < 2) {
        PX4_WARN("usage: my_task {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            PX4_INFO("already running");
            return 0;
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd(
            "my_task",
            SCHED_DEFAULT,
            SCHED_PRIORITY_DEFAULT,
            2000,
            my_task_thread_main,
            (argv) ? (char *const *)&argv[2] : NULL
        );
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            PX4_INFO("running");
        } else {
            PX4_INFO("stopped");
        }
        return 0;
    }

    PX4_WARN("unknown command");
    return 1;
}

int my_task_thread_main(int argc, char *argv[])
{
    PX4_INFO("my_task started");
    thread_running = true;

    while (!thread_should_exit) {
        // 你的工作逻辑
        px4_usleep(100000);  // 100ms
    }

    thread_running = false;
    PX4_INFO("my_task stopped");
    return 0;
}
```

**CMakeLists.txt:**
```cmake
px4_add_module(
    MODULE examples__my_task
    MAIN my_task
    SRCS
        my_task.c
    DEPENDS
)
```

---

## 七、总结

1. **fake_imu** 使用工作队列，适合高频周期性任务，不能阻塞
2. **hello** 和 **matlab_csv_serial** 使用独立任务，可以阻塞
3. 所有模块通过 CMakeLists.txt 中的 `MAIN` 参数暴露命令到 NSH
4. `start` 命令的实现方式不同：
   - 工作队列：注册到调度器，不创建新线程
   - 独立任务：使用 `px4_task_spawn_cmd()` 创建新线程
5. 选择哪种方式取决于：
   - 是否需要阻塞操作
   - 执行频率要求
   - 代码复杂度
   - 资源占用考虑

---

## 八、实战案例：fake_imu + matlab_csv_serial 集成

本章节展示如何将 **工作队列架构**（fake_imu）和 **独立任务架构**（matlab_csv_serial）结合使用，实现完整的数据采集与分析流程。

### 8.1 系统架构

```
工作队列架构                 独立任务架构              外部工具
┌──────────────┐          ┌─────────────────┐       ┌──────────┐
│  fake_imu    │          │ matlab_csv_      │       │   PC     │
│              │  uORB    │   serial         │串口TX │          │
│ ScheduledWI  ├─────────>│                  ├──────>│ 串口工具  │
│              │          │ px4_task_spawn   │       │          │
│ 生成Chirp    │          │                  │       │ 保存CSV  │
│ 正弦波       │          │ poll() 等待数据   │       │          │
│ 8kHz采样     │          │ 格式化输出        │       │          │
└──────────────┘          └─────────────────┘       └────┬─────┘
                                                          │
                                                          v
                                                   ┌─────────────┐
                                                   │   MATLAB    │
                                                   │  FFT/时频   │
                                                   │   分析绘图    │
                                                   └─────────────┘
```

**关键点：**
- **fake_imu**：工作队列，高效生成数据，不能阻塞
- **matlab_csv_serial**：独立任务，可以阻塞在串口写入
- **uORB**：两个模块间的解耦通信机制

### 8.2 数据流分析

#### 8.2.1 fake_imu 发布数据

```cpp
// FakeImu::Run() - 在工作队列中执行，不能阻塞
void FakeImu::Run()
{
    // 1. 生成陀螺仪数据（FIFO格式）
    sensor_gyro_fifo_s gyro{};
    gyro.timestamp_sample = hrt_absolute_time();

    for (int n = 0; n < gyro.samples; n++) {
        // 生成 Chirp 正弦波
        gyro.x[n] = roundf(A * sin(2 * M_PI * x_F * t));
        gyro.y[n] = roundf(A * sin(2 * M_PI * y_F * t));
        gyro.z[n] = roundf(A * sin(2 * M_PI * z_F * t));
    }

    // 2. 通过 PX4Gyroscope 类发布（底层使用 uORB）
    _px4_gyro.updateFIFO(gyro);  // 非阻塞，立即返回

    // 3. 更新加速度计数据
    _px4_accel.update(timestamp, x_freq, y_freq, z_freq);  // 非阻塞

    // Run() 函数结束，工作队列继续调度其他任务
}
```

**特点：**
- 整个 `Run()` 函数执行时间很短（< 100μs）
- 不能有任何阻塞操作（sleep、poll、串口写入等）
- 数据发布后立即返回，由 uORB 负责分发

#### 8.2.2 matlab_csv_serial 订阅数据

```c
// matlab_csv_serial_thread_main() - 在独立任务中执行，可以阻塞
int matlab_csv_serial_thread_main(int argc, char *argv[])
{
    // 1. 打开串口（可能阻塞）
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    // 2. 订阅 uORB 主题
    int accel_sub = orb_subscribe(ORB_ID(sensor_accel));
    int gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));

    while (!thread_should_exit) {
        // 3. 阻塞等待数据（最多 500ms）
        struct pollfd fds[] = {
            { .fd = accel_sub, .events = POLLIN },
            { .fd = gyro_sub, .events = POLLIN }
        };

        int ret = poll(fds, 2, 500);  // 阻塞！

        if (ret > 0) {
            // 4. 读取数据
            orb_copy(ORB_ID(sensor_accel), accel_sub, &accel);
            orb_copy(ORB_ID(sensor_gyro), gyro_sub, &gyro);

            // 5. 过滤 fake_imu 的数据
            if (accel.device_id == 1310988) {  // fake_imu 的设备 ID
                // 6. 写入串口（可能阻塞）
                dprintf(serial_fd, "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    timestamp, accel.x, accel.y, accel.z,
                    gyro.x, gyro.y, gyro.z);
            }
        }
    }
}
```

**特点：**
- 使用 `poll()` 阻塞等待，不浪费 CPU
- 可以执行慢速 I/O 操作（串口写入）
- 不影响 fake_imu 的实时性

### 8.3 为什么需要两种架构？

#### 如果 fake_imu 也用独立任务？

```cpp
// ❌ 错误示范：在独立任务中生成高频数据
void fake_imu_task_main()
{
    while (!exit) {
        // 生成数据
        generate_sensor_data();

        // 发布数据
        publish_data();

        // 睡眠等待下一次
        usleep(125);  // 8kHz = 每 125μs 一次
    }
}
```

**问题：**
1. **调度延迟**：`usleep()` 精度有限，实际可能是 125-200μs，时间抖动大
2. **上下文切换开销**：每次唤醒都需要任务切换
3. **资源浪费**：独立任务需要独立的栈（2KB+）

**工作队列的优势：**
- 精确的定时调度（硬件定时器驱动）
- 无上下文切换开销（在同一个工作队列线程内执行）
- 多个模块共享工作队列，节省资源

#### 如果 matlab_csv_serial 也用工作队列？

```cpp
// ❌ 错误示范：在工作队列中执行阻塞 I/O
void MatlabSerial::Run()
{
    // 读取数据
    orb_copy(ORB_ID(sensor_accel), _accel_sub, &accel);

    // ❌ 阻塞写入串口
    write(_serial_fd, buffer, size);  // 可能阻塞数百毫秒！

    // 工作队列被阻塞，其他模块无法执行！
}
```

**问题：**
1. **阻塞工作队列**：串口写入可能需要几毫秒，工作队列中的其他任务全部卡住
2. **实时性破坏**：如果工作队列中有控制器，飞行器可能失控
3. **死锁风险**：如果等待的资源被工作队列中的其他任务持有

**独立任务的优势：**
- 可以安全地执行阻塞操作
- 不影响其他模块
- 可以使用 poll/select 等待多个事件

### 8.4 完整使用流程

#### 第一步：启动 fake_imu

```bash
nsh> fake_imu start
INFO  [fake_imu] Rate 8000.000, Interval: 125 us
```

**内部发生了什么：**
```cpp
fake_imu_main("start")
  → FakeImu::main("start")           // ModuleBase 提供
  → FakeImu::task_spawn()
  → new FakeImu()
  → init()
  → ScheduleOnInterval(125)          // 注册到工作队列
  → 返回，命令结束

[之后每 125μs]
工作队列线程 → FakeImu::Run() → 生成数据 → 发布到 uORB → 返回
```

#### 第二步：启动 matlab_csv_serial

```bash
nsh> matlab_csv_serial start /dev/ttyS0
INFO  [matlab_csv_serial] opening port /dev/ttyS0
INFO  [matlab_csv_serial] Serial port configured successfully
INFO  [matlab_csv_serial] Subscribing to fake_imu sensor data...
INFO  [matlab_csv_serial] Started! Writing CSV data to serial port...
```

**内部发生了什么：**
```c
matlab_csv_serial_main("start", "/dev/ttyS0")
  → px4_task_spawn_cmd("matlab_csv_serial", ..., thread_main, ...)
  → 创建新线程
  → 返回，命令结束

[新线程中]
matlab_csv_serial_thread_main()
  → 打开串口
  → 订阅 uORB 主题
  → while (1) {
      poll() 等待数据       // 阻塞，不占用 CPU
      orb_copy() 读取数据
      write() 写入串口      // 可能阻塞，但只影响自己
    }
```

#### 第三步：PC 端采集数据

```python
# Python 脚本
import serial

ser = serial.Serial('/dev/ttyUSB0', 921600)
with open('imu_data.csv', 'w') as f:
    while True:
        line = ser.readline().decode('utf-8')
        f.write(line)
        f.flush()
```

#### 第四步：MATLAB 分析

```matlab
% MATLAB 脚本
plot_fake_imu_data('imu_data.csv');
```

### 8.5 性能分析

#### CPU 负载

| 模块 | 架构 | CPU 使用率 | 说明 |
|------|------|-----------|------|
| fake_imu | 工作队列 | ~0.5% | 每 125μs 执行 <100μs |
| matlab_csv_serial | 独立任务 | ~0.3% | 大部分时间在 poll() 阻塞 |
| 合计 | - | <1% | 对系统影响很小 |

测量方法：
```bash
nsh> top
# 查看各任务的 CPU 占用
```

#### 数据吞吐量

- **fake_imu 生成速率**：8000 Hz
- **matlab_csv_serial 输出速率**：
  - 单次数据：约 60 字节（CSV 格式）
  - 每秒：60 × 8000 = 480 KB/s = 3.84 Mbps
  - 串口波特率：921600 bps ≈ 92 KB/s

**问题：** 串口带宽不足！

**解决方案：**
1. **降采样**：不是每个数据都发送
   ```c
   static uint32_t counter = 0;
   if ((counter++ % 10) == 0) {  // 每 10 个发送 1 个
       dprintf(serial_fd, ...);
   }
   ```

2. **二进制格式**：不用 CSV，改用二进制
   ```c
   // 每条数据只需 32 字节
   write(serial_fd, &data, sizeof(data));
   ```

3. **使用更快的接口**：
   - USB 高速（480 Mbps）
   - Ethernet（100 Mbps）
   - SD 卡（20+ MB/s）

### 8.6 扩展：多模块协作

fake_imu + matlab_csv_serial 模式可以扩展到更多场景：

#### 场景 1：多传感器融合测试

```
fake_imu (工作队列)  ────┐
fake_gps (工作队列)  ────┤ uORB
fake_mag (工作队列)  ────┤
                        ↓
                    ekf2 (工作队列)
                        ↓
                  logger (独立任务) → SD卡
                  mavlink (独立任务) → 遥测
```

#### 场景 2：实时数据可视化

```
fake_imu (工作队列)
    ↓ uORB
mavlink (独立任务) → 无线电 → QGroundControl
    ↓
实时波形显示
```

#### 场景 3：闭环仿真

```
fake_imu (工作队列) → 传感器数据
    ↓ uORB
mc_att_control (工作队列) → 控制输出
    ↓ uORB
pwm_out (工作队列) → 电机信号
    ↓
simulation_task (独立任务) → 物理仿真
    ↓ uORB
更新 fake_imu 输入 (闭环)
```

### 8.7 调试技巧

#### 1. 验证数据发布

```bash
nsh> listener sensor_accel
# 应该看到 device_id: 1310988
```

#### 2. 监控数据流

```bash
nsh> uorb top
# 查看各主题的发布频率
# sensor_accel: ~8000 Hz
# sensor_gyro: ~8000 Hz
```

#### 3. 检查串口输出

```bash
# 在 PC 端
cat /dev/ttyUSB0
# 应该看到滚动的数字
```

#### 4. 性能监控

```bash
nsh> work_queue status
# 查看工作队列状态
# hp_default: 应该有 fake_imu

nsh> ps
# 查看进程列表
# 应该有 matlab_csv_serial
```

### 8.8 常见陷阱

#### ❌ 陷阱 1：在工作队列中阻塞

```cpp
// 错误：在 Run() 中 sleep
void FakeImu::Run()
{
    generate_data();
    usleep(1000);  // ❌ 阻塞工作队列！
}
```

**正确做法：**
```cpp
void FakeImu::Run()
{
    generate_data();
    // 让工作队列调度器负责定时
}

bool FakeImu::init()
{
    ScheduleOnInterval(1000);  // ✅ 通过调度器实现延迟
}
```

#### ❌ 陷阱 2：忘记过滤 device_id

```c
// 错误：订阅所有传感器
orb_copy(ORB_ID(sensor_accel), accel_sub, &accel);
dprintf(serial_fd, ...);  // 可能输出真实传感器和 fake_imu 混合数据
```

**正确做法：**
```c
orb_copy(ORB_ID(sensor_accel), accel_sub, &accel);
if (accel.device_id == 1310988) {  // ✅ 只处理 fake_imu
    dprintf(serial_fd, ...);
}
```

#### ❌ 陷阱 3：串口缓冲区溢出

```c
// 错误：没有流控
while (!exit) {
    // 生成大量数据
    dprintf(serial_fd, "%s", huge_data);  // 串口写不过来
}
```

**正确做法：**
```c
// 降采样
if ((counter++ % DECIMATION) == 0) {
    dprintf(serial_fd, ...);
}

// 或检查缓冲区
int bytes_available;
ioctl(serial_fd, FIONWRITE, &bytes_available);
if (bytes_available > THRESHOLD) {
    dprintf(serial_fd, ...);
}
```

---

## 附录：相关API

### ModuleBase 主要接口
```cpp
static int main(int argc, char *argv[]);          // 统一命令入口
static int task_spawn(int argc, char *argv[]);    // 创建模块实例
static int custom_command(int argc, char *argv[]); // 自定义命令
static int print_usage(const char *reason);       // 打印帮助
bool should_exit();                               // 检查是否应该退出
void exit_and_cleanup();                          // 退出清理
```

### ScheduledWorkItem 主要接口
```cpp
void ScheduleOnInterval(uint32_t interval_us);    // 设置定时调度
void ScheduleDelayed(uint32_t delay_us);          // 延迟调度
void ScheduleClear();                             // 清除调度
void Run() override;                              // 工作函数（需重写）
```

### px4_task_spawn_cmd 参数
```c
int px4_task_spawn_cmd(
    const char *name,           // 任务名
    int scheduler,              // 调度器类型（SCHED_DEFAULT等）
    int priority,               // 优先级
    int stack_size,             // 栈大小（字节）
    px4_main_t entry,           // 入口函数
    char *const argv[]          // 参数
);
```

### uORB 订阅和发布
```c
// 订阅
int orb_subscribe(const struct orb_metadata *meta);
int orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance);

// 复制数据
int orb_copy(const struct orb_metadata *meta, int handle, void *buffer);

// 检查更新
bool orb_check(int handle, bool *updated);

// 发布
orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data);
int orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data);
```

