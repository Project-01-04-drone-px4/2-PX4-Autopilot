# 01-fake_imu 传感器模拟器代码详解

## 1. 模块概述

### 1.1 模块作用

`fake_imu` 是 PX4 提供的一个**测试用虚拟 IMU 传感器驱动**，主要用于：

| 功能 | 说明 |
|------|------|
| **算法测试** | 无需真实硬件即可测试惯导算法（EKF2、姿态解算等） |
| **滤波器测试** | 专门用于测试动态陷波滤波器（Dynamic Notch Filter） |
| **FFT 测试** | 生成特定频率的扫频信号，验证 FFT 频谱分析功能 |
| **振动分析** | 模拟不同频率的振动，测试振动抑制算法 |
| **开发学习** | 作为示例代码，学习如何编写 IMU 驱动 |

**核心特点**：
- 🎯 生成**线性扫频（Chirp）信号**：频率随时间线性增加
- 📊 支持 FIFO 模式：一次发布多个采样点（高频数据）
- 🔧 完全软件实现：不依赖任何硬件
- ⚙️ 基于工作队列：使用 `ScheduledWorkItem` 定时任务

---

## 2. 代码结构分析

### 2.1 文件组成

```
src/examples/fake_imu/
├── FakeImu.hpp         # 类声明（头文件）
├── FakeImu.cpp         # 类实现（主逻辑）
├── CMakeLists.txt      # 编译配置
└── Kconfig             # 菜单配置
```

---

### 2.2 类继承关系

```cpp
class FakeImu : public ModuleBase<FakeImu>,     // 模块基类（提供 start/stop 等功能）
                public ModuleParams,             // 参数管理
                public px4::ScheduledWorkItem    // 定时工作队列
```

**继承分析**：

| 基类 | 作用 | 提供的功能 |
|------|------|----------|
| `ModuleBase<FakeImu>` | PX4 标准模块框架 | `start/stop/status` 命令、生命周期管理 |
| `ModuleParams` | 参数系统集成 | 动态参数读取（本例未使用） |
| `ScheduledWorkItem` | 工作队列调度 | 定时执行 `Run()` 函数 |

---

## 3. 核心成员变量详解

### 3.1 关键常量

```cpp
static constexpr double IMU_RATE_HZ = 8000;  // IMU 内部采样率：8kHz
```

**说明**：
- 这是**内部生成数据的采样率**（虚拟 IMU 的采样频率）
- 实际发布频率由 `_sensor_interval_us` 控制（约 800Hz）
- 每次发布包含 10 个采样点（FIFO 模式）

---

### 3.2 核心对象

```cpp
PX4Accelerometer _px4_accel;  // 加速度计发布器
PX4Gyroscope _px4_gyro;        // 陀螺仪发布器
```

**作用**：
- `PX4Accelerometer` 和 `PX4Gyroscope` 是 PX4 提供的**传感器抽象层**
- 自动处理：
  - ✅ 设备 ID 生成
  - ✅ 数据缩放（Scale）
  - ✅ uORB 消息发布
  - ✅ 传感器校准框架集成

**设备 ID**：
```cpp
_px4_accel(1310988)  // DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
_px4_gyro(1310988)
```
- `1310988` 是设备类型编码，表明这是模拟设备

---

### 3.3 时间与间隔

```cpp
hrt_abstime _time_start_us{0};        // 模块启动时间戳（用于计算扫频时间）
uint32_t _sensor_interval_us{1250};   // 发布间隔：1250μs = 800Hz
```

**计算逻辑**：
```cpp
_sensor_interval_us = roundf(1.e6f / _px4_gyro.get_max_rate_hz());
// 如果 max_rate_hz = 800，则：
// _sensor_interval_us = 1,000,000 / 800 = 1250 μs
```

---

## 4. 初始化流程详解

### 4.1 构造函数 `FakeImu()`

```cpp
FakeImu::FakeImu() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
    _px4_accel(1310988),
    _px4_gyro(1310988)
{
    // 1️⃣ 计算发布间隔
    _sensor_interval_us = roundf(1.e6f / _px4_gyro.get_max_rate_hz());

    // 2️⃣ 打印配置信息
    PX4_INFO("Rate %.3f, Interval: %" PRId32 " us",
             (double)_px4_gyro.get_max_rate_hz(), _sensor_interval_us);

    // 3️⃣ 配置加速度计量程（这里不重要，随意设置）
    _px4_accel.set_range(2000.f);

    // 4️⃣ 配置陀螺仪缩放因子
    _px4_gyro.set_scale(math::radians(2000.f) / (INT16_MAX - 1));
    // 量程：±2000 度/秒
    // 将原始 int16 数据转换为 rad/s
}
```

**工作队列配置**：
- `px4::wq_configurations::hp_default`：高优先级默认工作队列
- 确保 IMU 数据及时处理

---

### 4.2 初始化函数 `init()`

```cpp
bool FakeImu::init()
{
    ScheduleOnInterval(_sensor_interval_us);  // 启动定时器：每 1250μs 执行一次 Run()
    return true;
}
```

**效果**：
- 工作队列会以 **800Hz** 的频率调用 `Run()` 函数
- 每次调用发布一批 IMU 数据

---

## 5. 核心算法：Run() 函数详解

### 5.1 函数执行流程

```cpp
void FakeImu::Run()
{
    // ========== 1️⃣ 退出检查 ==========
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // ========== 2️⃣ 准备 FIFO 数据结构 ==========
    sensor_gyro_fifo_s gyro{};
    gyro.timestamp_sample = hrt_absolute_time();
    gyro.samples = roundf(IMU_RATE_HZ / (1e6 / _sensor_interval_us));  // 计算采样点数
    gyro.dt = 1e6 / IMU_RATE_HZ;  // 每个采样点的时间间隔

    // ========== 3️⃣ 扫频参数配置 ==========
    const double dt_s = 1 / IMU_RATE_HZ;  // 采样周期：1/8000 = 0.000125 秒

    // X 轴：0 - 10 Hz 扫频
    const double x_f0 = 0.0;
    const double x_f1 = 10.0;

    // Y 轴：0 - 100 Hz 扫频
    const double y_f0 = 0.0;
    const double y_f1 = 100.0;

    // Z 轴：0 - 1000 Hz 扫频
    const double z_f0 = 0.0;
    const double z_f1 = 1000.0;

    // 振幅（最大值）
    static constexpr double A = (INT16_MAX - 1);

    // ========== 4️⃣ 初始化起始时间 ==========
    if (_time_start_us == 0) {
        _time_start_us = gyro.timestamp_sample;
    }

    // ========== 5️⃣ 扫频周期：10 秒 ==========
    const double T = 10.0;

    // 当前时间（相对于启动时间）
    const double timestamp_sample_s = (gyro.timestamp_sample - _time_start_us) / 1e6;

    float x_freq = 0;
    float y_freq = 0;
    float z_freq = 0;

    // ========== 6️⃣ 生成 FIFO 数据（多个采样点） ==========
    for (int n = 0; n < gyro.samples; n++) {
        // 计算每个采样点对应的时间
        const double t = timestamp_sample_s - (gyro.samples - n - 1) * dt_s;

        // ========== 7️⃣ 线性扫频公式（Chirp） ==========
        const double x_F = x_f0 + (x_f1 - x_f0) * t / (2 * T);
        const double y_F = y_f0 + (y_f1 - y_f0) * t / (2 * T);
        const double z_F = z_f0 + (z_f1 - z_f0) * t / (2 * T);

        // ========== 8️⃣ 生成正弦波数据 ==========
        gyro.x[n] = roundf(A * sin(2 * M_PI * x_F * t));
        gyro.y[n] = roundf(A * sin(2 * M_PI * y_F * t));
        gyro.z[n] = roundf(A * sin(2 * M_PI * z_F * t));

        // ========== 9️⃣ 第一个采样点：更新加速度计数据 ==========
        if (n == 0) {
            // 计算当前瞬时频率
            x_freq = (x_f1 - x_f0) * (t / T) + x_f0;
            y_freq = (y_f1 - y_f0) * (t / T) + y_f0;
            z_freq = (z_f1 - z_f0) * (t / T) + z_f0;

            // 用频率值作为加速度计数据（仅用于测试）
            _px4_accel.update(gyro.timestamp_sample, x_freq, y_freq, z_freq);
        }
    }

    // ========== 🔟 发布陀螺仪 FIFO 数据 ==========
    _px4_gyro.updateFIFO(gyro);
}
```

---

### 5.2 关键参数计算

#### FIFO 采样点数

```cpp
gyro.samples = roundf(IMU_RATE_HZ / (1e6 / _sensor_interval_us));
            = roundf(8000 / (1000000 / 1250))
            = roundf(8000 / 800)
            = 10
```

**含义**：
- 每次 `Run()` 调用生成 **10 个采样点**
- 这 10 个点的采样率是 **8kHz**
- 发布频率是 **800Hz**

#### 采样点时间间隔

```cpp
gyro.dt = 1e6 / IMU_RATE_HZ = 1000000 / 8000 = 125 μs
```

**含义**：FIFO 中相邻采样点间隔 125 微秒

---

### 5.3 线性扫频（Chirp）原理

#### 频率计算公式

```cpp
F(t) = f0 + (f1 - f0) * t / (2 * T)
```

**参数**：
- `f0`：起始频率
- `f1`：终止频率
- `T`：扫频周期（10秒）
- `t`：当前时间

**示例（Z 轴）**：
- `f0 = 0 Hz`，`f1 = 1000 Hz`，`T = 10s`
- 在 `t = 0s` 时：`F = 0 + (1000 - 0) * 0 / 20 = 0 Hz`
- 在 `t = 5s` 时：`F = 0 + 1000 * 5 / 20 = 250 Hz`
- 在 `t = 10s` 时：`F = 0 + 1000 * 10 / 20 = 500 Hz`
- 在 `t = 20s` 时：`F = 0 + 1000 * 20 / 20 = 1000 Hz`

**注意**：公式中是 `2*T`，所以完整扫频需要 **20 秒**！

#### 信号生成公式

```cpp
signal(t) = A * sin(2 * π * F * t)
```

**参数**：
- `A = INT16_MAX - 1 = 32766`：振幅
- `F`：瞬时频率（随时间变化）
- `t`：时间

---

### 5.4 三轴扫频参数对比

| 轴 | 起始频率 | 终止频率 | 扫频范围 | 完成时间 | 应用场景 |
|----|---------|---------|---------|---------|---------|
| **X 轴** | 0 Hz | 10 Hz | 0 - 10 Hz | 20 秒 | 低频振动测试 |
| **Y 轴** | 0 Hz | 100 Hz | 0 - 100 Hz | 20 秒 | 中频振动测试 |
| **Z 轴** | 0 Hz | 1000 Hz | 0 - 1000 Hz | 20 秒 | 高频振动测试（电机频率） |

**设计意图**：
- ✅ 覆盖完整的振动频谱（0 - 1000 Hz）
- ✅ 测试 FFT 算法的频率识别能力
- ✅ 验证动态陷波滤波器在不同频率下的性能

---

## 6. 数据发布流程

### 6.1 陀螺仪数据发布

```cpp
_px4_gyro.updateFIFO(gyro);
```

**内部流程**：
1. 验证 FIFO 数据有效性
2. 应用传感器校准（旋转矩阵、偏置等）
3. 应用缩放因子（Scale）
4. 发布到 uORB 主题：`sensor_gyro_fifo`

**发布的消息结构**：
```cpp
struct sensor_gyro_fifo_s {
    uint64_t timestamp;          // 时间戳
    uint64_t timestamp_sample;   // 最后一个采样点的时间戳
    uint32_t device_id;          // 设备 ID
    float dt;                    // 采样间隔（125 μs）
    uint8_t samples;             // 采样点数（10）
    int16_t x[32];               // X 轴原始数据
    int16_t y[32];               // Y 轴原始数据
    int16_t z[32];               // Z 轴原始数据
};
```

---

### 6.2 加速度计数据发布

```cpp
_px4_accel.update(gyro.timestamp_sample, x_freq, y_freq, z_freq);
```

**特殊用法**：
- 这里并**不是真实的加速度数据**
- 而是将**当前频率值**作为加速度计数据
- 目的：方便在日志中查看当前扫频到哪个频率了

---

## 7. 可选功能：模拟 ESC 状态

### 7.1 编译开关

```cpp
#define FAKE_IMU_FAKE_ESC_STATUS  // 取消注释以启用
```

**作用**：
- 模拟电机（ESC）的 RPM 数据
- 用于测试基于电机 RPM 的动态陷波滤波器

---

### 7.2 ESC 数据生成逻辑

```cpp
#if defined(FAKE_IMU_FAKE_ESC_STATUS)

// 每 100ms 发布一次 ESC 状态
if (hrt_elapsed_time(&_esc_status_pub.get().timestamp) > 100_ms) {
    auto &esc_status = _esc_status_pub.get();
    esc_status.esc_count = 3;  // 模拟 3 个电机

    // ESC 0：RPM 跟随 X 轴频率
    if (!(timestamp_sample_s > 1.5 && timestamp_sample_s < 2.0)) {
        esc_status.esc[0].timestamp = hrt_absolute_time();
        esc_status.esc[0].esc_rpm = x_freq * 60;  // 频率转 RPM（Hz × 60）
    }

    // ESC 1：RPM 跟随 Y 轴频率
    if (!(timestamp_sample_s > 2.5 && timestamp_sample_s < 3.0)) {
        esc_status.esc[1].esc_rpm = y_freq * 60;
    }

    // ESC 2：RPM 跟随 Z 轴频率
    if (!(timestamp_sample_s > 3.5 && timestamp_sample_s < 4.0)) {
        esc_status.esc[2].esc_rpm = z_freq * 60;
    }

    // 模拟所有电机同时掉线（5.5 - 5.6 秒）
    if (timestamp_sample_s > 5.5 && timestamp_sample_s < 5.6) {
        esc_status.esc[0].esc_rpm = 0;
        esc_status.esc[1].esc_rpm = 0;
        esc_status.esc[2].esc_rpm = 0;
    }

    esc_status.timestamp = hrt_absolute_time();
    _esc_status_pub.update();
}

#endif
```

**模拟场景**：
1. **正常运行**：3 个电机的 RPM 分别跟随 X、Y、Z 轴的频率
2. **单电机掉线**：
   - ESC 0：在 1.5 - 2.0 秒掉线
   - ESC 1：在 2.5 - 3.0 秒掉线
   - ESC 2：在 3.5 - 4.0 秒掉线
3. **全部掉线**：在 5.5 - 5.6 秒所有电机 RPM = 0

**测试目的**：
- ✅ 验证陷波滤波器在电机数据丢失时的鲁棒性
- ✅ 测试 RPM 数据超时处理逻辑

---

## 8. ModuleBase 框架集成

### 8.1 任务创建

```cpp
int FakeImu::task_spawn(int argc, char *argv[])
{
    FakeImu *instance = new FakeImu();

    if (instance) {
        _object.store(instance);          // 保存实例指针
        _task_id = task_id_is_work_queue; // 标记为工作队列任务

        if (instance->init()) {
            return PX4_OK;
        }
    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;
    return PX4_ERROR;
}
```

**特点**：
- 使用 `task_id_is_work_queue` 表示这是工作队列任务
- 不创建独立线程，而是复用工作队列线程

---

### 8.2 命令行接口

```cpp
extern "C" __EXPORT int fake_imu_main(int argc, char *argv[])
{
    return FakeImu::main(argc, argv);
}
```

**使用方式**：
```bash
nsh> fake_imu start   # 启动模拟器
nsh> fake_imu stop    # 停止模拟器
nsh> fake_imu status  # 查看状态
```

---

## 9. 数据流向图

```
┌──────────────┐
│  Run() 函数   │  每 1250μs (800Hz) 执行一次
│ (工作队列)    │
└──────┬───────┘
       │
       ├─► 生成 10 个采样点（8kHz）
       │   ├─ X 轴：0 - 10 Hz 扫频
       │   ├─ Y 轴：0 - 100 Hz 扫频
       │   └─ Z 轴：0 - 1000 Hz 扫频
       │
       ├─► PX4Gyroscope::updateFIFO()
       │           │
       │           ▼
       │   ┌────────────────────┐
       │   │ sensor_gyro_fifo   │ uORB 主题
       │   └────────────────────┘
       │           │
       │           ▼
       │   ┌────────────────────┐
       │   │ VehicleIMU 模块    │ 数据融合
       │   └────────────────────┘
       │           │
       │           ▼
       │   ┌────────────────────┐
       │   │ sensor_combined    │ 融合后的 IMU 数据
       │   └────────────────────┘
       │           │
       │           ▼
       │   ┌────────────────────┐
       │   │    EKF2 模块       │ 姿态估计
       │   └────────────────────┘
       │
       └─► PX4Accelerometer::update()
                   │
                   ▼
           ┌────────────────────┐
           │  sensor_accel      │ uORB 主题
           └────────────────────┘
```

---

## 10. 应用场景详解

### 10.1 动态陷波滤波器测试

**场景**：
```bash
# 1. 启动 fake_imu
nsh> fake_imu start

# 2. 启动 gyro_fft（FFT 分析模块）
nsh> gyro_fft start

# 3. 记录日志
nsh> logger on

# 4. 等待 20 秒完成扫频

# 5. 分析日志
# - 查看 FFT 是否正确识别频率
# - 验证陷波滤波器是否跟踪频率变化
```

**预期结果**：
- FFT 模块应正确检测到 0 - 1000 Hz 的频率变化
- 陷波滤波器应动态调整陷波频率
- 滤波后的信号应大幅减小振动幅度

---

### 10.2 振动分析

**分析流程**：
1. 使用 `fake_imu` 生成已知频率的振动信号
2. 通过 FlightPlot 或 PlotJuggler 绘制频谱图
3. 验证 FFT 窗口大小、频率分辨率等参数

---

### 10.3 算法开发

**用途**：
- ✅ 开发新的滤波算法时，使用已知信号验证
- ✅ 调试 EKF2 参数时，无需真实飞行
- ✅ CI/CD 自动化测试

---

## 11. 与真实 IMU 驱动的对比

| 特性 | `fake_imu` | 真实 IMU（如 BMI270） |
|------|-----------|---------------------|
| **数据来源** | 软件生成（数学公式） | 硬件传感器（SPI/I2C） |
| **采样率** | 8kHz（虚拟） | 实际硬件限制（1-8kHz） |
| **噪声** | 无噪声（纯净信号） | 含有传感器噪声 |
| **漂移** | 无漂移 | 有温漂和时间漂移 |
| **中断** | 定时器触发 | 硬件 FIFO 中断 |
| **校准** | 不需要 | 需要工厂校准 |
| **应用** | 算法测试、学习 | 实际飞行 |

---

## 12. 编译与使用

### 12.1 编译配置

在板子配置文件中启用（如 `boards/micoair/h743/default.px4board`）：
```
CONFIG_EXAMPLES_FAKE_IMU=y
```

或使用 menuconfig：
```bash
make micoair_h743_default boardconfig
# 导航到: examples -> fake_imu
```

---

### 12.2 运行命令

```bash
# 启动
nsh> fake_imu start

# 查看状态
nsh> fake_imu status

# 查看发布的数据
nsh> listener sensor_gyro_fifo
nsh> listener sensor_accel

# 停止
nsh> fake_imu stop
```

---

## 13. 代码改进建议

### 13.1 可配置参数

当前扫频参数是硬编码的，可以改为参数：

```cpp
DEFINE_PARAMETERS(
    (ParamFloat<px4::params::FAKE_IMU_X_F0>) _x_f0,
    (ParamFloat<px4::params::FAKE_IMU_X_F1>) _x_f1,
    (ParamFloat<px4::params::FAKE_IMU_PERIOD>) _period
)
```

---

### 13.2 支持不同波形

除了正弦波，还可以添加：
- 方波（测试滤波器阶跃响应）
- 锯齿波
- 白噪声
- 脉冲信号

---

### 13.3 多设备支持

目前只模拟一个 IMU，可以扩展为多 IMU：

```cpp
PX4Accelerometer _px4_accel0;
PX4Accelerometer _px4_accel1;
PX4Gyroscope _px4_gyro0;
PX4Gyroscope _px4_gyro1;
```

---

## 14. 总结

### 14.1 核心要点

| 要点 | 说明 |
|------|------|
| **主要用途** | 测试动态陷波滤波器、FFT 算法、振动分析 |
| **信号类型** | 线性扫频（Chirp）信号 |
| **采样模式** | FIFO 模式（每次发布 10 个采样点） |
| **扫频范围** | X: 0-10Hz, Y: 0-100Hz, Z: 0-1000Hz |
| **扫频周期** | 20 秒（注意公式中是 2*T） |
| **发布频率** | 800Hz（FIFO 内部是 8kHz） |
| **基础框架** | `ModuleBase` + `ScheduledWorkItem` |

---

### 14.2 学习价值

通过学习 `fake_imu`，你可以掌握：

1. ✅ **PX4 模块开发模式**：`ModuleBase` 框架的使用
2. ✅ **工作队列调度**：`ScheduledWorkItem` 定时任务
3. ✅ **传感器抽象层**：`PX4Accelerometer`、`PX4Gyroscope` 的使用
4. ✅ **FIFO 数据处理**：高频传感器数据批量发布
5. ✅ **uORB 发布机制**：传感器数据如何流入系统
6. ✅ **数学信号生成**：Chirp 扫频算法实现
7. ✅ **测试工具开发**：如何为算法开发配套测试工具

---

### 14.3 相关文档索引

| 文档 | 说明 |
|------|------|
| `06-动态陷波滤波器配置与实现原理.md` | 本模块的主要测试对象 |
| `07-FFT动态陷波带宽计算算法详解.md` | FFT 算法与本模块配合使用 |
| `10-PX4工作队列架构与启动机制.md` | 工作队列原理 |
| `05-飞行器角速度计算Run函数源码分析.md` | 真实 IMU 数据处理流程 |

---

**文档版本**：v1.0
**创建日期**：2025-10-30
**适用 PX4 版本**：v1.14+
**作者**：基于 fake_imu 源码分析整理

