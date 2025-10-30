# 04-PX4Gyroscope 参数系统与采样率配置详解

## 1. 问题引入

在 `fake_imu` 代码中看到：

```cpp
_sensor_interval_us = roundf(1.e6f / _px4_gyro.get_max_rate_hz());
```

**疑问**：`get_max_rate_hz()` 返回的值是怎么确定的？

---

## 2. 完整的调用链

### 2.1 从 fake_imu 开始

```cpp
// FakeImu.cpp - 构造函数
FakeImu::FakeImu() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
    _px4_accel(1310988),
    _px4_gyro(1310988)   // ← 创建 PX4Gyroscope 对象
{
    // 调用 get_max_rate_hz() 获取最大采样率
    _sensor_interval_us = roundf(1.e6f / _px4_gyro.get_max_rate_hz());
    //                                   ^^^^^^^^^^^^^^^^^^^^^^^^
    //                                   这个值是怎么来的？
}
```

---

### 2.2 PX4Gyroscope 构造函数

```cpp
// src/lib/drivers/gyroscope/PX4Gyroscope.cpp (第69-77行)

PX4Gyroscope::PX4Gyroscope(uint32_t device_id, enum Rotation rotation) :
    _device_id{device_id},
    _rotation{rotation}
{
    // advertise immediately to keep instance numbering in sync
    _sensor_pub.advertise();

    // ========== 关键！从参数系统读取 ==========
    param_get(param_find("IMU_GYRO_RATEMAX"), &_imu_gyro_rate_max);
    //                   ^^^^^^^^^^^^^^^^^
    //                   参数名称
}
```

**重要发现**：
- `_imu_gyro_rate_max` 的值来自**参数系统**
- 参数名称：`IMU_GYRO_RATEMAX`

---

### 2.3 get_max_rate_hz() 方法实现

```cpp
// src/lib/drivers/gyroscope/PX4Gyroscope.hpp (第50行)

int32_t get_max_rate_hz() const {
    return math::constrain(
        _imu_gyro_rate_max,              // 从参数读取的值
        static_cast<int32_t>(100),       // 最小值：100 Hz
        static_cast<int32_t>(4000)       // 最大值：4000 Hz
    );
}
```

**功能**：
- 返回 `_imu_gyro_rate_max` 的值
- **限制范围**：100 - 4000 Hz
- 如果参数值超出范围，会被限制到边界值

---

### 2.4 参数定义

```cpp
// src/modules/sensors/vehicle_angular_velocity/imu_gyro_parameters.c (第150行)

/**
 * Gyro control data maximum publication rate
 *
 * This is the maximum rate the gyro control data (vehicle_angular_velocity) will be
 * published for the rate controller.
 *
 * @min 100
 * @max 2000
 * @value 100 100 Hz
 * @value 250 250 Hz
 * @value 400 400 Hz (默认值)
 * @value 800 800 Hz
 * @value 1000 1 kHz
 * @value 2000 2 kHz
 * @unit Hz
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_INT32(IMU_GYRO_RATEMAX, 400);
```

**参数详情**：
- **名称**：`IMU_GYRO_RATEMAX`
- **类型**：INT32
- **默认值**：400 Hz
- **范围**：100 - 2000 Hz
- **单位**：Hz
- **重启要求**：修改后需要重启飞控

---

## 3. 完整的数据流图

```
┌────────────────────────────────────────┐
│  参数系统 (Parameter System)            │
│                                        │
│  参数定义:                              │
│  PARAM_DEFINE_INT32(IMU_GYRO_RATEMAX, 400)
│                                        │
│  文件: imu_gyro_parameters.c           │
└────────────┬───────────────────────────┘
             │
             │ 系统启动时加载
             │ 或 param set IMU_GYRO_RATEMAX 800
             │
             ▼
┌────────────────────────────────────────┐
│  PX4Gyroscope 构造函数                  │
│                                        │
│  param_get(param_find("IMU_GYRO_RATEMAX"),
│            &_imu_gyro_rate_max);      │
│                                        │
│  _imu_gyro_rate_max = 400  (默认)     │
└────────────┬───────────────────────────┘
             │
             │ 调用 get_max_rate_hz()
             │
             ▼
┌────────────────────────────────────────┐
│  get_max_rate_hz()                    │
│                                        │
│  return constrain(                     │
│      _imu_gyro_rate_max,   // 400     │
│      100,                   // min     │
│      4000                   // max     │
│  );                                    │
│                                        │
│  返回: 400 Hz                          │
└────────────┬───────────────────────────┘
             │
             │ 在 fake_imu 中使用
             │
             ▼
┌────────────────────────────────────────┐
│  FakeImu 构造函数                      │
│                                        │
│  _sensor_interval_us =                 │
│      1,000,000 / 400                   │
│    = 2500 μs                           │
│                                        │
│  每 2500 微秒 (400 Hz) 发布一次数据     │
└────────────────────────────────────────┘
```

---

## 4. 为什么是 400 Hz？

### 4.1 历史背景

**PX4 的发展过程**：

| 时期 | 典型频率 | 应用场景 | 硬件能力 |
|------|---------|---------|---------|
| **早期** (2013-) | 250 Hz | 航拍、巡航 | 低性能MCU |
| **中期** (2015-) | 400 Hz | 多旋翼、固定翼 | STM32F4系列 |
| **现代** (2020-) | 800-1000 Hz | 竞速无人机、敏捷飞行 | STM32F7/H7系列 |
| **高端** | 2000 Hz | 极限竞速 | 专用硬件 |

**400 Hz 的选择理由**：

1. **控制回路需求**：
   - 姿态控制回路（Rate Loop）需要足够快的更新率
   - 400 Hz 对于大多数应用来说是**性能和效率的平衡点**

2. **CPU 负载**：
   - 400 Hz：CPU 占用约 5-10%
   - 800 Hz：CPU 占用约 10-15%
   - 1600 Hz：CPU 占用约 20-30%

3. **延迟考虑**：
   - 400 Hz → 2.5ms 延迟
   - 对于大多数飞行器，2.5ms 的延迟是可接受的

4. **兼容性**：
   - 适用于大多数硬件平台
   - 从 STM32F4 到 H7 都能稳定运行

---

### 4.2 不同应用的推荐值

#### 航拍无人机

```bash
param set IMU_GYRO_RATEMAX 400  # 默认值即可
```

**原因**：
- 运动相对平缓
- 优先考虑稳定性和续航
- 400 Hz 完全够用

---

#### 竞速无人机

```bash
param set IMU_GYRO_RATEMAX 800  # 或 1000
```

**原因**：
- 需要极快的响应
- 高速翻滚、急转弯
- 更高频率 → 更低延迟

---

#### 固定翼

```bash
param set IMU_GYRO_RATEMAX 250  # 或 400
```

**原因**：
- 运动变化慢
- 不需要太高频率
- 省 CPU 资源

---

#### 大型多旋翼（重载）

```bash
param set IMU_GYRO_RATEMAX 400  # 标准配置
```

**原因**：
- 惯性大，响应慢
- 400 Hz 足够

---

## 5. 参数的限制与验证

### 5.1 硬件限制

虽然参数定义的范围是 100-2000 Hz，但 `get_max_rate_hz()` 会进一步限制：

```cpp
int32_t get_max_rate_hz() const {
    return math::constrain(
        _imu_gyro_rate_max,
        static_cast<int32_t>(100),   // ← 最小值
        static_cast<int32_t>(4000)   // ← 最大值
    );
}
```

**实际有效范围**：100 - 4000 Hz

**为什么最大值是 4000 Hz？**
- 超过 4000 Hz，CPU 负载过高
- IMU 硬件通常最高采样率也就 8000 Hz
- 4000 Hz 是理论上的极限值（实际很少使用）

---

### 5.2 代码中的验证

```cpp
// src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp

// IMU_GYRO_RATEMAX
int32_t imu_gyro_ratemax = _param_imu_gyro_ratemax.get();

if (imu_gyro_ratemax < 0) {
    PX4_WARN("IMU_GYRO_RATEMAX invalid (%" PRId32 "), resetting to default %" PRId32 ")",
             imu_gyro_ratemax, 400);
    imu_gyro_ratemax = 400;
}

// constrain IMU_GYRO_RATEMAX 50-10,000 Hz
imu_gyro_ratemax = math::constrain(imu_gyro_ratemax, 50, 10'000);

if (imu_gyro_ratemax != _param_imu_gyro_ratemax.get()) {
    PX4_WARN("IMU_GYRO_RATEMAX updated %" PRId32 " -> %" PRIu32,
             _param_imu_gyro_ratemax.get(), imu_gyro_ratemax);
    _param_imu_gyro_ratemax.set(imu_gyro_ratemax);
    _param_imu_gyro_ratemax.commit_no_notification();
}
```

**验证逻辑**：
1. 检查参数是否为负数
2. 限制范围：50 - 10000 Hz
3. 如果超出范围，自动修正并警告

---

## 6. 实际使用示例

### 6.1 查看当前值

```bash
# 方法1: 在 nsh 中
nsh> param show IMU_GYRO_RATEMAX

# 方法2: 在 QGroundControl 中
# 打开 Parameters -> Sensors -> IMU_GYRO_RATEMAX
```

---

### 6.2 修改参数

```bash
# 临时修改（重启后失效）
nsh> param set IMU_GYRO_RATEMAX 800

# 永久保存
nsh> param set IMU_GYRO_RATEMAX 800
nsh> param save

# 重启生效
nsh> reboot
```

---

### 6.3 在 fake_imu 中的实际计算

#### 默认值 (400 Hz)

```cpp
_px4_gyro.get_max_rate_hz() = 400

_sensor_interval_us = 1,000,000 / 400 = 2500 μs

ScheduleOnInterval(2500);  // 每 2500 微秒执行一次 Run()
```

**效果**：
- 发布频率：400 Hz
- 发布间隔：2.5 毫秒

---

#### 修改为 800 Hz

```bash
nsh> param set IMU_GYRO_RATEMAX 800
nsh> param save
nsh> reboot
nsh> fake_imu start
```

```cpp
_px4_gyro.get_max_rate_hz() = 800

_sensor_interval_us = 1,000,000 / 800 = 1250 μs

ScheduleOnInterval(1250);  // 每 1250 微秒执行一次 Run()
```

**效果**：
- 发布频率：800 Hz
- 发布间隔：1.25 毫秒

---

## 7. fake_imu 中的实际影响

### 7.1 采样点数量的变化

```cpp
void FakeImu::Run()
{
    sensor_gyro_fifo_s gyro{};

    // 根据发布频率计算采样点数
    gyro.samples = roundf(IMU_RATE_HZ / (1e6 / _sensor_interval_us));
    //                    ^^^^^^^^^^   ^^^^^^^^^^^^^^^^^^^^^^
    //                    8000 Hz      发布频率

    gyro.dt = 1e6 / IMU_RATE_HZ;  // 每个采样点的间隔
}
```

#### 默认 400 Hz

```cpp
gyro.samples = roundf(8000 / (1e6 / 2500))
             = roundf(8000 / 400)
             = 20 个采样点

gyro.dt = 1000000 / 8000 = 125 μs
```

**含义**：
- 每次 `Run()` 调用生成 **20 个采样点**
- 每个采样点间隔 **125 微秒**（8 kHz）
- 发布频率 **400 Hz**

---

#### 修改为 800 Hz

```cpp
gyro.samples = roundf(8000 / (1e6 / 1250))
             = roundf(8000 / 800)
             = 10 个采样点

gyro.dt = 1000000 / 8000 = 125 μs
```

**含义**：
- 每次 `Run()` 调用生成 **10 个采样点**
- 每个采样点间隔 **125 微秒**（不变）
- 发布频率 **800 Hz**

---

### 7.2 性能对比

| 参数值 | 发布间隔 | FIFO采样点数 | 单次数据量 | CPU占用 | 数据延迟 |
|--------|---------|------------|-----------|---------|---------|
| 100 Hz | 10 ms | 80 | 960 B | 极低 | 10 ms |
| 250 Hz | 4 ms | 32 | 384 B | 低 | 4 ms |
| **400 Hz** | **2.5 ms** | **20** | **240 B** | **中** | **2.5 ms** |
| 800 Hz | 1.25 ms | 10 | 120 B | 高 | 1.25 ms |
| 1000 Hz | 1 ms | 8 | 96 B | 很高 | 1 ms |
| 2000 Hz | 0.5 ms | 4 | 48 B | 极高 | 0.5 ms |

**说明**：
- 单次数据量 = 采样点数 × 12 字节（xyz，每个 int16）

---

## 8. 参数系统的工作原理

### 8.1 参数定义宏

```c
PARAM_DEFINE_INT32(IMU_GYRO_RATEMAX, 400);
```

**展开后**：
```c
const struct param_info_s __param__IMU_GYRO_RATEMAX __attribute__((used, section("__param"))) = {
    .name = "IMU_GYRO_RATEMAX",
    .type = PARAM_TYPE_INT32,
    .default_val.i = 400,
};
```

**作用**：
- 在编译时创建参数元数据
- 存储在特殊的内存段 `__param`
- 系统启动时自动加载

---

### 8.2 参数读取流程

```cpp
// 1. 查找参数
param_t param_handle = param_find("IMU_GYRO_RATEMAX");

// 2. 读取参数值
int32_t value;
param_get(param_handle, &value);

// 3. 使用参数
_imu_gyro_rate_max = value;
```

---

### 8.3 参数存储位置

```
┌──────────────────────────────────────┐
│  参数存储位置 (NuttX/硬件平台)         │
├──────────────────────────────────────┤
│  1. Flash 存储                        │
│     /fs/microsd/params (SD卡)        │
│     或 FRAM (快速非易失性存储器)       │
│                                      │
│  2. RAM 缓存                          │
│     系统启动时加载到内存               │
│                                      │
│  3. 默认值                            │
│     编译时嵌入代码                    │
└──────────────────────────────────────┘
```

**读取优先级**：
1. 用户保存的值（SD 卡或 FRAM）
2. 如果没有保存过，使用默认值（400）

---

## 9. 与真实 IMU 驱动的对比

### 9.1 真实 IMU（如 BMI270）

```cpp
// src/drivers/imu/bosch/bmi270/BMI270.cpp

void BMI270::RunImpl()
{
    // IMU 硬件有自己的采样率 (ODR)
    const uint16_t odr = 1600;  // BMI270: 1600 Hz

    // 但发布频率由 IMU_GYRO_RATEMAX 控制
    const uint8_t samples_per_transfer = odr / _param_imu_gyro_ratemax.get();

    // 设置 FIFO 水位标记
    const uint16_t fifo_watermark = samples_per_transfer * 12;  // 12字节/样本

    // 当 FIFO 达到水位时触发中断
    RegisterWrite(Register::FIFO_WTM_0, fifo_watermark & 0xFF);
}
```

**机制**：
- 硬件以 1600 Hz 采样（由 ODR 寄存器配置）
- 驱动以 `IMU_GYRO_RATEMAX` 频率读取 FIFO
- 每次读取多个采样点

**示例**：
- `IMU_GYRO_RATEMAX = 400`
- 硬件 ODR = 1600 Hz
- 每次读取：1600 / 400 = 4 个采样点

---

### 9.2 fake_imu

```cpp
// FakeImu.cpp

void FakeImu::Run()
{
    // 软件生成采样点
    static constexpr double IMU_RATE_HZ = 8000;  // 虚拟采样率

    // 根据发布频率计算采样点数
    gyro.samples = roundf(IMU_RATE_HZ / (1e6 / _sensor_interval_us));

    // 生成数据
    for (int n = 0; n < gyro.samples; n++) {
        gyro.x[n] = ...;  // 数学公式生成
        gyro.y[n] = ...;
        gyro.z[n] = ...;
    }
}
```

**机制**：
- 没有真实硬件，纯软件模拟
- 虚拟采样率 8000 Hz（比真实 IMU 更高）
- 根据 `IMU_GYRO_RATEMAX` 计算每次应该生成多少个点

---

## 10. 调试技巧

### 10.1 验证参数是否生效

```bash
# 1. 查看参数
nsh> param show IMU_GYRO_RATEMAX

# 2. 启动 fake_imu 并查看日志
nsh> fake_imu start
# 输出: Rate 400.000, Interval: 2500 us
#       ^^^         ^^^^
#       确认频率    确认间隔

# 3. 查看实际发布频率
nsh> listener sensor_gyro_fifo
# 观察两次消息之间的时间间隔
```

---

### 10.2 性能分析

```bash
# 查看 CPU 占用
nsh> top

# 查看陀螺仪数据更新率
nsh> uorb top | grep sensor_gyro

# 查看性能计数器
nsh> perf
```

---

### 10.3 常见问题

#### Q1: 修改参数后没有生效？

```bash
# 1. 确认已保存
nsh> param save

# 2. 必须重启
nsh> reboot

# 3. 确认参数值
nsh> param show IMU_GYRO_RATEMAX
```

---

#### Q2: CPU 占用过高？

```bash
# 降低频率
nsh> param set IMU_GYRO_RATEMAX 250
nsh> param save
nsh> reboot
```

---

#### Q3: 延迟太大？

```bash
# 提高频率
nsh> param set IMU_GYRO_RATEMAX 800
nsh> param save
nsh> reboot
```

---

## 11. 总结

### 11.1 核心要点

| 要点 | 说明 |
|------|------|
| **参数来源** | 参数系统：`IMU_GYRO_RATEMAX` |
| **默认值** | 400 Hz |
| **有效范围** | 100 - 4000 Hz（代码限制） |
| **修改方式** | `param set IMU_GYRO_RATEMAX <value>` |
| **生效条件** | 需要 `param save` + `reboot` |

---

### 11.2 完整的调用链总结

```
参数定义
  PARAM_DEFINE_INT32(IMU_GYRO_RATEMAX, 400)
         ↓
  参数系统加载（系统启动时）
         ↓
  PX4Gyroscope 构造函数
    param_get(param_find("IMU_GYRO_RATEMAX"), &_imu_gyro_rate_max)
         ↓
  get_max_rate_hz()
    return constrain(_imu_gyro_rate_max, 100, 4000)
         ↓
  fake_imu 使用
    _sensor_interval_us = 1e6 / get_max_rate_hz()
         ↓
  工作队列调度
    ScheduleOnInterval(_sensor_interval_us)
         ↓
  定期调用 Run()
    生成数据并发布
```

---

### 11.3 实际应用建议

**选择合适的频率**：

| 应用场景 | 推荐值 | 原因 |
|---------|-------|------|
| 航拍 | 250-400 Hz | 平稳、省电 |
| 一般飞行 | 400 Hz | 默认值，平衡 |
| 竞速 | 800-1000 Hz | 快速响应 |
| 极限竞速 | 1000-2000 Hz | 最低延迟 |
| 固定翼 | 250-400 Hz | 运动慢，够用 |

---

### 11.4 相关文档索引

| 文档 | 说明 |
|------|------|
| `01-fake_imu传感器模拟器代码详解.md` | fake_imu 的完整分析 |
| `02-BMI270数据结构与信号链路分析.md` | 真实 IMU 如何使用此参数 |
| `20-PX4参数系统存储机制与配置流程详解.md` | 参数系统原理 |

---

**文档版本**：v1.0
**创建日期**：2025-10-30
**适用 PX4 版本**：v1.14+
**作者**：基于 PX4Gyroscope 源码分析整理

