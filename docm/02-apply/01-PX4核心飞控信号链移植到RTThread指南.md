# PX4核心飞控信号链移植到RT-Thread完整指南

## 概述

本文档详细说明如何将PX4的核心飞控信号链移植到RT-Thread操作系统，基于MicoAir H743飞控板，使用单BMI270 IMU传感器，PWM电机控制。

**移植目标**：
- 最小化移植 - 只保留核心信号链
- 单IMU配置 - 仅使用BMI270
- EKF2单例模式
- PWM电机输出
- 完整的姿态和角速率控制

---

## 一、PX4端配置修改

### 1.1 修改板级配置 - 只启用BMI270

**文件**: `boards/micoair/h743/default.px4board`

**当前配置**（第25-26行）：
```ini
CONFIG_DRIVERS_IMU_BOSCH_BMI088=y
CONFIG_DRIVERS_IMU_BOSCH_BMI270=y
```

**修改为**（禁用BMI088，只保留BMI270）：
```ini
# CONFIG_DRIVERS_IMU_BOSCH_BMI088 is not set
CONFIG_DRIVERS_IMU_BOSCH_BMI270=y
```

**说明**：通过注释掉BMI088配置，编译时将不包含BMI088驱动代码。

---

### 1.2 配置EKF2单例模式

**文件**: `boards/micoair/h743/init/rc.board_defaults`

如果该文件不存在，创建它并添加以下参数：

```bash
# EKF2单例模式配置
param set SENS_IMU_MODE 1        # 单实例模式（默认值，可省略）
param set EKF2_MULTI_IMU 0       # 禁用多IMU模式（默认值，可省略）
param set EKF2_MULTI_MAG 0       # 禁用多磁力计模式（默认值，可省略）

# 传感器选择 - 强制选择BMI270
param set SENS_IMU_PRIME 1       # 如果BMI270是Instance 1
# 或
param set SENS_IMU_PRIME 0       # 如果BMI270是Instance 0
```

**参数说明**：

| 参数 | 值 | 作用 | 代码位置 |
|------|---|------|---------|
| `SENS_IMU_MODE` | 1 | 单实例模式，Sensors模块进行IMU选择 | `src/modules/sensors/sensor_params.c:246` |
| `EKF2_MULTI_IMU` | 0 | 禁用EKF2多实例 | `src/modules/ekf2/params_multi.yaml:5` |
| `EKF2_MULTI_MAG` | 0 | 禁用多磁力计EKF2 | `src/modules/ekf2/params_multi.yaml:15` |

**核心逻辑**（`src/modules/ekf2/EKF2.cpp:2773-2839`）：
```cpp
int32_t sens_imu_mode = 1;
param_get(param_find("SENS_IMU_MODE"), &sens_imu_mode);

if (sens_imu_mode == 0) {
    // 多实例模式
    multi_mode = true;
    param_get(param_find("EKF2_MULTI_IMU"), &imu_instances);
} else {
    // 单实例模式 ← 我们使用这个
    multi_mode = false;
}
```

---

### 1.3 配置PWM电机输出

**文件**: `boards/micoair/h743/init/rc.board_defaults`

添加PWM参数：

```bash
# PWM输出配置
param set PWM_MAIN_FUNC1 101     # 电机1 (Motor 1)
param set PWM_MAIN_FUNC2 102     # 电机2 (Motor 2)
param set PWM_MAIN_FUNC3 103     # 电机3 (Motor 3)
param set PWM_MAIN_FUNC4 104     # 电机4 (Motor 4)

# PWM频率配置（Timer 1-4对应不同通道组）
param set PWM_MAIN_TIM1 400      # PWM 400Hz
param set PWM_MAIN_TIM2 400      # PWM 400Hz

# PWM范围配置
param set PWM_MAIN_MIN1 1000     # 最小脉宽 1000us
param set PWM_MAIN_MIN2 1000
param set PWM_MAIN_MIN3 1000
param set PWM_MAIN_MIN4 1000

param set PWM_MAIN_MAX1 2000     # 最大脉宽 2000us
param set PWM_MAIN_MAX2 2000
param set PWM_MAIN_MAX3 2000
param set PWM_MAIN_MAX4 2000

param set PWM_MAIN_DIS1 1000     # 解锁时脉宽
param set PWM_MAIN_DIS2 1000
param set PWM_MAIN_DIS3 1000
param set PWM_MAIN_DIS4 1000

param set PWM_MAIN_FAIL1 1000    # 失效保护脉宽
param set PWM_MAIN_FAIL2 1000
param set PWM_MAIN_FAIL3 1000
param set PWM_MAIN_FAIL4 1000
```

**PWM配置说明**（`src/drivers/pwm_out/module.yaml:13-31`）：

| 参数值 | 协议 | 速率 |
|-------|------|-----|
| `400` | PWM | 400Hz |
| `200` | PWM | 200Hz |
| `100` | PWM | 100Hz |
| `-1` | OneShot | - |
| `-3` | DShot600 | 600kbit/s |
| `-4` | DShot300 | 300kbit/s |

**验证配置**：
```bash
make micoair_h743_default
# 编译成功后烧录
# 在NSH shell中执行：
param show SENS_IMU_MODE
param show EKF2_MULTI_IMU
param show PWM_MAIN_TIM1
```

---

## 二、核心信号链模块清单

基于文档 `09-IMU到电机完整数据流与控制链路.md`，需要移植的模块：

### 2.1 完整信号链

```
BMI270 驱动
    ↓ sensor_gyro_fifo, sensor_accel_fifo
VehicleIMU
    ↓ vehicle_imu
VehicleAngularVelocity (可选但推荐)
    ↓ vehicle_angular_velocity
EKF2
    ↓ vehicle_attitude, vehicle_local_position
mc_att_control (姿态控制 - 外环)
    ↓ vehicle_rates_setpoint
mc_rate_control (角速率控制 - 内环)
    ↓ vehicle_torque_setpoint, vehicle_thrust_setpoint
control_allocator (混控)
    ↓ actuator_motors
PWM输出驱动
    ↓ 电机PWM信号
```

### 2.2 模块依赖表

| 序号 | 模块名 | 源文件 | 关键函数 | 行号 | 依赖 | 优先级 |
|-----|--------|--------|---------|------|-----|--------|
| 1 | **BMI270驱动** | `src/drivers/imu/bosch/bmi270/BMI270.cpp` | `RunImpl()`, `FIFORead()` | 255, 732 | 硬件SPI | 最高 |
| 2 | **VehicleIMU** | `src/modules/sensors/vehicle_imu/VehicleIMU.cpp` | `Run()` | 169-600 | sensor_gyro_fifo | 高 |
| 3 | **VehicleAngularVelocity** | `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` | `Run()`, `FilterAngularVelocity()` | 784, 725 | sensor_gyro_fifo | 高 |
| 4 | **EKF2** | `src/modules/ekf2/EKF2.cpp`<br>`src/modules/ekf2/EKF/ekf.cpp` | `Run()`, `update()` | 435, 137 | vehicle_imu | 中 |
| 5 | **mc_att_control** | `src/modules/mc_att_control/mc_att_control_main.cpp` | `Run()` | 205-400 | vehicle_attitude | 中 |
| 6 | **mc_rate_control** | `src/modules/mc_rate_control/MulticopterRateControl.cpp` | `Run()` | 103-280 | vehicle_angular_velocity | 高 |
| 7 | **control_allocator** | `src/modules/control_allocator/ControlAllocator.cpp` | `Run()`, `allocate()` | 303, 432 | torque/thrust_setpoint | 高 |
| 8 | **PWMOut** | `src/drivers/pwm_out/PWMOut.cpp` | `Run()`, `updateOutputs()` | 34, 128 | actuator_motors | 高 |

---

## 三、详细移植步骤

### 阶段一：硬件抽象层（HAL）移植

#### 3.1 uORB通信机制移植

**PX4原理**：uORB是发布-订阅机制的消息总线

**RT-Thread实现方案**：
1. 使用RT-Thread的mailbox/messagequeue
2. 或自己实现轻量级pub-sub

**关键文件**：
```
src/modules/uORB/uORB.h                    (uORB API定义)
src/modules/uORB/uORBManager.cpp           (管理器)
src/modules/uORB/Publication.hpp           (发布者)
src/modules/uORB/Subscription.hpp          (订阅者)
```

**核心API**：
```cpp
// 发布者
orb_advertise(ORB_ID(sensor_gyro_fifo), &data);
orb_publish(ORB_ID(sensor_gyro_fifo), pub_handle, &data);

// 订阅者
int sub = orb_subscribe(ORB_ID(sensor_gyro_fifo));
orb_copy(ORB_ID(sensor_gyro_fifo), sub, &data);
bool updated = orb_check(sub, &updated);
```

**RT-Thread简化实现**：
```c
// 使用消息队列实现
struct uorb_topic {
    rt_mq_t mq;
    void *latest_data;
    rt_mutex_t mutex;
};

// 发布
void orb_publish(struct uorb_topic *topic, void *data, size_t size) {
    rt_mutex_take(topic->mutex, RT_WAITING_FOREVER);
    memcpy(topic->latest_data, data, size);
    rt_mq_send(topic->mq, data, size);
    rt_mutex_release(topic->mutex);
}

// 订阅
bool orb_copy(struct uorb_topic *topic, void *data, size_t size) {
    rt_mutex_take(topic->mutex, RT_WAITING_FOREVER);
    memcpy(data, topic->latest_data, size);
    rt_mutex_release(topic->mutex);
    return true;
}
```

---

#### 3.2 工作队列机制移植

**PX4工作队列**（`src/lib/WorkQueue/WorkQueue.hpp`）：

PX4使用不同优先级的工作队列：
- `wq:SPI2` - IMU采集
- `wq:rate_ctrl` - 角速率控制（667Hz）
- `wq:INS0` - 状态估计
- `wq:nav_and_controllers` - 姿态控制

**RT-Thread实现**：
```c
// 使用RT-Thread线程实现工作队列
struct work_queue {
    rt_thread_t thread;
    rt_mq_t work_mq;
    rt_uint8_t priority;
};

// 工作队列初始化
void work_queue_init(struct work_queue *wq, const char *name,
                     rt_uint8_t priority, rt_uint32_t stack_size) {
    wq->work_mq = rt_mq_create(name, sizeof(work_item_t), 16, RT_IPC_FLAG_FIFO);
    wq->thread = rt_thread_create(name, work_queue_thread, wq,
                                  stack_size, priority, 10);
    rt_thread_startup(wq->thread);
}

// 工作队列线程
void work_queue_thread(void *param) {
    struct work_queue *wq = (struct work_queue *)param;
    work_item_t item;

    while (1) {
        if (rt_mq_recv(wq->work_mq, &item, sizeof(item), RT_WAITING_FOREVER) == RT_EOK) {
            item.func(item.arg);  // 执行工作项
        }
    }
}
```

**优先级映射**：

| PX4工作队列 | RT-Thread优先级 | 频率 |
|------------|----------------|------|
| `wq:rate_ctrl` | 5 (最高) | 667Hz |
| `wq:SPI2` | 6 | 800Hz |
| `wq:INS0` | 10 | 265Hz |
| `wq:nav_and_controllers` | 15 | 193Hz |

---

#### 3.3 高精度时间戳

**PX4使用**（`src/drivers/drv_hrt.h`）：
```cpp
hrt_abstime timestamp = hrt_absolute_time();  // 微秒级时间戳
```

**RT-Thread实现**：
```c
// 使用systick或硬件定时器
static uint64_t _hrt_base_time = 0;

uint64_t hrt_absolute_time(void) {
    // 方案1：使用RT-Thread的tick
    return rt_tick_get() * (1000000 / RT_TICK_PER_SECOND);

    // 方案2：使用硬件定时器（推荐，更精确）
    // 需要配置一个32位递增定时器
    uint32_t timer_cnt = TIM5->CNT;  // 假设使用TIM5
    return _hrt_base_time + timer_cnt;  // 1MHz时钟
}
```

---

### 阶段二：BMI270驱动移植

#### 3.4 BMI270核心代码

**文件结构**：
```
src/drivers/imu/bosch/bmi270/
├── BMI270.cpp              ← 主驱动文件（2000行）
├── BMI270.hpp              ← 类定义
├── bmi270_main.cpp         ← 启动入口
└── Bosch_BMI270_registers.hpp  ← 寄存器定义
```

**关键函数与行号**：

| 函数 | 行号 | 作用 | 代码位置 |
|------|------|------|---------|
| `init()` | 86-252 | 初始化传感器 | `BMI270.cpp` |
| `RunImpl()` | 255-475 | 主循环（状态机） | `BMI270.cpp` |
| `Reset()` | 480-560 | 软复位 | `BMI270.cpp` |
| `Configure()` | 564-586 | 配置寄存器 | `BMI270.cpp` |
| `DataReadyInterruptCallback()` | 590-593 | 中断回调 | `BMI270.cpp` |
| `FIFORead()` | 732-850 | 读取FIFO | `BMI270.cpp` |
| `ProcessGyro()` | 697-711 | 处理陀螺仪帧 | `BMI270.cpp` |
| `ProcessAccel()` | 714-729 | 处理加速度计帧 | `BMI270.cpp` |

**核心初始化流程**（`BMI270.cpp:86-252`）：
```cpp
int BMI270::init()
{
    // 1. SPI总线初始化
    int ret = SPI::init();

    // 2. 软复位
    Reset();

    // 3. 上传配置文件（BMI270 config file）
    // 第150-180行
    write_config_file();

    // 4. 配置FIFO
    // 第200-220行
    RegisterWrite(Register::FIFO_CONFIG_0, ...);
    RegisterWrite(Register::FIFO_CONFIG_1, ...);

    // 5. 配置数据中断
    // 第230-240行
    RegisterWrite(Register::INT1_IO_CTRL, ...);

    // 6. 启动定时器
    ScheduleOnInterval(FIFO_READ_INTERVAL_US);  // 1250us

    return PX4_OK;
}
```

**FIFO读取核心代码**（`BMI270.cpp:732-850`）：
```cpp
bool BMI270::FIFORead(const hrt_abstime &timestamp_sample, uint16_t fifo_bytes)
{
    sensor_gyro_fifo_s gyro_buffer{};
    sensor_accel_fifo_s accel_buffer{};

    gyro_buffer.timestamp_sample = timestamp_sample;
    gyro_buffer.dt = FIFO_SAMPLE_DT;  // 625us (1600Hz) 或 312.5us (3200Hz)
    gyro_buffer.scale = GYRO_SCALE;   // 0.00106 rad/s per LSB

    accel_buffer.timestamp_sample = timestamp_sample;
    accel_buffer.dt = FIFO_SAMPLE_DT;
    accel_buffer.scale = ACCEL_SCALE;  // 0.00598 m/s² per LSB

    // 读取FIFO数据
    uint8_t data_buffer[FIFO_MAX_SIZE];
    transfer((uint8_t *)&cmd, 1, data_buffer, fifo_bytes + 1);

    // 解析FIFO帧
    while (fifo_buffer_index < fifo_bytes) {
        const uint8_t header = data_buffer[fifo_buffer_index];

        switch (header & 0xFC) {  // 帧类型判断
            case sensor_gyro_frame:
                ProcessGyro(&gyro_buffer, &data_buffer[fifo_buffer_index]);
                fifo_buffer_index += 7;  // 1字节header + 6字节数据
                break;

            case sensor_accel_frame:
                ProcessAccel(&accel_buffer, &data_buffer[fifo_buffer_index]);
                fifo_buffer_index += 7;
                break;

            case sensor_both_frame:
                ProcessGyro(&gyro_buffer, &data_buffer[fifo_buffer_index]);
                ProcessAccel(&accel_buffer, &data_buffer[fifo_buffer_index + 6]);
                fifo_buffer_index += 13;  // 1 + 6 + 6
                break;

            default:
                fifo_buffer_index++;
                break;
        }
    }

    // 发布FIFO数据
    if (gyro_buffer.samples > 0) {
        _px4_gyro.updateFIFO(gyro_buffer);  // 发布 sensor_gyro_fifo
    }

    if (accel_buffer.samples > 0) {
        _px4_accel.updateFIFO(accel_buffer);  // 发布 sensor_accel_fifo
    }

    return true;
}
```

**陀螺仪数据处理**（`BMI270.cpp:697-711`）：
```cpp
void BMI270::ProcessGyro(sensor_gyro_fifo_s *gyro_buffer, const uint8_t *data)
{
    const int16_t gyro_x = (data[2] << 8) | data[1];  // 小端序
    const int16_t gyro_y = (data[4] << 8) | data[3];
    const int16_t gyro_z = (data[6] << 8) | data[5];

    // 填充到FIFO缓冲区
    const int index = gyro_buffer->samples;
    gyro_buffer->x[index] = gyro_x;
    gyro_buffer->y[index] = gyro_y;
    gyro_buffer->z[index] = gyro_z;
    gyro_buffer->samples++;
}
```

**RT-Thread移植要点**：

1. **SPI驱动接口**：
   - 替换 `SPI::transfer()` 为 RT-Thread的 `rt_spi_transfer()`
   - 配置SPI速度：10MHz（初始化），20MHz（运行）

2. **中断配置**：
   ```c
   // 配置BMI270的INT1引脚连接到MCU GPIO
   rt_pin_mode(BMI270_INT1_PIN, PIN_MODE_INPUT_PULLDOWN);
   rt_pin_attach_irq(BMI270_INT1_PIN, PIN_IRQ_MODE_RISING,
                     bmi270_data_ready_callback, NULL);
   rt_pin_irq_enable(BMI270_INT1_PIN, PIN_IRQ_ENABLE);
   ```

3. **定时器触发**：
   ```c
   // 创建定时器，每1.25ms触发一次FIFO读取
   rt_timer_t fifo_timer = rt_timer_create("bmi270", bmi270_run,
                                            NULL, 1250,
                                            RT_TIMER_FLAG_PERIODIC);
   rt_timer_start(fifo_timer);
   ```

---

### 阶段三：VehicleIMU模块移植

#### 3.5 VehicleIMU核心代码

**文件**: `src/modules/sensors/vehicle_imu/VehicleIMU.cpp`

**关键函数**（`VehicleIMU.cpp:169-600`）：
```cpp
void VehicleIMU::Run()
{
    // 1. 订阅FIFO数据
    sensor_gyro_fifo_s sensor_gyro_fifo;
    sensor_accel_fifo_s sensor_accel_fifo;

    if (_sensor_gyro_fifo_sub.update(&sensor_gyro_fifo)) {

        // 2. 遍历FIFO样本
        const int N = sensor_gyro_fifo.samples;
        const float dt = sensor_gyro_fifo.dt;

        for (int n = 0; n < N; n++) {
            // 3. 转换为物理单位（rad/s 和 m/s²）
            const Vector3f gyro(
                sensor_gyro_fifo.x[n] * sensor_gyro_fifo.scale,
                sensor_gyro_fifo.y[n] * sensor_gyro_fifo.scale,
                sensor_gyro_fifo.z[n] * sensor_gyro_fifo.scale
            );

            const Vector3f accel(
                sensor_accel_fifo.x[n] * sensor_accel_fifo.scale,
                sensor_accel_fifo.y[n] * sensor_accel_fifo.scale,
                sensor_accel_fifo.z[n] * sensor_accel_fifo.scale
            );

            // 4. 积分：角速度 × dt → delta_angle
            //         加速度 × dt → delta_velocity
            _imu_down_sampler.update(gyro * dt, accel * dt);
        }

        // 5. 发布 vehicle_imu
        vehicle_imu_s imu;
        imu.timestamp = hrt_absolute_time();
        imu.timestamp_sample = sensor_gyro_fifo.timestamp_sample;
        imu.delta_angle = _imu_down_sampler.delta_angle();        // 角度增量
        imu.delta_velocity = _imu_down_sampler.delta_velocity();  // 速度增量
        imu.delta_angle_dt = _imu_down_sampler.delta_angle_dt();  // 积分时间
        _vehicle_imu_pub.publish(imu);
    }
}
```

**数据转换**：
```
sensor_gyro_fifo:
  - x[N], y[N], z[N]  (int16_t, N=2-3)
  - dt = 625us
  - scale = 0.00106 rad/s per LSB
    ↓
  gyro_rad_s = raw_value * scale
    ↓
  delta_angle = gyro_rad_s * dt
    ↓
vehicle_imu:
  - delta_angle[3]  (float, radians)
  - delta_velocity[3]  (float, m/s)
  - delta_angle_dt  (float, seconds)
```

**RT-Thread移植**：
```c
// 创建线程处理vehicle_imu
void vehicle_imu_thread(void *param) {
    struct uorb_topic *gyro_fifo_topic;
    sensor_gyro_fifo_s gyro_data;
    vehicle_imu_s imu_data;

    while (1) {
        // 等待新的FIFO数据
        if (orb_copy(gyro_fifo_topic, &gyro_data, sizeof(gyro_data))) {
            // 处理数据
            for (int i = 0; i < gyro_data.samples; i++) {
                float gyro_x = gyro_data.x[i] * gyro_data.scale;
                float gyro_y = gyro_data.y[i] * gyro_data.scale;
                float gyro_z = gyro_data.z[i] * gyro_data.scale;

                imu_data.delta_angle[0] += gyro_x * gyro_data.dt * 1e-6f;
                imu_data.delta_angle[1] += gyro_y * gyro_data.dt * 1e-6f;
                imu_data.delta_angle[2] += gyro_z * gyro_data.dt * 1e-6f;
            }

            // 发布
            orb_publish(vehicle_imu_topic, &imu_data, sizeof(imu_data));
        }

        rt_thread_mdelay(1);
    }
}
```

---

### 阶段四：VehicleAngularVelocity滤波器移植

#### 3.6 角速度滤波核心代码

**文件**: `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`

**关键函数**（`VehicleAngularVelocity.cpp:784-919`）：
```cpp
void VehicleAngularVelocity::Run()
{
    // 1. 订阅FIFO数据
    sensor_gyro_fifo_s sensor_fifo_data;

    if (_sensor_gyro_fifo_sub.update(&sensor_fifo_data)) {
        const int N = sensor_fifo_data.samples;

        // 2. 对每个轴应用滤波器
        for (int axis = 0; axis < 3; axis++) {
            // 3. 动态陷波滤波（可选）
            UpdateDynamicNotchEscRpm(timestamp);   // ESC RPM陷波
            UpdateDynamicNotchFFT(timestamp);      // FFT陷波

            // 4. 静态陷波滤波
            for (auto &notch : _notch_filters[axis]) {
                angular_velocity(axis) = notch.apply(angular_velocity(axis));
            }

            // 5. 低通滤波
            angular_velocity(axis) = _lp_filter[axis].apply(angular_velocity(axis));

            // 6. 计算角加速度（数值微分）
            angular_acceleration(axis) = (angular_velocity(axis) - _angular_velocity_prev(axis)) / dt;
        }

        // 7. 发布
        vehicle_angular_velocity_s angular_velocity_msg;
        angular_velocity_msg.xyz[0] = angular_velocity(0);
        angular_velocity_msg.xyz[1] = angular_velocity(1);
        angular_velocity_msg.xyz[2] = angular_velocity(2);
        angular_velocity_msg.xyz_derivative[0] = angular_acceleration(0);
        angular_velocity_msg.xyz_derivative[1] = angular_acceleration(1);
        angular_velocity_msg.xyz_derivative[2] = angular_acceleration(2);

        _vehicle_angular_velocity_pub.publish(angular_velocity_msg);
    }
}
```

**滤波器链**：
```
原始角速度
  ↓
动态陷波滤波（ESC RPM）  ← 可选
  ↓
动态陷波滤波（FFT）      ← 可选
  ↓
静态陷波滤波（固定频率）
  ↓
低通滤波器（截止频率80-100Hz）
  ↓
校准（零偏、旋转矩阵）
  ↓
输出：vehicle_angular_velocity
```

**低通滤波器实现**（`src/lib/mathlib/math/filter/LowPassFilter2p.hpp`）：
```cpp
float LowPassFilter2p::apply(float sample)
{
    // 二阶Butterworth低通滤波
    float delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
    float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;

    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    return output;
}
```

**RT-Thread移植建议**：
- 最小版本：只实现低通滤波器
- 完整版本：包含陷波滤波器（抑制电机振动）

---

### 阶段五：EKF2估计器移植

#### 3.7 EKF2核心代码

**文件**：
- `src/modules/ekf2/EKF2.cpp` - 主模块（2900行）
- `src/modules/ekf2/EKF/ekf.cpp` - 核心算法（3800行）

**EKF2主循环**（`EKF2.cpp:435-864`）：
```cpp
void EKF2::Run()
{
    // 1. 订阅IMU数据
    vehicle_imu_s imu;
    if (_vehicle_imu_sub.update(&imu)) {

        // 2. 填充IMU样本到缓冲区
        imuSample imu_sample;
        imu_sample.time_us = imu.timestamp_sample;
        imu_sample.delta_ang = Vector3f(imu.delta_angle);
        imu_sample.delta_vel = Vector3f(imu.delta_velocity);
        imu_sample.delta_ang_dt = imu.delta_angle_dt;
        imu_sample.delta_vel_dt = imu.delta_velocity_dt;

        _ekf.setIMUData(imu_sample);  // 推入缓冲区

        // 3. 调用EKF核心算法
        if (_ekf.update()) {

            // 4. 发布姿态
            vehicle_attitude_s att;
            _ekf.get_quat().copyTo(att.q);  // 四元数
            att.timestamp = hrt_absolute_time();
            _vehicle_attitude_pub.publish(att);

            // 5. 发布位置
            vehicle_local_position_s lpos;
            _ekf.get_position().copyTo(lpos.x);
            _ekf.get_velocity().copyTo(lpos.vx);
            _vehicle_local_position_pub.publish(lpos);
        }
    }
}
```

**EKF核心算法**（`ekf.cpp:137-196`）：
```cpp
bool Ekf::update()
{
    if (!_imu_updated) {
        return false;
    }

    // 1. 获取延迟补偿的IMU数据
    const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

    // 2. 状态预测（基于IMU积分）
    predictState(imu_sample_delayed);         // 姿态、速度、位置预测

    // 3. 协方差预测
    predictCovariance(imu_sample_delayed);    // 误差协方差传播

    // 4. 融合其他传感器（GPS, MAG, BARO等）
    controlFusionModes(imu_sample_delayed);

    // 5. 输出预测器校正
    _output_predictor.correctOutputStates(...);

    return true;
}
```

**状态预测**（`ekf.cpp:~700`）：
```cpp
void Ekf::predictState(const imuSample &imu_sample)
{
    // 1. 角速度积分 → 姿态更新
    const Vector3f delta_angle = imu_sample.delta_ang - _state.delta_ang_bias;
    const Quaternion dq = Quaternion(AxisAngle(delta_angle));
    _state.quat_nominal = _state.quat_nominal * dq;  // 四元数乘法
    _state.quat_nominal.normalize();

    // 2. 加速度积分 → 速度更新
    const Vector3f delta_vel_body = imu_sample.delta_vel - _state.delta_vel_bias;
    const Vector3f delta_vel_earth = _R_to_earth * delta_vel_body;
    _state.vel += delta_vel_earth + _state.accel_bias * imu_sample.delta_vel_dt;

    // 3. 速度积分 → 位置更新
    _state.pos += _state.vel * imu_sample.delta_vel_dt;
}
```

**RT-Thread移植策略**：

1. **简化版EKF**：
   - 只保留姿态估计（去掉位置/速度估计）
   - 不融合GPS/MAG/BARO
   - 代码量可减少到500-800行

2. **完整版EKF**：
   - 保留完整状态（24维）
   - 需要Matrix库支持（Eigen或自己实现）
   - 需要1-2MB Flash

**状态向量**（24维）：
```
[0-3]   四元数 (姿态)
[4-6]   速度 (NED)
[7-9]   位置 (NED)
[10-12] IMU delta_angle 偏差
[13-15] IMU delta_velocity 偏差
[16-18] 地磁场 (NED)
[19-21] 地磁场偏差 (Body)
[22-23] 风速 (NE)
```

**简化版状态向量**（仅姿态，10维）：
```
[0-3]   四元数 (姿态)
[4-6]   陀螺仪偏差
[7-9]   加速度计偏差
```

---

### 阶段六：姿态控制器移植

#### 3.8 mc_att_control核心代码

**文件**: `src/modules/mc_att_control/mc_att_control_main.cpp`

**主循环**（`mc_att_control_main.cpp:205-400`）：
```cpp
void MulticopterAttitudeControl::Run()
{
    // 1. 订阅当前姿态（来自EKF2）
    vehicle_attitude_s v_att;
    if (_vehicle_attitude_sub.update(&v_att)) {

        const Quatf q_current(v_att.q);  // 当前姿态四元数

        // 2. 获取姿态设定值（来自遥控器或导航）
        vehicle_attitude_setpoint_s att_sp;
        _vehicle_attitude_setpoint_sub.copy(&att_sp);
        const Quatf q_desired(att_sp.q_d);  // 期望姿态四元数

        // 3. 计算姿态误差
        const Quatf q_error = q_current.inversed() * q_desired;

        // 4. PID控制器计算角速率设定值
        Vector3f rates_sp = _attitude_control.update(q_error);  // 输出：rad/s

        // 5. 发布角速率设定值
        vehicle_rates_setpoint_s rates_setpoint;
        rates_setpoint.roll = rates_sp(0);
        rates_setpoint.pitch = rates_sp(1);
        rates_setpoint.yaw = rates_sp(2);
        _vehicle_rates_setpoint_pub.publish(rates_setpoint);
    }
}
```

**姿态PID控制器**（`src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:80-120`）：
```cpp
Vector3f AttitudeControl::update(const Quatf &q_error)
{
    // 1. 四元数误差 → 角度误差
    Vector3f angle_error = AxisAngle(q_error).axis() * AxisAngle(q_error).angle();

    // 2. P控制
    Vector3f rates_sp = angle_error.emult(_gain_p);  // element-wise乘法

    // 限幅
    rates_sp(0) = math::constrain(rates_sp(0), -_rate_limit(0), _rate_limit(0));
    rates_sp(1) = math::constrain(rates_sp(1), -_rate_limit(1), _rate_limit(1));
    rates_sp(2) = math::constrain(rates_sp(2), -_rate_limit(2), _rate_limit(2));

    return rates_sp;
}
```

**参数**：
- `MC_ROLL_P` = 6.5 (Roll P增益)
- `MC_PITCH_P` = 6.5 (Pitch P增益)
- `MC_YAW_P` = 2.8 (Yaw P增益)

**RT-Thread实现**：
```c
typedef struct {
    float kp_roll;
    float kp_pitch;
    float kp_yaw;
    float max_rate_roll;
    float max_rate_pitch;
    float max_rate_yaw;
} att_control_param_t;

void attitude_control(quaternion_t *q_current, quaternion_t *q_desired,
                      vector3_t *rates_sp, att_control_param_t *param)
{
    // 1. 四元数误差
    quaternion_t q_error;
    quaternion_inverse(&q_error, q_current);
    quaternion_multiply(&q_error, &q_error, q_desired);

    // 2. 转角度误差
    axis_angle_t aa;
    quaternion_to_axis_angle(&aa, &q_error);

    // 3. P控制
    rates_sp->x = aa.axis.x * aa.angle * param->kp_roll;
    rates_sp->y = aa.axis.y * aa.angle * param->kp_pitch;
    rates_sp->z = aa.axis.z * aa.angle * param->kp_yaw;

    // 4. 限幅
    rates_sp->x = CONSTRAIN(rates_sp->x, -param->max_rate_roll, param->max_rate_roll);
    rates_sp->y = CONSTRAIN(rates_sp->y, -param->max_rate_pitch, param->max_rate_pitch);
    rates_sp->z = CONSTRAIN(rates_sp->z, -param->max_rate_yaw, param->max_rate_yaw);
}
```

---

### 阶段七：角速率控制器移植

#### 3.9 mc_rate_control核心代码

**文件**: `src/modules/mc_rate_control/MulticopterRateControl.cpp`

**主循环**（`MulticopterRateControl.cpp:103-280`）：
```cpp
void MulticopterRateControl::Run()
{
    // 1. 订阅当前角速度
    vehicle_angular_velocity_s angular_velocity;
    if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

        const Vector3f rates(angular_velocity.xyz);  // 当前角速度
        const Vector3f angular_accel(angular_velocity.xyz_derivative);  // 角加速度

        // 2. 获取角速率设定值（来自姿态控制器）
        vehicle_rates_setpoint_s rates_setpoint;
        _vehicle_rates_setpoint_sub.update(&rates_setpoint);

        const Vector3f rates_sp(
            rates_setpoint.roll,
            rates_setpoint.pitch,
            rates_setpoint.yaw
        );

        // 3. PID控制器计算力矩
        Vector3f torque_sp = _rate_control.update(
            rates,              // 当前角速度
            rates_sp,           // 期望角速度
            angular_accel,      // 角加速度（前馈）
            dt,
            _landed
        );

        // 4. 发布力矩设定值
        vehicle_torque_setpoint_s torque_setpoint;
        torque_setpoint.xyz[0] = torque_sp(0);
        torque_setpoint.xyz[1] = torque_sp(1);
        torque_setpoint.xyz[2] = torque_sp(2);
        _vehicle_torque_setpoint_pub.publish(torque_setpoint);
    }
}
```

**PID控制器**（`src/modules/mc_rate_control/RateControl/RateControl.cpp:80-150`）：
```cpp
Vector3f RateControl::update(const Vector3f &rate,
                             const Vector3f &rate_sp,
                             const Vector3f &angular_accel,
                             const float dt,
                             const bool landed)
{
    Vector3f torque;

    for (int axis = 0; axis < 3; axis++) {
        // 1. 计算误差
        float rate_error = rate_sp(axis) - rate(axis);

        // 2. P项
        float p_term = rate_error * _gain_p(axis);

        // 3. I项
        if (!landed) {
            _rate_int(axis) += rate_error * dt * _gain_i(axis);
            // 积分限幅
            _rate_int(axis) = math::constrain(_rate_int(axis),
                                              -_lim_int(axis), _lim_int(axis));
        } else {
            _rate_int(axis) = 0.0f;
        }

        // 4. D项
        float d_term = -angular_accel(axis) * _gain_d(axis);

        // 5. 前馈项
        float ff_term = rate_sp(axis) * _gain_ff(axis);

        // 6. 总输出 = P + I + D + FF
        torque(axis) = p_term + _rate_int(axis) + d_term + ff_term;
    }

    return torque;
}
```

**参数**：
- `MC_ROLLRATE_P` = 0.15 (Roll角速率P增益)
- `MC_ROLLRATE_I` = 0.2 (Roll角速率I增益)
- `MC_ROLLRATE_D` = 0.003 (Roll角速率D增益)
- `MC_ROLLRATE_FF` = 0.0 (前馈增益)

**RT-Thread实现**：
```c
typedef struct {
    float kp[3];      // P增益 [roll, pitch, yaw]
    float ki[3];      // I增益
    float kd[3];      // D增益
    float integral[3]; // 积分项
    float max_integral[3];  // 积分限幅
} rate_control_param_t;

void rate_control(vector3_t *rate, vector3_t *rate_sp, vector3_t *angular_accel,
                  vector3_t *torque_sp, rate_control_param_t *param, float dt)
{
    for (int i = 0; i < 3; i++) {
        // 误差
        float error = rate_sp->data[i] - rate->data[i];

        // P
        float p = error * param->kp[i];

        // I
        param->integral[i] += error * dt * param->ki[i];
        param->integral[i] = CONSTRAIN(param->integral[i],
                                        -param->max_integral[i],
                                        param->max_integral[i]);

        // D
        float d = -angular_accel->data[i] * param->kd[i];

        // 输出
        torque_sp->data[i] = p + param->integral[i] + d;
    }
}
```

---

### 阶段八：混控分配器移植

#### 3.10 control_allocator核心代码

**文件**: `src/modules/control_allocator/ControlAllocator.cpp`

**主循环**（`ControlAllocator.cpp:303-461`）：
```cpp
void ControlAllocator::Run()
{
    // 1. 订阅力矩和推力设定值
    vehicle_torque_setpoint_s torque_sp;
    vehicle_thrust_setpoint_s thrust_sp;

    _vehicle_torque_setpoint_sub.update(&torque_sp);
    _vehicle_thrust_setpoint_sub.update(&thrust_sp);

    // 2. 组装控制向量 [6×1]
    matrix::Vector<float, NUM_AXES> control_sp;
    control_sp(0) = torque_sp.xyz[0];   // Roll力矩
    control_sp(1) = torque_sp.xyz[1];   // Pitch力矩
    control_sp(2) = torque_sp.xyz[2];   // Yaw力矩
    control_sp(3) = thrust_sp.xyz[0];   // X推力
    control_sp(4) = thrust_sp.xyz[1];   // Y推力
    control_sp(5) = thrust_sp.xyz[2];   // Z推力

    // 3. 混控矩阵计算
    _control_allocation->setControlSetpoint(control_sp);
    _control_allocation->allocate();

    // 4. 获取电机输出
    const matrix::Vector<float, NUM_ACTUATORS> &actuator_sp =
        _control_allocation->getActuatorSetpoint();

    // 5. 发布到电机
    actuator_motors_s motors;
    for (int i = 0; i < _num_motors; i++) {
        motors.control[i] = actuator_sp(i);  // [-1, 1]
    }
    _actuator_motors_pub.publish(motors);
}
```

**混控矩阵**（四旋翼X构型）：
```
文件: src/modules/control_allocator/ControlAllocation/ControlAllocation.cpp

效率矩阵 B [6×4]:
        Motor1  Motor2  Motor3  Motor4
Roll  [  1.0   -1.0    -1.0     1.0  ]  ← X构型对称
Pitch [  1.0    1.0    -1.0    -1.0  ]
Yaw   [ -1.0    1.0    -1.0     1.0  ]  ← 对角线相反
Tx    [  0.0    0.0     0.0     0.0  ]  ← 无侧向推力
Ty    [  0.0    0.0     0.0     0.0  ]
Tz    [  1.0    1.0     1.0     1.0  ]  ← 垂直推力

执行器输出 = B^+ × 控制向量
(B^+ 是伪逆矩阵)

电机输出[4×1] = B^+ × [Roll力矩, Pitch力矩, Yaw力矩, 0, 0, Z推力]^T
```

**混控计算**（`ControlAllocation.cpp:~300`）：
```cpp
void ControlAllocation::allocate()
{
    // 方法1：伪逆法（最优）
    _actuator_sp = _control_allocation_matrix_pseudo_inv * _control_sp;

    // 方法2：顺序去饱和法（PX4默认）
    // 更复杂但能更好处理饱和
    sequentialDesaturation();

    // 限幅到 [-1, 1]
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        _actuator_sp(i) = math::constrain(_actuator_sp(i), -1.0f, 1.0f);
    }
}
```

**RT-Thread实现（简化版伪逆法）**：
```c
// 四旋翼X构型混控矩阵的伪逆（预计算）
static const float B_pinv[4][4] = {
    {  0.25f,  0.25f, -0.25f,  0.25f },  // Motor 1
    { -0.25f,  0.25f,  0.25f,  0.25f },  // Motor 2
    { -0.25f, -0.25f, -0.25f,  0.25f },  // Motor 3
    {  0.25f, -0.25f,  0.25f,  0.25f }   // Motor 4
};

void control_allocator(float torque_roll, float torque_pitch, float torque_yaw,
                       float thrust, float motor_output[4])
{
    // 混控计算
    for (int i = 0; i < 4; i++) {
        motor_output[i] = B_pinv[i][0] * torque_roll +
                         B_pinv[i][1] * torque_pitch +
                         B_pinv[i][2] * torque_yaw +
                         B_pinv[i][3] * thrust;

        // 限幅 [-1, 1]
        motor_output[i] = CONSTRAIN(motor_output[i], -1.0f, 1.0f);
    }
}
```

---

### 阶段九：PWM输出驱动移植

#### 3.11 PWMOut核心代码

**文件**: `src/drivers/pwm_out/PWMOut.cpp`

**更新输出**（`PWMOut.cpp:128-180`）：
```cpp
bool PWMOut::updateOutputs(uint16_t outputs[MAX_ACTUATORS],
                           unsigned num_outputs, unsigned num_control_groups_updated)
{
    // 1. 订阅电机控制
    actuator_motors_s actuator_motors;
    if (_actuator_motors_sub.update(&actuator_motors)) {

        for (size_t i = 0; i < num_outputs; i++) {
            // 2. 归一化值 [-1, 1] → PWM脉宽 [1000, 2000] us
            float control = actuator_motors.control[i];  // [-1, 1]

            // 线性映射
            uint16_t pwm = (uint16_t)((control + 1.0f) * 0.5f *
                                      (_pwm_max[i] - _pwm_min[i]) + _pwm_min[i]);

            // 3. 设置PWM
            outputs[i] = pwm;
            up_pwm_servo_set(i, pwm);  // 硬件PWM输出
        }
    }

    return true;
}
```

**硬件PWM配置**（`platforms/nuttx/src/px4/stm/stm32_common/io_timer/io_timer.c`）：
```c
int up_pwm_servo_set(unsigned channel, uint16_t value)
{
    // 1. 获取定时器配置
    const io_timer_channel_allocation_t *chan = &io_timers_channel_mapping[channel];

    // 2. 设置占空比
    uint32_t period = timer_io_get_period(chan->timer_index);
    uint32_t duty = (uint32_t)value * period / PWM_PERIOD_US;  // PWM_PERIOD_US = 20000 (50Hz)

    // 3. 更新CCR寄存器
    switch (chan->timer_channel) {
        case 1: TIMx->CCR1 = duty; break;
        case 2: TIMx->CCR2 = duty; break;
        case 3: TIMx->CCR3 = duty; break;
        case 4: TIMx->CCR4 = duty; break;
    }

    return OK;
}
```

**RT-Thread实现**：
```c
#include <rtthread.h>
#include <rtdevice.h>

// PWM设备句柄
static struct rt_device_pwm *pwm_dev;

void pwm_out_init(void)
{
    // 查找PWM设备
    pwm_dev = (struct rt_device_pwm *)rt_device_find("pwm1");

    // 配置PWM频率：400Hz (周期2500us)
    rt_pwm_set(pwm_dev, 1, 2500 * 1000, 0);  // 通道1
    rt_pwm_set(pwm_dev, 2, 2500 * 1000, 0);  // 通道2
    rt_pwm_set(pwm_dev, 3, 2500 * 1000, 0);  // 通道3
    rt_pwm_set(pwm_dev, 4, 2500 * 1000, 0);  // 通道4

    // 使能PWM
    rt_pwm_enable(pwm_dev, 1);
    rt_pwm_enable(pwm_dev, 2);
    rt_pwm_enable(pwm_dev, 3);
    rt_pwm_enable(pwm_dev, 4);
}

void pwm_out_update(float motor_output[4])
{
    for (int i = 0; i < 4; i++) {
        // 归一化 [-1, 1] → PWM [1000, 2000] us
        uint16_t pwm_us = (uint16_t)((motor_output[i] + 1.0f) * 500.0f + 1000.0f);

        // 限幅
        pwm_us = (pwm_us < 1000) ? 1000 : ((pwm_us > 2000) ? 2000 : pwm_us);

        // 设置PWM脉宽（RT-Thread单位：纳秒）
        rt_pwm_set(pwm_dev, i + 1, 2500 * 1000, pwm_us * 1000);
    }
}
```

---

## 四、完整移植文件清单

### 4.1 必需移植的核心文件

| 模块 | 源文件 | 大小 | 依赖 | 优先级 |
|------|--------|------|-----|--------|
| **uORB** | `src/modules/uORB/*` | 50KB | 无 | P0 |
| **Matrix库** | `src/lib/matrix/*` | 100KB | 无 | P0 |
| **BMI270驱动** | `src/drivers/imu/bosch/bmi270/BMI270.cpp` | 60KB | SPI | P1 |
| **PX4Gyroscope** | `src/lib/drivers/gyroscope/PX4Gyroscope.cpp` | 20KB | uORB | P1 |
| **VehicleIMU** | `src/modules/sensors/vehicle_imu/VehicleIMU.cpp` | 30KB | uORB | P1 |
| **VehicleAngularVelocity** | `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` | 40KB | Filter | P2 |
| **EKF2** | `src/modules/ekf2/EKF2.cpp`<br>`src/modules/ekf2/EKF/ekf.cpp` | 200KB | Matrix | P1 |
| **mc_att_control** | `src/modules/mc_att_control/*` | 30KB | Matrix | P1 |
| **mc_rate_control** | `src/modules/mc_rate_control/*` | 20KB | Matrix | P1 |
| **control_allocator** | `src/modules/control_allocator/*` | 40KB | Matrix | P1 |
| **PWMOut** | `src/drivers/pwm_out/PWMOut.cpp` | 15KB | HAL | P1 |

**总代码量估算**：
- 核心代码：~600KB
- 依赖库（Matrix等）：~200KB
- 总计：~800KB Flash

---

### 4.2 可选移植的增强文件

| 模块 | 作用 | 代码量 | 优先级 |
|------|------|--------|--------|
| **gyro_fft** | 陀螺仪FFT分析 | 30KB | P3 |
| **动态陷波滤波** | 电机振动抑制 | 20KB | P3 |
| **ESC遥测** | 电机RPM回传 | 15KB | P4 |
| **logger** | 日志记录 | 50KB | P2 |
| **mavlink** | 地面站通信 | 200KB | P2 |

---

## 五、RT-Thread移植架构设计

### 5.1 线程架构

```
┌─────────────────────────────────────────┐
│ RT-Thread调度器                          │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│ 线程1: bmi270_task (优先级5, 800Hz)      │
│ └─ 读取FIFO → 发布sensor_gyro_fifo      │
└─────────────────────────────────────────┘
              ↓ (uORB)
┌─────────────────────────────────────────┐
│ 线程2: vehicle_imu_task (优先级8)        │
│ └─ 积分 → 发布vehicle_imu               │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│ 线程3: angular_velocity_task (优先级5)   │
│ └─ 滤波 → 发布vehicle_angular_velocity  │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│ 线程4: ekf2_task (优先级10)              │
│ └─ EKF → 发布vehicle_attitude           │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│ 线程5: control_task (优先级7, 667Hz)     │
│ ├─ mc_att_control                       │
│ ├─ mc_rate_control                      │
│ ├─ control_allocator                    │
│ └─ pwm_out                              │
└─────────────────────────────────────────┘
              ↓ (PWM)
          [电机硬件]
```

### 5.2 优先级分配

| 线程 | RT-Thread优先级 | 频率 | 栈大小 |
|------|----------------|------|--------|
| `bmi270_task` | 5 (高) | 800Hz | 2KB |
| `angular_velocity_task` | 5 (高) | 667Hz | 2KB |
| `control_task` | 7 (中高) | 667Hz | 4KB |
| `vehicle_imu_task` | 8 (中) | 265Hz | 2KB |
| `ekf2_task` | 10 (中) | 193Hz | 8KB |

---

### 5.3 内存需求估算

| 项目 | 大小 |
|------|------|
| 代码段（Flash） | 800KB |
| 线程栈 | 20KB |
| uORB消息缓冲 | 10KB |
| EKF状态矩阵 | 20KB |
| 滤波器系数 | 5KB |
| **总计** | **~850KB Flash + 60KB RAM** |

---

## 六、验证和调试

### 6.1 分阶段测试

**阶段1：IMU数据采集**
```bash
# PX4: listener sensor_gyro_fifo
# RT-Thread: 打印FIFO数据
验证：3轴陀螺数据范围正常（±500 deg/s）
```

**阶段2：VehicleIMU输出**
```bash
# PX4: listener vehicle_imu
# RT-Thread: 打印delta_angle
验证：积分值合理
```

**阶段3：EKF2姿态**
```bash
# PX4: listener vehicle_attitude
# RT-Thread: 打印四元数
验证：静止时姿态稳定
```

**阶段4：控制输出**
```bash
# PX4: listener actuator_motors
# RT-Thread: 打印电机输出
验证：解锁后电机输出 > 0
```

**阶段5：PWM输出**
```bash
# 用示波器测量PWM脉宽
验证：1000-2000us范围
```

---

### 6.2 关键调试点

| 调试点 | 方法 | 预期值 |
|-------|------|--------|
| BMI270 FIFO频率 | 计时器测量 | 800Hz |
| 角速率控制频率 | 计数器 | 667Hz |
| 姿态估计延迟 | 时间戳差 | < 10ms |
| PWM输出频率 | 示波器 | 400Hz |
| CPU占用率 | RT-Thread top | < 60% |

---

## 七、常见问题和解决方案

### Q1: BMI270初始化失败
**原因**：
- SPI通信失败
- config文件上传错误

**解决**：
```cpp
// 检查WHO_AM_I寄存器
uint8_t chip_id = RegisterRead(Register::CHIP_ID);
if (chip_id != 0x24) {
    PX4_ERR("Wrong chip ID: 0x%02x", chip_id);
}

// 增加SPI延迟
usleep(1000);
```

### Q2: EKF2姿态发散
**原因**：
- IMU数据错误
- 参数不匹配

**解决**：
```bash
# 检查陀螺偏差
param set EKF2_ABL_LIM 0.4  # 增大陀螺偏差限制
param set EKF2_GBIAS_INIT 0.1  # 初始偏差

# 检查加速度计范围
param set EKF2_ACC_NOISE 0.35  # 增大加速度噪声
```

### Q3: 电机不响应
**原因**：
- 混控矩阵错误
- PWM未初始化
- 未解锁

**解决**：
```cpp
// 检查电机映射
param show CA_ROTOR0_PX  # 应为 0.707 (X构型)
param show CA_ROTOR0_PY  # 应为 0.707

// 强制解锁（调试用）
commander arm -f
```

---

## 八、总结

### 8.1 移植清单

- [x] 修改`default.px4board`只启用BMI270
- [x] 配置EKF2单例模式参数
- [x] 配置PWM输出参数
- [ ] 移植uORB通信机制
- [ ] 移植BMI270驱动
- [ ] 移植VehicleIMU模块
- [ ] 移植VehicleAngularVelocity（可选）
- [ ] 移植EKF2估计器
- [ ] 移植mc_att_control
- [ ] 移植mc_rate_control
- [ ] 移植control_allocator
- [ ] 移植PWM输出驱动
- [ ] 系统集成测试

### 8.2 关键代码行号速查表

| 模块 | 文件 | 关键函数 | 行号 |
|------|------|---------|------|
| BMI270初始化 | `BMI270.cpp` | `init()` | 86-252 |
| BMI270 FIFO读取 | `BMI270.cpp` | `FIFORead()` | 732-850 |
| VehicleIMU处理 | `VehicleIMU.cpp` | `Run()` | 169-600 |
| 角速度滤波 | `VehicleAngularVelocity.cpp` | `Run()` | 784-919 |
| EKF2主循环 | `EKF2.cpp` | `Run()` | 435-864 |
| EKF状态预测 | `ekf.cpp` | `update()` | 137-196 |
| 姿态控制 | `mc_att_control_main.cpp` | `Run()` | 205-400 |
| 角速率控制 | `MulticopterRateControl.cpp` | `Run()` | 103-280 |
| 混控分配 | `ControlAllocator.cpp` | `Run()` | 303-461 |
| PWM输出 | `PWMOut.cpp` | `updateOutputs()` | 128-180 |
| EKF2单例配置 | `EKF2.cpp` | `task_spawn()` | 2773-2839 |

---

## 九、参考资料

1. **PX4开发者文档**: https://docs.px4.io/
2. **EKF2算法**: `src/modules/ekf2/EKF2.cpp:2773` (模式选择逻辑)
3. **BMI270数据手册**: Bosch官方文档
4. **RT-Thread文档**: https://www.rt-thread.org/document/site/
5. **本仓库文档**:
   - `docm/00-algorithm/09-IMU到电机完整数据流与控制链路.md`
   - `docm/00-algorithm/19-EKF2多实例与单实例模式选择机制详解.md`
   - `docm/00-algorithm/13-DShot电机输出协议与双向遥测机制.md`

---

**文档版本**: v1.0
**创建日期**: 2025-10-31
**作者**: 基于PX4 v1.14+分析

