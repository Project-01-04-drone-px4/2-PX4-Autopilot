# RT-Thread任务架构与调度设计

## 概述

本文档详细说明将PX4核心飞控信号链移植到RT-Thread时的任务划分、优先级分配和调度策略。

---

## 一、任务架构总览

### 1.1 任务总数

**最小配置（5个任务）**：
1. `imu_task` - BMI270 IMU采集
2. `imu_process_task` - VehicleIMU数据处理
3. `estimator_task` - EKF2姿态估计
4. `control_task` - 完整控制链（姿态+角速率+混控+PWM）
5. `angular_velocity_task` - 角速度滤波（可选，推荐）

**完整配置（7个任务）**：
- 在最小配置基础上增加：
6. `attitude_control_task` - 单独的姿态控制
7. `rate_control_task` - 单独的角速率控制+混控+PWM

---

## 二、详细任务配置表

### 2.1 最小配置（推荐）

| 任务ID | 任务名 | 包含模块 | RT优先级 | 频率 | 栈大小 | 触发方式 | 说明 |
|--------|--------|---------|---------|------|--------|---------|------|
| **1** | `imu_task` | BMI270驱动 | **5** (最高) | 800Hz | 2KB | 定时器/中断 | FIFO读取 |
| **2** | `angular_velocity_task` | VehicleAngularVelocity | **6** (高) | 667Hz | 2KB | 消息队列 | 滤波处理 |
| **3** | `imu_process_task` | VehicleIMU | **8** (中高) | 265Hz | 2KB | 消息队列 | IMU积分 |
| **4** | `estimator_task` | EKF2 | **10** (中) | 193Hz | 8KB | 消息队列 | 姿态估计 |
| **5** | `control_task` | mc_att_control<br>mc_rate_control<br>control_allocator<br>PWMOut | **7** (高) | 667Hz | 4KB | 消息队列 | 完整控制链 |

**总资源占用**：
- 任务数：5个
- 总栈：18KB RAM
- CPU占用：预计40-60%（STM32H7 @400MHz）

---

### 2.2 完整配置（性能优先）

| 任务ID | 任务名 | 包含模块 | RT优先级 | 频率 | 栈大小 | 触发方式 | 说明 |
|--------|--------|---------|---------|------|--------|---------|------|
| **1** | `imu_task` | BMI270驱动 | **5** (最高) | 800Hz | 2KB | 定时器/中断 | FIFO读取 |
| **2** | `angular_velocity_task` | VehicleAngularVelocity | **6** (高) | 667Hz | 2KB | 消息队列 | 滤波处理 |
| **3** | `rate_control_task` | mc_rate_control<br>control_allocator<br>PWMOut | **7** (高) | 667Hz | 3KB | 消息队列 | 内环控制 |
| **4** | `imu_process_task` | VehicleIMU | **8** (中高) | 265Hz | 2KB | 消息队列 | IMU积分 |
| **5** | `attitude_control_task` | mc_att_control | **9** (中) | 193Hz | 2KB | 消息队列 | 外环控制 |
| **6** | `estimator_task` | EKF2 | **10** (中) | 193Hz | 8KB | 消息队列 | 姿态估计 |
| **7** | `telemetry_task` | MAVLink/日志 | **15** (低) | 50Hz | 4KB | 定时器 | 可选 |

**总资源占用**：
- 任务数：7个
- 总栈：23KB RAM
- CPU占用：预计50-70%

---

## 三、任务详细设计

### 任务1: `imu_task` (BMI270 IMU采集)

#### 基本信息
```c
任务名：    imu_task
优先级：    5 (RT-Thread: 数字越小优先级越高)
频率：      800Hz (周期 1.25ms)
栈大小：    2048 字节
触发方式：  定时器周期触发 或 GPIO中断触发
```

#### 包含模块
- **BMI270驱动** (`src/drivers/imu/bosch/bmi270/BMI270.cpp`)

#### 主要工作
```c
void imu_task_entry(void *param)
{
    rt_timer_t fifo_timer;

    // 初始化BMI270
    bmi270_init();

    // 创建定时器：1.25ms周期（800Hz）
    fifo_timer = rt_timer_create("bmi270", bmi270_fifo_read,
                                  NULL,
                                  rt_tick_from_millisecond(1.25),  // 1.25ms
                                  RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(fifo_timer);

    while (1) {
        // 主循环等待（实际工作在定时器回调中）
        rt_thread_mdelay(1000);
    }
}

void bmi270_fifo_read(void *param)
{
    sensor_gyro_fifo_s gyro_fifo;
    sensor_accel_fifo_s accel_fifo;

    // 1. 读取FIFO数据（参考 BMI270.cpp:732-850）
    uint8_t fifo_data[FIFO_MAX_SIZE];
    bmi270_spi_read(REG_FIFO_DATA, fifo_data, fifo_length);

    // 2. 解析FIFO帧
    parse_fifo_frames(fifo_data, &gyro_fifo, &accel_fifo);

    // 3. 发布到uORB
    orb_publish(ORB_ID(sensor_gyro_fifo), &gyro_fifo);
    orb_publish(ORB_ID(sensor_accel_fifo), &accel_fifo);
}
```

#### 关键代码位置
| 函数 | 文件 | 行号 |
|------|------|------|
| `FIFORead()` | `BMI270.cpp` | 732-850 |
| `ProcessGyro()` | `BMI270.cpp` | 697-711 |
| `ProcessAccel()` | `BMI270.cpp` | 714-729 |

#### 性能指标
- **执行时间**: ~200-300us
- **WCET**: 500us
- **CPU占用**: ~16-24% (300us × 800Hz)

---

### 任务2: `angular_velocity_task` (角速度滤波)

#### 基本信息
```c
任务名：    angular_velocity_task
优先级：    6
频率：      667Hz (周期 1.5ms)
栈大小：    2048 字节
触发方式：  订阅 sensor_gyro_fifo 消息触发
```

#### 包含模块
- **VehicleAngularVelocity** (`src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`)

#### 主要工作
```c
void angular_velocity_task_entry(void *param)
{
    sensor_gyro_fifo_s gyro_fifo;
    vehicle_angular_velocity_s angular_vel;

    // 初始化滤波器
    lowpass_filter_init(&lpf_x, 80.0f, 1500.0f);  // 80Hz截止，1500Hz采样
    lowpass_filter_init(&lpf_y, 80.0f, 1500.0f);
    lowpass_filter_init(&lpf_z, 80.0f, 1500.0f);

    while (1) {
        // 等待新的FIFO数据（阻塞等待）
        if (orb_copy_wait(ORB_ID(sensor_gyro_fifo), &gyro_fifo, RT_WAITING_FOREVER)) {

            // 处理每个样本
            for (int i = 0; i < gyro_fifo.samples; i++) {
                // 转换为物理单位
                float gyro_x = gyro_fifo.x[i] * gyro_fifo.scale;
                float gyro_y = gyro_fifo.y[i] * gyro_fifo.scale;
                float gyro_z = gyro_fifo.z[i] * gyro_fifo.scale;

                // 低通滤波（参考 VehicleAngularVelocity.cpp:725-768）
                angular_vel.xyz[0] = lowpass_filter_apply(&lpf_x, gyro_x);
                angular_vel.xyz[1] = lowpass_filter_apply(&lpf_y, gyro_y);
                angular_vel.xyz[2] = lowpass_filter_apply(&lpf_z, gyro_z);
            }

            // 计算角加速度（数值微分）
            angular_vel.xyz_derivative[0] = (angular_vel.xyz[0] - prev_x) / dt;
            // ...

            // 发布
            orb_publish(ORB_ID(vehicle_angular_velocity), &angular_vel);
        }
    }
}
```

#### 关键代码位置
| 函数 | 文件 | 行号 |
|------|------|------|
| `Run()` | `VehicleAngularVelocity.cpp` | 784-919 |
| `FilterAngularVelocity()` | `VehicleAngularVelocity.cpp` | 725-768 |

#### 性能指标
- **执行时间**: ~100-150us
- **WCET**: 200us
- **CPU占用**: ~10-13% (150us × 667Hz)

---

### 任务3: `imu_process_task` (IMU数据处理)

#### 基本信息
```c
任务名：    imu_process_task
优先级：    8
频率：      265Hz (周期 3.77ms)
栈大小：    2048 字节
触发方式：  订阅 sensor_gyro_fifo 消息触发
```

#### 包含模块
- **VehicleIMU** (`src/modules/sensors/vehicle_imu/VehicleIMU.cpp`)

#### 主要工作
```c
void imu_process_task_entry(void *param)
{
    sensor_gyro_fifo_s gyro_fifo;
    sensor_accel_fifo_s accel_fifo;
    vehicle_imu_s imu_out;

    // 积分器初始化
    Vector3f delta_angle_accum = {0};
    Vector3f delta_velocity_accum = {0};

    while (1) {
        // 等待FIFO数据
        if (orb_copy_wait(ORB_ID(sensor_gyro_fifo), &gyro_fifo, RT_WAITING_FOREVER)) {

            // 同时获取加速度计数据
            orb_copy(ORB_ID(sensor_accel_fifo), &accel_fifo);

            // 处理FIFO中的每个样本（参考 VehicleIMU.cpp:169-600）
            for (int i = 0; i < gyro_fifo.samples; i++) {
                // 转换为物理单位
                float gyro_x = gyro_fifo.x[i] * gyro_fifo.scale;  // rad/s
                float accel_x = accel_fifo.x[i] * accel_fifo.scale;  // m/s²
                // ...

                // 积分：角速度 × dt → 角度增量
                delta_angle_accum.x += gyro_x * gyro_fifo.dt * 1e-6f;
                delta_angle_accum.y += gyro_y * gyro_fifo.dt * 1e-6f;
                delta_angle_accum.z += gyro_z * gyro_fifo.dt * 1e-6f;

                // 积分：加速度 × dt → 速度增量
                delta_velocity_accum.x += accel_x * accel_fifo.dt * 1e-6f;
                // ...
            }

            // 发布积分结果
            imu_out.delta_angle[0] = delta_angle_accum.x;
            imu_out.delta_angle[1] = delta_angle_accum.y;
            imu_out.delta_angle[2] = delta_angle_accum.z;
            imu_out.delta_velocity[0] = delta_velocity_accum.x;
            // ...

            orb_publish(ORB_ID(vehicle_imu), &imu_out);

            // 清零积分器
            delta_angle_accum = {0};
            delta_velocity_accum = {0};
        }
    }
}
```

#### 关键代码位置
| 函数 | 文件 | 行号 |
|------|------|------|
| `Run()` | `VehicleIMU.cpp` | 169-600 |

#### 性能指标
- **执行时间**: ~50-80us
- **WCET**: 100us
- **CPU占用**: ~2-3% (80us × 265Hz)

---

### 任务4: `estimator_task` (EKF2姿态估计)

#### 基本信息
```c
任务名：    estimator_task
优先级：    10
频率：      193Hz (周期 5.18ms)
栈大小：    8192 字节 (EKF需要较大栈)
触发方式：  订阅 vehicle_imu 消息触发
```

#### 包含模块
- **EKF2** (`src/modules/ekf2/EKF2.cpp`, `src/modules/ekf2/EKF/ekf.cpp`)

#### 主要工作
```c
void estimator_task_entry(void *param)
{
    vehicle_imu_s imu_data;
    vehicle_attitude_s attitude;
    vehicle_local_position_s position;

    // EKF初始化
    ekf_init();

    while (1) {
        // 等待IMU数据
        if (orb_copy_wait(ORB_ID(vehicle_imu), &imu_data, RT_WAITING_FOREVER)) {

            // 填充IMU样本到EKF缓冲区（参考 EKF2.cpp:435-864）
            imuSample imu_sample;
            imu_sample.delta_ang[0] = imu_data.delta_angle[0];
            imu_sample.delta_ang[1] = imu_data.delta_angle[1];
            imu_sample.delta_ang[2] = imu_data.delta_angle[2];
            imu_sample.delta_vel[0] = imu_data.delta_velocity[0];
            imu_sample.delta_vel[1] = imu_data.delta_velocity[1];
            imu_sample.delta_vel[2] = imu_data.delta_velocity[2];

            ekf_set_imu_data(&imu_sample);

            // 调用EKF更新（参考 ekf.cpp:137-196）
            if (ekf_update()) {
                // 获取姿态四元数
                ekf_get_quaternion(attitude.q);

                // 获取位置速度（简化版可省略）
                ekf_get_position(position.x, position.y, position.z);
                ekf_get_velocity(position.vx, position.vy, position.vz);

                // 发布姿态
                orb_publish(ORB_ID(vehicle_attitude), &attitude);
                orb_publish(ORB_ID(vehicle_local_position), &position);
            }
        }
    }
}
```

#### 关键代码位置
| 函数 | 文件 | 行号 |
|------|------|------|
| `Run()` | `EKF2.cpp` | 435-864 |
| `update()` | `ekf.cpp` | 137-196 |
| `predictState()` | `ekf.cpp` | ~700 |
| `predictCovariance()` | `ekf.cpp` | ~500 |

#### 性能指标
- **执行时间**: ~300-500us
- **WCET**: 800us
- **CPU占用**: ~10-15% (500us × 193Hz)

---

### 任务5: `control_task` (完整控制链)

#### 基本信息（最小配置）
```c
任务名：    control_task
优先级：    7
频率：      667Hz (周期 1.5ms)
栈大小：    4096 字节
触发方式：  订阅 vehicle_angular_velocity 消息触发
```

#### 包含模块
- **mc_att_control** - 姿态控制 (外环)
- **mc_rate_control** - 角速率控制 (内环)
- **control_allocator** - 混控分配
- **PWMOut** - PWM输出

#### 主要工作
```c
void control_task_entry(void *param)
{
    vehicle_angular_velocity_s angular_vel;
    vehicle_attitude_s attitude;
    vehicle_attitude_setpoint_s att_sp;
    vehicle_rates_setpoint_s rates_sp;
    vehicle_torque_setpoint_s torque_sp;
    actuator_motors_s motors;

    // 控制器初始化
    attitude_control_init();
    rate_control_init();

    while (1) {
        // 等待高频角速度数据（667Hz触发）
        if (orb_copy_wait(ORB_ID(vehicle_angular_velocity), &angular_vel, RT_WAITING_FOREVER)) {

            // ===== 1. 姿态控制（外环，193Hz更新） =====
            static uint32_t att_cnt = 0;
            if (++att_cnt >= 3) {  // 667/3 ≈ 193Hz
                att_cnt = 0;

                // 获取当前姿态和设定值
                orb_copy(ORB_ID(vehicle_attitude), &attitude);
                orb_copy(ORB_ID(vehicle_attitude_setpoint), &att_sp);

                // 姿态PID控制（参考 mc_att_control_main.cpp:205-400）
                attitude_control(&attitude, &att_sp, &rates_sp);
            }

            // ===== 2. 角速率控制（内环，667Hz） =====
            // 参考 MulticopterRateControl.cpp:103-280
            rate_control(&angular_vel, &rates_sp, &torque_sp);

            // ===== 3. 混控分配（667Hz） =====
            // 参考 ControlAllocator.cpp:303-461
            control_allocator(&torque_sp, &motors);

            // ===== 4. PWM输出（667Hz） =====
            // 参考 PWMOut.cpp:128-180
            pwm_output_update(&motors);
        }
    }
}
```

#### 关键代码位置
| 模块 | 函数 | 文件 | 行号 |
|------|------|------|------|
| 姿态控制 | `Run()` | `mc_att_control_main.cpp` | 205-400 |
| 角速率控制 | `Run()` | `MulticopterRateControl.cpp` | 103-280 |
| 混控 | `Run()`, `allocate()` | `ControlAllocator.cpp` | 303, 432 |
| PWM输出 | `updateOutputs()` | `PWMOut.cpp` | 128-180 |

#### 性能指标
- **执行时间**: ~150-200us (姿态控制时 +50us)
- **WCET**: 300us
- **CPU占用**: ~13-20% (200us × 667Hz)

---

## 四、任务间通信（uORB主题）

### 4.1 主题列表

| 主题名 | 发布者 | 订阅者 | 频率 | 数据大小 | 队列深度 |
|--------|--------|--------|------|---------|---------|
| `sensor_gyro_fifo` | imu_task | angular_velocity_task<br>imu_process_task | 800Hz | ~200B | 4 |
| `sensor_accel_fifo` | imu_task | imu_process_task | 800Hz | ~200B | 4 |
| `vehicle_imu` | imu_process_task | estimator_task | 265Hz | 100B | 8 |
| `vehicle_angular_velocity` | angular_velocity_task | control_task | 667Hz | 64B | 4 |
| `vehicle_attitude` | estimator_task | control_task | 193Hz | 80B | 4 |
| `vehicle_attitude_setpoint` | 遥控器/导航 | control_task | 50Hz | 64B | 2 |
| `vehicle_rates_setpoint` | control_task (姿态控制) | control_task (角速率控制) | 193Hz | 48B | 2 |
| `vehicle_torque_setpoint` | control_task (角速率控制) | control_task (混控) | 667Hz | 48B | 2 |
| `actuator_motors` | control_task (混控) | control_task (PWM输出) | 667Hz | 64B | 2 |

**总内存占用（uORB）**: ~2KB (主题数据) + ~8KB (队列缓冲) = **~10KB**

---

### 4.2 数据流图

```
[imu_task: 800Hz, 优先级5]
    ↓ sensor_gyro_fifo
    ↓ sensor_accel_fifo
    ├─→ [angular_velocity_task: 667Hz, 优先级6]
    │       ↓ vehicle_angular_velocity
    │       ↓
    │   [control_task: 667Hz, 优先级7]
    │       ├─ mc_att_control (193Hz子循环)
    │       │     ↓ vehicle_rates_setpoint
    │       ├─ mc_rate_control (667Hz)
    │       │     ↓ vehicle_torque_setpoint
    │       ├─ control_allocator (667Hz)
    │       │     ↓ actuator_motors
    │       └─ PWMOut (667Hz)
    │             ↓ [电机硬件]
    │
    └─→ [imu_process_task: 265Hz, 优先级8]
            ↓ vehicle_imu
            ↓
        [estimator_task: 193Hz, 优先级10]
            ↓ vehicle_attitude ──────────┘
            ↓ vehicle_local_position
```

---

## 五、优先级设计理念

### 5.1 优先级分配原则

| 优先级 | 分类 | 任务 | 原因 |
|--------|------|------|------|
| **5** | 最高 | imu_task | 传感器数据采集，不能丢失 |
| **6** | 高 | angular_velocity_task | 内环控制依赖，低延迟 |
| **7** | 高 | control_task | 控制回路实时性要求高 |
| **8** | 中高 | imu_process_task | 为EKF提供数据 |
| **10** | 中 | estimator_task | 姿态估计相对不紧急 |
| **15+** | 低 | 日志、遥测等 | 非实时任务 |

### 5.2 优先级倒置防护

**RT-Thread配置**：
```c
// 启用优先级继承（防止优先级倒置）
#define RT_USING_MUTEX
#define RT_IPC_INHERIT_PRIO  // 互斥量优先级继承
```

**关键资源保护**：
- uORB主题发布/订阅使用互斥锁
- SPI总线访问使用信号量

---

## 六、频率与调度分析

### 6.1 频率汇总表

| 任务 | 目标频率 | 实际周期 | 允许抖动 | 说明 |
|------|---------|---------|---------|------|
| imu_task | 800Hz | 1.25ms | ±50us | 硬实时 |
| angular_velocity_task | 667Hz | 1.5ms | ±100us | 软实时 |
| control_task | 667Hz | 1.5ms | ±100us | 软实时 |
| imu_process_task | 265Hz | 3.77ms | ±200us | 软实时 |
| estimator_task | 193Hz | 5.18ms | ±500us | 非实时 |

### 6.2 CPU占用估算（STM32H7 @400MHz）

| 任务 | 执行时间 | 频率 | CPU占用 | 备注 |
|------|---------|------|---------|------|
| imu_task | 300us | 800Hz | 24% | FIFO读取+解析 |
| angular_velocity_task | 150us | 667Hz | 10% | 滤波计算 |
| control_task | 200us | 667Hz | 13% | 完整控制链 |
| imu_process_task | 80us | 265Hz | 2% | 简单积分 |
| estimator_task | 500us | 193Hz | 10% | EKF矩阵运算 |
| **总计** | - | - | **59%** | 剩余41%裕量 |

### 6.3 最坏情况响应时间（WCRT）

**关键路径**：IMU中断 → 电机PWM输出

```
IMU中断触发 (t=0)
    ↓ (抢占)
imu_task执行: 300us
    ↓ (发布sensor_gyro_fifo)
angular_velocity_task就绪
    ↓ (抢占)
angular_velocity_task执行: 150us
    ↓ (发布vehicle_angular_velocity)
control_task就绪
    ↓ (抢占)
control_task执行: 200us
    ↓ (PWM更新)
电机响应 (t=650us)

总延迟：~650us (最坏情况)
平均延迟：~400-500us
```

---

## 七、内存占用总结

### 7.1 RAM占用

| 类型 | 大小 | 说明 |
|------|------|------|
| 任务栈（5个任务） | 18KB | 2+2+2+8+4 KB |
| uORB主题数据 | 2KB | 9个主题 |
| uORB队列缓冲 | 8KB | 消息队列 |
| EKF状态矩阵 | 15KB | 24×24协方差矩阵 |
| 滤波器系数 | 3KB | 各种滤波器 |
| 代码数据段 | 10KB | 全局变量等 |
| **总计** | **~56KB** | 建议预留80KB |

### 7.2 Flash占用

| 模块 | 大小 | 说明 |
|------|------|------|
| uORB通信框架 | 50KB | 简化版 |
| Matrix数学库 | 100KB | 矩阵运算 |
| BMI270驱动 | 60KB | 完整驱动 |
| VehicleIMU/AngularVelocity | 70KB | 传感器处理 |
| EKF2核心算法 | 200KB | 状态估计 |
| 控制器（姿态+角速率+混控） | 90KB | PID控制 |
| PWM输出 | 15KB | 硬件驱动 |
| 其他（滤波器等） | 50KB | 辅助功能 |
| **总计** | **~635KB** | 建议预留800KB |

---

## 八、RT-Thread配置建议

### 8.1 内核配置

```c
// rtconfig.h

// 调度器
#define RT_USING_HOOK                    // 使能钩子函数
#define RT_TICK_PER_SECOND 1000          // 1000Hz系统tick
#define RT_THREAD_PRIORITY_MAX 32        // 支持32级优先级
#define RT_USING_OVERFLOW_CHECK          // 栈溢出检测

// IPC
#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE

// 内存管理
#define RT_USING_MEMPOOL
#define RT_USING_HEAP
#define RT_USING_SMALL_MEM               // 小内存管理算法

// 定时器
#define RT_USING_TIMER_SOFT              // 软件定时器
#define RT_TIMER_THREAD_PRIO 4           // 定时器线程优先级
#define RT_TIMER_THREAD_STACK_SIZE 2048
```

### 8.2 SPI配置（BMI270）

```c
// SPI1配置示例（STM32H7）
#define BSP_USING_SPI1
#define BSP_SPI1_TX_USING_DMA            // 启用DMA提高效率
#define BSP_SPI1_RX_USING_DMA
#define BSP_SPI1_CLK_SPEED 20000000      // 20MHz
```

### 8.3 PWM配置

```c
// TIM1配置示例
#define BSP_USING_PWM
#define BSP_USING_PWM1
#define BSP_USING_PWM1_CH1               // 电机1
#define BSP_USING_PWM1_CH2               // 电机2
#define BSP_USING_PWM1_CH3               // 电机3
#define BSP_USING_PWM1_CH4               // 电机4
```

---

## 九、启动顺序

### 9.1 初始化流程

```c
int main(void)
{
    // 1. 系统初始化
    rt_hw_board_init();

    // 2. 硬件初始化
    spi_init();           // SPI总线
    pwm_init();           // PWM定时器
    gpio_init();          // GPIO（中断引脚）

    // 3. uORB初始化
    orb_init();

    // 4. 创建主题
    orb_advertise(ORB_ID(sensor_gyro_fifo));
    orb_advertise(ORB_ID(vehicle_angular_velocity));
    orb_advertise(ORB_ID(vehicle_imu));
    orb_advertise(ORB_ID(vehicle_attitude));
    orb_advertise(ORB_ID(actuator_motors));

    // 5. 创建任务（按优先级从低到高）
    rt_thread_t thread;

    // 优先级10: estimator_task
    thread = rt_thread_create("estimator", estimator_task_entry,
                              NULL, 8192, 10, 10);
    rt_thread_startup(thread);

    // 优先级8: imu_process_task
    thread = rt_thread_create("imu_proc", imu_process_task_entry,
                              NULL, 2048, 8, 10);
    rt_thread_startup(thread);

    // 优先级7: control_task
    thread = rt_thread_create("control", control_task_entry,
                              NULL, 4096, 7, 10);
    rt_thread_startup(thread);

    // 优先级6: angular_velocity_task
    thread = rt_thread_create("ang_vel", angular_velocity_task_entry,
                              NULL, 2048, 6, 10);
    rt_thread_startup(thread);

    // 优先级5: imu_task (最后创建，最高优先级)
    thread = rt_thread_create("imu", imu_task_entry,
                              NULL, 2048, 5, 10);
    rt_thread_startup(thread);

    // 6. 启动调度器
    // RT-Thread自动启动，无需手动调用

    return 0;
}
```

---

## 十、调试和监控

### 10.1 性能监控命令

```c
// 在RT-Thread MSH shell中

// 查看线程状态
msh> list_thread
thread   pri  status      sp     stack size max used left tick  error
-------- ---  ------- ---------- ----------  ------  ---------- ---
imu       05  suspend 0x00000100 0x00000800    25%   0x0000000a 000
ang_vel   06  suspend 0x00000100 0x00000800    30%   0x0000000a 000
control   07  ready   0x00000200 0x00001000    35%   0x00000000 000
imu_proc  08  suspend 0x00000100 0x00000800    20%   0x0000000a 000
estimator 10  suspend 0x00000400 0x00002000    45%   0x0000000a 000

// 查看CPU占用率
msh> top
CPU usage: 59%
thread      cpu
imu         24%
ang_vel     10%
control     13%
imu_proc     2%
estimator   10%

// 查看信号量/互斯量
msh> list_sem
msh> list_mutex
```

### 10.2 关键性能指标

| 指标 | 目标值 | 测量方法 |
|------|--------|---------|
| IMU采集频率 | 800Hz ±1% | 计数器 |
| 控制回路频率 | 667Hz ±2% | 计数器 |
| 端到端延迟 | < 1ms | 时间戳差值 |
| CPU占用率 | < 70% | `top`命令 |
| 最大栈使用 | < 80% | `list_thread` |

---

## 十一、可选优化

### 11.1 更激进的方案（极限性能）

如果需要更低延迟，可以将control_task拆分：

```
任务架构变更：
5个任务 → 7个任务

新增：
- rate_control_task (优先级7, 667Hz) - 只包含角速率控制+混控+PWM
- attitude_control_task (优先级9, 193Hz) - 只包含姿态控制

优点：
- 角速率控制延迟降低到 ~500us
- 更细粒度的优先级控制

缺点：
- 增加2个任务栈（+5KB RAM）
- 增加任务切换开销
```

### 11.2 更保守的方案（资源受限）

如果RAM/Flash有限，可以简化：

```
简化方案：
- 去除VehicleAngularVelocity（不做滤波，直接用原始数据）
- 简化EKF2（只估计姿态，不估计位置速度）

资源节省：
- -1个任务（angular_velocity_task）
- -2KB RAM（栈）
- -100KB Flash（EKF简化版）

代价：
- 控制性能略降（无滤波）
- 无位置估计（只能用于姿态模式）
```

---

## 十二、总结

### 最小配置推荐（5任务）

| 任务 | 优先级 | 频率 | 栈 | 模块 |
|------|--------|------|-----|------|
| imu_task | 5 | 800Hz | 2KB | BMI270 |
| angular_velocity_task | 6 | 667Hz | 2KB | VehicleAngularVelocity |
| control_task | 7 | 667Hz | 4KB | 姿态+角速率+混控+PWM |
| imu_process_task | 8 | 265Hz | 2KB | VehicleIMU |
| estimator_task | 10 | 193Hz | 8KB | EKF2 |

**总资源**: 5任务, 18KB栈, 56KB RAM, 635KB Flash

---

## 附录A：快速参考卡片

```
┌─────────────────────────────────────────────────────────┐
│ RT-Thread 飞控任务配置速查表                             │
├─────────────────────────────────────────────────────────┤
│ 任务1: imu_task                                          │
│   优先级: 5 (最高) | 频率: 800Hz | 栈: 2KB              │
│   模块: BMI270驱动                                       │
│   触发: 定时器1.25ms                                     │
│   输出: sensor_gyro_fifo, sensor_accel_fifo             │
├─────────────────────────────────────────────────────────┤
│ 任务2: angular_velocity_task                             │
│   优先级: 6 (高) | 频率: 667Hz | 栈: 2KB                │
│   模块: VehicleAngularVelocity                           │
│   触发: 订阅sensor_gyro_fifo                             │
│   输出: vehicle_angular_velocity                         │
├─────────────────────────────────────────────────────────┤
│ 任务3: control_task                                      │
│   优先级: 7 (高) | 频率: 667Hz | 栈: 4KB                │
│   模块: mc_att_control(193Hz子循环) +                   │
│         mc_rate_control + allocator + PWMOut            │
│   触发: 订阅vehicle_angular_velocity                     │
│   输出: actuator_motors → PWM硬件                       │
├─────────────────────────────────────────────────────────┤
│ 任务4: imu_process_task                                  │
│   优先级: 8 (中高) | 频率: 265Hz | 栈: 2KB              │
│   模块: VehicleIMU                                       │
│   触发: 订阅sensor_gyro_fifo                             │
│   输出: vehicle_imu                                      │
├─────────────────────────────────────────────────────────┤
│ 任务5: estimator_task                                    │
│   优先级: 10 (中) | 频率: 193Hz | 栈: 8KB               │
│   模块: EKF2                                             │
│   触发: 订阅vehicle_imu                                  │
│   输出: vehicle_attitude, vehicle_local_position        │
└─────────────────────────────────────────────────────────┘

CPU占用: ~59% (STM32H7 @400MHz)
RAM占用: 56KB
Flash占用: 635KB
```

---

**文档版本**: v1.0
**创建日期**: 2025-10-31
**对应主文档**: `01-PX4核心飞控信号链移植到RTThread指南.md`

