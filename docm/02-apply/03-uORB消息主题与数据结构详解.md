# uORB消息主题与数据结构详解

## 概述

本文档详细列出RT-Thread移植所需的所有uORB消息主题、数据结构定义、发布者、订阅者，以及工作队列设计分析。

---

## 一、消息主题总览

### 1.1 核心消息主题列表

| # | 主题名 | 发布者 | 订阅者 | 频率 | 数据大小 | 队列深度 |
|---|--------|--------|--------|------|---------|---------|
| 1 | `sensor_gyro_fifo` | imu_task | angular_velocity_task<br>imu_process_task | 800Hz | ~220B | 4 |
| 2 | `sensor_accel_fifo` | imu_task | imu_process_task | 800Hz | ~220B | 4 |
| 3 | `vehicle_imu` | imu_process_task | estimator_task | 265Hz | 80B | 8 |
| 4 | `vehicle_angular_velocity` | angular_velocity_task | control_task | 667Hz | 64B | 4 |
| 5 | `vehicle_attitude` | estimator_task | control_task | 193Hz | 80B | 4 |
| 6 | `vehicle_attitude_setpoint` | 外部(遥控/导航) | control_task | 50Hz | 64B | 2 |
| 7 | `vehicle_rates_setpoint` | control_task (姿态控制) | control_task (角速率控制) | 193Hz | 48B | 2 |
| 8 | `vehicle_torque_setpoint` | control_task (角速率控制) | control_task (混控) | 667Hz | 48B | 2 |
| 9 | `vehicle_thrust_setpoint` | control_task (角速率控制) | control_task (混控) | 667Hz | 48B | 2 |
| 10 | `actuator_motors` | control_task (混控) | control_task (PWM输出) | 667Hz | 80B | 2 |
| 11 | `vehicle_local_position` | estimator_task | (可选)导航模块 | 193Hz | 350B | 4 |

**总内存占用**: ~2.5KB (主题数据) + ~8KB (队列缓冲) = **~10.5KB**

---

## 二、详细数据结构定义

### 2.1 主题1: sensor_gyro_fifo

**文件位置**: `msg/SensorGyroFifo.msg`

**C结构体定义**:
```c
typedef struct {
    uint64_t timestamp;          // 系统启动后时间 (微秒)
    uint64_t timestamp_sample;   // 采样时间戳

    uint32_t device_id;          // 设备唯一ID

    float dt;                    // 样本间隔时间 (微秒)
    float scale;                 // 缩放因子 (rad/s per LSB)

    uint8_t samples;             // 有效样本数量 (通常2-3)

    int16_t x[32];               // X轴陀螺数据 (int16原始值)
    int16_t y[32];               // Y轴陀螺数据
    int16_t z[32];               // Z轴陀螺数据
} sensor_gyro_fifo_s;

// 数据大小: 8+8+4+4+4+1 + 32*2*3 = 221字节
```

**数据示例**:
```c
sensor_gyro_fifo_s gyro_fifo = {
    .timestamp = 1234567890,
    .timestamp_sample = 1234567000,
    .device_id = 0x1E0270,       // BMI270设备ID
    .dt = 625.0f,                // 625us = 1600Hz采样
    .scale = 0.00106526f,        // 0.00106 rad/s per LSB
    .samples = 2,                // 2个样本
    .x = {1234, 1240, ...},      // 原始int16数据
    .y = {-567, -560, ...},
    .z = {89, 92, ...}
};
```

**发布者**: `imu_task` (任务1)
**订阅者**:
- `angular_velocity_task` (任务2)
- `imu_process_task` (任务4)

**队列深度**: 4 (防止数据丢失)

---

### 2.2 主题2: sensor_accel_fifo

**文件位置**: `msg/SensorAccelFifo.msg`

**C结构体定义**:
```c
typedef struct {
    uint64_t timestamp;
    uint64_t timestamp_sample;

    uint32_t device_id;

    float dt;                    // 样本间隔 (微秒)
    float scale;                 // 缩放因子 (m/s² per LSB)

    uint8_t samples;             // 有效样本数

    int16_t x[32];               // X轴加速度数据
    int16_t y[32];               // Y轴加速度数据
    int16_t z[32];               // Z轴加速度数据
} sensor_accel_fifo_s;

// 数据大小: 221字节 (同gyro_fifo)
```

**发布者**: `imu_task` (任务1)
**订阅者**: `imu_process_task` (任务4)

---

### 2.3 主题3: vehicle_imu

**文件位置**: `msg/VehicleImu.msg`

**C结构体定义**:
```c
typedef struct {
    uint64_t timestamp;
    uint64_t timestamp_sample;

    uint32_t accel_device_id;
    uint32_t gyro_device_id;

    float delta_angle[3];        // 角度增量 (rad) over delta_angle_dt
    float delta_velocity[3];     // 速度增量 (m/s) over delta_velocity_dt

    uint32_t delta_angle_dt;     // 积分时间 (微秒)
    uint32_t delta_velocity_dt;  // 积分时间 (微秒)

    uint8_t delta_angle_clipping;    // 陀螺裁剪标志
    uint8_t delta_velocity_clipping; // 加速度裁剪标志

    uint8_t accel_calibration_count;
    uint8_t gyro_calibration_count;
} vehicle_imu_s;

// 数据大小: 8+8+4+4 + 4*3*2 + 4*2 + 4 = 80字节
```

**数据转换示例**:
```c
// 输入: sensor_gyro_fifo (int16原始值)
int16_t gyro_raw_x = 1234;
float gyro_scale = 0.00106526f;
float dt = 625e-6f;  // 625us = 0.000625s

// 转换为物理单位
float gyro_rad_s = gyro_raw_x * gyro_scale;  // 1.315 rad/s

// 积分为角度增量
float delta_angle = gyro_rad_s * dt;  // 0.000822 rad

// 输出: vehicle_imu
vehicle_imu_s imu = {
    .delta_angle = {0.000822f, ...},
    .delta_angle_dt = 625  // 微秒
};
```

**发布者**: `imu_process_task` (任务4)
**订阅者**: `estimator_task` (任务5)

---

### 2.4 主题4: vehicle_angular_velocity

**文件位置**: `msg/versioned/VehicleAngularVelocity.msg`

**C结构体定义**:
```c
typedef struct {
    uint64_t timestamp;
    uint64_t timestamp_sample;

    float xyz[3];                // 角速度 (rad/s) [roll, pitch, yaw]
    float xyz_derivative[3];     // 角加速度 (rad/s²)
} vehicle_angular_velocity_s;

// 数据大小: 8+8 + 4*3*2 = 40字节
```

**数据示例**:
```c
vehicle_angular_velocity_s ang_vel = {
    .timestamp = 1234567890,
    .timestamp_sample = 1234567000,
    .xyz = {0.1f, -0.05f, 0.02f},           // roll/pitch/yaw 角速度
    .xyz_derivative = {1.5f, -0.8f, 0.3f}   // 角加速度
};
```

**发布者**: `angular_velocity_task` (任务2)
**订阅者**: `control_task` (任务3)

**数据流**:
```
sensor_gyro_fifo (原始int16)
    ↓ 转换为物理单位
float gyro_rad_s = raw * scale
    ↓ 低通滤波
float filtered = lpf_apply(gyro_rad_s)
    ↓ 数值微分
float accel = (filtered - prev) / dt
    ↓ 输出
vehicle_angular_velocity
```

---

### 2.5 主题5: vehicle_attitude

**文件位置**: `msg/versioned/VehicleAttitude.msg`

**C结构体定义**:
```c
typedef struct {
    uint64_t timestamp;
    uint64_t timestamp_sample;

    float q[4];                  // 姿态四元数 [w, x, y, z] (Hamilton约定)
    float delta_q_reset[4];      // 上次重置的四元数变化
    uint8_t quat_reset_counter;  // 四元数重置计数器
} vehicle_attitude_s;

// 数据大小: 8+8 + 4*4*2 + 1 = 49字节
```

**四元数说明**:
```c
// Hamilton约定: q = [w, x, y, z]
// 其中: w = cos(θ/2), [x,y,z] = sin(θ/2) * axis

// 示例: 绕Z轴旋转45度
vehicle_attitude_s att = {
    .q = {0.9239f, 0.0f, 0.0f, 0.3827f}  // [w, x, y, z]
    // w = cos(45°/2) = 0.9239
    // z = sin(45°/2) = 0.3827
};

// 四元数转欧拉角 (roll, pitch, yaw):
float roll  = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
float pitch = asin(2*(q[0]*q[2] - q[3]*q[1]));
float yaw   = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
```

**发布者**: `estimator_task` (任务5)
**订阅者**: `control_task` (任务3)

---

### 2.6 主题6: vehicle_attitude_setpoint

**文件位置**: `msg/versioned/VehicleAttitudeSetpoint.msg`

**C结构体定义**:
```c
typedef struct {
    uint64_t timestamp;

    float yaw_sp_move_rate;      // 偏航速率命令 (rad/s)

    float q_d[4];                // 期望姿态四元数 [w, x, y, z]

    float thrust_body[3];        // 归一化推力 [-1, 1]
} vehicle_attitude_setpoint_s;

// 数据大小: 8 + 4 + 4*4 + 4*3 = 40字节
```

**数据来源**:
- 手动模式: 由遥控器摇杆输入转换
- 定点模式: 由位置控制器生成
- 自动模式: 由导航模块生成

**发布者**: 外部模块(rc_update, mc_pos_control, navigator等)
**订阅者**: `control_task` (任务3的姿态控制子模块)

---

### 2.7 主题7: vehicle_rates_setpoint

**文件位置**: `msg/versioned/VehicleRatesSetpoint.msg`

**C结构体定义**:
```c
typedef struct {
    uint64_t timestamp;

    float roll;                  // Roll角速率设定值 (rad/s)
    float pitch;                 // Pitch角速率设定值 (rad/s)
    float yaw;                   // Yaw角速率设定值 (rad/s)

    float thrust_body[3];        // 归一化推力 [-1, 1]

    bool reset_integral;         // 重置PID积分项标志
} vehicle_rates_setpoint_s;

// 数据大小: 8 + 4*3 + 4*3 + 1 = 33字节
```

**发布者**: `control_task` (任务3的姿态控制子模块)
**订阅者**: `control_task` (任务3的角速率控制子模块)

**数据流**:
```
vehicle_attitude (当前姿态)
    + vehicle_attitude_setpoint (期望姿态)
    ↓ 姿态PID控制器
vehicle_rates_setpoint (期望角速率)
```

---

### 2.8 主题8: vehicle_torque_setpoint

**文件位置**: `msg/VehicleTorqueSetpoint.msg`

**C结构体定义**:
```c
typedef struct {
    uint64_t timestamp;
    uint64_t timestamp_sample;

    float xyz[3];                // 归一化力矩 [-1, 1]
                                // xyz[0]: Roll力矩
                                // xyz[1]: Pitch力矩
                                // xyz[2]: Yaw力矩
} vehicle_torque_setpoint_s;

// 数据大小: 8+8 + 4*3 = 28字节
```

**发布者**: `control_task` (任务3的角速率控制子模块)
**订阅者**: `control_task` (任务3的混控子模块)

---

### 2.9 主题9: vehicle_thrust_setpoint

**文件位置**: `msg/VehicleThrustSetpoint.msg`

**C结构体定义**:
```c
typedef struct {
    uint64_t timestamp;
    uint64_t timestamp_sample;

    float xyz[3];                // 归一化推力 [-1, 1]
                                // xyz[0]: X推力 (前向)
                                // xyz[1]: Y推力 (右向)
                                // xyz[2]: Z推力 (下向，负值=向上)
} vehicle_thrust_setpoint_s;

// 数据大小: 8+8 + 4*3 = 28字节
```

**发布者**: `control_task` (任务3的角速率控制子模块)
**订阅者**: `control_task` (任务3的混控子模块)

**多旋翼说明**:
```c
// 对于四旋翼X构型:
thrust_setpoint.xyz[0] = 0.0f;      // 无侧向推力
thrust_setpoint.xyz[1] = 0.0f;      // 无侧向推力
thrust_setpoint.xyz[2] = -0.5f;     // 向上50%油门
```

---

### 2.10 主题10: actuator_motors

**文件位置**: `msg/versioned/ActuatorMotors.msg`

**C结构体定义**:
```c
typedef struct {
    uint64_t timestamp;
    uint64_t timestamp_sample;

    uint16_t reversible_flags;   // 可反转电机标志位

    uint8_t NUM_CONTROLS;        // = 12
    float control[12];           // 归一化电机输出 [-1, 1]
                                // control[0] = 电机1
                                // control[1] = 电机2
                                // control[2] = 电机3
                                // control[3] = 电机4
                                // ...
} actuator_motors_s;

// 数据大小: 8+8+2 + 1 + 4*12 = 67字节
```

**数据示例 (四旋翼)**:
```c
actuator_motors_s motors = {
    .timestamp = 1234567890,
    .timestamp_sample = 1234567800,
    .reversible_flags = 0,       // 不支持反转
    .NUM_CONTROLS = 12,
    .control = {
        0.45f,   // 电机1: 45%油门
        0.48f,   // 电机2: 48%油门
        0.42f,   // 电机3: 42%油门
        0.50f,   // 电机4: 50%油门
        NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN  // 其他电机未使用
    }
};
```

**发布者**: `control_task` (任务3的混控子模块)
**订阅者**: `control_task` (任务3的PWM输出子模块)

**后续转换**:
```c
// 归一化值 [-1, 1] → PWM脉宽 [1000, 2000] us
uint16_t pwm_us = (uint16_t)((control[i] + 1.0f) * 500.0f + 1000.0f);

// 示例:
// control = -1.0 → pwm = 1000us (最小)
// control =  0.0 → pwm = 1500us (中间)
// control =  1.0 → pwm = 2000us (最大)
```

---

### 2.11 主题11: vehicle_local_position (可选)

**文件位置**: `msg/versioned/VehicleLocalPosition.msg`

**C结构体定义**:
```c
typedef struct {
    uint64_t timestamp;
    uint64_t timestamp_sample;

    bool xy_valid;               // x, y有效标志
    bool z_valid;                // z有效标志
    bool v_xy_valid;             // vx, vy有效标志
    bool v_z_valid;              // vz有效标志

    // 位置 (NED坐标系, 单位: m)
    float x;                     // North (北)
    float y;                     // East (东)
    float z;                     // Down (下，负值=高度)

    // 位置重置增量
    float delta_xy[2];
    uint8_t xy_reset_counter;
    float delta_z;
    uint8_t z_reset_counter;

    // 速度 (NED坐标系, 单位: m/s)
    float vx;                    // North速度
    float vy;                    // East速度
    float vz;                    // Down速度
    float z_deriv;               // Down位置导数

    // 速度重置增量
    float delta_vxy[2];
    uint8_t vxy_reset_counter;
    float delta_vz;
    uint8_t vz_reset_counter;

    // 加速度 (NED坐标系, 单位: m/s²)
    float ax;
    float ay;
    float az;

    // 航向
    float heading;               // 偏航角 (rad)
    float heading_var;           // 偏航方差
    float unaided_heading;       // 仅陀螺积分的航向
    float delta_heading;
    uint8_t heading_reset_counter;
    bool heading_good_for_control;

    float tilt_var;

    // 全局参考点
    bool xy_global;
    bool z_global;
    uint64_t ref_timestamp;
    double ref_lat;              // 纬度 (度)
    double ref_lon;              // 经度 (度)
    float ref_alt;               // 海拔 (m)

    // 到地面距离
    bool dist_bottom_valid;
    float dist_bottom;
    float dist_bottom_var;
    float delta_dist_bottom;
    uint8_t dist_bottom_reset_counter;
    uint8_t dist_bottom_sensor_bitfield;

    // 误差估计
    float eph;                   // 水平位置误差 (m)
    float epv;                   // 垂直位置误差 (m)
    float evh;                   // 水平速度误差 (m/s)
    float evv;                   // 垂直速度误差 (m/s)

    bool dead_reckoning;

    // 限制
    float vxy_max;
    float vz_max;
    float hagl_min;
    float hagl_max_z;
    float hagl_max_xy;
} vehicle_local_position_s;

// 数据大小: ~350字节 (较大)
```

**说明**:
- 此主题数据量大，仅在需要位置控制时使用
- 简化版移植可以只输出姿态，不输出位置

---

## 三、数据流图

### 3.1 完整数据流

```
┌─────────────────────────────────────────────────────────────┐
│ 任务1: imu_task (800Hz)                                      │
│   模块: BMI270驱动                                           │
└─────────────────────────────────────────────────────────────┘
          ↓ sensor_gyro_fifo (221B × 800Hz = 177KB/s)
          ↓ sensor_accel_fifo (221B × 800Hz = 177KB/s)
          ├─────────────────────────────┐
          ↓                             ↓
┌──────────────────────────┐   ┌──────────────────────────────┐
│ 任务2:                   │   │ 任务4:                       │
│ angular_velocity_task    │   │ imu_process_task             │
│ (667Hz)                  │   │ (265Hz)                      │
└──────────────────────────┘   └──────────────────────────────┘
          ↓                             ↓
    vehicle_angular_velocity     vehicle_imu (80B × 265Hz = 21KB/s)
    (40B × 667Hz = 27KB/s)              ↓
          ↓                     ┌──────────────────────────────┐
          ↓                     │ 任务5: estimator_task        │
          ↓                     │ (193Hz)                      │
          ↓                     └──────────────────────────────┘
          ↓                             ↓ vehicle_attitude (49B × 193Hz = 9.5KB/s)
          ↓                             ↓ vehicle_local_position (350B × 193Hz = 68KB/s)
          ↓                             ↓
┌───────────────────────────────────────────────────────────────┐
│ 任务3: control_task (667Hz)                                   │
│                                                                │
│  ┌──────────────────────────────────────────────────────┐    │
│  │ 子模块1: mc_att_control (193Hz子循环)                │    │
│  │   输入: vehicle_attitude                              │    │
│  │   输入: vehicle_attitude_setpoint                     │    │
│  │   输出: vehicle_rates_setpoint                        │    │
│  └──────────────────────────────────────────────────────┘    │
│            ↓ vehicle_rates_setpoint (33B × 193Hz = 6.4KB/s)  │
│  ┌──────────────────────────────────────────────────────┐    │
│  │ 子模块2: mc_rate_control (667Hz)                     │    │
│  │   输入: vehicle_angular_velocity                      │    │
│  │   输入: vehicle_rates_setpoint                        │    │
│  │   输出: vehicle_torque_setpoint                       │    │
│  │   输出: vehicle_thrust_setpoint                       │    │
│  └──────────────────────────────────────────────────────┘    │
│            ↓ vehicle_torque_setpoint (28B × 667Hz = 19KB/s)  │
│            ↓ vehicle_thrust_setpoint (28B × 667Hz = 19KB/s)  │
│  ┌──────────────────────────────────────────────────────┐    │
│  │ 子模块3: control_allocator (667Hz)                   │    │
│  │   输入: vehicle_torque_setpoint                       │    │
│  │   输入: vehicle_thrust_setpoint                       │    │
│  │   输出: actuator_motors                               │    │
│  └──────────────────────────────────────────────────────┘    │
│            ↓ actuator_motors (67B × 667Hz = 45KB/s)          │
│  ┌──────────────────────────────────────────────────────┐    │
│  │ 子模块4: PWMOut (667Hz)                              │    │
│  │   输入: actuator_motors                               │    │
│  │   输出: PWM硬件                                       │    │
│  └──────────────────────────────────────────────────────┘    │
└───────────────────────────────────────────────────────────────┘
          ↓ PWM信号 (1000-2000us)
      [电机1-4硬件]
```

### 3.2 数据吞吐量统计

| 数据流 | 大小 | 频率 | 吞吐量 | 说明 |
|--------|------|------|--------|------|
| sensor_gyro_fifo | 221B | 800Hz | 177 KB/s | 最大数据流 |
| sensor_accel_fifo | 221B | 800Hz | 177 KB/s | 最大数据流 |
| vehicle_angular_velocity | 40B | 667Hz | 27 KB/s | 高频 |
| vehicle_local_position | 350B | 193Hz | 68 KB/s | 较大但低频 |
| actuator_motors | 67B | 667Hz | 45 KB/s | 高频 |
| 其他主题 | - | - | ~100 KB/s | 累计 |
| **总计** | - | - | **~600 KB/s** | 全部数据流 |

---

## 四、工作队列(WQ)必要性分析

### 4.1 PX4的工作队列机制

在PX4中，工作队列(Work Queue)用于：
1. **任务调度**: 将多个模块运行在同一个线程中
2. **减少上下文切换**: 相关模块串行执行，无切换开销
3. **保证执行顺序**: 同一WQ中的任务按顺序执行
4. **降低任务数量**: 减少线程栈内存占用

**PX4工作队列示例**:
```
wq:rate_ctrl (优先级最高, 667Hz)
  ├─ VehicleAngularVelocity  (触发源)
  ├─ mc_rate_control         (同步执行)
  └─ control_allocator       (同步执行)

优点:
- 3个模块共享1个线程栈(4KB)
- 无上下文切换延迟
- 保证执行顺序
```

---

### 4.2 RT-Thread是否需要工作队列？

#### 方案A: 使用独立线程（当前设计）

**优点**:
```
✓ 实现简单，每个任务独立
✓ RT-Thread原生调度，无需额外框架
✓ 任务优先级灵活调整
✓ 便于调试，每个任务独立监控
```

**缺点**:
```
✗ 5个任务 = 5个线程栈 = 18KB RAM
✗ 任务间切换有开销（~5-10us per switch）
✗ 可能的调度抖动
```

**内存占用**:
```
任务1: 2KB 栈
任务2: 2KB 栈
任务3: 4KB 栈
任务4: 2KB 栈
任务5: 8KB 栈
────────────
总计: 18KB
```

---

#### 方案B: 使用工作队列（优化版）

**设计思路**: 将高频任务合并到工作队列

```
WQ1: rate_ctrl_wq (优先级6, 667Hz)
  ├─ angular_velocity_task      (触发源)
  ├─ rate_control               (跟随执行)
  ├─ control_allocator          (跟随执行)
  └─ pwm_output                 (跟随执行)

独立线程1: imu_task (优先级5, 800Hz)
独立线程2: imu_process_task (优先级8, 265Hz)
独立线程3: estimator_task (优先级10, 193Hz)
独立线程4: attitude_control_task (优先级9, 193Hz)
```

**优点**:
```
✓ 高频控制链无上下文切换（angular_velocity → PWM一气呵成）
✓ 节省内存: 4个子模块共享1个栈
  原来: 2KB + 3KB(rate) + 1KB(allocator) = 6KB
  现在: 4KB (共享)
  节省: 2KB
✓ 降低延迟: angular_velocity到PWM输出 < 200us
✓ 更接近PX4原设计
```

**缺点**:
```
✗ 需要实现工作队列框架（额外2KB Flash）
✗ 调试稍复杂
✗ 姿态控制需要在工作队列中做子循环判断
```

**内存对比**:
```
方案A (独立线程):
  5个任务 × 栈 = 18KB

方案B (工作队列):
  3个独立线程 + 1个WQ = 2+2+8+4 = 16KB
  节省: 2KB
```

---

### 4.3 推荐方案

#### 阶段1: 使用独立线程（快速原型）

**理由**:
1. 实现简单，快速验证
2. 18KB RAM对STM32H7可接受
3. 便于调试和性能分析

**实现**:
```c
// 直接使用RT-Thread线程
rt_thread_t thread;

thread = rt_thread_create("ang_vel", angular_velocity_task, ...);
thread = rt_thread_create("control", control_task, ...);
// ...
```

---

#### 阶段2: 优化为工作队列（性能优化）

**何时优化**:
- CPU占用 > 70%
- 调度延迟 > 500us
- 内存不足

**实现框架**:
```c
// 简化的工作队列实现
typedef struct work_queue {
    rt_thread_t thread;
    rt_mq_t work_mq;
    work_item_t *work_list;
} work_queue_t;

typedef struct work_item {
    void (*func)(void *arg);
    void *arg;
    struct work_item *next;
} work_item_t;

// 工作队列线程
void wq_thread(void *param) {
    work_queue_t *wq = (work_queue_t *)param;

    while (1) {
        // 等待触发
        rt_mq_recv(wq->work_mq, ...);

        // 执行工作项链表
        work_item_t *item = wq->work_list;
        while (item) {
            item->func(item->arg);
            item = item->next;
        }
    }
}
```

---

### 4.4 最终建议

**初期移植（推荐）**:
```
使用独立线程，共5个任务
  - 实现简单
  - 调试方便
  - 18KB RAM可接受
```

**后期优化（可选）**:
```
合并高频控制链到工作队列
  - 降低延迟
  - 节省2KB RAM
  - 提高CPU利用率
```

**判断标准**:
```
如果满足以下条件，则使用独立线程：
  ✓ RAM > 256KB
  ✓ CPU占用 < 60%
  ✓ 延迟 < 1ms

如果出现以下情况，则优化为工作队列：
  ✗ RAM紧张 (< 200KB可用)
  ✗ CPU占用 > 70%
  ✗ 延迟 > 1.5ms
  ✗ 任务切换抖动明显
```

---

## 五、uORB实现建议

### 5.1 简化版uORB（推荐）

**核心功能**:
```c
// 主题定义
typedef struct {
    const char *name;
    size_t size;
    rt_mq_t mq;            // 消息队列
    void *latest_data;     // 最新数据缓存
    rt_mutex_t mutex;      // 互斥锁
} orb_topic_t;

// API
int orb_publish(orb_topic_t *topic, const void *data);
int orb_subscribe(orb_topic_t *topic);
int orb_copy(orb_topic_t *topic, void *data);
bool orb_check(orb_topic_t *topic, bool *updated);
int orb_copy_wait(orb_topic_t *topic, void *data, int timeout);
```

**实现示例**:
```c
// 发布
int orb_publish(orb_topic_t *topic, const void *data) {
    rt_mutex_take(topic->mutex, RT_WAITING_FOREVER);

    // 更新最新数据
    memcpy(topic->latest_data, data, topic->size);

    // 发送到消息队列（通知订阅者）
    rt_mq_send(topic->mq, data, topic->size);

    rt_mutex_release(topic->mutex);
    return 0;
}

// 订阅并阻塞等待
int orb_copy_wait(orb_topic_t *topic, void *data, int timeout) {
    // 阻塞等待新数据
    if (rt_mq_recv(topic->mq, data, topic->size, timeout) == RT_EOK) {
        return 0;
    }
    return -1;
}

// 非阻塞获取最新数据
int orb_copy(orb_topic_t *topic, void *data) {
    rt_mutex_take(topic->mutex, RT_WAITING_FOREVER);
    memcpy(data, topic->latest_data, topic->size);
    rt_mutex_release(topic->mutex);
    return 0;
}
```

**内存占用**:
```
uORB框架代码: ~5KB Flash
11个主题 × (40B数据 + 256B消息队列) = ~3.3KB RAM
```

---

## 六、总结

### 6.1 消息主题清单

| 主题 | 大小 | 发布频率 | 发布者 | 订阅者 |
|------|------|---------|--------|--------|
| sensor_gyro_fifo | 221B | 800Hz | imu_task | ang_vel, imu_proc |
| sensor_accel_fifo | 221B | 800Hz | imu_task | imu_proc |
| vehicle_imu | 80B | 265Hz | imu_proc | estimator |
| vehicle_angular_velocity | 40B | 667Hz | ang_vel | control |
| vehicle_attitude | 49B | 193Hz | estimator | control |
| vehicle_attitude_setpoint | 40B | 50Hz | 外部 | control |
| vehicle_rates_setpoint | 33B | 193Hz | control (att) | control (rate) |
| vehicle_torque_setpoint | 28B | 667Hz | control (rate) | control (alloc) |
| vehicle_thrust_setpoint | 28B | 667Hz | control (rate) | control (alloc) |
| actuator_motors | 67B | 667Hz | control (alloc) | control (pwm) |
| vehicle_local_position | 350B | 193Hz | estimator | (可选) |

### 6.2 工作队列建议

**初期**: 使用独立线程（5个任务）
- 优点: 实现简单，调试方便
- 缺点: 18KB栈内存

**后期**: 可选优化为工作队列
- 优点: 节省2KB，降低延迟
- 缺点: 需实现WQ框架

**判断**:
- RAM充足 → 独立线程
- RAM紧张或延迟要求高 → 工作队列

---

**文档版本**: v1.0
**创建日期**: 2025-10-31
**关联文档**:
- `02-RTThread任务架构与调度设计.md`
- `01-PX4核心飞控信号链移植到RTThread指南.md`

