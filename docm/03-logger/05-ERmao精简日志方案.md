# ERmao 模式精简日志方案

## 一、方案概述

### 问题背景

1. **数据丢失问题**：原始主题的 uORB 队列深度不足（如 sensor_gyro_fifo 只有 4），导致高速记录时数据丢失
2. **数据冗余问题**：原始消息包含大量与控制信号链无关的字段（如设备ID、校准计数器、重置标志等）
3. **性能问题**：消息体积大，增加 SD 卡写入压力和内存占用

### 解决方案

创建**精简版专用日志消息**（`Log*` 系列），特点：
- ✅ **只包含核心信号链数据**（去除元数据和冗余字段）
- ✅ **增加队列深度**（32 或 16，防止数据丢失）
- ✅ **减小消息体积**（降低带宽需求）
- ✅ **全速率记录**（按原始发布频率记录）

---

## 二、精简消息定义对比

### 2.1 sensor_gyro_fifo → LogGyroFifo

#### 原始消息 (221 字节)
```cpp
struct sensor_gyro_fifo_s {
    uint64 timestamp;        // 8 字节
    uint64 timestamp_sample; // 8 字节
    uint32 device_id;        // 4 字节 ❌ 去除
    float32 dt;              // 4 字节 ❌ 去除（可从 timestamp 计算）
    float32 scale;           // 4 字节 ✅ 保留（需要缩放）
    uint8 samples;           // 1 字节 ✅ 保留
    int16[32] x;             // 64 字节 ❌ 太大，只有 2 个有效
    int16[32] y;             // 64 字节 ❌ 太大
    int16[32] z;             // 64 字节 ❌ 太大
    uint8 ORB_QUEUE_LENGTH = 4;  // ❌ 太小
};
// 总计：221 字节
```

#### 精简消息 (117 字节，减少 47%)
```cpp
struct log_gyro_fifo_s {
    uint64 timestamp;        // 8 字节
    uint64 timestamp_sample; // 8 字节
    float32 scale;           // 4 字节（用于数据验证）
    uint8 samples;           // 1 字节
    float32[8] x;            // 32 字节（只存储有效样本，已缩放为 rad/s）
    float32[8] y;            // 32 字节
    float32[8] z;            // 32 字节
    uint8 ORB_QUEUE_LENGTH = 32;  // ✅ 增大到 32
};
// 总计：117 字节，减少 104 字节（-47%）
```

**优化点**：
- ✅ 去除 device_id（记录时固定，可在日志元数据中记录一次）
- ✅ 去除 dt（可从 timestamp 差值计算）
- ✅ 数组从 32 个减少到 8 个（实际只有 2 个有效样本）
- ✅ 直接存储 float32 rad/s（已缩放），方便后处理
- ✅ 队列深度从 4 增加到 32（8 倍，可容忍更长延迟）

---

### 2.2 vehicle_angular_velocity → LogAngularVelocity

#### 原始消息 (56 字节)
```cpp
struct vehicle_angular_velocity_s {
    uint64 timestamp;           // 8 字节
    uint64 timestamp_sample;    // 8 字节
    float32[3] xyz;             // 12 字节 ✅ 保留
    float32[3] xyz_derivative;  // 12 字节 ❌ 去除（角加速度，非核心）
    uint8 ORB_QUEUE_LENGTH = 8;  // ❌ 队列较小
};
// 总计：40 字节
```

#### 精简消息 (28 字节，减少 30%)
```cpp
struct log_angular_velocity_s {
    uint64 timestamp;        // 8 字节
    uint64 timestamp_sample; // 8 字节
    float32[3] xyz;          // 12 字节
    uint8 ORB_QUEUE_LENGTH = 32;  // ✅ 增大到 32
};
// 总计：28 字节，减少 12 字节（-30%）
```

**优化点**：
- ✅ 去除 xyz_derivative（角加速度可后处理计算）
- ✅ 队列深度从 8 增加到 32

---

### 2.3 vehicle_attitude → LogAttitude

#### 原始消息 (57 字节)
```cpp
struct vehicle_attitude_s {
    uint64 timestamp;        // 8 字节
    uint64 timestamp_sample; // 8 字节
    float32[4] q;            // 16 字节 ❌ 去除（可从欧拉角反推）
    float32[4] delta_q_reset;// 16 字节 ❌ 去除（重置相关）
    uint8 quat_reset_counter;// 1 字节 ❌ 去除
    float32 roll;            // 4 字节 ✅ 保留
    float32 pitch;           // 4 字节 ✅ 保留
    float32 yaw;             // 4 字节 ✅ 保留
    uint8 ORB_QUEUE_LENGTH = 8;  // ❌ 队列较小
};
// 总计：65 字节
```

#### 精简消息 (28 字节，减少 57%)
```cpp
struct log_attitude_s {
    uint64 timestamp;        // 8 字节
    uint64 timestamp_sample; // 8 字节
    float32 roll;            // 4 字节
    float32 pitch;           // 4 字节
    float32 yaw;             // 4 字节
    uint8 ORB_QUEUE_LENGTH = 16;  // ✅ 增大到 16
};
// 总计：28 字节，减少 37 字节（-57%）
```

**优化点**：
- ✅ 去除四元数 q（欧拉角是主要分析对象）
- ✅ 去除重置相关字段（delta_q_reset, quat_reset_counter）
- ✅ 队列深度从 8 增加到 16

---

### 2.4 vehicle_attitude_setpoint → LogAttitudeSetpoint

#### 原始消息 (52 字节)
```cpp
struct vehicle_attitude_setpoint_s {
    uint64 timestamp;        // 8 字节
    float32 yaw_sp_move_rate;// 4 字节 ❌ 去除（通常为 NAN）
    float32[4] q_d;          // 16 字节 ❌ 去除
    float32 roll_body;       // 4 字节 ✅ 保留
    float32 pitch_body;      // 4 字节 ✅ 保留
    float32 yaw_body;        // 4 字节 ✅ 保留
    float32[3] thrust_body;  // 12 字节 ❌ 去除（控制输出在 actuator_motors）
    uint8 ORB_QUEUE_LENGTH = 8;  // ❌ 队列较小
};
// 总计：56 字节
```

#### 精简消息 (20 字节，减少 64%)
```cpp
struct log_attitude_setpoint_s {
    uint64 timestamp;        // 8 字节
    float32 roll_body;       // 4 字节
    float32 pitch_body;      // 4 字节
    float32 yaw_body;        // 4 字节
    uint8 ORB_QUEUE_LENGTH = 16;  // ✅ 增大到 16
};
// 总计：20 字节，减少 36 字节（-64%）
```

**优化点**：
- ✅ 去除四元数 q_d
- ✅ 去除 yaw_sp_move_rate
- ✅ 去除 thrust_body（推力在 actuator_motors 中记录）
- ✅ 队列深度从 8 增加到 16

---

### 2.5 vehicle_rates_setpoint → LogRatesSetpoint

#### 原始消息 (36 字节)
```cpp
struct vehicle_rates_setpoint_s {
    uint64 timestamp;        // 8 字节
    float32 roll;            // 4 字节 ✅ 保留
    float32 pitch;           // 4 字节 ✅ 保留
    float32 yaw;             // 4 字节 ✅ 保留
    float32[3] thrust_body;  // 12 字节 ❌ 去除
    bool reset_integral;     // 1 字节 ❌ 去除
    uint8 ORB_QUEUE_LENGTH = 8;  // ❌ 队列较小
};
// 总计：37 字节
```

#### 精简消息 (20 字节，减少 46%)
```cpp
struct log_rates_setpoint_s {
    uint64 timestamp;        // 8 字节
    float32 roll;            // 4 字节
    float32 pitch;           // 4 字节
    float32 yaw;             // 4 字节
    uint8 ORB_QUEUE_LENGTH = 16;  // ✅ 增大到 16
};
// 总计：20 字节，减少 17 字节（-46%）
```

**优化点**：
- ✅ 去除 thrust_body
- ✅ 去除 reset_integral
- ✅ 队列深度从 8 增加到 16

---

### 2.6 actuator_motors → LogActuatorMotors

#### 原始消息 (152 字节)
```cpp
struct actuator_motors_s {
    uint64 timestamp;        // 8 字节
    uint64 timestamp_sample; // 8 字节
    bool reversible_flags;   // 1 字节 ❌ 去除
    float32[16] control;     // 64 字节 ❌ 太大，四旋翼只用 4 个
    uint8 ORB_QUEUE_LENGTH = 8;  // ❌ 队列较小
};
// 总计：81 字节
```

#### 精简消息 (32 字节，减少 60%)
```cpp
struct log_actuator_motors_s {
    uint64 timestamp;        // 8 字节
    uint64 timestamp_sample; // 8 字节
    float32[4] control;      // 16 字节（只记录 4 个电机）
    uint8 ORB_QUEUE_LENGTH = 16;  // ✅ 增大到 16
};
// 总计：32 字节，减少 49 字节（-60%）
```

**优化点**：
- ✅ 去除 reversible_flags
- ✅ 数组从 16 个减少到 4 个（四旋翼）
- ✅ 队列深度从 8 增加到 16

---

### 2.7 sensor_combined → LogSensorCombined

#### 原始消息 (64 字节)
```cpp
struct sensor_combined_s {
    uint64 timestamp;                       // 8 字节
    int32 RELATIVE_TIMESTAMP_INVALID;       // 4 字节 ❌ 去除（常量）
    float32[3] gyro_rad;                    // 12 字节 ✅ 保留
    uint32 gyro_integral_dt;                // 4 字节 ❌ 去除
    int32 accelerometer_timestamp_relative; // 4 字节 ❌ 去除
    float32[3] accelerometer_m_s2;          // 12 字节 ✅ 保留
    uint32 accelerometer_integral_dt;       // 4 字节 ❌ 去除
    uint8 accelerometer_clipping;           // 1 字节 ❌ 去除
    uint8 gyro_clipping;                    // 1 字节 ❌ 去除
    uint8 accel_calibration_count;          // 1 字节 ❌ 去除
    uint8 gyro_calibration_count;           // 1 字节 ❌ 去除
    uint8 ORB_QUEUE_LENGTH = 8;             // ❌ 队列较小
};
// 总计：52 字节
```

#### 精简消息 (32 字节，减少 38%)
```cpp
struct log_sensor_combined_s {
    uint64 timestamp;            // 8 字节
    float32[3] gyro_rad;         // 12 字节
    float32[3] accelerometer_m_s2;// 12 字节
    uint8 ORB_QUEUE_LENGTH = 32;  // ✅ 增大到 32
};
// 总计：32 字节，减少 20 字节（-38%）
```

**优化点**：
- ✅ 去除所有时间戳相关字段（相对时间戳、积分时间）
- ✅ 去除裁剪标志和校准计数器
- ✅ 队列深度从 8 增加到 32

---

### 2.8 vehicle_imu → LogVehicleImu

#### 原始消息 (64 字节)
```cpp
struct vehicle_imu_s {
    uint64 timestamp;            // 8 字节
    uint64 timestamp_sample;     // 8 字节
    uint32 accel_device_id;      // 4 字节 ❌ 去除
    uint32 gyro_device_id;       // 4 字节 ❌ 去除
    float32[3] delta_angle;      // 12 字节 ✅ 保留
    float32[3] delta_velocity;   // 12 字节 ✅ 保留
    uint32 delta_angle_dt;       // 4 字节 ❌ 去除
    uint32 delta_velocity_dt;    // 4 字节 ❌ 去除
    uint8 delta_angle_clipping;  // 1 字节 ❌ 去除
    uint8 delta_velocity_clipping;// 1 字节 ❌ 去除
    uint8 accel_calibration_count;// 1 字节 ❌ 去除
    uint8 gyro_calibration_count;// 1 字节 ❌ 去除
    uint8 ORB_QUEUE_LENGTH = 8;  // ❌ 队列较小
};
// 总计：60 字节
```

#### 精简消息 (40 字节，减少 33%)
```cpp
struct log_vehicle_imu_s {
    uint64 timestamp;        // 8 字节
    uint64 timestamp_sample; // 8 字节
    float32[3] delta_angle;  // 12 字节
    float32[3] delta_velocity;// 12 字节
    uint8 ORB_QUEUE_LENGTH = 16;  // ✅ 增大到 16
};
// 总计：40 字节，减少 20 字节（-33%）
```

**优化点**：
- ✅ 去除设备 ID
- ✅ 去除积分时间（dt 可从 timestamp 计算）
- ✅ 去除裁剪标志和校准计数器
- ✅ 队列深度从 8 增加到 16

---

## 三、数据量对比

### 原始消息总数据量（每秒）

假设采样率：
- sensor_gyro_fifo: 50 Hz
- vehicle_angular_velocity: 667 Hz
- vehicle_imu: 250 Hz
- sensor_combined: 1000 Hz
- vehicle_attitude: 250 Hz
- vehicle_attitude_setpoint: 250 Hz
- vehicle_rates_setpoint: 250 Hz
- actuator_motors: 250 Hz

**原始总数据量**：
```
sensor_gyro_fifo:         221 × 50   = 11,050 字节/秒
vehicle_angular_velocity:  40 × 667  = 26,680 字节/秒
vehicle_imu:               60 × 250  = 15,000 字节/秒
sensor_combined:           52 × 1000 = 52,000 字节/秒
vehicle_attitude:          65 × 250  = 16,250 字节/秒
vehicle_attitude_setpoint: 56 × 250  = 14,000 字节/秒
vehicle_rates_setpoint:    37 × 250  =  9,250 字节/秒
actuator_motors:           81 × 250  = 20,250 字节/秒
---------------------------------------------------
总计：                              164,480 字节/秒 ≈ 161 KB/s
```

### 精简消息总数据量（每秒）

**精简总数据量**：
```
log_gyro_fifo:             117 × 50   =  5,850 字节/秒 (-47%)
log_angular_velocity:       28 × 667  = 18,676 字节/秒 (-30%)
log_vehicle_imu:            40 × 250  = 10,000 字节/秒 (-33%)
log_sensor_combined:        32 × 1000 = 32,000 字节/秒 (-38%)
log_attitude:               28 × 250  =  7,000 字节/秒 (-57%)
log_attitude_setpoint:      20 × 250  =  5,000 字节/秒 (-64%)
log_rates_setpoint:         20 × 250  =  5,000 字节/秒 (-46%)
log_actuator_motors:        32 × 250  =  8,000 字节/秒 (-60%)
---------------------------------------------------
总计：                               91,526 字节/秒 ≈ 89 KB/s
```

**节省**：164 - 89 = **75 KB/s（-45%）**

---

## 四、队列深度对比

| 主题                      | 原始队列 | 精简队列 | 增加倍数 | 内存增加       |
|---------------------------|---------|---------|---------|---------------|
| sensor_gyro_fifo          | 4       | 32      | 8x      | +3.3 KB       |
| vehicle_angular_velocity  | 8       | 32      | 4x      | +0.7 KB       |
| vehicle_imu               | 8       | 16      | 2x      | +0.3 KB       |
| sensor_combined           | 8       | 32      | 4x      | +0.8 KB       |
| vehicle_attitude          | 8       | 16      | 2x      | +0.2 KB       |
| vehicle_attitude_setpoint | 8       | 16      | 2x      | +0.2 KB       |
| vehicle_rates_setpoint    | 8       | 16      | 2x      | +0.2 KB       |
| actuator_motors           | 8       | 16      | 2x      | +0.2 KB       |
| **总计**                  | -       | -       | -       | **+5.9 KB**   |

**内存代价**：增加约 6 KB（可接受）

---

## 五、实现步骤

### 步骤 1: 创建精简消息定义 ✅ 已完成

已创建 8 个精简消息文件：
- `msg/LogGyroFifo.msg`
- `msg/LogAngularVelocity.msg`
- `msg/LogVehicleImu.msg`
- `msg/LogSensorCombined.msg`
- `msg/LogAttitude.msg`
- `msg/LogAttitudeSetpoint.msg`
- `msg/LogRatesSetpoint.msg`
- `msg/LogActuatorMotors.msg`

### 步骤 2: 修改 Logger 配置

修改 `src/modules/logger/logged_topics.cpp`，使用精简消息：

```cpp
void LoggedTopics::add_ermao_topics()
{
    // 使用精简版消息（Log* 系列）
    add_optional_topic("log_gyro_fifo");
    add_topic("log_angular_velocity");
    add_topic("log_vehicle_imu");
    add_topic("log_sensor_combined");
    add_topic("log_attitude");
    add_topic("log_attitude_setpoint");
    add_topic("log_rates_setpoint");
    add_topic("log_actuator_motors");
}
```

### 步骤 3: 修改发布者代码（关键步骤）

需要在原始消息的发布者处同时发布精简消息。

#### 3.1 sensor_gyro_fifo → log_gyro_fifo

修改 IMU 驱动（例如 `src/drivers/imu/bosch/bmi270/BMI270.cpp`）

#### 3.2 vehicle_angular_velocity → log_angular_velocity

修改 `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`

#### 3.3 其他主题类似

---

## 六、优势总结

| 优势               | 说明                                      |
|-------------------|-------------------------------------------|
| ✅ 数据不丢失      | 队列深度增加 2-8 倍，容忍更长延迟         |
| ✅ 数据量减少 45%  | 减轻 SD 卡写入压力，延长 SD 卡寿命        |
| ✅ 内存增加仅 6 KB | 代价很小                                  |
| ✅ 全速率记录      | 按原始发布频率记录，不丢失时间分辨率      |
| ✅ 后处理简化      | 去除冗余字段，分析代码更简洁              |
| ✅ 兼容性好        | 不影响原有控制代码，只是新增日志消息      |

---

## 七、后续工作

1. ✅ **消息定义**：已完成
2. ⏳ **修改 Logger 配置**：需要更新 `logged_topics.cpp`
3. ⏳ **修改发布者代码**：需要在各个模块中添加精简消息发布
4. ⏳ **编译测试**：验证消息生成和编译
5. ⏳ **仿真测试**：验证数据记录是否完整
6. ⏳ **真机测试**：验证实际飞行场景

---

**文档版本**：1.0
**最后更新**：2025-11-01
**状态**：第一步已完成（消息定义）

