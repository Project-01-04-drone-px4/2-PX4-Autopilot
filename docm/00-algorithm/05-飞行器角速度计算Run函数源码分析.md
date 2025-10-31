# VehicleAngularVelocity::Run() 函数详细分析

## 概述

`VehicleAngularVelocity::Run()` 是 PX4 角速度处理模块的核心执行函数，负责：
- 从传感器获取陀螺仪原始数据
- 应用滤波器处理
- 进行校准和偏差补偿
- 发布处理后的角速度数据

该函数位于：`src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` (784-919行)

---

## 函数整体结构

```cpp
void VehicleAngularVelocity::Run()
```

该函数作为 ScheduledWorkItem 的回调函数，在调度器触发时执行。

---

## 逐行代码分析

### 1. 性能计数开始 (786行)

```cpp
perf_begin(_cycle_perf);
```

**功能**：启动性能计数器，用于测量整个 Run() 函数的执行时间。

**说明**：
- `_cycle_perf` 是性能计数器对象
- 用于性能分析和调试
- 对应的 `perf_end()` 会在函数退出前调用

---

### 2. 备份调度 (789行)

```cpp
ScheduleDelayed(10_ms);
```

**功能**：安排一个备份调度，在 10 毫秒后再次执行 Run() 函数。

**说明**：
- 这是一个兜底机制，防止回调机制失效
- 正常情况下，函数通过传感器数据回调触发
- 如果回调未触发，10ms 后会强制执行一次
- `10_ms` 是 PX4 的时间字面量（用户定义字面量）

---

### 3. 获取当前时间 (791行)

```cpp
const hrt_abstime time_now_us = hrt_absolute_time();
```

**功能**：获取当前的高精度绝对时间（微秒）。

**说明**：
- `hrt_abstime` 是高精度时间戳类型（通常是 uint64_t）
- `hrt_absolute_time()` 返回系统启动以来的微秒数
- 用于后续的超时检查和时间戳计算

---

### 4. 参数更新 (793行)

```cpp
ParametersUpdate();
```

**功能**：检查并更新参数配置。

**说明**：
- 检查 `_parameter_update_sub` 订阅是否有更新
- 更新滤波器参数（低通、陷波滤波器等）
- 更新采样率配置（`IMU_GYRO_RATEMAX`）
- 如果参数变化可能触发 `_reset_filters = true`

**相关参数**：
- `IMU_GYRO_CUTOFF`：角速度低通滤波器截止频率
- `IMU_GYRO_NF0_FRQ`/`IMU_GYRO_NF0_BW`：陷波滤波器 0 的频率和带宽
- `IMU_GYRO_NF1_FRQ`/`IMU_GYRO_NF1_BW`：陷波滤波器 1 的频率和带宽
- `IMU_DGYRO_CUTOFF`：角加速度低通滤波器截止频率
- `IMU_GYRO_RATEMAX`：最大发布频率
- `IMU_GYRO_DNF_EN`：动态陷波滤波器使能

---

### 5. 传感器选择更新 (796行)

```cpp
const bool selection_updated = SensorSelectionUpdate(time_now_us);
```

**功能**：更新传感器选择，确保使用正确的陀螺仪传感器。

**说明**：
- 检查 `sensor_selection` 话题，获取系统选择的陀螺仪设备 ID
- 如果选择的传感器改变，会切换订阅
- 优先使用 FIFO 数据（`sensor_gyro_fifo`），否则使用普通数据（`sensor_gyro`）
- 传感器切换时会重置滤波器

**返回值**：
- `true`：传感器选择发生了变化
- `false`：传感器选择未变化

---

### 6. 采样率更新检查 (798-804行)

```cpp
if (selection_updated || _update_sample_rate) {
    if (!UpdateSampleRate()) {
        // sensor sample rate required to run
        perf_end(_cycle_perf);
        return;
    }
}
```

**功能**：在传感器切换或需要更新采样率时，更新采样率配置。

**详细说明**：

**条件触发**：
- `selection_updated`：传感器选择发生变化
- `_update_sample_rate`：采样率需要更新的标志

**UpdateSampleRate() 执行内容**：
- 从 `vehicle_imu_status` 获取传感器的实际采样率
- 计算 `_filter_sample_rate_hz`（滤波器工作频率）
- 根据 `IMU_GYRO_RATEMAX` 参数计算需要的采样数
- 设置订阅的 `required_updates`（批量处理样本数）
- 计算 `_publish_interval_min_us`（最小发布间隔）

**失败处理**（800-803行）：
- 如果无法获取有效的采样率，函数直接返回
- 结束性能计数并退出
- 不进行数据处理

---

### 7. 传感器校准更新 (806行)

```cpp
_calibration.SensorCorrectionsUpdate(selection_updated);
```

**功能**：更新传感器校准和修正参数。

**说明**：
- `_calibration` 是 `Calibration` 对象，管理传感器校准
- 包含：零偏、缩放因子、旋转矩阵
- `selection_updated` 为 true 时强制更新
- 从 `sensor_correction` 话题获取校准数据

**校准内容**：
- 温度补偿
- 工厂校准参数
- 板级旋转矩阵
- 传感器安装方向

---

### 8. 传感器偏差更新 (808行)

```cpp
SensorBiasUpdate(selection_updated);
```

**功能**：更新估计器提供的陀螺仪偏差。

**说明**：
- 从 `estimator_sensor_bias` 话题获取实时偏差估计
- 这是 EKF 估计的传感器偏差
- 用于补偿传感器漂移
- 传感器切换时强制更新
- 偏差存储在 `_bias` 成员变量中

**偏差来源**：
- EKF2 估计器实时计算
- 包含零偏漂移、温度影响等

---

### 9. 滤波器重置 (810-818行)

```cpp
if (_reset_filters) {
    ResetFilters(time_now_us);

    if (_reset_filters) {
        // not safe to run until filters configured
        perf_end(_cycle_perf);
        return;
    }
}
```

**功能**：在需要时重置所有滤波器。

**详细说明**：

**触发条件**（`_reset_filters = true`）：
- 传感器选择发生变化
- 采样率发生变化（>1% 误差）
- 滤波器参数改变

**ResetFilters() 执行内容**（811行）：
1. 重置角速度低通滤波器（3轴）
2. 重置陷波滤波器 0 和 1（3轴）
3. 重置角加速度低通滤波器（3轴）
4. 重置动态陷波滤波器（ESC RPM 和 FFT）
5. 使用上次发布的值作为初始状态

**安全检查**（813-817行）：
- 如果重置后 `_reset_filters` 仍为 true，说明配置失败
- 函数直接返回，不处理数据
- 确保滤波器配置正确后才运行

---

### 10. 动态陷波滤波器更新 - ESC RPM (820行)

```cpp
UpdateDynamicNotchEscRpm(time_now_us);
```

**功能**：更新基于 ESC RPM 的动态陷波滤波器。

**说明**（仅在非受限闪存版本中启用）：
- 从 `esc_status` 话题获取电机转速
- 根据 RPM 计算振动频率
- 为每个 ESC 的多个谐波创建陷波滤波器
- 动态跟踪电机转速变化

**工作原理**：
- ESC 转速（RPM）→ 频率（Hz）：`freq = RPM / 60`
- 支持多个谐波（`IMU_GYRO_DNF_HMC` 参数）
- 每个 ESC、每个谐波、每个轴都有独立的滤波器
- 转速低于阈值时，滤波器"停驻"在最小频率
- 超时时逐步禁用滤波器

**用途**：
- 消除电机振动引起的陀螺仪噪声
- 提高姿态估计精度

---

### 11. 动态陷波滤波器更新 - FFT (821行)

```cpp
UpdateDynamicNotchFFT(time_now_us);
```

**功能**：更新基于 FFT 分析的动态陷波滤波器。

**说明**（仅在非受限闪存版本中启用）：
- 从 `sensor_gyro_fft` 话题获取频谱分析结果
- FFT 识别陀螺仪数据中的峰值频率
- 为检测到的峰值频率创建陷波滤波器

**工作原理**：
- 使用 FFT 分析陀螺仪数据频谱
- 识别最显著的振动频率（峰值）
- 每个轴支持多个峰值（`MAX_NUM_FFT_PEAKS`）
- 动态调整陷波频率跟踪振动变化

**用途**：
- 自动检测和消除未知振动源
- 无需手动配置振动频率
- 适用于机架共振等复杂情况

---

### 12. FIFO 数据处理分支 (823-865行)

```cpp
if (_fifo_available) {
    // process all outstanding fifo messages
    ...
}
```

**功能**：处理 FIFO 模式的传感器数据。

**说明**：
- FIFO（First In First Out）模式：传感器缓存多个样本后一次性传输
- 相比单次采样，FIFO 可以减少中断次数，提高效率
- 适用于高采样率传感器（如 BMI270）

---

#### 12.1 FIFO 循环处理 (825-829行)

```cpp
int sensor_sub_updates = 0;
sensor_gyro_fifo_s sensor_fifo_data;

while ((sensor_sub_updates < sensor_gyro_fifo_s::ORB_QUEUE_LENGTH) && _sensor_gyro_fifo_sub.update(&sensor_fifo_data)) {
    sensor_sub_updates++;
```

**功能**：循环处理所有待处理的 FIFO 消息。

**说明**：
- `sensor_sub_updates`：计数器，限制单次处理的消息数
- `sensor_gyro_fifo_s::ORB_QUEUE_LENGTH`：队列最大长度（通常为 8）
- `_sensor_gyro_fifo_sub.update()`：获取下一条 FIFO 数据
- 循环直到处理完所有数据或达到限制

---

#### 12.2 计算采样间隔和样本数 (831-833行)

```cpp
const float inverse_dt_s = 1e6f / sensor_fifo_data.dt;
const int N = sensor_fifo_data.samples;
static constexpr int FIFO_SIZE_MAX = sizeof(sensor_fifo_data.x) / sizeof(sensor_fifo_data.x[0]);
```

**功能**：提取 FIFO 数据的关键参数。

**说明**：
- `inverse_dt_s`：采样频率（Hz）
  - `sensor_fifo_data.dt`：样本间隔（微秒）
  - `inverse_dt_s = 1,000,000 / dt`（转换为 Hz）
- `N`：本次 FIFO 中的样本数量
- `FIFO_SIZE_MAX`：FIFO 缓冲区最大容量

---

#### 12.3 数据有效性检查 (835行)

```cpp
if ((sensor_fifo_data.dt > 0) && (N > 0) && (N <= FIFO_SIZE_MAX)) {
```

**功能**：验证 FIFO 数据的有效性。

**检查项**：
- `dt > 0`：采样间隔有效
- `N > 0`：至少有一个样本
- `N <= FIFO_SIZE_MAX`：样本数不超过缓冲区大小

---

#### 12.4 定义变量 (836-839行)

```cpp
Vector3f angular_velocity_uncalibrated;
Vector3f angular_acceleration_uncalibrated;

int16_t *raw_data_array[] {sensor_fifo_data.x, sensor_fifo_data.y, sensor_fifo_data.z};
```

**功能**：准备数据处理的变量。

**说明**：
- `angular_velocity_uncalibrated`：存储滤波后的未校准角速度（3轴）
- `angular_acceleration_uncalibrated`：存储滤波后的未校准角加速度（3轴）
- `raw_data_array`：指针数组，分别指向 X、Y、Z 轴的原始数据
  - FIFO 数据格式：`int16_t x[N], y[N], z[N]`

---

#### 12.5 三轴数据处理循环 (841-852行)

```cpp
for (int axis = 0; axis < 3; axis++) {
    // copy raw int16 sensor samples to float array for filtering
    float data[FIFO_SIZE_MAX];

    for (int n = 0; n < N; n++) {
        data[n] = sensor_fifo_data.scale * raw_data_array[axis][n];
    }

    // save last filtered sample
    angular_velocity_uncalibrated(axis) = FilterAngularVelocity(axis, data, N);
    angular_acceleration_uncalibrated(axis) = FilterAngularAcceleration(axis, inverse_dt_s, data, N);
}
```

**功能**：对 X、Y、Z 三个轴分别进行数据处理。

**详细步骤**：

**1. 数据类型转换和缩放（843-847行）**：
- 创建浮点数组 `data[FIFO_SIZE_MAX]`
- 遍历 N 个样本
- 将 `int16_t` 原始数据转换为物理单位（rad/s）
- 公式：`data[n] = scale × raw_data[n]`
  - `scale`：缩放因子（通常为满量程/32768）

**2. 角速度滤波（850行）**：
```cpp
angular_velocity_uncalibrated(axis) = FilterAngularVelocity(axis, data, N);
```
- 对数组中的 N 个样本批量应用滤波器
- 滤波器链（顺序）：
  1. 动态陷波滤波器（ESC RPM，多个）
  2. 动态陷波滤波器（FFT，多个）
  3. 陷波滤波器 0（`IMU_GYRO_NF0_FRQ`）
  4. 陷波滤波器 1（`IMU_GYRO_NF1_FRQ`）
  5. 低通滤波器（`IMU_GYRO_CUTOFF`）
- 返回最后一个滤波后的样本值

**3. 角加速度滤波（851行）**：
```cpp
angular_acceleration_uncalibrated(axis) = FilterAngularAcceleration(axis, inverse_dt_s, data, N);
```
- 对滤波后的角速度数据进行微分
- 应用角加速度低通滤波器（`IMU_DGYRO_CUTOFF`）
- 更新 `_angular_velocity_raw_prev` 用于下次微分
- 返回滤波后的角加速度

---

#### 12.6 数据发布 (854-863行)

```cpp
// Publish
if (!_sensor_gyro_fifo_sub.updated()) {
    if (CalibrateAndPublish(sensor_fifo_data.timestamp_sample,
                angular_velocity_uncalibrated,
                angular_acceleration_uncalibrated)) {

        perf_end(_cycle_perf);
        return;
    }
}
```

**功能**：在没有新数据时发布处理结果。

**详细说明**：

**条件检查（855行）**：
```cpp
if (!_sensor_gyro_fifo_sub.updated())
```
- 检查是否还有待处理的 FIFO 数据
- 如果有新数据，继续循环处理
- 如果没有新数据，尝试发布

**校准和发布（856-858行）**：
```cpp
if (CalibrateAndPublish(...))
```
调用 `CalibrateAndPublish()` 函数：
1. 检查发布间隔（`_publish_interval_min_us`）
2. 应用校准：`_calibration.Correct()`
3. 减去偏差：`- _bias`
4. 旋转到飞控坐标系
5. 发布 `vehicle_angular_velocity` 话题

**成功返回（860-862行）**：
- 如果成功发布，结束性能计数
- 退出 `Run()` 函数
- 等待下次调度

---

### 13. 非 FIFO 数据处理分支 (867-911行)

```cpp
} else {
    // process all outstanding messages
    int sensor_sub_updates = 0;
    sensor_gyro_s sensor_data;

    while ((sensor_sub_updates < sensor_gyro_s::ORB_QUEUE_LENGTH) && _sensor_sub.update(&sensor_data)) {
        ...
    }
}
```

**功能**：处理非 FIFO 模式的传感器数据（单次采样模式）。

**说明**：
- 用于不支持 FIFO 的传感器
- 每次只包含一个样本
- 处理流程类似 FIFO，但只处理单个数据点

---

#### 13.1 循环处理消息 (868-873行)

```cpp
int sensor_sub_updates = 0;
sensor_gyro_s sensor_data;

while ((sensor_sub_updates < sensor_gyro_s::ORB_QUEUE_LENGTH) && _sensor_sub.update(&sensor_data)) {
    sensor_sub_updates++;
```

**功能**：循环处理所有待处理的单次采样数据。

**说明**：
- 与 FIFO 处理类似的循环结构
- `sensor_gyro_s`：单次采样数据结构
- 限制最大处理数量防止阻塞

---

#### 13.2 数据有效性检查 (875行)

```cpp
if (Vector3f(sensor_data.x, sensor_data.y, sensor_data.z).isAllFinite()) {
```

**功能**：检查三轴数据是否都是有限值（非 NaN、非 Inf）。

**说明**：
- `isAllFinite()`：matrix 库函数
- 防止无效数据进入处理流程
- 保护后续滤波和计算

---

#### 13.3 时间戳初始化和修正 (877-879行)

```cpp
if (_timestamp_sample_last == 0 || (sensor_data.timestamp_sample <= _timestamp_sample_last)) {
    _timestamp_sample_last = sensor_data.timestamp_sample - 1e6f / _filter_sample_rate_hz;
}
```

**功能**：处理时间戳异常情况。

**说明**：

**条件 1**：`_timestamp_sample_last == 0`
- 第一次运行，没有历史时间戳
- 估算上一个样本的时间戳

**条件 2**：`sensor_data.timestamp_sample <= _timestamp_sample_last`
- 时间戳回退（异常情况）
- 时钟同步问题或传感器重启

**修正方法**：
- 基于采样率估算：`当前时间 - (1秒 / 采样率)`
- 例如：1000 Hz 采样率 → 减去 1000 微秒

---

#### 13.4 计算实际采样间隔 (881-883行)

```cpp
const float inverse_dt_s = 1.f / math::constrain(((sensor_data.timestamp_sample - _timestamp_sample_last) * 1e-6f),
                   0.00002f, 0.02f);
_timestamp_sample_last = sensor_data.timestamp_sample;
```

**功能**：计算实际的采样频率。

**详细说明**：

**时间差计算**：
- `sensor_data.timestamp_sample - _timestamp_sample_last`：微秒
- `× 1e-6f`：转换为秒

**限制范围**：
- 最小：`0.00002f` 秒（50 kHz，防止除零和过高频率）
- 最大：`0.02f` 秒（50 Hz，防止异常低频）

**逆采样间隔**：
- `inverse_dt_s = 1 / dt`：频率（Hz）
- 用于角加速度计算

**更新时间戳**：
- 保存当前时间戳供下次使用

---

#### 13.5 数据准备 (885-888行)

```cpp
Vector3f angular_velocity_uncalibrated;
Vector3f angular_acceleration_uncalibrated;

float raw_data_array[] {sensor_data.x, sensor_data.y, sensor_data.z};
```

**功能**：准备单次采样的数据处理。

**说明**：
- 创建结果变量
- 将三轴数据组织为数组
- 数据已经是浮点格式（与 FIFO 的 int16 不同）

---

#### 13.6 三轴滤波处理 (890-897行)

```cpp
for (int axis = 0; axis < 3; axis++) {
    // copy sensor sample to float array for filtering
    float data[1] {raw_data_array[axis]};

    // save last filtered sample
    angular_velocity_uncalibrated(axis) = FilterAngularVelocity(axis, data);
    angular_acceleration_uncalibrated(axis) = FilterAngularAcceleration(axis, inverse_dt_s, data);
}
```

**功能**：对单个样本应用滤波器链。

**详细说明**：

**数据包装（892行）**：
```cpp
float data[1] {raw_data_array[axis]};
```
- 创建长度为 1 的数组
- 符合滤波函数接口（接受数组）
- 默认参数 N=1

**角速度滤波（895行）**：
- 调用 `FilterAngularVelocity(axis, data)`
- 应用完整的滤波器链（同 FIFO）
- 返回滤波后的单个值

**角加速度滤波（896行）**：
- 调用 `FilterAngularAcceleration(axis, inverse_dt_s, data)`
- 使用前一个样本计算微分
- 应用低通滤波

---

#### 13.7 数据发布 (899-909行)

```cpp
// Publish
if (!_sensor_sub.updated()) {
    if (CalibrateAndPublish(sensor_data.timestamp_sample,
                angular_velocity_uncalibrated,
                angular_acceleration_uncalibrated)) {

        perf_end(_cycle_perf);
        return;
    }
}
```

**功能**：发布处理后的数据。

**说明**：
- 与 FIFO 分支完全相同的发布逻辑
- 检查是否还有待处理数据
- 调用校准和发布函数
- 成功后退出

---

### 14. 传感器超时处理 (913-916行)

```cpp
// force reselection on timeout
if (time_now_us > _last_publish + 500_ms) {
    SensorSelectionUpdate(time_now_us, true);
}
```

**功能**：处理传感器数据超时。

**详细说明**：

**超时检测**：
- 检查距离上次发布的时间
- 阈值：500 毫秒
- `_last_publish`：上次成功发布的时间戳

**超时处理**：
- 强制重新选择传感器：`SensorSelectionUpdate(time_now_us, true)`
- `force = true` 参数强制执行选择逻辑
- 可能切换到备用传感器
- 或重新初始化当前传感器

**目的**：
- 检测传感器失效
- 自动故障恢复
- 提高系统可靠性

---

### 15. 性能计数结束 (918行)

```cpp
perf_end(_cycle_perf);
```

**功能**：结束性能计数，记录本次执行时间。

**说明**：
- 与函数开始的 `perf_begin()` 配对
- 记录执行时间统计
- 用于性能分析和优化
- 可通过 `perf_print_counter()` 查看

---

## 函数执行流程图

```
开始 Run()
    ↓
[1] perf_begin() ─────────────────────→ 性能计数开始
    ↓
[2] ScheduleDelayed(10ms) ────────────→ 设置备份调度
    ↓
[3] time_now_us = hrt_absolute_time() → 获取当前时间
    ↓
[4] ParametersUpdate() ───────────────→ 更新参数
    ↓
[5] SensorSelectionUpdate() ──────────→ 更新传感器选择
    ↓
[6] 需要更新采样率? ──No──→ 跳过
    │
    └─Yes→ UpdateSampleRate()
           失败? ──Yes──→ [返回]
           │
           └─No─→ 继续
    ↓
[7] SensorCorrectionsUpdate() ────────→ 更新校准参数
    ↓
[8] SensorBiasUpdate() ───────────────→ 更新偏差估计
    ↓
[9] 需要重置滤波器? ──No──→ 跳过
    │
    └─Yes→ ResetFilters()
           失败? ──Yes──→ [返回]
           │
           └─No─→ 继续
    ↓
[10] UpdateDynamicNotchEscRpm() ──────→ 更新 ESC RPM 滤波器
    ↓
[11] UpdateDynamicNotchFFT() ─────────→ 更新 FFT 滤波器
    ↓
[12] FIFO 可用? ──Yes──→ FIFO 分支
    │                     │
    │                     ├─ 循环处理 FIFO 消息
    │                     ├─ 转换原始数据
    │                     ├─ 应用滤波器链
    │                     ├─ 计算角加速度
    │                     └─ 校准并发布
    │
    └─No─→ 非 FIFO 分支
              │
              ├─ 循环处理单次采样
              ├─ 检查数据有效性
              ├─ 计算采样间隔
              ├─ 应用滤波器链
              ├─ 计算角加速度
              └─ 校准并发布
    ↓
[14] 超时检查 (>500ms)
    │
    └─ 超时? ──Yes──→ 强制重新选择传感器
       │
       └─No─→ 继续
    ↓
[15] perf_end() ──────────────────────→ 性能计数结束
    ↓
结束 Run()
```

---

## 关键数据流

### 原始数据 → 最终输出

```
[传感器原始数据]
    ↓ (FIFO: int16_t → float × scale)
    ↓ (非FIFO: 已是 float)
[未校准角速度原始数据]
    ↓
[动态陷波滤波器 - ESC RPM] (可选)
    ↓
[动态陷波滤波器 - FFT] (可选)
    ↓
[陷波滤波器 0] (IMU_GYRO_NF0)
    ↓
[陷波滤波器 1] (IMU_GYRO_NF1)
    ↓
[低通滤波器] (IMU_GYRO_CUTOFF)
    ↓
[滤波后的未校准角速度]
    ↓ (Calibration::Correct)
[应用校准参数]
    ↓ (减去偏差)
[已校准角速度] ────────→ 发布到 vehicle_angular_velocity.xyz
    ↓
[计算角加速度] (微分)
    ↓
[角加速度低通滤波器] (IMU_DGYRO_CUTOFF)
    ↓
[滤波后的角加速度] ────→ 发布到 vehicle_angular_velocity.xyz_derivative
```

---

## 滤波器链详解

### 角速度滤波器链（按应用顺序）

1. **动态陷波滤波器 - ESC RPM**（多个）
   - 输入：ESC 转速（RPM）
   - 频率：动态，跟踪电机转速
   - 谐波：可配置（`IMU_GYRO_DNF_HMC`）
   - 目的：消除电机振动

2. **动态陷波滤波器 - FFT**（多个峰值）
   - 输入：FFT 频谱分析结果
   - 频率：动态，跟踪检测到的峰值
   - 目的：消除未知振动源

3. **陷波滤波器 0**（静态）
   - 频率：`IMU_GYRO_NF0_FRQ`
   - 带宽：`IMU_GYRO_NF0_BW`
   - 目的：消除已知固定频率振动

4. **陷波滤波器 1**（静态）
   - 频率：`IMU_GYRO_NF1_FRQ`
   - 带宽：`IMU_GYRO_NF1_BW`
   - 目的：消除另一个已知固定频率振动

5. **低通滤波器**
   - 截止频率：`IMU_GYRO_CUTOFF`
   - 目的：去除高频噪声

### 角加速度滤波器

1. **微分计算**
   - 公式：`(当前角速度 - 上次角速度) / dt`
   - 输入：滤波后的角速度

2. **低通滤波器**
   - 截止频率：`IMU_DGYRO_CUTOFF`
   - 目的：平滑角加速度信号

---

## 重要成员变量

### 时间相关
- `_last_publish`：上次发布时间戳（微秒）
- `_timestamp_sample_last`：上次样本时间戳（微秒）
- `_publish_interval_min_us`：最小发布间隔（微秒）

### 传感器相关
- `_selected_sensor_device_id`：当前选择的传感器设备 ID
- `_fifo_available`：是否使用 FIFO 模式
- `_filter_sample_rate_hz`：滤波器工作采样率（Hz）

### 滤波器相关
- `_lp_filter_velocity[3]`：角速度低通滤波器（3轴）
- `_notch_filter0_velocity[3]`：陷波滤波器 0（3轴）
- `_notch_filter1_velocity[3]`：陷波滤波器 1（3轴）
- `_lp_filter_acceleration[3]`：角加速度低通滤波器（3轴）
- `_dynamic_notch_filter_esc_rpm`：ESC RPM 动态陷波滤波器数组
- `_dynamic_notch_filter_fft[3][MAX_NUM_FFT_PEAKS]`：FFT 动态陷波滤波器

### 校准和偏差
- `_calibration`：校准对象（零偏、缩放、旋转）
- `_bias`：实时估计的陀螺仪偏差（Vector3f）

### 输出数据
- `_angular_velocity`：校准后的角速度（Vector3f，rad/s）
- `_angular_acceleration`：滤波后的角加速度（Vector3f，rad/s²）
- `_angular_velocity_raw_prev`：上次原始角速度（用于微分）

### 标志位
- `_reset_filters`：是否需要重置滤波器
- `_update_sample_rate`：是否需要更新采样率

---

## 性能优化要点

### 1. 批量处理
- FIFO 模式一次处理多个样本
- 减少函数调用开销
- 提高缓存利用率

### 2. 早期返回
- 参数无效时立即返回
- 避免不必要的计算
- 保护系统稳定性

### 3. 限制循环次数
- `sensor_sub_updates < ORB_QUEUE_LENGTH`
- 防止单次执行时间过长
- 保证实时性

### 4. 滤波器状态保持
- 滤波器对象在类成员中
- 状态在调用间保持
- 避免重复初始化

---

## 常见问题和调试

### 1. 数据不更新
**可能原因**：
- 传感器超时（检查 500ms 超时）
- 采样率更新失败（检查 `UpdateSampleRate()`）
- 滤波器重置失败（检查 `_reset_filters`）

**调试方法**：
- 检查 `_last_publish` 时间戳
- 查看性能计数器（`perf_print_counter`）
- 检查传感器选择（`_selected_sensor_device_id`）

### 2. 数据跳变
**可能原因**：
- 时间戳回退（检查 877-879 行）
- 传感器切换（检查 selection_updated）
- 滤波器重置（检查 ResetFilters 调用）

**调试方法**：
- 监控时间戳连续性
- 检查传感器选择变化
- 查看滤波器重置计数

### 3. 高延迟
**可能原因**：
- 队列积压（`sensor_sub_updates` 达到上限）
- 滤波器计算量大（动态陷波滤波器过多）
- 发布间隔过小（`_publish_interval_min_us` 设置）

**调试方法**：
- 检查性能计数器执行时间
- 减少动态陷波滤波器数量
- 调整 `IMU_GYRO_RATEMAX` 参数

---

## 相关参数总结

| 参数名称 | 类型 | 默认值 | 说明 |
|---------|------|--------|------|
| `IMU_GYRO_RATEMAX` | INT32 | 400 | 最大发布频率（Hz），50-10000 |
| `IMU_GYRO_CUTOFF` | FLOAT | 30.0 | 角速度低通滤波器截止频率（Hz）|
| `IMU_GYRO_NF0_FRQ` | FLOAT | 0.0 | 陷波滤波器 0 频率（Hz），0=禁用 |
| `IMU_GYRO_NF0_BW` | FLOAT | 0.0 | 陷波滤波器 0 带宽（Hz）|
| `IMU_GYRO_NF1_FRQ` | FLOAT | 0.0 | 陷波滤波器 1 频率（Hz），0=禁用 |
| `IMU_GYRO_NF1_BW` | FLOAT | 0.0 | 陷波滤波器 1 带宽（Hz）|
| `IMU_DGYRO_CUTOFF` | FLOAT | 30.0 | 角加速度低通滤波器截止频率（Hz）|
| `IMU_GYRO_DNF_EN` | INT32 | 0 | 动态陷波滤波器使能（位掩码）|
| `IMU_GYRO_DNF_HMC` | INT32 | 3 | ESC RPM 谐波数量（1-10）|
| `IMU_GYRO_DNF_BW` | FLOAT | 15.0 | 动态陷波滤波器带宽（Hz）|
| `IMU_GYRO_DNF_MIN` | FLOAT | 25.0 | 动态陷波滤波器最小频率（Hz）|

---

## 总结

`VehicleAngularVelocity::Run()` 函数是一个高度优化的实时数据处理管道，主要特点：

1. **模块化设计**：参数更新、传感器选择、滤波、校准、发布各自独立
2. **鲁棒性**：多层数据验证，超时保护，故障恢复
3. **灵活性**：支持 FIFO 和非 FIFO 模式，动态和静态滤波器
4. **性能优化**：批量处理，早期返回，状态保持
5. **可调试性**：性能计数，状态跟踪，完整日志

该函数是 PX4 姿态估计的关键组件，为飞控系统提供高质量的角速度数据。

