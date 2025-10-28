# 动态陷波滤波器配置详解

## 概述

本文档详细说明 PX4 中陀螺仪动态陷波滤波器的配置、参数和工作原理。

**相关文件**：
- `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`
- `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.hpp`
- `src/modules/sensors/vehicle_angular_velocity/imu_gyro_parameters.c`

---

## 一、陀螺仪滤波器架构全景

### 1.1 完整滤波器链（按应用顺序）

```
原始陀螺仪数据
    ↓
[1] 动态陷波滤波器 - ESC RPM (多个，可选)
    ↓
[2] 动态陷波滤波器 - FFT (多个，可选)
    ↓
[3] 静态陷波滤波器 0 (单个，可选)
    ↓
[4] 静态陷波滤波器 1 (单个，可选)
    ↓
[5] 低通滤波器 (单个，必有)
    ↓
滤波后的角速度
```

**代码实现**（`VehicleAngularVelocity.cpp:725-768`）：

```cpp
float VehicleAngularVelocity::FilterAngularVelocity(int axis, float data[], int N)
{
#if !defined(CONSTRAINED_FLASH)
    // [1] 动态陷波 - ESC RPM
    if (_dynamic_notch_filter_esc_rpm) {
        for (int esc = 0; esc < MAX_NUM_ESCS; esc++) {
            if (_esc_available[esc]) {
                for (int harmonic = 0; harmonic < _esc_rpm_harmonics; harmonic++) {
                    if (_dynamic_notch_filter_esc_rpm[harmonic][axis][esc].getNotchFreq() > 0.f) {
                        _dynamic_notch_filter_esc_rpm[harmonic][axis][esc].applyArray(data, N);
                    }
                }
            }
        }
    }

    // [2] 动态陷波 - FFT
    if (_dynamic_notch_fft_available) {
        for (int peak = MAX_NUM_FFT_PEAKS - 1; peak >= 0; peak--) {
            if (_dynamic_notch_filter_fft[axis][peak].getNotchFreq() > 0.f) {
                _dynamic_notch_filter_fft[axis][peak].applyArray(data, N);
            }
        }
    }
#endif

    // [3] 静态陷波 0
    if (_notch_filter0_velocity[axis].getNotchFreq() > 0.f) {
        _notch_filter0_velocity[axis].applyArray(data, N);
    }

    // [4] 静态陷波 1
    if (_notch_filter1_velocity[axis].getNotchFreq() > 0.f) {
        _notch_filter1_velocity[axis].applyArray(data, N);
    }

    // [5] 低通滤波器
    _lp_filter_velocity[axis].applyArray(data, N);

    return data[N - 1];
}
```

---

### 1.2 滤波器对照表

| # | 滤波器名称 | 类型 | 数量 | 默认状态 | 控制参数 | 用途 |
|---|-----------|------|------|---------|----------|------|
| 1 | **动态陷波 - ESC RPM** | 动态 | ESC数 × 谐波数 × 3轴 | ❌ 禁用 | `IMU_GYRO_DNF_EN` bit 0 | 消除电机振动 |
| 2 | **动态陷波 - FFT** | 动态 | 峰值数 × 3轴 | ❌ 禁用 | `IMU_GYRO_DNF_EN` bit 1 | 消除未知振动源 |
| 3 | **静态陷波 0** | 静态 | 1 × 3轴 | ❌ 禁用 | `IMU_GYRO_NF0_FRQ` | 消除固定频率振动 |
| 4 | **静态陷波 1** | 静态 | 1 × 3轴 | ❌ 禁用 | `IMU_GYRO_NF1_FRQ` | 消除另一固定频率 |
| 5 | **低通滤波器** | 静态 | 1 × 3轴 | ✅ **启用** | `IMU_GYRO_CUTOFF` | 去除高频噪声 |

---

## 二、动态陷波滤波器参数

### 2.1 主控制参数

#### IMU_GYRO_DNF_EN（主开关）

```c
// src/modules/sensors/vehicle_angular_velocity/imu_gyro_parameters.c:185
/**
* IMU gyro dynamic notch filtering
*
* Enable bank of dynamically updating notch filters.
* Requires ESC RPM feedback or onboard FFT (IMU_GYRO_FFT_EN).
* @group Sensors
* @min 0
* @max 3
* @bit 0 ESC RPM
* @bit 1 FFT
*/
PARAM_DEFINE_INT32(IMU_GYRO_DNF_EN, 0);
```

**位掩码定义**：

```cpp
// VehicleAngularVelocity.hpp:138-141
enum DynamicNotch {
    EscRpm = 1,  // 0b01
    FFT    = 2,  // 0b10
};
```

**参数值含义**：

| 值 | 二进制 | Bit 0 | Bit 1 | ESC RPM | FFT | 说明 |
|----|--------|-------|-------|---------|-----|------|
| **0** | `00` | 0 | 0 | ❌ | ❌ | **两者都禁用**（默认） |
| **1** | `01` | 1 | 0 | ✅ | ❌ | **仅启用 ESC RPM** |
| **2** | `10` | 0 | 1 | ❌ | ✅ | **仅启用 FFT** |
| **3** | `11` | 1 | 1 | ✅ | ✅ | **两者都启用** |

---

### 2.2 通用参数

| 参数名 | 类型 | 默认值 | 范围 | 单位 | 说明 |
|--------|------|--------|------|------|------|
| `IMU_GYRO_DNF_EN` | INT32 | 0 | 0-3 | - | 动态陷波主开关（位掩码） |
| `IMU_GYRO_DNF_BW` | FLOAT | 15.0 | 5-30 | Hz | **陷波滤波器带宽** |
| `IMU_GYRO_DNF_HMC` | INT32 | 3 | 1-7 | - | **ESC RPM 谐波数量** |
| `IMU_GYRO_DNF_MIN` | FLOAT | 25.0 | - | Hz | **最低陷波频率** |

---

## 三、ESC RPM 动态陷波滤波器详解

### 3.1 工作原理

ESC RPM 动态陷波滤波器根据电机转速实时计算振动频率并创建陷波滤波器。

**数据流**：

```
ESC 遥测 (esc_status)
    ↓
RPM 值 (每个 ESC)
    ↓
转换为频率：esc_hz = RPM / 60
    ↓
计算谐波频率：freq = esc_hz × (1, 2, 3, ...)
    ↓
创建陷波滤波器（每个谐波、每个轴、每个 ESC）
    ↓
应用到陀螺仪数据
```

---

### 3.2 参数配置机制

#### 3.2.1 谐波数量（IMU_GYRO_DNF_HMC）

**定义**：

```c
// imu_gyro_parameters.c:210
/**
* IMU gyro dynamic notch filter harmonics
*
* ESC RPM number of harmonics (multiples of RPM) for ESC RPM dynamic notch filtering.
*
* @group Sensors
* @min 1
* @max 7
*/
PARAM_DEFINE_INT32(IMU_GYRO_DNF_HMC, 3);
```

**含义**：为每个 ESC 创建多少个谐波陷波滤波器。

**示例**（4 个电机，3 个谐波）：

```
电机 1: 2000 RPM → 33.3 Hz
  - 谐波 1: 33.3 Hz
  - 谐波 2: 66.6 Hz
  - 谐波 3: 99.9 Hz

电机 2: 2100 RPM → 35.0 Hz
  - 谐波 1: 35.0 Hz
  - 谐波 2: 70.0 Hz
  - 谐波 3: 105.0 Hz

... (电机 3, 4)

总滤波器数 = 4 ESC × 3 谐波 × 3 轴 = 36 个陷波滤波器
```

---

#### 3.2.2 中心频率计算

**代码实现**（`VehicleAngularVelocity.cpp:593-600`）：

```cpp
const float esc_hz = abs(esc_report.esc_rpm) / 60.f;

for (int harmonic = 0; harmonic < _esc_rpm_harmonics; harmonic++) {
    // 计算谐波频率
    const float frequency_hz = math::max(
        esc_hz * (harmonic + 1),                        // 基频 × 谐波次数
        freq_min + (harmonic * 0.5f * bandwidth_hz)     // 最小频率 + 谐波间隔
    );

    // ... 设置滤波器
}
```

**公式详解**：

```
基本谐波频率 = RPM / 60 × 谐波次数

谐波次数:
  harmonic = 0 → 1倍频（基频）
  harmonic = 1 → 2倍频（第一谐波）
  harmonic = 2 → 3倍频（第二谐波）
  ...

低转速保护:
  如果计算出的频率 < 最小频率，则使用：
  频率 = IMU_GYRO_DNF_MIN + (谐波次数 × 0.5 × 带宽)
```

**实际计算示例**：

假设：
- RPM = 1200
- IMU_GYRO_DNF_MIN = 25 Hz
- IMU_GYRO_DNF_BW = 15 Hz
- IMU_GYRO_DNF_HMC = 3

```
esc_hz = 1200 / 60 = 20 Hz

谐波 0 (基频):
  esc_hz × 1 = 20 Hz
  freq_min + 0 × 0.5 × 15 = 25 Hz
  → max(20, 25) = 25 Hz ✓

谐波 1 (2倍频):
  esc_hz × 2 = 40 Hz
  freq_min + 1 × 0.5 × 15 = 32.5 Hz
  → max(40, 32.5) = 40 Hz ✓

谐波 2 (3倍频):
  esc_hz × 3 = 60 Hz
  freq_min + 2 × 0.5 × 15 = 40 Hz
  → max(60, 40) = 60 Hz ✓
```

---

#### 3.2.3 带宽设置（IMU_GYRO_DNF_BW）

**定义**：

```c
// imu_gyro_parameters.c:199
/**
* IMU gyro ESC notch filter bandwidth
*
* Bandwidth per notch filter when using dynamic notch filtering with ESC RPM.
*
* @group Sensors
* @unit Hz
* @min 5
* @max 30
*/
PARAM_DEFINE_FLOAT(IMU_GYRO_DNF_BW, 15.f);
```

**应用**（`VehicleAngularVelocity.cpp:582-614`）：

```cpp
const float bandwidth_hz = _param_imu_gyro_dnf_bw.get();  // 读取参数

for (int axis = 0; axis < 3; axis++) {
    auto &nf = _dynamic_notch_filter_esc_rpm[harmonic][axis][esc];

    // 设置陷波滤波器参数
    nf.setParameters(
        _filter_sample_rate_hz,  // 采样率 (如 2000 Hz)
        frequency_hz,            // 中心频率（上面计算的）
        bandwidth_hz             // 带宽（从参数读取）
    );
}
```

**带宽的意义**：

```
陷波滤波器的频率响应：

增益
  ↑
1 |────┐            ┌────
  |    │            │
  |    │  ← 带宽 →  │
  |    └────────────┘
0 |___________________________→ 频率
      fc-BW/2  fc  fc+BW/2

fc: 中心频率
BW: 带宽（IMU_GYRO_DNF_BW）

衰减区间: [fc - BW/2, fc + BW/2]
```

**带宽选择指南**：

| 带宽值 | 衰减范围 | 优点 | 缺点 | 适用场景 |
|--------|---------|------|------|----------|
| 5 Hz | 窄 | 保留更多信号 | 可能漏掉振动 | 精确已知频率 |
| **15 Hz** | **中等** | **平衡** | - | **推荐默认** |
| 30 Hz | 宽 | 覆盖范围大 | 去除更多信号 | 宽带振动 |

---

#### 3.2.4 最低频率保护（IMU_GYRO_DNF_MIN）

**定义**：

```c
// imu_gyro_parameters.c:222
/**
* IMU gyro dynamic notch filter minimum frequency
*
* Minimum notch filter frequency in Hz.
*
* @group Sensors
* @unit Hz
*/
PARAM_DEFINE_FLOAT(IMU_GYRO_DNF_MIN, 25.f);
```

**作用**（`VehicleAngularVelocity.cpp:583-600`）：

```cpp
const float freq_min = math::max(_param_imu_gyro_dnf_min.get(), bandwidth_hz);

// 低转速时停驻在最小频率
const float frequency_hz = math::max(
    esc_hz * (harmonic + 1),
    freq_min + (harmonic * 0.5f * bandwidth_hz)  // 使用最小频率
);
```

**为什么需要最低频率**：

1. **避免过低频率**：
   - 控制回路频率通常 > 50 Hz
   - 陷波频率太低会影响控制性能

2. **悬停时的保护**：
   - 电机转速很低时（如着陆后）
   - 停驻在安全频率，而不是禁用滤波器

3. **谐波分离**：
   - 通过 `harmonic * 0.5f * bandwidth_hz` 间隔
   - 确保多个谐波不重叠

**示例**（低转速情况）：

```
RPM = 300 → esc_hz = 5 Hz
IMU_GYRO_DNF_MIN = 25 Hz
IMU_GYRO_DNF_BW = 15 Hz

不使用最低频率保护:
  谐波 1: 5 Hz   ← 太低！
  谐波 2: 10 Hz  ← 太低！
  谐波 3: 15 Hz  ← 太低！

使用最低频率保护:
  谐波 1: max(5, 25) = 25 Hz      ✓
  谐波 2: max(10, 32.5) = 32.5 Hz ✓
  谐波 3: max(15, 40) = 40 Hz     ✓
```

---

### 3.3 滤波器结构

#### 3.3.1 多维数组结构

```cpp
// VehicleAngularVelocity.hpp
NotchFilterHarmonic *_dynamic_notch_filter_esc_rpm;

// 实际是一个三维数组：
// _dynamic_notch_filter_esc_rpm[谐波][轴][ESC]

typedef NotchFilter<float> NotchFilterHarmonic[3][MAX_NUM_ESCS];
```

**数据结构示意**：

```
_dynamic_notch_filter_esc_rpm
├─ 谐波 0 (1倍频)
│  ├─ X 轴
│  │  ├─ ESC 0 → NotchFilter(freq, bw)
│  │  ├─ ESC 1 → NotchFilter(freq, bw)
│  │  ├─ ESC 2 → NotchFilter(freq, bw)
│  │  └─ ESC 3 → NotchFilter(freq, bw)
│  ├─ Y 轴 (同上)
│  └─ Z 轴 (同上)
├─ 谐波 1 (2倍频)
│  └─ ... (同上)
└─ 谐波 2 (3倍频)
   └─ ... (同上)
```

**总滤波器数量**：

```
总数 = 谐波数 × 3轴 × ESC数
     = IMU_GYRO_DNF_HMC × 3 × 4 (四旋翼)
     = 3 × 3 × 4
     = 36 个独立的陷波滤波器
```

---

#### 3.3.2 关于"权重"的说明

**重要**：动态陷波滤波器**没有权重参数**！

每个陷波滤波器是**独立的二阶陷波滤波器**，参数只有：
- **中心频率**（自动计算）
- **带宽**（统一配置）
- **采样率**（系统决定）

**不存在的参数**：
- ❌ 谐波权重
- ❌ 滤波器增益
- ❌ ESC 权重

**串联滤波器的效果**：

```
信号 → [陷波1] → [陷波2] → [陷波3] → ... → 输出

每个陷波滤波器完全衰减其中心频率附近的信号。
最终效果是所有陷波频率的叠加。
```

---

### 3.4 更新机制

#### 3.4.1 频率变化检测

```cpp
// VehicleAngularVelocity.cpp:606-608
const float notch_freq_delta = fabsf(nf.getNotchFreq() - frequency_hz);
const bool notch_freq_changed = (notch_freq_delta > 0.1f);
```

**更新条件**：
- 频率变化 > 0.1 Hz
- 或强制更新（参数改变、初始化）

**目的**：
- 减少不必要的滤波器重配置
- 避免频繁切换引起的抖动

---

#### 3.4.2 初始化限制

```cpp
// VehicleAngularVelocity.cpp:610-611
const bool allow_update = !axis_init[axis] ||
                         (nf.initialized() && notch_freq_delta < nf.getBandwidth());
```

**每次迭代每轴只允许初始化一个新滤波器**

**原因**：
- 初始化滤波器需要计算资源
- 分散初始化避免 CPU 峰值
- 保证系统实时性

---

#### 3.4.3 超时保护

```cpp
// VehicleAngularVelocity.cpp:634-660
// 3秒无 ESC 数据则禁用滤波器
if (time_now_us > _last_esc_rpm_notch_update[esc] + DYNAMIC_NOTCH_FITLER_TIMEOUT) {
    // 从高频到低频逐步禁用
    for (int harmonic = _esc_rpm_harmonics - 1; harmonic >= 0; harmonic--) {
        nf.disable();
    }
}
```

**超时值**：
```cpp
static constexpr hrt_abstime DYNAMIC_NOTCH_FITLER_TIMEOUT = 3_s;
```

---

## 四、FFT 动态陷波滤波器详解

### 4.1 工作原理

FFT 动态陷波滤波器通过频谱分析自动识别振动频率。

**数据流**：

```
原始陀螺仪数据 (高速采样)
    ↓
GYRO_FFT 模块 (FFT 分析)
    ↓
识别峰值频率 (sensor_gyro_fft)
    ↓
创建陷波滤波器 (每个峰值、每个轴)
    ↓
应用到陀螺仪数据
```

---

### 4.2 参数配置

#### 4.2.1 峰值频率来源

**代码**（`VehicleAngularVelocity.cpp:689-694`）：

```cpp
float *peak_frequencies[] {
    sensor_gyro_fft.peak_frequencies_x,
    sensor_gyro_fft.peak_frequencies_y,
    sensor_gyro_fft.peak_frequencies_z
};

for (int axis = 0; axis < 3; axis++) {
    for (int peak = 0; peak < MAX_NUM_FFT_PEAKS; peak++) {
        const float peak_freq = peak_frequencies[axis][peak];
        // ... 创建陷波滤波器
    }
}
```

**峰值数量**：
```cpp
#define MAX_NUM_FFT_PEAKS 5  // 每轴最多 5 个峰值
```

**总滤波器数**：
```
总数 = 峰值数 × 3轴
     = 5 × 3
     = 15 个独立陷波滤波器
```

---

#### 4.2.2 带宽自动计算

**代码**（`VehicleAngularVelocity.cpp:687`）：

```cpp
// 带宽基于 FFT 分辨率
const float bandwidth = math::constrain(sensor_gyro_fft.resolution_hz, 8.f, 30.f);
```

**FFT 分辨率**：
```
分辨率 = 采样率 / FFT 窗口大小

示例：
  采样率 = 2000 Hz
  窗口 = 256 样本
  分辨率 = 2000 / 256 = 7.8 Hz → 限制到 8 Hz
```

**带宽范围**：8-30 Hz（自动限制）

---

#### 4.2.3 最低频率

**代码**（`VehicleAngularVelocity.cpp:685`）：

```cpp
static constexpr float peak_freq_min = 10.f; // 硬编码 10 Hz
```

**低于 10 Hz 的峰值会被忽略**。

---

#### 4.2.4 滤波器更新

**代码**（`VehicleAngularVelocity.cpp:698-713`）：

```cpp
if (peak_freq > peak_freq_min) {
    // 更新条件：频率变化 > 0.1 Hz
    if (force || !nf.initialized() ||
        (fabsf(nf.getNotchFreq() - peak_freq) > 0.1f)) {

        nf.setParameters(_filter_sample_rate_hz, peak_freq, bandwidth);
    }
} else {
    // 峰值消失，禁用滤波器
    nf.disable();
}
```

---

### 4.3 FFT vs ESC RPM 对比

| 特性 | ESC RPM | FFT |
|------|---------|-----|
| **数据来源** | ESC 遥测 | FFT 频谱分析 |
| **滤波器数** | ESC数 × 谐波数 × 3 | 峰值数 × 3 |
| **中心频率** | 计算自 RPM | 自动检测峰值 |
| **带宽** | 参数配置 | 基于 FFT 分辨率 |
| **最低频率** | 参数可配 | 硬编码 10 Hz |
| **CPU 开销** | 低 | 中等（需要 FFT） |
| **精度** | 高（直接测量） | 取决于 FFT 分辨率 |
| **依赖** | ESC 遥测功能 | GYRO_FFT 模块 |

---

## 五、静态陷波滤波器

### 5.1 参数配置

| 参数 | 类型 | 默认值 | 范围 | 说明 |
|------|------|--------|------|------|
| `IMU_GYRO_NF0_FRQ` | FLOAT | 0.0 | 0-1000 Hz | 陷波 0 中心频率（0=禁用） |
| `IMU_GYRO_NF0_BW` | FLOAT | 20.0 | 0-100 Hz | 陷波 0 带宽 |
| `IMU_GYRO_NF1_FRQ` | FLOAT | 0.0 | 0-1000 Hz | 陷波 1 中心频率（0=禁用） |
| `IMU_GYRO_NF1_BW` | FLOAT | 20.0 | 0-100 Hz | 陷波 1 带宽 |

### 5.2 使用场景

**适用于**：
- 已知固定频率的振动（如机架共振）
- 不随转速变化的振动源
- 简单配置，无需 ESC 遥测或 FFT

**示例**：

```bash
# 机架共振在 80 Hz
param set IMU_GYRO_NF0_FRQ 80
param set IMU_GYRO_NF0_BW 20

# 桨叶通过频率约 120 Hz (静止时)
param set IMU_GYRO_NF1_FRQ 120
param set IMU_GYRO_NF1_BW 20
```

---

## 六、实际配置示例

### 6.1 场景 1：四旋翼 + ESC 遥测（推荐）

```bash
# 启用 ESC RPM 动态陷波
param set IMU_GYRO_DNF_EN 1

# 配置参数
param set IMU_GYRO_DNF_HMC 3      # 3 个谐波
param set IMU_GYRO_DNF_BW 15      # 15 Hz 带宽
param set IMU_GYRO_DNF_MIN 25     # 最低 25 Hz

# 保存并重启
param save
reboot
```

**效果**：
- 为 4 个电机各创建 3 个谐波陷波（共 36 个滤波器）
- 自动跟踪电机转速变化
- 消除电机相关振动

---

### 6.2 场景 2：FFT 自动检测

```bash
# 启用 FFT 动态陷波
param set IMU_GYRO_DNF_EN 2

# 确保 GYRO_FFT 模块已启用（板子配置中已有）
# CONFIG_MODULES_GYRO_FFT=y

# 保存并重启
param save
reboot
```

**效果**：
- 自动检测最多 5 个峰值频率（每轴）
- 无需 ESC 遥测
- 适用于复杂振动环境

---

### 6.3 场景 3：ESC RPM + FFT 双重保护（高性能）

```bash
# 同时启用两种陷波
param set IMU_GYRO_DNF_EN 3

# ESC RPM 参数
param set IMU_GYRO_DNF_HMC 3
param set IMU_GYRO_DNF_BW 15
param set IMU_GYRO_DNF_MIN 25

# 保存并重启
param save
reboot
```

**效果**：
- ESC RPM 处理电机振动
- FFT 处理其他振动源
- 最大限度消除干扰

**代价**：
- 最高 CPU 负载
- 最多滤波器数量（36 + 15 = 51 个）

---

### 6.4 场景 4：固定频率振动

```bash
# 禁用动态陷波
param set IMU_GYRO_DNF_EN 0

# 使用静态陷波
param set IMU_GYRO_NF0_FRQ 80    # 机架共振
param set IMU_GYRO_NF0_BW 20

param set IMU_GYRO_NF1_FRQ 150   # 其他固定振动
param set IMU_GYRO_NF1_BW 20

# 保存
param save
```

**效果**：
- 最低 CPU 开销
- 只处理已知频率

---

## 七、调试和验证

### 7.1 检查当前配置

```bash
# 查看所有动态陷波参数
param show IMU_GYRO_DNF*

# 输出示例：
# IMU_GYRO_DNF_EN  : 1
# IMU_GYRO_DNF_BW  : 15.000000
# IMU_GYRO_DNF_HMC : 3
# IMU_GYRO_DNF_MIN : 25.000000
```

---

### 7.2 查看滤波器状态

```bash
# 查看 VehicleAngularVelocity 性能统计
vehicle_angular_velocity status

# 输出包含：
# gyro dynamic notch filter ESC RPM disable: X events
# gyro dynamic notch filter ESC RPM init: X events
# gyro dynamic notch filter ESC RPM update: X events
# gyro dynamic notch filter FFT disable: X events
# gyro dynamic notch filter FFT update: X events
```

**性能计数器含义**：

| 计数器 | 含义 |
|--------|------|
| `ESC RPM init` | ESC RPM 滤波器初始化次数 |
| `ESC RPM update` | 频率更新次数（转速变化） |
| `ESC RPM disable` | 滤波器禁用次数（超时） |
| `FFT update` | FFT 滤波器更新次数 |
| `FFT disable` | FFT 滤波器禁用次数 |

---

### 7.3 验证 ESC 遥测

```bash
# 查看 ESC 状态
esc_status

# 输出应包含：
# ESC 0: RPM=2000, ...
# ESC 1: RPM=2010, ...
# ESC 2: RPM=1990, ...
# ESC 3: RPM=2005, ...
```

如果 RPM 全是 0，说明没有遥测数据，ESC RPM 陷波不会工作。

---

### 7.4 验证 FFT 模块

```bash
# 查看 FFT 模块状态
gyro_fft status

# 输出应包含峰值频率信息
```

---

### 7.5 实时监控

```bash
# 监控陀螺仪 FFT 数据
listener sensor_gyro_fft

# 输出包含：
# peak_frequencies_x: [80.5, 120.3, 0, 0, 0]
# peak_frequencies_y: [81.2, 119.8, 0, 0, 0]
# peak_frequencies_z: [80.8, 120.5, 0, 0, 0]
```

---

## 八、常见问题

### Q1: 动态陷波对性能影响有多大？

**A**: 取决于滤波器数量

| 配置 | 滤波器数 | CPU 占用 | 适用 |
|------|---------|---------|------|
| 仅低通 | 3 | 基准 | 低噪声环境 |
| + 静态陷波×2 | 9 | +5% | 固定振动 |
| + ESC RPM (3谐波×4电机) | 45 | +15% | 推荐 |
| + FFT (5峰×3轴) | 60 | +25% | 高性能 |

---

### Q2: 为什么我的 ESC RPM 陷波不工作？

**排查步骤**：

1. 检查参数：`param show IMU_GYRO_DNF_EN`（应为 1 或 3）
2. 检查 ESC 遥测：`esc_status`（RPM 应非 0）
3. 检查性能计数：`vehicle_angular_velocity status`
4. 检查 ESC 协议：需要 DShot 或 CAN ESC

---

### Q3: FFT 陷波的带宽为什么不能手动设置？

**A**: FFT 带宽基于 FFT 分辨率自动计算，因为：
- FFT 只能精确到分辨率
- 带宽小于分辨率会降低效果
- 带宽过大会去除有用信号

---

### Q4: 可以只用部分 ESC 的 RPM 数据吗？

**A**: 可以！代码会自动处理：
```cpp
// 只为有数据的 ESC 创建滤波器
if (esc_connected && (time_now_us < esc_report.timestamp + TIMEOUT)) {
    // 创建滤波器
}
```

断开或无数据的 ESC 不会创建滤波器。

---

### Q5: 谐波数设置为多少合适？

**推荐**：

| 应用 | 谐波数 | 原因 |
|------|-------|------|
| 日常飞行 | 2-3 | 平衡性能和效果 |
| 竞速/穿越 | 3-4 | 高转速，高谐波明显 |
| 载重/慢速 | 1-2 | 低转速，高谐波弱 |

---

## 九、总结

### 9.1 关键要点

1. **陀螺仪有 5 组滤波器**：
   - 2 个动态陷波（ESC RPM + FFT）
   - 2 个静态陷波
   - 1 个低通滤波器

2. **动态陷波可同时使用**：
   - `IMU_GYRO_DNF_EN` 位掩码控制
   - ESC RPM 和 FFT 是互补的

3. **没有"权重"概念**：
   - 每个陷波滤波器独立工作
   - 参数只有频率和带宽
   - 串联应用，叠加效果

4. **频率自动计算**：
   - ESC RPM: `RPM/60 × 谐波次数`
   - FFT: 自动检测峰值
   - 都有最低频率保护

5. **带宽统一配置**：
   - ESC RPM: `IMU_GYRO_DNF_BW`
   - FFT: 基于 FFT 分辨率
   - 静态陷波: 独立配置

### 9.2 配置建议

**推荐配置（ESC 支持遥测）**：
```bash
param set IMU_GYRO_DNF_EN 1
param set IMU_GYRO_DNF_HMC 3
param set IMU_GYRO_DNF_BW 15
param set IMU_GYRO_DNF_MIN 25
```

**高性能配置（ESC 遥测 + FFT）**：
```bash
param set IMU_GYRO_DNF_EN 3
param set IMU_GYRO_DNF_HMC 3
param set IMU_GYRO_DNF_BW 15
param set IMU_GYRO_DNF_MIN 25
```

**简单配置（无 ESC 遥测）**：
```bash
param set IMU_GYRO_DNF_EN 2  # 仅 FFT
```

---

## 附录：相关代码位置

| 功能 | 文件 | 行数 |
|------|------|------|
| 滤波器链 | `VehicleAngularVelocity.cpp` | 725-768 |
| ESC RPM 更新 | `VehicleAngularVelocity.cpp` | 569-664 |
| FFT 更新 | `VehicleAngularVelocity.cpp` | 666-723 |
| 参数定义 | `imu_gyro_parameters.c` | 174-222 |
| 数据结构 | `VehicleAngularVelocity.hpp` | 138-205 |

