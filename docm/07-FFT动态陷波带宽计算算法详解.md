# FFT 动态陷波滤波器带宽自动计算详解

## 概述

本文档详细解释 PX4 中 FFT 动态陷波滤波器如何自动计算带宽的机制。

**核心代码**：
```cpp
// src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp:687
const float bandwidth = math::constrain(sensor_gyro_fft.resolution_hz, 8.f, 30.f);
```

**相关文件**：
- `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` (666-723行)
- `src/modules/gyro_fft/GyroFFT.cpp` (FFT 模块)

---

## 一、核心代码解析

### 1.1 代码含义

```cpp
const float bandwidth = math::constrain(sensor_gyro_fft.resolution_hz, 8.f, 30.f);
```

**分解说明**：

| 组成部分 | 类型 | 含义 |
|---------|------|------|
| `sensor_gyro_fft.resolution_hz` | FLOAT | FFT 的频率分辨率（Hz） |
| `math::constrain(value, min, max)` | 函数 | 约束函数，限制值在范围内 |
| `8.f` | FLOAT | 最小带宽（8 Hz） |
| `30.f` | FLOAT | 最大带宽（30 Hz） |
| `bandwidth` | FLOAT | 计算出的陷波滤波器带宽 |

**逻辑**：

```cpp
if (resolution_hz < 8.0) {
    bandwidth = 8.0;           // 使用最小值
} else if (resolution_hz > 30.0) {
    bandwidth = 30.0;          // 使用最大值
} else {
    bandwidth = resolution_hz; // 使用 FFT 分辨率
}
```

---

### 1.2 完整代码上下文

```cpp
void VehicleAngularVelocity::UpdateDynamicNotchFFT(const hrt_abstime &time_now_us, bool force)
{
#if !defined(CONSTRAINED_FLASH)
    const bool enabled = _param_imu_gyro_dnf_en.get() & DynamicNotch::FFT;

    if (enabled && (_sensor_gyro_fft_sub.updated() || force)) {

        sensor_gyro_fft_s sensor_gyro_fft;

        if (_sensor_gyro_fft_sub.copy(&sensor_gyro_fft)
            && (sensor_gyro_fft.device_id == _selected_sensor_device_id)
            && (time_now_us < sensor_gyro_fft.timestamp + DYNAMIC_NOTCH_FITLER_TIMEOUT)
            && ((fabsf(sensor_gyro_fft.sensor_sample_rate_hz - _filter_sample_rate_hz) / _filter_sample_rate_hz) < 0.02f)) {

            // 最低峰值频率（硬编码）
            static constexpr float peak_freq_min = 10.f;

            // ★★★ 自动计算带宽（核心代码）★★★
            const float bandwidth = math::constrain(sensor_gyro_fft.resolution_hz, 8.f, 30.f);

            // 获取三轴的峰值频率数组
            float *peak_frequencies[] {
                sensor_gyro_fft.peak_frequencies_x,
                sensor_gyro_fft.peak_frequencies_y,
                sensor_gyro_fft.peak_frequencies_z
            };

            // 为每个轴的每个峰值创建陷波滤波器
            for (int axis = 0; axis < 3; axis++) {
                for (int peak = 0; peak < MAX_NUM_FFT_PEAKS; peak++) {

                    const float peak_freq = peak_frequencies[axis][peak];
                    auto &nf = _dynamic_notch_filter_fft[axis][peak];

                    if (peak_freq > peak_freq_min) {
                        // 使用自动计算的 bandwidth
                        if (force || !nf.initialized() || (fabsf(nf.getNotchFreq() - peak_freq) > 0.1f)) {
                            nf.setParameters(
                                _filter_sample_rate_hz,  // 采样率
                                peak_freq,               // 中心频率（FFT 检测）
                                bandwidth                // 带宽（自动计算）★
                            );
                        }
                    } else {
                        nf.disable();
                    }
                }
            }
        }
    }
#endif // !CONSTRAINED_FLASH
}
```

---

## 二、FFT 频率分辨率 (resolution_hz)

### 2.1 定义

**FFT 频率分辨率**：FFT 能够区分的最小频率间隔。

### 2.2 计算公式

```
FFT 分辨率 (Hz) = 采样率 (Hz) / FFT 窗口大小 (样本数)

resolution_hz = sample_rate / fft_window_size
```

### 2.3 实际计算示例

#### 示例 1：标准配置

```
采样率：2000 Hz
FFT 窗口：256 样本

分辨率 = 2000 / 256 = 7.8125 Hz
```

**含义**：FFT 只能区分相隔至少 7.8 Hz 的两个频率。

#### 示例 2：高分辨率配置

```
采样率：2000 Hz
FFT 窗口：512 样本

分辨率 = 2000 / 512 = 3.90625 Hz
```

**含义**：更细的频率分辨率，能检测更接近的频率。

#### 示例 3：低分辨率配置

```
采样率：2000 Hz
FFT 窗口：64 样本

分辨率 = 2000 / 64 = 31.25 Hz
```

**含义**：粗糙的频率分辨率，更新快但精度低。

---

### 2.4 FFT 参数对照表

| 采样率 | FFT 窗口 | 分辨率 | 更新率 | 延迟 | 特点 |
|--------|---------|--------|--------|------|------|
| 2000 Hz | 64 | 31.25 Hz | 高 | 低 | 快速响应，低精度 |
| 2000 Hz | 128 | 15.625 Hz | 中高 | 低 | 平衡 |
| 2000 Hz | 256 | 7.8125 Hz | 中 | 中 | **推荐默认** |
| 2000 Hz | 512 | 3.906 Hz | 低 | 高 | 高精度，慢响应 |
| 2000 Hz | 1024 | 1.953 Hz | 很低 | 很高 | 最高精度，很慢 |

---

## 三、为什么用 FFT 分辨率作为带宽？

### 3.1 核心原理：匹配检测精度

**问题**：FFT 检测到的峰值频率不是精确点，而是一个范围。

#### FFT 检测的不确定性

```
假设：
  真实振动频率 = 80 Hz
  FFT 分辨率 = 8 Hz

FFT 检测结果：
  报告峰值频率 = 80 Hz（量化后的值）

实际频率可能在：
  [80 - 分辨率/2, 80 + 分辨率/2]
  = [76 Hz, 84 Hz]
```

**FFT 频谱图示**：

```
FFT 幅度
  ↑
  │         ┌───┐
  │        ╱     ╲      ← 峰值实际是一个"山包"
  │       ╱       ╲        不是一个点！
  │      ╱         ╲
  │_____╱___________╲_______________→ 频率
  │    76   80    84
  │    └─────┬─────┘
  │       8 Hz
  │    (分辨率范围)
```

---

### 3.2 带宽过窄的问题

#### 场景 1：带宽 < 分辨率

```
FFT 检测：峰值 = 80 Hz
FFT 分辨率：8 Hz
实际振动可能在：[76, 84] Hz

设置带宽 = 5 Hz：
陷波范围 = [80 - 2.5, 80 + 2.5] = [77.5, 82.5] Hz

问题：
  真实振动 = 76 Hz → 未被陷波覆盖！❌
  真实振动 = 84 Hz → 未被陷波覆盖！❌
```

**图示**：

```
频率轴：
  70      75      80      85      90
  │       │       │       │       │
  ├───────┼───────┼───────┼───────┤
          └──────FFT不确定性──────┘ (76-84 Hz)
                  └──陷波──┘         (77.5-82.5 Hz)
                                     太窄！漏掉边缘
```

---

#### 场景 2：带宽 = 分辨率

```
FFT 检测：峰值 = 80 Hz
FFT 分辨率：8 Hz
实际振动可能在：[76, 84] Hz

设置带宽 = 8 Hz：
陷波范围 = [80 - 4, 80 + 4] = [76, 84] Hz

结果：
  真实振动在 [76, 84] Hz 的任何位置都被覆盖！✅
```

**图示**：

```
频率轴：
  70      75      80      85      90
  │       │       │       │       │
  ├───────┼───────┼───────┼───────┤
          └──────FFT不确定性──────┘ (76-84 Hz)
          └──────陷波范围────────┘ (76-84 Hz)
                                   完美匹配！
```

---

### 3.3 带宽过宽的问题

#### 场景 3：带宽 >> 分辨率

```
FFT 检测：峰值 = 80 Hz
FFT 分辨率：8 Hz
实际振动可能在：[76, 84] Hz

设置带宽 = 50 Hz：
陷波范围 = [80 - 25, 80 + 25] = [55, 105] Hz

问题：
  去除了过多的频率范围
  可能影响控制信号（如 60 Hz、100 Hz 的控制响应）❌
```

**图示**：

```
频率轴：
  50      60      80      100     110
  │       │       │       │       │
  ├───────┼───────┼───────┼───────┤
          └──FFT不确定性──┘ (76-84 Hz)
  └──────────陷波范围──────────┘ (55-105 Hz)
                          太宽！去除有用信号
```

---

## 四、带宽限制范围：8-30 Hz

### 4.1 下限：8 Hz

```cpp
math::constrain(..., 8.f, ...)  // 最小 8 Hz
```

#### 4.1.1 为什么不能更小？

**原因 1：避免过窄陷波**

即使 FFT 分辨率很小（如 3 Hz），也不能用那么窄的带宽：

```
FFT 分辨率 = 3 Hz
如果带宽也 = 3 Hz：

问题：
1. 振动频率本身会随飞行状态变化（±2 Hz）
2. FFT 量化误差
3. 传感器漂移

结果：可能漏掉实际振动
```

**原因 2：数值稳定性**

陷波滤波器的系数计算：

```cpp
// 二阶陷波滤波器设计
Q = center_freq / bandwidth

如果 bandwidth 太小：
  Q 值过大 → 滤波器变得"尖锐"
  → 数值精度问题
  → 可能不稳定
```

**原因 3：振动频率的自然变化**

```
电机转速：2000 RPM ±50 RPM
基频：33.3 Hz ±0.8 Hz

如果带宽只有 3 Hz：
  陷波覆盖 [31.8, 34.8] Hz
  实际振动可能在 [32.5, 34.1] Hz → 需要更宽的余量
```

---

#### 4.1.2 实际效果示例

| FFT 分辨率 | 不限制 | 限制到 8 Hz | 说明 |
|-----------|--------|------------|------|
| 3.9 Hz | 3.9 Hz | **8 Hz** | 太窄，强制使用 8 Hz |
| 5.0 Hz | 5.0 Hz | **8 Hz** | 太窄，强制使用 8 Hz |
| 7.8 Hz | 7.8 Hz | **8 Hz** | 略窄，强制使用 8 Hz |
| 10 Hz | 10 Hz | 10 Hz | 合理，保持原值 |

---

### 4.2 上限：30 Hz

```cpp
math::constrain(..., ..., 30.f)  // 最大 30 Hz
```

#### 4.2.1 为什么不能更大？

**原因 1：保护控制带宽**

飞控的控制回路频率：

```
姿态控制回路：50-200 Hz
角速度控制回路：100-400 Hz

如果陷波带宽 = 50 Hz，中心在 100 Hz：
  陷波范围 = [75, 125] Hz
  → 影响 80-120 Hz 的控制响应
  → 控制性能下降！
```

**原因 2：去除有用信号**

```
陀螺仪有效信号频带：0-150 Hz
如果陷波太宽：
  去除过多有效信号
  → 丢失快速运动信息
  → 延迟增加
```

**原因 3：FFT 分辨率不应该太粗糙**

```
如果 FFT 分辨率 > 30 Hz：
  说明 FFT 配置有问题（窗口太小）
  → 检测精度太低，不可靠
  → 应该调整 FFT 参数，而不是增加带宽
```

---

#### 4.2.2 实际效果示例

| FFT 分辨率 | 不限制 | 限制到 30 Hz | 说明 |
|-----------|--------|-------------|------|
| 15 Hz | 15 Hz | 15 Hz | 合理，保持原值 |
| 25 Hz | 25 Hz | 25 Hz | 合理，保持原值 |
| 31.25 Hz | 31.25 Hz | **30 Hz** | 太宽，限制到 30 Hz |
| 62.5 Hz | 62.5 Hz | **30 Hz** | 太宽，限制到 30 Hz |

---

## 五、不同配置的带宽计算

### 5.1 计算表格

| 采样率 | FFT 窗口 | 原始分辨率 | constrain 后 | 实际带宽 | 评价 |
|--------|---------|-----------|-------------|---------|------|
| 2000 Hz | 1024 | 1.95 Hz | 8 Hz | **8 Hz** | 触及下限，精度高但带宽保护 |
| 2000 Hz | 512 | 3.91 Hz | 8 Hz | **8 Hz** | 触及下限，高精度配置 |
| 2000 Hz | 256 | 7.81 Hz | 8 Hz | **8 Hz** | 略低于下限 |
| 2000 Hz | 128 | 15.63 Hz | 15.63 Hz | **15.63 Hz** | ✅ 理想范围 |
| 2000 Hz | 64 | 31.25 Hz | 30 Hz | **30 Hz** | 触及上限，分辨率太低 |
| 2000 Hz | 32 | 62.5 Hz | 30 Hz | **30 Hz** | 严重超限，FFT 配置不合理 |
| 8000 Hz | 512 | 15.63 Hz | 15.63 Hz | **15.63 Hz** | ✅ 理想，高采样率 |
| 8000 Hz | 1024 | 7.81 Hz | 8 Hz | **8 Hz** | 触及下限 |

---

### 5.2 推荐配置

#### 配置 1：平衡型（推荐）

```
采样率：2000 Hz
FFT 窗口：128 样本
分辨率：15.6 Hz
带宽：15.6 Hz

优点：
  ✓ 分辨率适中
  ✓ 更新速度快
  ✓ 带宽合理

缺点：
  - 频率精度一般
```

---

#### 配置 2：高精度型

```
采样率：2000 Hz
FFT 窗口：256 样本
分辨率：7.8 Hz
带宽：8 Hz (限制)

优点：
  ✓ 频率精度高
  ✓ 带宽窄，去除信号少

缺点：
  - 更新速度较慢
  - 延迟较大
```

---

#### 配置 3：快速响应型

```
采样率：2000 Hz
FFT 窗口：64 样本
分辨率：31.25 Hz
带宽：30 Hz (限制)

优点：
  ✓ 更新速度最快
  ✓ 延迟最小

缺点：
  - 频率精度低
  - 带宽宽，可能影响控制
```

---

## 六、与 ESC RPM 带宽的对比

### 6.1 两种带宽的区别

| 特性 | ESC RPM 带宽 | FFT 带宽 |
|------|-------------|---------|
| **设置方式** | 参数配置 | **自动计算** |
| **参数** | `IMU_GYRO_DNF_BW` | 无（基于分辨率） |
| **典型值** | 15 Hz（固定） | 8-30 Hz（动态） |
| **依据** | 经验值 | **检测精度** |
| **可调性** | ✅ 用户可调 | ❌ 自动适配 |
| **变化性** | 固定不变 | 随 FFT 配置变化 |

---

### 6.2 为什么设计不同？

#### ESC RPM：已知频率源

```
ESC 转速测量精度：±1%
频率变化速度：慢（秒级）
振动频率清晰：基频 + 谐波

→ 可以使用固定带宽（如 15 Hz）
→ 经验值，适用于大多数情况
```

#### FFT：未知频率探测

```
FFT 分辨率：取决于配置
检测精度：受限于分辨率
峰值定位：量化误差 = 分辨率

→ 必须匹配检测精度
→ 自动适配，确保覆盖
```

---

### 6.3 同时使用时的协调

```cpp
// 应用顺序（VehicleAngularVelocity.cpp:725-768）

// [1] ESC RPM 陷波（带宽 = 15 Hz，参数配置）
for (int esc = 0; esc < MAX_NUM_ESCS; esc++) {
    for (int harmonic = 0; harmonic < _esc_rpm_harmonics; harmonic++) {
        _dynamic_notch_filter_esc_rpm[harmonic][axis][esc].applyArray(data, N);
    }
}

// [2] FFT 陷波（带宽 = 自动计算，8-30 Hz）
for (int peak = 0; peak < MAX_NUM_FFT_PEAKS; peak++) {
    _dynamic_notch_filter_fft[axis][peak].applyArray(data, N);
}
```

**两者互补**：
- ESC RPM 处理已知的电机振动（精确但有限）
- FFT 处理未知振动源（广泛但受限于分辨率）

---

## 七、实际验证

### 7.1 查看当前 FFT 分辨率和带宽

```bash
# 连接飞控后监听 FFT 数据
listener sensor_gyro_fft

# 输出示例：
# timestamp: 123456789
# device_id: 6684690
# sensor_sample_rate_hz: 2000.0
# resolution_hz: 7.8125          ← FFT 分辨率
# peak_frequencies_x: [80.5, 120.3, 0, 0, 0]
# peak_frequencies_y: [81.2, 119.8, 0, 0, 0]
# peak_frequencies_z: [80.8, 120.5, 0, 0, 0]
```

**计算带宽**：

```
resolution_hz = 7.8125
bandwidth = constrain(7.8125, 8, 30)
         = 8 Hz  ← 实际使用的带宽
```

---

### 7.2 验证陷波滤波器设置

虽然无法直接查看滤波器参数，但可以通过性能计数器验证：

```bash
vehicle_angular_velocity status

# 输出包含：
# gyro dynamic notch filter FFT update: X events
```

如果 `FFT update` 计数增加，说明滤波器在工作。

---

### 7.3 调整 FFT 参数（如果需要）

FFT 参数在 GYRO_FFT 模块中配置：

```bash
# 查看 FFT 参数
param show IMU_GYRO_FFT*

# 常见参数：
# IMU_GYRO_FFT_EN  - 启用 FFT
# IMU_GYRO_FFT_LEN - FFT 窗口大小（影响分辨率）
# IMU_GYRO_FFT_SNR - 信噪比阈值
```

**修改 FFT 窗口大小**：

```bash
# 增加分辨率（更大窗口）
param set IMU_GYRO_FFT_LEN 512  # 原来可能是 256

# 或减小分辨率（更小窗口，更快更新）
param set IMU_GYRO_FFT_LEN 128

# 保存并重启
param save
reboot
```

---

## 八、常见问题

### Q1: 为什么不让用户手动设置 FFT 陷波带宽？

**A**: 因为带宽必须匹配 FFT 分辨率：

- 如果用户设置 `带宽 = 10 Hz`，但 `分辨率 = 30 Hz`
  - → 陷波太窄，漏掉振动
- 如果用户设置 `带宽 = 50 Hz`，但 `分辨率 = 8 Hz`
  - → 陷波太宽，去除有用信号

**自动计算避免配置错误。**

---

### Q2: 可以修改 8-30 Hz 的范围吗？

**A**: 可以修改代码：

```cpp
// VehicleAngularVelocity.cpp:687
// 原来：
const float bandwidth = math::constrain(sensor_gyro_fft.resolution_hz, 8.f, 30.f);

// 修改为（例如 5-40 Hz）：
const float bandwidth = math::constrain(sensor_gyro_fft.resolution_hz, 5.f, 40.f);
```

但**不推荐**，因为：
- 8-30 Hz 是经过验证的安全范围
- 超出范围可能影响稳定性

---

### Q3: 如果 FFT 分辨率一直触及限制怎么办？

#### 情况 1：分辨率 < 8 Hz（总是 8 Hz）

**说明**：FFT 窗口太大

**解决**：

```bash
# 减小 FFT 窗口（如果支持配置）
param set IMU_GYRO_FFT_LEN 128  # 降低分辨率，但提高更新率
```

---

#### 情况 2：分辨率 > 30 Hz（总是 30 Hz）

**说明**：FFT 窗口太小，分辨率太低

**解决**：

```bash
# 增大 FFT 窗口
param set IMU_GYRO_FFT_LEN 256  # 提高分辨率

# 或增加采样率（如果硬件支持）
param set IMU_GYRO_RATEMAX 1600
```

---

### Q4: ESC RPM 和 FFT 的带宽冲突吗？

**A**: 不冲突，两者独立工作：

```
ESC RPM 陷波（15 Hz 带宽）：
  谐波 1: 33 Hz ± 7.5 Hz = [25.5, 40.5] Hz
  谐波 2: 66 Hz ± 7.5 Hz = [58.5, 73.5] Hz
  谐波 3: 99 Hz ± 7.5 Hz = [91.5, 106.5] Hz

FFT 陷波（10 Hz 带宽）：
  峰值 1: 80 Hz ± 5 Hz = [75, 85] Hz  ← 与 ESC 不重叠
  峰值 2: 120 Hz ± 5 Hz = [115, 125] Hz
  峰值 3: 45 Hz ± 5 Hz = [40, 50] Hz

两者覆盖不同频率范围，互补！
```

---

## 九、设计思想总结

### 9.1 核心原则

**"陷波带宽应匹配检测精度"**

```
FFT 只能精确到 ±分辨率/2
→ 陷波带宽至少应等于分辨率
→ 才能确保覆盖实际振动频率
```

---

### 9.2 自适应设计优势

| 优势 | 说明 |
|------|------|
| **自动适配** | 无需手动配置，减少错误 |
| **精度匹配** | 带宽始终匹配 FFT 精度 |
| **安全保护** | 8-30 Hz 限制确保稳定性 |
| **灵活性** | 支持不同 FFT 配置 |

---

### 9.3 与 ESC RPM 的互补性

```
┌────────────────────────────────────┐
│ ESC RPM 动态陷波                    │
│ - 精确跟踪电机振动                  │
│ - 固定带宽（15 Hz）                 │
│ - 需要 ESC 遥测                     │
└────────────────────────────────────┘
              ↓
         串联应用
              ↓
┌────────────────────────────────────┐
│ FFT 动态陷波                        │
│ - 自动检测未知振动                  │
│ - 自适应带宽（8-30 Hz）             │
│ - 无需外部数据                      │
└────────────────────────────────────┘
              ↓
        完整的振动抑制
```

---

## 十、总结

### 关键要点

1. **FFT 陷波带宽自动计算**
   ```cpp
   bandwidth = constrain(fft_resolution, 8, 30)
   ```

2. **基于 FFT 分辨率**
   - 分辨率 = 采样率 / FFT 窗口
   - 带宽匹配检测精度

3. **8-30 Hz 保护范围**
   - 下限 8 Hz：避免过窄
   - 上限 30 Hz：避免过宽

4. **自动适配优势**
   - 无需手动配置
   - 确保覆盖实际振动
   - 保护控制性能

5. **与 ESC RPM 互补**
   - ESC RPM：已知振动（固定带宽）
   - FFT：未知振动（自适应带宽）

---

## 附录：代码位置索引

| 功能 | 文件 | 行数 |
|------|------|------|
| 带宽计算 | `VehicleAngularVelocity.cpp` | 687 |
| FFT 陷波更新 | `VehicleAngularVelocity.cpp` | 666-723 |
| 滤波器应用 | `VehicleAngularVelocity.cpp` | 743-749 |
| FFT 模块 | `src/modules/gyro_fft/GyroFFT.cpp` | - |
| 陷波滤波器类 | `src/lib/mathlib/math/filter/NotchFilter.hpp` | - |

