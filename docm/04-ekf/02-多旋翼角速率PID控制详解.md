# 多旋翼角速率 PID 控制详解

## 一、Rate Control 概述

### 1.1 控制架构

```
遥控输入/上层控制器
         ↓
  姿态角设定值 (roll, pitch, yaw)
         ↓
    [姿态控制器]
         ↓
  角速率设定值 (roll_rate, pitch_rate, yaw_rate)
         ↓
    【角速率PID控制器】 ← 本文重点
         ↓
     力矩设定值
         ↓
    [控制分配器]
         ↓
   电机PWM输出
```

### 1.2 代码位置

**主模块**:
- `src/modules/mc_rate_control/MulticopterRateControl.cpp`
- `src/modules/mc_rate_control/MulticopterRateControl.hpp`

**核心算法**:
- `src/lib/rate_control/rate_control.cpp`
- `src/lib/rate_control/rate_control.hpp`

---

## 二、PID 控制核心算法

### 2.1 控制方程

**代码位置**: `rate_control.cpp:71-85`

```cpp
Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp,
                             const Vector3f &angular_accel, const float dt,
                             const bool landed)
{
    // 计算角速率误差
    Vector3f rate_error = rate_sp - rate;

    // PID控制 + 前馈
    const Vector3f torque = _gain_p.emult(rate_error)      // P项: 比例
                          + _rate_int                       // I项: 积分
                          - _gain_d.emult(angular_accel)    // D项: 微分
                          + _gain_ff.emult(rate_sp);        // FF: 前馈

    // 更新积分项 (着陆时不更新)
    if (!landed) {
        updateIntegral(rate_error, dt);
    }

    return torque;
}
```

### 2.2 数学公式

**标准PID形式**:

$$\tau = K_p \cdot e + K_i \cdot \int e \, dt + K_d \cdot \dot{e} + K_{ff} \cdot \omega_{sp}$$

其中:
- $\tau$: 输出力矩 (Torque)
- $e = \omega_{sp} - \omega$: 角速率误差
- $\omega_{sp}$: 角速率设定值
- $\omega$: 当前角速率 (来自陀螺仪)
- $\dot{e} = -\ddot{\theta}$: 角加速度 (已测量，无需差分)

**注意**: PX4使用**直接测量的角加速度**，而不是对误差求导，避免噪声放大！

### 2.3 增益参数配置

**代码位置**: `MulticopterRateControl.cpp:78-92`

```cpp
void MulticopterRateControl::parameters_updated()
{
    // 增益转换: Parallel形式 → Ideal形式
    const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(),
                                     _param_mc_pitchrate_k.get(),
                                     _param_mc_yawrate_k.get());

    _rate_control.setPidGains(
        rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
        rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
        rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get()))
    );

    _rate_control.setIntegratorLimit(
        Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get())
    );

    _rate_control.setFeedForwardGain(
        Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get())
    );
}
```

**两种PID形式**:

```
Parallel 形式 (并联):
  output = K_p * P * error + K_p * I * ∫error + K_p * D * ė
         = K * (P * error + I * ∫error + D * ė)

Ideal 形式 (理想):
  output = K * [error + (1/Ti) * ∫error + Td * ė]
         = K * P * error + K * I * ∫error + K * D * ė

设置方法:
  • MC_ROLLRATE_K = 1, MC_ROLLRATE_P = 实际P → Parallel形式
  • MC_ROLLRATE_K = K, MC_ROLLRATE_P = 1     → Ideal形式
```

---

## 三、默认参数分析

### 3.1 横滚(Roll)轴参数

```c
// mc_rate_control_params.c

MC_ROLLRATE_P  = 0.15   // 比例增益
MC_ROLLRATE_I  = 0.2    // 积分增益
MC_ROLLRATE_D  = 0.003  // 微分增益
MC_RR_INT_LIM  = 0.30   // 积分限幅
MC_ROLLRATE_FF = 0.0    // 前馈增益 (多旋翼通常为0)
MC_ROLLRATE_K  = 1.0    // 全局增益
```

**实际PID增益**:
```
P = 1.0 × 0.15  = 0.15
I = 1.0 × 0.2   = 0.2
D = 1.0 × 0.003 = 0.003
```

### 3.2 俯仰(Pitch)轴参数

```c
MC_PITCHRATE_P  = 0.15
MC_PITCHRATE_I  = 0.2
MC_PITCHRATE_D  = 0.003
MC_PR_INT_LIM   = 0.30
MC_PITCHRATE_FF = 0.0
MC_PITCHRATE_K  = 1.0
```

**特点**: 与Roll轴相同 (对称设计)

### 3.3 偏航(Yaw)轴参数

```c
MC_YAWRATE_P  = 0.2     // 比Roll/Pitch稍大
MC_YAWRATE_I  = 0.1     // 比Roll/Pitch小
MC_YAWRATE_D  = 0.0     // 通常为0 (偏航惯性小)
MC_YR_INT_LIM = 0.30
MC_YAWRATE_FF = 0.0
MC_YAWRATE_K  = 1.0
```

**特点**:
- P增益较大 (偏航响应需要快)
- I增益较小 (偏航扰动少)
- D增益为0 (偏航惯性小，微分不明显)

---

## 四、PID控制器详细分析

### 4.1 比例项 (P-Term)

**代码**:
```cpp
const Vector3f torque_p = _gain_p.emult(rate_error);
```

**作用**:
- 主要控制力矩来源
- 误差越大，输出越大
- 响应速度快

**数值示例**:
```
设定值: roll_rate_sp = 1.0 rad/s (57.3°/s)
当前值: roll_rate = 0.8 rad/s
误差:   e = 1.0 - 0.8 = 0.2 rad/s

P项输出: τ_p = 0.15 × 0.2 = 0.03 (归一化力矩)
```

### 4.2 积分项 (I-Term)

**代码位置**: `rate_control.cpp:88-118`

```cpp
void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
    for (int i = 0; i < 3; i++) {
        // 抗饱和: 防止积分继续累积
        if (_control_allocator_saturation_positive(i)) {
            rate_error(i) = math::min(rate_error(i), 0.f);
        }
        if (_control_allocator_saturation_negative(i)) {
            rate_error(i) = math::max(rate_error(i), 0.f);
        }

        // I增益非线性化: 大误差时降低积分作用
        float i_factor = rate_error(i) / math::radians(400.f);
        i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

        // 积分更新
        float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

        // 积分限幅
        _rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
    }
}
```

**特殊设计**:

1. **抗饱和 (Anti-Windup)**:
   - 电机饱和时停止积分累积
   - 防止积分项过大导致超调

2. **非线性I因子**:
```cpp
i_factor = 1 - (error / 400°)²

示例:
error = 50°  → i_factor = 1 - (50/400)² = 0.984  (几乎无影响)
error = 100° → i_factor = 1 - (100/400)² = 0.938 (轻微降低)
error = 200° → i_factor = 1 - (200/400)² = 0.75  (降低25%)
error = 400° → i_factor = 0                       (完全关闭)
```

**作用**: 防止大设定值变化后的积分回弹效应

3. **积分限幅**:
```
-MC_RR_INT_LIM ≤ ∫error ≤ +MC_RR_INT_LIM
```

### 4.3 微分项 (D-Term)

**代码**:
```cpp
const Vector3f torque_d = -_gain_d.emult(angular_accel);
```

**关键特性**:

1. **使用直接测量的角加速度**:
   - 不是对误差求导: `ė = d(error)/dt`
   - 而是: `ė = -d(rate)/dt = -angular_accel`
   - 避免了数值微分的噪声放大

2. **来源**: `vehicle_angular_velocity.xyz_derivative`
   ```cpp
   // VehicleAngularVelocity模块计算
   // 使用低通滤波器对陀螺数据求导
   ```

3. **作用**:
   - 阻尼快速振荡
   - 提高稳定性
   - 减少超调

**数值示例**:
```
angular_accel = 5.0 rad/s² (快速加速)
D增益: 0.003

D项输出: τ_d = -0.003 × 5.0 = -0.015 (阻尼作用)
```

### 4.4 前馈项 (Feed-Forward)

**代码**:
```cpp
const Vector3f torque_ff = _gain_ff.emult(rate_sp);
```

**用途**:
- 多旋翼: 通常为0 (对称设计，无需前馈)
- 直升机: 可能非0 (补偿旋翼陀螺效应)

---

## 五、完整控制回路

### 5.1 数据流向

```
┌──────────────────────────────────────────────────────────────┐
│  输入                                                          │
├──────────────────────────────────────────────────────────────┤
│  • vehicle_angular_velocity (来自EKF2/传感器)                 │
│    ├─ xyz: 当前角速率 [roll_rate, pitch_rate, yaw_rate]      │
│    └─ xyz_derivative: 角加速度                                │
│                                                                │
│  • vehicle_rates_setpoint (来自姿态控制器)                    │
│    ├─ roll: 横滚角速率设定                                    │
│    ├─ pitch: 俯仰角速率设定                                   │
│    └─ yaw: 偏航角速率设定                                     │
└────────────────────────────┬───────────────────────────────────┘
                             ↓
┌──────────────────────────────────────────────────────────────┐
│  PID 计算 (rate_control.cpp:71-85)                            │
├──────────────────────────────────────────────────────────────┤
│                                                                │
│  ① 计算误差:                                                  │
│     e = rate_sp - rate                                         │
│                                                                │
│  ② PID控制律:                                                 │
│     τ = K_p·e + K_i·∫e·dt + K_d·(-α) + K_ff·rate_sp          │
│                                  ↑                             │
│                            使用测量的角加速度                  │
│                                                                │
│  ③ 抗饱和积分更新:                                            │
│     if (!landed && !saturated) {                               │
│         ∫e += i_factor(e) · e · dt                            │
│     }                                                          │
│                                                                │
└────────────────────────────┬───────────────────────────────────┘
                             ↓
┌──────────────────────────────────────────────────────────────┐
│  输出                                                          │
├──────────────────────────────────────────────────────────────┤
│  • vehicle_torque_setpoint                                     │
│    └─ xyz[3]: 归一化力矩 [-1, 1]                              │
│                                                                │
│  • vehicle_thrust_setpoint                                     │
│    └─ xyz[3]: 归一化推力 [0, 1]                               │
└──────────────────────────────────────────────────────────────┘
```

### 5.2 执行频率

**代码位置**: `MulticopterRateControl.cpp:126-132`

```cpp
if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
    const hrt_abstime now = angular_velocity.timestamp_sample;
    const float dt = math::constrain((now - _last_run) * 1e-6f, 0.000125f, 0.02f);
    //                                                          ^^^^^^^^   ^^^^^
    //                                                          0.125ms    20ms
    _last_run = now;
```

**频率**:
- 订阅: `vehicle_angular_velocity` (通常250-1000Hz)
- 实际运行: 取决于VehicleAngularVelocity发布频率
- dt约束: 0.125ms ~ 20ms (防止异常)

---

## 六、高级特性

### 6.1 智能抗饱和 (Anti-Windup)

**问题**: 电机饱和时积分项继续累积会导致严重超调

**解决方案**: `rate_control.cpp:91-99`

```cpp
void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
    for (int i = 0; i < 3; i++) {
        // 防止正向饱和时的进一步正向积分
        if (_control_allocator_saturation_positive(i)) {
            rate_error(i) = math::min(rate_error(i), 0.f);
            //                                      ^^^
            //                              只允许负误差(减小积分)
        }

        // 防止负向饱和时的进一步负向积分
        if (_control_allocator_saturation_negative(i)) {
            rate_error(i) = math::max(rate_error(i), 0.f);
            //                                      ^^^
            //                              只允许正误差(减小积分)
        }
```

**原理**:
```
场景: 横滚左满舵

电机输出: 左侧=0%，右侧=100% → 饱和！
误差: e = +1.5 rad/s (还想继续左滚)

传统PID: ∫e 继续增大 → 积分项累积到很大
PX4方案: 检测到饱和 → rate_error强制≤0 → 积分不再增大

结果: 避免回弹超调
```

### 6.2 非线性I因子

**代码位置**: `rate_control.cpp:107-108`

```cpp
float i_factor = rate_error(i) / math::radians(400.f);
i_factor = math::max(0.0f, 1.f - i_factor * i_factor);
```

**曲线图**:

```
i_factor
   1.0│●●●●●●●●●●●
      │           ●●
   0.8│              ●●
      │                 ●●
   0.6│                    ●
      │                     ●
   0.4│                      ●
      │                       ●
   0.2│                        ●
      │                         ●
   0.0│                          ●●●●●●●
      └────────────────────────────────→ |error|
      0°   100°  200°  300°  400°  500°
```

**作用**:
- 小误差(<100°): i_factor≈1, 积分正常工作
- 中误差(100-300°): 积分作用逐渐减弱
- 大误差(>400°): 积分完全关闭

**防止**: 大机动后的积分回弹

### 6.3 积分限幅

**代码**:
```cpp
_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
```

**默认限制**:
```
MC_RR_INT_LIM = 0.30  (横滚)
MC_PR_INT_LIM = 0.30  (俯仰)
MC_YR_INT_LIM = 0.30  (偏航)
```

**物理意义**:
- 限制积分项贡献的最大力矩
- 0.30 表示积分项最多提供30%的满量程力矩
- 防止积分饱和和windup

---

## 七、实际运行示例

### 7.1 典型场景：横滚控制

**场景**: 遥控杆给出横滚指令

```
═══════════════════════════════════════════════════════════════
初始状态 (悬停)
═══════════════════════════════════════════════════════════════
rate_sp = [0, 0, 0] rad/s
rate = [0, 0, 0] rad/s
_rate_int = [0, 0, 0]

PID输出:
  P = 0.15 × 0 = 0
  I = 0
  D = -0.003 × 0 = 0
  Total = 0  ← 无力矩，保持悬停

═══════════════════════════════════════════════════════════════
t=0ms: 遥控杆输入 (横滚右满杆)
═══════════════════════════════════════════════════════════════
rate_sp = [2.0, 0, 0] rad/s  (约115°/s)
rate = [0, 0, 0] rad/s
error = 2.0 rad/s

PID输出:
  P = 0.15 × 2.0 = 0.30
  I = 0 (积分刚开始)
  D = -0.003 × 0 = 0
  Total = 0.30  → 右侧电机加速，左侧减速

═══════════════════════════════════════════════════════════════
t=10ms: 飞机开始横滚
═══════════════════════════════════════════════════════════════
rate_sp = [2.0, 0, 0] rad/s
rate = [0.5, 0, 0] rad/s (开始转动)
error = 1.5 rad/s
angular_accel = [50, 0, 0] rad/s² (加速中)

积分更新:
  i_factor = 1 - (1.5/6.98)² = 0.954
  ∫e += 0.954 × 0.2 × 1.5 × 0.01 = 0.00286

PID输出:
  P = 0.15 × 1.5 = 0.225
  I = 0.00286
  D = -0.003 × 50 = -0.15  ← 阻尼加速度
  Total = 0.225 + 0.003 - 0.15 = 0.078

═══════════════════════════════════════════════════════════════
t=100ms: 接近设定值
═══════════════════════════════════════════════════════════════
rate_sp = [2.0, 0, 0] rad/s
rate = [1.95, 0, 0] rad/s (快到了)
error = 0.05 rad/s
angular_accel = [2, 0, 0] rad/s² (减速)
∫e = 0.15 (累积了)

PID输出:
  P = 0.15 × 0.05 = 0.0075  (很小)
  I = 0.15                   (主要靠积分补偿)
  D = -0.003 × 2 = -0.006
  Total = 0.0075 + 0.15 - 0.006 = 0.1515

═══════════════════════════════════════════════════════════════
稳态 (t>200ms)
═══════════════════════════════════════════════════════════════
rate ≈ rate_sp = 2.0 rad/s
error ≈ 0
∫e ≈ 稳态值 (补偿气动阻力等)

PID输出: 主要靠I项维持，P和D项很小
```

### 7.2 三轴耦合考虑

**虽然是三个独立PID**，但实际会耦合:

```
横滚力矩影响:
  • 主要: Roll轴旋转
  • 次要: Pitch轴耦合 (机体不完全对称)
  • 微小: Yaw轴耦合 (陀螺效应)

控制分配器会处理这些耦合！
```

---

## 八、调优指南

### 8.1 参数调整原则

**P增益 (MC_ROLLRATE_P)**:
- 增大: 响应更快，但易振荡
- 减小: 响应慢，但更稳定
- 调节: 从小到大，直到出现轻微振荡，然后减小20%

**I增益 (MC_ROLLRATE_I)**:
- 增大: 消除稳态误差更快，但易超调
- 减小: 响应更平稳
- 调节: P调好后，慢慢增加I直到稳态误差消失

**D增益 (MC_ROLLRATE_D)**:
- 增大: 阻尼更强，减少超调
- 减小: 响应更快
- 调节: 通常保持小值 (0.003)，过大会放大噪声

### 8.2 典型调优步骤

```bash
# Step 1: 只调P增益
param set MC_ROLLRATE_I 0
param set MC_ROLLRATE_D 0
param set MC_ROLLRATE_P 0.05  # 从小开始
# 逐渐增大P，直到出现振荡
param set MC_ROLLRATE_P 0.20  # 假设这时振荡
param set MC_ROLLRATE_P 0.16  # 减小20%

# Step 2: 调I增益
param set MC_ROLLRATE_I 0.05
# 逐渐增大，直到稳态误差消失且无超调
param set MC_ROLLRATE_I 0.2

# Step 3: 微调D增益 (如果有高频振荡)
param set MC_ROLLRATE_D 0.003

# Step 4: 同样步骤调Pitch
param set MC_PITCHRATE_P 0.16
param set MC_PITCHRATE_I 0.2
param set MC_PITCHRATE_D 0.003

# Step 5: 调Yaw (通常P大I小D为0)
param set MC_YAWRATE_P 0.2
param set MC_YAWRATE_I 0.1
param set MC_YAWRATE_D 0.0

param save
```

### 8.3 常见问题

**问题1: 飞机振荡**
```
原因: P或D增益过大
解决:
  • 降低P增益10-20%
  • 或降低D增益
```

**问题2: 响应慢，跟踪差**
```
原因: P增益太小
解决: 逐步增大P，直到响应满意
```

**问题3: 稳态误差**
```
原因: I增益太小
解决: 增大I增益
```

**问题4: 翻转后回弹**
```
原因: 积分累积过大
解决:
  • 降低I增益
  • 或减小MC_RR_INT_LIM
```

---

## 九、数值仿真示例

### 9.1 阶跃响应

**输入**: 横滚角速率从0跃变到1 rad/s

**PID参数**:
```
P = 0.15
I = 0.2
D = 0.003
```

**响应曲线**:

```
rate (rad/s)
  1.2│                    ╭─────────
     │                   ╱
  1.0│                 ╱╱  ← 设定值
     │               ╱╱
  0.8│            ╱╱╱
     │         ╱╱╱
  0.6│      ╱╱╱
     │   ╱╱╱
  0.4│ ╱╱
     │╱
  0.2│●
     │
  0.0●────────────────────────────→ time
     0  50  100 150 200 250 300 (ms)

性能指标:
  • 上升时间: ~80ms
  • 超调: <5%
  • 稳态误差: <0.01 rad/s
```

---

## 十、代码调用流程

### 10.1 主循环

**MulticopterRateControl.cpp:103-272**

```cpp
void MulticopterRateControl::Run()
{
    // ① 获取角速率数据 (回调触发)
    vehicle_angular_velocity_s angular_velocity;
    if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

        const Vector3f rates{angular_velocity.xyz};
        const Vector3f angular_accel{angular_velocity.xyz_derivative};
        const float dt = ...;

        // ② 获取设定值
        vehicle_rates_setpoint_s vehicle_rates_setpoint{};
        if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
            _rates_setpoint(0) = vehicle_rates_setpoint.roll;
            _rates_setpoint(1) = vehicle_rates_setpoint.pitch;
            _rates_setpoint(2) = vehicle_rates_setpoint.yaw;
        }

        // ③ 运行PID控制器
        if (_vehicle_control_mode.flag_control_rates_enabled) {
            Vector3f torque_setpoint =
                _rate_control.update(rates, _rates_setpoint, angular_accel, dt, landed);

            // ④ 偏航轴低通滤波 (减少高频振动)
            torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

            // ⑤ 发布力矩设定值
            _vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);
        }
    }
}
```

### 10.2 订阅机制

**代码位置**: `MulticopterRateControl.cpp:67`

```cpp
_vehicle_angular_velocity_sub.registerCallback();
```

**特点**:
- 使用 `SubscriptionCallback` (事件驱动)
- `vehicle_angular_velocity` 更新时自动触发 `Run()`
- 无需轮询，CPU效率高

---

## 十一、与姿态控制器的关系

### 11.1 级联控制结构

```
┌─────────────────────────────────────────────────────────┐
│ 姿态控制器 (mc_att_control)                             │
│                                                           │
│ 输入: 姿态设定 (roll, pitch, yaw角度)                    │
│ 输出: 角速率设定 (roll_rate, pitch_rate, yaw_rate)      │
│                                                           │
│ 控制律: 比例控制                                         │
│   rate_sp = K_att × (angle_sp - angle)                   │
└──────────────────────┬──────────────────────────────────┘
                       ↓ vehicle_rates_setpoint
┌─────────────────────────────────────────────────────────┐
│ 角速率控制器 (mc_rate_control) ← 本文                   │
│                                                           │
│ 输入: 角速率设定                                         │
│ 输出: 力矩设定                                           │
│                                                           │
│ 控制律: PID控制                                          │
│   torque = K_p×e + K_i×∫e + K_d×ė + K_ff×sp             │
└──────────────────────┬──────────────────────────────────┘
                       ↓ vehicle_torque_setpoint
┌─────────────────────────────────────────────────────────┐
│ 控制分配器 (control_allocator)                          │
│                                                           │
│ 输入: 力矩设定 + 推力设定                                │
│ 输出: 电机PWM                                            │
│                                                           │
│ 分配: 力矩 → 4个/6个/8个电机的转速                      │
└─────────────────────────────────────────────────────────┘
```

---

## 十二、Yaw轴特殊处理

### 12.1 低通滤波器

**代码位置**: `MulticopterRateControl.cpp:223`

```cpp
// 偏航轴力矩低通滤波
torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);
```

**参数**:
```c
MC_YAW_TQ_CUTOFF = 2.0 Hz  // 默认截止频率
```

**原因**:
- 偏航力矩变化会引起旋翼加速度
- 旋翼加速度产生陀螺力矩
- 导致振动和噪声
- 低通滤波平滑力矩变化

### 12.2 Yaw PID特点

```
与 Roll/Pitch 的区别:

Roll/Pitch:
  • 依赖升力矢量倾斜
  • 响应快
  • 需要较大D增益 (阻尼)

Yaw:
  • 依赖差分推力
  • 惯性小
  • D增益通常为0
  • P增益较大 (快速响应)
  • 加低通滤波器 (减振)
```

---

## 十三、电池电压补偿

### 13.1 电压下降补偿

**代码位置**: `MulticopterRateControl.cpp:241-256`

```cpp
if (_param_mc_bat_scale_en.get()) {
    if (_battery_status_sub.updated()) {
        battery_status_s battery_status;
        if (_battery_status_sub.copy(&battery_status) &&
            battery_status.connected &&
            battery_status.scale > 0.f) {
            _battery_status_scale = battery_status.scale;
        }
    }

    if (_battery_status_scale > 0.f) {
        for (int i = 0; i < 3; i++) {
            vehicle_thrust_setpoint.xyz[i] *= _battery_status_scale;
            vehicle_torque_setpoint.xyz[i] *= _battery_status_scale;
        }
    }
}
```

**参数**:
```c
MC_BAT_SCALE_EN = 0  // 默认禁用
```

**作用**:
- 电池电压下降 → 电机扭矩下降
- 自动放大控制输出补偿
- 保持飞行特性一致

**例子**:
```
满电 (100%): scale = 1.0  → 输出不变
半电 (60%):  scale = 1.2  → 输出放大20%
低电 (30%):  scale = 1.5  → 输出放大50%
```

---

## 十四、性能监控

### 14.1 控制器状态发布

**话题**: `rate_ctrl_status`

**代码位置**: `MulticopterRateControl.cpp:226-229`

```cpp
rate_ctrl_status_s rate_ctrl_status{};
_rate_control.getRateControlStatus(rate_ctrl_status);
rate_ctrl_status.timestamp = hrt_absolute_time();
_controller_status_pub.publish(rate_ctrl_status);
```

**内容**:
```cpp
rate_ctrl_status.rollspeed_integ  = _rate_int(0);   // 横滚积分项
rate_ctrl_status.pitchspeed_integ = _rate_int(1);   // 俯仰积分项
rate_ctrl_status.yawspeed_integ   = _rate_int(2);   // 偏航积分项
```

**监控命令**:
```bash
listener rate_ctrl_status

# 查看积分项数值
# 正常范围: -0.3 ~ +0.3
# 如果接近限幅值，说明存在持续扰动
```

---

## 十五、总结

### 15.1 核心公式

$$\tau = K_p \cdot e + K_i \cdot \int_{t_0}^{t} i_{factor}(e) \cdot e \, dt + K_d \cdot (-\alpha) + K_{ff} \cdot \omega_{sp}$$

### 15.2 关键特性

| 特性 | 实现方式 | 优势 |
|------|----------|------|
| **抗饱和** | 条件积分 | 防止积分windup |
| **非线性I** | i_factor降低 | 防止大机动回弹 |
| **直接微分** | 使用测量的α | 避免噪声放大 |
| **前馈** | 可选FF项 | 提高跟踪性能 |
| **Yaw滤波** | 2Hz低通 | 减少振动 |

### 15.3 默认参数总结

| 轴 | P | I | D | 积分限幅 |
|----|---|---|---|----------|
| **Roll** | 0.15 | 0.2 | 0.003 | 0.30 |
| **Pitch** | 0.15 | 0.2 | 0.003 | 0.30 |
| **Yaw** | 0.2 | 0.1 | 0.0 | 0.30 |

### 15.4 与EKF2的配合

```
EKF2 (200Hz)                Rate Control (250-1000Hz)
    ↓                              ↓
vehicle_attitude            vehicle_angular_velocity
    ↓                              ↓
姿态控制器                   角速率PID控制器
    ↓                              ↓
vehicle_rates_setpoint ────→   torque_setpoint
```

**配合要点**:
- EKF2 提供高精度角速率 (来自Output Predictor)
- Rate Control 实现内环稳定
- 高频率保证控制精度

---

**文档版本**: v1.0
**创建日期**: 2025-11-04
**适用机型**: 多旋翼 (Multicopter)

