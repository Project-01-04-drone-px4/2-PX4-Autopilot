# DShot 电机输出与双向遥测完整数据流分析

## 概述

本文档详细梳理 PX4 从混控分配器输出到 DShot 电机驱动的完整数据链路，包括：
1. 如何确定使用 PWM 还是 DShot
2. DShot 编码过程（0-2047 映射）
3. DMA 发送机制
4. 双向 DShot (Bidirectional DShot) 的 RPM 回传
5. eRPM 数据解析和发布

---

## 一、PWM vs DShot 选择机制

### 1.1 参数配置决定

**关键参数**：`PWM_MAIN_TIMx` 或 `PWM_AUX_TIMx`（x = 1-8）

| 参数值 | 协议 | 速率 | 说明 |
|-------|------|------|------|
| **-5** | DShot150 | 150 kbit/s | 最慢 DShot |
| **-4** | DShot300 | 300 kbit/s | 常用 DShot |
| **-3** | DShot600 | 600 kbit/s | 最快 DShot |
| **-1** | OneShot | - | OneShot 协议 |
| **50** | PWM 50Hz | 50 Hz | 标准 PWM |
| **100** | PWM 100Hz | 100 Hz | 快速 PWM |
| **200** | PWM 200Hz | 200 Hz | 高速 PWM |
| **400** | PWM 400Hz | 400 Hz | 超高速 PWM |
| **> 400** | 自定义 PWM | 自定义 | 任意 PWM 频率 |

**配置文件**：
```
src/drivers/pwm_out/module.yaml
```

```yaml
pwm_timer_param:
    description:
        short: Output Protocol Configuration for ${label}
        long: |
            Select which Output Protocol to use for outputs ${label}.
            Custom PWM rates can be used by directly setting any value >0.
    type: enum
    default: 400
    values:
        -5: DShot150    # ⭐ 负值表示 DShot
        -4: DShot300
        -3: DShot600
        -1: OneShot
        50: PWM 50 Hz
        100: PWM 100 Hz
        200: PWM 200 Hz
        400: PWM 400 Hz
    reboot_required: true
```

---

### 1.2 驱动模块选择

**板载配置文件示例**：
```
boards/micoair/h743/default.px4board
```

```ini
# 第 39 行：启用 PWM 输出驱动（可配置为 PWM 或 DShot）
CONFIG_DRIVERS_PWM_OUT=y

# 第 23 行：启用专用 DShot 驱动
CONFIG_DRIVERS_DSHOT=y
```

**驱动选择逻辑**：
1. **PWM 驱动** (`pwm_out`)：
   - 可以输出 PWM 或 DShot
   - 通过定时器参数切换协议
   - 文件：`src/drivers/pwm_out/PWMOut.cpp`

2. **DShot 驱动** (`dshot`)：
   - 专用 DShot 驱动
   - 支持双向 DShot
   - 支持 DShot 命令
   - 文件：`src/drivers/dshot/DShot.cpp`

**启动检测**：
```bash
# 查看当前使用的驱动
dmesg | grep -E "(pwm_out|dshot)"

# 输出示例（使用 DShot 驱动）：
# INFO  [dshot] mode: DShot600, channels: 4

# 输出示例（使用 PWM 驱动 + DShot 协议）：
# INFO  [pwm_out] PWM_MAIN: mode: DShot300, rate: 300000
```

---

## 二、数据流概览

### 2.1 完整数据流图

```
┌──────────────────────────────────────────────────────────────────┐
│ [1] 混控分配器 (control_allocator)                               │
│     工作队列: wq:rate_ctrl, 667 Hz                               │
│     输入: vehicle_torque_setpoint, vehicle_thrust_setpoint       │
│     输出: actuator_motors                                        │
│     范围: [-1, 1] (归一化)                                       │
├──────────────────────────────────────────────────────────────────┤
│ actuator_motors.control[0-3] = [-1, 1]                          │
└──────────────────────────────────────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────────┐
│ [2] DShot 驱动主循环                                             │
│     文件: src/drivers/dshot/DShot.cpp                            │
│     工作队列: wq:hp_default                                      │
│                                                                  │
│  (2.1) 订阅 actuator_motors                                      │
│  (2.2) MixingOutput::update() 触发                               │
│  (2.3) updateOutputs() 被回调                                    │
└──────────────────────────────────────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────────┐
│ [3] 电机速度映射和限制                                           │
│     DShot.cpp:375-431 updateOutputs()                            │
│                                                                  │
│  输入: outputs[i] ∈ [0, 1999]                                    │
│  输出: DShot值 ∈ [0, 2047]                                       │
│                                                                  │
│  • 0      : 停止 (DSHOT_DISARM_VALUE)                           │
│  • 1-47   : 保留给 DShot 命令                                   │
│  • 48-2047: 油门值 (2000 steps)                                 │
└──────────────────────────────────────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────────┐
│ [4] DShot 协议编码                                               │
│     文件: platforms/nuttx/src/px4/stm/stm32_common/dshot/       │
│           dshot.c:576-619 dshot_motor_data_set()                 │
│                                                                  │
│  DShot 帧格式 (16 位):                                           │
│  ┌──────────┬──┬────┐                                           │
│  │ 11-bit   │1 │4bit│                                           │
│  │ Throttle │T │CRC │                                           │
│  └──────────┴──┴────┘                                           │
│   Bits 1-11: 油门值 (0-2047)                                    │
│   Bit 12:    遥测请求位                                         │
│   Bits 13-16: XOR 校验和                                        │
└──────────────────────────────────────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────────┐
│ [5] DMA 缓冲区准备                                               │
│     dshot.c:614-618                                              │
│                                                                  │
│  将 16 位 DShot 帧转换为 PWM 脉冲序列:                           │
│  • 逻辑 1: 14/20 占空比 (MOTOR_PWM_BIT_1)                        │
│  • 逻辑 0:  7/20 占空比 (MOTOR_PWM_BIT_0)                        │
│                                                                  │
│  DMA 缓冲区: uint32_t[17 × num_motors]                           │
└──────────────────────────────────────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────────┐
│ [6] DMA 发送触发                                                 │
│     DShot.cpp:428 up_dshot_trigger()                             │
│     dshot.c:334-393 up_dshot_trigger()                           │
│                                                                  │
│  • 配置定时器为 DShot 模式                                       │
│  • 启动 DMA 传输 (Memory → Peripheral)                           │
│  • 定时器自动生成 PWM 波形                                       │
└──────────────────────────────────────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────────┐
│ 硬件输出: DShot 信号到 ESC                                       │
│                                                                  │
│  物理层: 单线数字信号                                            │
│  速率: 150/300/600 kbit/s                                        │
│  每帧时长: ~107us (DShot600) / ~213us (DShot300)                 │
└──────────────────────────────────────────────────────────────────┘
                           │
                           ↓ (双向 DShot 启用时)
┌──────────────────────────────────────────────────────────────────┐
│ [7] ESC 回传 eRPM 数据                                           │
│                                                                  │
│  发送完成后，ESC 在同一根线上回传 eRPM:                          │
│  • 延迟: ~30us                                                   │
│  • 编码: GCR (Group Code Recording)                              │
│  • 数据: 20 位 (16 位 eRPM + 4 位 CRC)                           │
└──────────────────────────────────────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────────┐
│ [8] DMA 捕获 eRPM 信号                                           │
│     dshot.c:395-499 dma_burst_finished_callback()                │
│                                                                  │
│  • DMA 发送完成中断触发                                          │
│  • 切换 GPIO 为输入捕获模式                                      │
│  • 启动 DMA 接收 (Peripheral → Memory)                           │
│  • 捕获脉冲宽度序列                                              │
└──────────────────────────────────────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────────┐
│ [9] eRPM 数据解码                                                │
│     dshot.c:547-569 process_capture_results()                    │
│                                                                  │
│  • 解析 GCR 编码的脉冲                                           │
│  • 提取 eRPM 值                                                  │
│  • 校验 CRC                                                      │
│  • 转换: eRPM → RPM = eRPM * 100 / (极对数 / 2)                  │
└──────────────────────────────────────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────────┐
│ [10] 发布 esc_status 主题                                        │
│      DShot.cpp:302-336 handle_new_bdshot_erpm()                  │
│      DShot.cpp:264-295 publish_esc_status()                      │
│                                                                  │
│  主题: esc_status                                                │
│  包含: RPM, 电压, 电流, 温度                                     │
│  订阅者: gyro_fft, logger, mavlink                               │
└──────────────────────────────────────────────────────────────────┘
```

---

## 三、详细代码分析

### [1] 混控分配器输出

**文件位置**：
```
src/modules/control_allocator/ControlAllocator.cpp
```

**关键代码**：
```cpp
// ControlAllocator.cpp:446 publish_actuator_controls()

void ControlAllocator::publish_actuator_controls()
{
    actuator_motors_s motors{};

    for (int i = 0; i < num_motors; i++) {
        // ⭐ 归一化输出范围 [-1, 1]
        motors.control[i] = actuator_sp[i];  // 混控分配结果
    }

    motors.timestamp = hrt_absolute_time();
    motors.reversible_flags = _control_allocation->getReversibleOutputs();

    // ⭐ 发布到 actuator_motors 主题
    _actuator_motors_pub.publish(motors);
}
```

**actuator_motors 数据结构**：
```cpp
// msg/actuator_motors.msg

uint64 timestamp                  # 时间戳 (us)
uint16 reversible_flags           # 可逆标志位掩码
float32[16] control               # 归一化电机输出 [-1, 1]
```

---

### [2] DShot 驱动订阅和处理

#### 2.1 初始化和订阅

```cpp
// src/drivers/dshot/DShot.cpp:44-53

DShot::DShot() :
    OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)  // 第 45 行
{
    // ⭐ 设置禁用值为 0 (停止电机)
    _mixing_output.setAllDisarmedValues(DSHOT_DISARM_VALUE);  // 第 47 行

    // ⭐ 设置最小值为 1 (0 保留给停止命令)
    _mixing_output.setAllMinValues(DSHOT_MIN_THROTTLE);  // 第 48 行

    // ⭐ 设置最大值为 1999
    _mixing_output.setAllMaxValues(DSHOT_MAX_THROTTLE);  // 第 49 行
}
```

**DShot 常量定义**：
```cpp
// src/drivers/dshot/DShot.h:56-58

static constexpr int DSHOT_DISARM_VALUE = 0;       // 停止值
static constexpr int DSHOT_MIN_THROTTLE = 1;       // 最小油门
static constexpr int DSHOT_MAX_THROTTLE = 1999;    // 最大油门 (0-2047, 保留48个命令)
```

---

#### 2.2 主循环

```cpp
// src/drivers/dshot/DShot.cpp:463-540

void DShot::Run()
{
    perf_begin(_cycle_perf);

    // ⭐ MixingOutput 更新 (订阅 actuator_motors)
    _mixing_output.update();  // 第 475 行
    // 这会触发 updateOutputs() 回调

    // 更新输出状态
    bool outputs_on = true;  // 简化
    if (_outputs_on != outputs_on) {
        enable_dshot_outputs(outputs_on);
    }

    // ⭐ 处理串口遥测 (如果启用)
    if (_telemetry) {
        const int telem_update = _telemetry->update(_num_motors);  // 第 485 行

        if (telem_update >= 0) {
            const int need_to_publish = handle_new_telemetry_data(
                telem_update,
                _telemetry->latestESCData(),
                _bidirectional_dshot_enabled);  // 第 488 行

            // 串口遥测和双向 DShot 不同时发布
            if (!_bidirectional_dshot_enabled && need_to_publish) {
                publish_esc_status();  // 第 493 行
            }
        }
    }

    // ⭐ 处理双向 DShot 的 eRPM 数据
    if (_bidirectional_dshot_enabled) {
        const int need_to_publish = handle_new_bdshot_erpm();  // 第 500 行

        if (need_to_publish) {
            publish_esc_status();  // 第 503 行
        }
    }

    // 参数更新
    if (_parameter_update_sub.updated()) {
        update_params();
    }

    // 处理 DShot 命令 (如反转方向, 3D 模式等)
    handle_vehicle_commands();  // 第 527 行

    // 检查订阅更新
    _mixing_output.updateSubscriptions(true);  // 第 537 行

    perf_end(_cycle_perf);
}
```

---

### [3] 电机速度映射和限制

#### 3.1 updateOutputs() 回调

```cpp
// src/drivers/dshot/DShot.cpp:375-431

bool DShot::updateOutputs(uint16_t outputs[MAX_ACTUATORS],
                         unsigned num_outputs,
                         unsigned num_control_groups_updated)
{
    if (!_outputs_on) {
        return false;
    }

    int requested_telemetry_index = -1;
    if (_telemetry) {
        requested_telemetry_index = _telemetry->getRequestMotorIndex();  // 第 385 行
    }

    int telemetry_index = 0;

    // ⭐ 遍历所有电机输出
    for (int i = 0; i < (int)num_outputs; i++) {

        uint16_t output = outputs[i];  // 第 392 行
        // 输入范围: [0, 1999] 或 DSHOT_DISARM_VALUE (0)

        // ⭐ 停止电机的情况
        if (output == DSHOT_DISARM_VALUE) {  // 第 394 行

            if (_current_command.valid() && (_current_command.motor_mask & (1 << i))) {
                // 如果有待发送的 DShot 命令
                up_dshot_motor_command(i, _current_command.command, true);  // 第 397 行

            } else {
                // 发送停止命令
                up_dshot_motor_command(i, DShot_cmd_motor_stop,
                                      telemetry_index == requested_telemetry_index);  // 第 400 行
            }

        } else {
            // ⭐ 正常油门输出

            // 处理 3D 模式或可逆输出
            if (_param_dshot_3d_enable.get() || (_reversible_outputs & (1u << i))) {
                output = convert_output_to_3d_scaling(output);  // 第 406 行
            }

            // ⭐ 设置 DShot 数据 (限制最大值)
            up_dshot_motor_data_set(i,
                                   math::min(output, static_cast<uint16_t>(DSHOT_MAX_THROTTLE)),
                                   telemetry_index == requested_telemetry_index);  // 第 409-410 行
        }

        telemetry_index += _mixing_output.isFunctionSet(i);  // 第 413 行
    }

    // ⭐ 命令计数器递减
    if (_current_command.valid()) {
        --_current_command.num_repetitions;  // 第 418 行

        // DShot 命令通常需要重复发送多次
        if (_current_command.num_repetitions == 0 && _current_command.save) {
            _current_command.save = false;
            _current_command.num_repetitions = 10;
            _current_command.command = dshot_command_t::DShot_cmd_save_settings;  // 第 424 行
        }
    }

    // ⭐ 触发 DMA 发送
    up_dshot_trigger();  // 第 428 行

    return true;
}
```

**关键点**：
- 输入：`outputs[i]` 范围 `[0, 1999]` (已经由 MixingOutput 限制)
- 输出：DShot 值 `[0, 2047]`
  - 0: 停止
  - 1-47: DShot 命令
  - 48-2047: 油门值
- 每次更新后调用 `up_dshot_trigger()` 发送

---

#### 3.2 3D 模式映射 (可选)

```cpp
// src/drivers/dshot/DShot.cpp:433-461

uint16_t DShot::convert_output_to_3d_scaling(uint16_t output)
{
    // ⭐ DShot 3D 模式将油门范围分为两段：
    // Direction 1) 48 是最慢, 1047 是最快
    // Direction 2) 1049 是最慢, 2047 是最快
    // 死区: DSHOT_3D_DEAD_L 到 DSHOT_3D_DEAD_H

    if (output >= _param_dshot_3d_dead_l.get() &&
        output < _param_dshot_3d_dead_h.get()) {
        return DSHOT_DISARM_VALUE;  // 死区内停止 第 440 行
    }

    bool upper_range = output >= 1000;  // 第 443 行

    if (upper_range) {
        output -= 1000;  // [1000, 1999] → [0, 999]
    } else {
        output = 999 - output;  // [0, 999] → [999, 0] 反向 第 449 行
    }

    // 应用最小油门限制
    float max_output = 999.f;
    float min_output = max_output * _param_dshot_min.get();  // 第 453 行
    output = math::min(max_output,
                      (min_output + output * (max_output - min_output) / max_output));  // 第 454 行

    if (upper_range) {
        output += 1000;  // 第 457 行
    }

    return output;
}
```

---

### [4] DShot 协议编码

**文件位置**：
```
platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c
```

#### 4.1 DShot 帧格式

```
DShot 协议 (16 位):
┌─────────────────────────────────────────────────────────────────┐
│ Bits 1-11 (11位): 油门值或命令                                 │
│   0:        电机停止                                            │
│   1-47:     特殊命令 (见下表)                                   │
│   48-2047:  油门值 (2000 steps)                                 │
├─────────────────────────────────────────────────────────────────┤
│ Bit 12 (1位): 遥测请求位                                        │
│   0:        不请求遥测                                          │
│   1:        请求遥测 (对于双向 DShot 无效)                      │
├─────────────────────────────────────────────────────────────────┤
│ Bits 13-16 (4位): CRC 校验和                                    │
│   XOR 校验 (标准 DShot)                                         │
│   Inverted XOR 校验 (双向 DShot)                                │
└─────────────────────────────────────────────────────────────────┘

示例:
  油门值 = 1000
  遥测请求 = 1

  Packet = (1000 << 5) | (1 << 4) | checksum
         = 0b 111 1101 0001 xxxx  (xxxx = CRC)
```

**DShot 命令表** (1-47):
| 命令值 | 枚举名 | 说明 |
|-------|--------|------|
| 0 | DShot_cmd_motor_stop | 电机停止 |
| 1 | DShot_cmd_beep1 | 蜂鸣 1 |
| 2 | DShot_cmd_beep2 | 蜂鸣 2 |
| 3 | DShot_cmd_beep3 | 蜂鸣 3 |
| 4 | DShot_cmd_beep4 | 蜂鸣 4 |
| 5 | DShot_cmd_beep5 | 蜂鸣 5 |
| 6 | DShot_cmd_esc_info | 请求 ESC 信息 |
| 7 | DShot_cmd_spin_direction_1 | 设置旋转方向 1 |
| 8 | DShot_cmd_spin_direction_2 | 设置旋转方向 2 |
| 9 | DShot_cmd_3d_mode_off | 禁用 3D 模式 |
| 10 | DShot_cmd_3d_mode_on | 启用 3D 模式 |
| 12 | DShot_cmd_save_settings | 保存设置到 ESC |
| 20 | DShot_cmd_spin_direction_normal | 正常旋转方向 |
| 21 | DShot_cmd_spin_direction_reversed | 反向旋转方向 |

---

#### 4.2 编码函数

```cpp
// platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c:576-619

/**
 * DShot 数据编码
 *
 * bits  1-11  - 油门值 (0-47 为命令, 48-2047 为油门)
 * bit   12    - 遥测使能/禁用
 * bits  13-16 - XOR 校验和
 */
void dshot_motor_data_set(unsigned channel, uint16_t data, bool telemetry)
{
    uint8_t timer_index = timer_io_channels[channel].timer_index;  // 第 578 行
    uint8_t timer_channel_index = timer_io_channels[channel].timer_channel - 1;
    bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];

    if (!channel_initialized) {
        return;
    }

    uint16_t packet = 0;
    uint16_t checksum = 0;

    // ⭐ 1. 组装数据包
    packet |= data << DSHOT_THROTTLE_POSITION;  // 位 [11:1] 第 589 行
    // DSHOT_THROTTLE_POSITION = 5 (左移 5 位)

    packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;  // 位 12 第 590 行
    // DSHOT_TELEMETRY_POSITION = 4

    uint16_t csum_data = packet;

    // ⭐ 2. XOR 校验和计算
    csum_data >>= NIBBLES_SIZE;  // 右移 4 位 第 595 行
    // NIBBLES_SIZE = 4

    for (uint8_t i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {  // 第 597 行
        checksum ^= (csum_data & 0x0F);  // 每个 nibble (4位) 进行 XOR 第 598 行
        csum_data >>= NIBBLES_SIZE;
    }
    // DSHOT_NUMBER_OF_NIBBLES = 3

    // ⭐ 3. 添加校验和 (双向 DShot 使用反向校验)
    if (_bidirectional) {
        packet |= ((~checksum) & 0x0F);  // 取反 第 603 行
    } else {
        packet |= ((checksum) & 0x0F);  // 第 606 行
    }

    // ⭐ 4. 转换为 PWM 脉冲序列
    const io_timers_channel_mapping_element_t *mapping =
        &io_timers_channel_mapping.element[timer_index];  // 第 610 行
    uint8_t num_motors = mapping->channel_count_including_gaps;
    uint8_t timer_channel = timer_io_channels[channel].timer_channel -
                           mapping->lowest_timer_channel;  // 第 612 行

    // 将 16 位数据转换为 16 个 PWM 脉冲
    for (uint8_t motor_data_index = 0; motor_data_index < ONE_MOTOR_DATA_SIZE; motor_data_index++) {
        // ONE_MOTOR_DATA_SIZE = 16

        dshot_output_buffer[timer_index][motor_data_index * num_motors + timer_channel] =
            (packet & 0x8000) ? MOTOR_PWM_BIT_1 : MOTOR_PWM_BIT_0;  // MSB 优先 第 615-616 行
        // MOTOR_PWM_BIT_1 = 14 (逻辑 1 的 PWM 占空比计数)
        // MOTOR_PWM_BIT_0 = 7  (逻辑 0 的 PWM 占空比计数)

        packet <<= 1;  // 左移，处理下一位
    }
}
```

**PWM 脉冲编码**：
```
DShot 使用 PWM 脉冲宽度编码数字位:

定时器周期 = 20 个计数 (对于 DShot600: 1/600kHz × 20 = 33.3us)

逻辑 1: 高电平 14 计数, 低电平 6 计数 (70% 占空比)
    ████████████████▁▁▁▁▁▁
    |<-- 14 -->|<-6->|

逻辑 0: 高电平 7 计数, 低电平 13 计数 (35% 占空比)
    ████████▁▁▁▁▁▁▁▁▁▁▁▁▁
    |<- 7->|<-- 13 --->|

一个完整的 DShot 帧 = 16 位 = 16 个 PWM 脉冲
帧时长 = 16 × 33.3us = 533us (DShot600)
```

---

### [5] DMA 缓冲区准备

#### 5.1 缓冲区结构

```cpp
// platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c:83-87

#define CHANNEL_OUTPUT_BUFF_SIZE    17u  // 16 位数据 + 1 位间隔
#define CHANNEL_CAPTURE_BUFF_SIZE   32u  // 捕获缓冲区 (双向 DShot)

#define DSHOT_OUTPUT_BUFFER_SIZE(channel_count) \
    (DMA_ALIGN_UP(sizeof(uint32_t) * CHANNEL_OUTPUT_BUFF_SIZE * channel_count))  // 第 86 行

#define DSHOT_CAPTURE_BUFFER_SIZE(channel_count) \
    (DMA_ALIGN_UP(sizeof(uint16_t) * CHANNEL_CAPTURE_BUFF_SIZE * channel_count))  // 第 87 行
```

```cpp
// dshot.c:116-123

// ⭐ 输出缓冲区 (所有定时器共享)
static uint8_t dshot_burst_buffer_array[
    MAX_IO_TIMERS * DSHOT_OUTPUT_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER)
] px4_cache_aligned_data() = {};  // 第 117-118 行

static uint32_t *dshot_output_buffer[MAX_IO_TIMERS] = {};  // 第 119 行

// ⭐ 捕获缓冲区 (双向 DShot)
static uint16_t dshot_capture_buffer[MAX_NUM_CHANNELS_PER_TIMER][CHANNEL_CAPTURE_BUFF_SIZE]
px4_cache_aligned_data() = {};  // 第 122-123 行
```

**缓冲区布局** (以 4 个电机为例):
```
dshot_output_buffer[timer_index]:
┌──────────────────────────────────────────────────────────┐
│ uint32_t[17 × 4] = 68 个 uint32_t                        │
├──────────────────────────────────────────────────────────┤
│ 交织存储 (Interleaved):                                  │
│                                                          │
│ [Bit0_M0, Bit0_M1, Bit0_M2, Bit0_M3,                     │
│  Bit1_M0, Bit1_M1, Bit1_M2, Bit1_M3,                     │
│  ...                                                     │
│  Bit15_M0, Bit15_M1, Bit15_M2, Bit15_M3,                 │
│  Gap_M0, Gap_M1, Gap_M2, Gap_M3]                         │
└──────────────────────────────────────────────────────────┘

每个 uint32_t 包含 4 个通道的同一位:
  Bit 0: 电机 0 的该位 PWM 值 (0-20)
  Bit 8: 电机 1 的该位 PWM 值
  Bit 16: 电机 2 的该位 PWM 值
  Bit 24: 电机 3 的该位 PWM 值
```

---

### [6] DMA 发送触发

#### 6.1 触发函数

```cpp
// platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c:334-393

void up_dshot_trigger()
{
    // ⭐ 1. 启用 DShot 模式
    io_timer_set_enable(true,
                       _bidirectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot,
                       IO_TIMER_ALL_MODES_CHANNELS);  // 第 337-338 行

    // ⭐ 2. 遍历所有使用的定时器
    for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {
        if (timer_configs[timer_index].enabled &&
            timer_configs[timer_index].initialized) {  // 第 342 行

            uint32_t channel_count =
                io_timers_channel_mapping.element[timer_index].channel_count_including_gaps;  // 第 344 行

            // ⭐ 3. 配置定时器为 DShot 突发模式
            io_timer_set_dshot_burst_mode(timer_index, _dshot_frequency, channel_count);  // 第 346 行

            if (_bidirectional) {
                // 双向 DShot: 释放并重新分配 DMA
                if (timer_configs[timer_index].dma_handle != NULL) {
                    stm32_dmastop(timer_configs[timer_index].dma_handle);  // 第 351 行
                    stm32_dmafree(timer_configs[timer_index].dma_handle);
                    timer_configs[timer_index].dma_handle = NULL;
                }

                // 分配 DMA 通道
                timer_configs[timer_index].dma_handle =
                    stm32_dmachannel(io_timers[timer_index].dshot.dma_map_up);  // 第 357 行

                if (timer_configs[timer_index].dma_handle == NULL) {
                    PX4_WARN("DMA allocation for timer %u failed", timer_index);
                    continue;
                }
            }

            // ⭐ 4. 刷新 D-Cache (确保 DMA 看到最新数据)
            up_clean_dcache((uintptr_t) dshot_output_buffer[timer_index],
                          (uintptr_t) dshot_output_buffer[timer_index] +
                          DSHOT_OUTPUT_BUFFER_SIZE(channel_count));  // 第 366-368 行

            // ⭐ 5. 配置 DMA
            px4_stm32_dmasetup(
                timer_configs[timer_index].dma_handle,
                io_timers[timer_index].base + STM32_GTIM_DMAR_OFFSET,  // 定时器 DMAR 寄存器
                (uint32_t)(dshot_output_buffer[timer_index]),           // 源地址
                channel_count * CHANNEL_OUTPUT_BUFF_SIZE,                // 传输大小
                DSHOT_DMA_SCR);  // DMA 配置 第 370-374 行

            // ⭐ 6. 清除 UDE 标志
            io_timer_update_dma_req(timer_index, false);  // 第 377 行

            // ⭐ 7. 启动 DMA 传输
            if (timer_configs[timer_index].bidirectional) {
                // 双向模式：注册完成回调
                stm32_dmastart(timer_configs[timer_index].dma_handle,
                              dma_burst_finished_callback,
                              &timer_configs[timer_index].timer_index,
                              false);  // 第 381-383 行
            } else {
                // 单向模式：无回调
                stm32_dmastart(timer_configs[timer_index].dma_handle,
                              NULL, NULL, false);  // 第 386 行
            }

            // ⭐ 8. 启用 DMA 更新请求
            io_timer_update_dma_req(timer_index, true);  // 第 390 行
        }
    }
}
```

**DMA 配置详解**：
```cpp
// dshot.c:69-74

// ⭐ DMA 流配置寄存器 (发送)
#define DSHOT_DMA_SCR (DMA_SCR_PRIHI |        // 高优先级
                      DMA_SCR_MSIZE_32BITS |  // 内存 32 位
                      DMA_SCR_PSIZE_32BITS |  // 外设 32 位
                      DMA_SCR_MINC |          // 内存地址递增
                      DMA_SCR_DIR_M2P |       // 方向: Memory → Peripheral
                      DMA_SCR_TCIE |          // 传输完成中断
                      DMA_SCR_TEIE |          // 传输错误中断
                      DMA_SCR_DMEIE)          // 直接模式错误中断

// ⭐ DMA 流配置寄存器 (双向接收)
#define DSHOT_BIDIRECTIONAL_DMA_SCR (DMA_SCR_PRIHI |
                                    DMA_SCR_MSIZE_16BITS |  // 内存 16 位
                                    DMA_SCR_PSIZE_16BITS |  // 外设 16 位
                                    DMA_SCR_MINC |
                                    DMA_SCR_DIR_P2M |       // 方向: Peripheral → Memory
                                    DMA_SCR_TCIE |
                                    DMA_SCR_TEIE |
                                    DMA_SCR_DMEIE)
```

**DMA 传输流程**：
```
1. CPU 准备 DMA 缓冲区 (dshot_output_buffer)
   └─ 包含所有电机的 PWM 脉冲宽度序列

2. 刷新 D-Cache
   └─ 确保 DMA 控制器看到最新数据

3. 配置 DMA
   └─ 源地址: dshot_output_buffer
   └─ 目标地址: TIMx_DMAR 寄存器
   └─ 传输大小: num_motors × 17 个 uint32_t

4. 启动 DMA
   └─ DMA 控制器自动传输数据到定时器

5. 定时器突发模式
   └─ 每次 DMA 请求，定时器读取 4 个通道的 PWM 值
   └─ 自动更新 CCR1-CCR4 寄存器
   └─ 生成 PWM 波形

6. 传输完成
   └─ 触发 DMA 完成中断 (双向模式)
   └─ 切换到接收模式
```

---

### [7] 双向 DShot：ESC 回传 eRPM

#### 7.1 DMA 完成回调

```cpp
// platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c:424-499

static void dma_burst_finished_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
    uint8_t timer_index = *((uint8_t *)arg);  // 第 426 行

    // ⭐ 1. 停止 DMA (发送已完成)
    stm32_dmastop(timer_configs[timer_index].dma_handle);  // 第 428 行

    // ⭐ 2. 禁用 DMA 更新请求
    io_timer_update_dma_req(timer_index, false);  // 第 431 行

    // ⭐ 3. 禁用所有通道 (准备切换为输入捕获)
    io_timer_set_enable(false, IOTimerChanMode_DshotInverted, IO_TIMER_ALL_MODES_CHANNELS);  // 第 434 行

    // ⭐ 4. 选择要捕获的通道
    select_next_capture_channel(timer_index);  // 第 437 行
    // 使用轮询方式，每次捕获一个通道

    uint8_t capture_channel = timer_configs[timer_index].capture_channel_index;  // 第 439 行

    // ⭐ 5. 重新配置通道为输入捕获模式
    uint8_t output_channel = output_channel_from_timer_channel(timer_index, capture_channel);  // 第 442 行
    io_timer_unallocate_channel(output_channel);  // 第 443 行
    io_timer_channel_init(output_channel, IOTimerChanMode_Capture, NULL, NULL);  // 第 444 行

    // ⭐ 6. 分配定时器用于捕获 DMA
    io_timer_allocate_timer(timer_index, IOTimerChanMode_Capture);  // 第 447 行

    // 释放发送 DMA
    stm32_dmafree(timer_configs[timer_index].dma_handle);  // 第 450 行

    // ⭐ 7. 分配捕获 DMA
    timer_configs[timer_index].dma_handle =
        stm32_dmachannel(timer_io_channels[output_channel].dma_map_capture);  // 第 453 行

    if (timer_configs[timer_index].dma_handle == NULL) {
        PX4_ERR("DMA alloc failed");
        return;
    }

    // ⭐ 8. 清空捕获缓冲区
    memset(dshot_capture_buffer[capture_channel], 0, sizeof(dshot_capture_buffer[capture_channel]));  // 第 460 行

    // 刷新 D-Cache
    up_clean_dcache((uintptr_t) dshot_capture_buffer,
                   (uintptr_t) dshot_capture_buffer + DSHOT_CAPTURE_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER));  // 第 463-465 行

    // ⭐ 9. 配置捕获 DMA
    px4_stm32_dmasetup(
        timer_configs[timer_index].dma_handle,
        io_timers[timer_index].base + (STM32_GTIM_CCR1_OFFSET + (capture_channel * 4)),  // CCRx 寄存器
        (uint32_t) &dshot_capture_buffer[capture_channel][0],  // 目标地址
        CHANNEL_CAPTURE_BUFF_SIZE,                             // 传输大小
        DSHOT_BIDIRECTIONAL_DMA_SCR);  // 第 468-473 行

    // ⭐ 10. 启动捕获 DMA (注册完成回调)
    stm32_dmastart(timer_configs[timer_index].dma_handle,
                  NULL,  // 不在中断中处理
                  NULL,
                  false);  // 第 476-479 行

    // ⭐ 11. 启用捕获 DMA 请求
    io_timer_capture_dma_req(timer_index, capture_channel, true);  // 第 482 行

    // ⭐ 12. 安排延迟处理 (200us 后)
    // ESC 在发送完成约 30us 后开始回传，总共约 150us
    hrt_call_after(&_cc_call, 200, capture_complete_callback,
                  &timer_configs[timer_index].timer_index);  // 第 486-487 行
}
```

**时序图**：
```
DShot 帧发送                    ESC 回传 eRPM
──────────────►                 ◄──────────────
|<-- 533us -->|  |<-30us->|     |<-- 150us -->|
                  延迟

DMA 发送完成中断触发:
    ↓
切换 GPIO 为输入捕获模式
    ↓
启动 DMA 接收 (CCRx → Memory)
    ↓
200us 后 HRT 回调
    ↓
解析捕获的数据
```

---

#### 7.2 捕获完成处理

```cpp
// platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c:502-545

static void capture_complete_callback(void *arg)
{
    perf_begin(hrt_callback_perf);  // 第 504 行

    uint8_t timer_index = *((uint8_t *)arg);  // 第 506 行

    // ⭐ 1. 释放定时器的捕获模式
    io_timer_unallocate_timer(timer_index);  // 第 509 行

    uint8_t capture_channel = timer_configs[timer_index].capture_channel_index;  // 第 511 行

    // ⭐ 2. 禁用捕获 DMA 请求
    io_timer_capture_dma_req(timer_index, capture_channel, false);  // 第 514 行

    // ⭐ 3. 停止 DMA
    stm32_dmastop(timer_configs[timer_index].dma_handle);  // 第 517 行

    // ⭐ 4. 重新初始化所有输出通道为 DShot 模式
    for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {
        bool is_this_timer = timer_index == timer_io_channels[output_channel].timer_index;
        uint8_t timer_channel_index = timer_io_channels[output_channel].timer_channel - 1;
        bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];

        if (is_this_timer && channel_initialized) {
            io_timer_unallocate_channel(output_channel);  // 第 528 行
            // 重新初始化为 DShotInverted 模式
            io_timer_channel_init(output_channel, IOTimerChanMode_DshotInverted, NULL, NULL);  // 第 530 行
        }
    }

    // ⭐ 5. 无效化 D-Cache (确保 CPU 看到最新数据)
    up_invalidate_dcache((uintptr_t) dshot_capture_buffer,
                        (uintptr_t) dshot_capture_buffer +
                        DSHOT_CAPTURE_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER));  // 第 535-536 行

    // ⭐ 6. 处理捕获结果 (解析 eRPM)
    process_capture_results(timer_index, capture_channel);  // 第 539 行

    // ⭐ 7. 重新启用所有 DShot 通道
    io_timer_set_enable(true, IOTimerChanMode_DshotInverted, IO_TIMER_ALL_MODES_CHANNELS);  // 第 542 行

    perf_end(hrt_callback_perf);  // 第 544 行
}
```

---

### [8] eRPM 数据解析

#### 8.1 处理捕获结果

```cpp
// platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c:547-569

void process_capture_results(uint8_t timer_index, uint8_t channel_index)
{
    // ⭐ 1. 计算脉冲周期
    const unsigned period = calculate_period(timer_index, channel_index);  // 第 549 行

    uint8_t output_channel = output_channel_from_timer_channel(timer_index, channel_index);  // 第 551 行

    if (period == 0) {
        // 解析失败，设置 eRPM 为 0
        _erpms[output_channel] = 0;  // 第 556 行

    } else if (period == 65408) {
        // ⭐ 特殊情况：电机静止
        _erpms[output_channel] = 0;  // 第 560 行

    } else {
        // ⭐ 2. 将周期转换为 eRPM
        // eRPM = 60,000,000 / (period × 100)
        _erpms[output_channel] = (1000000 * 60 / 100 + period / 2) / period;  // 第 564 行
        // 公式: eRPM = 600,000 / period (us)
    }

    // ⭐ 3. 标记数据就绪
    _erpms_ready[output_channel] = true;  // 第 568 行
}
```

**eRPM 计算公式**：
```
ESC 回传的数据编码了一个周期值:
  周期 (us) = 捕获的脉冲宽度

eRPM = 60,000,000 / (周期 × 100)
     = 600,000 / 周期

示例:
  周期 = 1000 us
  eRPM = 600,000 / 1000 = 600 eRPM

  实际 RPM = eRPM × 100 / (极对数 / 2)
           = 600 × 100 / (14 / 2)  (假设 14 极)
           = 60,000 / 7
           = 8571 RPM
```

---

#### 8.2 计算周期 (GCR 解码)

```cpp
// platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c (calculate_period函数)
// 此函数较长，简化说明其功能：

static unsigned calculate_period(uint8_t timer_index, uint8_t channel_index)
{
    // ⭐ GCR (Group Code Recording) 解码
    //
    // 双向 DShot 回传数据使用 GCR 编码:
    //   5 位 GCR → 4 位数据
    //   总共 20 位 GCR → 16 位数据
    //   16 位数据 = 12 位周期 + 4 位 CRC

    uint16_t *capture_data = dshot_capture_buffer[channel_index];

    // 1. 检测边沿，提取 GCR 位
    // 2. GCR 解码: 5-bit GCR → 4-bit data
    // 3. 组装 16 位数据
    // 4. 验证 CRC
    // 5. 提取周期值 (12 位)

    if (crc_valid) {
        unsigned period = (decoded_data >> 4) & 0x0FFF;  // 提取 12 位周期
        read_ok[channel_index]++;
        return period;
    } else {
        read_fail_crc[channel_index]++;
        return 0;  // 解析失败
    }
}
```

**GCR 编码表** (5 位 GCR → 4 位数据):
```
Data  GCR     Data  GCR
0000  11001   1000  10011
0001  11010   1001  10100
0010  11011   1010  10101
0011  11100   1011  10110
0100  11101   1100  10111
0101  11110   1101  11000
0110  11111   1110  01001
0111  10010   1111  01010
```

---

### [9] 获取 eRPM 数据

```cpp
// platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c:627-651

// ⭐ 检查有多少通道的 eRPM 数据就绪
int up_bdshot_num_erpm_ready(void)
{
    int num_ready = 0;  // 第 629 行

    for (unsigned i = 0; i < MAX_TIMER_IO_CHANNELS; ++i) {
        if (_erpms_ready[i]) {
            ++num_ready;
        }
    }

    return num_ready;  // 第 637 行
}

// ⭐ 获取指定通道的 eRPM 值
int up_bdshot_get_erpm(uint8_t output_channel, int *erpm)
{
    uint8_t timer_index = timer_io_channels[output_channel].timer_index;  // 第 642 行
    uint8_t timer_channel_index = timer_io_channels[output_channel].timer_channel - 1;
    bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];

    if (channel_initialized) {
        // ⭐ 返回 eRPM 值
        *erpm = _erpms[output_channel];  // 第 647 行

        // ⭐ 清除就绪标志
        _erpms_ready[output_channel] = false;  // 第 648 行

        return PX4_OK;
    }

    return -EINVAL;
}
```

---

### [10] 发布 esc_status 主题

#### 10.1 处理双向 DShot 的 eRPM

```cpp
// src/drivers/dshot/DShot.cpp:302-336

int DShot::handle_new_bdshot_erpm(void)
{
    int num_erpms = 0;  // 第 304 行
    int telemetry_index = 0;
    int erpm;
    esc_status_s &esc_status = esc_status_pub.get();  // 第 307 行

    // ⭐ 填充时间戳和基本信息
    esc_status.timestamp = hrt_absolute_time();  // 第 309 行
    esc_status.counter = _esc_status_counter++;
    esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;  // 第 311 行
    esc_status.esc_armed_flags = _outputs_on;

    // ⭐ 等待所有电机的数据都就绪
    if (up_bdshot_num_erpm_ready() < _num_motors) {  // 第 315 行
        return 0;  // 未全部就绪，不发布
    }

    // ⭐ 遍历所有输出通道
    for (unsigned i = 0; i < _num_outputs; i++) {
        if (_mixing_output.isFunctionSet(i)) {  // 第 320 行

            // ⭐ 获取 eRPM
            if (up_bdshot_get_erpm(i, &erpm) == 0) {  // 第 321 行
                num_erpms++;

                // ⭐ 设置在线标志
                esc_status.esc_online_flags |= 1 << telemetry_index;  // 第 323 行

                // ⭐ 填充数据
                esc_status.esc[telemetry_index].timestamp = hrt_absolute_time();  // 第 324 行

                // ⭐ 转换 eRPM 到 RPM
                // RPM = eRPM × 100 / (极对数 / 2)
                esc_status.esc[telemetry_index].esc_rpm =
                    (erpm * 100) / (_param_mot_pole_count.get() / 2);  // 第 325 行

                esc_status.esc[telemetry_index].actuator_function =
                    _actuator_functions[telemetry_index];  // 第 326 行
            }

            ++telemetry_index;  // 第 329 行
        }
    }

    perf_count(_bdshot_rpm_perf);  // 第 333 行

    return num_erpms;  // 第 335 行
}
```

---

#### 10.2 发布 esc_status

```cpp
// src/drivers/dshot/DShot.cpp:264-295

void DShot::publish_esc_status()
{
    esc_status_s &esc_status = esc_status_pub.get();  // 第 266 行

    // ⭐ 设置时间戳
    esc_status.timestamp = hrt_absolute_time();  // 第 268 行

    // ⭐ 发布主题
    esc_status_pub.publish();  // 第 288 行

    // ⭐ 清除在线标志 (下一次更新前)
    esc_status.esc_online_flags = 0;  // 第 299 行
}
```

---

#### 10.3 esc_status 数据结构

```cpp
// msg/esc_status.msg

uint64 timestamp                        # 时间戳 (us)

uint32 counter                          # 更新计数器
uint8 esc_count                         # ESC 数量
uint8 esc_connectiontype                # 连接类型
                                        #   0: PWM
                                        #   1: CAN
                                        #   2: OneShot
                                        #   3: DShot

uint8 CONNECTED_ESC_MAX = 8            # 最大 ESC 数量

uint8 esc_armed_flags                   # 解锁标志位掩码
uint8 esc_online_flags                  # 在线标志位掩码

# ⭐ 每个 ESC 的详细信息
EscReport[8] esc

# EscReport 结构:
struct EscReport {
    uint64 timestamp                    # 采样时间戳
    uint8 esc_address                   # ESC 地址 (CAN)
    uint16 esc_rpm                      # ⭐ 电机转速 (RPM)
    float32 esc_voltage                 # 电压 (V)
    float32 esc_current                 # 电流 (A)
    float32 esc_temperature             # 温度 (°C)
    uint16 actuator_function            # 执行器功能 ID
    uint8 failures                      # 故障标志
    int8 esc_power                      # 功率百分比 (%)
}
```

**示例数据**：
```
esc_status {
    timestamp: 123456789
    counter: 1234
    esc_count: 4
    esc_connectiontype: 3  (DShot)
    esc_armed_flags: 0x0F   (4 个电机全部解锁)
    esc_online_flags: 0x0F  (4 个 ESC 全部在线)

    esc[0] {
        timestamp: 123456789
        esc_rpm: 5234           ⭐ 电机 0: 5234 RPM
        esc_voltage: 12.6       (12.6V)
        esc_current: 3.2        (3.2A)
        esc_temperature: 45.0   (45°C)
        actuator_function: 101  (Motor 1)
    }

    esc[1] {
        esc_rpm: 5189           ⭐ 电机 1: 5189 RPM
        // ...
    }

    // ... esc[2], esc[3]
}
```

---

### [11] 谁订阅 esc_status？

#### 11.1 主要订阅者

```bash
# 查看订阅者
uorb top esc_status

# 输出示例:
# Publishers: 1
# Subscribers: 3
#   - gyro_fft (动态陷波滤波器)
#   - logger (数据记录)
#   - mavlink (遥测发送)
```

---

#### 11.2 gyro_fft 模块 (动态陷波滤波器)

**文件位置**：
```
src/modules/gyro_fft/GyroFFT.cpp
```

```cpp
// GyroFFT.cpp (订阅 esc_status)

class GyroFFT : public ModuleParams, public px4::ScheduledWorkItem
{
private:
    // ⭐ 订阅 esc_status
    uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};

    void Run() override {
        esc_status_s esc_status;

        // ⭐ 读取 ESC RPM 数据
        if (_esc_status_sub.updated()) {
            _esc_status_sub.copy(&esc_status);

            // ⭐ 使用 RPM 数据更新动态陷波滤波器
            for (int i = 0; i < esc_status.esc_count; i++) {
                float rpm = esc_status.esc[i].esc_rpm;

                // 计算基频和谐波频率
                float freq = rpm / 60.0f;  // Hz

                // 更新陷波滤波器中心频率
                // 见文档 07: FFT 动态陷波带宽计算
            }
        }
    }
};
```

**详见**：
- 文档 07: `FFT_Dynamic_Notch_Bandwidth_Calculation.md`
- 文档 05: `VehicleAngularVelocity_Data_Flow.md`

---

#### 11.3 logger 模块

**自动记录** `esc_status` 到 ULog 文件：
```bash
# 查看记录的 ESC 数据
ulog_info 2024-10-28_12-34-56.ulg | grep esc_status

# 输出示例:
# esc_status (150 messages, 10.5 kB)
```

---

#### 11.4 mavlink 模块

**发送 ESC 遥测到地面站**：
```cpp
// MAVLink ESC_INFO 消息 (ID 290)
mavlink_esc_info_t {
    uint64_t time_usec;              // 时间戳
    uint16_t counter;                // 计数器
    uint8_t count;                   // ESC 数量
    uint8_t connection_type;         // 连接类型
    uint8_t info;                    // 信息标志
    uint16_t failure_flags[4];       // 故障标志
    uint32_t error_count[4];         // 错误计数
    uint16_t temperature[4];         // 温度 (°C × 100)
}

// MAVLink ESC_STATUS 消息 (ID 291)
mavlink_esc_status_t {
    uint64_t time_usec;              // 时间戳
    int32_t rpm[4];                  // ⭐ RPM
    float voltage[4];                // 电压 (V)
    float current[4];                // 电流 (A)
}
```

地面站 (QGroundControl, Mission Planner) 可显示实时 RPM 数据。

---

## 四、完整数据流时序图

### 4.1 单次控制周期 (667 Hz)

```
时间轴 (us):  0         150       200       533       733       883
              │          │         │         │         │         │
┌─────────────▼──────────┴─────────┴─────────┴─────────┴─────────┴───┐
│ control_allocator 输出                                               │
│   混控计算 → actuator_motors 发布                                    │
└──────────────────────────────────────────────────────────────────────┘
              │
              ↓ ~50 us (MixingOutput 处理)
┌──────────────────────────────────────────────────────────────────────┐
│ DShot::updateOutputs()                                               │
│   • 映射速度 [0-1999]                                                │
│   • 编码 DShot 帧                                                    │
│   • 填充 DMA 缓冲区                                                  │
│   • 触发 DMA 发送                                                    │
└──────────────────────────────────────────────────────────────────────┘
              │
              ↓ DMA 自动传输 (~533 us)
┌──────────────────────────────────────────────────────────────────────┐
│ 硬件: 定时器生成 DShot 波形                                          │
│   16 位 × 33.3us/位 = 533us (DShot600)                               │
└──────────────────────────────────────────────────────────────────────┘
              │
              ↓ ESC 处理延迟 (~30 us)
┌──────────────────────────────────────────────────────────────────────┐
│ ESC 回传 eRPM (双向 DShot)                                           │
│   GCR 编码, 20 位, ~150us                                            │
└──────────────────────────────────────────────────────────────────────┘
              │
              ↓ DMA 捕获 (~150 us)
┌──────────────────────────────────────────────────────────────────────┐
│ dma_burst_finished_callback()                                        │
│   • 切换为输入捕获模式                                               │
│   • 启动捕获 DMA                                                     │
│   • 安排 HRT 回调 (200us)                                            │
└──────────────────────────────────────────────────────────────────────┘
              │
              ↓ 200 us 后
┌──────────────────────────────────────────────────────────────────────┐
│ capture_complete_callback()                                          │
│   • 解析 GCR 编码                                                    │
│   • 计算 eRPM                                                        │
│   • 转换为 RPM                                                       │
│   • 存储到 _erpms[]                                                  │
└──────────────────────────────────────────────────────────────────────┘
              │
              ↓ 下一个 Run() 周期
┌──────────────────────────────────────────────────────────────────────┐
│ DShot::handle_new_bdshot_erpm()                                      │
│   • 检查所有电机 eRPM 就绪                                           │
│   • 填充 esc_status 数据                                             │
│   • 发布 esc_status 主题                                             │
└──────────────────────────────────────────────────────────────────────┘
              │
              ↓
┌──────────────────────────────────────────────────────────────────────┐
│ 订阅者处理                                                           │
│   • gyro_fft: 更新动态陷波频率                                       │
│   • logger: 记录 RPM 数据                                            │
│   • mavlink: 发送到地面站                                            │
└──────────────────────────────────────────────────────────────────────┘
```

**总时长分析** (DShot600, 双向):
```
发送:    533 us  (16 位 DShot 帧)
延迟:     30 us  (ESC 处理)
回传:    150 us  (20 位 GCR eRPM)
处理:    200 us  (解析和转换)
─────────────────
总计:    913 us  (< 1500us 控制周期)

剩余:    587 us  (用于其他处理)
```

---

## 五、参数配置总结

### 5.1 DShot 基本参数

| 参数 | 说明 | 默认值 | 范围 |
|-----|------|-------|------|
| **PWM_MAIN_TIM1** | 主输出定时器 1 协议 | 400 | -5 到自定义 |
| | -5: DShot150 | | |
| | -4: DShot300 | | |
| | -3: DShot600 | | |
| **DSHOT_MIN** | DShot 最小输出 | 0.055 | 0.0 - 1.0 |
| **DSHOT_BIDIR_EN** | 启用双向 DShot | 0 (禁用) | 0 或 1 |
| **DSHOT_3D_ENABLE** | 启用 3D 模式 | 0 (禁用) | 0 或 1 |
| **MOT_POLE_COUNT** | 电机极对数 | 14 | 2 - 50 |

---

### 5.2 配置示例

#### 启用 DShot600 + 双向遥测

```bash
# 1. 设置输出协议为 DShot600
param set PWM_MAIN_TIM1 -3

# 2. 启用双向 DShot (需重启)
param set DSHOT_BIDIR_EN 1

# 3. 设置电机极对数 (根据实际电机)
param set MOT_POLE_COUNT 14

# 4. 设置最小油门 (确保电机始终旋转)
param set DSHOT_MIN 0.08

# 5. 保存并重启
param save
reboot
```

---

#### 验证双向 DShot 工作

```bash
# 1. 检查 DShot 驱动状态
dshot status

# 输出示例:
# DShot: mode DShot600, bidirectional enabled
# Motors: 4
# ESC telemetry: 4/4 online

# 2. 监听 esc_status
listener esc_status

# 输出示例:
# esc_status {
#     esc_count: 4
#     esc_online_flags: 0x0F
#     esc[0] { esc_rpm: 5234 ... }
#     esc[1] { esc_rpm: 5189 ... }
#     ...
# }

# 3. 查看性能统计
perf

# 查找 "bdshot_rpm" 计数器
```

---

## 六、故障排查

### 6.1 DShot 无输出

**症状**：电机不转，但系统正常

**检查**：
```bash
# 1. 验证 DShot 驱动已启动
dshot status

# 2. 检查混控输出
listener actuator_motors

# 3. 检查解锁状态
commander status

# 4. 检查电机映射
actuator_test

# 5. 手动测试电机
dshot  esc_identify 1
```

**常见原因**：
- ESC 不支持 DShot
- 定时器配置错误
- DMA 通道冲突
- 电源问题

---

### 6.2 双向 DShot 无 RPM 数据

**症状**：`esc_online_flags = 0` 或部分电机无 RPM

**检查**：
```bash
# 1. 确认双向 DShot 已启用
param show DSHOT_BIDIR_EN
# 应该为 1

# 2. 检查 ESC 支持
# BlHeli_32: 支持
# BlHeli_S: 不支持
# BLHeli32 固件版本 > 32.7

# 3. 查看解析统计
# 需要在代码中添加调试输出
```

**常见原因**：
- ESC 固件不支持双向 DShot
- 信号质量差（干扰、布线）
- 定时器捕获配置错误
- DMA 缓冲区对齐问题

---

### 6.3 RPM 值异常

**症状**：RPM 值为 0 或明显错误

**检查**：
```bash
# 1. 验证极对数设置
param show MOT_POLE_COUNT
# 应该与实际电机匹配

# 2. 计算公式验证
# RPM = eRPM × 100 / (极对数 / 2)

# 3. 查看原始 eRPM
# 需要在代码中添加调试输出
```

**常见原因**：
- `MOT_POLE_COUNT` 参数错误
- GCR 解码失败
- ESC 回传数据格式不兼容

---

## 七、总结

### 7.1 关键路径

```
混控分配 → DShot 编码 → DMA 发送 → ESC → DMA 捕获 → eRPM 解析 → esc_status 发布
[667Hz]    [50us]        [533us]    [30us]  [150us]      [100us]       [订阅者]

总延迟: ~863 us (< 1.5ms 控制周期)
```

---

### 7.2 性能要点

1. **DShot 优势**：
   - 数字信号，无需校准
   - CRC 校验，抗干扰强
   - 支持命令（反转、3D 等）
   - 双向遥测（RPM）

2. **双向 DShot 优势**：
   - 实时 RPM 反馈
   - 用于动态陷波滤波器
   - 无需额外遥测线
   - 低延迟（<1ms）

3. **开销**：
   - CPU: 很低（DMA 传输）
   - 内存: ~1KB 缓冲区
   - 延迟: ~900us (发送+接收)

---

### 7.3 关键文件索引

| 功能 | 文件 | 关键函数/行号 |
|-----|------|-------------|
| **DShot 驱动** | `src/drivers/dshot/DShot.cpp` | `updateOutputs()` (375行) |
|  |  | `Run()` (463行) |
|  |  | `handle_new_bdshot_erpm()` (302行) |
| **DShot 编码** | `platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c` | `dshot_motor_data_set()` (576行) |
| **DMA 发送** |  | `up_dshot_trigger()` (334行) |
| **DMA 捕获** |  | `dma_burst_finished_callback()` (424行) |
| **eRPM 解析** |  | `process_capture_results()` (547行) |
|  |  | `calculate_period()` |
| **获取 eRPM** |  | `up_bdshot_get_erpm()` (640行) |
| **主题定义** | `msg/esc_status.msg` | 数据结构 |

---

**最后更新**：2025-10-28
**作者**：PX4 架构分析

