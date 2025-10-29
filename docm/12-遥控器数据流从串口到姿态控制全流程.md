# RC 遥控数据完整数据流分析：从串口中断到姿态控制器

## 概述

本文档详细梳理 PX4 多旋翼从 RC 遥控器接收机串口接收数据开始，经过协议解析、数据处理、滤波，最终送入姿态控制器参与 PID 计算的完整数据流路径。

**数据链路涉及的关键模块**：
1. RC 驱动模块（SBUS/DSM/CRSF 等）
2. rc_update 模块（数据校准和转换）
3. mc_att_control 姿态控制器（滤波和 PID 计算）

---

## 一、完整数据流概览

### 1.1 数据流图

```
┌─────────────────────────────────────────────────────────────────┐
│ [1] 遥控接收机硬件                                              │
│     SBUS/DSM/CRSF 串口接收                                      │
│     波特率: 100000 (SBUS) / 115200 (DSM) / 420000 (CRSF)      │
├─────────────────────────────────────────────────────────────────┤
│ 串口 UART (例如 /dev/ttyS3)                                     │
└─────────────────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────────────────┐
│ [2] RC 驱动模块 (工作队列: wq:ttyS3)                           │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │  sbus_rc     │  │   dsm_rc     │  │  crsf_rc     │         │
│  │  (最常用)    │  │  (Spektrum)  │  │ (TBS/ELRS)   │         │
│  └──────────────┘  └──────────────┘  └──────────────┘         │
│                                                                 │
│  功能:                                                          │
│  • 串口轮询读取字节流                                           │
│  • 协议解析 (帧同步、CRC校验)                                   │
│  • 解码通道数据 (11bit -> 16bit PWM值)                          │
│  • 提取信号质量 (RSSI, Link Quality)                           │
│  • 发布: input_rc                                               │
└─────────────────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────────────────┐
│ uORB 主题: input_rc                                             │
│  • 原始通道值 (values[18]: 988-2012us PWM)                      │
│  • 信号状态 (rc_lost, rssi)                                     │
│  • 更新频率: ~72 Hz (SBUS) / ~100 Hz (DSM)                      │
└─────────────────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────────────────┐
│ [3] rc_update 模块 (工作队列: wq:hp_default)                   │
│                                                                 │
│  功能:                                                          │
│  • 订阅 input_rc                                                │
│  • 通道校准 (应用 RCx_MIN/TRIM/MAX/REV)                         │
│  • 映射功能通道 (RC_MAP_ROLL/PITCH/YAW/THROTTLE)                │
│  • 归一化到 [-1, 1] 范围                                        │
│  • 解析开关状态                                                 │
│  • 发布: rc_channels                                            │
│  • 发布: manual_control_setpoint                                │
│  • 发布: manual_control_switches                                │
└─────────────────────────────────────────────────────────────────┘
                       ↓
          ┌────────────┴────────────┐
          ↓                         ↓
┌─────────────────────┐   ┌─────────────────────┐
│ uORB: rc_channels   │   │ uORB: manual_       │
│  • 校准后的通道值   │   │   control_setpoint  │
│  • 范围: [-1, 1]    │   │                     │
│  • 18 通道          │   │  • roll [-1, 1]     │
│                     │   │  • pitch [-1, 1]    │
│  (供其他模块使用)   │   │  • yaw [-1, 1]      │
│                     │   │  • throttle [-1, 1] │
│                     │   │  • 辅助通道         │
└─────────────────────┘   └─────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────┐
│ [4] mc_att_control 姿态控制器 (工作队列: wq:nav_and_controllers)│
│                                                                 │
│  功能:                                                          │
│  • 订阅 manual_control_setpoint                                 │
│  • 订阅 vehicle_attitude (当前姿态)                             │
│  • Roll/Pitch 滤波 (AlphaFilter, 时间常数 MC_MAN_TILT_TAU)     │
│  • Yaw 速率生成                                                 │
│  • Throttle 曲线映射                                            │
│  • 生成姿态四元数设定值                                         │
│  • 姿态 PID 控制器                                              │
│  • 发布: vehicle_attitude_setpoint                              │
│  • 发布: vehicle_rates_setpoint (角速率设定值)                  │
└─────────────────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────────────────┐
│ uORB: vehicle_rates_setpoint                                    │
│  • roll_rate (rad/s)                                            │
│  • pitch_rate (rad/s)                                           │
│  • yaw_rate (rad/s)                                             │
│  • thrust_body[3]                                               │
└─────────────────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────────────────┐
│ [5] mc_rate_control 角速率控制器 (工作队列: wq:rate_ctrl)       │
│                                                                 │
│  • PID 控制计算                                                 │
│  • 输出扭矩设定值                                               │
└─────────────────────────────────────────────────────────────────┘
```

---

## 二、详细数据流分析

### [1] RC 驱动层：串口接收和协议解析

#### 1.1 SBUS 协议驱动（最常用）

**文件位置**：
```
src/drivers/rc/sbus_rc/SbusRc.cpp
src/lib/rc/sbus.cpp
src/lib/rc/sbus.h
```

**工作队列**：
- 自动映射到串口对应的工作队列
- 例如：`/dev/ttyS3` → `wq:ttyS3`
- 通过 `serial_port_to_wq()` 函数映射

---

#### 1.2 串口初始化和配置

```cpp
// src/drivers/rc/sbus_rc/SbusRc.cpp:160-179

if (_rc_scan_begin == 0) {
    _rc_scan_begin = cycle_timestamp;

    // ⭐ 打开串口设备
    if (_rcs_fd < 0) {
        _rcs_fd = open(_device, O_RDWR | O_NONBLOCK);  // 第 165 行
    }

    // ⭐ 配置 SBUS 串口参数
    sbus_config(_rcs_fd, board_rc_singlewire(_device));  // 第 168 行

    // ⭐ 配置信号反相（SBUS 需要）
    if (!board_rc_invert_input(_device, true)) {
#if defined(TIOCSINVERT)
        ioctl(_rcs_fd, TIOCSINVERT, SER_INVERT_ENABLED_RX | SER_INVERT_ENABLED_TX);  // 第 174 行
#endif
    }

    // 清空串口缓冲区
    tcflush(_rcs_fd, TCIOFLUSH);  // 第 179 行
}
```

**SBUS 串口配置**：
```cpp
// src/lib/rc/sbus.cpp:109-159

int sbus_config(int sbus_fd, bool singlewire)
{
#if defined(__PX4_LINUX)
    struct termios2 t;

    if (ioctl(sbus_fd, TCGETS2, &t) != 0) {
        return -errno;
    }

    // ⭐ 设置波特率为 100000
    t.c_ispeed = 100000;  // 第 118 行
    t.c_ospeed = 100000;

    // ⭐ 设置为 8E2 (8位数据，偶校验，2个停止位)
    t.c_cflag &= ~CSIZE;
    t.c_cflag |= CS8;      // 8位数据位 第 122 行
    t.c_cflag |= PARENB;   // 使能校验 第 123 行
    t.c_cflag &= ~PARODD;  // 偶校验 第 124 行
    t.c_cflag |= CSTOPB;   // 2个停止位 第 125 行

    // 原始模式，无回显
    t.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    t.c_oflag &= ~OPOST;

    return ioctl(sbus_fd, TCSETS2, &t);
#else
    struct termios t;
    tcgetattr(sbus_fd, &t);

    cfsetspeed(&t, 100000);  // 第 144 行

    t.c_cflag |= (CLOCAL | CREAD | CS8 | PARENB | CSTOPB);  // 第 146 行
    t.c_cflag &= ~(PARODD | CRTSCTS);
    t.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    t.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    t.c_oflag = 0;

    return tcsetattr(sbus_fd, TCSANOW, &t);
#endif
}
```

**关键参数**：
- **波特率**：100000 bps
- **数据格式**：8E2 (8数据位，偶校验，2停止位)
- **信号反相**：SBUS 信号是反相的，需要硬件或软件反相
- **单线模式**：有些接收机支持单线半双工

---

#### 1.3 串口数据读取（主循环）

```cpp
// src/drivers/rc/sbus_rc/SbusRc.cpp:123-180

void SbusRc::Run()
{
    const hrt_abstime cycle_timestamp = hrt_absolute_time();

    constexpr hrt_abstime rc_scan_max = 3_s;

    // ⭐ 读取串口所有可用数据
    uint8_t rcs_buf[SBUS_BUFFER_SIZE] {};  // SBUS_BUFFER_SIZE = 256 第 151 行
    int newBytes = ::read(_rcs_fd, &rcs_buf[0], SBUS_BUFFER_SIZE);  // 第 152 行

    if (newBytes > 0) {
        _bytes_rx += newBytes;
    }

    const bool rc_scan_locked = _rc_scan_locked;

    if (_rc_scan_locked || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

        if (newBytes > 0) {
            uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
            uint16_t raw_rc_count = 0;
            bool sbus_failsafe = false;
            bool sbus_frame_drop = false;
            unsigned sbus_frame_drops = 0;

            // ⭐ 调用 SBUS 协议解析函数
            if (sbus_parse(cycle_timestamp, &rcs_buf[0], newBytes,
                          &raw_rc_values[0], &raw_rc_count,
                          &sbus_failsafe, &sbus_frame_drop, &sbus_frame_drops,
                          input_rc_s::RC_INPUT_MAX_CHANNELS)  // 第 191-192 行
               ) {
                // 解析成功，准备发布...
            }
        }
    }
}
```

**说明**：
- 使用 `read()` 系统调用轮询读取串口数据
- 非阻塞模式 (`O_NONBLOCK`)
- 每次读取最多 256 字节
- 调用协议解析器处理字节流

---

#### 1.4 SBUS 协议解析

**SBUS 帧格式**：
```
字节索引:  0     1-22         23         24
        ┌────┬──────────────┬─────────┬────────┐
        │0x0F│ 通道数据 22B  │ 标志位  │结束字节│
        └────┴──────────────┴─────────┴────────┘
          ^         ^            ^         ^
       起始符    16通道       failsafe   0x00/0x04/0x14/0x24
                 11bit编码     framelost
```

**解析函数**：
```cpp
// src/lib/rc/sbus.cpp:568-694

bool sbus_decode(uint64_t frame_time, uint8_t *frame, uint16_t *values,
                 uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop,
                 uint16_t max_values)
{
    // ⭐ 1. 检查起始字节
    if (frame[0] != SBUS_START_SYMBOL) {  // 0x0F 第 573 行
        sbus_frame_drops++;
        sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
        return false;
    }

    // ⭐ 2. 检查结束字节（确定 SBUS 版本）
    switch (frame[24]) {  // 第 589 行
        case 0x00:
            sbus_decode_state = SBUS2_DECODE_STATE_SBUS1_SYNC;
            break;
        case 0x04:
            sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE;
            break;
        // ... 其他 SBUS2 数据帧
        default:
            sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
            return false;
    }

    // ⭐ 3. 解码 16 个通道数据（每个通道 11 位）
    unsigned chancount = (max_values > SBUS_INPUT_CHANNELS) ?
                         SBUS_INPUT_CHANNELS : max_values;  // 第 624 行

    for (unsigned channel = 0; channel < chancount; channel++) {
        unsigned value = 0;

        // 使用解码矩阵提取 11 位数据
        for (unsigned pick = 0; pick < 3; pick++) {
            const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];  // 第 632 行

            if (decode->mask != 0) {
                unsigned piece = frame[1 + decode->byte];
                piece >>= decode->rshift;
                piece &= decode->mask;
                piece <<= decode->lshift;

                value |= piece;
            }
        }

        // ⭐ 4. 将 11 位值 (0-2047) 映射到 PWM 值 (1000-2000us)
        /*
         * Futaba FX-30/R6108SB 测量值:
         *   -+100% on TX: SBus 350/1024/1700 → PWM 1100/1520/1950us
         *   -+140% on TX: SBus  78/1024/1964 → PWM  930/1520/2112us
         */
        if (value == 0) {
            values[channel] = UINT16_MAX;  // 无效通道
        } else {
            // 线性映射: [200, 1800] → [1000, 2000]
            values[channel] = (uint16_t)((value - SBUS_RANGE_MIN) /
                                        (SBUS_RANGE_MAX - SBUS_RANGE_MIN) *
                                        (SBUS_TARGET_MAX - SBUS_TARGET_MIN) +
                                        SBUS_TARGET_MIN);  // 第 646-649 行
        }
    }

    // ⭐ 5. 提取标志位
    *sbus_failsafe = (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT));  // 第 653 行
    *sbus_frame_drop = (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT));  // 第 654 行

    *num_values = chancount;

    return true;
}
```

**SBUS 解码矩阵示例**（通道 0-2）：
```cpp
// src/lib/rc/sbus.cpp:520-565

static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
    /*  0 */ { {1, 0, 0xff, 0}, {2, 0, 0x07, 8}, {0, 0, 0x00, 0} },  // ch1: byte1[0:7] + byte2[0:2]
    /*  1 */ { {2, 3, 0xff, 0}, {3, 0, 0x3f, 8}, {0, 0, 0x00, 0} },  // ch2: byte2[3:7] + byte3[0:5]
    /*  2 */ { {3, 6, 0xff, 0}, {4, 0, 0xff, 8}, {5, 0, 0x01, 9} },  // ch3: byte3[6:7] + byte4[0:7] + byte5[0]
    // ... 其他 13 个通道
};
```

**位提取说明**：
- `byte`：从第几个字节读取
- `rshift`：右移位数
- `mask`：掩码
- `lshift`：左移位数

例如通道 0：
```
位布局: [byte2:0-2][byte1:0-7] = 11 位
值 = (byte1 & 0xff) << 0 | (byte2 & 0x07) << 8
```

---

#### 1.5 发布到 uORB: input_rc

```cpp
// src/drivers/rc/sbus_rc/SbusRc.cpp:191-244

if (sbus_parse(...)) {
    // ⭐ 构造 input_rc 消息
    input_rc_s input_rc{};
    input_rc.timestamp_last_signal = cycle_timestamp;  // 第 196 行
    input_rc.channel_count = math::constrain(raw_rc_count, (uint16_t)0,
                                             (uint16_t)input_rc_s::RC_INPUT_MAX_CHANNELS);  // 第 197 行
    input_rc.rssi = -1;  // SBUS 无 RSSI
    input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS;  // 第 199 行

    unsigned valid_chans = 0;

    // 填充通道值
    for (unsigned i = 0; i < input_rc.channel_count; i++) {
        input_rc.values[i] = raw_rc_values[i];  // 第 204 行

        if (raw_rc_values[i] != UINT16_MAX) {
            valid_chans++;
        }
    }

    input_rc.channel_count = valid_chans;  // 第 211 行

    // ⭐ 从通道中提取 RSSI（如果配置了 RC_RSSI_PWM_CHAN）
    if ((_param_rc_rssi_pwm_chan.get() > 0) &&
        (_param_rc_rssi_pwm_chan.get() < input_rc.channel_count)) {
        const int32_t rssi_pwm_chan = _param_rc_rssi_pwm_chan.get();
        const int32_t rssi_pwm_min = _param_rc_rssi_pwm_min.get();
        const int32_t rssi_pwm_max = _param_rc_rssi_pwm_max.get();

        // RSSI 通道值映射到 0-100
        int rc_rssi = ((input_rc.values[rssi_pwm_chan - 1] - rssi_pwm_min) * 100) /
                      (rssi_pwm_max - rssi_pwm_min);  // 第 219 行
        input_rc.rssi = math::constrain(rc_rssi, 0, 100);  // 第 220 行
    }

    if (valid_chans == 0) {
        input_rc.rssi = 0;
    }

    input_rc.rc_failsafe = sbus_failsafe;  // 第 227 行
    input_rc.rc_lost = (valid_chans == 0);  // 第 228 行
    input_rc.rc_lost_frame_count = sbus_frame_drops;  // 第 229 行

    input_rc.link_quality = -1;  // SBUS 无 Link Quality
    input_rc.rssi_dbm = NAN;     // SBUS 无 dBm RSSI

    // ⭐ 发布到 uORB
    input_rc.timestamp = hrt_absolute_time();  // 第 234 行
    _input_rc_pub.publish(input_rc);  // 第 235 行
    perf_count(_publish_interval_perf);

    _timestamp_last_signal = cycle_timestamp;
    rc_updated = true;

    if (valid_chans > 0) {
        _rc_scan_locked = true;  // 锁定协议
    }
}
```

**input_rc 数据结构**：
```cpp
// msg/input_rc.msg

uint64 timestamp               # 时间戳 (us)
uint64 timestamp_last_signal   # 最后有效信号时间戳 (us)

uint8 channel_count            # 通道数量 (0-18)
uint16[18] values              # 通道值 (988-2012 us PWM 范围)

int8 rssi                      # 信号强度 (0-100, -1=无效)
float32 rssi_dbm              # 信号强度 dBm (NAN=无效)
int8 link_quality             # 链路质量 (0-100, -1=无效)

bool rc_failsafe              # Failsafe 标志
bool rc_lost                  # 信号丢失标志
uint16 rc_lost_frame_count    # 丢帧计数

uint8 input_source            # 输入源 (SBUS=3, DSM=4, CRSF=6, etc)
```

---

### [2] rc_update 模块：数据校准和转换

**文件位置**：
```
src/modules/rc_update/rc_update.cpp
src/modules/rc_update/rc_update.h
```

**工作队列**：`wq:hp_default`

---

#### 2.1 模块初始化

```cpp
// src/modules/rc_update/rc_update.cpp:67-100

RCUpdate::RCUpdate() :
    ModuleParams(nullptr),
    WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)  // 第 69 行
{
    // ⭐ 初始化参数句柄（RC1_MIN, RC1_TRIM, RC1_MAX, RC1_REV 等）
    for (unsigned i = 0; i < RC_MAX_CHAN_COUNT; i++) {
        char nbuf[16];

        /* min values */
        snprintf(nbuf, sizeof(nbuf), "RC%d_MIN", i + 1);
        _parameter_handles.min[i] = param_find(nbuf);  // 第 77 行

        /* trim values */
        snprintf(nbuf, sizeof(nbuf), "RC%d_TRIM", i + 1);
        _parameter_handles.trim[i] = param_find(nbuf);  // 第 81 行

        /* max values */
        snprintf(nbuf, sizeof(nbuf), "RC%d_MAX", i + 1);
        _parameter_handles.max[i] = param_find(nbuf);  // 第 85 行

        /* channel reverse */
        snprintf(nbuf, sizeof(nbuf), "RC%d_REV", i + 1);
        _parameter_handles.rev[i] = param_find(nbuf);  // 第 89 行
    }

    // ... 其他初始化
}
```

---

#### 2.2 订阅 input_rc（回调触发）

```cpp
// src/modules/rc_update/rc_update.cpp:102-118

bool RCUpdate::init()
{
    // ⭐ 注册 input_rc 订阅回调
    if (!_input_rc_sub.registerCallback()) {  // 第 103 行
        PX4_ERR("input_rc callback registration failed!");
        return false;
    }

    ScheduleNow();
    return true;
}
```

**说明**：
- 使用 `SubscriptionCallbackWorkItem` 机制
- 当 `input_rc` 发布新数据时，自动触发 `Run()` 函数
- 回调在 `wq:hp_default` 工作队列中执行

---

#### 2.3 主处理循环：Run()

```cpp
// src/modules/rc_update/rc_update.cpp:345-520

void RCUpdate::Run()
{
    perf_begin(_loop_perf);

    // 检查参数更新
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
        parameters_updated(true);
    }

    // ⭐ 处理 input_rc 更新
    input_rc_s input_rc;
    if (_input_rc_sub.update(&input_rc)) {  // 第 358 行

        const bool input_source_stable = input_rc.input_source == _input_source;
        _input_source = input_rc.input_source;

        const bool channel_count_stable = input_rc.channel_count == _channel_count_previous;
        _channel_count_previous = input_rc.channel_count;

        const unsigned channel_count_limited = math::min<unsigned>(
            input_rc.channel_count, RC_MAX_CHAN_COUNT);  // 第 366 行

        // ⭐ 信号丢失检测
        bool signal_lost = false;
        if ((input_rc.rssi > 0) && (input_rc.rssi < 100)) {
            signal_lost = input_rc.rssi < _param_rc_rssi_th.get();  // 第 380 行
        }

        if (input_rc.link_quality >= 0) {
            signal_lost |= input_rc.link_quality < _param_rc_lq_th.get();  // 第 384 行
        }

        // ⭐ 读取并缩放所有通道值
        for (unsigned int i = 0; i < channel_count_limited; i++) {
            // 浮点转换
            const float value = input_rc.values[i];  // 第 440 行
            const float min = _parameters.min[i];
            const float trim = _parameters.trim[i];
            const float max = _parameters.max[i];

            // ⭐ 分段线性插值：应用 RC 校准
            // [min, trim, max] → [-1, 0, 1]
            _rc.channels[i] = math::interpolateNXY(
                value,
                {min, trim, max},   // 输入范围
                {-1.f, 0.f, 1.f}    // 输出范围
            );  // 第 446 行

            // ⭐ 应用反向
            if (_parameters.rev[i]) {
                _rc.channels[i] = -_rc.channels[i];  // 第 449 行
            }

            // 处理异常值
            if (!PX4_ISFINITE(_rc.channels[i])) {
                _rc.channels[i] = 0.f;
            }
        }

        // ⭐ 信号丢失迟滞（防止误判）
        // 信号恢复后 100ms 内的数据可能有毛刺，需要忽略
        _rc_signal_lost_hysteresis.set_hysteresis_time_from(true, 100_ms);  // 第 462 行
        _rc_signal_lost_hysteresis.set_state_and_update(signal_lost, hrt_absolute_time());

        _rc.channel_count = input_rc.channel_count;  // 第 465 行
        _rc.rssi = input_rc.rssi;
        _rc.signal_lost = _rc_signal_lost_hysteresis.get_state();
        _rc.timestamp = input_rc.timestamp_last_signal;
        _rc.frame_drop_count = input_rc.rc_lost_frame_count;

        // ⭐ 发布 rc_channels（即使信号无效也发布，用于调试）
        _rc_channels_pub.publish(_rc);  // 第 472 行

        // ⭐ 只在信号稳定且有效时发布 manual_control_setpoint
        if (input_source_stable && channel_count_stable &&
            !_rc_signal_lost_hysteresis.get_state()) {  // 第 475 行

            if ((input_rc.timestamp_last_signal > _last_timestamp_signal) &&
                (input_rc.timestamp_last_signal < _last_timestamp_signal + VALID_DATA_MIN_INTERVAL_US)) {

                perf_count(_valid_data_interval_perf);

                // 检查通道是否真的更新了
                bool rc_updated = false;
                for (unsigned i = 0; i < channel_count_limited; i++) {
                    if (_rc_values_previous[i] != input_rc.values[i]) {
                        rc_updated = true;
                        break;
                    }
                }

                // ⭐ 限制处理频率（通道有更新或 >300ms）
                if (rc_updated ||
                    (hrt_elapsed_time(&_last_manual_control_input_publish) > 300_ms)) {
                    UpdateManualControlInput(input_rc.timestamp_last_signal);  // 第 494 行
                }

                // ⭐ 更新开关状态
                UpdateManualSwitches(input_rc.timestamp_last_signal);  // 第 497 行

                // ⭐ 从 RC 更新参数（如果使能）
                if (hrt_elapsed_time(&_last_rc_to_param_map_time) > 1_s) {
                    set_params_from_rc();  // 第 501 行
                    _last_rc_to_param_map_time = hrt_absolute_time();
                }
            }

            _last_timestamp_signal = input_rc.timestamp_last_signal;

        } else {
            // RC 输入不稳定或丢失，清除开关状态
            if (_manual_switches_last_publish.timestamp_sample != 0) {
                _manual_switches_last_publish = {};
            }
        }

        memcpy(_rc_values_previous, input_rc.values,
               sizeof(input_rc.values[0]) * channel_count_limited);  // 第 515 行
    }

    perf_end(_loop_perf);
}
```

**关键操作总结**：
1. 读取 `input_rc` 数据
2. 应用校准参数（min/trim/max/rev）
3. 归一化到 [-1, 1]
4. 信号质量检测和迟滞
5. 发布 `rc_channels`
6. 生成并发布 `manual_control_setpoint`
7. 解析并发布开关状态

---

#### 2.4 生成 manual_control_setpoint

```cpp
// src/modules/rc_update/rc_update.cpp:663-687

void RCUpdate::UpdateManualControlInput(const hrt_abstime &timestamp_sample)
{
    manual_control_setpoint_s manual_control_input{};
    manual_control_input.timestamp_sample = timestamp_sample;  // 第 666 行
    manual_control_input.data_source = manual_control_setpoint_s::SOURCE_RC;  // 第 667 行

    // ⭐ 提取控制量（通过功能映射）
    // RC_MAP_ROLL/PITCH/YAW/THROTTLE 参数指定哪个通道对应哪个控制轴
    manual_control_input.roll = get_rc_value(rc_channels_s::FUNCTION_ROLL, -1.f, 1.f);  // 第 670 行
    manual_control_input.pitch = get_rc_value(rc_channels_s::FUNCTION_PITCH, -1.f, 1.f);  // 第 671 行
    manual_control_input.yaw = get_rc_value(rc_channels_s::FUNCTION_YAW, -1.f, 1.f);  // 第 672 行
    manual_control_input.throttle = get_rc_value(rc_channels_s::FUNCTION_THROTTLE, -1.f, 1.f);  // 第 673 行
    manual_control_input.flaps = get_rc_value(rc_channels_s::FUNCTION_FLAPS, -1.f, 1.f);  // 第 674 行
    manual_control_input.aux1  = get_rc_value(rc_channels_s::FUNCTION_AUX_1, -1.f, 1.f);  // 第 675 行
    manual_control_input.aux2  = get_rc_value(rc_channels_s::FUNCTION_AUX_2, -1.f, 1.f);  // 第 676 行
    manual_control_input.aux3  = get_rc_value(rc_channels_s::FUNCTION_AUX_3, -1.f, 1.f);  // 第 677 行
    manual_control_input.aux4  = get_rc_value(rc_channels_s::FUNCTION_AUX_4, -1.f, 1.f);  // 第 678 行
    manual_control_input.aux5  = get_rc_value(rc_channels_s::FUNCTION_AUX_5, -1.f, 1.f);  // 第 679 行
    manual_control_input.aux6  = get_rc_value(rc_channels_s::FUNCTION_AUX_6, -1.f, 1.f);  // 第 680 行
    manual_control_input.valid = _rc_calibrated;  // 第 681 行

    // ⭐ 发布 manual_control_setpoint
    manual_control_input.timestamp = hrt_absolute_time();  // 第 684 行
    _manual_control_input_pub.publish(manual_control_input);  // 第 685 行
    _last_manual_control_input_publish = manual_control_input.timestamp;
}
```

**get_rc_value() 函数**：
```cpp
// src/modules/rc_update/rc_update.cpp:265-272

float RCUpdate::get_rc_value(uint8_t func, float min_value, float max_value) const
{
    // ⭐ 根据功能映射获取通道值
    // 例如：RC_MAP_ROLL = 1 → 使用通道 0 (channels[0])
    if (_rc.function[func] >= 0) {
        return math::constrain(_rc.channels[_rc.function[func]],
                              min_value, max_value);  // 第 268 行
    }

    return 0.f;
}
```

**manual_control_setpoint 数据结构**：
```cpp
// msg/manual_control_setpoint.msg

uint64 timestamp
uint64 timestamp_sample

uint8 data_source        # RC=0, Mavlink=1, etc

float32 roll             # [-1, 1]  向右为正
float32 pitch            # [-1, 1]  向前为正
float32 yaw              # [-1, 1]  逆时针为正
float32 throttle         # [-1, 1]  向上为正

float32 flaps            # [-1, 1]
float32 aux1             # [-1, 1]
float32 aux2             # [-1, 1]
float32 aux3             # [-1, 1]
float32 aux4             # [-1, 1]
float32 aux5             # [-1, 1]
float32 aux6             # [-1, 1]

bool valid               # 校准是否完成
```

---

### [3] mc_att_control 姿态控制器：滤波和 PID 计算

**文件位置**：
```
src/modules/mc_att_control/mc_att_control_main.cpp
src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
```

**工作队列**：`wq:nav_and_controllers`

---

#### 3.1 订阅 manual_control_setpoint

```cpp
// src/modules/mc_att_control/mc_att_control.hpp:114-122

class MulticopterAttitudeControl : public ModuleBase<MulticopterAttitudeControl>,
                                   public ModuleParams,
                                   public px4::WorkItem
{
private:
    // ⭐ 订阅 manual_control_setpoint
    uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};  // 第 116 行

    // 当前手动控制设定值
    manual_control_setpoint_s _manual_control_setpoint{};  // 第 135 行

    // ... 其他成员
};
```

---

#### 3.2 主循环：Run()

```cpp
// src/modules/mc_att_control/mc_att_control_main.cpp:205-394

void MulticopterAttitudeControl::Run()
{
    // ⭐ 更新 manual_control_setpoint
    if (_manual_control_setpoint_sub.updated()) {
        const manual_control_setpoint_s manual_control_setpoint_prev = _manual_control_setpoint;
        _manual_control_setpoint_sub.copy(&_manual_control_setpoint);  // 第 218 行

        // 缓慢更新油门参考值（用于油门曲线）
        if (_manual_control_setpoint.throttle > manual_control_setpoint_prev.throttle) {
            _man_thr_sp_ref = _manual_control_setpoint.throttle;
        } else {
            _man_thr_sp_ref -= (_man_thr_sp_ref - _manual_control_setpoint.throttle) * _param_man_thr_dec.get();
        }
    }

    // 更新悬停油门估计
    if (_hover_thrust_estimate_sub.updated()) {
        hover_thrust_estimate_s hover_thrust_estimate;
        if (_hover_thrust_estimate_sub.update(&hover_thrust_estimate)) {
            if (hover_thrust_estimate.valid) {
                _hover_thrust_estimate = math::constrain(hover_thrust_estimate.hover_thrust, .05f, .9f);  // 第 231 行
            }
        }
    }

    // ⭐ 当姿态更新时运行控制器
    vehicle_attitude_s v_att;
    if (_vehicle_attitude_sub.update(&v_att)) {  // 第 243 行

        // ⭐ 计算 dt（限制在 0.2-20 ms）
        const float dt = math::constrain(((v_att.timestamp_sample - _last_run) * 1e-6f),
                                        0.0002f, 0.02f);  // 第 246 行
        _last_run = v_att.timestamp_sample;

        const Quatf q{v_att.q};

        // ... 省略部分代码

        bool run_att_ctrl = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
                           && (is_hovering || is_tailsitter_transition);

        if (run_att_ctrl) {
            // ⭐ 如果在手动/自稳模式，从摇杆输入生成姿态设定值
            if (_vehicle_control_mode.flag_control_manual_enabled &&
                !_vehicle_control_mode.flag_control_altitude_enabled &&
                !_vehicle_control_mode.flag_control_velocity_enabled &&
                !_vehicle_control_mode.flag_control_position_enabled) {  // 第 295-298 行

                // ⭐ 关键函数：从 manual_control_setpoint 生成姿态设定值
                generate_attitude_setpoint(q, dt);  // 第 300 行

            } else {
                // 其他模式下重置滤波器
                _man_roll_input_filter.reset(0.f);  // 第 303 行
                _man_pitch_input_filter.reset(0.f);
                _yaw_setpoint_stabilized = NAN;
                _stick_yaw.reset(Eulerf(q).psi(), _unaided_heading);
            }

            // 检查新的姿态设定值
            if (_vehicle_attitude_setpoint_sub.updated()) {
                vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

                if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint) &&
                    (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {

                    _attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d),
                                                         vehicle_attitude_setpoint.yaw_sp_move_rate);  // 第 316 行
                    _thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
                    _last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
                }
            }

            // ... 处理航向重置

            // ⭐ 姿态 PID 控制器
            Vector3f rates_sp = _attitude_control.update(q);  // 第 342 行

            // ... 自动调参逻辑

            // ⭐ 发布角速率设定值
            vehicle_rates_setpoint_s rates_setpoint{};
            rates_setpoint.roll = rates_sp(0);   // 第 359 行
            rates_setpoint.pitch = rates_sp(1);  // 第 360 行
            rates_setpoint.yaw = rates_sp(2);    // 第 361 行
            _thrust_setpoint_body.copyTo(rates_setpoint.thrust_body);
            rates_setpoint.timestamp = hrt_absolute_time();

            _vehicle_rates_setpoint_pub.publish(rates_setpoint);  // 第 365 行
        }
    }

    perf_end(_loop_perf);
}
```

---

#### 3.3 从摇杆输入生成姿态设定值（含滤波）

```cpp
// src/modules/mc_att_control/mc_att_control_main.cpp:137-202

void MulticopterAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt)
{
    vehicle_attitude_setpoint_s attitude_setpoint{};

    // ⭐ 避免在解锁手势时累积偏航误差
    const bool arming_gesture = (_manual_control_setpoint.throttle < -.9f) &&
                                (_param_mc_airmode.get() != 2);  // 第 142 行

    if (arming_gesture || !_heading_good_for_control) {
        _yaw_setpoint_stabilized = NAN;  // 解锁航向
    }

    // ⭐ Yaw 处理
    const float yaw = Eulerf(q).psi();  // 当前偏航角 第 148 行
    const float yaw_stick_input = math::expo_deadzone(_manual_control_setpoint.yaw,
                                                      .6f, _param_man_deadzone.get());  // 第 149 行
    _stick_yaw.generateYawSetpoint(attitude_setpoint.yaw_sp_move_rate,
                                   _yaw_setpoint_stabilized,
                                   yaw_stick_input, yaw, dt, _unaided_heading);  // 第 150-151 行

    /*
     * ⭐ Roll & Pitch 输入映射
     * ----------------------------------------
     * 控制两个角度：
     * - 倾斜角度：sqrt(roll² + pitch²)
     * - 倾斜方向：XY 平面上的最大倾斜方向，也定义运动方向
     *
     * 这允许简单地限制倾斜角度，飞行器朝摇杆指向的方向飞行，
     * 摇杆输入的变化是线性的。
     */

    // ⭐ 设置 Alpha 滤波器参数
    _man_roll_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());  // 第 163 行
    _man_pitch_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());  // 第 164 行

    // ⭐ 应用滤波器并乘以最大倾斜角
    // MC_MAN_TILT_MAX: 最大倾斜角 (默认 35°)
    // MC_MAN_TILT_TAU: 滤波时间常数 (默认 0.25s)
    Vector2f v = Vector2f(
        _man_roll_input_filter.update(_manual_control_setpoint.roll * _man_tilt_max),   // 第 167 行
        -_man_pitch_input_filter.update(_manual_control_setpoint.pitch * _man_tilt_max)  // 第 168 行
    );

    float v_norm = v.norm();  // 倾斜角度向量的模 第 169 行

    // ⭐ 限制到配置的最大倾斜角度
    if (v_norm > _man_tilt_max) {
        v *= _man_tilt_max / v_norm;  // 第 172 行
    }

    // ⭐ 从倾斜向量构造 Roll-Pitch 四元数
    Quatf q_sp_rp = AxisAnglef(v(0), v(1), 0.f);  // 第 175 行

    // 确保在航向解锁时有有效的姿态四元数
    const float yaw_setpoint = PX4_ISFINITE(_yaw_setpoint_stabilized) ?
                               _yaw_setpoint_stabilized : yaw;  // 第 177 行

    // ⭐ 构造 Yaw 四元数
    const Quatf q_sp_yaw(cosf(yaw_setpoint / 2.f), 0.f, 0.f, sinf(yaw_setpoint / 2.f));  // 第 182 行

    if (_vtol) {
        // VTOL 特殊处理：修正倾斜设定值以应对偏航误差
        AttitudeControlMath::correctTiltSetpointForYawError(q_sp_rp, q, q_sp_yaw);  // 第 190 行
    }

    // ⭐ 将期望的倾斜与偏航设定值对齐
    Quatf q_sp = q_sp_yaw * q_sp_rp;  // 第 194 行

    q_sp.copyTo(attitude_setpoint.q_d);  // 期望姿态四元数 第 196 行

    // ⭐ Throttle 处理：应用油门曲线
    attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_setpoint.throttle);  // 第 198 行

    attitude_setpoint.timestamp = hrt_absolute_time();
    _vehicle_attitude_setpoint_pub.publish(attitude_setpoint);  // 第 201 行
}
```

**关键滤波器：AlphaFilter**：
```cpp
// src/lib/mathlib/math/filter/AlphaFilter.hpp:51-88

class AlphaFilter
{
public:
    AlphaFilter() = default;
    explicit AlphaFilter(float sample_interval, float time_constant) {
        setParameters(sample_interval, time_constant);
    }

    /**
     * ⭐ 设置滤波器参数
     *
     * @param sample_interval 采样间隔 (秒)，例如 dt = 0.005s (200 Hz)
     * @param time_constant 滤波时间常数 (秒)，决定收敛速度
     */
    void setParameters(float sample_interval, float time_constant)
    {
        const float denominator = time_constant + sample_interval;  // 第 70 行

        if (denominator > FLT_EPSILON) {
            // ⭐ 计算滤波器系数
            // alpha 接近 1 → 响应快，滤波弱
            // alpha 接近 0 → 响应慢，滤波强
            _alpha = sample_interval / denominator;  // 第 73 行
        } else {
            _alpha = 1.f;  // 无滤波
        }

        _time_constant = time_constant;
    }

    /**
     * ⭐ 更新滤波器
     *
     * @param input 新的输入值
     * @return 滤波后的输出值
     */
    float update(float input)
    {
        // ⭐ 一阶低通滤波器（指数移动平均）
        // y[n] = α * x[n] + (1 - α) * y[n-1]
        return _state = input * _alpha + _state * (1.f - _alpha);  // 第 82 行
    }

    void reset(float val) { _state = val; }  // 第 84 行
    float getState() const { return _state; }

private:
    float _state{0.f};
    float _alpha{1.f};
    float _time_constant{0.f};
};
```

**滤波器作用**：
- **平滑摇杆输入**，避免突然的大幅度倾斜变化
- **时间常数 MC_MAN_TILT_TAU**：默认 0.25s
  - 值越大 → 响应越慢，越平滑
  - 值越小 → 响应越快，但可能抖动
- **独立滤波 Roll 和 Pitch**，保持线性响应特性

**示例计算**：
```
假设：
  dt = 0.005s (200 Hz 控制频率)
  MC_MAN_TILT_TAU = 0.25s

计算滤波系数：
  α = dt / (tau + dt) = 0.005 / (0.25 + 0.005) = 0.0196

摇杆输入变化时的响应：
  y[0] = 0.0 (初始)
  摇杆突变到 1.0
  y[1] = 0.0196 * 1.0 + 0.9804 * 0.0 = 0.0196
  y[2] = 0.0196 * 1.0 + 0.9804 * 0.0196 = 0.0388
  ...
  约 250ms (50 次迭代) 后达到 63% 最终值
```

---

#### 3.4 油门曲线处理

```cpp
// src/modules/mc_att_control/mc_att_control_main.cpp:120-134

float MulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
    // ⭐ 将摇杆输入从 [-1, 1] 映射到 [0, 1]
    const float throttle = (throttle_stick_input + 1.f) * .5f;  // 第 123 行

    // ⭐ 限制油门最小值和最大值
    const float thr_min = _manual_throttle_minimum.getState();  // 最小油门（根据是否着陆调整）
    const float thr_max = _manual_throttle_maximum.getState();  // 最大油门（根据是否准备好调整）

    // ⭐ 应用指数曲线（中心点为悬停油门）
    // MPC_MANTHR_MIN: 最小油门 (默认 0.08)
    // MPC_THR_HOVER: 悬停油门估计
    const float thrust = math::gradual3(throttle,
                                        thr_min,
                                        _hover_thrust_slew_rate.getState(),
                                        thr_max);  // 第 129 行

    // 限制到 [thr_min, thr_max]
    return math::min(thrust, _manual_throttle_maximum.getState());  // 第 133 行
}
```

**gradual3() 函数**：
```cpp
// src/lib/mathlib/math/Functions.hpp

template<typename T>
constexpr T gradual3(T value, T value_min, T value_middle, T value_max)
{
    // ⭐ 三段式非线性映射
    // [0, 0.5] → [value_min, value_middle]  指数曲线
    // [0.5, 1] → [value_middle, value_max]  指数曲线

    if (value < T(0.5)) {
        return math::interpolate(value, T(0), T(0.5), value_min, value_middle);
    } else {
        return math::interpolate(value, T(0.5), T(1), value_middle, value_max);
    }
}
```

**油门曲线效果**：
```
摇杆位置         映射后油门
   -1.0    →     0.08  (最小油门，MPC_MANTHR_MIN)
   -0.0    →     0.55  (悬停油门，MPC_THR_HOVER 估计)
   +1.0    →     1.00  (最大油门)

曲线特点：
  • 中间段(悬停附近)分辨率高，便于精细控制
  • 两端段响应快，便于快速机动
```

---

#### 3.5 姿态 PID 控制器

```cpp
// src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:55-108

Vector3f AttitudeControl::update(const Quatf &q) const
{
    Quatf qd = _attitude_setpoint_q;  // 期望姿态四元数

    // ⭐ 计算忽略偏航的简化期望姿态（优先 Roll 和 Pitch）
    const Vector3f e_z = q.dcm_z();      // 当前 Z 轴（体轴）第 60 行
    const Vector3f e_z_d = qd.dcm_z();   // 期望 Z 轴 第 61 行
    Quatf qd_red(e_z, e_z_d);           // 简化四元数 第 62 行

    if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
        // 特殊情况：飞行器和推力方向完全相反
        qd_red = qd;  // 第 68 行
    } else {
        // ⭐ 将从当前到期望推力向量的旋转转换为世界坐标系
        qd_red *= q;  // 第 73 行
    }

    // ⭐ 提取偏航增量四元数
    Quatf qd_dyaw = qd_red.inversed() * qd;  // 第 78 行
    qd_dyaw.canonicalize();
    qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);  // 第 81 行
    qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);  // 第 82 行

    // ⭐ 缩放偏航角度并重新组合期望姿态
    // _yaw_w: 偏航权重 (MC_YAW_WEIGHT, 默认 0.4)
    qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f,
                        sinf(_yaw_w * asinf(qd_dyaw(3))));  // 第 85 行

    // ⭐ 四元数姿态控制律：qe 是从 q 到 qd 的旋转
    const Quatf qe = q.inversed() * qd;  // 第 88 行

    // ⭐ 使用 sin(α/2) 缩放的旋转轴作为姿态误差（见四元数轴角定义）
    // 对于小角度: sin(α/2) ≈ α/2
    Vector3f eq = 2.f * qe.imag();  // 第 91 行

    if (qe(0) < 0.f) {
        eq = -eq;  // 确保最短路径旋转 第 94 行
    }

    // ⭐ 应用 P 控制器：角速率设定值 = Kp * 姿态误差
    // _proportional_gain: [MC_ROLL_P, MC_PITCH_P, MC_YAW_P]
    Vector3f rate_setpoint = eq.emult(_proportional_gain);  // 第 98 行

    // ⭐ 加上前馈角速率
    rate_setpoint += _rate_feedforward;  // 第 101 行

    // ⭐ 限制角速率到最大值
    // [MC_ROLLRATE_MAX, MC_PITCHRATE_MAX, MC_YAWRATE_MAX]
    return matrix::constrain(rate_setpoint,
                            -_rate_limit, _rate_limit);  // 第 105 行
}
```

**控制算法总结**：
1. **优先级加权**：Roll/Pitch 优先于 Yaw（通过 `MC_YAW_WEIGHT`）
2. **四元数误差计算**：`qe = q^{-1} * qd`
3. **P 控制器**：`ω_sp = Kp * eq`（无 I 和 D，因为角速率环有完整的 PID）
4. **前馈**：从摇杆生成的期望角速率
5. **限幅**：角速率限制在最大值以内

---

## 三、数据流时序分析

### 3.1 单次数据流时序图

```
时间轴 (ms):  0        14        16        21
              │         │         │         │
┌─────────────▼─────────┴─────────┴─────────┴─────┐
│ SBUS 帧到达（串口）                               │
│   14ms (72Hz) 一帧                               │
└──────────────────────────────────────────────────┘
              │
              ↓ ~100 us (串口读取+解析)
┌─────────────────────────────────────────────────┐
│ input_rc 发布                                   │
│   wq:ttyS3                                      │
└──────────────────────────────────────────────────┘
              │
              ↓ ~200 us (回调触发+校准)
┌─────────────────────────────────────────────────┐
│ manual_control_setpoint 发布                    │
│   wq:hp_default                                 │
└──────────────────────────────────────────────────┘
              │
              ↓ ~5 ms (等待姿态更新 193Hz)
┌─────────────────────────────────────────────────┐
│ generate_attitude_setpoint()                    │
│   • 滤波摇杆输入                                │
│   • 生成姿态四元数                              │
│   wq:nav_and_controllers                        │
└──────────────────────────────────────────────────┘
              │
              ↓ ~150 us (PID 计算)
┌─────────────────────────────────────────────────┐
│ vehicle_rates_setpoint 发布                     │
│   wq:nav_and_controllers                        │
└──────────────────────────────────────────────────┘
              │
              ↓ ~1.5 ms (等待下一次角速率控制 667Hz)
┌─────────────────────────────────────────────────┐
│ mc_rate_control PID 计算                        │
│   wq:rate_ctrl                                  │
└──────────────────────────────────────────────────┘
```

**总延迟分析**：
```
从 RC 接收机接收到角速率设定值：
  SBUS 串口读取:         ~100 us
  协议解析:              ~50 us
  rc_update 处理:        ~200 us
  等待姿态更新:          ~5 ms (193 Hz)
  姿态控制器计算:        ~150 us
  ─────────────────────────────
  总延迟:                ~5.5 ms

加上角速率控制和混控:
  等待角速率控制:        ~1.5 ms (667 Hz)
  角速率 PID:            ~80 us
  混控分配:              ~100 us
  ─────────────────────────────
  从 RC 接收到电机输出:  ~7.2 ms
```

---

### 3.2 频率层级

```
┌────────────────────────────────────────────────┐
│ 72 Hz   - SBUS 帧率（标准）                    │
│           300 Hz 可能但不可靠                  │
└────────────────────────────────────────────────┘
              ↓
┌────────────────────────────────────────────────┐
│ ~72 Hz  - rc_update 处理频率                   │
│           实际跟随 input_rc 发布频率           │
└────────────────────────────────────────────────┘
              ↓
┌────────────────────────────────────────────────┐
│ 193 Hz  - 姿态控制器频率                       │
│           跟随 vehicle_attitude 发布频率       │
│           (由 EKF2 决定)                       │
└────────────────────────────────────────────────┘
              ↓
┌────────────────────────────────────────────────┐
│ 667 Hz  - 角速率控制器频率                     │
│           内环高频控制                         │
└────────────────────────────────────────────────┘
```

**频率不匹配处理**：
- RC 数据更新慢（72 Hz），但姿态控制器运行快（193 Hz）
- 姿态控制器每次都使用**最新的** `manual_control_setpoint` 值
- 如果 RC 数据没更新，姿态控制器仍用上次的值继续运行
- 这样保证了快速的姿态响应，不受 RC 更新频率限制

---

## 四、关键参数总结

### 4.1 RC 校准参数

| 参数 | 说明 | 默认值 | 示例 |
|-----|------|-------|------|
| `RC1_MIN` | 通道 1 最小值 | 1000 | 988 (SBUS) |
| `RC1_TRIM` | 通道 1 中位值 | 1500 | 1500 |
| `RC1_MAX` | 通道 1 最大值 | 2000 | 2012 (SBUS) |
| `RC1_REV` | 通道 1 反向 | 0 (正向) | 1 (反向) |

**作用**：
- 将原始 PWM 值映射到 [-1, 1] 范围
- 补偿接收机输出的差异
- 支持通道反向

---

### 4.2 RC 功能映射参数

| 参数 | 说明 | 默认值 | 可能值 |
|-----|------|-------|--------|
| `RC_MAP_ROLL` | Roll 通道 | 1 | 1-18 |
| `RC_MAP_PITCH` | Pitch 通道 | 2 | 1-18 |
| `RC_MAP_THROTTLE` | Throttle 通道 | 3 | 1-18 |
| `RC_MAP_YAW` | Yaw 通道 | 4 | 1-18 |
| `RC_MAP_FLTMODE` | 飞行模式开关 | 5 | 0 (禁用), 1-18 |

**作用**：
- 将物理通道映射到逻辑功能
- 支持不同遥控器的通道顺序

---

### 4.3 姿态控制参数

| 参数 | 说明 | 默认值 | 单位 | 影响 |
|-----|------|-------|------|------|
| **MC_MAN_TILT_TAU** | Roll/Pitch 滤波时间常数 | 0.25 | s | 摇杆响应速度 |
| **MC_MAN_TILT_MAX** | 最大倾斜角 | 35 | deg | 最大倾斜限制 |
| **MC_MAN_DEADZONE** | 摇杆死区 | 0.05 | - | 中位死区大小 |
| **MC_YAW_WEIGHT** | 偏航权重 | 0.4 | - | Yaw 优先级 |
| **MC_ROLL_P** | Roll P 增益 | 6.5 | - | Roll 响应速度 |
| **MC_PITCH_P** | Pitch P 增益 | 6.5 | - | Pitch 响应速度 |
| **MC_YAW_P** | Yaw P 增益 | 2.8 | - | Yaw 响应速度 |
| **MC_ROLLRATE_MAX** | 最大 Roll 角速率 | 220 | deg/s | Roll 速率限制 |
| **MC_PITCHRATE_MAX** | 最大 Pitch 角速率 | 220 | deg/s | Pitch 速率限制 |
| **MC_YAWRATE_MAX** | 最大 Yaw 角速率 | 200 | deg/s | Yaw 速率限制 |

---

### 4.4 油门参数

| 参数 | 说明 | 默认值 | 单位 | 影响 |
|-----|------|-------|------|------|
| **MPC_THR_HOVER** | 悬停油门 | 0.5 | - | 油门曲线中心 |
| **MPC_MANTHR_MIN** | 最小油门 | 0.08 | - | 摇杆最低油门 |
| **MPC_MAN_THR_DEC** | 油门下降速率 | 0.05 | 1/s | 油门下降平滑 |

---

## 五、关键文件和函数索引

### 5.1 RC 驱动层

| 文件 | 关键函数 | 行号 | 作用 |
|-----|---------|------|------|
| **src/drivers/rc/sbus_rc/SbusRc.cpp** | `Run()` | 123-180 | SBUS 驱动主循环 |
|  | `sbus_parse()` | 191 | 调用解析器 |
| **src/lib/rc/sbus.cpp** | `sbus_config()` | 109-159 | 配置串口 |
|  | `sbus_decode()` | 568-694 | 解析 SBUS 帧 |
| **src/drivers/rc/dsm_rc/DsmRc.cpp** | `Run()` | 209-293 | DSM 驱动主循环 |
| **src/drivers/rc/crsf_rc/CrsfRc.cpp** | `Run()` | 120-226 | CRSF 驱动主循环 |

---

### 5.2 rc_update 模块

| 文件 | 关键函数 | 行号 | 作用 |
|-----|---------|------|------|
| **src/modules/rc_update/rc_update.cpp** | 构造函数 | 67-100 | 初始化参数句柄 |
|  | `init()` | 102-118 | 注册回调 |
|  | `Run()` | 345-520 | 主处理循环 |
|  | `get_rc_value()` | 265-272 | 获取功能通道值 |
|  | `UpdateManualControlInput()` | 663-687 | 生成 manual_control_setpoint |
|  | `UpdateManualSwitches()` | 596-661 | 解析开关状态 |

---

### 5.3 姿态控制器

| 文件 | 关键函数 | 行号 | 作用 |
|-----|---------|------|------|
| **src/modules/mc_att_control/mc_att_control_main.cpp** | `Run()` | 205-394 | 主循环 |
|  | `generate_attitude_setpoint()` | 137-202 | 从摇杆生成姿态设定值 |
|  | `throttle_curve()` | 120-134 | 油门曲线映射 |
| **src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp** | `update()` | 55-108 | 姿态 PID 控制 |

---

### 5.4 滤波器

| 文件 | 类/函数 | 行号 | 作用 |
|-----|---------|------|------|
| **src/lib/mathlib/math/filter/AlphaFilter.hpp** | `AlphaFilter` | 51-88 | 一阶低通滤波器 |
|  | `setParameters()` | 68-77 | 设置滤波参数 |
|  | `update()` | 81-83 | 滤波更新 |

---

## 六、调试方法

### 6.1 查看 RC 原始数据

```bash
# 监听 input_rc 主题
listener input_rc

# 输出示例:
# timestamp: 123456789
# timestamp_last_signal: 123456789
# channel_count: 16
# values[0]: 1500  (Roll 通道原始值)
# values[1]: 1500  (Pitch 通道原始值)
# values[2]: 1000  (Throttle 通道原始值)
# values[3]: 1500  (Yaw 通道原始值)
# rssi: 85
# rc_lost: false
```

---

### 6.2 查看校准后的数据

```bash
# 监听 rc_channels 主题
listener rc_channels

# 输出示例:
# timestamp: 123456789
# channel_count: 16
# channels[0]: 0.0     (Roll: -1 到 1)
# channels[1]: 0.0     (Pitch: -1 到 1)
# channels[2]: -1.0    (Throttle: -1 到 1)
# channels[3]: 0.0     (Yaw: -1 到 1)
# rssi: 85
# signal_lost: false
```

---

### 6.3 查看手动控制设定值

```bash
# 监听 manual_control_setpoint 主题
listener manual_control_setpoint

# 输出示例:
# timestamp: 123456789
# timestamp_sample: 123456789
# data_source: 0 (RC)
# roll: 0.0
# pitch: 0.0
# yaw: 0.0
# throttle: -1.0
# valid: true
```

---

### 6.4 查看姿态设定值

```bash
# 监听 vehicle_attitude_setpoint 主题
listener vehicle_attitude_setpoint

# 输出示例:
# timestamp: 123456789
# q_d[0]: 1.0     (四元数 w)
# q_d[1]: 0.0     (四元数 x)
# q_d[2]: 0.0     (四元数 y)
# q_d[3]: 0.0     (四元数 z)
# thrust_body[0]: 0.0
# thrust_body[1]: 0.0
# thrust_body[2]: -0.5  (向下推力)
# yaw_sp_move_rate: 0.0
```

---

### 6.5 查看角速率设定值

```bash
# 监听 vehicle_rates_setpoint 主题
listener vehicle_rates_setpoint

# 输出示例:
# timestamp: 123456789
# roll: 0.0    (rad/s)
# pitch: 0.0   (rad/s)
# yaw: 0.0     (rad/s)
# thrust_body[0]: 0.0
# thrust_body[1]: 0.0
# thrust_body[2]: -0.5
```

---

### 6.6 RC 校准检查

```bash
# 查看 RC 参数
param show RC*

# 查看功能映射
param show RC_MAP_*

# 查看姿态参数
param show MC_*

# 实时调参（例如增加滤波时间常数）
param set MC_MAN_TILT_TAU 0.5
```

---

## 七、常见问题和解决方案

### Q1: RC 数据不稳定、抖动

**可能原因**：
1. RC 信号质量差（RSSI 低）
2. 串口配置错误
3. 电磁干扰

**解决方案**：
```bash
# 1. 检查信号质量
listener input_rc
# 查看 rssi, link_quality

# 2. 检查串口配置
dmesg | grep sbus
# 应该看到 "valid SBUS"

# 3. 增加滤波时间常数
param set MC_MAN_TILT_TAU 0.35  # 默认 0.25
```

---

### Q2: 摇杆响应慢、延迟大

**可能原因**：
1. 滤波时间常数太大
2. RC 帧率低
3. 系统负载过高

**解决方案**：
```bash
# 1. 减小滤波时间常数
param set MC_MAN_TILT_TAU 0.15  # 默认 0.25

# 2. 检查系统负载
top
# CPU 占用应 < 80%

# 3. 检查 RC 更新频率
uorb top input_rc
# 应该 > 50 Hz
```

---

### Q3: 最大倾斜角限制不够

**解决方案**：
```bash
# 增加最大倾斜角
param set MC_MAN_TILT_MAX 45  # 默认 35 度

# 增加最大角速率
param set MC_ROLLRATE_MAX 300  # 默认 220 deg/s
param set MC_PITCHRATE_MAX 300  # 默认 220 deg/s
```

---

### Q4: 油门响应不线性

**解决方案**：
```bash
# 调整悬停油门估计
param set MPC_THR_HOVER 0.6  # 默认 0.5

# 调整最小油门
param set MPC_MANTHR_MIN 0.1  # 默认 0.08

# 调整油门下降速率
param set MPC_MAN_THR_DEC 0.08  # 默认 0.05
```

---

## 八、总结

### 8.1 数据流总结

```
串口接收 → 协议解析 → 校准归一化 → 滤波平滑 → PID 控制 → 电机输出
  (72Hz)    (100us)    (200us)     (滤波)    (150us)    (667Hz)

关键点:
  • 串口轮询读取，非中断驱动
  • 协议解析在 wq:ttyS3 工作队列
  • rc_update 在 wq:hp_default
  • 姿态控制在 wq:nav_and_controllers (193 Hz)
  • 角速率控制在 wq:rate_ctrl (667 Hz)
```

---

### 8.2 性能要点

1. **延迟优化**：
   - 总延迟 ~7 ms（从 RC 接收到电机输出）
   - 主要延迟在等待姿态更新（5 ms）
   - 滤波器增加 ~0.25s 的"感觉"延迟（63% 响应时间）

2. **频率匹配**：
   - RC 输入慢（72 Hz），但不影响控制回路
   - 姿态控制器使用最新的 RC 值运行在 193 Hz
   - 角速率控制器运行在 667 Hz，保证快速响应

3. **滤波权衡**：
   - 滤波时间常数越大 → 越平滑，但响应越慢
   - 滤波时间常数越小 → 响应越快，但可能抖动
   - 默认 0.25s 是经验最优值

4. **PID 分级**：
   - 姿态环：P 控制器（外环）
   - 角速率环：完整 PID 控制器（内环）
   - 串级结构保证稳定性和响应性

---

**最后更新**：2025-10-28
**作者**：PX4 架构分析

