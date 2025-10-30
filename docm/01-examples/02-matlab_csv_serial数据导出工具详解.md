# 02-matlab_csv_serial 数据导出工具详解

## 1. 模块概述

### 1.1 模块作用

`matlab_csv_serial` 是 PX4 提供的一个**传感器数据实时导出工具**，主要用于：

| 功能 | 说明 |
|------|------|
| **数据导出** | 将 IMU 数据实时输出到串口，格式为 CSV（逗号分隔值） |
| **MATLAB 集成** | 数据格式与 MATLAB 兼容，方便数据分析 |
| **实时分析** | 通过串口将飞控数据传输到电脑，实时绘图或分析 |
| **算法验证** | 导出原始传感器数据，在 MATLAB/Python 中验证算法 |
| **硬件测试** | 测试传感器性能、噪声水平、校准效果 |

**典型应用场景**：
- 🔬 传感器性能测试（噪声分析、频谱分析）
- 📊 数据记录与离线分析
- 🧪 算法开发（将数据导入 MATLAB 测试新算法）
- 🔍 故障诊断（分析异常振动、传感器故障）

---

## 2. 代码结构分析

### 2.1 文件组成

```
src/examples/matlab_csv_serial/
├── matlab_csv_serial.c     # 主程序（纯 C 语言实现）
├── CMakeLists.txt          # 编译配置
└── Kconfig                 # 菜单配置
```

**特点**：
- ✅ 使用传统 C 语言（而非 C++ 的 ModuleBase 框架）
- ✅ 传统的 `start/stop/status` 命令模式
- ✅ 使用全局变量和线程标志控制生命周期

---

### 2.2 编程模式对比

| 特性 | `matlab_csv_serial` | `fake_imu` |
|------|---------------------|-----------|
| **编程语言** | C 语言 | C++ 语言 |
| **框架** | 传统线程模式 | `ModuleBase` 框架 |
| **任务调度** | 独立线程 | 工作队列 |
| **生命周期管理** | 手动标志位 | `should_exit()` |
| **适用场景** | 简单工具、老代码移植 | 现代模块开发 |

---

## 3. 全局变量与控制标志

### 3.1 线程控制变量

```c
static bool thread_should_exit = false;   // 退出标志
static bool thread_running = false;       // 运行状态标志
static int daemon_task;                   // 线程 ID
```

**作用**：

| 变量 | 用途 | 类似概念 |
|------|------|---------|
| `thread_should_exit` | 外部请求退出时设为 true | `AppState::_exitRequested` |
| `thread_running` | 标记线程是否正在运行 | `AppState::_isRunning` |
| `daemon_task` | 保存线程 ID，用于管理 | 线程句柄 |

**与之前学习的 `AppState` 对比**：
- 这是**更原始的实现方式**
- `AppState` 是对这种模式的**封装**
- 功能相同，但 `AppState` 更面向对象

---

## 4. 主入口函数详解

### 4.1 完整代码分析

```c
int matlab_csv_serial_main(int argc, char *argv[])
{
    // ========== 1️⃣ 参数检查 ==========
    if (argc < 2) {
        usage("missing command");
    }

    // ========== 2️⃣ start 命令 ==========
    if (!strcmp(argv[1], "start")) {
        // 检查是否已经运行
        if (thread_running) {
            warnx("already running\n");
            exit(0);
        }

        // 清除退出标志
        thread_should_exit = false;

        // 创建后台线程
        daemon_task = px4_task_spawn_cmd(
            "matlab_csv_serial",        // 线程名称
            SCHED_DEFAULT,               // 调度策略：默认
            SCHED_PRIORITY_MAX - 5,      // 优先级：最高优先级 - 5
            2000,                        // 栈大小：2KB
            matlab_csv_serial_thread_main,  // 线程入口函数
            (argv) ? (char *const *)&argv[2] : (char *const *)NULL  // 传递参数
        );
        exit(0);
    }

    // ========== 3️⃣ stop 命令 ==========
    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;  // 设置退出标志
        exit(0);
    }

    // ========== 4️⃣ status 命令 ==========
    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("running");
        } else {
            warnx("stopped");
        }
        exit(0);
    }

    // ========== 5️⃣ 未知命令 ==========
    usage("unrecognized command");
    exit(1);
}
```

---

### 4.2 命令行参数传递

**启动方式**：
```bash
nsh> matlab_csv_serial start /dev/ttyS1
```

**参数传递流程**：
```
argv[0] = "matlab_csv_serial"  # 命令名
argv[1] = "start"              # 子命令
argv[2] = "/dev/ttyS1"         # 串口设备路径
         ↓
&argv[2] 传递给线程入口函数
         ↓
matlab_csv_serial_thread_main() 的 argv[1]
```

---

## 5. 线程主函数详解

### 5.1 函数执行流程

```c
int matlab_csv_serial_thread_main(int argc, char *argv[])
{
    // ========== 1️⃣ 参数检查 ==========
    if (argc < 2) {
        errx(1, "need a serial port name as argument");
    }

    const char *uart_name = argv[1];  // 获取串口名称
    warnx("opening port %s", uart_name);

    // ========== 2️⃣ 打开串口 ==========
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
    unsigned speed = 921600;  // 波特率：921600 bps

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
    }

    // ========== 3️⃣ 配置串口 ==========
    struct termios uart_config;
    int termios_state;

    // 读取当前配置
    if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
        warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
        close(serial_fd);
        return -1;
    }

    // 清除 ONLCR 标志（禁止 LF 自动转换为 CRLF）
    uart_config.c_oflag &= ~ONLCR;

    // ========== 4️⃣ 设置波特率（非 USB 串口） ==========
    // USB 串口（/dev/ttyACM0/1）不需要设置波特率
    if (strcmp(uart_name, "/dev/ttyACM0") != OK &&
        strcmp(uart_name, "/dev/ttyACM1") != OK) {

        if (cfsetispeed(&uart_config, speed) < 0 ||
            cfsetospeed(&uart_config, speed) < 0) {
            warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
            close(serial_fd);
            return -1;
        }
    }

    // 应用配置
    if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR SET CONF %s\n", uart_name);
        close(serial_fd);
        return -1;
    }

    // ========== 5️⃣ 订阅传感器主题 ==========
    struct sensor_accel_s accel0;
    struct sensor_accel_s accel1;
    struct sensor_gyro_s gyro0;
    struct sensor_gyro_s gyro1;

    int accel0_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);  // 加速度计 0
    int accel1_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 1);  // 加速度计 1
    int gyro0_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 0);    // 陀螺仪 0
    int gyro1_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 1);    // 陀螺仪 1

    // ========== 6️⃣ 设置运行标志 ==========
    thread_running = true;

    // ========== 7️⃣ 主循环 ==========
    while (!thread_should_exit) {
        // poll 等待数据
        struct pollfd fds[] = {
            { .fd = accel0_sub, .events = POLLIN }  // 等待 accel0 更新
        };

        // 等待最多 500ms
        int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 500);

        if (ret < 0) {
            // poll 错误，忽略
        } else if (ret == 0) {
            // 超时（500ms 内无数据）
            warnx("no sensor data");
        } else {
            // 有数据到达
            if (fds[0].revents & POLLIN) {
                // ========== 8️⃣ 读取所有传感器数据 ==========
                orb_copy(ORB_ID(sensor_accel), accel0_sub, &accel0);
                orb_copy(ORB_ID(sensor_accel), accel1_sub, &accel1);
                orb_copy(ORB_ID(sensor_gyro), gyro0_sub, &gyro0);
                orb_copy(ORB_ID(sensor_gyro), gyro1_sub, &gyro1);

                // ========== 9️⃣ 输出 CSV 格式数据 ==========
                dprintf(serial_fd,
                    "%"PRId64",%d,%d,%d,%d,%d,%d\n",
                    accel0.timestamp,   // 时间戳
                    (int)accel0.x,      // 加速度计 0：X 轴
                    (int)accel0.y,      // 加速度计 0：Y 轴
                    (int)accel0.z,      // 加速度计 0：Z 轴
                    (int)accel1.x,      // 加速度计 1：X 轴
                    (int)accel1.y,      // 加速度计 1：Y 轴
                    (int)accel1.z       // 加速度计 1：Z 轴
                );
            }
        }
    }

    // ========== 🔟 清理退出 ==========
    warnx("exiting");
    thread_running = false;
    fflush(stdout);
    return 0;
}
```

---

## 6. 关键技术点详解

### 6.1 串口配置

#### ONLCR 标志

```c
uart_config.c_oflag &= ~ONLCR;
```

**作用**：
- `ONLCR`（Output NL to CR-NL）：将换行符（LF, `\n`）自动转换为回车换行（CRLF, `\r\n`）
- **清除此标志**：保持原始的 `\n`，避免 CSV 格式错乱

**示例**：
```
启用 ONLCR：
  "123,456,789\n" → "123,456,789\r\n"  ❌ MATLAB 可能解析错误

禁用 ONLCR：
  "123,456,789\n" → "123,456,789\n"    ✅ 标准 CSV 格式
```

---

#### USB 串口特殊处理

```c
if (strcmp(uart_name, "/dev/ttyACM0") != OK &&
    strcmp(uart_name, "/dev/ttyACM1") != OK) {
    // 只对硬件串口设置波特率
    cfsetispeed(&uart_config, speed);
    cfsetospeed(&uart_config, speed);
}
```

**原因**：
- USB 虚拟串口（CDC-ACM）不需要设置波特率
- 波特率由 USB 协议自动协商
- 尝试设置可能导致错误

---

### 6.2 uORB 多实例订阅

```c
int accel0_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);  // 实例 0
int accel1_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 1);  // 实例 1
```

**多实例订阅**：
- 同一个主题可以有多个发布者（多个传感器）
- `orb_subscribe_multi()` 通过实例编号区分
- 编号从 0 开始

**典型应用**：
- 双 IMU 系统（主 IMU + 备份 IMU）
- 多冗余传感器（提高可靠性）

---

### 6.3 Poll 机制

```c
struct pollfd fds[] = {
    { .fd = accel0_sub, .events = POLLIN }
};

int ret = poll(fds, 1, 500);  // 等待最多 500ms
```

**Poll 工作原理**：

```
┌─────────────┐
│ 主循环阻塞   │
│  poll()     │
└──────┬──────┘
       │
       ├─► 情况1：accel0 有新数据 → 立即返回 → 读取数据
       │
       ├─► 情况2：500ms 超时 → 返回 0 → 打印警告
       │
       └─► 情况3：错误 → 返回 -1 → 忽略
```

**优势**：
- ✅ 节省 CPU（不需要忙等）
- ✅ 数据驱动（有数据时立即处理）
- ✅ 超时保护（避免永久阻塞）

---

### 6.4 数据同步策略

```c
// 以 accel0 为触发源
if (fds[0].revents & POLLIN) {
    // 同时读取所有传感器数据
    orb_copy(ORB_ID(sensor_accel), accel0_sub, &accel0);
    orb_copy(ORB_ID(sensor_accel), accel1_sub, &accel1);
    orb_copy(ORB_ID(sensor_gyro), gyro0_sub, &gyro0);
    orb_copy(ORB_ID(sensor_gyro), gyro1_sub, &gyro1);
}
```

**同步策略**：
- 以 `accel0` 作为**主时钟**
- `accel0` 更新时，读取所有传感器的**最新数据**
- 保证所有数据的时间戳相近

**潜在问题**：
- ⚠️ 如果其他传感器更新慢，可能读取到旧数据
- ⚠️ 不同传感器的采样率不同时，数据可能不对齐

---

## 7. 输出数据格式详解

### 7.1 CSV 格式

```c
dprintf(serial_fd, "%"PRId64",%d,%d,%d,%d,%d,%d\n",
    accel0.timestamp,   // 列 1：时间戳（微秒）
    (int)accel0.x,      // 列 2：加速度计 0 X 轴（原始值）
    (int)accel0.y,      // 列 3：加速度计 0 Y 轴
    (int)accel0.z,      // 列 4：加速度计 0 Z 轴
    (int)accel1.x,      // 列 5：加速度计 1 X 轴
    (int)accel1.y,      // 列 6：加速度计 1 Y 轴
    (int)accel1.z       // 列 7：加速度计 1 Z 轴
);
```

**输出示例**：
```
1698765432100,123,-456,789,120,-450,785
1698765432102,125,-458,791,122,-452,787
1698765432104,127,-460,793,124,-454,789
...
```

---

### 7.2 数据单位

| 列 | 字段 | 单位 | 数据类型 | 说明 |
|----|------|------|---------|------|
| 1 | `timestamp` | 微秒 (μs) | `int64_t` | 系统启动后的时间 |
| 2-4 | `accel0.x/y/z` | 原始值 | `int` | 需要乘以 scale 转换为 m/s² |
| 5-7 | `accel1.x/y/z` | 原始值 | `int` | 需要乘以 scale 转换为 m/s² |

**单位转换**：
```matlab
% 读取 CSV 文件
data = csvread('imu_data.csv');

% 提取时间戳（转换为秒）
time_s = data(:, 1) / 1e6;

% 提取加速度数据（假设 scale = 0.001）
accel0_x = data(:, 2) * 0.001;  % m/s²
accel0_y = data(:, 3) * 0.001;
accel0_z = data(:, 4) * 0.001;

% 绘图
plot(time_s, accel0_x);
xlabel('Time (s)');
ylabel('Acceleration (m/s²)');
```

---

### 7.3 为什么只输出加速度计？

**原因分析**：
1. **示例代码简化**：展示基本用法，避免输出过多数据
2. **带宽限制**：921600 bps 的带宽有限，输出太多数据会丢帧
3. **易于扩展**：用户可以根据需要添加陀螺仪、磁力计等数据

**扩展示例**：
```c
// 输出加速度计 + 陀螺仪
dprintf(serial_fd,
    "%"PRId64",%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
    accel0.timestamp,
    (int)accel0.x, (int)accel0.y, (int)accel0.z,
    (int)gyro0.x, (int)gyro0.y, (int)gyro0.z,
    (int)accel1.x, (int)accel1.y, (int)accel1.z
);
```

---

## 8. 使用场景详解

### 8.1 实时数据采集

**硬件连接**：
```
飞控 UART → USB-TTL 转换器 → 电脑
      ↓
   /dev/ttyS1 (飞控侧)
      ↓
   COM3 或 /dev/ttyUSB0 (电脑侧)
```

**步骤**：
```bash
# 1. 飞控端启动
nsh> matlab_csv_serial start /dev/ttyS1

# 2. 电脑端读取（Linux）
cat /dev/ttyUSB0 > imu_data.csv

# 或使用 Python 实时绘图
python3 plot_imu_realtime.py /dev/ttyUSB0
```

---

### 8.2 MATLAB 数据分析

**MATLAB 脚本示例**：

```matlab
%% 1. 读取数据
data = csvread('imu_data.csv');

%% 2. 提取时间和加速度
time = data(:, 1) / 1e6;  % 转换为秒
accel0_x = data(:, 2);
accel0_y = data(:, 3);
accel0_z = data(:, 4);

%% 3. 时间序列绘图
figure;
subplot(3,1,1);
plot(time, accel0_x);
ylabel('Accel X');
grid on;

subplot(3,1,2);
plot(time, accel0_y);
ylabel('Accel Y');
grid on;

subplot(3,1,3);
plot(time, accel0_z);
ylabel('Accel Z');
xlabel('Time (s)');
grid on;

%% 4. FFT 频谱分析
Fs = 1 / mean(diff(time));  % 采样率
N = length(accel0_x);
f = (0:N-1) * (Fs/N);       % 频率轴

Y = fft(accel0_x);
P = abs(Y/N);

figure;
plot(f(1:N/2), P(1:N/2));
xlabel('Frequency (Hz)');
ylabel('Amplitude');
title('Accel X Frequency Spectrum');
grid on;

%% 5. 噪声分析
accel_mag = sqrt(accel0_x.^2 + accel0_y.^2 + accel0_z.^2);
noise_std = std(accel_mag);
fprintf('Noise Standard Deviation: %.4f\n', noise_std);
```

---

### 8.3 Python 实时绘图

```python
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# 打开串口
ser = serial.Serial('/dev/ttyUSB0', 921600)

# 数据缓冲区
max_points = 500
time_data = np.zeros(max_points)
accel_x = np.zeros(max_points)
accel_y = np.zeros(max_points)
accel_z = np.zeros(max_points)

# 初始化绘图
fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

def update(frame):
    try:
        line = ser.readline().decode('utf-8').strip()
        values = [float(x) for x in line.split(',')]

        # 滚动缓冲区
        time_data[:-1] = time_data[1:]
        accel_x[:-1] = accel_x[1:]
        accel_y[:-1] = accel_y[1:]
        accel_z[:-1] = accel_z[1:]

        # 添加新数据
        time_data[-1] = values[0] / 1e6
        accel_x[-1] = values[1]
        accel_y[-1] = values[2]
        accel_z[-1] = values[3]

        # 更新绘图
        ax1.clear()
        ax1.plot(time_data, accel_x)
        ax1.set_ylabel('Accel X')
        ax1.grid()

        ax2.clear()
        ax2.plot(time_data, accel_y)
        ax2.set_ylabel('Accel Y')
        ax2.grid()

        ax3.clear()
        ax3.plot(time_data, accel_z)
        ax3.set_ylabel('Accel Z')
        ax3.set_xlabel('Time (s)')
        ax3.grid()

    except Exception as e:
        print(f"Error: {e}")

ani = FuncAnimation(fig, update, interval=50)
plt.show()
```

---

## 9. 性能分析

### 9.1 数据带宽计算

**单行数据大小**：
```
1698765432100,123,-456,789,120,-450,785\n
↑             ↑                        ↑
约 42 字节（包括逗号和换行符）
```

**传输速率**：
- 假设采样率：1000 Hz
- 每秒数据量：42 字节 × 1000 = 42 KB/s = 336 Kbps
- 串口波特率：921600 bps ≈ 115.2 KB/s

**结论**：
- ✅ 波特率足够（115.2 > 42）
- ✅ 有约 64% 的余量

**如果输出所有传感器（accel + gyro）**：
- 每行约 70 字节
- 1000 Hz → 70 KB/s = 560 Kbps
- 仍然在带宽内 ✅

---

### 9.2 CPU 开销

| 操作 | 开销 | 说明 |
|------|------|------|
| `poll()` 等待 | 极低 | 阻塞等待，不占用 CPU |
| `orb_copy()` | 低 | 内存拷贝（约 100 字节） |
| `dprintf()` | 中等 | 格式化字符串 + 串口 DMA |
| **总体影响** | **低** | 约占用 1-2% CPU |

---

## 10. 常见问题与解决方案

### 10.1 无数据输出

**现象**：串口打开正常，但没有数据输出

**排查步骤**：

```bash
# 1. 检查模块是否运行
nsh> matlab_csv_serial status

# 2. 检查传感器是否发布数据
nsh> listener sensor_accel

# 3. 检查串口是否被占用
nsh> ls /dev/tty*

# 4. 尝试回环测试
nsh> echo "test" > /dev/ttyS1
```

---

### 10.2 数据乱码

**原因**：
- ❌ 波特率不匹配
- ❌ 数据位/停止位/校验位配置错误
- ❌ 硬件连接问题（TX/RX 接反）

**解决方案**：
```bash
# 确保电脑端波特率与飞控一致
stty -F /dev/ttyUSB0 921600 cs8 -cstopb -parenb
```

---

### 10.3 数据丢失

**现象**：CSV 文件中有缺失的行

**原因**：
- ⚠️ 串口缓冲区溢出
- ⚠️ 电脑端读取速度慢

**解决方案**：
1. 降低采样率（修改代码，跳过部分数据）
2. 增大串口缓冲区
3. 使用 DMA 模式

---

### 10.4 时间戳不连续

**现象**：时间戳突然跳变

**原因**：
- 传感器发布频率不稳定
- 系统负载过高，丢失部分数据

**解决方案**：
```matlab
% 检测时间戳跳变
dt = diff(time);
jumps = find(dt > 2 * median(dt));
fprintf('Found %d time jumps\n', length(jumps));
```

---

## 11. 代码改进建议

### 11.1 添加陀螺仪数据

```c
// 修改输出格式
dprintf(serial_fd,
    "%"PRId64",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
    accel0.timestamp,
    (int)accel0.x, (int)accel0.y, (int)accel0.z,  // 加速度计 0
    (int)accel1.x, (int)accel1.y, (int)accel1.z,  // 加速度计 1
    (int)gyro0.x, (int)gyro0.y, (int)gyro0.z,     // 陀螺仪 0
    (int)gyro1.x, (int)gyro1.y, (int)gyro1.z      // 陀螺仪 1
);
```

---

### 11.2 添加采样率控制

```c
// 定义采样间隔（每 N 次更新输出一次）
int output_interval = 10;  // 1000 Hz → 100 Hz
int counter = 0;

while (!thread_should_exit) {
    // ... poll and orb_copy ...

    counter++;
    if (counter >= output_interval) {
        dprintf(serial_fd, ...);  // 输出数据
        counter = 0;
    }
}
```

---

### 11.3 添加二进制模式

CSV 格式占用带宽大，可以改为二进制：

```c
struct __attribute__((packed)) imu_data_binary {
    uint64_t timestamp;
    int16_t accel0[3];
    int16_t accel1[3];
    int16_t gyro0[3];
    int16_t gyro1[3];
};

struct imu_data_binary data;
data.timestamp = accel0.timestamp;
memcpy(data.accel0, &accel0.x, 6);
// ...

write(serial_fd, &data, sizeof(data));
```

**优势**：
- ✅ 带宽节省 60% 以上
- ✅ 更高采样率
- ❌ 需要专门的解析工具

---

### 11.4 添加缓冲区

使用缓冲区批量写入，减少系统调用：

```c
char buffer[1024];
int buf_len = 0;

// 累积数据
buf_len += snprintf(buffer + buf_len, sizeof(buffer) - buf_len,
    "%"PRId64",%d,%d,%d\n", ...);

// 每 10 行或缓冲区满时写入
if (buf_len > 900 || counter % 10 == 0) {
    write(serial_fd, buffer, buf_len);
    buf_len = 0;
}
```

---

## 12. 与现代模块开发的对比

### 12.1 传统方式（本例）

```c
// 全局变量
static bool thread_running = false;
static int daemon_task;

// 手动线程管理
daemon_task = px4_task_spawn_cmd(...);

// 手动退出控制
while (!thread_should_exit) { ... }
```

**优点**：
- ✅ 简单直观
- ✅ 易于理解

**缺点**：
- ❌ 全局变量（不安全）
- ❌ 无自动清理
- ❌ 代码重复

---

### 12.2 现代方式（ModuleBase）

```cpp
class DataExporter : public ModuleBase<DataExporter>,
                     public px4::ScheduledWorkItem
{
    void Run() override {
        // 数据导出逻辑
    }
};
```

**优点**：
- ✅ 面向对象
- ✅ 自动生命周期管理
- ✅ 统一的命令接口

---

## 13. 总结

### 13.1 核心要点

| 要点 | 说明 |
|------|------|
| **主要用途** | 将 IMU 数据实时导出到串口（CSV 格式） |
| **目标用户** | 算法开发者、硬件测试工程师 |
| **输出格式** | CSV（逗号分隔值） |
| **波特率** | 921600 bps |
| **数据源** | `sensor_accel` 和 `sensor_gyro` |
| **触发机制** | Poll 机制（数据驱动） |
| **编程模式** | 传统 C 语言 + 线程 |

---

### 13.2 学习价值

通过学习 `matlab_csv_serial`，你可以掌握：

1. ✅ **传统模块开发模式**：start/stop/status 命令实现
2. ✅ **串口编程**：POSIX termios 接口使用
3. ✅ **uORB 多实例订阅**：订阅多个传感器
4. ✅ **Poll 机制**：高效的事件驱动编程
5. ✅ **数据格式设计**：CSV 导出与 MATLAB 集成
6. ✅ **实时数据流**：从飞控到 PC 的数据管道

---

### 13.3 实际应用

| 应用场景 | 说明 |
|---------|------|
| **传感器测试** | 分析传感器噪声、漂移、温度特性 |
| **算法验证** | 在 MATLAB 中测试新算法，验证效果 |
| **故障诊断** | 飞行异常时，导出数据分析根因 |
| **教学演示** | 实时展示传感器数据，教学用 |
| **数据记录** | 记录测试数据，建立数据库 |

---

### 13.4 相关文档索引

| 文档 | 说明 |
|------|------|
| `01-fake_imu传感器模拟器代码详解.md` | 可与本模块配合，导出模拟数据 |
| `02-BMI270数据结构与信号链路分析.md` | 了解真实传感器数据来源 |
| `16-PX4_uORB消息系统架构与通信机制.md` | uORB 订阅机制详解 |
| `32-PX4_AppState应用状态管理机制详解.md` | 现代模块生命周期管理 |

---

**文档版本**：v1.0
**创建日期**：2025-10-30
**适用 PX4 版本**：v1.14+
**作者**：基于 matlab_csv_serial 源码分析整理

