# matlab_csv_serial - Fake IMU 数据采集与分析工具

## 概述

本模块用于从 PX4 的 `fake_imu` 模块采集传感器数据，通过串口以 CSV 格式输出，然后在 MATLAB 中进行分析和可视化。

`fake_imu` 生成线性调频（Chirp）信号，可用于：
- 测试传感器数据采集链路
- 验证串口通信
- 频率响应分析
- FFT 和时频分析实验

---

## 系统架构

```
┌─────────────┐         ┌──────────────────┐         ┌─────────────┐
│  fake_imu   │  uORB   │ matlab_csv_serial│ 串口TX  │   PC/笔记本  │
│  (工作队列)  ├────────>│   (独立任务)      ├────────>│  串口接收工具 │
│             │         │                  │         │             │
│ 生成Chirp   │         │ 订阅sensor_accel │         │ 保存为TXT/CSV│
│ 正弦波信号   │         │ 订阅sensor_gyro  │         │             │
└─────────────┘         └──────────────────┘         └──────┬──────┘
                                                             │
                                                             v
                                                      ┌─────────────┐
                                                      │   MATLAB    │
                                                      │ 读取并绘图   │
                                                      └─────────────┘
```

---

## 快速开始

### 1. 硬件连接

#### 方案 A：使用 USB 转 TTL 串口（推荐）
```
飞控板            USB转TTL模块          电脑
UART_TX  ───>     RX              USB ──> PC
UART_RX  <───     TX              (接收数据)
GND      ───>     GND
```

#### 方案 B：使用飞控的 USB 口（如果支持）
```
飞控 USB 口 ──> 电脑 USB 口
```

### 2. 确定串口设备

在飞控板的配置文件中查看可用串口：

```bash
# 查看 boards/micoair/h743/default.px4board
CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS0"   # TELEM1
CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS1"   # TELEM2
CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS2"   # GPS
...
```

**常用串口：**
- `/dev/ttyS0` - TELEM1 口
- `/dev/ttyS1` - TELEM2 口
- `/dev/ttyACM0` - USB 虚拟串口

### 3. PX4 端操作

#### 3.1 连接到 NSH Shell
```bash
# 通过串口或 USB 连接
# 使用工具如：screen, minicom, QGroundControl MAVLink Console 等
```

#### 3.2 启动 fake_imu
```bash
nsh> fake_imu start
```

**验证运行状态：**
```bash
nsh> fake_imu status
# 应该显示: running
```

**查看参数（可选）：**
```bash
nsh> param show FAKE_IMU*
# FAKE_IMU_X_F0    0.000000    # X轴起始频率 (Hz)
# FAKE_IMU_X_F1   10.000000    # X轴结束频率 (Hz)
# FAKE_IMU_Y_F0    0.000000    # Y轴起始频率 (Hz)
# FAKE_IMU_Y_F1  100.000000    # Y轴结束频率 (Hz)
# FAKE_IMU_Z_F0    0.000000    # Z轴起始频率 (Hz)
# FAKE_IMU_Z_F1 1000.000000    # Z轴结束频率 (Hz)
# FAKE_IMU_PERIOD 10.000000    # 扫频周期 (秒)
```

**修改参数（如需要）：**
```bash
nsh> param set FAKE_IMU_Z_F1 500    # 修改Z轴最大频率为500Hz
nsh> param save                      # 保存参数（重启后生效）
```

#### 3.3 启动 matlab_csv_serial
```bash
# 使用 TELEM1 口 (/dev/ttyS0)
nsh> matlab_csv_serial start /dev/ttyS0

# 或使用其他串口
nsh> matlab_csv_serial start /dev/ttyS1

# 查看状态
nsh> matlab_csv_serial status
# 应该显示: running
```

**输出示例：**
```
INFO  [matlab_csv_serial] opening port /dev/ttyS0
INFO  [matlab_csv_serial] Serial port configured successfully
INFO  [matlab_csv_serial] Subscribing to fake_imu sensor data (device ID 1310988)...
INFO  [matlab_csv_serial] Started! Writing CSV data to serial port...
INFO  [matlab_csv_serial] CSV format: timestamp_us,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
INFO  [matlab_csv_serial] Samples: accel=1000, gyro=1000
...
```

#### 3.4 停止采集
```bash
nsh> matlab_csv_serial stop
nsh> fake_imu stop
```

### 4. PC 端数据采集

#### 方案 A：使用 Python (推荐)

创建 `capture_serial.py`：

```python
#!/usr/bin/env python3
import serial
import time
import sys

# 配置参数
PORT = '/dev/ttyUSB0'      # Linux: /dev/ttyUSB0, macOS: /dev/cu.usbserial-*, Windows: COM3
BAUDRATE = 921600
OUTPUT_FILE = 'imu_data.csv'

def main():
    print(f"正在打开串口: {PORT} @ {BAUDRATE} bps")

    try:
        # 打开串口
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"串口已打开")

        # 打开输出文件
        with open(OUTPUT_FILE, 'w') as f:
            print(f"正在保存数据到: {OUTPUT_FILE}")
            print("按 Ctrl+C 停止采集...")

            line_count = 0
            start_time = time.time()

            while True:
                try:
                    # 读取一行
                    line = ser.readline().decode('utf-8', errors='ignore').strip()

                    if line:
                        # 写入文件
                        f.write(line + '\n')
                        f.flush()  # 立即写入磁盘

                        line_count += 1

                        # 每1000行打印一次状态
                        if line_count % 1000 == 0:
                            elapsed = time.time() - start_time
                            rate = line_count / elapsed if elapsed > 0 else 0
                            print(f"已采集 {line_count} 行 ({rate:.1f} 行/秒)")

                except KeyboardInterrupt:
                    print("\n用户中断，正在保存...")
                    break

        print(f"\n采集完成！")
        print(f"总共采集 {line_count} 行数据")
        print(f"保存到: {OUTPUT_FILE}")

    except serial.SerialException as e:
        print(f"串口错误: {e}")
        sys.exit(1)
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已关闭")

if __name__ == '__main__':
    main()
```

运行：
```bash
# 安装依赖
pip3 install pyserial

# 执行采集
chmod +x capture_serial.py
./capture_serial.py
```

#### 方案 B：使用 minicom (Linux)

```bash
# 捕获到文件
minicom -D /dev/ttyUSB0 -b 921600 -C imu_data.txt

# 按 Ctrl+A 然后 Q 退出
```

#### 方案 C：使用 screen (Linux/macOS)

```bash
# 开始记录
screen -L -Logfile imu_data.txt /dev/ttyUSB0 921600

# 按 Ctrl+A 然后 K 停止
```

#### 方案 D：使用串口助手软件 (Windows)

推荐软件：
- **SSCOM** - 中文界面，简单易用
- **Tera Term** - 功能强大
- **Putty** - 支持日志记录

配置：
- 波特率：921600
- 数据位：8
- 停止位：1
- 校验：无
- 流控：无

启用"保存到文件"功能，开始接收数据。

### 5. 数据格式

采集到的数据格式为 CSV：

```csv
# timestamp_us,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
1234567890123,0.123456,-0.234567,9.805432,0.001234,0.002345,-0.000123
1234567891373,0.125678,-0.236789,9.803210,0.001456,0.002567,-0.000234
...
```

**字段说明：**
- `timestamp_us` - 时间戳（微秒）
- `accel_x/y/z` - 加速度（m/s²）
- `gyro_x/y/z` - 角速度（rad/s）

### 6. MATLAB 分析

#### 6.1 TXT 转 CSV（如果需要）

如果你的数据保存为 `.txt` 文件，可以直接重命名：

```bash
# Linux/macOS
mv imu_data.txt imu_data.csv

# Windows
ren imu_data.txt imu_data.csv
```

或者在 MATLAB 中使用提供的转换函数：

```matlab
% 在 MATLAB 命令窗口
convert_txt_to_csv('imu_data.txt', 'imu_data.csv');
```

#### 6.2 运行 MATLAB 脚本

```matlab
% 1. 切换到脚本所在目录
cd('/home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot/src/examples/matlab_csv_serial')

% 2. 运行绘图脚本
plot_fake_imu_data('imu_data.csv');

% 或者使用默认文件名
plot_fake_imu_data();  % 会寻找 imu_data.csv
```

#### 6.3 查看结果

脚本会生成一个包含6个子图的窗口：

1. **加速度计时域** - 显示 X/Y/Z 三轴随时间变化
2. **陀螺仪时域** - 显示 X/Y/Z 三轴随时间变化
3. **加速度计频谱** - X轴的FFT频谱分析
4. **陀螺仪频谱** - Z轴的FFT频谱分析
5. **加速度计时频分析** - X轴的Spectrogram（可看到频率扫描）
6. **陀螺仪时频分析** - Z轴的Spectrogram（可看到频率扫描）

**你应该能看到：**
- 时域图中的正弦波形
- 频谱图中的单一峰值（如果采集时间较短）或宽带谱（如果完整扫频）
- 时频图中清晰的斜线（频率随时间线性增加）

---

## 高级配置

### 调整 fake_imu 参数

fake_imu 使用线性调频（Chirp）信号，参数控制每个轴的扫频范围：

```bash
# X轴：0 -> 10 Hz (慢速扫频)
param set FAKE_IMU_X_F0 0
param set FAKE_IMU_X_F1 10

# Y轴：0 -> 100 Hz (中速扫频)
param set FAKE_IMU_Y_F0 0
param set FAKE_IMU_Y_F1 100

# Z轴：0 -> 1000 Hz (快速扫频)
param set FAKE_IMU_Z_F0 0
param set FAKE_IMU_Z_F1 1000

# 扫频周期：10秒完成一次完整扫频
param set FAKE_IMU_PERIOD 10

# 保存参数（可选，重启后也生效）
param save
```

**建议配置：**

| 应用场景 | X_F1 | Y_F1 | Z_F1 | PERIOD | 说明 |
|---------|------|------|------|--------|------|
| 低频测试 | 5 | 10 | 20 | 10 | 适合慢速采集 |
| 中频测试 | 10 | 50 | 200 | 10 | 平衡测试 |
| 高频测试 | 50 | 500 | 2000 | 20 | 测试高频响应 |
| 快速验证 | 10 | 50 | 100 | 5 | 快速看到结果 |

### 修改串口波特率

如果需要更改波特率（不推荐，921600 是最优的）：

在 `matlab_csv_serial.c` 中修改：

```c
unsigned speed = 921600;  // 改为 115200, 460800 等
```

然后重新编译。

### 输出更多数据

当前输出 7 列数据。如果想输出更多信息（如设备ID、温度等），可以修改 `matlab_csv_serial.c` 中的输出格式：

```c
dprintf(serial_fd, "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%.2f\n",
    (unsigned long long)timestamp,
    (double)accel.x, (double)accel.y, (double)accel.z,
    (double)gyro.x, (double)gyro.y, (double)gyro.z,
    accel.device_id,
    (double)accel.temperature);
```

相应地，MATLAB 脚本也需要更新变量名。

---

## 故障排查

### 问题 1：fake_imu 启动失败

**症状：**
```
ERROR [fake_imu] alloc failed
```

**解决：**
- 检查内存是否足够
- 尝试停止其他不必要的模块

### 问题 2：matlab_csv_serial 无法打开串口

**症状：**
```
ERROR [matlab_csv_serial] failed to open port: /dev/ttyS0
```

**解决：**
1. 确认串口设备存在：
   ```bash
   nsh> ls /dev/tty*
   ```

2. 检查串口是否被其他程序占用：
   ```bash
   nsh> dmesg | grep tty
   ```

3. 尝试其他串口：
   ```bash
   nsh> matlab_csv_serial start /dev/ttyS1
   ```

### 问题 3：PC 端收不到数据

**排查步骤：**

1. **检查硬件连接**
   - 确认 TX/RX 接线正确
   - 确认 GND 已连接

2. **检查 PC 端串口**
   ```bash
   # Linux
   ls -l /dev/ttyUSB*

   # 给予权限
   sudo chmod 666 /dev/ttyUSB0
   # 或加入 dialout 组
   sudo usermod -a -G dialout $USER
   ```

3. **验证串口通信**
   ```bash
   # 使用 minicom 查看原始数据
   minicom -D /dev/ttyUSB0 -b 921600
   ```

   应该能看到滚动的数字输出。

4. **检查波特率**
   - 确保 PC 端工具设置为 921600
   - 数据位 8，停止位 1，无校验

### 问题 4：MATLAB 读取数据失败

**症状：**
```
Error: 文件不存在: imu_data.csv
```

**解决：**
1. 确认文件路径正确
2. 使用绝对路径：
   ```matlab
   plot_fake_imu_data('/full/path/to/imu_data.csv');
   ```

**症状：**
```
Error: 没有读取到数据！
```

**解决：**
1. 检查文件是否为空
2. 确认文件格式正确（CSV，逗号分隔）
3. 检查是否有注释行（以 `#` 开头）

### 问题 5：频谱图显示异常

**可能原因：**
- 采样时间太短（建议至少采集 10 秒）
- 采样率不够（fake_imu 默认约 8kHz，应该足够）
- 数据中有异常值

**解决：**
- 采集更长时间的数据
- 检查数据是否连续（时间戳应该单调增加）

---

## 文件说明

```
matlab_csv_serial/
├── CMakeLists.txt              # 构建配置
├── Kconfig                      # 配置选项
├── matlab_csv_serial.c          # 主程序（已修改为订阅fake_imu）
├── plot_fake_imu_data.m         # MATLAB 绘图脚本
└── README.md                    # 本文档
```

---

## 实际应用案例

### 案例 1：验证陀螺仪动态范围

**目标：** 测试飞控陀螺仪能否正确处理 0-1000 Hz 的信号

**步骤：**
1. 设置 fake_imu 参数：
   ```bash
   param set FAKE_IMU_Z_F0 0
   param set FAKE_IMU_Z_F1 1000
   param set FAKE_IMU_PERIOD 20
   ```

2. 采集 20 秒数据

3. 在 MATLAB 中查看陀螺仪 Z轴 的时频图

**预期结果：** 应该看到从 0 到 1000 Hz 的清晰斜线

### 案例 2：测试串口通信可靠性

**目标：** 验证串口在高速率下的数据完整性

**步骤：**
1. 采集 60 秒数据
2. 检查时间戳是否连续
3. 检查是否有数据丢失

**MATLAB 验证代码：**
```matlab
% 读取数据
data = readtable('imu_data.csv');

% 检查时间戳间隔
dt = diff(data.timestamp);
dt_us = dt;  % 微秒
dt_mean = mean(dt_us);
dt_std = std(dt_us);

fprintf('平均时间间隔: %.2f us (%.1f Hz)\n', dt_mean, 1e6/dt_mean);
fprintf('标准差: %.2f us\n', dt_std);

% 检测丢失数据
expected_interval = median(dt_us);
gaps = find(dt_us > expected_interval * 2);

if isempty(gaps)
    fprintf('✓ 数据完整，无丢失\n');
else
    fprintf('✗ 检测到 %d 处数据间隙\n', length(gaps));
    fprintf('  位置: %s\n', mat2str(gaps'));
end
```

### 案例 3：频率响应分析

**目标：** 分析整个数据链路的频率响应特性

**步骤：**
1. 使用宽频扫描（0-2000 Hz）
2. 采集完整周期的数据
3. 使用 FFT 分析每个频段的幅值

**进阶分析：** 可以计算传递函数，识别滤波器特性

---

## 参考资料

### PX4 相关
- [PX4 用户指南](https://docs.px4.io/)
- [uORB 消息](https://docs.px4.io/main/en/middleware/uorb.html)
- [模块开发](https://docs.px4.io/main/en/modules/modules_main.html)

### MATLAB 相关
- [readtable 文档](https://www.mathworks.com/help/matlab/ref/readtable.html)
- [FFT 文档](https://www.mathworks.com/help/matlab/ref/fft.html)
- [spectrogram 文档](https://www.mathworks.com/help/signal/ref/spectrogram.html)

### 信号处理
- [Chirp 信号 - 维基百科](https://en.wikipedia.org/wiki/Chirp)
- [时频分析](https://en.wikipedia.org/wiki/Time%E2%80%93frequency_analysis)

---

## 常见问题 (FAQ)

**Q1: 为什么选择 921600 波特率？**

A: 这是常见串口支持的最高标准波特率。fake_imu 以约 8kHz 输出数据，每行约 60 字节，需要 480 kbps，921600 bps 提供足够的带宽余量。

**Q2: 可以同时运行多个 matlab_csv_serial 实例吗？**

A: 可以，但需要使用不同的串口。每个串口只能被一个实例使用。

**Q3: 数据量会很大吗？**

A: 是的。以 8kHz 采样率，每行 60 字节：
- 每秒：约 480 KB
- 每分钟：约 28 MB
- 每小时：约 1.7 GB

建议采集时间不要太长（1-5 分钟足够分析）。

**Q4: 为什么时频图没有显示清晰的斜线？**

A: 可能原因：
1. 采集时间太短，没有完整的扫频周期
2. 窗口大小不合适
3. 数据有丢失或时间戳不连续

建议采集至少一个完整周期（FAKE_IMU_PERIOD 参数设定的时间）。

**Q5: 可以实时绘图吗？**

A: 当前脚本是离线分析。如需实时绘图，可以：
1. 使用 Python + matplotlib 读取串口实时绘制
2. 使用 MATLAB 的 serialport 对象实时读取
3. 参考 `src/examples/px4_mavlink_debug` 使用 MAVLink 传输

---

## 技术支持

如有问题，请：
1. 检查本文档的"故障排查"章节
2. 查看 PX4 日志：`dmesg`
3. 在 PX4 论坛提问：https://discuss.px4.io/

---

## 更新历史

- **2025-10-30** - 初始版本，修改为订阅 fake_imu 数据
- 基于原始 matlab_csv_serial 模块改进

---

## 许可证

与 PX4 项目相同，采用 BSD 3-Clause License。

