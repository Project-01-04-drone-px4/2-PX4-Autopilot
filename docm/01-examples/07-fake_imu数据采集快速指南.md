# Fake IMU 数据采集与 MATLAB 分析 - 快速指南

## 一、概述

本指南说明如何使用 `fake_imu` + `matlab_csv_serial` 组合进行传感器数据采集和分析。

**整个流程：**
```
PX4飞控(fake_imu) → 串口 → PC接收保存 → MATLAB分析绘图
```

---

## 二、快速步骤

### 步骤 1: 硬件连接

```
飞控 UART 口 ────── USB转TTL模块 ────── PC USB口
  (TELEM1)            (RX/TX/GND)
```

### 步骤 2: PX4 端操作

连接到 NSH Shell，运行以下命令：

```bash
# 1. 启动 fake_imu（生成测试数据）
nsh> fake_imu start

# 2. 启动 matlab_csv_serial（输出到串口）
nsh> matlab_csv_serial start /dev/ttyS0

# 3. 验证运行状态
nsh> fake_imu status
nsh> matlab_csv_serial status
```

### 步骤 3: PC 端采集数据

**方法 A: 使用 Python 脚本**（推荐）

创建 `capture.py`:
```python
#!/usr/bin/env python3
import serial

ser = serial.Serial('/dev/ttyUSB0', 921600)  # Windows: 'COM3'
with open('imu_data.csv', 'w') as f:
    print("正在采集数据，按 Ctrl+C 停止...")
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore')
            f.write(line)
            f.flush()
    except KeyboardInterrupt:
        print("\n采集完成！")
```

运行：
```bash
pip3 install pyserial
python3 capture.py
# 采集 30-60 秒后按 Ctrl+C 停止
```

**方法 B: 使用串口助手软件**（Windows）
- 打开 SSCOM 或 Tera Term
- 设置：波特率 921600，8N1
- 启用"保存到文件"
- 开始接收

### 步骤 4: MATLAB 分析

```matlab
% 1. 切换到脚本目录
cd('你的路径/src/examples/matlab_csv_serial')

% 2. 运行分析脚本
plot_fake_imu_data('imu_data.csv');
```

**你会看到 6 个图：**
1. 加速度计时域波形（正弦波）
2. 陀螺仪时域波形（正弦波）
3. 加速度计频谱（FFT）
4. 陀螺仪频谱（FFT）
5. 加速度计时频图（清晰的频率扫描斜线）
6. 陀螺仪时频图（清晰的频率扫描斜线）

---

## 三、数据格式

CSV 文件格式：
```csv
# timestamp_us,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
1234567890123,0.123456,-0.234567,9.805432,0.001234,0.002345,-0.000123
1234567891373,0.125678,-0.236789,9.803210,0.001456,0.002567,-0.000234
...
```

---

## 四、常见问题

### Q1: fake_imu 启动失败
```bash
nsh> fake_imu start
ERROR [fake_imu] alloc failed
```
**解决：** 停止其他不必要的模块，释放内存

### Q2: matlab_csv_serial 无法打开串口
```bash
nsh> matlab_csv_serial start /dev/ttyS0
ERROR [matlab_csv_serial] failed to open port
```
**解决：**
- 尝试其他串口：`/dev/ttyS1`, `/dev/ttyS2`
- 检查串口是否被占用

### Q3: PC 端收不到数据
**检查：**
1. 硬件连接（TX/RX/GND）
2. PC 串口设备存在（Linux: `/dev/ttyUSB0`, Windows: `COM3`）
3. 串口权限（Linux: `sudo chmod 666 /dev/ttyUSB0`）
4. 波特率正确（921600）

### Q4: MATLAB 无法读取文件
**检查：**
1. 文件路径正确
2. 文件不为空
3. CSV 格式正确（逗号分隔）

---

## 五、调整参数

### 修改扫频范围

```bash
nsh> param set FAKE_IMU_Z_F0 0      # Z轴起始频率 0 Hz
nsh> param set FAKE_IMU_Z_F1 500    # Z轴结束频率 500 Hz
nsh> param set FAKE_IMU_PERIOD 10   # 扫频周期 10 秒
```

常用配置：

| 场景 | Z_F1 | PERIOD | 说明 |
|------|------|--------|------|
| 快速测试 | 100 | 5 | 5秒看到结果 |
| 标准测试 | 1000 | 10 | 平衡测试 |
| 高频测试 | 2000 | 20 | 测试高频响应 |

---

## 六、完整命令速查

### PX4 端
```bash
# 启动
fake_imu start
matlab_csv_serial start /dev/ttyS0

# 检查状态
fake_imu status
matlab_csv_serial status
listener sensor_accel
uorb top

# 停止
matlab_csv_serial stop
fake_imu stop
```

### PC 端（Linux）
```bash
# 查看串口
ls /dev/ttyUSB*

# 给予权限
sudo chmod 666 /dev/ttyUSB0

# 快速查看数据
cat /dev/ttyUSB0

# Python 采集
python3 capture.py
```

### MATLAB
```matlab
% 分析数据
plot_fake_imu_data('imu_data.csv');

% 或指定完整路径
plot_fake_imu_data('/full/path/to/imu_data.csv');
```

---

## 七、TXT 转 CSV

如果你的数据保存为 `.txt` 文件：

**方法 1: 直接重命名**
```bash
# Linux/macOS
mv imu_data.txt imu_data.csv

# Windows
ren imu_data.txt imu_data.csv
```

**方法 2: MATLAB 转换**
```matlab
convert_txt_to_csv('imu_data.txt', 'imu_data.csv');
```

---

## 八、文件位置

```
src/examples/matlab_csv_serial/
├── matlab_csv_serial.c      # 主程序（已修改）
├── plot_fake_imu_data.m     # MATLAB 分析脚本
└── README.md                # 详细文档

docm/01-examples/
├── PX4示例程序架构分析.md     # 架构深入分析
└── fake_imu数据采集快速指南.md # 本文档
```

---

## 九、进阶应用

### 长时间采集
```bash
# PX4 端正常运行
# PC 端后台采集
nohup python3 capture.py > capture.log 2>&1 &
```

### 实时监控
```bash
# 实时查看最新数据
tail -f imu_data.csv
```

### 批量分析
```matlab
% 分析多个文件
files = {'data1.csv', 'data2.csv', 'data3.csv'};
for i = 1:length(files)
    figure(i);
    plot_fake_imu_data(files{i});
end
```

---

## 十、架构理解

**关键概念：**

1. **fake_imu**：运行在工作队列中
   - 不创建独立线程
   - 精确定时（8kHz）
   - 不能阻塞

2. **matlab_csv_serial**：独立任务
   - 创建独立线程
   - 可以阻塞（poll、串口写入）
   - 不影响 fake_imu

3. **uORB**：消息总线
   - 解耦两个模块
   - 发布-订阅模式
   - 非阻塞通信

**为什么这样设计？**
- fake_imu 需要高频精确执行 → 工作队列
- matlab_csv_serial 需要等待和阻塞 I/O → 独立任务
- 两者通过 uORB 通信，互不干扰

详细分析见：`docm/01-examples/PX4示例程序架构分析.md`

---

## 十一、获取帮助

- **详细文档**：`src/examples/matlab_csv_serial/README.md`
- **架构分析**：`docm/01-examples/PX4示例程序架构分析.md`
- **PX4 论坛**：https://discuss.px4.io/
- **GitHub Issues**：https://github.com/PX4/PX4-Autopilot/issues

---

## 十二、常用排查命令

```bash
# PX4 内部检查
nsh> dmesg                    # 系统日志
nsh> ps                       # 进程列表
nsh> top                      # CPU 占用
nsh> work_queue status        # 工作队列状态
nsh> uorb top                 # uORB 主题发布频率
nsh> listener sensor_accel    # 监听加速度计数据

# PC 端检查（Linux）
ls -l /dev/ttyUSB*            # 查看串口设备
dmesg | grep tty              # 串口连接日志
lsusb                         # USB 设备列表
```

---

**祝你使用愉快！如有问题，请查阅详细文档或寻求技术支持。**

