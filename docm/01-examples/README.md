# 01-examples - PX4 示例代码分析文档集

## 📁 文件夹说明

本文件夹包含 PX4 Examples 示例代码的详细分析文档，旨在帮助开发者深入理解 PX4 示例模块的实现原理和使用方法。

---

## 📚 文档列表

| 序号 | 文档名称 | 模块路径 | 主要内容 |
|------|---------|---------|---------|
| 01 | [fake_imu传感器模拟器代码详解](./01-fake_imu传感器模拟器代码详解.md) | `src/examples/fake_imu/` | 虚拟 IMU 传感器、扫频信号生成、动态陷波滤波器测试 |
| 02 | [matlab_csv_serial数据导出工具详解](./02-matlab_csv_serial数据导出工具详解.md) | `src/examples/matlab_csv_serial/` | 传感器数据实时导出、CSV 格式、MATLAB 集成 |
| 03 | [ModuleBase框架与传统模块开发对比详解](./03-ModuleBase框架与传统模块开发对比详解.md) | 框架对比 | CRTP 模式、命令处理机制、虚函数 vs 模板 |
| 04 | [PX4Gyroscope参数系统与采样率配置详解](./04-PX4Gyroscope参数系统与采样率配置详解.md) | 参数系统 | IMU_GYRO_RATEMAX、参数读取、采样率配置 |
| 05 | [fake_imu参数配置使用指南](./05-fake_imu参数配置使用指南.md) | `src/examples/fake_imu/` | 扫频参数配置、实际应用案例、参数调优 |
| 06 | [fake_imu数据记录与频谱分析完整流程](./06-fake_imu数据记录与频谱分析完整流程.md) | 实战指南 | 日志记录、数据下载、FFT分析、MATLAB/Python示例 |

---

## 🎯 学习路线建议

### 初学者路线

```
1. matlab_csv_serial (简单，传统 C 语言)
   ↓
2. ModuleBase框架对比文档 (理解框架设计)
   ↓
3. fake_imu (中等，ModuleBase 框架)
```

**原因**：
- `matlab_csv_serial` 使用传统 C 语言和简单的线程模型，易于理解
- 先阅读框架对比文档，理解 CRTP 模式和命令处理机制
- `fake_imu` 使用现代的 ModuleBase 框架，是学习标准模块开发的好例子

---

### 进阶开发者路线

```
1. fake_imu
   ↓
2. 结合 gyro_fft 模块源码
   ↓
3. 开发自己的测试工具
```

---

## 🔗 模块关联关系

```
┌─────────────────┐
│   fake_imu      │  生成扫频信号
│  (虚拟传感器)    │
└────────┬────────┘
         │ 发布 sensor_gyro_fifo
         ▼
┌─────────────────┐
│   gyro_fft      │  FFT 频谱分析
│  (频谱分析)      │
└────────┬────────┘
         │ 检测频率
         ▼
┌─────────────────┐
│ Dynamic Notch   │  动态陷波滤波
│  (振动抑制)      │
└─────────────────┘
         │
         ▼
┌─────────────────┐
│matlab_csv_serial│  导出数据
│  (数据导出)      │  ↓
└─────────────────┘  MATLAB 分析验证
```

---

## 📖 文档特点

### 1. 详细的代码解析

每个文档都包含：
- ✅ 完整的代码流程分析
- ✅ 关键函数逐行注释
- ✅ 数据结构详解
- ✅ 算法原理说明

### 2. 丰富的示例代码

- MATLAB 数据分析脚本
- Python 实时绘图示例
- Shell 命令使用方法
- 代码改进建议

### 3. 实战应用场景

- 测试工具开发
- 算法验证流程
- 故障诊断方法
- 性能分析技巧

---

## 🛠️ 快速上手

### fake_imu 使用

```bash
# 1. 启动虚拟 IMU
nsh> fake_imu start

# 2. 查看数据
nsh> listener sensor_gyro_fifo

# 3. 启动 FFT 分析（如果已编译）
nsh> gyro_fft start

# 4. 停止
nsh> fake_imu stop
```

### matlab_csv_serial 使用

```bash
# 1. 启动数据导出（指定串口）
nsh> matlab_csv_serial start /dev/ttyS1

# 2. 电脑端接收数据（Linux）
cat /dev/ttyUSB0 > imu_data.csv

# 3. MATLAB 分析
data = csvread('imu_data.csv');
plot(data(:,1)/1e6, data(:,2));

# 4. 停止导出
nsh> matlab_csv_serial stop
```

---

## 🔍 深度主题索引

### 编程模式对比

| 主题 | `matlab_csv_serial` | `fake_imu` |
|------|-------------------|-----------|
| 编程语言 | C 语言 | C++ 语言 |
| 模块框架 | 传统线程 | ModuleBase |
| 任务调度 | 独立线程 + poll | ScheduledWorkItem |
| 生命周期 | 手动标志位 | should_exit() |
| 数据发布 | 串口输出 | uORB 发布 |

### 适用场景对比

| 场景 | `fake_imu` | `matlab_csv_serial` |
|------|-----------|-------------------|
| 算法测试 | ✅ 理想 | ⚠️ 需要真实传感器 |
| 硬件测试 | ❌ 不适用 | ✅ 理想 |
| 教学演示 | ✅ 适合 | ✅ 适合 |
| 数据分析 | ⚠️ 需配合日志 | ✅ 实时导出 |
| 振动分析 | ✅ 扫频测试 | ✅ 实际数据 |

---

## 📝 相关文档参考

### 核心框架文档

- `10-PX4工作队列架构与启动机制.md` - 工作队列原理
- `16-PX4_uORB消息系统架构与通信机制.md` - uORB 通信机制
- `32-PX4_AppState应用状态管理机制详解.md` - 模块生命周期管理

### 传感器相关文档

- `01-BMI270传感器寄存器配置详解.md` - 真实 IMU 驱动
- `02-BMI270数据结构与信号链路分析.md` - 传感器数据流
- `05-飞行器角速度计算Run函数源码分析.md` - IMU 数据处理

### 算法相关文档

- `06-动态陷波滤波器配置与实现原理.md` - fake_imu 的测试目标
- `07-FFT动态陷波带宽计算算法详解.md` - 频谱分析算法
- `18-EKF2纯IMU模式数据流与滤波策略分析.md` - IMU 数据应用

---

## 🚀 扩展学习

### 其他 PX4 示例模块

```
src/examples/
├── dyn_hello/          # 动态加载模块示例
├── fake_gps/           # 虚拟 GPS 传感器
├── fake_magnetometer/  # 虚拟磁力计
├── hello/              # 基础示例（start/stop/status）
├── px4_mavlink_debug/  # MAVLink 调试
├── px4_simple_app/     # 简单应用示例
└── work_item/          # 工作队列示例
```

**推荐下一步学习**：
1. `hello/` - 学习基本的 AppState 使用
2. `work_item/` - 学习工作队列编程
3. `px4_simple_app/` - 综合示例

---

## 💡 开发建议

### 编写新模块时的选择

**使用传统模式（类似 matlab_csv_serial）**：
- ✅ 简单的工具脚本
- ✅ 与外部系统交互（串口、网络）
- ✅ 短期使用的测试代码

**使用 ModuleBase 框架（类似 fake_imu）**：
- ✅ 需要集成到 PX4 生态系统
- ✅ 长期维护的模块
- ✅ 需要参数系统、事件系统支持
- ✅ 遵循 PX4 编码规范

---

## 🔧 故障排查

### fake_imu 常见问题

**Q1: 启动后没有数据**
```bash
# 检查模块状态
nsh> fake_imu status

# 查看工作队列
nsh> work_queue status

# 查看 uORB 主题
nsh> uorb top
```

**Q2: FFT 模块无法检测到频率**
- 确认 `gyro_fft` 已启动
- 检查采样率配置
- 查看日志：`dmesg`

---

### matlab_csv_serial 常见问题

**Q1: 串口打开失败**
```bash
# 检查设备是否存在
nsh> ls /dev/tty*

# 检查权限
nsh> ls -l /dev/ttyS1
```

**Q2: 数据乱码**
- 检查波特率是否一致（921600）
- 确认 TX/RX 是否接反
- 使用示波器检查信号

---

## 📊 性能对比

| 指标 | `fake_imu` | `matlab_csv_serial` |
|------|-----------|-------------------|
| CPU 占用 | 约 2% | 约 1-2% |
| 内存占用 | 约 4KB | 约 2KB |
| 实时性 | 高（工作队列） | 中（独立线程） |
| 带宽消耗 | 无（内部发布） | 336 Kbps（串口） |

---

## 📅 更新记录

| 日期 | 版本 | 说明 |
|------|------|------|
| 2025-10-30 | v1.4 | 新增数据记录与频谱分析完整流程文档（实战指南） |
| 2025-10-30 | v1.3 | fake_imu 参数化改进 + 使用指南文档 |
| 2025-10-30 | v1.2 | 新增 PX4Gyroscope 参数系统文档（IMU_GYRO_RATEMAX 详解） |
| 2025-10-30 | v1.1 | 新增 ModuleBase 框架对比文档（CRTP 模式详解） |
| 2025-10-30 | v1.0 | 初始版本，包含 fake_imu 和 matlab_csv_serial 分析 |

---

## 👥 贡献者

- 文档整理：基于 PX4 源码分析
- PX4 版本：v1.14+
- 测试平台：MicoAir H743 / SITL

---

## 📧 反馈与建议

如有问题或建议，请通过以下方式反馈：
- 在相关文档末尾添加注释
- 创建新的分析文档
- 提交改进建议

---

**文档集版本**：v1.0
**最后更新**：2025-10-30

