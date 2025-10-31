# micoair_h743 编译与调试快速指南

## 🎯 快速开始

### ✅ 已完成配置

1. **launch.json 已配置** ✓
   - 位置：`.vscode/launch.json`
   - JLink Release 配置：使用 `build/micoair_h743_default/micoair_h743_default.elf`
   - JLink Debug 配置：使用 `build/micoair_h743_debug/micoair_h743_debug.elf`

2. **编译脚本已创建** ✓
   - 位置：项目根目录 `build_debug.sh`
   - 支持交互式选择编译类型

3. **详细文档已创建** ✓
   - 位置：`boards/micoair/h743/debug/编译Debug版本说明.md`

## 🚀 编译固件

### 方法1：使用快速编译脚本（推荐）

```bash
cd /home/gg/01-code/01-px4/px4_latest/2-PX4-Autopilot
./build_debug.sh
```

脚本会提示您选择编译类型：
- **选项 1**: MinSizeRel - 最小体积（默认生产版本）
- **选项 2**: RelWithDebInfo - 带调试信息（**推荐用于调试**）⭐
- **选项 3**: Debug - 完全无优化（深度调试）
- **选项 4**: Release - 完全优化（性能最佳）

### 方法2：命令行直接编译

#### 编译 Release 版本（默认，当前已编译）
```bash
make micoair_h743_default
```
**文件位置**：`build/micoair_h743_default/micoair_h743_default.elf` ✓

#### 编译 RelWithDebInfo 版本（推荐调试）
```bash
# 设置环境变量
export PX4_CMAKE_BUILD_TYPE=RelWithDebInfo

# 清理并重新编译
rm -rf build/micoair_h743_default
make micoair_h743_default
```

#### 编译 Debug 版本（完全无优化）
```bash
export PX4_CMAKE_BUILD_TYPE=Debug
rm -rf build/micoair_h743_default
make micoair_h743_default
```

## 🔧 调试固件

### 步骤1：编译固件

建议使用 **RelWithDebInfo** 版本：
```bash
./build_debug.sh  # 选择选项 2
```

### 步骤2：连接硬件

1. 连接 JLink 调试器到 micoair_h743 板子
2. 连接电源
3. 确保 JLink 驱动已安装

### 步骤3：在 VSCode 中调试

1. 打开 VSCode
2. 按 **F5** 或点击 "Run and Debug"
3. 选择调试配置：
   - **"jlink (micoair_h743) - Release"** - 使用当前编译的固件
   - **"jlink (micoair_h743) - Debug"** - 使用debug目录的固件

4. 调试功能：
   - ✅ 设置断点（点击行号左侧）
   - ✅ 单步执行（F10 - 跳过, F11 - 进入）
   - ✅ 查看变量（Variables 面板）
   - ✅ 监视表达式（Watch 面板）
   - ✅ 查看调用栈（Call Stack 面板）

## 📊 编译类型对比

| 编译类型 | 优化 | 调试信息 | ELF大小 | 适用场景 |
|---------|------|---------|---------|---------|
| **MinSizeRel** | `-Os` | 部分 | ~46MB | ✅ 生产发布 |
| **RelWithDebInfo** | `-O2` | ✅ 完整 | ~65MB | ⭐ **推荐调试** |
| **Debug** | `-O0` | ✅ 完整 | ~120MB | 深度调试 |
| **Release** | `-O3` | 无 | ~50MB | 性能测试 |

## 🔥 烧录固件

### 使用 JLink 烧录

```bash
# 方法1：使用 make upload
make micoair_h743_default upload

# 方法2：使用 JLink 脚本
cd boards/micoair/h743/debug
# 编辑 download_firmware.jlink 确保路径正确
JLinkExe -CommandFile download_firmware.jlink
```

### 使用其他烧录工具

```bash
# 生成的固件文件
build/micoair_h743_default/micoair_h743_default.px4
build/micoair_h743_default/micoair_h743_default.elf
build/micoair_h743_default/micoair_h743_default.bin
```

## 📝 当前编译状态

```bash
# 查看当前编译类型
grep CMAKE_BUILD_TYPE build/micoair_h743_default/CMakeCache.txt

# 查看ELF文件
ls -lh build/micoair_h743_default/micoair_h743_default.elf

# 检查调试符号
arm-none-eabi-readelf -S build/micoair_h743_default/micoair_h743_default.elf | grep debug
```

**当前已编译版本**：
- ✅ 类型：**MinSizeRel** (Release with minimal size)
- ✅ 文件：`build/micoair_h743_default/micoair_h743_default.elf` (46MB)
- ✅ 状态：编译成功，可以烧录和运行
- ⚠️ 调试：部分调试信息，建议重新编译为 RelWithDebInfo

## 💡 常用命令

```bash
# 清理构建
rm -rf build/micoair_h743_default

# 完整构建
make micoair_h743_default

# 快速增量构建
make micoair_h743_default -j$(nproc)

# 查看编译选项
make micoair_h743_default help

# 只编译特定模块
make modules__gg_imu_logger

# 清理所有构建
make clean

# 清理并重新配置
make distclean
```

## 🐛 调试技巧

### 1. 查看变量优化问题

如果变量显示 `<optimized out>`：
- 使用 **Debug** 版本（-O0）
- 或在代码中添加 `volatile` 关键字

### 2. 硬件断点

```cpp
// 在代码中插入断点
__asm__ volatile("bkpt 0");
```

### 3. 串口调试

```bash
# 连接到PX4串口
screen /dev/ttyACM0 115200

# 或使用minicom
minicom -D /dev/ttyACM0 -b 115200
```

### 4. 查看系统日志

在PX4 Console中：
```bash
dmesg          # 查看系统消息
top            # 查看CPU使用率
free           # 查看内存使用
uorb top       # 查看uORB消息
```

## 📚 相关文档

- [详细编译说明](./编译Debug版本说明.md)
- [JLink使用指南](./README_JLink.md)
- [使用说明](./使用说明.md)
- [PX4官方调试文档](https://docs.px4.io/main/en/debug/)

## ❓ 常见问题

### Q: 如何切换回Release版本？

```bash
unset PX4_CMAKE_BUILD_TYPE  # 清除环境变量
rm -rf build/micoair_h743_default
make micoair_h743_default
```

### Q: Debug版本太大无法烧录？

使用 **RelWithDebInfo** 代替完全的 Debug：
```bash
export PX4_CMAKE_BUILD_TYPE=RelWithDebInfo
rm -rf build/micoair_h743_default
make micoair_h743_default
```

### Q: 如何只调试某个模块？

在模块的 CMakeLists.txt 中添加：
```cmake
set_source_files_properties(your_file.cpp PROPERTIES COMPILE_FLAGS "-O0 -g3")
```

### Q: VSCode无法连接JLink？

1. 检查JLink驱动是否安装：`JLinkExe -v`
2. 检查设备连接：`JLinkExe` 然后输入 `connect`
3. 检查USB权限：`sudo chmod 666 /dev/bus/usb/XXX/XXX`

## 🎉 快速测试

编译并测试您的固件：

```bash
# 1. 编译RelWithDebInfo版本
./build_debug.sh  # 选择选项2

# 2. 烧录固件
make micoair_h743_default upload

# 3. 在VSCode中按F5开始调试
```

---

**提示**：如需更详细的说明，请查看 [编译Debug版本说明.md](./编译Debug版本说明.md)

