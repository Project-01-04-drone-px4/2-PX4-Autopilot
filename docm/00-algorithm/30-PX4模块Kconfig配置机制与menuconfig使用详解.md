# GG_IMU_LOGGER 在 menuconfig 中的路径说明

## 📋 Kconfig 包含机制详解

### 1. 包含链路径

```
./Kconfig (根配置文件)
  │
  ├─ line 4: mainmenu "PX4 Firmware Configuration"
  │
  ├─ line 197-199: menu "drivers"
  │                  source "src/drivers/Kconfig"
  │                  endmenu
  │
  ├─ line 201-203: menu "modules"          ← 这里！
  │                  source "src/modules/Kconfig"   ← 包含modules的Kconfig
  │                  endmenu
  │
  └─ line 205-207: menu "systemcmds"
                     source "src/systemcmds/Kconfig"
                     endmenu
```

### 2. src/modules/Kconfig 的内容

```kconfig
rsource "*/Kconfig"
```

**`rsource` 的作用：**
- `rsource` = recursive source（递归包含）
- `"*/Kconfig"` = 所有子目录中的 Kconfig 文件
- 自动扫描 `src/modules/` 下的所有子目录
- 如果子目录包含 `Kconfig` 文件，就自动包含进来

**被自动包含的文件（示例）：**
```
src/modules/
  ├─ airship_att_control/Kconfig
  ├─ battery_status/Kconfig
  ├─ commander/Kconfig
  ├─ ekf2/Kconfig
  ├─ gg_imu_logger/Kconfig        ← 您的模块！
  ├─ logger/Kconfig
  ├─ mavlink/Kconfig
  └─ ... (其他所有模块)
```

### 3. src/modules/gg_imu_logger/Kconfig 的内容

```kconfig
menuconfig MODULES_GG_IMU_LOGGER
	bool "gg_imu_logger"
	default n
	---help---
		Enable support for GG IMU Logger module

		This module subscribes to vehicle_imu topics and logs the data
		to a separate ulog file on the SD card.
```

## 🗺️ 在 menuconfig 中的完整路径

```
PX4 Firmware Configuration (主菜单)
  │
  ├─ Toolchain
  ├─ Serial ports
  ├─ File paths
  ├─ drivers
  │
  ├─ modules  ←┐
  │   │         │ 按这里进入 modules 菜单
  │   ├─ airship_att_control
  │   ├─ battery_status
  │   ├─ commander
  │   ├─ ekf2
  │   ├─ gg_imu_logger  ← 您的模块在这里！
  │   ├─ logger
  │   ├─ mavlink
  │   └─ ...
  │
  ├─ systemcmds
  ├─ examples
  └─ platforms
```

## 🔍 如何在 menuconfig 中找到它

### 方法 1：直接导航（推荐）

1. 运行：
   ```bash
   make micoair_h743_default menuconfig
   ```

2. 使用方向键导航到 **`modules`** 菜单项

3. 按 **Enter** 进入 modules 菜单

4. 向下滚动找到 **`gg_imu_logger`**
   - 如果显示 `[*]` = 已启用
   - 如果显示 `[ ]` = 未启用
   - 按空格键切换启用/禁用

### 方法 2：搜索功能（最快）

1. 在 menuconfig 界面按 **`/`** 键（斜杠）进入搜索模式

2. 输入：`GG_IMU_LOGGER`

3. 按 Enter，会显示：
   ```
   Symbol: MODULES_GG_IMU_LOGGER [=y]
   Type  : bool
   Prompt: gg_imu_logger
   Location:
     -> modules
   Defined at src/modules/gg_imu_logger/Kconfig:1
   ```

4. 按数字键（如 `1`）直接跳转到该选项

## 📝 配置方式对比

### 方式 A：修改 default.px4board（永久配置）

**文件位置：** `boards/micoair/h743/default.px4board`

**添加：**
```
CONFIG_MODULES_GG_IMU_LOGGER=y
```

**优点：**
- ✅ 每次重新构建都生效
- ✅ 作为板子的默认配置
- ✅ 适合稳定功能

**缺点：**
- ⚠️ 需要手动编辑文件

---

### 方式 B：使用 menuconfig（临时配置）

**命令：**
```bash
make micoair_h743_default menuconfig
```

**优点：**
- ✅ 可视化界面
- ✅ 方便浏览所有选项
- ✅ 实时查看依赖关系

**缺点：**
- ⚠️ 清除构建目录会丢失（除非保存到 default.px4board）

---

## ✅ 验证配置是否生效

### 1. 检查 default.px4board

```bash
grep "GG_IMU" boards/micoair/h743/default.px4board
```

**预期输出：**
```
CONFIG_MODULES_GG_IMU_LOGGER=y
```

### 2. 检查构建系统

```bash
grep -r "modules__gg_imu_logger" build/micoair_h743_default/ | head -n 3
```

**预期输出：**
```
build/micoair_h743_default/CMakeFiles/TargetDirectories.txt:/home/.../modules__gg_imu_logger.dir
build/micoair_h743_default/rules.ninja:rule CXX_COMPILER__modules__gg_imu_logger
build/micoair_h743_default/rules.ninja:rule CXX_STATIC_LIBRARY_LINKER__modules__gg_imu_logger
```

### 3. 编译后验证

```bash
make micoair_h743_default
```

查看编译输出中是否有：
```
[XXX/1279] Building CXX object src/modules/gg_imu_logger/CMakeFiles/modules__gg_imu_logger.dir/gg_imu_logger.cpp.obj
[XXX/1279] Linking CXX static library src/modules/gg_imu_logger/libmodules__gg_imu_logger.a
```

## 🎯 总结

| 项目 | 说明 |
|------|------|
| **Kconfig文件位置** | `src/modules/gg_imu_logger/Kconfig` |
| **包含方式** | 通过 `src/modules/Kconfig` 的 `rsource "*/Kconfig"` 自动包含 |
| **menuconfig路径** | 主菜单 → modules → gg_imu_logger |
| **搜索关键词** | 按 `/` 然后输入 `GG_IMU_LOGGER` |
| **配置项名称** | `CONFIG_MODULES_GG_IMU_LOGGER` |
| **当前状态** | 已在 default.px4board 中设置为 `y`（启用） |

## 🚀 常见问题

### Q: 为什么我在 menuconfig 中找不到？

**A:** 可能原因：
1. ✅ 实际上已经在那里了，只是被其他选项遮挡了（使用搜索功能 `/` 查找）
2. ❌ Kconfig 文件格式错误（缩进必须使用 Tab）
3. ❌ 构建缓存没有更新（执行 `rm -rf build/micoair_h743_default`）

### Q: 如何保存 menuconfig 的更改？

**A:**
1. 在 menuconfig 中按 `S` 保存配置
2. 保存到 `build/micoair_h743_default/.config`
3. 要永久保存，需要将更改复制到 `boards/micoair/h743/default.px4board`

### Q: default.px4board 和 menuconfig 哪个优先级高？

**A:**
- 如果 `build/.config` 存在，使用它（menuconfig 修改后的配置）
- 如果不存在，使用 `default.px4board`（默认配置）
- 清除构建目录后，会重新从 `default.px4board` 读取

