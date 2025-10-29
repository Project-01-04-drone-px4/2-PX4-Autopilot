#!/bin/bash
# PX4 快速编译脚本 - micoair_h743

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}PX4 编译脚本 - micoair_h743${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""

# 显示菜单
echo "请选择编译类型："
echo "  1) MinSizeRel  - 最小体积 (默认, 生产版本)"
echo "  2) RelWithDebInfo - 带调试信息的优化版本 (推荐用于调试)"
echo "  3) Debug       - 完全无优化 (深度调试, 文件很大)"
echo "  4) Release     - 完全优化 (性能最佳)"
echo ""
read -p "请输入选项 [1-4] (默认=2): " choice

# 设置构建类型
case $choice in
    1)
        BUILD_TYPE="MinSizeRel"
        echo -e "${YELLOW}选择: MinSizeRel (最小体积)${NC}"
        unset PX4_CMAKE_BUILD_TYPE
        ;;
    3)
        BUILD_TYPE="Debug"
        echo -e "${YELLOW}选择: Debug (完全无优化)${NC}"
        export PX4_CMAKE_BUILD_TYPE=Debug
        ;;
    4)
        BUILD_TYPE="Release"
        echo -e "${YELLOW}选择: Release (完全优化)${NC}"
        export PX4_CMAKE_BUILD_TYPE=Release
        ;;
    2|*)
        BUILD_TYPE="RelWithDebInfo"
        echo -e "${YELLOW}选择: RelWithDebInfo (推荐调试版本)${NC}"
        export PX4_CMAKE_BUILD_TYPE=RelWithDebInfo
        ;;
esac

echo ""
read -p "是否清理之前的构建? [y/N]: " clean_build

if [[ $clean_build =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}正在清理 build/micoair_h743_default ...${NC}"
    rm -rf build/micoair_h743_default
    echo -e "${GREEN}清理完成${NC}"
fi

echo ""
echo -e "${GREEN}开始编译 (构建类型: $BUILD_TYPE)...${NC}"
echo ""

# 记录开始时间
start_time=$(date +%s)

# 编译
make micoair_h743_default

# 检查编译结果
if [ $? -eq 0 ]; then
    end_time=$(date +%s)
    duration=$((end_time - start_time))

    echo ""
    echo -e "${GREEN}======================================${NC}"
    echo -e "${GREEN}编译成功！${NC}"
    echo -e "${GREEN}======================================${NC}"
    echo -e "编译类型: ${YELLOW}$BUILD_TYPE${NC}"
    echo -e "耗时: ${YELLOW}${duration}秒${NC}"
    echo ""

    # 显示文件信息
    ELF_FILE="build/micoair_h743_default/micoair_h743_default.elf"
    PX4_FILE="build/micoair_h743_default/micoair_h743_default.px4"

    if [ -f "$ELF_FILE" ]; then
        ELF_SIZE=$(ls -lh "$ELF_FILE" | awk '{print $5}')
        echo -e "ELF文件: ${GREEN}$ELF_FILE${NC}"
        echo -e "ELF大小: ${YELLOW}$ELF_SIZE${NC}"
    fi

    if [ -f "$PX4_FILE" ]; then
        PX4_SIZE=$(ls -lh "$PX4_FILE" | awk '{print $5}')
        echo -e "PX4文件: ${GREEN}$PX4_FILE${NC}"
        echo -e "PX4大小: ${YELLOW}$PX4_SIZE${NC}"
    fi

    echo ""
    echo -e "${GREEN}下一步操作：${NC}"
    echo "  1. 烧录固件："
    echo "     ${YELLOW}make micoair_h743_default upload${NC}"
    echo ""
    echo "  2. 使用JLink调试："
    echo "     ${YELLOW}在VSCode中按F5，选择 'jlink (micoair_h743) - Release'${NC}"
    echo ""

    # 检查是否有调试符号
    if command -v arm-none-eabi-readelf &> /dev/null; then
        if arm-none-eabi-readelf -S "$ELF_FILE" 2>/dev/null | grep -q ".debug_info"; then
            echo -e "  ${GREEN}✓${NC} 包含调试符号 - 可以使用断点和变量查看"
        else
            echo -e "  ${RED}✗${NC} 不包含调试符号 - 建议使用 RelWithDebInfo 或 Debug"
        fi
    fi

else
    echo ""
    echo -e "${RED}======================================${NC}"
    echo -e "${RED}编译失败！${NC}"
    echo -e "${RED}======================================${NC}"
    echo ""
    echo -e "${YELLOW}请检查错误信息${NC}"
    exit 1
fi

