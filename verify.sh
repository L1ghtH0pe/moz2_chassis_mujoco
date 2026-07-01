#!/bin/bash

# 快速验证脚本 - 检查项目完整性

echo "=================================="
echo "项目完整性验证"
echo "=================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0

check() {
    local name=$1
    local command=$2

    echo -n "检查 $name... "
    if eval "$command" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ 通过${NC}"
        ((PASS++))
        return 0
    else
        echo -e "${RED}✗ 失败${NC}"
        ((FAIL++))
        return 1
    fi
}

check_file() {
    local name=$1
    local file=$2

    echo -n "检查 $name... "
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓ 存在${NC}"
        ((PASS++))
        return 0
    else
        echo -e "${RED}✗ 缺失${NC}"
        ((FAIL++))
        return 1
    fi
}

echo "=== 核心文件检查 ==="
check_file "可执行文件" "build/swerve_sim"
check_file "模型文件" "models/wheel_base_modified.xml"
check_file "VIO设备头文件" "src/vio/vio_device.h"
check_file "VIO设备实现" "src/vio/vio_device.cpp"
check_file "VIO输入头文件" "src/control/vio_input.h"
check_file "VIO输入实现" "src/control/vio_input.cpp"
echo ""

echo "=== 启动脚本检查 ==="
check_file "主菜单脚本" "start.sh"
check_file "键盘模式脚本" "start_keyboard_mode.sh"
check_file "VIO模式脚本" "start_vio_mode.sh"
check "start.sh可执行权限" "[ -x start.sh ]"
check "start_keyboard_mode.sh可执行权限" "[ -x start_keyboard_mode.sh ]"
check "start_vio_mode.sh可执行权限" "[ -x start_vio_mode.sh ]"
echo ""

echo "=== 文档检查 ==="
check_file "脚本使用说明" "SCRIPTS_README.md"
check_file "VIO集成指南" "VIO_INTEGRATION_GUIDE.md"
check_file "VIO测试指南" "VIO_TESTING_GUIDE.md"
check_file "项目总结" "VIO_PROJECT_SUMMARY.md"
check_file "最终总结" "FINAL_SUMMARY.md"
check_file "VIO集成计划" ".claude/plans/vio-integration-plan.md"
echo ""

echo "=== VIO SDK检查 ==="
VIO_SDK="/home/mxz/桌面/Ubuntu20.04/x86_64"
check_file "VIO SDK库" "$VIO_SDK/lib/libcarina_vio.so"
check_file "VIO头文件" "$VIO_SDK/include/carina_a1088.h"
check_file "VIO配置" "$VIO_SDK/custom_config.yaml"
check_file "VIO数据库" "$VIO_SDK/database.bin"
echo ""

echo "=== 编译系统检查 ==="
check_file "CMakeLists.txt" "CMakeLists.txt"
check_file "build目录" "build/Makefile"
echo ""

echo "=== Git状态检查 ==="
check "Git仓库" "[ -d .git ]"
check "远程仓库配置" "git remote get-url origin"
echo ""

echo "=================================="
echo "验证结果"
echo "=================================="
echo -e "${GREEN}通过: $PASS${NC}"
echo -e "${RED}失败: $FAIL${NC}"
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}✓ 项目完整性验证通过！${NC}"
    echo ""
    echo "下一步："
    echo "1. 运行 ./start.sh 启动交互式菜单"
    echo "2. 选择键盘模式测试基础功能"
    echo "3. 连接VIO设备后测试VIO模式"
    exit 0
else
    echo -e "${RED}✗ 发现 $FAIL 个问题，请检查${NC}"
    exit 1
fi
