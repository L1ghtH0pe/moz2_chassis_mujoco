#!/bin/bash

# 快速启动菜单
# 提供交互式菜单选择启动模式

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 项目路径
PROJECT_DIR="/home/mxz/桌面/moz2_chassis_mujoco"

clear

print_logo() {
    echo -e "${CYAN}"
    echo "  ╔════════════════════════════════════════════╗"
    echo "  ║                                            ║"
    echo "  ║      三舵轮底盘仿真系统 v0.2.0            ║"
    echo "  ║      Swerve Chassis Simulator              ║"
    echo "  ║                                            ║"
    echo "  ╚════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo ""
}

print_menu() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}请选择启动模式${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "  ${GREEN}1${NC}) 键盘控制模式"
    echo -e "     ${YELLOW}├─${NC} 使用WASD+QE键控制底盘"
    echo -e "     ${YELLOW}└─${NC} 适合快速测试和演示"
    echo ""
    echo -e "  ${GREEN}2${NC}) VIO控制模式"
    echo -e "     ${YELLOW}├─${NC} 使用Carina A1088 VIO设备"
    echo -e "     ${YELLOW}└─${NC} 底盘跟随真实运动"
    echo ""
    echo -e "  ${GREEN}3${NC}) VIO控制模式（详细输出）"
    echo -e "     ${YELLOW}├─${NC} VIO模式 + 实时状态显示"
    echo -e "     ${YELLOW}└─${NC} 适合调试和数据分析"
    echo ""
    echo -e "  ${GREEN}4${NC}) 编译项目"
    echo -e "     ${YELLOW}└─${NC} 重新编译源代码"
    echo ""
    echo -e "  ${GREEN}5${NC}) 检查VIO设备"
    echo -e "     ${YELLOW}└─${NC} 检测VIO设备连接状态"
    echo ""
    echo -e "  ${RED}0${NC}) 退出"
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

check_vio_device() {
    echo -e "${CYAN}[检查VIO设备]${NC}"
    echo ""

    echo -e "${YELLOW}USB设备列表:${NC}"
    lsusb | grep -i "carina\|a1088" || echo "未找到VIO设备"
    echo ""

    echo -e "${YELLOW}串口设备:${NC}"
    if ls /dev/ttyUSB* 2>/dev/null; then
        ls -l /dev/ttyUSB* 2>/dev/null
    else
        echo "未找到串口设备"
    fi
    echo ""

    echo -e "${YELLOW}VIO SDK路径:${NC}"
    VIO_SDK_DIR="/home/mxz/桌面/Ubuntu20.04/x86_64"
    if [ -d "$VIO_SDK_DIR" ]; then
        echo -e "${GREEN}[存在] $VIO_SDK_DIR${NC}"

        # 检查关键文件
        echo -e "\n${YELLOW}SDK文件检查:${NC}"
        [ -f "$VIO_SDK_DIR/lib/libcarina_vio.so" ] && echo -e "${GREEN}✓${NC} libcarina_vio.so" || echo -e "${RED}✗${NC} libcarina_vio.so"
        [ -f "$VIO_SDK_DIR/config/carina_config.yaml" ] && echo -e "${GREEN}✓${NC} carina_config.yaml" || echo -e "${RED}✗${NC} carina_config.yaml"
        [ -f "$VIO_SDK_DIR/database/carina_vio.db" ] && echo -e "${GREEN}✓${NC} carina_vio.db" || echo -e "${RED}✗${NC} carina_vio.db"
    else
        echo -e "${RED}[不存在] $VIO_SDK_DIR${NC}"
    fi

    echo ""
    echo -e "${YELLOW}按 Enter 键返回菜单...${NC}"
    read
}

compile_project() {
    echo -e "${CYAN}[编译项目]${NC}"
    echo ""

    cd "$PROJECT_DIR" || exit 1

    if [ ! -d "build" ]; then
        echo -e "${YELLOW}创建build目录...${NC}"
        mkdir build
    fi

    cd build || exit 1

    echo -e "${YELLOW}运行cmake...${NC}"
    cmake .. || {
        echo -e "${RED}cmake失败${NC}"
        echo -e "${YELLOW}按 Enter 键返回菜单...${NC}"
        read
        return 1
    }

    echo ""
    echo -e "${YELLOW}编译中...${NC}"
    make -j4 || {
        echo -e "${RED}编译失败${NC}"
        echo -e "${YELLOW}按 Enter 键返回菜单...${NC}"
        read
        return 1
    }

    echo ""
    echo -e "${GREEN}编译成功！${NC}"
    echo -e "${YELLOW}按 Enter 键返回菜单...${NC}"
    read
}

start_keyboard_mode() {
    clear
    "$PROJECT_DIR/start_keyboard_mode.sh"
    EXIT_CODE=$?

    echo ""
    echo -e "${YELLOW}按 Enter 键返回菜单...${NC}"
    read
    return $EXIT_CODE
}

start_vio_mode() {
    clear
    "$PROJECT_DIR/start_vio_mode.sh"
    EXIT_CODE=$?

    echo ""
    echo -e "${YELLOW}按 Enter 键返回菜单...${NC}"
    read
    return $EXIT_CODE
}

start_vio_mode_verbose() {
    clear
    "$PROJECT_DIR/start_vio_mode.sh" --verbose
    EXIT_CODE=$?

    echo ""
    echo -e "${YELLOW}按 Enter 键返回菜单...${NC}"
    read
    return $EXIT_CODE
}

# ==================== 主循环 ====================

while true; do
    clear
    print_logo
    print_menu

    read -p "请选择 [0-5]: " choice

    case $choice in
        1)
            start_keyboard_mode
            ;;
        2)
            start_vio_mode
            ;;
        3)
            start_vio_mode_verbose
            ;;
        4)
            clear
            compile_project
            ;;
        5)
            clear
            check_vio_device
            ;;
        0)
            echo ""
            echo -e "${CYAN}再见！${NC}"
            echo ""
            exit 0
            ;;
        *)
            echo ""
            echo -e "${RED}无效选择，请输入 0-5${NC}"
            sleep 1
            ;;
    esac
done
