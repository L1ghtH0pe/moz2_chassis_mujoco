#!/bin/bash

# 键盘模式启动脚本
# 用于启动三舵轮底盘键盘控制仿真

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 项目路径
PROJECT_DIR="/home/mxz/桌面/moz2_chassis_mujoco"

# 可执行文件
EXECUTABLE="${PROJECT_DIR}/build/swerve_sim"

# 模型文件
MODEL_FILE="${PROJECT_DIR}/models/wheel_base_modified.xml"

# ==================== 函数定义 ====================

print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  三舵轮底盘键盘模式启动脚本${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

print_usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -m, --model PATH    指定模型文件路径"
    echo "  -h, --help          显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                  # 使用默认配置启动"
    echo "  $0 --model /path/to/model.xml"
    echo ""
}

check_files() {
    echo -e "${YELLOW}[检查] 检查必需文件...${NC}"

    local all_ok=true

    # 检查可执行文件
    if [ -f "$EXECUTABLE" ]; then
        echo -e "${GREEN}[成功] 可执行文件: $EXECUTABLE${NC}"
    else
        echo -e "${RED}[错误] 找不到可执行文件: $EXECUTABLE${NC}"
        echo -e "${YELLOW}[提示] 请先编译项目: cd build && make${NC}"
        all_ok=false
    fi

    # 检查模型文件
    if [ -f "$MODEL_FILE" ]; then
        echo -e "${GREEN}[成功] 模型文件: $MODEL_FILE${NC}"
    else
        echo -e "${RED}[错误] 找不到模型文件: $MODEL_FILE${NC}"
        all_ok=false
    fi

    echo ""

    if [ "$all_ok" = false ]; then
        return 1
    fi
    return 0
}

print_controls() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}控制说明${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo -e "${GREEN}W/S${NC} : 前进/后退"
    echo -e "${GREEN}A/D${NC} : 左移/右移"
    echo -e "${GREEN}Q/E${NC} : 左转/右转"
    echo -e ""
    echo -e "${GREEN}鼠标左键${NC} : 旋转视角"
    echo -e "${GREEN}鼠标右键${NC} : 平移视角"
    echo -e "${GREEN}鼠标滚轮${NC} : 缩放"
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

print_info() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}启动信息${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo -e "可执行文件: ${GREEN}$EXECUTABLE${NC}"
    echo -e "模型文件:   ${GREEN}$MODEL_FILE${NC}"
    echo -e "控制模式:   ${GREEN}键盘${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

start_simulation() {
    echo -e "${GREEN}[启动] 启动键盘模式仿真...${NC}"
    echo ""

    # 构建命令
    CMD="$EXECUTABLE $MODEL_FILE"

    # 显示完整命令
    echo -e "${BLUE}[命令] $CMD${NC}"
    echo ""

    # 切换到项目目录
    cd "$PROJECT_DIR" || exit 1

    # 执行
    $CMD

    # 捕获退出码
    EXIT_CODE=$?

    echo ""
    if [ $EXIT_CODE -eq 0 ]; then
        echo -e "${GREEN}[成功] 仿真正常退出${NC}"
    else
        echo -e "${RED}[错误] 仿真异常退出，退出码: $EXIT_CODE${NC}"
    fi

    return $EXIT_CODE
}

# ==================== 参数解析 ====================

while [[ $# -gt 0 ]]; do
    case $1 in
        -m|--model)
            MODEL_FILE="$2"
            shift 2
            ;;
        -h|--help)
            print_header
            print_usage
            exit 0
            ;;
        *)
            echo -e "${RED}[错误] 未知选项: $1${NC}"
            print_usage
            exit 1
            ;;
    esac
done

# ==================== 主流程 ====================

print_header

# 检查文件
if ! check_files; then
    echo -e "${RED}[失败] 文件检查失败，请修复上述问题后重试${NC}"
    exit 1
fi

# 显示控制说明
print_controls

# 显示启动信息
print_info

# 确认启动
echo -e "${YELLOW}按 Enter 键启动仿真，或 Ctrl+C 取消...${NC}"
read

# 启动仿真
start_simulation

exit $?
