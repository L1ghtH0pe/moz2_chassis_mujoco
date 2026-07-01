#!/bin/bash

# VIO模式启动脚本
# 用于启动三舵轮底盘VIO控制仿真

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 项目路径
PROJECT_DIR="/home/mxz/桌面/moz2_chassis_mujoco"
VIO_SDK_DIR="/home/mxz/桌面/Ubuntu20.04/x86_64"

# 可执行文件
EXECUTABLE="${PROJECT_DIR}/build/swerve_sim"

# 模型文件
MODEL_FILE="${PROJECT_DIR}/models/wheel_base_modified.xml"

# VIO配置
VIO_CONFIG="${VIO_SDK_DIR}/custom_config.yaml"
VIO_DATABASE="${VIO_SDK_DIR}/database.bin"

# 默认选项
VERBOSE=""

# ==================== 函数定义 ====================

print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  三舵轮底盘VIO模式启动脚本${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

print_usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -v, --verbose       启用详细输出模式"
    echo "  -c, --config PATH   指定VIO配置文件路径"
    echo "  -d, --database PATH 指定VIO数据库路径"
    echo "  -m, --model PATH    指定模型文件路径"
    echo "  -h, --help          显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                  # 使用默认配置启动"
    echo "  $0 -v               # 启用详细输出"
    echo "  $0 --config /path/to/config.yaml --database /path/to/db.db"
    echo ""
}

check_vio_device() {
    echo -e "${YELLOW}[检查] 检查VIO设备连接...${NC}"

    local found_device=false

    # 检查USB设备（检查Faraday/Carina/A1088等可能的名称）
    if lsusb | grep -qE "Carina|A1088|Faraday|0906:5678"; then
        echo -e "${GREEN}[成功] 找到VIO相关USB设备${NC}"
        lsusb | grep -E "Carina|A1088|Faraday|0906:5678"
        found_device=true
    else
        echo -e "${YELLOW}[警告] 未通过USB设备名称找到VIO设备${NC}"
        echo "当前USB设备列表:"
        lsusb | head -5
    fi

    # 检查串口设备（可选，某些VIO设备不使用串口）
    if ls /dev/ttyUSB* 2>/dev/null; then
        echo -e "${GREEN}[信息] 找到串口设备${NC}"
        ls -l /dev/ttyUSB* 2>/dev/null

        # 检查权限
        for dev in /dev/ttyUSB*; do
            if [ -r "$dev" ] && [ -w "$dev" ]; then
                echo -e "${GREEN}[成功] $dev 权限正常${NC}"
            else
                echo -e "${YELLOW}[警告] $dev 权限不足${NC}"
                echo -e "${YELLOW}[提示] 如需使用串口，运行: sudo chmod 666 $dev${NC}"
            fi
        done
        found_device=true
    else
        echo -e "${YELLOW}[信息] 未找到串口设备（某些VIO设备不需要串口）${NC}"
    fi

    echo ""

    # 总是返回成功，让VIO SDK自己检测设备
    if [ "$found_device" = false ]; then
        echo -e "${YELLOW}[提示] 设备检测不确定，但将继续启动让VIO SDK自行检测${NC}"
        echo -e "${YELLOW}[提示] 如果启动失败，请确认VIO设备已正确连接${NC}"
        echo ""
    fi

    return 0
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

    # 检查VIO配置文件
    if [ -f "$VIO_CONFIG" ]; then
        echo -e "${GREEN}[成功] VIO配置: $VIO_CONFIG${NC}"
    else
        echo -e "${RED}[错误] 找不到VIO配置: $VIO_CONFIG${NC}"
        all_ok=false
    fi

    # 检查VIO数据库
    if [ -f "$VIO_DATABASE" ]; then
        echo -e "${GREEN}[成功] VIO数据库: $VIO_DATABASE${NC}"
    else
        echo -e "${RED}[错误] 找不到VIO数据库: $VIO_DATABASE${NC}"
        all_ok=false
    fi

    # 检查VIO库
    if [ -f "${VIO_SDK_DIR}/lib/libcarina_vio.so" ]; then
        echo -e "${GREEN}[成功] VIO库: ${VIO_SDK_DIR}/lib/libcarina_vio.so${NC}"
    else
        echo -e "${RED}[错误] 找不到VIO库: ${VIO_SDK_DIR}/lib/libcarina_vio.so${NC}"
        all_ok=false
    fi

    echo ""

    if [ "$all_ok" = false ]; then
        return 1
    fi
    return 0
}

set_environment() {
    echo -e "${YELLOW}[配置] 设置环境变量...${NC}"

    # 设置库路径 - 优先使用新版VIO库
    # 根据README_SLAM_SETUP.md，新版libcarina_vio(1).so与头文件配套
    export LD_LIBRARY_PATH="$HOME/carina_deps:${VIO_SDK_DIR}/lib:$LD_LIBRARY_PATH"
    echo -e "${GREEN}[成功] LD_LIBRARY_PATH=${LD_LIBRARY_PATH}${NC}"

    # 切换到项目目录
    cd "$PROJECT_DIR" || exit 1
    echo -e "${GREEN}[成功] 工作目录: $(pwd)${NC}"

    echo ""
}

print_info() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}启动信息${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo -e "可执行文件: ${GREEN}$EXECUTABLE${NC}"
    echo -e "模型文件:   ${GREEN}$MODEL_FILE${NC}"
    echo -e "VIO配置:    ${GREEN}$VIO_CONFIG${NC}"
    echo -e "VIO数据库:  ${GREEN}$VIO_DATABASE${NC}"
    echo -e "详细输出:   ${GREEN}${VERBOSE:-否}${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

start_simulation() {
    echo -e "${GREEN}[启动] 启动VIO模式仿真...${NC}"
    echo ""

    # 构建命令
    CMD="$EXECUTABLE $MODEL_FILE --mode vio --vio-config $VIO_CONFIG --vio-database $VIO_DATABASE"

    if [ -n "$VERBOSE" ]; then
        CMD="$CMD --verbose"
    fi

    # 显示完整命令
    echo -e "${BLUE}[命令] $CMD${NC}"
    echo ""

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
        -v|--verbose)
            VERBOSE="--verbose"
            shift
            ;;
        -c|--config)
            VIO_CONFIG="$2"
            shift 2
            ;;
        -d|--database)
            VIO_DATABASE="$2"
            shift 2
            ;;
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

# 检查VIO设备（仅提示信息，不阻止启动）
check_vio_device

# 设置环境
set_environment

# 显示启动信息
print_info

# 确认启动
echo -e "${YELLOW}按 Enter 键启动仿真，或 Ctrl+C 取消...${NC}"
read

# 启动仿真
start_simulation

exit $?
