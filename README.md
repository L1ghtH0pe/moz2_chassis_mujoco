# 三舵轮底盘 MuJoCo 仿真系统

基于 MuJoCo 的三舵轮底盘运动学仿真与控制系统。

## 项目概述

本项目实现了一个三舵轮（Swerve Drive）底盘的物理仿真系统，支持键盘和 VIO（视觉惯性里程计）两种输入方式，用于验证底盘运动学模型和控制算法。

### 技术栈

- **物理引擎**: MuJoCo 3.x
- **图形库**: GLFW3 + OpenGL
- **编程语言**: C++17
- **构建系统**: CMake 3.16+

### 系统架构

```
输入层 (Input Layer)
  ├── 键盘输入 (keyboard_input)
  └── VIO输入 (vio_input)
       ↓
运动学层 (Kinematics Layer)
  └── 舵轮逆运动学 (swerve_kinematics)
       ↓
仿真层 (Simulation Layer)
  └── MuJoCo仿真器 (mujoco_simulator)
```

## 功能特性

- ✅ 三舵轮逆运动学解算
- ✅ 键盘实时控制（WASD + 旋转）
- ✅ VIO数据输入支持
- ✅ MuJoCo 物理仿真
- ✅ 实时可视化渲染
- ✅ 轮子转速和转角监控

## 目录结构

```
moz2_chassis_mujoco/
├── src/
│   ├── main.cpp                    # 主程序入口
│   ├── kinematics/
│   │   ├── swerve_kinematics.h     # 舵轮运动学接口
│   │   └── swerve_kinematics.cpp   # 运动学实现
│   ├── control/
│   │   ├── keyboard_input.h        # 键盘输入接口
│   │   ├── keyboard_input.cpp      # 键盘输入实现
│   │   ├── vio_input.h             # VIO输入接口
│   │   └── vio_input.cpp           # VIO输入实现
│   └── simulation/
│       ├── mujoco_simulator.h      # 仿真器接口
│       └── mujoco_simulator.cpp    # 仿真器实现
├── models/
│   └── swerve_chassis.xml          # MuJoCo模型文件
├── tests/                          # 单元测试
├── CMakeLists.txt                  # CMake构建配置
└── README.md                       # 项目文档
```

## 依赖安装

### Ubuntu/Debian

```bash
# 安装 MuJoCo
wget https://github.com/google-deepmind/mujoco/releases/download/3.1.0/mujoco-3.1.0-linux-x86_64.tar.gz
tar -xzf mujoco-3.1.0-linux-x86_64.tar.gz
sudo mv mujoco-3.1.0 /usr/local/mujoco
echo 'export LD_LIBRARY_PATH=/usr/local/mujoco/lib:$LD_LIBRARY_PATH' >> ~/.bashrc

# 安装 GLFW3 和 OpenGL
sudo apt-get update
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

### macOS

```bash
# 使用 Homebrew
brew install mujoco glfw
```

## 构建方法

```bash
# 创建构建目录
mkdir build && cd build

# 配置 CMake
cmake ..

# 编译
make -j$(nproc)

# 运行
./swerve_sim
```

## 使用说明

### 键盘控制模式

```
W/S     - 前进/后退
A/D     - 左移/右移
Q/E     - 左旋/右旋
ESC     - 退出程序
```

### VIO 控制模式

程序会监听 VIO 数据流（待实现具体协议），根据位姿变化计算底盘速度指令。

## 运动学模型

三舵轮底盘采用全向移动配置，三个轮子呈 120° 均匀分布。逆运动学计算公式：

```
输入: (vx, vy, ω) - 底盘速度
输出: (v1, θ1, v2, θ2, v3, θ3) - 各轮速度和转角
```

详细算法见 `src/kinematics/swerve_kinematics.cpp`。

## 开发计划

- [x] 项目结构初始化
- [ ] 实现运动学模块
- [ ] 创建 MuJoCo 模型
- [ ] 实现键盘输入
- [ ] 实现仿真器集成
- [ ] VIO 输入支持
- [ ] 性能优化与测试

## 贡献指南

欢迎提交 Issue 和 Pull Request！

## 许可证

MIT License

## 联系方式

如有问题，请提交 GitHub Issue。
