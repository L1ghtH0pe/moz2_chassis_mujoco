# 三舵轮底盘MuJoCo仿真系统

基于MuJoCo的三舵轮底盘仿真系统，支持键盘控制和VIO设备控制两种模式。

[![Version](https://img.shields.io/badge/version-0.2.0-blue.svg)](https://github.com/L1ghtH0pe/moz2_chassis_mujoco)
[![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)](https://github.com/L1ghtH0pe/moz2_chassis_mujoco)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

## ✨ 特性

- 🎮 **键盘控制模式**：使用WASD+QE键控制底盘运动
- 🤖 **VIO控制模式**：通过Carina A1088 VIO设备控制底盘跟随真实运动
- 🚀 **一键启动**：交互式彩色菜单，自动检测和诊断
- 📊 **实时可视化**：MuJoCo 3D仿真环境，带有可视化坐标系和方向箭头
- 🛡️ **数据验证**：多层验证机制（NaN/Inf/跳变/异常检测）确保系统稳定
- 📚 **完整文档**：使用指南、测试指南、故障排查

## 🚀 快速开始

### 1. 克隆仓库
```bash
git clone https://github.com/L1ghtH0pe/moz2_chassis_mujoco.git
cd moz2_chassis_mujoco
```

### 2. 安装依赖

**Ubuntu 20.04+**:
```bash
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libeigen3-dev
```

**MuJoCo**: 已安装在 `/home/mxz/mujoco` 或 `/usr/local/mujoco`

### 3. 编译项目
```bash
mkdir build && cd build
cmake ..
make -j4
cd ..
```

### 4. 启动仿真

**最简单方式（推荐）**：
```bash
./start.sh
```

然后选择：
- 按 `1` → 键盘控制模式（WASD+QE）
- 按 `2` → VIO控制模式
- 按 `3` → VIO控制模式（详细输出）
- 按 `4` → 编译项目
- 按 `5` → 检查VIO设备

**或直接启动**：
```bash
# 键盘模式
./start_keyboard_mode.sh

# VIO模式
./start_vio_mode.sh --verbose
```

## 🎮 控制说明

### 键盘模式
- `W/S`: 前进/后退
- `A/D`: 左移/右移
- `Q/E`: 左转/右转
- 鼠标左键: 旋转视角
- 鼠标右键: 平移视角
- 鼠标滚轮: 缩放

### VIO模式
底盘自动跟随VIO设备运动（需要连接Carina A1088设备）

## 📖 文档

| 文档 | 说明 |
|------|------|
| [SCRIPTS_README.md](SCRIPTS_README.md) | 启动脚本使用说明（3种启动方式） |
| [VIO_INTEGRATION_GUIDE.md](VIO_INTEGRATION_GUIDE.md) | VIO功能完整使用指南 |
| [VIO_TESTING_GUIDE.md](VIO_TESTING_GUIDE.md) | 详细测试步骤和验证方法 |
| [DELIVERY_REPORT.md](DELIVERY_REPORT.md) | 项目交付报告（26/26验证通过） |
| [FINAL_SUMMARY.md](FINAL_SUMMARY.md) | 项目完成总结 |

## 🏗️ 项目结构

```
moz2_chassis_mujoco/
├── src/                      # 源代码
│   ├── vio/                  # VIO设备封装
│   │   ├── vio_device.h
│   │   └── vio_device.cpp
│   ├── control/              # 输入控制
│   │   ├── keyboard_input.h/cpp
│   │   └── vio_input.h/cpp   # VIO输入（重构）
│   ├── kinematics/           # 舵轮运动学
│   │   └── swerve_kinematics.h/cpp
│   └── simulation/           # MuJoCo仿真器
│       └── mujoco_simulator.h/cpp
├── models/                   # MuJoCo模型
│   └── wheel_base_modified.xml
├── start.sh                  # 主启动菜单（交互式）
├── start_keyboard_mode.sh    # 键盘模式启动
├── start_vio_mode.sh         # VIO模式启动
├── verify.sh                 # 项目完整性验证
└── docs/                     # 详细文档
```

## ✅ 验证

运行验证脚本检查项目完整性：

```bash
./verify.sh
```

**预期输出**：
```
✓ 核心文件检查：6/6 通过
✓ 启动脚本检查：6/6 通过
✓ 文档检查：6/6 通过
✓ VIO SDK检查：4/4 通过
✓ 编译系统检查：2/2 通过
✓ Git状态检查：2/2 通过

总计：26/26 通过 ✓
```

## 🔧 系统要求

- **操作系统**：Ubuntu 20.04+
- **编译器**：GCC 7.5+, C++17支持
- **依赖库**：
  - MuJoCo (>= 2.3.0)
  - GLFW3
  - OpenGL
  - Eigen3
  - Carina VIO SDK（仅VIO模式需要）

## 📝 技术特性

### 坐标系定义
- **世界系**：X前 Y左 Z上
- **机体系**：X前 Y左 Z上
- **欧拉角**：ZYX顺序（Yaw-Pitch-Roll）
- **量纲**：SI标准（米、弧度、秒）

### 数据处理
- **坐标转换**：世界系 ↔ 机体系
- **数据验证**：NaN/Inf检测、位置跳变检测（>10m）、速度异常检测（>5m/s）
- **低通滤波**：平滑速度指令（α=0.2）
- **线程安全**：VIO数据使用互斥锁保护

### 运动学模型
三舵轮底盘采用全向移动配置，三个轮子呈120°均匀分布：
```
输入：(vx, vy, ωz) - 底盘速度
输出：(θ1, v1, θ2, v2, θ3, v3) - 各轮转角和速度
```

算法确保舵轮转角始终 < 90°（通过反向轮速实现）

## 🔍 故障排查

### 问题1：VIO设备未连接
```bash
# 检查USB设备
lsusb

# 检查串口权限
sudo chmod 666 /dev/ttyUSB*
```

### 问题2：找不到VIO库
```bash
export LD_LIBRARY_PATH=/home/mxz/桌面/Ubuntu20.04/x86_64/lib:$LD_LIBRARY_PATH
```

### 问题3：编译失败
```bash
# 使用启动菜单编译
./start.sh
# 选择 "4) 编译项目"
```

更多问题请参考：
- [VIO_INTEGRATION_GUIDE.md](VIO_INTEGRATION_GUIDE.md) - 使用指南
- [SCRIPTS_README.md](SCRIPTS_README.md) - 脚本说明

## 🛣️ 开发路线图

### ✅ v0.2.0（当前版本）
- ✅ 键盘控制模式
- ✅ VIO控制模式（3自由度：XY + Yaw）
- ✅ 舵轮转角优化（确保<90°）
- ✅ 交互式启动脚本系统
- ✅ 完整文档体系（6个文档，~4370行）
- ✅ 项目验证脚本

### 🔜 v0.3.0（计划中）
- [ ] 6自由度支持（Z + Pitch + Roll）
- [ ] 自动模式切换（平地 ↔ 坡面）
- [ ] 重新设计舵轮协调逻辑
- [ ] 数据记录与回放功能

### 📋 v0.4.0（未来）
- [ ] 实时状态HUD显示
- [ ] 轨迹跟踪控制器
- [ ] 多地形自适应
- [ ] 性能分析工具

## 🏆 项目亮点

### 1. 用户友好
- 交互式彩色菜单
- 自动检测和诊断
- 详细错误提示
- 一键启动

### 2. 工程化实践
- 模块化设计（输入/运动学/仿真分层）
- 线程安全（VIO数据处理）
- 完善的错误处理
- Git版本控制

### 3. 完整文档
- 6个文档，涵盖使用/测试/故障排查
- 从入门到精通
- 包含测试报告模板

### 4. 可扩展架构
- 预留6自由度接口
- 支持多种运动模式
- 易于添加新输入源

## 📊 项目统计

- **C++代码**：~1000行
- **Shell脚本**：~745行
- **文档**：~4370行
- **总计**：~6115行
- **Git提交**：7次
- **验证通过**：26/26项

## 🤝 贡献

欢迎提交Issue和Pull Request！

开发流程：
1. Fork本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 提交Pull Request

## 📄 许可证

本项目基于实验和研究目的开发。

## 📧 联系

- **GitHub**: https://github.com/L1ghtH0pe/moz2_chassis_mujoco
- **问题反馈**: [GitHub Issues](https://github.com/L1ghtH0pe/moz2_chassis_mujoco/issues)

## 🙏 致谢

- [MuJoCo](https://mujoco.org/) - 物理仿真引擎
- [Eigen](https://eigen.tuxfamily.org/) - 线性代数库
- Carina VIO SDK - 视觉惯性里程计

---

**🚀 立即开始**：`./start.sh`

**📚 完整文档**：查看 [VIO_INTEGRATION_GUIDE.md](VIO_INTEGRATION_GUIDE.md)

**✅ 验证项目**：`./verify.sh`
