# 三舵轮底盘VIO集成使用指南

## 项目概述

本项目实现了三舵轮底盘的MuJoCo仿真系统，支持两种控制模式：
1. **键盘模式**：通过WASD+QE键控制底盘运动
2. **VIO模式**：通过Carina A1088 VIO设备控制底盘跟随真实运动

## 编译

```bash
cd build
cmake ..
make -j4
```

## 使用方法

### 1. 键盘控制模式（默认）

```bash
./build/swerve_sim models/wheel_base_modified.xml
```

**控制按键：**
- `W/S`: 前进/后退
- `A/D`: 左移/右移
- `Q/E`: 左转/右转
- 鼠标左键: 旋转视角
- 鼠标右键: 平移视角
- 鼠标滚轮: 缩放

### 2. VIO控制模式

```bash
./build/swerve_sim models/wheel_base_modified.xml \
    --mode vio \
    --vio-config /path/to/vio_config.yaml \
    --vio-database /path/to/vio_database.db
```

**可选参数：**
- `--verbose`: 启用详细输出，显示VIO位姿和速度信息

**示例（使用VIO SDK默认配置）：**
```bash
./build/swerve_sim models/wheel_base_modified.xml \
    --mode vio \
    --vio-config /home/mxz/桌面/Ubuntu20.04/x86_64/config/carina_config.yaml \
    --vio-database /home/mxz/桌面/Ubuntu20.04/x86_64/database/carina_vio.db \
    --verbose
```

## VIO模式说明

### 坐标系定义

**世界坐标系（VIO输出）：**
- X轴：前
- Y轴：左  
- Z轴：上

**底盘机体坐标系：**
- X轴：前进方向
- Y轴：左移方向
- Z轴：向上

### 运动模式

#### 平地模式（默认）
- **控制自由度**：X, Y, Yaw（3 DOF）
- **锁定自由度**：Z, Pitch, Roll
- **适用场景**：平地运动

#### 坡面模式（未来扩展）
- **控制自由度**：X, Y, Z, Yaw, Pitch, Roll（6 DOF）
- **适用场景**：上下坡、不平地面

### 数据验证

VIO输入模块会自动验证数据质量：
- 检测NaN/Inf值
- 检测位置跳变（>10m）
- 检测速度异常（>5m/s）
- 超时保护（数据丢失时自动停车）

### 低通滤波

速度指令经过低通滤波平滑处理，避免抖动：
- 滤波系数：α = 0.2
- 公式：`output = 0.2 * raw + 0.8 * prev`

## 调试

### 查看VIO数据

使用`--verbose`参数查看实时VIO数据：

```bash
./build/swerve_sim models/wheel_base_modified.xml --mode vio --verbose \
    --vio-config ... --vio-database ...
```

输出示例：
```
[VIO状态]
  位置: (1.234, 0.456, 0.012) m
  姿态: Roll=0.5° Pitch=1.2° Yaw=15.3°
  速度指令: vx=0.5 vy=0.0 wz=0.0
```

### 常见问题

#### 1. VIO设备未连接
```
[VIO] 设备未连接
错误: VIO设备初始化失败
```
**解决方法：**
- 检查VIO设备USB连接
- 确认设备权限：`sudo chmod 666 /dev/ttyUSB*`
- 运行VIO SDK自带测试程序验证连接

#### 2. 找不到VIO库
```
error while loading shared libraries: libcarina_vio.so
```
**解决方法：**
```bash
export LD_LIBRARY_PATH=/home/mxz/桌面/Ubuntu20.04/x86_64/lib:$LD_LIBRARY_PATH
```

#### 3. 底盘不动或乱动
**检查项：**
- VIO数据是否正常更新（查看`--verbose`输出）
- 速度指令是否合理（不应该全为0或异常大）
- 舵角是否正确（橙色箭头方向）
- 轮子是否转动（观察白色标记）

## 架构说明

### 模块结构

```
src/
├── vio/
│   ├── vio_device.h/cpp       # VIO设备封装
├── control/
│   ├── input_interface.h      # 输入接口（抽象基类）
│   ├── keyboard_input.h/cpp   # 键盘输入实现
│   └── vio_input.h/cpp        # VIO输入实现
├── kinematics/
│   └── swerve_kinematics.h/cpp # 舵轮运动学
├── simulation/
│   └── mujoco_simulator.h/cpp  # MuJoCo仿真器
└── main.cpp                    # 主程序
```

### 数据流

```
VIO Device (A1088)
    │
    ├─ Pose Callback ─────► VIOInput::onVIOPoseUpdate()
    │                              │
    └─ IMU Callback ──────► VIOInput::onVIOIMUUpdate()
                                   │
                                   ▼
                          坐标系转换 & 滤波
                                   │
                                   ▼
                          速度指令 (vx, vy, wz)
                                   │
                                   ▼
                   SwerveKinematics::inverseKinematics()
                                   │
                                   ▼
                    轮子指令 (舵角 + 轮速)
                                   │
                                   ▼
              MuJoCoSimulator::setActuatorCommands()
```

## 下一步开发

### 短期（已完成）
- [x] VIO SDK集成封装
- [x] VIO输入控制器实现
- [x] 坐标系转换与数据验证
- [x] 键盘/VIO模式切换

### 中期（计划中）
- [ ] 6自由度模型扩展（支持Z/Pitch/Roll）
- [ ] 自动模式切换（平地↔坡面）
- [ ] 数据记录与回放
- [ ] 实时状态可视化（HUD）

### 长期（未来）
- [ ] 轨迹跟踪控制
- [ ] 多地形自适应
- [ ] 硬件在环测试

## 参考文档

- [原始计划文档](docs/superpowers/plans/2026-06-30-swerve-chassis-plan.md)
- [VIO集成计划](.claude/plans/vio-integration-plan.md)
- [VIO SDK文档](/home/mxz/桌面/Ubuntu20.04/README.md)
- [MuJoCo文档](https://mujoco.readthedocs.io/)

## 版本历史

- **v0.2.0** (2026-06-30): VIO集成，支持VIO控制模式
- **v0.1.0** (2026-06-30): 初始版本，支持键盘控制模式
