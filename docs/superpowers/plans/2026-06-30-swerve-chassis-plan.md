# 三舵轮底盘 MuJoCo 仿真系统实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现三舵轮底盘的 MuJoCo 仿真系统，支持键盘控制（WASD+QE）实现平移和旋转运动，预留 VIO 接口。

**Architecture:** 分层模块化设计 - 输入层（键盘/VIO）、运动学层（舵轮逆解算）、仿真层（MuJoCo 封装）。采用接口抽象实现输入源解耦，运动学模块独立可测试。

**Tech Stack:** C++17, MuJoCo (>= 2.3.0), GLFW3, OpenGL, CMake

---

## 文件结构规划

本项目将创建以下文件：

**模型文件：**
- `models/wheel_base_modified.xml` - 修改后的 MuJoCo 模型（3自由度关节）

**头文件：**
- `src/kinematics/chassis_geometry.h` - 底盘几何参数定义
- `src/kinematics/swerve_kinematics.h` - 舵轮运动学接口
- `src/control/input_interface.h` - 输入接口基类
- `src/control/keyboard_input.h` - 键盘输入实现
- `src/control/vio_input.h` - VIO输入预留
- `src/simulation/mujoco_simulator.h` - MuJoCo仿真封装

**实现文件：**
- `src/kinematics/swerve_kinematics.cpp` - 舵轮运动学实现
- `src/control/keyboard_input.cpp` - 键盘输入实现
- `src/control/vio_input.cpp` - VIO输入框架
- `src/simulation/mujoco_simulator.cpp` - MuJoCo仿真实现
- `src/main.cpp` - 主程序入口

**构建文件：**
- `CMakeLists.txt` - CMake 配置
- `README.md` - 项目说明

---

## Task 1: 项目结构初始化

**Files:**
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/CMakeLists.txt`
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/README.md`

- [ ] **步骤 1: 创建目录结构**

```bash
cd /home/mxz/桌面/moz2_chassis_mujoco
mkdir -p src/kinematics src/control src/simulation models tests
```

- [ ] **步骤 2: 编写 CMakeLists.txt**

创建文件 `/home/mxz/桌面/moz2_chassis_mujoco/CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.16)
project(swerve_chassis_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 MuJoCo
find_package(mujoco REQUIRED)

# 查找 GLFW3
find_package(glfw3 REQUIRED)

# 查找 OpenGL
find_package(OpenGL REQUIRED)

# 源文件
set(SOURCES
    src/main.cpp
    src/kinematics/swerve_kinematics.cpp
    src/control/keyboard_input.cpp
    src/control/vio_input.cpp
    src/simulation/mujoco_simulator.cpp
)

# 可执行文件
add_executable(swerve_sim ${SOURCES})

# 链接库
target_link_libraries(swerve_sim
    mujoco::mujoco
    glfw
    ${OPENGL_LIBRARIES}
)

# 包含目录
target_include_directories(swerve_sim PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

# 编译选项
target_compile_options(swerve_sim PRIVATE
    -Wall -Wextra
)
```

- [ ] **步骤 3: 编写 README.md**

创建文件 `/home/mxz/桌面/moz2_chassis_mujoco/README.md`：

```markdown
# 三舵轮底盘 MuJoCo 仿真系统

## 项目简介

基于 MuJoCo 的三舵轮底盘仿真系统，支持键盘控制和 VIO 数据接入。

## 依赖安装

### Ubuntu/Debian

```bash
# 安装 GLFW3
sudo apt-get install libglfw3-dev

# 安装 OpenGL
sudo apt-get install libgl1-mesa-dev libglu1-mesa-dev

# 安装 MuJoCo (>= 2.3.0)
# 请参考官方文档: https://mujoco.readthedocs.io/en/stable/programming.html#getting-started
```

## 编译

```bash
mkdir build
cd build
cmake ..
make
```

## 运行

### 键盘控制模式

```bash
./swerve_sim
```

### 控制说明

- **W**: 前进
- **S**: 后退
- **A**: 左移
- **D**: 右移
- **Q**: 逆时针旋转
- **E**: 顺时针旋转
- **ESC**: 退出

### VIO 模式（预留）

```bash
./swerve_sim --vio
```

## 坐标系定义

- **世界坐标系**: X前 Y左 Z上
- **机体坐标系**: X前（舵轮1,2方向） Y左 Z上
- **Yaw角**: 绕Z轴，逆时针为正

## 运动学参数

- 最大线速度: 0.5 m/s
- 最大角速度: 0.5 rad/s
- 轮半径: 0.075 m
- 舵轮位置:
  - 轮1: (0.19, -0.19) m
  - 轮2: (0.19, 0.19) m
  - 轮3: (-0.19, 0) m

## 故障排查

### 编译错误: 找不到 mujoco

确保已正确安装 MuJoCo 并设置环境变量。

### 运行时错误: 无法加载模型

检查 `models/wheel_base_modified.xml` 文件是否存在。

## 后续开发

- [ ] VIO 数据接入完整实现
- [ ] 轨迹记录和回放
- [ ] 参数配置文件
- [ ] 实时状态显示

## 参考文档

- 设计文档: `docs/superpowers/specs/2026-06-30-swerve-chassis-design.md`
- 实施计划: `docs/superpowers/plans/2026-06-30-swerve-chassis-plan.md`
```

- [ ] **步骤 4: 提交初始化**

```bash
git add CMakeLists.txt README.md
git commit -m "chore: 初始化项目结构和构建配置

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 2: 修改 MuJoCo 模型

**Files:**
- Modify: `/home/mxz/下载/wheel_base/wheel_base.xml:25`
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/models/wheel_base_modified.xml`

- [ ] **步骤 1: 复制原始模型到项目**

```bash
cp /home/mxz/下载/wheel_base/wheel_base.xml /home/mxz/桌面/moz2_chassis_mujoco/models/wheel_base_modified.xml
```

- [ ] **步骤 2: 修改关节定义**

修改 `models/wheel_base_modified.xml` 第 25 行，将：
```xml
<joint name="free_joint" type="free" />
```

替换为：
```xml
<joint name="base_x" type="slide" axis="1 0 0" />
<joint name="base_y" type="slide" axis="0 1 0" />
<joint name="base_yaw" type="hinge" axis="0 0 1" />
```

- [ ] **步骤 3: 提交模型修改**

```bash
git add models/wheel_base_modified.xml
git commit -m "feat: 修改底盘模型为3自由度关节

- 将 free_joint 替换为 slide(x,y) + hinge(yaw)
- 锁定 z/pitch/roll 确保初期稳定性

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 3: 底盘几何参数定义

**Files:**
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/src/kinematics/chassis_geometry.h`

- [ ] **步骤 1: 编写几何参数头文件**

创建文件 `src/kinematics/chassis_geometry.h`：

```cpp
#ifndef CHASSIS_GEOMETRY_H
#define CHASSIS_GEOMETRY_H

namespace swerve_chassis {

// 底盘几何参数
struct ChassisGeometry {
    // 舵轮位置（机体坐标系，单位：米）
    struct WheelPosition {
        double x;
        double y;
    };
    
    static constexpr WheelPosition WHEEL_POSITIONS[3] = {
        {0.19, -0.19},  // 轮1: 右前
        {0.19,  0.19},  // 轮2: 左前
        {-0.19, 0.0}    // 轮3: 后中
    };
    
    // 轮半径（单位：米）
    static constexpr double WHEEL_RADIUS = 0.075;
    
    // 控制参数
    static constexpr double MAX_LINEAR_VELOCITY = 0.5;   // m/s
    static constexpr double MAX_ANGULAR_VELOCITY = 0.5;  // rad/s
    static constexpr double ACCEL_TIME = 0.5;            // s
    
    // 运动学阈值
    static constexpr double SPEED_THRESHOLD = 0.001;  // m/s
};

}  // namespace swerve_chassis

#endif  // CHASSIS_GEOMETRY_H
```

- [ ] **步骤 2: 提交几何参数定义**

```bash
git add src/kinematics/chassis_geometry.h
git commit -m "feat: 添加底盘几何参数定义

- 定义三个舵轮位置
- 定义轮半径和控制参数
- 定义运动学阈值

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 4: 舵轮运动学模块

**Files:**
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/src/kinematics/swerve_kinematics.h`
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/src/kinematics/swerve_kinematics.cpp`

- [ ] **步骤 1: 编写运动学头文件**

创建文件 `src/kinematics/swerve_kinematics.h`：

```cpp
#ifndef SWERVE_KINEMATICS_H
#define SWERVE_KINEMATICS_H

#include "chassis_geometry.h"

namespace swerve_chassis {

// 单个轮子的控制指令
struct WheelCommand {
    double steer_angle;  // 舵角 (rad)
    double wheel_speed;  // 轮速 (rad/s)
};

// 舵轮运动学求解器
class SwerveKinematics {
public:
    SwerveKinematics();
    
    // 逆运动学：底盘速度 -> 轮子指令
    // 输入：vx, vy (m/s, 机体坐标系), wz (rad/s)
    // 输出：wheels[3] 包含每个轮子的舵角和轮速
    void inverseKinematics(
        double vx, double vy, double wz,
        WheelCommand wheels[3]
    );
    
private:
    // 舵轮位置
    ChassisGeometry::WheelPosition positions_[3];
    
    // 轮半径
    double wheel_radius_;
    
    // 上一次的舵角（用于低速时保持）
    double last_steer_angles_[3];
};

}  // namespace swerve_chassis

#endif  // SWERVE_KINEMATICS_H
```

- [ ] **步骤 2: 编写运动学实现文件**

创建文件 `src/kinematics/swerve_kinematics.cpp`：

```cpp
#include "kinematics/swerve_kinematics.h"
#include <cmath>
#include <algorithm>

namespace swerve_chassis {

SwerveKinematics::SwerveKinematics()
    : wheel_radius_(ChassisGeometry::WHEEL_RADIUS),
      last_steer_angles_{0.0, 0.0, 0.0} {
    // 初始化舵轮位置
    for (int i = 0; i < 3; i++) {
        positions_[i] = ChassisGeometry::WHEEL_POSITIONS[i];
    }
}

void SwerveKinematics::inverseKinematics(
    double vx, double vy, double wz,
    WheelCommand wheels[3]
) {
    // 输入限幅（防御性编程）
    vx = std::clamp(vx, -2.0, 2.0);
    vy = std::clamp(vy, -2.0, 2.0);
    wz = std::clamp(wz, -5.0, 5.0);
    
    // 对每个舵轮计算指令
    for (int i = 0; i < 3; i++) {
        // 1. 计算轮心速度（机体坐标系）
        // v_wheel = v_chassis + wz x r
        // 其中 wz x r = (-wz*y, wz*x) 是旋转引起的切向速度
        double vi_x = vx - wz * positions_[i].y;
        double vi_y = vy + wz * positions_[i].x;
        
        // 2. 计算轮心速度模长
        double speed = std::sqrt(vi_x * vi_x + vi_y * vi_y);
        
        // 3. 根据速度大小决定舵角和轮速
        if (speed < ChassisGeometry::SPEED_THRESHOLD) {
            // 低速时保持舵角，轮速为0
            wheels[i].steer_angle = last_steer_angles_[i];
            wheels[i].wheel_speed = 0.0;
        } else {
            // 正常速度：计算舵角和轮速
            wheels[i].steer_angle = std::atan2(vi_y, vi_x);
            wheels[i].wheel_speed = speed / wheel_radius_;
            
            // 记录舵角
            last_steer_angles_[i] = wheels[i].steer_angle;
        }
        
        // 4. 轮速限幅（安全保护）
        wheels[i].wheel_speed = std::clamp(wheels[i].wheel_speed, -50.0, 50.0);
    }
}

}  // namespace swerve_chassis
```

- [ ] **步骤 3: 提交运动学模块**

```bash
git add src/kinematics/swerve_kinematics.h src/kinematics/swerve_kinematics.cpp
git commit -m "feat: 实现舵轮逆运动学求解器

- 实现标准三轮舵轮运动学算法
- 低速时保持舵角避免抖动
- 添加输入和输出限幅保护

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 5: 输入接口基类

**Files:**
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/src/control/input_interface.h`

- [ ] **步骤 1: 编写输入接口头文件**

创建文件 `src/control/input_interface.h`：

```cpp
#ifndef INPUT_INTERFACE_H
#define INPUT_INTERFACE_H

namespace swerve_chassis {

// 输入接口基类（抽象类）
class InputInterface {
public:
    virtual ~InputInterface() = default;
    
    // 获取当前速度指令（机体坐标系）
    // 输出：vx, vy (m/s), wz (rad/s)
    virtual void getVelocityCommand(double& vx, double& vy, double& wz) = 0;
    
    // 更新输入状态（每帧调用）
    // 输入：dt 时间增量 (s)
    virtual void update(double dt) = 0;
};

}  // namespace swerve_chassis

#endif  // INPUT_INTERFACE_H
```

- [ ] **步骤 2: 提交输入接口**

```bash
git add src/control/input_interface.h
git commit -m "feat: 添加输入接口基类

- 定义抽象输入接口
- 支持键盘和VIO实现切换

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 6: 键盘输入实现

**Files:**
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/src/control/keyboard_input.h`
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/src/control/keyboard_input.cpp`

- [ ] **步骤 1: 编写键盘输入头文件**

创建文件 `src/control/keyboard_input.h`：

```cpp
#ifndef KEYBOARD_INPUT_H
#define KEYBOARD_INPUT_H

#include "control/input_interface.h"
#include "kinematics/chassis_geometry.h"
#include <GLFW/glfw3.h>

namespace swerve_chassis {

// 键盘输入实现
class KeyboardInput : public InputInterface {
public:
    explicit KeyboardInput(GLFWwindow* window);
    
    void getVelocityCommand(double& vx, double& vy, double& wz) override;
    void update(double dt) override;
    
private:
    GLFWwindow* window_;
    
    // 目标速度（按键直接控制）
    double target_vx_;
    double target_vy_;
    double target_wz_;
    
    // 当前速度（平滑加速后）
    double current_vx_;
    double current_vy_;
    double current_wz_;
    
    // 控制参数
    static constexpr double MAX_LINEAR_VEL = ChassisGeometry::MAX_LINEAR_VELOCITY;
    static constexpr double MAX_ANGULAR_VEL = ChassisGeometry::MAX_ANGULAR_VELOCITY;
    static constexpr double ACCEL_TIME = ChassisGeometry::ACCEL_TIME;
};

}  // namespace swerve_chassis

#endif  // KEYBOARD_INPUT_H
```

- [ ] **步骤 2: 编写键盘输入实现文件**

创建文件 `src/control/keyboard_input.cpp`：

```cpp
#include "control/keyboard_input.h"
#include <algorithm>

namespace swerve_chassis {

KeyboardInput::KeyboardInput(GLFWwindow* window)
    : window_(window),
      target_vx_(0.0), target_vy_(0.0), target_wz_(0.0),
      current_vx_(0.0), current_vy_(0.0), current_wz_(0.0) {
}

void KeyboardInput::update(double dt) {
    // 读取按键状态并更新目标速度
    target_vx_ = 0.0;
    target_vy_ = 0.0;
    target_wz_ = 0.0;
    
    // W/S: 前进/后退
    if (glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS) {
        target_vx_ = MAX_LINEAR_VEL;
    }
    if (glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS) {
        target_vx_ = -MAX_LINEAR_VEL;
    }
    
    // A/D: 左移/右移
    if (glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS) {
        target_vy_ = MAX_LINEAR_VEL;
    }
    if (glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS) {
        target_vy_ = -MAX_LINEAR_VEL;
    }
    
    // Q/E: 逆时针/顺时针旋转
    if (glfwGetKey(window_, GLFW_KEY_Q) == GLFW_PRESS) {
        target_wz_ = MAX_ANGULAR_VEL;
    }
    if (glfwGetKey(window_, GLFW_KEY_E) == GLFW_PRESS) {
        target_wz_ = -MAX_ANGULAR_VEL;
    }
    
    // 平滑加速（线性插值）
    double alpha = std::min(1.0, dt / ACCEL_TIME);
    current_vx_ += (target_vx_ - current_vx_) * alpha;
    current_vy_ += (target_vy_ - current_vy_) * alpha;
    current_wz_ += (target_wz_ - current_wz_) * alpha;
}

void KeyboardInput::getVelocityCommand(double& vx, double& vy, double& wz) {
    vx = current_vx_;
    vy = current_vy_;
    wz = current_wz_;
}

}  // namespace swerve_chassis
```

- [ ] **步骤 3: 提交键盘输入模块**

```bash
git add src/control/keyboard_input.h src/control/keyboard_input.cpp
git commit -m "feat: 实现键盘输入控制

- WASD控制平移，QE控制旋转
- 平滑加速避免突变
- 遵循控制参数限制

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```


---

## Task 7: VIO 输入框架（预留）

**Files:**
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/src/control/vio_input.h`
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/src/control/vio_input.cpp`

- [ ] **步骤 1: 编写 VIO 输入头文件**

创建文件 `src/control/vio_input.h`：

```cpp
#ifndef VIO_INPUT_H
#define VIO_INPUT_H

#include "control/input_interface.h"

namespace swerve_chassis {

// VIO 输入实现（预留接口）
class VIOInput : public InputInterface {
public:
    VIOInput();
    
    void getVelocityCommand(double& vx, double& vy, double& wz) override;
    void update(double dt) override;
    
    // VIO 数据更新接口（供外部调用）
    // pose[32]: 位姿数据（TWB矩阵、速度等）
    // imu[6]: IMU数据（加速度、角速度）
    void updateVIOData(const float pose[32], const float imu[6]);
    
private:
    // 当前速度指令
    double vx_;
    double vy_;
    double wz_;
    
    // 数据有效性标志
    bool data_valid_;
    
    // 辅助函数
    void extractVelocityFromPose(const float pose[32]);
    double extractYawRateFromIMU(const float imu[6]);
};

}  // namespace swerve_chassis

#endif  // VIO_INPUT_H
```

- [ ] **步骤 2: 编写 VIO 输入实现文件**

创建文件 `src/control/vio_input.cpp`：

```cpp
#include "control/vio_input.h"
#include <cmath>
#include <iostream>

namespace swerve_chassis {

VIOInput::VIOInput()
    : vx_(0.0), vy_(0.0), wz_(0.0),
      data_valid_(false) {
}

void VIOInput::update(double dt) {
    // VIO模式下update不做处理
    // 数据通过updateVIOData异步更新
    (void)dt;  // 避免未使用参数警告
}

void VIOInput::getVelocityCommand(double& vx, double& vy, double& wz) {
    if (data_valid_) {
        vx = vx_;
        vy = vy_;
        wz = wz_;
    } else {
        // 无有效数据时输出零速度
        vx = 0.0;
        vy = 0.0;
        wz = 0.0;
    }
}

void VIOInput::updateVIOData(const float pose[32], const float imu[6]) {
    // 数据有效性检查
    bool valid = true;
    
    // 检查位置是否为NaN或异常大
    for (int i = 12; i < 15; i++) {
        if (std::isnan(pose[i]) || std::abs(pose[i]) > 1000.0) {
            valid = false;
            break;
        }
    }
    
    if (valid) {
        // 提取速度和角速度
        extractVelocityFromPose(pose);
        wz_ = extractYawRateFromIMU(imu);
        data_valid_ = true;
    } else {
        std::cerr << "Warning: Invalid VIO data received" << std::endl;
        vx_ = vy_ = wz_ = 0.0;
        data_valid_ = false;
    }
}

void VIOInput::extractVelocityFromPose(const float pose[32]) {
    // TODO: 实现世界系到机体系的速度转换
    // 当前简化实现：假设 pose[16-18] 已经是机体系速度
    // 完整实现需要：
    // 1. 从 pose[0-15] 提取旋转矩阵
    // 2. 计算当前 yaw 角
    // 3. 将世界系速度转换到机体系
    
    vx_ = pose[16];
    vy_ = pose[17];
    // pose[18] 是 vz，这里不使用
}

double VIOInput::extractYawRateFromIMU(const float imu[6]) {
    // 从 IMU 数据提取 z 轴角速度
    // imu[3-5] 是角速度 (rad/s)
    return imu[5];  // wz
}

}  // namespace swerve_chassis
```

- [ ] **步骤 3: 提交 VIO 输入框架**

```bash
git add src/control/vio_input.h src/control/vio_input.cpp
git commit -m "feat: 添加VIO输入框架（预留）

- 定义VIO数据接口
- 实现基础数据验证
- TODO: 完整坐标系转换逻辑

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 8: MuJoCo 仿真器封装

**Files:**
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/src/simulation/mujoco_simulator.h`
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/src/simulation/mujoco_simulator.cpp`

- [ ] **步骤 1: 编写仿真器头文件**

创建文件 `src/simulation/mujoco_simulator.h`：

```cpp
#ifndef MUJOCO_SIMULATOR_H
#define MUJOCO_SIMULATOR_H

#include "kinematics/swerve_kinematics.h"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <string>

namespace swerve_chassis {

// MuJoCo 仿真器封装
class MuJoCoSimulator {
public:
    explicit MuJoCoSimulator(const std::string& model_path);
    ~MuJoCoSimulator();
    
    // 初始化仿真器（加载模型、创建窗口等）
    bool initialize();
    
    // 设置执行器指令
    void setActuatorCommands(const WheelCommand wheels[3]);
    
    // 仿真步进
    void step();
    
    // 渲染
    void render();
    
    // 获取窗口（用于输入处理）
    GLFWwindow* getWindow() { return window_; }
    
    // 查询底盘状态
    void getBaseState(double& x, double& y, double& yaw);
    
private:
    // MuJoCo 数据结构
    mjModel* model_;
    mjData* data_;
    
    // 渲染相关
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
    GLFWwindow* window_;
    
    // 模型路径
    std::string model_path_;
    
    // 初始化辅助函数
    bool loadModel();
    bool createWindow();
    void setupCamera();
    
    // GLFW 回调
    static void errorCallback(int error, const char* description);
};

}  // namespace swerve_chassis

#endif  // MUJOCO_SIMULATOR_H
```

- [ ] **步骤 2: 编写仿真器实现文件**

创建文件 `src/simulation/mujoco_simulator.cpp`：

```cpp
#include "simulation/mujoco_simulator.h"
#include <iostream>
#include <cmath>

namespace swerve_chassis {

MuJoCoSimulator::MuJoCoSimulator(const std::string& model_path)
    : model_(nullptr), data_(nullptr), window_(nullptr),
      model_path_(model_path) {
}

MuJoCoSimulator::~MuJoCoSimulator() {
    // 清理 MuJoCo 资源
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
    
    // 清理渲染资源
    if (window_) {
        mjr_freeContext(&con_);
        mjv_freeScene(&scn_);
        glfwDestroyWindow(window_);
        glfwTerminate();
    }
}

bool MuJoCoSimulator::initialize() {
    // 1. 加载模型
    if (!loadModel()) {
        return false;
    }
    
    // 2. 创建窗口
    if (!createWindow()) {
        return false;
    }
    
    // 3. 初始化渲染
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_makeScene(model_, &scn_, 2000);
    mjr_defaultContext(&con_);
    mjr_makeContext(model_, &con_, mjFONTSCALE_150);
    
    // 4. 设置相机
    setupCamera();
    
    std::cout << "MuJoCo simulator initialized successfully" << std::endl;
    return true;
}

bool MuJoCoSimulator::loadModel() {
    // 加载 XML 模型
    char error[1000] = "Could not load model";
    model_ = mj_loadXML(model_path_.c_str(), nullptr, error, 1000);
    
    if (!model_) {
        std::cerr << "Error loading model: " << error << std::endl;
        return false;
    }
    
    // 创建数据
    data_ = mj_makeData(model_);
    if (!data_) {
        std::cerr << "Error creating MuJoCo data" << std::endl;
        return false;
    }
    
    std::cout << "Model loaded: " << model_path_ << std::endl;
    return true;
}

bool MuJoCoSimulator::createWindow() {
    // 设置错误回调
    glfwSetErrorCallback(errorCallback);
    
    // 初始化 GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }
    
    // 创建窗口
    window_ = glfwCreateWindow(1200, 900, "Swerve Chassis Simulator", nullptr, nullptr);
    if (!window_) {
        std::cerr << "Failed to create window" << std::endl;
        glfwTerminate();
        return false;
    }
    
    // 设置上下文
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);  // 启用垂直同步
    
    return true;
}

void MuJoCoSimulator::setupCamera() {
    // 设置相机位置和朝向
    cam_.type = mjCAMERA_FREE;
    cam_.distance = 2.0;
    cam_.azimuth = 135.0;
    cam_.elevation = -20.0;
    cam_.lookat[0] = 0.0;
    cam_.lookat[1] = 0.0;
    cam_.lookat[2] = 0.2;
}

void MuJoCoSimulator::setActuatorCommands(const WheelCommand wheels[3]) {
    // 设置舵电机角度（ctrl[0-2]）
    data_->ctrl[0] = wheels[0].steer_angle;
    data_->ctrl[1] = wheels[1].steer_angle;
    data_->ctrl[2] = wheels[2].steer_angle;
    
    // 设置轮电机速度（ctrl[3-5]）
    data_->ctrl[3] = wheels[0].wheel_speed;
    data_->ctrl[4] = wheels[1].wheel_speed;
    data_->ctrl[5] = wheels[2].wheel_speed;
}

void MuJoCoSimulator::step() {
    // 执行物理仿真步进
    mj_step(model_, data_);
    
    // 检测异常
    if (data_->warning[mjWARN_BADQVEL].number > 0) {
        std::cerr << "Warning: Bad qvel detected at time " 
                  << data_->time << std::endl;
    }
}

void MuJoCoSimulator::render() {
    // 获取窗口大小
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    
    // 设置视口
    mjrRect viewport = {0, 0, width, height};
    
    // 更新场景
    mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, 
                    mjCAT_ALL, &scn_);
    
    // 渲染场景
    mjr_render(viewport, &scn_, &con_);
    
    // 交换缓冲区
    glfwSwapBuffers(window_);
}

void MuJoCoSimulator::getBaseState(double& x, double& y, double& yaw) {
    // 从 qpos 读取底盘状态
    // 对于 slide(x) + slide(y) + hinge(yaw) 关节
    x = data_->qpos[0];    // base_x
    y = data_->qpos[1];    // base_y
    yaw = data_->qpos[2];  // base_yaw
}

void MuJoCoSimulator::errorCallback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

}  // namespace swerve_chassis
```

- [ ] **步骤 3: 提交仿真器模块**

```bash
git add src/simulation/mujoco_simulator.h src/simulation/mujoco_simulator.cpp
git commit -m "feat: 实现MuJoCo仿真器封装

- 封装模型加载和初始化
- 封装渲染和窗口管理
- 提供执行器控制接口

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 9: 主程序实现

**Files:**
- Create: `/home/mxz/桌面/moz2_chassis_mujoco/src/main.cpp`

- [ ] **步骤 1: 编写主程序**

创建文件 `src/main.cpp`：

```cpp
#include "simulation/mujoco_simulator.h"
#include "control/keyboard_input.h"
#include "control/vio_input.h"
#include "kinematics/swerve_kinematics.h"
#include <iostream>
#include <memory>
#include <string>

using namespace swerve_chassis;

int main(int argc, char** argv) {
    // 解析命令行参数
    bool use_vio = false;
    if (argc > 1 && std::string(argv[1]) == "--vio") {
        use_vio = true;
        std::cout << "Running in VIO mode" << std::endl;
    } else {
        std::cout << "Running in keyboard mode" << std::endl;
        std::cout << "Controls: W/S=前后, A/D=左右, Q/E=旋转, ESC=退出" << std::endl;
    }
    
    // 1. 初始化 MuJoCo 仿真器
    std::string model_path = "models/wheel_base_modified.xml";
    MuJoCoSimulator simulator(model_path);
    
    if (!simulator.initialize()) {
        std::cerr << "Failed to initialize simulator" << std::endl;
        return 1;
    }
    
    // 2. 创建输入源
    std::unique_ptr<InputInterface> input;
    if (use_vio) {
        input = std::make_unique<VIOInput>();
    } else {
        input = std::make_unique<KeyboardInput>(simulator.getWindow());
    }
    
    // 3. 创建运动学求解器
    SwerveKinematics kinematics;
    
    // 4. 主循环
    double last_time = glfwGetTime();
    
    while (!glfwWindowShouldClose(simulator.getWindow())) {
        // 计算帧时间
        double current_time = glfwGetTime();
        double dt = current_time - last_time;
        last_time = current_time;
        
        // 更新输入
        input->update(dt);
        
        // 获取速度指令
        double vx, vy, wz;
        input->getVelocityCommand(vx, vy, wz);
        
        // 运动学解算
        WheelCommand wheels[3];
        kinematics.inverseKinematics(vx, vy, wz, wheels);
        
        // 设置执行器
        simulator.setActuatorCommands(wheels);
        
        // 仿真步进
        simulator.step();
        
        // 渲染
        simulator.render();
        
        // 处理事件
        glfwPollEvents();
        
        // 检查 ESC 键退出
        if (glfwGetKey(simulator.getWindow(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(simulator.getWindow(), GLFW_TRUE);
        }
    }
    
    std::cout << "Simulation ended" << std::endl;
    return 0;
}
```

- [ ] **步骤 2: 提交主程序**

```bash
git add src/main.cpp
git commit -m "feat: 实现主程序入口

- 命令行参数支持键盘/VIO模式切换
- 实现主控制循环
- 集成所有模块

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 10: 编译和测试

**Files:**
- Test: 所有源文件编译
- Test: 仿真运行和控制测试

- [ ] **步骤 1: 创建构建目录并编译**

```bash
cd /home/mxz/桌面/moz2_chassis_mujoco
mkdir -p build
cd build
cmake ..
make
```

预期输出：
```
[ 16%] Building CXX object CMakeFiles/swerve_sim.dir/src/kinematics/swerve_kinematics.cpp.o
[ 33%] Building CXX object CMakeFiles/swerve_sim.dir/src/control/keyboard_input.cpp.o
[ 50%] Building CXX object CMakeFiles/swerve_sim.dir/src/control/vio_input.cpp.o
[ 66%] Building CXX object CMakeFiles/swerve_sim.dir/src/simulation/mujoco_simulator.cpp.o
[ 83%] Building CXX object CMakeFiles/swerve_sim.dir/src/main.cpp.o
[100%] Linking CXX executable swerve_sim
[100%] Built target swerve_sim
```

- [ ] **步骤 2: 运行仿真器**

```bash
cd /home/mxz/桌面/moz2_chassis_mujoco/build
./swerve_sim
```

预期输出：
```
Running in keyboard mode
Controls: W/S=前后, A/D=左右, Q/E=旋转, ESC=退出
Model loaded: models/wheel_base_modified.xml
MuJoCo simulator initialized successfully
```

- [ ] **步骤 3: 测试键盘控制**

手动测试以下操作：

1. **静止测试**：启动后不按键，观察底盘是否稳定静止
   - 预期：底盘保持在原地，无抖动或漂移

2. **前进测试**：按住 W 键 3 秒
   - 预期：底盘向前（X 轴正方向）平滑加速并移动

3. **后退测试**：按住 S 键 3 秒
   - 预期：底盘向后（X 轴负方向）平滑移动

4. **左移测试**：按住 A 键 3 秒
   - 预期：底盘向左（Y 轴正方向）平移

5. **右移测试**：按住 D 键 3 秒
   - 预期：底盘向右（Y 轴负方向）平移

6. **逆时针旋转测试**：按住 Q 键 3 秒
   - 预期：底盘原地逆时针旋转

7. **顺时针旋转测试**：按住 E 键 3 秒
   - 预期：底盘原地顺时针旋转

8. **组合运动测试**：同时按住 W+A 键
   - 预期：底盘向左前方斜向移动

9. **组合运动测试**：同时按住 W+Q 键
   - 预期：底盘边前进边逆时针旋转

10. **停止测试**：松开所有按键
    - 预期：底盘平滑减速至静止

- [ ] **步骤 4: 检查物理稳定性**

运行 5 分钟，观察是否出现以下异常：

- [ ] 轮子离地
- [ ] 整车飞起
- [ ] 轮子跳动
- [ ] 舵电机异常摆动
- [ ] 控制台出现 Warning 信息

如果出现异常，记录详细现象并检查：
- 运动学计算是否正确
- MuJoCo 模型参数是否合理
- 控制指令是否超出范围

- [ ] **步骤 5: 验证运动学正确性**

使用简单的测试验证运动学：

1. 纯前进（W键）：观察三个舵轮是否都朝向前方（0°）
2. 纯旋转（Q键）：观察三个舵轮是否呈切向分布
3. 对角移动（W+A）：观察舵轮是否朝向左前方（约45°）

- [ ] **步骤 6: 提交测试确认**

```bash
git add -A
git commit -m "test: 验证编译和基本功能测试通过

- 编译成功无错误
- 键盘控制响应正常
- 物理仿真稳定
- 运动学解算正确

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## 实施计划总结

### 任务清单

- [ ] Task 1: 项目结构初始化
- [ ] Task 2: 修改 MuJoCo 模型
- [ ] Task 3: 底盘几何参数定义
- [ ] Task 4: 舵轮运动学模块
- [ ] Task 5: 输入接口基类
- [ ] Task 6: 键盘输入实现
- [ ] Task 7: VIO 输入框架（预留）
- [ ] Task 8: MuJoCo 仿真器封装
- [ ] Task 9: 主程序实现
- [ ] Task 10: 编译和测试

### 预计时间

- Task 1-3: 0.5 天（基础框架）
- Task 4: 1 天（运动学核心）
- Task 5-7: 1 天（输入层）
- Task 8: 1.5 天（仿真封装）
- Task 9-10: 1 天（集成和测试）

**总计：5 天**

### 关键风险

1. **MuJoCo 环境配置**：确保 MuJoCo >= 2.3.0 正确安装
2. **运动学正确性**：Task 10 的测试阶段重点验证
3. **物理稳定性**：模型参数可能需要微调

### 后续扩展

完成此计划后，可进行以下扩展：

- 实现 VIO 数据的完整坐标系转换
- 添加实时状态显示（HUD）
- 实现轨迹记录和回放
- 恢复 6 自由度并实现姿态稳定控制
- 添加参数配置文件支持

