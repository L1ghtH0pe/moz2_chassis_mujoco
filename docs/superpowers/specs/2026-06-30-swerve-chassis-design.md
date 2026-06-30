# 三舵轮底盘 MuJoCo 仿真系统设计文档

**日期：** 2026-06-30  
**版本：** 1.0  
**状态：** 已批准

## 1. 项目概述

### 1.1 目标

为三舵轮底盘创建 MuJoCo 物理仿真系统，支持键盘控制和未来的 VIO 模块接入，实现精确的平移和旋转运动控制。

### 1.2 核心需求

- 基于现有 `wheel_base.xml` 模型创建仿真
- 键盘控制（WASD + QE）实现平移和 yaw 旋转
- 预留 VIO 模块接口用于后续集成
- C++ 实现，纯 XML 建模（不使用 STL 网格文件）
- 确保舵轮运动学解算正确，避免物理仿真异常

### 1.3 技术约束

- 初期锁定 z/pitch/roll 自由度（仅允许 x/y/yaw 运动）
- 6 个电机协调控制（3 舵电机 + 3 轮电机）
- 保守的速度参数确保初期稳定性

## 2. 系统架构

### 2.1 整体架构

系统采用分层模块化设计：

```
输入层 (Input Layer)
    ├── KeyboardInput: 处理 GLFW 键盘事件
    └── VIOInput: 预留 VIO 数据接口
         ↓ (实现 InputInterface)
控制层 (Control Layer)
    └── ChassisController: 速度指令平滑、限幅
         ↓ (输出 vx, vy, wz)
运动学层 (Kinematics Layer)
    └── SwerveKinematics: 逆运动学解算
         ↓ (输出 3个舵角 + 3个轮速)
仿真层 (Simulation Layer)
    └── MuJoCoSimulator: 执行器控制、物理步进、渲染
```

### 2.2 数据流

```
键盘事件 → KeyboardInput::update() 
         → 速度增量 → ChassisController::setVelocity()
         → 限幅/平滑 → SwerveKinematics::inverseKinematics()
         → 电机指令 → MuJoCo actuators
         → 物理仿真 → 渲染显示
```

## 3. 坐标系定义

### 3.1 世界坐标系 (World Frame)

- **原点：** 地面中心
- **X 轴：** 向前（红色）
- **Y 轴：** 向左（绿色）
- **Z 轴：** 向上（蓝色）

### 3.2 底盘机体坐标系 (Base Frame)

- **原点：** 底盘几何中心
- **X 轴：** 底盘正前方（舵轮1和2所在的方向）
- **Y 轴：** 底盘左侧
- **Z 轴：** 垂直向上
- **Yaw 角：** 绕 Z 轴旋转，逆时针为正

### 3.3 底盘几何参数

从 `wheel_base.xml` 提取的参数：

- **舵轮1位置（机体系）：** (0.19, -0.19, -0.012) m
- **舵轮2位置（机体系）：** (0.19, 0.19, -0.012) m
- **舵轮3位置（机体系）：** (-0.19, 0, -0.012) m
- **轮半径：** 0.075 m
- **底盘质量：** 约 32 kg（主体）+ 4.11 kg × 3（轮子）≈ 44.33 kg

### 3.4 舵轮坐标系

每个舵轮局部坐标系：

- 舵角为 0 时，轮子滚动方向为局部 X 轴正方向
- 舵角逆时针旋转为正（从上往下看）

## 4. 运动控制量纲

### 4.1 速度指令

- **vx:** 机体坐标系 X 方向线速度 (m/s)，正值向前
- **vy:** 机体坐标系 Y 方向线速度 (m/s)，正值向左
- **wz:** 绕 Z 轴角速度 (rad/s)，正值逆时针

### 4.2 电机输出

- **舵电机角度 (rad):** 从机体系 X 轴逆时针测量，范围 [-π, π]
- **轮电机速度 (rad/s):** 正值使底盘沿舵向前进

### 4.3 控制限制（保守参数）

- **最大线速度：** 0.5 m/s
- **最大角速度：** 0.5 rad/s
- **加速时间：** 0.5 s（线性加速到最大速度）

## 5. 舵轮运动学

### 5.1 逆运动学算法

**输入：** 底盘期望速度 (vx, vy, wz) - 机体坐标系  
**输出：** 每个舵轮的舵角 θᵢ 和轮速 ωᵢ

**算法步骤：**

对于第 i 个舵轮（i = 1, 2, 3）：

1. **计算轮心速度**

```
舵轮位置向量：rᵢ = (xᵢ, yᵢ)
旋转引起的切向速度：v_rot = wz × rᵢ = (-wz·yᵢ, wz·xᵢ)
轮心速度（机体系）：
    vᵢ_x = vx + (-wz·yᵢ)
    vᵢ_y = vy + wz·xᵢ
```

2. **计算舵角**

```
θᵢ = atan2(vᵢ_y, vᵢ_x)
```

3. **计算轮速**

```
轮心速度模长：|vᵢ| = sqrt(vᵢ_x² + vᵢ_y²)
轮子角速度：ωᵢ = |vᵢ| / r  （r = 0.075 m 轮半径）
```

### 5.2 特殊情况处理

- 当 |vᵢ| < 阈值（0.001 m/s）时，保持上一次舵角不变，轮速设为 0
- 避免原地微小抖动导致舵轮频繁摆动

### 5.3 数值示例

**纯前进（vx=1, vy=0, wz=0）：**
- 三个舵轮舵角都为 0°
- 轮速都为 1/0.075 ≈ 13.33 rad/s

**纯旋转（vx=0, vy=0, wz=1）：**
- 舵轮1：θ₁ ≈ 45°，速度指向右前
- 舵轮2：θ₂ ≈ -135°，速度指向右后
- 舵轮3：θ₃ = 180°，速度指向后

## 6. MuJoCo 模型修改

### 6.1 关节修改

**当前问题：**
- `free_joint` 允许 6 自由度，可能导致不稳定

**修改方案：**

将 `wheel_base.xml` 第 25 行的 `free_joint` 替换为：

```xml
<joint name="base_x" type="slide" axis="1 0 0" />
<joint name="base_y" type="slide" axis="0 1 0" />
<joint name="base_yaw" type="hinge" axis="0 0 1" />
```

**约束效果：**
- 底盘只能在 XY 平面滑动和绕 Z 轴旋转
- Z 位置、pitch、roll 被完全锁定
- 符合初期稳定性要求

**后续扩展：**
- 调试成功后可改回 `free_joint` + PD 控制器
- 或添加弱阻尼约束允许小幅 Z/pitch/roll 运动

### 6.2 执行器配置

当前 XML 中的执行器配置保持不变：

- **舵电机：** `position` 执行器，PD 控制（kp=2500, kv=100）
- **轮电机：** `velocity` 执行器，速度控制（kv=20）

**控制指令映射：**

```cpp
data->ctrl[0] = steer_angle_1;  // 舵1角度 (rad)
data->ctrl[1] = steer_angle_2;  // 舵2角度
data->ctrl[2] = steer_angle_3;  // 舵3角度
data->ctrl[3] = wheel_speed_1;  // 轮1速度 (rad/s)
data->ctrl[4] = wheel_speed_2;  // 轮2速度
data->ctrl[5] = wheel_speed_3;  // 轮3速度
```

## 7. 模块接口设计

### 7.1 InputInterface（输入接口基类）

```cpp
class InputInterface {
public:
    virtual ~InputInterface() = default;
    
    // 获取当前速度指令（机体坐标系）
    virtual void getVelocityCommand(double& vx, double& vy, double& wz) = 0;
    
    // 更新输入状态（每帧调用）
    virtual void update(double dt) = 0;
};
```

### 7.2 KeyboardInput（键盘输入实现）

```cpp
class KeyboardInput : public InputInterface {
public:
    KeyboardInput(GLFWwindow* window);
    
    void getVelocityCommand(double& vx, double& vy, double& wz) override;
    void update(double dt) override;
    
private:
    GLFWwindow* window_;
    double target_vx_, target_vy_, target_wz_;
    double current_vx_, current_vy_, current_wz_;
    
    static constexpr double MAX_LINEAR_VEL = 0.5;   // m/s
    static constexpr double MAX_ANGULAR_VEL = 0.5;  // rad/s
    static constexpr double ACCEL_TIME = 0.5;       // s
};
```

**按键映射：**

- **W:** 前进（vx = +0.5）
- **S:** 后退（vx = -0.5）
- **A:** 左移（vy = +0.5）
- **D:** 右移（vy = -0.5）
- **Q:** 逆时针旋转（wz = +0.5）
- **E:** 顺时针旋转（wz = -0.5）

**加速处理：**

```cpp
// 线性插值实现平滑加速
double alpha = dt / ACCEL_TIME;
current_vx_ += (target_vx_ - current_vx_) * alpha;
```

### 7.3 VIOInput（VIO 输入预留）

```cpp
class VIOInput : public InputInterface {
public:
    VIOInput();
    
    void getVelocityCommand(double& vx, double& vy, double& wz) override;
    void update(double dt) override;
    
    // VIO 数据更新接口（供外部调用）
    void updateVIOData(const float pose[32], const float imu[6]);
    
private:
    double vx_, vy_, wz_;
    bool data_valid_;
    
    void extractVelocityFromPose(const float pose[32]);
    double extractYawRateFromIMU(const float imu[6]);
};
```

**VIO 数据结构：**

- **pose[32]:**
  - pose[0-15]: 4x4 变换矩阵 TWB（世界到机体）
  - pose[16-18]: 速度（世界坐标系）
  - pose[22-24]: 陀螺仪偏置
  - pose[25-27]: 加速度计偏置

- **imu[6]:**
  - imu[0-2]: 加速度 (m/s²)
  - imu[3-5]: 角速度 (rad/s)

**速度处理逻辑：**

1. 从 `pose[16-18]` 提取世界系速度
2. 从 `pose[0-15]` 提取旋转矩阵，计算当前 yaw
3. 将世界系速度转换到机体系：v_body = R^T * v_world
4. 从 `imu[5]` 直接获取 wz（绕 z 轴角速度）

### 7.4 SwerveKinematics（舵轮运动学）

```cpp
struct WheelCommand {
    double steer_angle;  // rad
    double wheel_speed;  // rad/s
};

class SwerveKinematics {
public:
    SwerveKinematics();
    
    // 逆运动学：底盘速度 → 轮子指令
    void inverseKinematics(
        double vx, double vy, double wz,
        WheelCommand wheels[3]
    );
    
private:
    struct WheelPosition {
        double x, y;
    } positions_[3];
    
    double wheel_radius_;
    double last_steer_angles_[3];
    
    static constexpr double SPEED_THRESHOLD = 0.001;  // m/s
};
```

### 7.5 MuJoCoSimulator（仿真封装）

```cpp
class MuJoCoSimulator {
public:
    MuJoCoSimulator(const std::string& model_path);
    ~MuJoCoSimulator();
    
    bool initialize();
    void setActuatorCommands(const WheelCommand wheels[3]);
    void step();
    void render();
    
    GLFWwindow* getWindow() { return window_; }
    void getBaseState(double& x, double& y, double& yaw);
    
private:
    mjModel* model_;
    mjData* data_;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
    GLFWwindow* window_;
    std::string model_path_;
};
```

## 8. 主程序流程

### 8.1 main.cpp 结构

```cpp
int main(int argc, char** argv) {
    // 1. 初始化 MuJoCo 仿真器
    MuJoCoSimulator simulator("models/wheel_base_modified.xml");
    if (!simulator.initialize()) {
        return 1;
    }
    
    // 2. 创建输入源
    std::unique_ptr<InputInterface> input;
    if (argc > 1 && std::string(argv[1]) == "--vio") {
        input = std::make_unique<VIOInput>();
    } else {
        input = std::make_unique<KeyboardInput>(simulator.getWindow());
    }
    
    // 3. 创建运动学求解器
    SwerveKinematics kinematics;
    
    // 4. 主循环
    double last_time = glfwGetTime();
    while (!glfwWindowShouldClose(simulator.getWindow())) {
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
        
        // 设置执行器并仿真
        simulator.setActuatorCommands(wheels);
        simulator.step();
        simulator.render();
        
        glfwPollEvents();
    }
    
    return 0;
}
```

### 8.2 运行模式

- **键盘模式：** `./swerve_sim`
- **VIO 模式：** `./swerve_sim --vio`（预留）

## 9. 错误处理和稳定性

### 9.1 运动学层安全检查

```cpp
void SwerveKinematics::inverseKinematics(...) {
    // 输入限幅
    vx = std::clamp(vx, -2.0, 2.0);
    vy = std::clamp(vy, -2.0, 2.0);
    wz = std::clamp(wz, -5.0, 5.0);
    
    for (int i = 0; i < 3; i++) {
        // 计算轮心速度
        double vi_x = vx - wz * positions_[i].y;
        double vi_y = vy + wz * positions_[i].x;
        double speed = std::sqrt(vi_x * vi_x + vi_y * vi_y);
        
        // 低速时保持舵角
        if (speed < SPEED_THRESHOLD) {
            wheels[i].steer_angle = last_steer_angles_[i];
            wheels[i].wheel_speed = 0.0;
        } else {
            wheels[i].steer_angle = std::atan2(vi_y, vi_x);
            wheels[i].wheel_speed = speed / wheel_radius_;
            last_steer_angles_[i] = wheels[i].steer_angle;
        }
        
        // 轮速限幅
        wheels[i].wheel_speed = std::clamp(wheels[i].wheel_speed, -50.0, 50.0);
    }
}
```

### 9.2 MuJoCo 异常检测

```cpp
void MuJoCoSimulator::step() {
    mj_step(model_, data_);
    
    // 检测警告
    if (data_->warning[mjWARN_BADQVEL].number > 0) {
        std::cerr << "Warning: Bad qvel detected" << std::endl;
    }
    
    // 检测异常高度
    double base_z = data_->qpos[2];
    if (base_z > 1.0 || base_z < -0.1) {
        std::cerr << "Warning: Abnormal base height: " << base_z << std::endl;
    }
}
```

### 9.3 VIO 数据有效性检查

```cpp
void VIOInput::updateVIOData(const float pose[32], const float imu[6]) {
    bool valid = true;
    
    // 检查 NaN 和异常值
    for (int i = 12; i < 15; i++) {
        if (std::isnan(pose[i]) || std::abs(pose[i]) > 1000.0) {
            valid = false;
            break;
        }
    }
    
    if (valid) {
        extractVelocityFromPose(pose);
        wz_ = imu[5];
        data_valid_ = true;
    } else {
        std::cerr << "Invalid VIO data received" << std::endl;
        vx_ = vy_ = wz_ = 0.0;
        data_valid_ = false;
    }
}
```

## 10. 测试策略

### 10.1 单元测试

**运动学模块测试用例：**

1. **纯前进测试：** vx=1, vy=0, wz=0 → 舵角都为 0°
2. **纯旋转测试：** vx=0, vy=0, wz=1 → 验证舵角分布
3. **混合运动测试：** 各种组合
4. **零速度测试：** 验证舵角保持逻辑

### 10.2 集成测试

**仿真稳定性检查清单：**

- [ ] 轮子始终贴地（无离地现象）
- [ ] 整车无飞起（z 坐标稳定）
- [ ] 轮子无跳动（位置连续变化）
- [ ] 轮子无单方向乱动（速度与指令一致）
- [ ] 舵电机正常响应
- [ ] 舵电机无乱动（低速时保持角度）

**功能测试：**

1. 启动仿真，观察底盘静止状态
2. 依次测试单个按键（W/S/A/D/Q/E）
3. 测试组合按键（W+A、W+Q 等）
4. 长时间运行测试（10 分钟）

### 10.3 性能要求

- **仿真频率：** 500 Hz（timestep=0.002）
- **渲染频率：** 60 FPS
- **控制延迟：** < 5 ms

## 11. 项目结构

```
swerve_chassis_mujoco/
├── CMakeLists.txt
├── README.md
├── models/
│   ├── wheel_base_original.xml
│   └── wheel_base_modified.xml
├── src/
│   ├── main.cpp
│   ├── kinematics/
│   │   ├── chassis_geometry.h
│   │   ├── swerve_kinematics.h
│   │   └── swerve_kinematics.cpp
│   ├── control/
│   │   ├── input_interface.h
│   │   ├── keyboard_input.h
│   │   ├── keyboard_input.cpp
│   │   ├── vio_input.h
│   │   └── vio_input.cpp
│   └── simulation/
│       ├── mujoco_simulator.h
│       └── mujoco_simulator.cpp
├── tests/
│   ├── CMakeLists.txt
│   └── test_kinematics.cpp
└── docs/
    └── superpowers/
        └── specs/
            └── 2026-06-30-swerve-chassis-design.md
```

## 12. 编译配置

### 12.1 依赖库

**必需：**
- MuJoCo (>= 2.3.0)
- GLFW3
- OpenGL

### 12.2 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(swerve_chassis_sim)

set(CMAKE_CXX_STANDARD 17)

find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(swerve_sim
    src/main.cpp
    src/kinematics/swerve_kinematics.cpp
    src/control/keyboard_input.cpp
    src/control/vio_input.cpp
    src/simulation/mujoco_simulator.cpp
)

target_link_libraries(swerve_sim
    mujoco::mujoco
    glfw
    GL
)

target_include_directories(swerve_sim PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)
```

## 13. 实施阶段

### 阶段 1：基础框架（1-2 天）
- 修改 XML 模型（3 自由度）
- 搭建 CMake 工程
- 实现 MuJoCoSimulator 基础功能
- 验证模型加载和渲染

### 阶段 2：运动学和控制（2-3 天）
- 实现 SwerveKinematics
- 编写单元测试验证运动学
- 实现 KeyboardInput
- 集成控制回路

### 阶段 3：测试和调优（1-2 天）
- 运行集成测试
- 调整参数（如需要）
- 修复 bug
- 添加调试信息显示

### 阶段 4：VIO 接口预留（0.5 天）
- 实现 VIOInput 框架
- 添加坐标转换逻辑
- 文档说明接入方法

**总计：约 5-7 天开发时间**

## 14. 风险和缓解措施

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 运动学公式错误导致底盘运动异常 | 高 | 单元测试 + 手算验证特殊情况 |
| MuJoCo 物理参数不当导致不稳定 | 中 | 参考官方示例，保守设置质量/惯量 |
| 舵轮频繁抖动影响控制 | 中 | 低速阈值 + 舵角保持逻辑 |
| VIO 坐标系理解错误 | 中 | 预留坐标系切换开关，实测验证 |
| 编译环境问题 | 低 | 提供详细依赖安装文档 |

## 15. 后续扩展

### 15.1 VIO 集成（后续阶段）

- 实现 `VIOInput::updateVIOData()` 的完整逻辑
- 验证坐标系转换正确性
- 添加 VIO 数据时间戳同步
- 实现闭环控制（如需要）

### 15.2 高级功能（可选）

- 轨迹记录和回放
- 参数配置文件支持
- 实时状态显示（速度、位置、舵角等）
- 数据日志导出（CSV 格式）
- 恢复 6 自由度（free_joint + 姿态稳定控制）

## 16. 参考资料

- MuJoCo 官方文档：https://mujoco.readthedocs.io/
- 舵轮运动学论文：相关三轮全向移动机器人文献
- GLFW 文档：https://www.glfw.org/documentation.html

---

**Why：** 本设计采用分层模块化架构，将输入、控制、运动学和仿真解耦，便于后续 VIO 接入和功能扩展。运动学算法基于标准舵轮运动学模型，通过单元测试保证正确性。初期锁定 z/pitch/roll 自由度以确保仿真稳定性。

**How to apply：** 按照实施阶段逐步开发，先实现键盘控制并验证运动学正确性，再预留 VIO 接口。开发过程中严格遵循接口设计，确保模块间解耦。测试时重点关注物理仿真稳定性和运动学精度。
