# VIO集成实施计划

## 项目目标

将Carina A1088 VIO设备集成到三舵轮底盘MuJoCo仿真系统中，实现底盘跟随VIO运动的功能。

## 需求分析

### 用户需求
1. **坐标系对齐**：确保VIO坐标系与底盘坐标系正确转换
2. **舵轮解算无误**：不出现轮子离地、飞起、跳动等bug
3. **运动自由度**：支持xyz和roll/pitch/yaw，但z/pitch/roll仅在上坡时使用，平地不乱动
4. **配置化控制**：支持键盘控制和VIO控制两种模式切换
5. **规范化设计**：明确定义坐标系、正方向、量纲等

### 技术约束
- 现有系统已实现键盘控制，工作正常
- VIO SDK路径：`/home/mxz/桌面/Ubuntu20.04/x86_64/`
- VIO输出：pose[32]包含TWB矩阵(0-15)、速度(16-18)、bias等
- IMU输出：imu[6]包含加速度(0-2)和角速度(3-5)

## 坐标系定义

### 1. 世界坐标系（VIO输出）
- **X轴**：前
- **Y轴**：左
- **Z轴**：上
- **Yaw**：绕Z轴，逆时针为正
- **Pitch**：绕Y轴，抬头为正
- **Roll**：绕X轴，右倾为正

### 2. 底盘机体坐标系
- **X轴**：前进方向（舵轮1,2方向）
- **Y轴**：左移方向
- **Z轴**：向上
- **Yaw**：绕Z轴旋转，逆时针为正

### 3. MuJoCo仿真坐标系
- 与世界坐标系一致
- 关节定义：
  - `base_x`: X方向平移
  - `base_y`: Y方向平移
  - `base_yaw`: Z轴旋转（当前已实现）
  - `base_z`: Z方向平移（需添加）
  - `base_pitch`: Y轴旋转（需添加）
  - `base_roll`: X轴旋转（需添加）

### 4. 量纲规范
- **位置**：米 (m)
- **速度**：米/秒 (m/s)
- **角度**：弧度 (rad)
- **角速度**：弧度/秒 (rad/s)
- **加速度**：米/秒² (m/s²)
- **轮速**：弧度/秒 (rad/s)

## 架构设计

### 1. 模块划分

```
┌─────────────────────────────────────────────────┐
│                  Main Program                    │
│            (支持--mode参数切换)                  │
└────────┬────────────────────────────────────────┘
         │
         ├─── Keyboard Mode ────┐
         │                      │
         └─── VIO Mode ─────────┤
                                │
         ┌──────────────────────┴──────────────────┐
         │          Input Layer                    │
         │  ┌──────────────┐   ┌────────────────┐ │
         │  │KeyboardInput │   │   VIOInput     │ │
         │  └──────────────┘   └────────────────┘ │
         └────────────┬──────────────────────────┬─┘
                      │                          │
         ┌────────────▼──────────────────────────▼─┐
         │        Kinematics Layer                 │
         │      SwerveKinematics (已实现)          │
         └────────────┬────────────────────────────┘
                      │
         ┌────────────▼────────────────────────────┐
         │       Simulation Layer                  │
         │    MuJoCoSimulator (需扩展)             │
         └─────────────────────────────────────────┘
```

### 2. VIO数据流

```
VIO Device (A1088)
    │
    ├─ Pose Callback ──────┐
    │   (TWB, velocity)     │
    │                       │
    └─ IMU Callback ────────┼───► VIOInput::updateVIOData()
        (acc, gyro)         │
                            │
                            ▼
                    数据验证 & 坐标转换
                            │
                            ├─ 世界系速度 → 机体系速度
                            ├─ 提取yaw/pitch/roll
                            └─ 提取z位置和速度
                            │
                            ▼
                    速度指令 (vx, vy, vz, wx, wy, wz)
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

### 3. 运动模式控制策略

#### 平地模式（默认）
- **控制自由度**：X, Y, Yaw（3 DOF）
- **锁定自由度**：Z, Pitch, Roll
- **触发条件**：Pitch < 5°, Roll < 5°, |vz| < 0.01 m/s

#### 坡面模式（自动切换）
- **控制自由度**：X, Y, Z, Yaw, Pitch, Roll（6 DOF）
- **触发条件**：Pitch > 5° 或 Roll > 5° 或 |vz| > 0.01 m/s
- **平滑过渡**：使用低通滤波避免模式切换抖动

## 实施任务

### Task 1: VIO SDK集成封装

**目标**：封装VIO SDK为独立的VIODevice类

**文件**：
- 创建：`src/vio/vio_device.h`
- 创建：`src/vio/vio_device.cpp`

**关键功能**：
```cpp
class VIODevice {
public:
    // 初始化VIO设备
    bool initialize(const std::string& config_path, 
                   const std::string& database_path);
    
    // 启动数据流
    bool start();
    
    // 停止数据流
    void stop();
    
    // 设置回调
    void setPoseCallback(std::function<void(const VIOPoseData&)> callback);
    void setIMUCallback(std::function<void(const VIOIMUData&)> callback);
    
    // 查询设备状态
    bool isConnected() const;
    
private:
    // VIO SDK回调适配器
    static void poseCallbackAdapter(float* pose, double ts, void* user_data);
    static void imuCallbackAdapter(float* imu, double ts, void* user_data);
};

// VIO数据结构
struct VIOPoseData {
    double timestamp;
    Eigen::Matrix4d TWB;        // 世界系到机体系变换矩阵
    Eigen::Vector3d velocity;   // 世界系速度
    Eigen::Vector3d bg;         // 陀螺仪bias
    Eigen::Vector3d ba;         // 加速度计bias
};

struct VIOIMUData {
    double timestamp;
    Eigen::Vector3d acceleration;  // m/s²
    Eigen::Vector3d gyroscope;     // rad/s
};
```

**依赖**：
- VIO SDK: `/home/mxz/桌面/Ubuntu20.04/x86_64/lib/libcarina_vio.so`
- 头文件: `/home/mxz/桌面/Ubuntu20.04/x86_64/include/carina_a1088.h`

---

### Task 2: VIOInput重构

**目标**：重写VIOInput以支持实时VIO数据流

**文件**：
- 修改：`src/control/vio_input.h`
- 修改：`src/control/vio_input.cpp`

**核心改进**：
1. **实时数据处理**：
   ```cpp
   void onVIOPoseUpdate(const VIOPoseData& data);
   void onVIOIMUUpdate(const VIOIMUData& data);
   ```

2. **坐标系转换**：
   ```cpp
   // 世界系速度 → 机体系速度
   Eigen::Vector3d transformVelocity(
       const Eigen::Vector3d& world_vel,
       const Eigen::Matrix3d& R_WB
   );
   
   // 提取欧拉角（ZYX顺序）
   Eigen::Vector3d extractEulerAngles(const Eigen::Matrix3d& R);
   ```

3. **运动模式判断**：
   ```cpp
   enum class MotionMode {
       FLAT_GROUND,  // 平地模式：XY + Yaw
       SLOPE         // 坡面模式：XYZ + RPY
   };
   
   MotionMode determineMotionMode(
       double pitch, double roll, double vz
   );
   ```

4. **数据滤波**：
   ```cpp
   // 低通滤波器平滑速度指令
   Eigen::Vector6d lowPassFilter(
       const Eigen::Vector6d& raw_vel,
       double alpha = 0.2
   );
   ```

**数据验证**：
- 检查NaN/Inf
- 检查位置跳变（>10m）
- 检查速度异常（>5m/s）
- 检查角速度异常（>10rad/s）

---

### Task 3: MuJoCo模型扩展到6自由度

**目标**：修改模型支持Z/Pitch/Roll运动（可选开启）

**文件**：
- 修改：`models/wheel_base_modified.xml`

**修改内容**：

**方案A：条件编译（推荐）**
```xml
<!-- 平地模式（3 DOF）-->
<joint name="base_x" type="slide" axis="1 0 0" />
<joint name="base_y" type="slide" axis="0 1 0" />
<joint name="base_yaw" type="hinge" axis="0 0 1" />

<!-- 坡面模式（6 DOF，注释掉用于平地）-->
<!--
<joint name="base_z" type="slide" axis="0 0 1" limited="true" range="0 0.2"/>
<joint name="base_pitch" type="hinge" axis="0 1 0" limited="true" range="-0.3 0.3"/>
<joint name="base_roll" type="hinge" axis="1 0 0" limited="true" range="-0.3 0.3"/>
-->
```

**方案B：运行时加载**
- 提供两个模型文件：
  - `wheel_base_3dof.xml`（平地）
  - `wheel_base_6dof.xml`（坡面）
- 根据命令行参数选择加载

**初期建议**：先使用3 DOF模型验证基本功能，确认稳定后再扩展到6 DOF。

---

### Task 4: 主程序改造

**目标**：支持VIO模式切换和配置

**文件**：
- 修改：`src/main.cpp`

**命令行参数**：
```bash
# 键盘模式
./swerve_sim models/wheel_base_modified.xml

# VIO模式
./swerve_sim models/wheel_base_modified.xml --mode vio \
    --vio-config /path/to/vio_config.yaml \
    --vio-database /path/to/database.db

# VIO模式（6自由度）
./swerve_sim models/wheel_base_6dof.xml --mode vio --full-dof
```

**主循环改造**：
```cpp
// 创建输入源（工厂模式）
std::unique_ptr<InputInterface> input;
std::unique_ptr<VIODevice> vio_device;

if (mode == "vio") {
    // 初始化VIO设备
    vio_device = std::make_unique<VIODevice>();
    if (!vio_device->initialize(vio_config, vio_database)) {
        std::cerr << "VIO设备初始化失败" << std::endl;
        return 1;
    }
    
    // 创建VIO输入
    auto vio_input = std::make_unique<VIOInput>();
    
    // 设置回调
    vio_device->setPoseCallback([&](const VIOPoseData& data) {
        vio_input->onVIOPoseUpdate(data);
    });
    
    vio_device->setIMUCallback([&](const VIOIMUData& data) {
        vio_input->onVIOIMUUpdate(data);
    });
    
    // 启动VIO
    vio_device->start();
    input = std::move(vio_input);
    
} else {
    // 键盘模式
    input = std::make_unique<KeyboardInput>(simulator.getWindow());
}

// 主循环
while (!simulator.shouldClose()) {
    input->update();
    Eigen::Vector3d vel_cmd = input->getVelocityCommand();
    
    // 运动学解算...
    // 仿真步进...
    // 渲染...
}
```

---

### Task 5: CMake构建配置

**目标**：添加VIO SDK依赖

**文件**：
- 修改：`CMakeLists.txt`

**修改内容**：
```cmake
# VIO SDK路径
set(VIO_SDK_PATH "/home/mxz/桌面/Ubuntu20.04/x86_64")

# 包含VIO头文件
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${VIO_SDK_PATH}/include
)

# 链接VIO库
link_directories(
    ${VIO_SDK_PATH}/lib
)

# 添加VIO源文件
set(SOURCES
    # ... 现有文件 ...
    src/vio/vio_device.cpp
)

# 链接库
target_link_libraries(swerve_sim
    # ... 现有库 ...
    carina_vio
    pthread
)

# 设置RPATH
set_target_properties(swerve_sim PROPERTIES
    INSTALL_RPATH "${VIO_SDK_PATH}/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)
```

---

### Task 6: 数据验证与调试工具

**目标**：提供调试输出和数据记录功能

**文件**：
- 创建：`src/utils/data_logger.h`
- 创建：`src/utils/data_logger.cpp`

**功能**：
```cpp
class DataLogger {
public:
    // 记录VIO数据
    void logVIOPose(const VIOPoseData& data);
    void logVIOIMU(const VIOIMUData& data);
    
    // 记录底盘状态
    void logChassisState(const ChassisState& state);
    
    // 记录轮子指令
    void logWheelCommands(const WheelCommand cmds[3]);
    
    // 导出为CSV
    void exportToCSV(const std::string& filename);
};
```

**调试输出**（可通过参数控制详细程度）：
```cpp
// 简要模式（默认）
[VIO] FPS: 30.2, Pos: (1.23, 0.45, 0.01), Yaw: 15.3°

// 详细模式（--verbose）
=== VIO Data ===
Timestamp: 123.456789
Position: (1.234, 0.456, 0.012) m
Orientation: (0.987, 0.001, 0.002, 0.156) [qw, qx, qy, qz]
Velocity (world): (0.5, 0.0, 0.0) m/s
Velocity (chassis): (0.5, 0.0, 0.0) m/s
Euler: Roll=0.5° Pitch=1.2° Yaw=15.3°

=== Wheel Commands ===
Wheel 1: Steer=12.3°, Speed=5.2 rad/s
Wheel 2: Steer=12.3°, Speed=5.2 rad/s
Wheel 3: Steer=12.3°, Speed=5.2 rad/s
```

---

### Task 7: 集成测试

**目标**：验证VIO集成的正确性和稳定性

**测试用例**：

#### 7.1 VIO设备连接测试
```cpp
// 测试VIO设备初始化
EXPECT_TRUE(vio_device.initialize(config, database));
EXPECT_TRUE(vio_device.isConnected());
```

#### 7.2 坐标系转换测试
```cpp
// 测试世界系→机体系速度转换
Eigen::Vector3d world_vel(1.0, 0.0, 0.0);  // 世界系X方向
Eigen::Matrix3d R_WB = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ())
                       .toRotationMatrix();
Eigen::Vector3d chassis_vel = transformVelocity(world_vel, R_WB);

// Yaw=45°时，世界系(1,0,0)应转换为机体系(0.707,0.707,0)
EXPECT_NEAR(chassis_vel.x(), 0.707, 0.01);
EXPECT_NEAR(chassis_vel.y(), 0.707, 0.01);
```

#### 7.3 运动学一致性测试
```cpp
// VIO输入应与键盘输入产生相同的轮子指令
VIOInput vio_input;
KeyboardInput kb_input;

// 设置相同的速度指令
vio_input.setVelocity(1.0, 0.0, 0.0);
kb_input.setVelocity(1.0, 0.0, 0.0);

// 运动学解算
WheelCommand vio_cmds[3], kb_cmds[3];
kinematics.inverseKinematics(vio_input.getVelocityCommand(), vio_cmds);
kinematics.inverseKinematics(kb_input.getVelocityCommand(), kb_cmds);

// 验证一致性
for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(vio_cmds[i].steer_angle, kb_cmds[i].steer_angle, 0.01);
    EXPECT_NEAR(vio_cmds[i].wheel_speed, kb_cmds[i].wheel_speed, 0.1);
}
```

#### 7.4 物理稳定性测试
**测试场景**：
1. **静止测试**：VIO输出零速度，底盘应保持静止（5分钟）
2. **直线运动测试**：VIO匀速直线运动，底盘跟随不抖动
3. **圆周运动测试**：VIO做圆周运动，底盘平滑跟随
4. **急停测试**：VIO突然停止，底盘应平滑减速

**检查项**：
- [ ] 轮子是否离地
- [ ] 底盘是否飞起
- [ ] 轮子是否跳动
- [ ] 舵电机是否异常摆动
- [ ] 是否有Warning输出

#### 7.5 边界条件测试
- **数据丢失**：VIO回调停止时，底盘应安全停止
- **数据异常**：NaN/Inf/跳变时，系统应拒绝并保持上一状态
- **高速运动**：速度>2m/s时应限幅
- **模式切换**：平地↔坡面切换应平滑无抖动

---

## 实施顺序与里程碑

### Phase 1: 基础集成（3天）
- [ ] Task 1: VIO SDK封装
- [ ] Task 2: VIOInput重构（仅平地模式）
- [ ] Task 5: CMake配置
- [ ] 里程碑：能够运行VIO模式，底盘跟随VIO在XY平面运动

### Phase 2: 功能完善（2天）
- [ ] Task 4: 主程序改造（支持模式切换）
- [ ] Task 6: 调试工具
- [ ] 里程碑：VIO模式稳定，支持参数配置和日志记录

### Phase 3: 6自由度扩展（2天，可选）
- [ ] Task 3: MuJoCo模型扩展
- [ ] Task 2扩展: VIOInput支持Z/Pitch/Roll
- [ ] 里程碑：支持坡面运动，自动模式切换

### Phase 4: 测试与优化（2天）
- [ ] Task 7: 集成测试
- [ ] 性能优化（回调频率、滤波参数等）
- [ ] 文档完善
- [ ] 里程碑：系统稳定可靠，通过所有测试用例

**总计：7-9天**

---

## 风险与缓解

### 风险1：VIO SDK版本兼容性
**影响**：API不兼容导致编译失败
**缓解**：
- 先验证SDK示例程序能否正常运行
- 使用SDK提供的C接口而非C++接口（更稳定）
- 预留时间处理API差异

### 风险2：坐标系转换错误
**影响**：底盘运动方向错误或不稳定
**缓解**：
- 编写单元测试验证每个转换函数
- 先用静态数据验证，再接入实时数据
- 添加可视化工具（显示VIO轨迹vs底盘轨迹）

### 风险3：物理不稳定
**影响**：底盘抖动、轮子离地等
**缓解**：
- 保留3 DOF模式作为fallback
- 逐步放开自由度（先XY，再Yaw，最后Z/RP）
- 添加速度/加速度限幅
- 使用低通滤波平滑指令

### 风险4：VIO数据质量
**影响**：VIO定位失败或漂移导致底盘失控
**缓解**：
- 实现数据验证机制（NaN检测、跳变检测）
- 添加超时保护（数据超过100ms未更新→停车）
- 提供键盘模式作为备用

---

## 验收标准

### 功能性
- [ ] VIO模式能正常启动，设备连接成功
- [ ] 底盘能跟随VIO在平面上运动（XY + Yaw）
- [ ] 键盘模式和VIO模式可通过参数切换
- [ ] 数据异常时系统能安全停止

### 稳定性
- [ ] 连续运行30分钟无崩溃
- [ ] 不出现轮子离地、飞起、跳动等异常
- [ ] VIO数据丢失时能graceful degradation

### 性能
- [ ] VIO回调延迟<10ms
- [ ] 控制循环频率>100Hz
- [ ] CPU占用<50%

### 可维护性
- [ ] 代码符合现有风格
- [ ] 关键函数有单元测试
- [ ] 提供调试日志和数据记录功能

---

## 后续扩展

### 短期（1-2周）
- 姿态稳定控制（PID控制Pitch/Roll）
- 轨迹跟踪（设定目标路径，底盘自动跟随）
- 实时状态显示（HUD显示速度、位置等）

### 中期（1个月）
- 多地形适应（自动识别坡度，调整控制参数）
- 障碍物检测（结合VIO的特征点数据）
- 路径规划集成

### 长期（3个月）
- 硬件在环测试（真实底盘+VIO）
- 多机协同仿真
- 强化学习控制器训练

---

## 参考文档

1. VIO SDK文档：`/home/mxz/桌面/Ubuntu20.04/README.md`
2. 原计划文档：`docs/superpowers/plans/2026-06-30-swerve-chassis-plan.md`
3. 设计文档：`docs/superpowers/specs/2026-06-30-swerve-chassis-design.md`
4. MuJoCo文档：https://mujoco.readthedocs.io/

---

## 附录：快速参考

### VIO Pose数据结构
```
pose[32]:
  [0-15]  : TWB矩阵（4x4）列主序
  [16-18] : 世界系速度 (vx, vy, vz)
  [19-21] : 保留
  [22-24] : 陀螺仪bias (bgx, bgy, bgz)
  [25-27] : 加速度计bias (bax, bay, baz)
  [28-31] : 保留
```

### TWB矩阵提取
```cpp
Eigen::Matrix4d TWB;
for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
        TWB(i, j) = pose[j*4 + i];  // 列主序
    }
}
Eigen::Matrix3d R = TWB.block<3,3>(0,0);  // 旋转矩阵
Eigen::Vector3d t = TWB.block<3,1>(0,3);  // 平移向量
```

### 欧拉角提取（ZYX顺序，对应Yaw-Pitch-Roll）
```cpp
Eigen::Vector3d eulerAngles(const Eigen::Matrix3d& R) {
    double sy = sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
    bool singular = sy < 1e-6;
    
    double roll, pitch, yaw;
    if (!singular) {
        roll  = atan2(R(2,1), R(2,2));
        pitch = atan2(-R(2,0), sy);
        yaw   = atan2(R(1,0), R(0,0));
    } else {
        roll  = atan2(-R(1,2), R(1,1));
        pitch = atan2(-R(2,0), sy);
        yaw   = 0;
    }
    return Eigen::Vector3d(roll, pitch, yaw);
}
```
