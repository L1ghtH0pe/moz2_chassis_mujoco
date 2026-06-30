# VIO集成项目总结

## 项目目标 ✅

将Carina A1088 VIO设备集成到三舵轮底盘MuJoCo仿真系统中，实现底盘跟随VIO运动的功能。

**目标达成**：✅ 已完成基础集成，系统编译通过，等待VIO设备测试。

---

## 完成的工作

### 1. VIO设备封装层
**文件**：`src/vio/vio_device.h`, `src/vio/vio_device.cpp`

**功能**：
- ✅ 封装Carina A1088 VIO SDK C接口
- ✅ 设备初始化、启动、停止、资源释放
- ✅ Pose回调（TWB矩阵、速度、bias）
- ✅ IMU回调（加速度、角速度）
- ✅ 数据解析与格式转换
- ✅ 设备状态查询（连接、版本、序列号）

**数据结构**：
```cpp
struct VIOPoseData {
    double timestamp;
    Eigen::Matrix4d TWB;        // 世界→机体变换矩阵
    Eigen::Vector3d velocity;   // 世界系速度
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d accel_bias;
};

struct VIOIMUData {
    double timestamp;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d gyroscope;
};
```

---

### 2. VIO输入控制器重构
**文件**：`src/control/vio_input.h`, `src/control/vio_input.cpp`

**功能**：
- ✅ 实时VIO数据流处理（线程安全）
- ✅ 坐标系转换：世界系 → 机体系
- ✅ 欧拉角提取（ZYX顺序：Yaw-Pitch-Roll）
- ✅ 低通滤波平滑速度指令（α=0.2）
- ✅ 数据验证：
  - NaN/Inf检测
  - 位置跳变检测（>10m）
  - 速度异常检测（>5m/s）
- ✅ 运动模式判断（平地/坡面）
- ✅ 速度限幅保证安全

**核心算法**：
```cpp
// 世界系→机体系速度转换
V_body = R_WB^T * V_world

// 欧拉角提取（ZYX）
roll  = atan2(R(2,1), R(2,2))
pitch = atan2(-R(2,0), sy)
yaw   = atan2(R(1,0), R(0,0))

// 低通滤波
output = 0.2 * raw + 0.8 * prev
```

---

### 3. 主程序改造
**文件**：`src/main.cpp`

**功能**：
- ✅ 支持 `--mode <keyboard|vio>` 参数切换控制模式
- ✅ VIO配置参数：`--vio-config`, `--vio-database`
- ✅ 详细输出模式：`--verbose`
- ✅ 工厂模式创建输入源（InputInterface）
- ✅ 统一的主循环处理两种模式

**使用示例**：
```bash
# 键盘模式
./build/swerve_sim models/wheel_base_modified.xml

# VIO模式
./build/swerve_sim models/wheel_base_modified.xml \
    --mode vio \
    --vio-config /path/to/config.yaml \
    --vio-database /path/to/database.db \
    --verbose
```

---

### 4. 构建系统配置
**文件**：`CMakeLists.txt`

**修改**：
- ✅ 添加VIO SDK路径：`/home/mxz/桌面/Ubuntu20.04/x86_64`
- ✅ 包含VIO头文件目录
- ✅ 链接 `libcarina_vio.so`
- ✅ 链接 `pthread`（VIO SDK依赖）
- ✅ 设置RPATH确保运行时找到动态库
- ✅ 添加 `src/vio/vio_device.cpp` 到源文件列表

**编译结果**：✅ 编译成功，无错误，仅有警告（未使用变量）

---

### 5. 文档

#### 5.1 VIO集成使用指南
**文件**：`VIO_INTEGRATION_GUIDE.md`

**内容**：
- 项目概述
- 编译方法
- 使用方法（键盘/VIO模式）
- 坐标系定义
- 运动模式说明
- 数据验证机制
- 调试方法
- 常见问题排查
- 架构说明
- 开发路线图

#### 5.2 VIO集成计划
**文件**：`.claude/plans/vio-integration-plan.md`

**内容**：
- 需求分析
- 坐标系定义
- 架构设计
- 实施任务分解（Task 1-7）
- 实施顺序与里程碑
- 风险与缓解
- 验收标准
- 快速参考（数据结构、公式）

#### 5.3 测试指南
**文件**：`VIO_TESTING_GUIDE.md`

**内容**：
- 环境准备
- 键盘模式回归测试
- VIO模式功能测试（6个测试场景）
- 数据验证测试
- 性能测试
- 稳定性测试
- 故障排查
- 测试报告模板

---

## 技术亮点

### 1. 模块化设计
- **输入层**：`InputInterface` 抽象基类，`KeyboardInput` 和 `VIOInput` 实现
- **运动学层**：`SwerveKinematics` 独立可测试
- **仿真层**：`MuJoCoSimulator` 封装MuJoCo
- **解耦**：各层独立，易于维护和扩展

### 2. 线程安全
```cpp
std::mutex data_mutex_;                    // 保护VIO数据
std::atomic<MotionMode> motion_mode_;     // 原子操作状态
std::atomic<bool> has_valid_data_;        // 原子操作标志
```

### 3. 数据验证
- 多层次验证：NaN/Inf → 跳变 → 异常值
- 优雅降级：检测到异常时安全停止，不崩溃
- 可配置：`setDataValidation(bool)` 启用/禁用

### 4. 坐标系转换
- 严格的数学定义（ZYX欧拉角）
- 考虑万向锁问题
- 世界系↔机体系转换正确性

### 5. 低通滤波
- 平滑速度指令，减少抖动
- 可调参数（FILTER_ALPHA = 0.2）
- 保持系统响应性和稳定性的平衡

---

## 坐标系规范

### 世界坐标系（VIO输出）
- **X轴**：前
- **Y轴**：左
- **Z轴**：上
- **Yaw**：绕Z轴，逆时针为正
- **Pitch**：绕Y轴，抬头为正
- **Roll**：绕X轴，右倾为正

### 底盘机体坐标系
- **X轴**：前进方向
- **Y轴**：左移方向
- **Z轴**：向上
- **Yaw**：绕Z轴旋转，逆时针为正

### 量纲规范
- 位置：米 (m)
- 速度：米/秒 (m/s)
- 角度：弧度 (rad)
- 角速度：弧度/秒 (rad/s)

---

## 项目状态

### Phase 1: 基础集成 ✅ 已完成
- [x] Task 1: VIO SDK封装
- [x] Task 2: VIOInput重构（平地模式）
- [x] Task 4: 主程序改造
- [x] Task 5: CMake配置
- [x] 文档完善

### Phase 2: 功能完善 🔜 待测试
- [ ] 实际VIO设备测试
- [ ] 性能调优
- [ ] Bug修复

### Phase 3: 6自由度扩展 📋 计划中
- [ ] Task 3: MuJoCo模型扩展（6 DOF）
- [ ] Task 2扩展: VIOInput支持Z/Pitch/Roll
- [ ] 自动模式切换（平地↔坡面）

### Phase 4: 测试与优化 📋 计划中
- [ ] Task 7: 集成测试套件
- [ ] 长时间稳定性测试
- [ ] 数据记录与回放

---

## 代码统计

```
文件统计：
- 新增文件：4个
  - src/vio/vio_device.h (118行)
  - src/vio/vio_device.cpp (217行)
  - VIO_INTEGRATION_GUIDE.md
  - VIO_TESTING_GUIDE.md
  
- 修改文件：4个
  - src/control/vio_input.h (大幅重构)
  - src/control/vio_input.cpp (大幅重构)
  - src/main.cpp (新增170行)
  - CMakeLists.txt (新增VIO依赖)

总计新增代码：~800行C++代码 + ~2500行文档
```

---

## 测试计划

### 前置条件
- [x] 代码编译通过
- [ ] VIO设备已连接
- [ ] VIO SDK示例程序可运行
- [ ] 设备权限配置正确

### 测试优先级

#### P0 - 必须通过（阻塞发布）
- [ ] 键盘模式回归测试
- [ ] VIO设备连接测试
- [ ] VIO模式静止测试
- [ ] VIO模式平移测试

#### P1 - 重要（建议通过）
- [ ] VIO模式旋转测试
- [ ] VIO模式组合运动测试
- [ ] 数据异常处理测试

#### P2 - 增强（可选）
- [ ] 性能测试（延迟、CPU占用）
- [ ] 长时间稳定性测试
- [ ] 数据记录功能

---

## 已知问题与限制

### 当前版本限制
1. **仅支持3自由度**（XY + Yaw）
   - 原因：模型为3 DOF，Z/Pitch/Roll未实现
   - 影响：无法在坡面或不平地面运动
   - 计划：Phase 3扩展为6 DOF

2. **舵轮协调已禁用**
   - 原因：之前的协调逻辑导致S/D键轮子不转
   - 影响：舵角未到位时轮速不降低，可能打滑
   - 计划：重新设计更合理的协调策略

3. **无轨迹记录**
   - 影响：无法回放和分析运动轨迹
   - 计划：Task 6添加数据日志功能

### 潜在风险
- VIO定位失败时的处理（已有数据验证，待实测）
- 高速运动时的稳定性（已有速度限幅，待实测）
- 长时间运行的内存泄漏（待测试）

---

## 下一步行动

### 立即行动（本周）
1. **VIO设备测试**
   - 连接VIO设备
   - 运行键盘模式验证无回归
   - 运行VIO模式进行基础测试
   - 记录测试结果

2. **Bug修复**
   - 根据测试结果修复发现的问题
   - 调整参数（滤波系数、阈值等）

3. **性能优化**
   - 测量回调延迟
   - 优化数据处理流程

### 短期目标（2周内）
- [ ] 完成所有P0和P1测试
- [ ] 重新设计舵轮协调逻辑
- [ ] 添加数据记录功能
- [ ] 编写单元测试

### 中期目标（1个月内）
- [ ] 实现6自由度支持
- [ ] 实时状态可视化（HUD）
- [ ] 轨迹跟踪控制器
- [ ] 完整的测试覆盖

---

## 致谢

本次VIO集成基于以下资源：
- Carina A1088 VIO SDK
- MuJoCo物理引擎
- Eigen线性代数库
- 之前的键盘控制基础实现

感谢原作者的基础工作，使得VIO集成能够快速完成。

---

## 参考资料

1. [VIO集成使用指南](VIO_INTEGRATION_GUIDE.md)
2. [VIO集成详细计划](.claude/plans/vio-integration-plan.md)
3. [VIO测试指南](VIO_TESTING_GUIDE.md)
4. [原始项目计划](docs/superpowers/plans/2026-06-30-swerve-chassis-plan.md)
5. [VIO SDK文档](/home/mxz/桌面/Ubuntu20.04/README.md)

---

**项目版本**：v0.2.0  
**完成日期**：2026年6月30日  
**状态**：✅ 基础集成完成，等待设备测试  
**Git Commit**：`e27c7d1` - feat: VIO集成 - 支持Carina A1088 VIO设备控制底盘
