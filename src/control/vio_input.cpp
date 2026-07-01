#include "control/vio_input.h"
#include <iostream>
#include <cmath>
#include <algorithm>

VIOInput::VIOInput(double max_linear_vel, double max_angular_vel)
    : max_linear_vel_(max_linear_vel),
      max_angular_vel_(max_angular_vel),
      velocity_command_(Eigen::Vector3d::Zero()),
      prev_velocity_command_(Eigen::Vector3d::Zero()),
      motion_mode_(MotionMode::FLAT_GROUND),
      has_valid_data_(false),
      enable_data_validation_(true),
      last_update_time_(0.0) {
}

bool VIOInput::update() {
    if (!has_valid_data_) {
        velocity_command_.setZero();
        return false;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    // === VIO坐标系说明 ===
    // 不进行坐标轴变换，直接使用VIO输出的速度和角速度
    // 通过实际测试来确定方向映射关系

    // 提取VIO的旋转矩阵和速度
    Eigen::Matrix3d R_WB = current_pose_data_.TWB.block<3,3>(0,0);
    Eigen::Vector3d velocity_world = current_pose_data_.velocity;  // 世界系速度

    // 提取欧拉角（用于调试显示）
    Eigen::Vector3d euler = extractEulerAngles(R_WB);
    double yaw = euler(2);

    // 世界系速度转换为机体系速度
    // 机体系速度 = R_WB^T * 世界系速度
    Eigen::Vector3d velocity_body = R_WB.transpose() * velocity_world;

    // IMU角速度
    Eigen::Vector3d gyro = current_imu_data_.gyroscope;

    // === 坐标系测试日志：只在有明显运动时打印 ===
    double vw_norm = velocity_world.norm();
    double gyro_norm = gyro.norm();
    static int debug_count = 0;
    if ((vw_norm > 0.02 || gyro_norm > 0.1) && (debug_count++ % 5 == 0)) {
        printf("\n========== 坐标系测试 ==========\n");
        printf("【VIO世界系速度】  X=%+.3f  Y=%+.3f  Z=%+.3f  (m/s)\n",
               velocity_world.x(), velocity_world.y(), velocity_world.z());
        printf("【VIO机体系速度】  X=%+.3f  Y=%+.3f  Z=%+.3f  (m/s)\n",
               velocity_body.x(), velocity_body.y(), velocity_body.z());
        printf("【IMU角速度】     X=%+.3f  Y=%+.3f  Z=%+.3f  (rad/s)\n",
               gyro.x(), gyro.y(), gyro.z());
        printf("【欧拉角】 Roll=%+.1f  Pitch=%+.1f  Yaw=%+.1f  (度)\n",
               euler(0)*180/M_PI, euler(1)*180/M_PI, yaw*180/M_PI);
        printf("--------------------------------\n");
        printf(">> 判断：哪个分量最大且符号是什么？\n");
        printf("================================\n");
    }

    // === 设置输出速度指令（根据实测坐标系映射）===
    // VIO为相机坐标系(X右 Y下 Z前)，底盘为(X前 Y左 Z上)
    // 实测结论：
    //   向前移动 → 机体系Z+     => 底盘vx(前) = +机体系Z
    //   向左移动 → 机体系X-     => 底盘vy(左) = -机体系X
    //   顺时针转 → IMU角速度Y+  => 底盘wz(逆时针) = -IMU角速度Y
    Eigen::Vector3d raw_command;
    raw_command(0) = velocity_body.z();    // vx: 前 = 相机Z(前)
    raw_command(1) = -velocity_body.x();   // vy: 左 = -相机X(右)

    // 角速度：底盘wz = -相机Y轴角速度
    if (current_imu_data_.timestamp > 0) {
        raw_command(2) = -gyro.y();
    } else {
        raw_command(2) = 0.0;
    }

    // === 死区处理：过滤小噪声 ===
    const double LINEAR_DEADZONE = 0.005;   // 减小线速度死区到 0.005 m/s
    const double ANGULAR_DEADZONE = 0.02;   // 减小角速度死区到 0.02 rad/s

    if (std::abs(raw_command.x()) < LINEAR_DEADZONE) raw_command.x() = 0.0;
    if (std::abs(raw_command.y()) < LINEAR_DEADZONE) raw_command.y() = 0.0;
    if (std::abs(raw_command.z()) < ANGULAR_DEADZONE) raw_command.z() = 0.0;

    // === 放大速度指令便于观测 ===
    const double SPEED_GAIN = 100.0;  // 100倍放大
    raw_command *= SPEED_GAIN;

    // 直接使用速度命令，不使用低通滤波（最快响应）
    velocity_command_ = raw_command;

    // 速度限幅（放大后需要更大的限幅值）
    const double MAX_LINEAR_VEL = 50.0;   // 增大到50 m/s，适应100倍增益
    const double MAX_ANGULAR_VEL = 20.0;  // 增大到20 rad/s

    double linear_speed = std::sqrt(velocity_command_.x() * velocity_command_.x() +
                                    velocity_command_.y() * velocity_command_.y());
    if (linear_speed > MAX_LINEAR_VEL) {
        double scale = MAX_LINEAR_VEL / linear_speed;
        velocity_command_.x() *= scale;
        velocity_command_.y() *= scale;
    }

    velocity_command_.z() = std::clamp(velocity_command_.z(),
                                       -MAX_ANGULAR_VEL,
                                       MAX_ANGULAR_VEL);

    prev_velocity_command_ = velocity_command_;

    return true;
}

Eigen::Vector3d VIOInput::getVelocityCommand() const {
    return velocity_command_;
}

void VIOInput::onVIOPoseUpdate(const swerve_chassis::VIOPoseData& data) {
    // 首次接收数据时打印提示
    static bool first_data = true;
    if (first_data) {
        std::cout << "[VIOInput] 首次接收到VIO Pose数据！" << std::endl;
        first_data = false;
    }

    // 数据验证
    if (enable_data_validation_ && !validateVIOData(data)) {
        std::cerr << "[VIOInput] VIO数据验证失败" << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    current_pose_data_ = data;
    last_update_time_ = data.timestamp;
    has_valid_data_ = true;
}

void VIOInput::onVIOIMUUpdate(const swerve_chassis::VIOIMUData& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_imu_data_ = data;
}

void VIOInput::setMotionMode(MotionMode mode) {
    motion_mode_.store(mode);
    std::cout << "[VIOInput] 运动模式切换为: "
              << (mode == MotionMode::FLAT_GROUND ? "平地模式" : "坡面模式")
              << std::endl;
}

VIOInput::MotionMode VIOInput::getMotionMode() const {
    return motion_mode_.load();
}

Eigen::Vector3d VIOInput::getCurrentPosition() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_pose_data_.TWB.block<3,1>(0,3);
}

Eigen::Vector3d VIOInput::getCurrentOrientation() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    Eigen::Matrix3d R = current_pose_data_.TWB.block<3,3>(0,0);
    return extractEulerAngles(R);
}

void VIOInput::setDataValidation(bool enable) {
    enable_data_validation_.store(enable);
}

double VIOInput::getLastUpdateTime() const {
    return last_update_time_;
}

// ==================== 私有方法 ====================

Eigen::Vector3d VIOInput::transformVelocity(const Eigen::Vector3d& world_vel,
                                            const Eigen::Matrix3d& R_WB) const {
    // 世界系速度转换为机体系速度
    // V_body = R_WB^T * V_world
    return R_WB.transpose() * world_vel;
}

Eigen::Vector3d VIOInput::extractEulerAngles(const Eigen::Matrix3d& R) const {
    // 提取欧拉角（ZYX顺序：Yaw-Pitch-Roll）
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)

    double sy = std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
    bool singular = sy < 1e-6;

    double roll, pitch, yaw;
    if (!singular) {
        roll  = std::atan2(R(2,1), R(2,2));
        pitch = std::atan2(-R(2,0), sy);
        yaw   = std::atan2(R(1,0), R(0,0));
    } else {
        roll  = std::atan2(-R(1,2), R(1,1));
        pitch = std::atan2(-R(2,0), sy);
        yaw   = 0.0;
    }

    return Eigen::Vector3d(roll, pitch, yaw);
}

VIOInput::MotionMode VIOInput::determineMotionMode(double pitch, double roll, double vz) const {
    // 判断是否需要切换到坡面模式
    if (std::abs(pitch) > PITCH_THRESHOLD ||
        std::abs(roll) > ROLL_THRESHOLD ||
        std::abs(vz) > VZ_THRESHOLD) {
        return MotionMode::SLOPE;
    }
    return MotionMode::FLAT_GROUND;
}

bool VIOInput::validateVIOData(const swerve_chassis::VIOPoseData& data) const {
    // 检查时间戳
    if (data.timestamp <= 0) {
        return false;
    }

    // 检查TWB矩阵是否有效
    if (!data.TWB.allFinite()) {
        std::cerr << "[VIOInput] TWB矩阵包含NaN或Inf" << std::endl;
        return false;
    }

    // 检查速度是否有效
    if (!data.velocity.allFinite()) {
        std::cerr << "[VIOInput] 速度数据包含NaN或Inf" << std::endl;
        return false;
    }

    // 检查速度是否异常（>5m/s）
    double speed = data.velocity.norm();
    if (speed > 5.0) {
        std::cerr << "[VIOInput] 速度异常: " << speed << " m/s" << std::endl;
        return false;
    }

    // 检查位置跳变（与上次位置差>10m）
    Eigen::Vector3d current_pos = data.TWB.block<3,1>(0,3);
    Eigen::Vector3d prev_pos = current_pose_data_.TWB.block<3,1>(0,3);
    double pos_diff = (current_pos - prev_pos).norm();
    if (has_valid_data_ && pos_diff > 10.0) {
        std::cerr << "[VIOInput] 位置跳变: " << pos_diff << " m" << std::endl;
        return false;
    }

    return true;
}

Eigen::Vector3d VIOInput::lowPassFilter(const Eigen::Vector3d& raw_vel,
                                        const Eigen::Vector3d& prev_vel,
                                        double alpha) const {
    // 低通滤波：output = alpha * raw + (1-alpha) * prev
    return alpha * raw_vel + (1.0 - alpha) * prev_vel;
}
