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

    // 提取旋转矩阵和平移向量
    Eigen::Matrix3d R_WB = current_pose_data_.TWB.block<3,3>(0,0);
    Eigen::Vector3d position = current_pose_data_.TWB.block<3,1>(0,3);

    // 提取欧拉角
    Eigen::Vector3d euler = extractEulerAngles(R_WB);
    double roll = euler(0);
    double pitch = euler(1);
    double yaw = euler(2);

    // 世界系速度
    Eigen::Vector3d world_velocity = current_pose_data_.velocity;

    // 自动判断运动模式
    MotionMode auto_mode = determineMotionMode(pitch, roll, world_velocity.z());

    // 如果启用自动模式切换
    // motion_mode_.store(auto_mode);  // 可选：自动切换模式

    // 转换为机体系速度
    Eigen::Vector3d chassis_velocity = transformVelocity(world_velocity, R_WB);

    // 根据运动模式设置输出
    Eigen::Vector3d raw_command;
    if (motion_mode_.load() == MotionMode::FLAT_GROUND) {
        // 平地模式：仅输出 vx, vy, wz
        raw_command << chassis_velocity.x(), chassis_velocity.y(), 0.0;

        // 从IMU获取角速度（如果有）
        if (current_imu_data_.timestamp > 0) {
            raw_command(2) = current_imu_data_.gyroscope.z();
        }
    } else {
        // 坡面模式：输出 vx, vy, wz（z速度暂不使用）
        // TODO: 扩展为6自由度控制
        raw_command << chassis_velocity.x(), chassis_velocity.y(), 0.0;

        if (current_imu_data_.timestamp > 0) {
            raw_command(2) = current_imu_data_.gyroscope.z();
        }
    }

    // 低通滤波平滑速度指令
    velocity_command_ = lowPassFilter(raw_command, prev_velocity_command_, FILTER_ALPHA);

    // 速度限幅
    double linear_speed = std::sqrt(velocity_command_.x() * velocity_command_.x() +
                                    velocity_command_.y() * velocity_command_.y());
    if (linear_speed > max_linear_vel_) {
        double scale = max_linear_vel_ / linear_speed;
        velocity_command_.x() *= scale;
        velocity_command_.y() *= scale;
    }

    velocity_command_.z() = std::clamp(velocity_command_.z(),
                                       -max_angular_vel_,
                                       max_angular_vel_);

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
