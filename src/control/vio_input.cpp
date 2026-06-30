#include "vio_input.h"
#include <algorithm>
#include <cmath>
#include <iostream>

VIOInput::VIOInput(double max_linear_vel, double max_angular_vel)
    : max_linear_vel_(max_linear_vel),
      max_angular_vel_(max_angular_vel),
      current_position_(Eigen::Vector3d::Zero()),
      current_orientation_(Eigen::Quaterniond::Identity()),
      target_position_(Eigen::Vector3d::Zero()),
      target_orientation_(Eigen::Quaterniond::Identity()),
      velocity_command_(Eigen::Vector3d::Zero()),
      has_current_pose_(false),
      has_target_pose_(false) {
}

bool VIOInput::update() {
    if (!validateData()) {
        velocity_command_.setZero();
        return false;
    }

    // TODO: 实现完整的轨迹跟踪控制器
    // 当前仅为简化实现，直接计算位置误差

    // 计算位置误差（世界坐标系）
    Eigen::Vector3d position_error = target_position_ - current_position_;

    // 简单的比例控制
    double kp_linear = 1.0;  // 比例增益
    Eigen::Vector3d world_vel = kp_linear * position_error;

    // 转换到底盘坐标系
    Eigen::Vector3d chassis_vel = worldToChassisVelocity(world_vel);

    // 限制速度
    double linear_speed = chassis_vel.head<2>().norm();
    if (linear_speed > max_linear_vel_) {
        chassis_vel.head<2>() *= (max_linear_vel_ / linear_speed);
    }

    // TODO: 实现姿态控制
    // 当前角速度设为 0
    chassis_vel.z() = 0.0;

    velocity_command_ = chassis_vel;
    return true;
}

Eigen::Vector3d VIOInput::getVelocityCommand() const {
    return velocity_command_;
}

void VIOInput::setTargetPose(const Eigen::Vector3d& position,
                              const Eigen::Quaterniond& orientation) {
    target_position_ = position;
    target_orientation_ = orientation.normalized();
    has_target_pose_ = true;
}

void VIOInput::updateCurrentPose(const Eigen::Vector3d& position,
                                  const Eigen::Quaterniond& orientation) {
    current_position_ = position;
    current_orientation_ = orientation.normalized();
    has_current_pose_ = true;
}

bool VIOInput::validateData() const {
    if (!has_current_pose_ || !has_target_pose_) {
        return false;
    }

    // 检查四元数有效性
    if (std::abs(current_orientation_.norm() - 1.0) > 1e-3 ||
        std::abs(target_orientation_.norm() - 1.0) > 1e-3) {
        std::cerr << "警告: 四元数未归一化" << std::endl;
        return false;
    }

    // 检查位置数据是否为 NaN
    if (!current_position_.allFinite() || !target_position_.allFinite()) {
        std::cerr << "警告: 位置数据包含 NaN 或 Inf" << std::endl;
        return false;
    }

    return true;
}

Eigen::Vector3d VIOInput::worldToChassisVelocity(const Eigen::Vector3d& world_vel) const {
    // TODO: 完整实现需要考虑底盘当前姿态
    // 当前简化：假设底盘 Z 轴与世界 Z 轴对齐，仅旋转 yaw 角

    // 提取 yaw 角（绕 Z 轴旋转）
    Eigen::Vector3d euler = current_orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
    double yaw = euler.z();

    // 旋转 XY 平面速度
    Eigen::Vector3d chassis_vel;
    chassis_vel.x() = world_vel.x() * std::cos(yaw) + world_vel.y() * std::sin(yaw);
    chassis_vel.y() = -world_vel.x() * std::sin(yaw) + world_vel.y() * std::cos(yaw);
    chassis_vel.z() = 0.0;  // 角速度暂时为 0

    return chassis_vel;
}
