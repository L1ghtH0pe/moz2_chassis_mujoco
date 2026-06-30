#include "keyboard_input.h"
#include <algorithm>
#include <cmath>

KeyboardInput::KeyboardInput(GLFWwindow* window,
                             double max_linear_vel,
                             double max_angular_vel,
                             double accel_time)
    : window_(window),
      max_linear_vel_(max_linear_vel),
      max_angular_vel_(max_angular_vel),
      accel_time_(accel_time),
      target_vel_(Eigen::Vector3d::Zero()),
      current_vel_(Eigen::Vector3d::Zero()),
      last_update_time_(glfwGetTime()) {
}

bool KeyboardInput::update() {
    if (!window_) {
        return false;
    }

    // 读取按键状态
    target_vel_.setZero();

    // W/S: 前进/后退 (X 轴)
    if (glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS) {
        target_vel_.x() = max_linear_vel_;
    }
    if (glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS) {
        target_vel_.x() = -max_linear_vel_;
    }

    // A/D: 左移/右移 (Y 轴)
    if (glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS) {
        target_vel_.y() = max_linear_vel_;
    }
    if (glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS) {
        target_vel_.y() = -max_linear_vel_;
    }

    // Q/E: 左转/右转 (Z 轴角速度)
    if (glfwGetKey(window_, GLFW_KEY_Q) == GLFW_PRESS) {
        target_vel_.z() = max_angular_vel_;
    }
    if (glfwGetKey(window_, GLFW_KEY_E) == GLFW_PRESS) {
        target_vel_.z() = -max_angular_vel_;
    }

    // 平滑加速
    double current_time = glfwGetTime();
    double dt = current_time - last_update_time_;
    last_update_time_ = current_time;

    if (dt > 0 && accel_time_ > 0) {
        double alpha = std::min(1.0, dt / accel_time_);
        current_vel_ = current_vel_ + alpha * (target_vel_ - current_vel_);
    } else {
        current_vel_ = target_vel_;
    }

    return true;
}

Eigen::Vector3d KeyboardInput::getVelocityCommand() const {
    return current_vel_;
}
