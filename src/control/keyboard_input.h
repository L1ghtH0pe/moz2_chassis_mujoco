#ifndef KEYBOARD_INPUT_H
#define KEYBOARD_INPUT_H

#include "input_interface.h"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>

/**
 * @brief 键盘输入实现
 *
 * 按键映射：
 * - W/S: 前进/后退
 * - A/D: 左移/右移
 * - Q/E: 左转/右转
 */
class KeyboardInput : public InputInterface {
public:
    /**
     * @brief 构造函数
     * @param window GLFW 窗口指针
     * @param max_linear_vel 最大线速度 (m/s)
     * @param max_angular_vel 最大角速度 (rad/s)
     * @param accel_time 加速时间 (s)
     */
    KeyboardInput(GLFWwindow* window,
                  double max_linear_vel = 1.0,
                  double max_angular_vel = 1.0,
                  double accel_time = 0.5);

    bool update() override;
    Eigen::Vector3d getVelocityCommand() const override;

private:
    GLFWwindow* window_;
    double max_linear_vel_;
    double max_angular_vel_;
    double accel_time_;

    Eigen::Vector3d target_vel_;  // 目标速度
    Eigen::Vector3d current_vel_; // 当前速度（平滑后）
    double last_update_time_;
};

#endif // KEYBOARD_INPUT_H
