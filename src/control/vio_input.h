#ifndef VIO_INPUT_H
#define VIO_INPUT_H

#include "input_interface.h"
#include <Eigen/Dense>
#include <memory>

/**
 * @brief VIO (Visual-Inertial Odometry) 输入接口
 *
 * 接收来自 VIO 系统的位姿和速度估计，转换为底盘速度指令。
 * 当前为预留框架，实际实现需要集成具体 VIO 系统。
 */
class VIOInput : public InputInterface {
public:
    /**
     * @brief 构造函数
     * @param max_linear_vel 最大线速度限制 (m/s)
     * @param max_angular_vel 最大角速度限制 (rad/s)
     */
    VIOInput(double max_linear_vel = 1.0, double max_angular_vel = 1.0);

    bool update() override;
    Eigen::Vector3d getVelocityCommand() const override;

    /**
     * @brief 设置目标位姿（世界坐标系）
     * @param position 目标位置 [x, y, z]
     * @param orientation 目标姿态四元数 [w, x, y, z]
     */
    void setTargetPose(const Eigen::Vector3d& position,
                       const Eigen::Quaterniond& orientation);

    /**
     * @brief 更新当前位姿估计（来自 VIO）
     * @param position 当前位置 [x, y, z]
     * @param orientation 当前姿态四元数 [w, x, y, z]
     */
    void updateCurrentPose(const Eigen::Vector3d& position,
                           const Eigen::Quaterniond& orientation);

private:
    /**
     * @brief 数据验证
     */
    bool validateData() const;

    /**
     * @brief 坐标转换：世界系 -> 底盘系
     * TODO: 实现完整的 SE(3) 转换
     */
    Eigen::Vector3d worldToChassisVelocity(const Eigen::Vector3d& world_vel) const;

    double max_linear_vel_;
    double max_angular_vel_;

    // 位姿数据
    Eigen::Vector3d current_position_;
    Eigen::Quaterniond current_orientation_;
    Eigen::Vector3d target_position_;
    Eigen::Quaterniond target_orientation_;

    // 输出速度指令
    Eigen::Vector3d velocity_command_;

    // 数据有效性标志
    bool has_current_pose_;
    bool has_target_pose_;
};

#endif // VIO_INPUT_H
