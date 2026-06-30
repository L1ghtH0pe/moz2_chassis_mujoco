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
    // current_steer_angles: 当前实际舵角（从qpos读取），用于舵角优化
    void inverseKinematics(
        double vx, double vy, double wz,
        WheelCommand wheels[3],
        const double* current_steer_angles = nullptr
    );

private:
    // 舵轮位置
    ChassisGeometry::WheelPosition positions_[3];

    // 轮半径
    double wheel_radius_;

    // 上一次的舵角目标（用于低速时保持和方向一致性）
    double last_steer_angles_[3];

    // 上一次是否使用了反向方案（用于保持方向一致性）
    bool last_use_reverse_[3];
};

}  // namespace swerve_chassis

#endif  // SWERVE_KINEMATICS_H
