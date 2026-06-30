#include "swerve_kinematics.h"
#include <cmath>

namespace swerve_chassis {

SwerveKinematics::SwerveKinematics()
    : wheel_radius_(ChassisGeometry::WHEEL_RADIUS) {

    // 初始化三个轮子的位置
    positions_[0] = ChassisGeometry::WHEEL_POSITIONS[0];
    positions_[1] = ChassisGeometry::WHEEL_POSITIONS[1];
    positions_[2] = ChassisGeometry::WHEEL_POSITIONS[2];

    // 初始化上一次舵角
    for (int i = 0; i < 3; ++i) {
        last_steer_angles_[i] = 0.0;
    }
}

void SwerveKinematics::inverseKinematics(
    double vx, double vy, double wz,
    WheelCommand wheels[3]
) {
    const double LOW_SPEED_THRESHOLD = 0.01;  // m/s

    // 计算底盘速度的模
    double v_chassis = std::sqrt(vx * vx + vy * vy);

    for (int i = 0; i < 3; ++i) {
        // 轮子位置
        double rx = positions_[i].x;
        double ry = positions_[i].y;

        // 计算轮心速度：vi = v_chassis + wz × ri
        // wz × ri = wz × (rx, ry, 0) = (-wz*ry, wz*rx, 0)
        double vix = vx - wz * ry;
        double viy = vy + wz * rx;

        // 计算轮心速度的模
        double vi_magnitude = std::sqrt(vix * vix + viy * viy);

        // 计算舵角和轮速
        if (vi_magnitude > LOW_SPEED_THRESHOLD) {
            // 正常速度：计算新的舵角
            wheels[i].steer_angle = std::atan2(viy, vix);
            last_steer_angles_[i] = wheels[i].steer_angle;
        } else {
            // 低速：保持上一次的舵角
            wheels[i].steer_angle = last_steer_angles_[i];
        }

        // 计算轮速 (rad/s) = 线速度 / 轮半径
        wheels[i].wheel_speed = vi_magnitude / wheel_radius_;

        // 限幅保护
        const double MAX_WHEEL_SPEED = 100.0;  // rad/s
        if (wheels[i].wheel_speed > MAX_WHEEL_SPEED) {
            wheels[i].wheel_speed = MAX_WHEEL_SPEED;
        }
    }
}

}  // namespace swerve_chassis
