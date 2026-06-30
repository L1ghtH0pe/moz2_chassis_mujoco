#ifndef CHASSIS_GEOMETRY_H
#define CHASSIS_GEOMETRY_H

namespace swerve_chassis {

// 底盘几何参数
struct ChassisGeometry {
    // 舵轮位置（机体坐标系，单位：米）
    struct WheelPosition {
        double x;
        double y;
    };

    static inline constexpr WheelPosition WHEEL_POSITIONS[3] = {
        {0.19, -0.19},  // 轮1: 右前
        {0.19,  0.19},  // 轮2: 左前
        {-0.19, 0.0}    // 轮3: 后中
    };

    // 轮半径（单位：米）
    static constexpr double WHEEL_RADIUS = 0.075;

    // 控制参数
    static constexpr double MAX_LINEAR_VELOCITY = 0.5;   // m/s
    static constexpr double MAX_ANGULAR_VELOCITY = 0.5;  // rad/s
    static constexpr double ACCEL_TIME = 0.5;            // s

    // 运动学阈值
    static constexpr double SPEED_THRESHOLD = 0.001;  // m/s
};

}  // namespace swerve_chassis

#endif  // CHASSIS_GEOMETRY_H
