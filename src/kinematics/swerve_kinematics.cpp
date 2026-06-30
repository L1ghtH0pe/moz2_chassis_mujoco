#include "swerve_kinematics.h"
#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    WheelCommand wheels[3],
    const double* current_steer_angles
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
            // 正常速度：计算新的舵角和轮速
            double new_angle = std::atan2(viy, vix);
            double new_speed = vi_magnitude / wheel_radius_;

            // 使用当前实际舵角（如果提供），否则使用上次目标角度
            double current_angle = (current_steer_angles != nullptr) ? current_steer_angles[i] : last_steer_angles_[i];

            // 归一化当前舵角到 [-π, π]
            while (current_angle > M_PI) current_angle -= 2 * M_PI;
            while (current_angle < -M_PI) current_angle += 2 * M_PI;

            // 舵角优化：比较两种方案，选择舵角变化更小的
            // 方案1：直接使用新舵角
            double angle_diff1 = new_angle - current_angle;
            // 归一化到 [-π, π]
            while (angle_diff1 > M_PI) angle_diff1 -= 2 * M_PI;
            while (angle_diff1 < -M_PI) angle_diff1 += 2 * M_PI;
            double cost1 = std::abs(angle_diff1);

            // 方案2：舵角+180°，轮速取反
            // 直接计算反向目标角度
            double reverse_target = new_angle + M_PI;
            // 归一化到 [-π, π]
            while (reverse_target > M_PI) reverse_target -= 2 * M_PI;
            while (reverse_target < -M_PI) reverse_target += 2 * M_PI;

            double angle_diff2 = reverse_target - current_angle;
            // 归一化到 [-π, π]
            while (angle_diff2 > M_PI) angle_diff2 -= 2 * M_PI;
            while (angle_diff2 < -M_PI) angle_diff2 += 2 * M_PI;
            double cost2 = std::abs(angle_diff2);

            // 调试输出
            static int debug_counter = 0;
            if (++debug_counter % 50 == 0 && (cost1 > 0.1 || cost2 > 0.1)) {
                printf("[轮%d舵角优化] 当前=%.2f° 新目标=%.2f° | 方案1代价:%.1f° 方案2代价:%.1f°",
                       i+1,
                       current_angle * 180.0 / M_PI,
                       new_angle * 180.0 / M_PI,
                       cost1 * 180.0 / M_PI,
                       cost2 * 180.0 / M_PI);
            }

            // 选择代价更小的方案
            double final_angle, final_speed;
            if (cost2 < cost1) {
                // 方案2更优：使用反向舵角和反向轮速
                final_angle = reverse_target;  // 关键修复：直接使用reverse_target
                final_speed = -new_speed;

                if (debug_counter % 50 == 0 && (cost1 > 0.1 || cost2 > 0.1)) {
                    printf(" -> 选方案2(目标 %.2f° 轮速反向)\n", final_angle * 180.0 / M_PI);
                }
            } else {
                // 方案1更优：直接使用新舵角和轮速
                final_angle = new_angle;
                final_speed = new_speed;

                if (debug_counter % 50 == 0 && (cost1 > 0.1 || cost2 > 0.1)) {
                    printf(" -> 选方案1(目标 %.2f° 轮速正向)\n", final_angle * 180.0 / M_PI);
                }
            }

            wheels[i].steer_angle = final_angle;
            wheels[i].wheel_speed = final_speed;
            last_steer_angles_[i] = final_angle;
        } else {
            // 低速：保持上一次的舵角
            wheels[i].steer_angle = last_steer_angles_[i];
            wheels[i].wheel_speed = vi_magnitude / wheel_radius_;
        }

        // 限幅保护
        const double MAX_WHEEL_SPEED = 100.0;  // rad/s
        if (wheels[i].wheel_speed > MAX_WHEEL_SPEED) {
            wheels[i].wheel_speed = MAX_WHEEL_SPEED;
        }
    }

    // 添加调试输出
    static int counter = 0;
    if (++counter % 50 == 0) {
        printf("=== 运动学输出 ===\n");
        printf("输入速度: vx=%.3f, vy=%.3f, wz=%.3f\n", vx, vy, wz);
        for (int i = 0; i < 3; i++) {
            printf("轮%d: 舵角=%.2f rad (%.1f°), 轮速=%.2f rad/s\n",
                   i+1, wheels[i].steer_angle,
                   wheels[i].steer_angle * 180.0 / M_PI,
                   wheels[i].wheel_speed);
        }
        printf("\n");
    }
}

}  // namespace swerve_chassis
