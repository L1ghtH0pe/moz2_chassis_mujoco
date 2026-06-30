#include "swerve_kinematics.h"
#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace swerve_chassis {

namespace {
// 将角度归一化到 [-π, π]
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}
}  // anonymous namespace

SwerveKinematics::SwerveKinematics()
    : wheel_radius_(ChassisGeometry::WHEEL_RADIUS) {

    // 初始化三个轮子的位置
    positions_[0] = ChassisGeometry::WHEEL_POSITIONS[0];
    positions_[1] = ChassisGeometry::WHEEL_POSITIONS[1];
    positions_[2] = ChassisGeometry::WHEEL_POSITIONS[2];

    // 初始化上一次舵角和反向标志
    for (int i = 0; i < 3; ++i) {
        last_steer_angles_[i] = 0.0;
        last_use_reverse_[i] = false;
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
            current_angle = normalize_angle(current_angle);
            double target_direction = normalize_angle(new_angle);  // 目标方向

            // 计算两种等价方案
            // 方案1：舵角指向目标方向，正向轮速
            double option1_steer = target_direction;
            double option1_speed = new_speed;
            double diff1 = normalize_angle(option1_steer - current_angle);

            // 方案2：舵角指向反方向，反向轮速
            double option2_steer = normalize_angle(target_direction + M_PI);
            double option2_speed = -new_speed;
            double diff2 = normalize_angle(option2_steer - current_angle);

            // 选择转角更小的方案
            double final_angle, final_speed;
            double actual_turn;
            bool use_reverse;

            const double ANGLE_EPSILON = 0.001;  // 角度比较容差
            double abs_diff1 = std::abs(diff1);
            double abs_diff2 = std::abs(diff2);

            if (abs_diff1 < abs_diff2 - ANGLE_EPSILON) {
                // 方案1转角更小
                final_angle = option1_steer;
                final_speed = option1_speed;
                actual_turn = diff1;
                use_reverse = false;
            } else if (abs_diff2 < abs_diff1 - ANGLE_EPSILON) {
                // 方案2转角更小
                final_angle = option2_steer;
                final_speed = option2_speed;
                actual_turn = diff2;
                use_reverse = true;
            } else {
                // 两方案转角相等
                const double MAX_TURN = M_PI / 2.0;  // 90度
                if (std::abs(abs_diff1 - MAX_TURN) < 0.01) {
                    // 都接近90°，保持当前角度+反向轮速
                    final_angle = current_angle;
                    final_speed = -new_speed;
                    actual_turn = 0.0;
                    use_reverse = true;
                } else {
                    // 其他相等情况，优先保持上次的选择
                    if (last_use_reverse_[i]) {
                        final_angle = option2_steer;
                        final_speed = option2_speed;
                        actual_turn = diff2;
                        use_reverse = true;
                    } else {
                        final_angle = option1_steer;
                        final_speed = option1_speed;
                        actual_turn = diff1;
                        use_reverse = false;
                    }
                }
            }

            // 到位判断：如果转角很小，直接保持当前角度，避免微小抖动
            const double POSITION_THRESHOLD = 2.0 * M_PI / 180.0;  // 2度
            if (std::abs(actual_turn) <= POSITION_THRESHOLD) {
                final_angle = current_angle;
            }

            // 调试输出：降低频率，只显示关键信息
            static int debug_counter = 0;
            const double DEBUG_THRESHOLD = 15.0 * M_PI / 180.0;  // 15度
            if (std::abs(actual_turn) > DEBUG_THRESHOLD && (++debug_counter % 50 == 0)) {
                printf("[轮%d] 转角%.1f° %s\n",
                       i+1,
                       std::abs(actual_turn) * 180.0 / M_PI,
                       use_reverse ? "(反向)" : "");
            }

            wheels[i].steer_angle = final_angle;
            wheels[i].wheel_speed = final_speed;
            last_steer_angles_[i] = final_angle;
            last_use_reverse_[i] = use_reverse;  // 保存反向标志
        } else {
            // 低速：保持上一次的舵角和反向标志
            wheels[i].steer_angle = last_steer_angles_[i];
            wheels[i].wheel_speed = vi_magnitude / wheel_radius_;
        }

        // 限幅保护
        const double MAX_WHEEL_SPEED = 100.0;  // rad/s
        if (wheels[i].wheel_speed > MAX_WHEEL_SPEED) {
            wheels[i].wheel_speed = MAX_WHEEL_SPEED;
        }
    }
}

}  // namespace swerve_chassis
