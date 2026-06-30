#ifndef VIO_INPUT_H
#define VIO_INPUT_H

#include "input_interface.h"
#include "vio/vio_device.h"
#include <Eigen/Dense>
#include <mutex>
#include <atomic>

/**
 * @brief VIO输入控制器
 *
 * 接收VIO设备的位姿和IMU数据，转换为底盘速度指令
 * 支持平地模式（3DOF: XY + Yaw）和坡面模式（6DOF: XYZ + RPY）
 */
class VIOInput : public InputInterface {
public:
    /**
     * @brief 运动模式
     */
    enum class MotionMode {
        FLAT_GROUND,  // 平地模式：仅控制XY + Yaw
        SLOPE         // 坡面模式：控制XYZ + Roll/Pitch/Yaw
    };

    /**
     * @brief 构造函数
     * @param max_linear_vel 最大线速度限制 (m/s)
     * @param max_angular_vel 最大角速度限制 (rad/s)
     */
    VIOInput(double max_linear_vel = 2.0, double max_angular_vel = 1.0);

    bool update() override;
    Eigen::Vector3d getVelocityCommand() const override;

    /**
     * @brief VIO位姿数据回调
     */
    void onVIOPoseUpdate(const swerve_chassis::VIOPoseData& data);

    /**
     * @brief VIO IMU数据回调
     */
    void onVIOIMUUpdate(const swerve_chassis::VIOIMUData& data);

    /**
     * @brief 设置运动模式
     */
    void setMotionMode(MotionMode mode);

    /**
     * @brief 获取当前运动模式
     */
    MotionMode getMotionMode() const;

    /**
     * @brief 获取当前位置（世界系）
     */
    Eigen::Vector3d getCurrentPosition() const;

    /**
     * @brief 获取当前姿态（欧拉角：roll, pitch, yaw）
     */
    Eigen::Vector3d getCurrentOrientation() const;

    /**
     * @brief 启用/禁用数据验证
     */
    void setDataValidation(bool enable);

    /**
     * @brief 获取最后一次数据更新时间
     */
    double getLastUpdateTime() const;

private:
    /**
     * @brief 坐标系转换：世界系速度 → 机体系速度
     */
    Eigen::Vector3d transformVelocity(const Eigen::Vector3d& world_vel,
                                     const Eigen::Matrix3d& R_WB) const;

    /**
     * @brief 提取欧拉角（ZYX顺序：Yaw-Pitch-Roll）
     */
    Eigen::Vector3d extractEulerAngles(const Eigen::Matrix3d& R) const;

    /**
     * @brief 自动判断运动模式
     */
    MotionMode determineMotionMode(double pitch, double roll, double vz) const;

    /**
     * @brief 数据验证
     */
    bool validateVIOData(const swerve_chassis::VIOPoseData& data) const;

    /**
     * @brief 低通滤波
     */
    Eigen::Vector3d lowPassFilter(const Eigen::Vector3d& raw_vel,
                                  const Eigen::Vector3d& prev_vel,
                                  double alpha) const;

    // 速度限制
    double max_linear_vel_;
    double max_angular_vel_;

    // 当前VIO数据（线程安全）
    mutable std::mutex data_mutex_;
    swerve_chassis::VIOPoseData current_pose_data_;
    swerve_chassis::VIOIMUData current_imu_data_;

    // 输出速度指令
    Eigen::Vector3d velocity_command_;
    Eigen::Vector3d prev_velocity_command_;

    // 运动模式
    std::atomic<MotionMode> motion_mode_;

    // 数据有效性
    std::atomic<bool> has_valid_data_;
    std::atomic<bool> enable_data_validation_;
    double last_update_time_;

    // 低通滤波参数
    static constexpr double FILTER_ALPHA = 0.2;

    // 模式切换阈值
    static constexpr double PITCH_THRESHOLD = 5.0 * M_PI / 180.0;  // 5度
    static constexpr double ROLL_THRESHOLD = 5.0 * M_PI / 180.0;   // 5度
    static constexpr double VZ_THRESHOLD = 0.01;                    // 0.01 m/s
};

#endif // VIO_INPUT_H
