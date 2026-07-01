#ifndef VIO_DEVICE_H
#define VIO_DEVICE_H

#include <Eigen/Dense>
#include <functional>
#include <string>
#include <memory>

namespace swerve_chassis {

/**
 * @brief VIO位姿数据结构
 */
struct VIOPoseData {
    double timestamp;              // 时间戳（秒）
    Eigen::Matrix4d TWB;          // 世界系到机体系的变换矩阵（4x4）
    Eigen::Vector3d velocity;     // 世界系速度 (m/s)
    Eigen::Vector3d gyro_bias;    // 陀螺仪偏置 (rad/s)
    Eigen::Vector3d accel_bias;   // 加速度计偏置 (m/s²)

    VIOPoseData()
        : timestamp(0.0),
          TWB(Eigen::Matrix4d::Identity()),
          velocity(Eigen::Vector3d::Zero()),
          gyro_bias(Eigen::Vector3d::Zero()),
          accel_bias(Eigen::Vector3d::Zero()) {}
};

/**
 * @brief VIO IMU数据结构
 */
struct VIOIMUData {
    double timestamp;                  // 时间戳（秒）
    Eigen::Vector3d acceleration;     // 加速度 (m/s²)
    Eigen::Vector3d gyroscope;        // 角速度 (rad/s)

    VIOIMUData()
        : timestamp(0.0),
          acceleration(Eigen::Vector3d::Zero()),
          gyroscope(Eigen::Vector3d::Zero()) {}
};

/**
 * @brief VIO设备封装类
 *
 * 封装Carina A1088 VIO设备的初始化、数据流管理和回调
 */
class VIODevice {
public:
    using PoseCallback = std::function<void(const VIOPoseData&)>;
    using IMUCallback = std::function<void(const VIOIMUData&)>;

    VIODevice();
    ~VIODevice();

    // 禁止拷贝
    VIODevice(const VIODevice&) = delete;
    VIODevice& operator=(const VIODevice&) = delete;

    /**
     * @brief 初始化VIO设备
     * @param config_path VIO配置文件路径
     * @param database_path VIO数据库文件路径
     * @return 初始化成功返回true
     */
    bool initialize(const std::string& config_path,
                   const std::string& database_path);

    /**
     * @brief 启动VIO数据流
     * @return 启动成功返回true
     */
    bool start();

    /**
     * @brief 停止VIO数据流
     */
    void stop();

    /**
     * @brief 释放VIO资源
     */
    void release();

    /**
     * @brief 查询设备连接状态
     * @return 设备已连接返回true
     */
    bool isConnected() const;

    /**
     * @brief 设置位姿数据回调
     */
    void setPoseCallback(PoseCallback callback);

    /**
     * @brief 设置IMU数据回调
     */
    void setIMUCallback(IMUCallback callback);

    /**
     * @brief 获取SDK版本信息
     */
    std::string getSDKVersion() const;

    /**
     * @brief 获取设备序列号
     */
    std::string getDeviceSN() const;

private:
    // VIO SDK C接口回调适配器
    static void poseCallbackAdapter(float* pose, double ts, void* user_data);
    static void imuCallbackAdapter(float* imu, double ts, void* user_data);
    static void vsyncCallbackAdapter(double ts, void* user_data);
    static void eventCallbackAdapter(unsigned char event, void* user_data);
    static void hmdStateCallbackAdapter(int state, void* user_data);

    // 解析pose数组到VIOPoseData
    static VIOPoseData parsePoseData(const float* pose, double timestamp);

    // 解析imu数组到VIOIMUData
    static VIOIMUData parseIMUData(const float* imu, double timestamp);

    // 用户回调
    PoseCallback pose_callback_;
    IMUCallback imu_callback_;

    // 配置路径
    std::string config_path_;
    std::string database_path_;

    // 设备状态
    bool initialized_;
    bool started_;
};

} // namespace swerve_chassis

#endif // VIO_DEVICE_H
