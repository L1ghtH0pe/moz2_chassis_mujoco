#include "vio/vio_device.h"
#include "carina_a1088.h"
#include <iostream>
#include <fstream>
#include <sstream>

namespace swerve_chassis {

VIODevice::VIODevice()
    : initialized_(false),
      started_(false) {
}

VIODevice::~VIODevice() {
    if (started_) {
        stop();
    }
    if (initialized_) {
        release();
    }
}

bool VIODevice::initialize(const std::string& config_path,
                          const std::string& database_path) {
    if (initialized_) {
        std::cerr << "[VIO] 设备已初始化" << std::endl;
        return true;
    }

    config_path_ = config_path;
    database_path_ = database_path;

    // 读取配置文件
    std::string config_content;
    std::ifstream config_file(config_path);
    if (config_file.is_open()) {
        std::stringstream buffer;
        buffer << config_file.rdbuf();
        config_content = buffer.str();
        config_file.close();
    } else {
        std::cerr << "[VIO] 无法打开配置文件: " << config_path << std::endl;
        return false;
    }

    // 检查设备连接
    std::cout << "[VIO] 检查设备连接状态..." << std::endl;
    if (!carina_a1088_is_device_connect()) {
        std::cerr << "[VIO] 设备未连接" << std::endl;
        return false;
    }

    // 初始化VIO SDK
    std::cout << "[VIO] 初始化VIO SDK..." << std::endl;
    char* db_path = const_cast<char*>(database_path_.c_str());
    carina_a1088_init(const_cast<char*>(config_content.c_str()), db_path);

    initialized_ = true;
    std::cout << "[VIO] VIO设备初始化成功" << std::endl;

    // 打印设备信息
    std::cout << "[VIO] SDK版本: " << getSDKVersion() << std::endl;
    std::cout << "[VIO] 设备序列号: " << getDeviceSN() << std::endl;

    return true;
}

bool VIODevice::start() {
    if (!initialized_) {
        std::cerr << "[VIO] 设备未初始化，无法启动" << std::endl;
        return false;
    }

    if (started_) {
        std::cerr << "[VIO] 设备已启动" << std::endl;
        return true;
    }

    std::cout << "[VIO] 启动VIO数据流..." << std::endl;

    // 启动VIO数据流，注册回调
    carina_a1088_start(
        poseCallbackAdapter,    // pose回调
        vsyncCallbackAdapter,   // vsync回调
        imuCallbackAdapter,     // imu回调
        nullptr,                // camera回调（不需要）
        nullptr,                // points回调（不需要）
        eventCallbackAdapter,   // event回调
        this                    // user_data（传递this指针）
    );

    started_ = true;
    std::cout << "[VIO] VIO数据流启动成功" << std::endl;

    return true;
}

void VIODevice::stop() {
    if (!started_) {
        return;
    }

    std::cout << "[VIO] 停止VIO数据流..." << std::endl;
    carina_a1088_stop();
    started_ = false;
}

void VIODevice::release() {
    if (!initialized_) {
        return;
    }

    std::cout << "[VIO] 释放VIO资源..." << std::endl;
    carina_a1088_release();
    initialized_ = false;
}

bool VIODevice::isConnected() const {
    return carina_a1088_is_device_connect();
}

void VIODevice::setPoseCallback(PoseCallback callback) {
    pose_callback_ = callback;
}

void VIODevice::setIMUCallback(IMUCallback callback) {
    imu_callback_ = callback;
}

std::string VIODevice::getSDKVersion() const {
    if (!initialized_) {
        return "未初始化";
    }
    return carina_a1088_get_sdk_version();
}

std::string VIODevice::getDeviceSN() const {
    if (!initialized_) {
        return "未初始化";
    }
    return carina_a1088_get_sn();
}

// ==================== 静态回调适配器 ====================

void VIODevice::poseCallbackAdapter(float* pose, double ts, void* user_data) {
    if (!user_data || !pose) {
        return;
    }

    VIODevice* device = static_cast<VIODevice*>(user_data);
    if (!device->pose_callback_) {
        return;
    }

    // 解析pose数据
    VIOPoseData data = parsePoseData(pose, ts);

    // 调用用户回调
    device->pose_callback_(data);
}

void VIODevice::imuCallbackAdapter(float* imu, double ts, void* user_data) {
    if (!user_data || !imu) {
        return;
    }

    VIODevice* device = static_cast<VIODevice*>(user_data);
    if (!device->imu_callback_) {
        return;
    }

    // 解析IMU数据
    VIOIMUData data = parseIMUData(imu, ts);

    // 调用用户回调
    device->imu_callback_(data);
}

void VIODevice::vsyncCallbackAdapter(double ts, void* user_data) {
    // VSync回调暂时不处理
    (void)ts;
    (void)user_data;
}

void VIODevice::eventCallbackAdapter(unsigned char event, void* user_data) {
    // 事件回调：打印事件信息
    (void)user_data;
    std::cout << "[VIO] 事件: " << static_cast<int>(event) << std::endl;
}

// ==================== 数据解析 ====================

VIOPoseData VIODevice::parsePoseData(const float* pose, double timestamp) {
    VIOPoseData data;
    data.timestamp = timestamp;

    // 提取TWB矩阵（列主序，pose[0-15]）
    for (int col = 0; col < 4; col++) {
        for (int row = 0; row < 4; row++) {
            data.TWB(row, col) = pose[col * 4 + row];
        }
    }

    // 提取世界系速度（pose[16-18]）
    data.velocity << pose[16], pose[17], pose[18];

    // 提取陀螺仪偏置（pose[22-24]）
    data.gyro_bias << pose[22], pose[23], pose[24];

    // 提取加速度计偏置（pose[25-27]）
    data.accel_bias << pose[25], pose[26], pose[27];

    return data;
}

VIOIMUData VIODevice::parseIMUData(const float* imu, double timestamp) {
    VIOIMUData data;
    data.timestamp = timestamp;

    // 提取加速度（imu[0-2]）
    data.acceleration << imu[0], imu[1], imu[2];

    // 提取角速度（imu[3-5]）
    data.gyroscope << imu[3], imu[4], imu[5];

    return data;
}

} // namespace swerve_chassis
