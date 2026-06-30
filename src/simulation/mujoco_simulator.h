#ifndef MUJOCO_SIMULATOR_H
#define MUJOCO_SIMULATOR_H

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>
#include <string>
#include <memory>

/**
 * @brief MuJoCo 仿真器封装
 *
 * 负责：
 * - 加载和管理 MuJoCo 模型
 * - 窗口和渲染管理
 * - 执行器控制
 * - 仿真步进
 */
class MuJoCoSimulator {
public:
    /**
     * @brief 构造函数
     * @param model_path XML 模型文件路径
     * @param window_width 窗口宽度
     * @param window_height 窗口高度
     */
    MuJoCoSimulator(const std::string& model_path,
                    int window_width = 1200,
                    int window_height = 900);

    ~MuJoCoSimulator();

    // 禁止拷贝
    MuJoCoSimulator(const MuJoCoSimulator&) = delete;
    MuJoCoSimulator& operator=(const MuJoCoSimulator&) = delete;

    /**
     * @brief 初始化仿真器
     * @return true 如果初始化成功
     */
    bool initialize();

    /**
     * @brief 设置轮子速度（rad/s）
     * @param wheel_velocities 4 个轮子的目标速度 [fl, fr, rl, rr]
     */
    void setWheelVelocities(const Eigen::Vector4d& wheel_velocities);

    /**
     * @brief 执行一步仿真
     */
    void step();

    /**
     * @brief 渲染当前帧
     */
    void render();

    /**
     * @brief 检查窗口是否应该关闭
     */
    bool shouldClose() const;

    /**
     * @brief 获取 GLFW 窗口指针
     */
    GLFWwindow* getWindow() const { return window_; }

    /**
     * @brief 获取底盘位姿（世界坐标系）
     * @return [x, y, z, qw, qx, qy, qz]
     */
    Eigen::Matrix<double, 7, 1> getChassispose() const;

    /**
     * @brief 获取底盘速度（底盘坐标系）
     * @return [vx, vy, vz, wx, wy, wz]
     */
    Eigen::Matrix<double, 6, 1> getChassisVelocity() const;

private:
    /**
     * @brief 鼠标按钮回调
     */
    static void mouseButtonCallback(GLFWwindow* window, int button, int act, int mods);

    /**
     * @brief 鼠标移动回调
     */
    static void mouseMoveCallback(GLFWwindow* window, double xpos, double ypos);

    /**
     * @brief 鼠标滚轮回调
     */
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

    std::string model_path_;
    int window_width_;
    int window_height_;

    // MuJoCo 对象
    mjModel* model_;
    mjData* data_;

    // 渲染对象
    GLFWwindow* window_;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;

    // 交互状态
    bool button_left_;
    bool button_middle_;
    bool button_right_;
    double last_mouse_x_;
    double last_mouse_y_;
};

#endif // MUJOCO_SIMULATOR_H
