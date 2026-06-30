#ifndef INPUT_INTERFACE_H
#define INPUT_INTERFACE_H

#include <Eigen/Dense>

/**
 * @brief 输入接口的纯虚基类
 *
 * 所有输入源（键盘、VIO、手柄等）必须实现此接口
 */
class InputInterface {
public:
    virtual ~InputInterface() = default;

    /**
     * @brief 更新输入状态
     * @return true 如果成功获取输入
     */
    virtual bool update() = 0;

    /**
     * @brief 获取目标速度（底盘坐标系）
     * @return 3D 速度向量 [vx, vy, omega_z]
     */
    virtual Eigen::Vector3d getVelocityCommand() const = 0;
};

#endif // INPUT_INTERFACE_H
