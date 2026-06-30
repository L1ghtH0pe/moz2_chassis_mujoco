#include "control/keyboard_input.h"
#include "control/mecanum_controller.h"
#include "simulation/mujoco_simulator.h"
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {
    // 检查命令行参数
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <model_xml_path>" << std::endl;
        return 1;
    }

    std::string model_path = argv[1];

    try {
        // 初始化仿真器
        std::cout << "正在加载模型: " << model_path << std::endl;
        MuJoCoSimulator simulator(model_path);
        if (!simulator.initialize()) {
            std::cerr << "错误: 仿真器初始化失败" << std::endl;
            return 1;
        }
        std::cout << "仿真器初始化成功" << std::endl;

        // 创建键盘输入
        KeyboardInput keyboard_input(simulator.getWindow(), 1.0, 1.0, 0.5);

        // 创建麦克纳姆轮控制器
        // 参数：轮距、轴距、轮半径
        MecanumController controller(0.4, 0.5, 0.076);

        std::cout << "控制提示:" << std::endl;
        std::cout << "  W/S: 前进/后退" << std::endl;
        std::cout << "  A/D: 左移/右移" << std::endl;
        std::cout << "  Q/E: 左转/右转" << std::endl;
        std::cout << "  鼠标左键: 旋转视角" << std::endl;
        std::cout << "  鼠标右键: 平移视角" << std::endl;
        std::cout << "  鼠标滚轮: 缩放" << std::endl;

        // 主控制循环
        auto last_time = std::chrono::high_resolution_clock::now();
        while (!simulator.shouldClose()) {
            // 计算时间步长
            auto current_time = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration<double>(current_time - last_time).count();
            last_time = current_time;

            // 更新键盘输入
            keyboard_input.update();
            Eigen::Vector3d vel_cmd = keyboard_input.getVelocityCommand();

            // 逆运动学：底盘速度 -> 轮子速度
            Eigen::Vector4d wheel_vels = controller.computeWheelVelocities(vel_cmd);

            // 设置仿真器执行器
            simulator.setWheelVelocities(wheel_vels);

            // 执行仿真步进
            simulator.step();

            // 渲染
            simulator.render();

            // 控制频率（可选）
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::cout << "仿真结束" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
