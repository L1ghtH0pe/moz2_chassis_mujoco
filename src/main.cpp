#include "control/keyboard_input.h"
#include "control/vio_input.h"
#include "vio/vio_device.h"
#include "kinematics/swerve_kinematics.h"
#include "simulation/mujoco_simulator.h"
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <string>

// 命令行参数解析
struct ProgramArgs {
    std::string model_path;
    std::string mode = "keyboard";  // "keyboard" or "vio"
    std::string vio_config = "";
    std::string vio_database = "";
    bool verbose = false;

    static ProgramArgs parse(int argc, char** argv) {
        ProgramArgs args;

        if (argc < 2) {
            throw std::runtime_error("用法: " + std::string(argv[0]) + " <model_xml_path> [选项]\n"
                                   "选项:\n"
                                   "  --mode <keyboard|vio>    控制模式（默认: keyboard）\n"
                                   "  --vio-config <path>      VIO配置文件路径\n"
                                   "  --vio-database <path>    VIO数据库文件路径\n"
                                   "  --verbose                详细输出\n");
        }

        args.model_path = argv[1];

        for (int i = 2; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--mode" && i + 1 < argc) {
                args.mode = argv[++i];
            } else if (arg == "--vio-config" && i + 1 < argc) {
                args.vio_config = argv[++i];
            } else if (arg == "--vio-database" && i + 1 < argc) {
                args.vio_database = argv[++i];
            } else if (arg == "--verbose") {
                args.verbose = true;
            }
        }

        return args;
    }
};

int main(int argc, char** argv) {
    try {
        // 解析命令行参数
        ProgramArgs args = ProgramArgs::parse(argc, argv);

        std::cout << "========================================" << std::endl;
        std::cout << "三舵轮底盘仿真系统" << std::endl;
        std::cout << "控制模式: " << args.mode << std::endl;
        std::cout << "========================================" << std::endl;

        // 初始化仿真器
        std::cout << "正在加载模型: " << args.model_path << std::endl;
        MuJoCoSimulator simulator(args.model_path);
        if (!simulator.initialize()) {
            std::cerr << "错误: 仿真器初始化失败" << std::endl;
            return 1;
        }
        std::cout << "仿真器初始化成功" << std::endl;

        // 创建舵轮运动学求解器
        swerve_chassis::SwerveKinematics kinematics;

        // 创建输入源
        std::unique_ptr<InputInterface> input;
        std::unique_ptr<swerve_chassis::VIODevice> vio_device;

        if (args.mode == "vio") {
            // ==================== VIO模式 ====================
            std::cout << "\n[VIO模式] 初始化VIO设备..." << std::endl;

            // 检查必要参数
            if (args.vio_config.empty() || args.vio_database.empty()) {
                std::cerr << "错误: VIO模式需要指定 --vio-config 和 --vio-database" << std::endl;
                return 1;
            }

            // 创建VIO设备
            vio_device = std::make_unique<swerve_chassis::VIODevice>();

            // 初始化VIO设备
            if (!vio_device->initialize(args.vio_config, args.vio_database)) {
                std::cerr << "错误: VIO设备初始化失败" << std::endl;
                return 1;
            }

            // 创建VIO输入控制器
            auto vio_input = std::make_unique<VIOInput>(2.0, 1.0);

            // 先转移所有权到input指针
            input = std::move(vio_input);

            // 通过input指针设置VIO回调（避免野指针）
            VIOInput* vio_input_ptr = dynamic_cast<VIOInput*>(input.get());
            vio_device->setPoseCallback([vio_input_ptr](const swerve_chassis::VIOPoseData& data) {
                vio_input_ptr->onVIOPoseUpdate(data);
            });

            vio_device->setIMUCallback([vio_input_ptr](const swerve_chassis::VIOIMUData& data) {
                vio_input_ptr->onVIOIMUUpdate(data);
            });

            // 启动VIO数据流
            if (!vio_device->start()) {
                std::cerr << "错误: VIO数据流启动失败" << std::endl;
                return 1;
            }

            std::cout << "[VIO模式] VIO设备启动成功，等待数据..." << std::endl;

        } else {
            // ==================== 键盘模式 ====================
            std::cout << "\n[键盘模式] 初始化键盘输入..." << std::endl;
            input = std::make_unique<KeyboardInput>(simulator.getWindow(), 2.0, 2.0, 1.0);

            std::cout << "控制提示:" << std::endl;
            std::cout << "  W/S: 前进/后退" << std::endl;
            std::cout << "  A/D: 左移/右移" << std::endl;
            std::cout << "  Q/E: 左转/右转" << std::endl;
            std::cout << "  鼠标左键: 旋转视角" << std::endl;
            std::cout << "  鼠标右键: 平移视角" << std::endl;
            std::cout << "  鼠标滚轮: 缩放" << std::endl;
        }

        std::cout << "\n开始仿真循环..." << std::endl;
        std::cout << "========================================\n" << std::endl;

        // 主仿真循环
        auto last_time = std::chrono::high_resolution_clock::now();
        int frame_count = 0;

        while (!simulator.shouldClose()) {
            auto current_time = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration<double>(current_time - last_time).count();
            last_time = current_time;

            // 更新输入
            if (!input->update()) {
                // 输入更新失败（例如VIO数据丢失），保持停止
                if (args.verbose && frame_count % 100 == 0) {
                    std::cout << "[警告] 输入更新失败，底盘保持停止" << std::endl;
                }
            }

            // 获取速度指令
            Eigen::Vector3d vel_cmd = input->getVelocityCommand();

            // 获取当前实际舵角
            double current_steer_angles[3];
            simulator.getCurrentSteerAngles(current_steer_angles);

            // 逆运动学：底盘速度 -> 轮子指令
            swerve_chassis::WheelCommand wheel_cmds[3];
            kinematics.inverseKinematics(vel_cmd(0), vel_cmd(1), vel_cmd(2),
                                        wheel_cmds, current_steer_angles);

            // 设置执行器指令（舵角 + 轮速）
            simulator.setActuatorCommands(wheel_cmds);

            // 执行仿真步进
            simulator.step();

            // 渲染
            simulator.render();

            // 详细输出（VIO模式）
            if (args.verbose && args.mode == "vio" && frame_count % 100 == 0) {
                auto vio_input_ptr = dynamic_cast<VIOInput*>(input.get());
                if (vio_input_ptr) {
                    Eigen::Vector3d pos = vio_input_ptr->getCurrentPosition();
                    Eigen::Vector3d euler = vio_input_ptr->getCurrentOrientation();

                    std::cout << "[VIO状态]" << std::endl;
                    std::cout << "  位置: (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ") m" << std::endl;
                    std::cout << "  姿态: Roll=" << euler(0)*180/M_PI << "° Pitch=" << euler(1)*180/M_PI
                             << "° Yaw=" << euler(2)*180/M_PI << "°" << std::endl;
                    std::cout << "  速度指令: vx=" << vel_cmd(0) << " vy=" << vel_cmd(1)
                             << " wz=" << vel_cmd(2) << std::endl;
                    std::cout << std::endl;
                }
            }

            frame_count++;
        }

        std::cout << "\n仿真结束" << std::endl;
        std::cout << "总帧数: " << frame_count << std::endl;

        // 清理资源
        if (vio_device) {
            vio_device->stop();
            vio_device->release();
        }

    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
