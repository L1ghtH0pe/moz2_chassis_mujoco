#include "mujoco_simulator.h"
#include <iostream>
#include <stdexcept>

MuJoCoSimulator::MuJoCoSimulator(const std::string& model_path,
                                 int window_width,
                                 int window_height)
    : model_path_(model_path),
      window_width_(window_width),
      window_height_(window_height),
      model_(nullptr),
      data_(nullptr),
      window_(nullptr),
      button_left_(false),
      button_middle_(false),
      button_right_(false),
      last_mouse_x_(0.0),
      last_mouse_y_(0.0) {
}

MuJoCoSimulator::~MuJoCoSimulator() {
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
    if (window_) {
        mjr_freeContext(&con_);
        glfwDestroyWindow(window_);
        glfwTerminate();
    }
    mjv_freeScene(&scn_);
}

bool MuJoCoSimulator::initialize() {
    // 加载模型
    char error[1000] = "无法加载模型";
    model_ = mj_loadXML(model_path_.c_str(), nullptr, error, 1000);
    if (!model_) {
        std::cerr << "错误: " << error << std::endl;
        return false;
    }

    data_ = mj_makeData(model_);
    if (!data_) {
        std::cerr << "错误: 无法创建 mjData" << std::endl;
        return false;
    }

    // 初始化 GLFW
    if (!glfwInit()) {
        std::cerr << "错误: 无法初始化 GLFW" << std::endl;
        return false;
    }

    // 创建窗口
    window_ = glfwCreateWindow(window_width_, window_height_,
                                "MuJoCo 仿真", nullptr, nullptr);
    if (!window_) {
        std::cerr << "错误: 无法创建窗口" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    // 初始化可视化
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);

    mjv_makeScene(model_, &scn_, 2000);
    mjr_makeContext(model_, &con_, mjFONTSCALE_150);

    // 设置相机位置
    cam_.azimuth = 90.0;
    cam_.elevation = -20.0;
    cam_.distance = 3.0;
    cam_.lookat[0] = 0.0;
    cam_.lookat[1] = 0.0;
    cam_.lookat[2] = 0.0;

    // 设置鼠标回调
    glfwSetWindowUserPointer(window_, this);
    glfwSetMouseButtonCallback(window_, mouseButtonCallback);
    glfwSetCursorPosCallback(window_, mouseMoveCallback);
    glfwSetScrollCallback(window_, scrollCallback);

    return true;
}

void MuJoCoSimulator::setWheelVelocities(const Eigen::Vector4d& wheel_velocities) {
    if (!model_ || !data_) return;

    // 查找执行器索引（假设执行器名称为 wheel_fl, wheel_fr, wheel_rl, wheel_rr）
    const char* actuator_names[] = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};

    for (int i = 0; i < 4; ++i) {
        int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, actuator_names[i]);
        if (actuator_id >= 0) {
            data_->ctrl[actuator_id] = wheel_velocities(i);
        }
    }
}

void MuJoCoSimulator::step() {
    if (!model_ || !data_) return;
    mj_step(model_, data_);
}

void MuJoCoSimulator::render() {
    if (!window_ || !model_ || !data_) return;

    // 获取窗口尺寸
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    mjrRect viewport = {0, 0, width, height};

    // 更新场景
    mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);

    // 渲染
    mjr_render(viewport, &scn_, &con_);

    // 交换缓冲区
    glfwSwapBuffers(window_);
    glfwPollEvents();
}

bool MuJoCoSimulator::shouldClose() const {
    return window_ ? glfwWindowShouldClose(window_) : true;
}

Eigen::Matrix<double, 7, 1> MuJoCoSimulator::getChassispose() const {
    Eigen::Matrix<double, 7, 1> pose;
    if (!model_ || !data_) {
        pose.setZero();
        pose(3) = 1.0; // qw = 1
        return pose;
    }

    // 查找底盘 body
    int body_id = mj_name2id(model_, mjOBJ_BODY, "base_link");
    if (body_id < 0) {
        std::cerr << "警告: 未找到 base_link" << std::endl;
        pose.setZero();
        pose(3) = 1.0;
        return pose;
    }

    // 位置
    pose.segment<3>(0) = Eigen::Map<const Eigen::Vector3d>(data_->xpos + 3 * body_id);

    // 四元数 [w, x, y, z]
    pose.segment<4>(3) = Eigen::Map<const Eigen::Vector4d>(data_->xquat + 4 * body_id);

    return pose;
}

Eigen::Matrix<double, 6, 1> MuJoCoSimulator::getChassisVelocity() const {
    Eigen::Matrix<double, 6, 1> velocity;
    if (!model_ || !data_) {
        velocity.setZero();
        return velocity;
    }

    int body_id = mj_name2id(model_, mjOBJ_BODY, "base_link");
    if (body_id < 0) {
        velocity.setZero();
        return velocity;
    }

    // 线速度和角速度（世界坐标系）
    Eigen::Vector3d linear_vel = Eigen::Map<const Eigen::Vector3d>(data_->cvel + 6 * body_id + 3);
    Eigen::Vector3d angular_vel = Eigen::Map<const Eigen::Vector3d>(data_->cvel + 6 * body_id);

    // TODO: 转换到底盘坐标系
    // 当前简化：直接返回世界坐标系速度
    velocity.head<3>() = linear_vel;
    velocity.tail<3>() = angular_vel;

    return velocity;
}

void MuJoCoSimulator::mouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
    MuJoCoSimulator* sim = static_cast<MuJoCoSimulator*>(glfwGetWindowUserPointer(window));
    if (!sim) return;

    sim->button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    sim->button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    sim->button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &sim->last_mouse_x_, &sim->last_mouse_y_);
}

void MuJoCoSimulator::mouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
    MuJoCoSimulator* sim = static_cast<MuJoCoSimulator*>(glfwGetWindowUserPointer(window));
    if (!sim) return;

    double dx = xpos - sim->last_mouse_x_;
    double dy = ypos - sim->last_mouse_y_;
    sim->last_mouse_x_ = xpos;
    sim->last_mouse_y_ = ypos;

    if (sim->button_left_) {
        mjv_moveCamera(sim->model_, mjMOUSE_ROTATE_V, dx * 0.01, dy * 0.01, &sim->scn_, &sim->cam_);
    } else if (sim->button_right_) {
        mjv_moveCamera(sim->model_, mjMOUSE_MOVE_V, dx * 0.01, dy * 0.01, &sim->scn_, &sim->cam_);
    }
}

void MuJoCoSimulator::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    MuJoCoSimulator* sim = static_cast<MuJoCoSimulator*>(glfwGetWindowUserPointer(window));
    if (!sim) return;

    mjv_moveCamera(sim->model_, mjMOUSE_ZOOM, 0, -yoffset * 0.05, &sim->scn_, &sim->cam_);
}
