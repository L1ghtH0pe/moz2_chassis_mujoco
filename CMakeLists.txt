cmake_minimum_required(VERSION 3.16)
project(swerve_chassis_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 MuJoCo（手动方式）
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
    PATHS /usr/local/mujoco/include /usr/include
    REQUIRED
)
find_library(MUJOCO_LIBRARY mujoco
    PATHS /usr/local/mujoco/lib /usr/lib
    REQUIRED
)

# 查找 GLFW3
find_package(glfw3 REQUIRED)

# 查找 OpenGL
find_package(OpenGL REQUIRED)

# 源文件
set(SOURCES
    src/main.cpp
    src/kinematics/swerve_kinematics.cpp
    src/control/keyboard_input.cpp
    src/control/vio_input.cpp
    src/simulation/mujoco_simulator.cpp
)

# 可执行文件
add_executable(swerve_sim ${SOURCES})

# 链接库
target_link_libraries(swerve_sim
    ${MUJOCO_LIBRARY}
    glfw
    ${OPENGL_LIBRARIES}
)

# 包含目录
target_include_directories(swerve_sim PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${MUJOCO_INCLUDE_DIR}
)

# 编译选项
target_compile_options(swerve_sim PRIVATE
    -Wall -Wextra
)
