# 启动脚本使用说明

## 快速开始

### 方式1：交互式菜单（推荐）

```bash
./start.sh
```

**功能**：
- 🎮 图形化菜单选择启动模式
- ✅ 自动检查文件和设备
- 🔧 提供编译和设备检查功能
- 📊 实时显示状态信息

**菜单选项**：
1. 键盘控制模式
2. VIO控制模式
3. VIO控制模式（详细输出）
4. 编译项目
5. 检查VIO设备
0. 退出

---

### 方式2：直接启动脚本

#### 键盘模式
```bash
./start_keyboard_mode.sh
```

**特点**：
- ✅ 自动检查可执行文件和模型
- 📖 显示控制按键说明
- ⚡ 快速启动，无需额外配置

**控制按键**：
- `W/S`: 前进/后退
- `A/D`: 左移/右移
- `Q/E`: 左转/右转
- 鼠标左键: 旋转视角
- 鼠标右键: 平移视角
- 鼠标滚轮: 缩放

---

#### VIO模式
```bash
# 基础模式
./start_vio_mode.sh

# 详细输出模式
./start_vio_mode.sh --verbose
```

**VIO模式启动前检查**：
- ✅ VIO设备连接状态
- ✅ USB串口权限
- ✅ SDK库文件存在
- ✅ 配置文件完整性

**自动处理**：
- 设置 `LD_LIBRARY_PATH`
- 检测设备连接
- 验证配置文件
- 提供错误修复建议

---

### 方式3：手动启动（高级用户）

#### 键盘模式
```bash
./build/swerve_sim models/wheel_base_modified.xml
```

#### VIO模式
```bash
export LD_LIBRARY_PATH=/home/mxz/桌面/Ubuntu20.04/x86_64/lib:$LD_LIBRARY_PATH

./build/swerve_sim models/wheel_base_modified.xml \
    --mode vio \
    --vio-config /home/mxz/桌面/Ubuntu20.04/x86_64/config/carina_config.yaml \
    --vio-database /home/mxz/桌面/Ubuntu20.04/x86_64/database/carina_vio.db \
    --verbose
```

---

## 脚本功能详解

### 1. start.sh（主菜单）

**功能列表**：
- 🎨 彩色交互式界面
- 📋 模式选择菜单
- 🔍 VIO设备检测
- 🛠️ 一键编译
- ♻️ 循环菜单，方便多次测试

**使用场景**：
- 日常开发测试
- 多模式切换
- 新手友好

---

### 2. start_keyboard_mode.sh（键盘模式）

**自动检查项**：
- ✅ 可执行文件 (`build/swerve_sim`)
- ✅ 模型文件 (`models/wheel_base_modified.xml`)

**参数选项**：
```bash
./start_keyboard_mode.sh [选项]

选项:
  -m, --model PATH    指定模型文件路径
  -h, --help          显示帮助信息
```

**示例**：
```bash
# 使用自定义模型
./start_keyboard_mode.sh --model /path/to/custom_model.xml
```

---

### 3. start_vio_mode.sh（VIO模式）

**自动检查项**：
- ✅ 可执行文件
- ✅ 模型文件
- ✅ VIO配置文件
- ✅ VIO数据库
- ✅ VIO SDK库
- ✅ USB设备连接
- ✅ 串口权限

**参数选项**：
```bash
./start_vio_mode.sh [选项]

选项:
  -v, --verbose       启用详细输出
  -c, --config PATH   指定VIO配置文件
  -d, --database PATH 指定VIO数据库
  -m, --model PATH    指定模型文件
  -h, --help          显示帮助信息
```

**示例**：
```bash
# 使用自定义配置
./start_vio_mode.sh \
    --config /path/to/custom_config.yaml \
    --database /path/to/custom_db.db \
    --verbose
```

**错误处理**：
- 🔴 设备未连接：提示检查USB
- 🔴 权限不足：提供 `chmod` 命令
- 🔴 文件缺失：显示具体缺失文件
- 🟡 警告可跳过：询问是否继续

---

## 常见问题解决

### 问题1：权限不足

**错误信息**：
```
[错误] /dev/ttyUSB0 权限不足
```

**解决方法**：
```bash
sudo chmod 666 /dev/ttyUSB*
```

或永久解决：
```bash
sudo usermod -a -G dialout $USER
# 注销后重新登录生效
```

---

### 问题2：找不到VIO库

**错误信息**：
```
error while loading shared libraries: libcarina_vio.so
```

**解决方法1**：使用启动脚本（自动设置）
```bash
./start_vio_mode.sh
```

**解决方法2**：手动设置环境变量
```bash
export LD_LIBRARY_PATH=/home/mxz/桌面/Ubuntu20.04/x86_64/lib:$LD_LIBRARY_PATH
```

**永久解决**：添加到 `~/.bashrc`
```bash
echo 'export LD_LIBRARY_PATH=/home/mxz/桌面/Ubuntu20.04/x86_64/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

---

### 问题3：VIO设备未找到

**检查步骤**：

1. **检查USB连接**
   ```bash
   lsusb | grep -i carina
   ```

2. **检查串口设备**
   ```bash
   ls -l /dev/ttyUSB*
   ```

3. **测试VIO SDK示例**
   ```bash
   cd /home/mxz/桌面/Ubuntu20.04/x86_64/cpp_example
   ./run_a1088_demo
   ```

4. **使用菜单检查**
   ```bash
   ./start.sh
   # 选择 "5) 检查VIO设备"
   ```

---

### 问题4：编译失败

**解决步骤**：

1. **使用菜单编译**
   ```bash
   ./start.sh
   # 选择 "4) 编译项目"
   ```

2. **手动编译**
   ```bash
   cd build
   cmake ..
   make -j4
   ```

3. **清理重新编译**
   ```bash
   rm -rf build
   mkdir build
   cd build
   cmake ..
   make -j4
   ```

---

## 目录结构

```
moz2_chassis_mujoco/
├── start.sh                   # 主启动菜单
├── start_keyboard_mode.sh     # 键盘模式启动脚本
├── start_vio_mode.sh          # VIO模式启动脚本
├── build/
│   └── swerve_sim            # 可执行文件
├── models/
│   └── wheel_base_modified.xml  # 模型文件
└── /home/mxz/桌面/Ubuntu20.04/x86_64/  # VIO SDK
    ├── lib/
    │   └── libcarina_vio.so
    ├── config/
    │   └── carina_config.yaml
    └── database/
        └── carina_vio.db
```

---

## 脚本特性

### 彩色输出
- 🔵 蓝色：标题和信息
- 🟢 绿色：成功状态
- 🟡 黄色：警告和提示
- 🔴 红色：错误信息

### 智能检测
- ✅ 自动检测缺失文件
- ✅ 自动检测VIO设备状态
- ✅ 自动检测串口权限
- ✅ 提供修复建议

### 用户友好
- 📖 清晰的使用说明
- ⚠️ 详细的错误提示
- ❓ 交互式确认
- 🔄 可重复运行

---

## 推荐工作流程

### 日常开发
1. 运行 `./start.sh`
2. 选择 "1) 键盘控制模式" 快速测试
3. 修改代码后选择 "4) 编译项目"
4. 重新测试

### VIO测试
1. 连接VIO设备
2. 运行 `./start.sh`
3. 选择 "5) 检查VIO设备" 确认连接
4. 选择 "3) VIO控制模式（详细输出）" 启动
5. 观察终端输出的VIO数据

### 快速启动
```bash
# 键盘模式
./start_keyboard_mode.sh

# VIO模式
./start_vio_mode.sh -v
```

---

## 脚本维护

### 修改默认路径

编辑脚本开头的变量：

**start_vio_mode.sh**：
```bash
PROJECT_DIR="/home/mxz/桌面/moz2_chassis_mujoco"
VIO_SDK_DIR="/home/mxz/桌面/Ubuntu20.04/x86_64"
```

**start_keyboard_mode.sh**：
```bash
PROJECT_DIR="/home/mxz/桌面/moz2_chassis_mujoco"
```

### 添加新功能

在主菜单脚本中添加新选项：

```bash
# start.sh
print_menu() {
    # ... 现有菜单 ...
    echo -e "  ${GREEN}6${NC}) 你的新功能"
    echo ""
}

# 添加对应的处理函数
your_new_function() {
    # 实现代码
}

# 在主循环中添加case
case $choice in
    # ... 现有选项 ...
    6)
        your_new_function
        ;;
esac
```

---

## 相关文档

- [VIO集成使用指南](VIO_INTEGRATION_GUIDE.md)
- [VIO测试指南](VIO_TESTING_GUIDE.md)
- [项目总结](VIO_PROJECT_SUMMARY.md)

---

## 更新日志

### v1.0.0 (2026-06-30)
- ✨ 初始版本
- ✨ 添加交互式主菜单
- ✨ 添加键盘模式启动脚本
- ✨ 添加VIO模式启动脚本
- ✨ 自动设备检测
- ✨ 彩色输出
- ✨ 错误提示和修复建议

---

**开始使用**：
```bash
./start.sh
```

**享受仿真！** 🎉
