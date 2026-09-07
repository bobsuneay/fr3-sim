# Ubuntu 22.04 复现 sim_ws/src 三个功能包

本文用于全新 Ubuntu 22.04 电脑，复现本仓库 sim_ws/src 下的三个包：

| 包名 | 用途 | 硬件 |
|---|---|---|
| fairino_dualarm_sim | 双臂协作流程演示、RViz 和视觉模拟 | 不连接真机 |
| fr3_bolt_cell | 侧装 FR3、头部深度相机、桌面螺栓 Gazebo 仿真 | 不连接真机 |
| fr3_real_bringup | 正常水平正装 FR3、HKV TG-9801、MoveIt 2 | mock 不连接；real 会连接 |

说明：第三个包的 real 模式需要额外安装法奥驱动和 HKV 夹爪驱动。首次复现请只做仿真和 mock。

## 1. 系统初始化

确认版本：

    lsb_release -a
    uname -m

本文假定 Ubuntu 22.04、x86_64。若是 ARM，必须先确认法奥 SDK 有 ARM 版本。

安装基础工具：

    sudo apt update
    sudo apt upgrade -y
    sudo apt install -y curl wget git build-essential cmake pkg-config \
      software-properties-common lsb-release gnupg2 ca-certificates \
      python3-pip python3-venv python3-dev python3-yaml python3-numpy \
      python3-pytest python3-colcon-common-extensions python3-rosdep \
      ripgrep

退出 Conda，确保使用系统 Python：

    conda deactivate
    which python3
    python3 --version

which python3 应显示 /usr/bin/python3。若显示 Conda 路径，关闭终端后重新打开。

## 2. 安装 ROS 2 Humble

    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo add-apt-repository universe

添加 ROS 软件源：

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update

安装 ROS、MoveIt、Gazebo Classic 和 ros2_control：

    sudo apt install -y \
      ros-humble-desktop ros-humble-moveit ros-humble-xacro \
      ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
      ros-humble-rviz2 ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
      ros-humble-ros2-control ros-humble-ros2-controllers \
      ros-humble-controller-manager ros-humble-joint-state-broadcaster \
      ros-humble-joint-trajectory-controller \
      ros-humble-gripper-controllers \
      ros-humble-forward-command-controller ros-humble-kdl-parser \
      ros-humble-moveit-kinematics ros-humble-moveit-planners \
      ros-humble-moveit-ros-move-group ros-humble-moveit-ros-visualization

初始化 rosdep：

    sudo rosdep init
    rosdep update

若 sudo rosdep init 提示已经初始化，可忽略该条错误。

每个新终端先执行：

    source /opt/ros/humble/setup.bash

也可以写入 bashrc：

    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
    source ~/.bashrc

检查：

    printenv ROS_DISTRO
    ros2 --version
    gazebo --version

预期 ROS 为 humble，Gazebo 为 Classic 11.x。fr3_bolt_cell 不使用 gz sim。

## 3. 获取仓库

    cd ~
    git clone https://github.com/bobsuneay/fr3-sim.git
    cd ~/fr3-sim
    git pull origin main

确认三个包：

    cd ~/fr3-sim/sim_ws
    test -f src/fairino_dualarm_sim/package.xml
    test -f src/fr3_bolt_cell/package.xml
    test -f src/fr3_real_bringup/package.xml
    find src -maxdepth 1 -mindepth 1 -type d -printf '%f\n'
    source /opt/ros/humble/setup.bash
    colcon list

## 4. 安装依赖和编译

    cd ~/fr3-sim/sim_ws
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
    /usr/bin/colcon build --symlink-install \
      --packages-select fairino_dualarm_sim fr3_bolt_cell fr3_real_bringup \
      --event-handlers console_direct+
    source install/setup.bash

检查安装：

    ros2 pkg prefix fairino_dualarm_sim
    ros2 pkg prefix fr3_bolt_cell
    ros2 pkg prefix fr3_real_bringup
    ros2 pkg executables fr3_bolt_cell
    ros2 pkg executables fr3_real_bringup

源码修改后：

    cd ~/fr3-sim/sim_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select PACKAGE_NAME
    source install/setup.bash

不要复制 Windows 的 .tools、build、install、log。

## 5. 启动 fairino_dualarm_sim

    source /opt/ros/humble/setup.bash
    source ~/fr3-sim/sim_ws/install/setup.bash
    export ROS_DOMAIN_ID=31
    ros2 launch fairino_dualarm_sim sim.launch.py

该包启动双臂 URDF、robot_state_publisher、sim_controller、sim_vision、grasp_demo 和 RViz。
它是双臂算法演示，不是两台真实 FR3 控制器。运行结束按 Ctrl+C。

## 6. 启动 fr3_bolt_cell Gazebo 仿真

先只启动 Gazebo 和控制器：

    source /opt/ros/humble/setup.bash
    source ~/fr3-sim/sim_ws/install/setup.bash
    export ROS_DOMAIN_ID=31
    ros2 launch fr3_bolt_cell gazebo.launch.py moveit:=false rviz:=false gui:=true

另开终端检查：

    source /opt/ros/humble/setup.bash
    source ~/fr3-sim/sim_ws/install/setup.bash
    export ROS_DOMAIN_ID=31
    ros2 control list_controllers
    ros2 control list_hardware_interfaces
    ros2 topic echo /joint_states --once
    ros2 topic hz /clock

检查头部相机：

    ros2 topic list | grep head_camera
    ros2 topic info /head_camera/points --verbose
    ros2 topic echo /head_camera/camera_info --once --qos-reliability best_effort
    ros2 topic hz /head_camera/image_raw

主要相机话题：

    /head_camera/image_raw
    /head_camera/depth/image_raw
    /head_camera/camera_info
    /head_camera/depth/camera_info
    /head_camera/points

关闭上一套 launch 后启动完整 MoveIt：

    ros2 launch fr3_bolt_cell gazebo.launch.py

RViz 中 Fixed Frame 设置为 world，规划组选择 fairino3_v6_group，先 Plan 再 Execute。
夹爪规划组为 gripper。仿真验收：

    ros2 run fr3_bolt_cell check_sim --timeout 60

注意：Gazebo 中的螺栓是仿真模型，不是视觉检测结果，也没有自动缺陷检测算法。

## 7. 启动 fr3_real_bringup mock

该包是水平桌面正装 FR3 + HKV TG-9801。mock 不连接 FR3 控制柜，也不打开夹爪串口。

    source /opt/ros/humble/setup.bash
    source ~/fr3-sim/sim_ws/install/setup.bash
    export ROS_DOMAIN_ID=32
    ros2 launch fr3_real_bringup mock.launch.py

检查：

    ros2 control list_controllers
    ros2 control list_hardware_interfaces
    ros2 action list -t | grep -E 'follow_joint_trajectory|tg9801_gripper_controller/command'
    ros2 run fr3_real_bringup check_feedback --seconds 5

应看到：

    /fairino3_controller/follow_joint_trajectory
    /tg9801_gripper_controller/command

MoveIt 规划组：

- fairino3_v6_group：FR3 六轴；
- gripper：HKV 的 gripper_joint。

HKV 的 gripper_joint 当前命令范围是 0 到 0.1 m，并通过 Humble 的
position_controllers/GripperActionController 提供 GripperCommand action；实际物理净开口需要现场标定。

## 8. 安装 HKV 夹爪驱动

仓库的三个包不包含 ros2_hkv_gripper 完整源码。将用户已有目录复制到 Ubuntu 工作空间：

    mkdir -p ~/hkv_ws/src
    cp -a /path/to/ros2_hkv_gripper ~/hkv_ws/src/
    source /opt/ros/humble/setup.bash
    cd ~/hkv_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install --packages-select ros2_hkv_gripper
    source install/setup.bash

如果包名不同，以 colcon list 输出为准：

    colcon list

检查：

    ros2 pkg prefix ros2_hkv_gripper
    ls -l /dev/serial/by-id/
    groups

如果用户不在 dialout：

    sudo usermod -aG dialout $USER

注销并重新登录后再检查 groups。不要长期使用 chmod 666 作为最终方案。

不要启动 ros2_hkv_gripper 自带的 gripper_control.launch.py；本仓库的 bringup.launch.py 已经把 HKV 加入同一个 controller_manager。

## 9. 安装法奥 FR3 驱动

真机模式需要与控制柜固件匹配的法奥 ROS 2 驱动。把用户已有 frcobot_ros2-main 目录复制到 Ubuntu：

    mkdir -p ~/fr3_driver_ws/src
    cp -a /path/to/frcobot_ros2-main ~/fr3_driver_ws/src/frcobot_ros2
    source /opt/ros/humble/setup.bash
    cd ~/fr3_driver_ws
    colcon list
    git -C src/frcobot_ros2 rev-parse HEAD
    uname -m

先从 FR3 WebApp 系统设置、关于页面记录控制柜软件版本，再选择匹配的 fairino_hardware_v* 包。
不要同时编译并 source 多个历史驱动版本。

检查插件、IP 和 ServoJ：

    rg -n 'FairinoHardwareInterface|CONTROLLER_IP_ADDRESS|ServoJ|GetActualJointPosDegree' \
      src/frcobot_ros2

下面 DRIVER_PACKAGE 只是占位符，不能直接认为 v3_9_9 一定适合你的控制柜：

    export DRIVER_PACKAGE=fairino_hardware_v3_9_9
    rosdep install --from-paths src/frcobot_ros2/$DRIVER_PACKAGE \
      src/frcobot_ros2/fairino_msgs --ignore-src -r -y
    colcon build --symlink-install --packages-up-to $DRIVER_PACKAGE \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash

部分版本把控制柜 IP 写在 C++ 头文件，修改 YAML 不一定会改变真实连接地址。

## 10. 配置真机参数

    mkdir -p ~/fr3_config
    cp -n ~/fr3-sim/sim_ws/src/fr3_real_bringup/config/real.example.yaml \
      ~/fr3_config/real.yaml
    cp -n ~/fr3-sim/sim_ws/src/fr3_real_bringup/config/cell.yaml \
      ~/fr3_config/cell.yaml
    nano ~/fr3_config/real.yaml
    nano ~/fr3_config/cell.yaml

必须由用户现场填写和确认：

- FR3 固件版本；
- 法奥驱动包和 commit；
- 控制柜 IP；
- 驱动源码实际使用的 IP；
- 桌面高度和基座 x/y/yaw；
- 法兰到夹爪、夹爪到 TCP 的实际变换；
- 工具质量、质心和控制柜负载；
- 急停、低速和工作空间检查。

三个检查项完成前不能改为 true：

    driver_reviewed: false
    geometry_tcp_payload_verified: false
    workcell_estop_and_low_speed_verified: false

## 11. 真机启动

每个新终端按 ROS、HKV、法奥、当前工作空间的顺序 source：

    source /opt/ros/humble/setup.bash
    source ~/hkv_ws/install/setup.bash
    source ~/fr3_driver_ws/install/setup.bash
    source ~/fr3-sim/sim_ws/install/setup.bash

检查网络：

    ip -br addr
    ip route
    ping -c 3 ROBOT_IP

第一次只连接并禁止 MoveIt 轨迹执行：

    ros2 launch fr3_real_bringup bringup.launch.py \
      mode:=real confirm_real:=true enable_execution:=false \
      real_config:=$HOME/fr3_config/real.yaml \
      cell:=$HOME/fr3_config/cell.yaml \
      serial_port:=/dev/serial/by-id/YOUR_GRIPPER \
      baud_rate:=1000000 slave_address:=1

检查：

    ros2 control list_controllers
    ros2 control list_hardware_components
    ros2 control list_hardware_interfaces
    ros2 run fr3_real_bringup check_feedback --seconds 5

预期 FR3 六轴反馈与 WebApp 一致，joint_state_broadcaster active，两个运动控制器 inactive。
该步骤不保证驱动完全只读；硬件插件可能连接、激活或发送保持指令。现场必须有人可操作物理停止装置。

确认反馈、姿态、TCP、夹爪方向和安全条件后，重新启动执行模式：

    ros2 launch fr3_real_bringup bringup.launch.py \
      mode:=real confirm_real:=true enable_execution:=true \
      real_config:=$HOME/fr3_config/real.yaml \
      cell:=$HOME/fr3_config/cell.yaml \
      serial_port:=/dev/serial/by-id/YOUR_GRIPPER \
      baud_rate:=1000000 slave_address:=1

首次只执行约 1° 的单轴小步：控制柜低速、RViz 缩放 0.05、先 Plan、现场确认后 Execute。
RViz Stop、Ctrl+C、杀进程和断网都不是急停。

## 12. 常见故障

ROS 命令不存在：

    source /opt/ros/humble/setup.bash

包找不到：

    cd ~/fr3-sim/sim_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash

Gazebo 找不到网格：

    cd ~/fr3-sim/sim_ws
    colcon build --symlink-install --packages-select fr3_bolt_cell
    source install/setup.bash

Python 找不到 numpy：

    sudo apt install -y python3-numpy python3-yaml
    conda deactivate
    which python3

FairinoHardwareInterface 找不到：

    ros2 pkg list | grep fairino
    rg -n 'FairinoHardwareInterface' ~/fr3_driver_ws/src

HKV 串口打不开：

    ls -l /dev/serial/by-id/
    groups

确认用户已加入 dialout，并且没有同时启动 HKV 自带 controller_manager。

## 13. 完成判据

仿真完成：

- 三个包均能被 ros2 pkg prefix 找到；
- fairino_dualarm_sim 能启动双臂 RViz 演示；
- fr3_bolt_cell 能启动 Gazebo、FR3、夹爪、相机和螺栓；
- check_sim 通过；
- fr3_real_bringup mock 能启动 FR3 和 HKV 两个虚拟控制器；
- MoveIt 能分别选择 fairino3_v6_group 和 gripper。

真机完成还必须由现场确认：

- 法奥驱动与控制柜固件匹配；
- IP、正装姿态、TCP、负载和质心正确；
- HKV 串口、开合方向和实际行程正确；
- 六轴反馈与实体姿态一致；
- FR3 和夹爪可在低速完成受控小步；
- 急停和保护停止有效。

## 14. 参考资料

- ROS 2 Humble：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- ros2_control Humble：https://control.ros.org/humble/
- MoveIt 2 Humble：https://moveit.picknik.ai/humble/
- 法奥驱动：https://github.com/FAIR-INNOVATION/frcobot_ros2
- 法奥 ROS 2 指南：https://fairino-doc-zhs.readthedocs.io/latest/ROSGuide/ros2guide.html
- 法奥 MoveIt 参考：https://blog.csdn.net/2301_78767880/article/details/147323555
