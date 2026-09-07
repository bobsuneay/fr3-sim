# FR3 真机调试步骤

## 1. 你必须先完成的事项

你需要从 WebApp 提供 FR3 型号/软件版本、控制柜 IP、已安装驱动包和版本。
现场还必须测量桌面高度、基座 x/y/yaw、法兰到夹爪的实际安装变换、TCP、
夹爪/转接件质量与质心，并在控制柜中设置正确的正装姿态、工具参数和负载。
急停、保护停止、低速限制、工作空间和线缆固定必须由现场人员验证。
没有这些信息只能运行 mock。

## 2. Ubuntu 构建与 mock

    source /opt/ros/humble/setup.bash
    cd ~/fr3-sim/sim_ws
    sudo apt update
    sudo apt install -y python3-colcon-common-extensions python3-yaml ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-rviz2 ros-humble-robot-state-publisher
    rosdep install --from-paths src/fr3_real_bringup --ignore-src -r -y
    colcon build --symlink-install --packages-select fr3_real_bringup
    source install/setup.bash
    ros2 launch fr3_real_bringup mock.launch.py

另一个终端检查：

    source /opt/ros/humble/setup.bash
    source ~/fr3-sim/sim_ws/install/setup.bash
    ros2 control list_controllers
    ros2 control list_hardware_interfaces
    ros2 run fr3_real_bringup check_feedback --seconds 5

确认 RViz 中只有桌子、正装 FR3、夹爪，规划组为 fairino3_v6_group，先 Plan 再 Execute。

## 3. 校准模型

编辑 config/cell.yaml。坐标原点在地面、桌面中心，+Z 向上；table.height 是桌面上表面高度，
base.x/y/yaw 是基座在桌面上的位置和朝向。工具链是：

    wrist3_link -> tool0 -> gripper_palm -> gripper_tcp

文件中的法兰/TCP 数值是上一版模型占位值，不是实测标定值。实体转接板、手指、电缆超出碰撞盒时，
必须增大 tool.collision_size 或增加真实碰撞几何。工具质量和质心在控制柜另行配置。

## 4. 选择并审核法奥驱动

参考用户给出的教程、法奥官方 ROS2 仓库和官方 ROS2 指南。不要同时 source 多个
fairino_hardware_v3_* 版本。选定版本后：

    source /opt/ros/humble/setup.bash
    ros2 pkg list | rg '^fairino'
    ros2 pkg prefix --share YOUR_DRIVER_PACKAGE
    rg -n 'FairinoHardwareInterface|CONTROLLER_IP_ADDRESS|ServoJ|GetActualJointPosDegree' "$(ros2 pkg prefix --share YOUR_DRIVER_PACKAGE)"

部分版本把 IP 写在头文件，所以本包不会假装 robot_ip 参数一定有效。确认 SDK 架构：

    uname -m
    file path/to/libfairino*.so
    ldd path/to/libfairino*.so

## 5. 填写真机门槛配置

    mkdir -p ~/fr3_config
    cp -n ~/fr3-sim/sim_ws/src/fr3_real_bringup/config/real.example.yaml ~/fr3_config/real.yaml
    nano ~/fr3_config/real.yaml

填入真实固件、驱动包、控制柜 IP 和驱动源码中核对过的 IP，确认控制周期。
完成现场安全、TCP/负载、驱动源码审核后，才把三个 checks 改为 true。

## 6. 真机连接但不执行 MoveIt 轨迹

先配置专用有线网卡并由你确认同网段，不要覆盖正在使用的远程连接网卡：

    ip -br addr
    ip route
    ping -c 3 ROBOT_IP

停止其它 SDK/ROS 控制程序，然后：

    source /opt/ros/humble/setup.bash
    source ~/fr3_driver_ws/install/setup.bash
    source ~/fr3-sim/sim_ws/install/setup.bash
    ros2 launch fr3_real_bringup bringup.launch.py mode:=real confirm_real:=true enable_execution:=false real_config:=$HOME/fr3_config/real.yaml

该模式会启动硬件插件，插件可能在 write() 中持续下发保持指令，所以不是只读模式。
现场必须有人能使用物理停止装置。检查：

    ros2 control list_controllers
    ros2 control list_hardware_components
    ros2 run fr3_real_bringup check_feedback --seconds 5

预期是 JSB active、fairino3_controller inactive，六轴角度与 WebApp 一致。

## 7. 首次执行

完成第 6 步后重新按现场流程启动：

    ros2 launch fr3_real_bringup bringup.launch.py mode:=real confirm_real:=true enable_execution:=true real_config:=$HOME/fr3_config/real.yaml

控制柜设置低速；RViz 保持 0.05 速度/加速度缩放。使用当前反馈作为 Start State，
先 Plan，现场确认净空后，只执行约 1° 的单轴小步动作，每次检查终点误差和控制器状态。
RViz Stop、Ctrl+C、杀进程和断网都不是急停。

## 8. 夹爪后续

本包不启动 ros2_hkv_gripper，也不把第七个关节塞到六轴法奥硬件插件里。
必须先确认串口设备、波特率、从站地址、开合方向、激活行为、实际行程和反馈换算，
再单独启动夹爪驱动；最终整合时使用独立 gripper controller，并更新 URDF/SRDF。
