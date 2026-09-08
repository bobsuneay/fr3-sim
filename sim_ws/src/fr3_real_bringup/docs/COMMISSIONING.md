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

### 4.1 你提供的 v3.0.0_robotV3.9.7 压缩包

你提供的目录包含以下几类包：

- `fairino_msgs`：法奥消息/服务接口，必须编译；
- `fairino_hardware`：版本识别/命令服务包，不是当前版本的 ros2_control 硬件插件主体；
- `fairino_hardware_v3_9_7`：当前目录中真正导出
  `fairino_hardware/FairinoHardwareInterface` 的版本化 ros2_control 插件；
- `fairino_description`：官方机器人外观和 URDF；
- `fairino3_v6_moveit2_config`：官方 FR3 MoveIt 2 配置。

如果 FR3 控制柜软件确实是 3.9.7，可把 `fairino_msgs` 和
`fairino_hardware_v3_9_7` 放入一个单独的驱动工作空间：

    mkdir -p ~/fr3_driver_ws/src
    cp -a /path/to/frcobot_ros2-v3.0.0_robotV3.9.7/fairino_msgs ~/fr3_driver_ws/src/
    cp -a /path/to/frcobot_ros2-v3.0.0_robotV3.9.7/fairino_hardware_v3_9_7 ~/fr3_driver_ws/src/
    source /opt/ros/humble/setup.bash
    cd ~/fr3_driver_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install --packages-up-to fairino_hardware_v3_9_7
    source install/setup.bash

`fairino_hardware` 可以额外编译，用于官方版本识别/命令服务，但它不是
`fr3_real_bringup` 加载的硬件插件本体：

    colcon build --symlink-install --packages-select fairino_hardware

不要把 `fairino_hardware_v3_9_0`、`fairino_hardware_v3_9_7`、`fairino_hardware_v3_9_9`
等多个版本同时编译并 source。它们可能导出同名的
`fairino_hardware/FairinoHardwareInterface`，会造成插件歧义或加载到错误版本。

对当前 `fr3_real_bringup`，不需要编译官方 `fairino_description` 和
`fairino3_v6_moveit2_config`，因为本包已经安装自己的 FR3 URDF、网格、SRDF 和 MoveIt 配置。
只有启动官方 `fairino3_v6_moveit2_config` 时，才需要编译并 source 这两个官方包。

确认插件确实来自选定版本：

    ros2 pkg prefix fairino_hardware_v3_9_7
    ros2 pkg prefix fairino_msgs
    rg -n 'FairinoHardwareInterface|CONTROLLER_IP_ADDRESS|ServoJ|GetActualJointPosDegree' \
      ~/fr3_driver_ws/src/fairino_hardware_v3_9_7

该压缩包的 v3.9.7 头文件使用默认控制柜地址 `192.168.58.2`。如果现场 IP 不同，
先按厂商允许的方式修改驱动源码/配置并重新编译；仅修改
`fr3_real_bringup/config/real.yaml` 中的 `controller_ip` 不一定会改变插件实际连接地址。

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

## 8. HKV 夹爪控制

当前包已经将 HKV 接入 MoveIt：规划组 `gripper` 只有 `gripper_joint`，
控制器是 Humble 的 `position_controllers/GripperActionController`，MoveIt action 为
`tg9801_gripper_controller/command`，类型是 `control_msgs/action/GripperCommand`。
夹爪不是 FR3 的第七轴；一个 controller_manager 中同时加载 FR3 和 HKV 两个硬件组件。

先确认夹爪串口、波特率、从站地址、开合方向、激活行为、实际行程和反馈换算：

    ls -l /dev/serial/by-id/
    groups

你提供的 `ros2_hkv_gripper` 目录需要复制到 Ubuntu 工作空间后单独编译一次：

    mkdir -p ~/hkv_ws/src
    cp -a /path/to/ros2_hkv_gripper ~/hkv_ws/src/
    source /opt/ros/humble/setup.bash
    cd ~/hkv_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install --packages-select ros2_hkv_gripper
    source install/setup.bash

如果源目录包名不是 `ros2_hkv_gripper`，以 `colcon list` 显示的实际包名为准。
不要启动它自己的 `gripper_control.launch.py`，因为本包会把夹爪硬件加载到同一个
controller_manager 中；只 source 这个工作空间即可。

真机启动时传入串口参数：

    ros2 launch fr3_real_bringup bringup.launch.py mode:=real confirm_real:=true enable_execution:=true real_config:=$HOME/fr3_config/real.yaml serial_port:=/dev/serial/by-id/YOUR_GRIPPER baud_rate:=1000000 slave_address:=1

启动后检查：

    ros2 control list_controllers
    ros2 control list_hardware_interfaces
    ros2 action list -t | rg 'command'

应该同时看到 `fairino3_controller` 和 `tg9801_gripper_controller`；FR3 使用
FollowJointTrajectory，HKV 使用 GripperCommand。MoveIt 中选择 `gripper` 规划组即可规划开合。

Humble 的专用控制器负责单自由度夹爪的目标位置、容差、停滞判断和 GripperCommand
action；它不是完整的双指力控器。你提供的 HKV 硬件插件只导出 position 命令接口，
`max_effort` 不会自动变成硬件力控；当前夹持力由 `target_force_percent` 参数固定设置。
若需要每次目标动态设置力，必须修改 HKV 硬件插件/控制器接口，不能仅修改 MoveIt YAML。
HKV 文档定义 `gripper_joint` 的 0 到 0.1 m 为开度命令，但实际机械净间距、
寄存器方向和闭合力仍需你现场验证。首次只执行很小的开度变化，确认手指方向和急停。

如果夹爪驱动激活失败，先单独按 ros2_hkv_gripper 的说明验证串口，再启动本包。
不要同时启动该包自带的第二个 controller_manager，否则会争用同一夹爪串口和
重复发布 robot_description。
