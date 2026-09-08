# MoveIt 2 robot_description 路径冲突与工作空间隔离排查

## 1. 问题现象

启动自己的 MoveIt 2 后，发现 move_group 加载了另一个工作空间的 URDF，例如：

    /home/cyberbraindualarm/fairino_ws/install/fairino3_v6_moveit2_config/share/fairino3_v6_moveit2_config/config/fairino3_v6_robot.urdf.xacro

同时出现：

    Joint 'gripper_joint' not found in model
    Joint 'rail_2_slider_l' not found in model
    Joint 'rail_2_slider_r' not found in model

或者出现：

    TF_OLD_DATA ignoring data from the past

这通常不是 MoveIt 2 的全局 URDF 缓存，而是以下问题之一：

1. move_group 在旧工作空间已经启动，之后才 source 新工作空间；
2. AMENT_PREFIX_PATH 中旧工作空间优先级更高；
3. launch 文件或 xacro 中写入了旧的绝对路径；
4. URDF 和 SRDF 来自不同工作空间；
5. 同时启动了两套 move_group、robot_state_publisher 或 controller_manager；
6. Gazebo 仿真时间和真实系统时间混用。

## 2. 先确认当前节点加载的文件

注意：以下命令查询的是当前已经运行的 move_group。重新 source 工作空间不会改变已经运行节点的参数。

    ros2 param get /move_group robot_description | grep "xacro from"

检查 SRDF：

    ros2 param get /move_group robot_description_semantic | grep -o "gripper_joint"

如果 URDF 来源显示旧工作空间，例如：

    /home/cyberbraindualarm/fairino_ws/install/...

而你希望使用：

    /home/suneasy/ros2_ws/install/...

说明当前 move_group 不是从你期望的工作空间启动的。

检查当前进程：

    ros2 node list

重点查看：

    /move_group
    /robot_state_publisher
    /controller_manager
    /rviz2

## 3. 关闭旧节点

优先回到启动 MoveIt 的终端，按：

    Ctrl+C

然后检查：

    ros2 node list

如果仍然存在旧节点，不要立即启动第二套 MoveIt。先检查是否还有 Gazebo、RViz 或控制器终端在运行。

理想状态是同一个 ROS Domain 中没有旧的：

    /move_group
    /robot_state_publisher
    /controller_manager
    /rviz2

如果无法确认旧节点来自哪个终端，最稳妥的方法是重启电脑后，从干净终端重新开始。

## 4. 清理当前终端环境

打开新的终端，执行：

    conda deactivate 2>/dev/null || true
    unset AMENT_PREFIX_PATH
    unset COLCON_PREFIX_PATH
    unset CMAKE_PREFIX_PATH
    source /opt/ros/humble/setup.bash

查看当前 overlay：

    echo "$AMENT_PREFIX_PATH" | tr ':' '\n'

此时最好只看到：

    /opt/ros/humble

检查 bashrc 是否自动加载旧工作空间：

    grep -nE 'fairino_ws|ros2_ws|fr3|cyberbraindualarm' ~/.bashrc

如果看到类似：

    source /home/cyberbraindualarm/fairino_ws/install/setup.bash

请将该行注释或删除，避免每次打开终端自动加载旧工作空间。

## 5. 只加载目标工作空间

假设你的目标工作空间是：

    /home/suneasy/ros2_ws

执行：

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash

查看 overlay 顺序：

    echo "$AMENT_PREFIX_PATH" | tr ':' '\n'

目标工作空间应该位于 ROS Humble 前面，例如：

    /home/suneasy/ros2_ws/install/...
    /opt/ros/humble

检查包来源：

    ros2 pkg prefix fairino3_v6_moveit2_config
    ros2 pkg prefix fairino_description

正确结果应为：

    /home/suneasy/ros2_ws/install/...

如果仍然显示：

    /home/cyberbraindualarm/fairino_ws/install/...

说明当前目标工作空间没有提供该包，或者仍然加载了旧 overlay。

## 6. 检查源码和安装目录中的旧路径

在目标工作空间中搜索旧路径：

    rg -n "/home/cyberbraindualarm|fairino_ws" \
      ~/ros2_ws/src \
      ~/ros2_ws/install

### 情况 A：旧路径只存在于 install

说明 install 目录中保存了以前生成的安装结果。重新编译：

    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install \
      --packages-select fairino_description fairino3_v6_moveit2_config
    source install/setup.bash

重新检查：

    ros2 pkg prefix fairino3_v6_moveit2_config

### 情况 B：旧路径存在于 src

说明源码或 launch 文件写死了旧路径。检查：

    rg -n "robot_description|robot_description_semantic|xacro|fairino_description" \
      ~/ros2_ws/src

不要在 launch 或 xacro 中写：

    /home/cyberbraindualarm/fairino_ws/install/...

建议使用：

    get_package_share_directory('fairino3_v6_moveit2_config')

或者：

    FindPackageShare('fairino3_v6_moveit2_config')

这样路径由 ROS 2 当前环境解析，不依赖某台电脑的绝对路径。

## 7. 重新编译目标工作空间

    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
    colcon build --symlink-install \
      --event-handlers console_direct+
    source install/setup.bash

检查：

    ros2 pkg list | grep fairino
    ros2 pkg prefix fairino3_v6_moveit2_config
    ros2 pkg prefix fairino_description

不要在同一个终端中继续 source 旧的：

    /home/cyberbraindualarm/fairino_ws/install/setup.bash

## 8. 保证 URDF 和 SRDF 成对匹配

MoveIt 2 至少需要以下两项：

    robot_description
    robot_description_semantic

它们必须描述同一个机器人。

### 无夹爪配置

URDF 中没有：

    gripper_joint
    rail_2_slider_l
    rail_2_slider_r

对应 SRDF 也不能包含：

    gripper group
    gripper_joint
    rail_2_slider_l
    rail_2_slider_r

### 有 HKV 夹爪配置

URDF 必须包含：

    gripper_joint
    rail_2_slider_l
    rail_2_slider_r
    rail_2_slider_l 的 mimic 关系
    rail_2_slider_r 的 mimic 关系

SRDF 必须包含：

    gripper group
    gripper_joint
    end_effector

如果 URDF 是无夹爪版本，而 SRDF 是带夹爪版本，就会出现：

    Joint 'gripper_joint' not found in model

这不是关节控制器问题，而是 MoveIt 机器人模型不一致。

## 9. 正确启动原始法奥 MoveIt

新终端中执行：

    conda deactivate 2>/dev/null || true
    unset AMENT_PREFIX_PATH
    unset COLCON_PREFIX_PATH
    unset CMAKE_PREFIX_PATH
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash

确认包来源：

    ros2 pkg prefix fairino3_v6_moveit2_config
    ros2 pkg prefix fairino_description

查找可用 launch 文件：

    find ~/ros2_ws/src -path "*/launch/*" -type f

再使用你的实际 launch 文件：

    ros2 launch fairino3_v6_moveit2_config YOUR_LAUNCH_FILE.launch.py

不要在同一 ROS Domain 中同时启动：

    ros2 launch fr3_real_bringup bringup.launch.py

也不要同时运行第二个：

    robot_state_publisher
    move_group
    controller_manager
    rviz2

## 10. 启动后验证 robot_description

另开终端，只 source 目标工作空间：

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash

检查 URDF 来源：

    ros2 param get /move_group robot_description \
      | grep -o "autogenerated by xacro from [^|]*"

检查是否含夹爪：

    ros2 param get /move_group robot_description \
      | grep -o "gripper_joint"

检查 SRDF：

    ros2 param get /move_group robot_description_semantic \
      | grep -o "gripper_joint"

如果使用无夹爪机器人，两个 grep 都不应输出夹爪关节。

如果使用 HKV 夹爪，URDF 和 SRDF 都应该能找到 gripper_joint。

## 11. 排查 TF_OLD_DATA

检查节点：

    ros2 node list

检查 TF 发布者：

    ros2 topic info /tf --verbose
    ros2 topic info /tf_static --verbose

常见原因：

- 两个 robot_state_publisher 同时发布相同 TF；
- 两个 move_group 同时运行；
- Gazebo 时间被重新设置；
- use_sim_time 配置不一致；
- 旧 launch 没有完全退出。

Gazebo 仿真通常使用：

    use_sim_time: true

真实机器人 MoveIt 通常使用：

    use_sim_time: false

不要将 Gazebo 仿真和真机 MoveIt 的时间配置混用。

## 12. fr3-sim 与原始法奥工作空间隔离

fr3-sim 中的真机包名称是：

    fr3_real_bringup

原始法奥 MoveIt 包通常是：

    fairino3_v6_moveit2_config

两者不是同一个包。建议使用不同 ROS Domain：

仿真：

    export ROS_DOMAIN_ID=32

原始法奥 MoveIt：

    export ROS_DOMAIN_ID=40

每个终端都要设置相同的 ROS_DOMAIN_ID。真实机械臂调试时，不要让 Gazebo、双臂演示和真实控制节点处于同一个 ROS Domain。

## 13. 最小可靠启动流程

关闭所有旧节点后，新终端执行：

    conda deactivate 2>/dev/null || true
    unset AMENT_PREFIX_PATH
    unset COLCON_PREFIX_PATH
    unset CMAKE_PREFIX_PATH
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash

确认路径：

    echo "$AMENT_PREFIX_PATH" | tr ':' '\n'
    ros2 pkg prefix fairino3_v6_moveit2_config
    ros2 pkg prefix fairino_description

编译：

    cd ~/ros2_ws
    colcon build --symlink-install
    source install/setup.bash

启动原始 MoveIt：

    ros2 launch fairino3_v6_moveit2_config YOUR_LAUNCH_FILE.launch.py

启动后确认：

    ros2 param get /move_group robot_description \
      | grep -o "autogenerated by xacro from [^|]*"

目标结果必须来自：

    /home/suneasy/ros2_ws/install/...

而不是：

    /home/cyberbraindualarm/fairino_ws/install/...

## 14. 结论

当前问题的关键不是 moveit_configs_utils 自动读取了一个全局缓存，而是：

1. 已经运行的 move_group 不会因重新 source 而改变；
2. 当前 move_group 实际来自旧的 fairino_ws；
3. URDF 与 SRDF 不匹配；
4. 可能存在多个 TF 和 MoveIt 节点；
5. 必须关闭旧节点，在干净终端中只 source 目标工作空间后重新启动。

官方参考：

- ROS 2 Humble：https://docs.ros.org/en/humble/
- MoveIt 2 Humble：https://moveit.picknik.ai/humble/
- ros2_control Humble：https://control.ros.org/humble/
