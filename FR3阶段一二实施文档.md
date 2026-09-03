# 法奥 FR3 阶段一、阶段二实施文档

本文配套回答中的详细步骤。

## 目标

完成 FR3 MoveIt 2 基础接口检查，并接入单臂夹爪。

## 阶段一验收

- fairino3_v6_moveit2_config 可启动。
- RViz 可规划并执行。
- joint_states 正常发布。
- TCP 和规划组名称已记录。
- 仿真与真实硬件插件可区分。

## A. 阶段一详细执行步骤

以下命令假设工作区为 ~/fr3_ws；如果你使用 ~/moveit2_ws，请统一替换路径。

### A.1 安装依赖

    sudo apt update
    sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
    sudo apt install -y ros-humble-xacro ros-humble-robot-state-publisher
    sudo apt install -y ros-humble-tf2-tools ros-humble-ros2-control
    sudo apt install -y ros-humble-ros2-controllers ros-humble-controller-manager
    sudo apt install -y ros-humble-moveit ros-humble-moveit-setup-assistant

验证：

    source /opt/ros/humble/setup.bash
    printenv ROS_DISTRO
    ros2 doctor --report

### A.2 编译法奥 FR3 包

    mkdir -p ~/fr3_ws/src
    cd ~/fr3_ws/src
    git clone https://github.com/FAIR-INNOVATION/frcobot_ros2.git
    cd ~/fr3_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source install/setup.bash

检查：

    ros2 pkg list | grep fairino
    ros2 pkg prefix fairino3_v6_moveit2_config
    ros2 pkg prefix fairino_description
    ros2 pkg prefix fairino_hardware
    ros2 pkg prefix fairino_msgs

如果某个包缺失：

    cd ~/fr3_ws
    colcon list | grep fairino
    colcon build --packages-select fairino_msgs
    colcon build --packages-select fairino_hardware
    colcon build --packages-select fairino_description
    colcon build --packages-select fairino3_v6_moveit2_config
    source install/setup.bash

### A.3 启动和检查 FR3 仿真

终端一：

    source /opt/ros/humble/setup.bash
    source ~/fr3_ws/install/setup.bash
    ros2 launch fairino3_v6_moveit2_config demo.launch.py

终端二：

    source /opt/ros/humble/setup.bash
    source ~/fr3_ws/install/setup.bash
    ros2 control list_controllers
    ros2 control list_hardware_interfaces
    ros2 topic echo /joint_states --once

终端三：

    ros2 node list
    ros2 topic list | sort
    ros2 action list

RViz 验证顺序：

    1. 选择 FR3 实际 Planning Group。
    2. 拖动末端交互球。
    3. 点击 Plan。
    4. 检查轨迹没有穿过桌面或机器人自身。
    5. 点击 Execute。

### A.4 查找真实规划组和 TCP

    FR3_SHARE=$(ros2 pkg prefix fairino3_v6_moveit2_config)/share/fairino3_v6_moveit2_config
    find $FR3_SHARE -maxdepth 2 -type f | sort
    grep -R \"group name\\|end_effector\\|parent_link\" $FR3_SHARE -n
    grep -R \"controller\\|joints:\" $FR3_SHARE/config -n

把查到的内容写入：

    mkdir -p ~/fr3_ws/src/fr3_task/config
    nano ~/fr3_ws/src/fr3_task/config/robot.yaml

示例：

    robot:
      planning_group: REPLACE_WITH_REAL_GROUP
      base_frame: REPLACE_WITH_REAL_BASE_FRAME
      tcp_frame: REPLACE_WITH_REAL_TCP_FRAME
      joints:
        - REPLACE_WITH_JOINT_1
        - REPLACE_WITH_JOINT_2
        - REPLACE_WITH_JOINT_3
        - REPLACE_WITH_JOINT_4
        - REPLACE_WITH_JOINT_5
        - REPLACE_WITH_JOINT_6

REPLACE_WITH_REAL_* 必须全部替换，不能留在正式配置里。

### A.5 检查 TF

    ros2 run tf2_tools view_frames
    ros2 run tf2_ros tf2_echo BASE_FRAME TCP_FRAME

将 BASE_FRAME 和 TCP_FRAME 替换为 robot.yaml 中的实际名称。确认：

    1. TF 持续输出。
    2. 平移单位为米。
    3. TCP 姿态方向与真实工具方向一致。

### A.6 切换真实硬件前的检查

查找硬件插件：

    grep -R \"<plugin>\" ~/fr3_ws/src/fairino* -n

仿真通常使用：

    <plugin>mock_components/GenericSystem</plugin>

真实 FR3 通常使用：

    <plugin>fairino_hardware/FairinoHardwareInterface</plugin>

先备份配置：

    cp ~/fr3_ws/src/fairino3_v6_moveit2_config/config/fairino3_v6_robot.ros2_control.xacro \
       ~/fr3_ws/src/fairino3_v6_moveit2_config/config/fairino3_v6_robot.ros2_control.xacro.bak

确认控制柜网络：

    ping -c 4 ROBOT_IP

真实启动后第一次只观察：

    ros2 topic echo /joint_states --once
    ros2 control list_hardware_interfaces
    ros2 control list_controllers

确认反馈正常、急停有效、机器人处于低速模式后，才执行安全命名姿态。

## B. 阶段二详细执行步骤：单臂夹爪

### B.1 选择夹爪控制方式

如果夹爪有独立位置控制和状态反馈，采用 ros2_control 关节方式。如果夹爪通过法奥 IO、Modbus、TCP 或厂家 SDK 控制，采用独立 ROS 2 节点方式。

### B.2 创建夹爪描述包

    cd ~/fr3_ws/src
    ros2 pkg create fr3_gripper_description --build-type ament_cmake
    mkdir -p fr3_gripper_description/urdf
    mkdir -p fr3_gripper_description/meshes
    mkdir -p fr3_gripper_description/config

创建：

    nano ~/fr3_ws/src/fr3_gripper_description/urdf/gripper.urdf.xacro

最小模型：

    <?xml version=\"1.0\"?>
    <robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"fr3_gripper\">
      <xacro:macro name=\"gripper\" params=\"prefix parent\">
        <link name=\"\${prefix}gripper_base_link\">
          <visual><geometry><box size=\"0.08 0.08 0.10\"/></geometry></visual>
          <collision><geometry><box size=\"0.08 0.08 0.10\"/></geometry></collision>
        </link>
        <joint name=\"\${prefix}gripper_mount_joint\" type=\"fixed\">
          <parent link=\"\${parent}\"/>
          <child link=\"\${prefix}gripper_base_link\"/>
          <origin xyz=\"0 0 0.10\" rpy=\"0 0 0\"/>
        </joint>
        <link name=\"\${prefix}finger_left_link\">
          <visual><geometry><box size=\"0.015 0.04 0.08\"/></geometry></visual>
          <collision><geometry><box size=\"0.015 0.04 0.08\"/></geometry></collision>
        </link>
        <link name=\"\${prefix}finger_right_link\">
          <visual><geometry><box size=\"0.015 0.04 0.08\"/></geometry></visual>
          <collision><geometry><box size=\"0.015 0.04 0.08\"/></geometry></collision>
        </link>
      </xacro:macro>
    </robot>

第一版使用 box 碰撞模型即可，后续再换成真实 CAD 网格。

### B.3 将夹爪连接到 FR3 TCP

查找 FR3 总 Xacro：

    find ~/fr3_ws/src -type f -name '*.xacro' | grep -E 'fairino3|robot'

在总 Xacro 中增加：

    <xacro:include filename=\"$(find fr3_gripper_description)/urdf/gripper.urdf.xacro\"/>
    <xacro:gripper prefix=\"\" parent=\"REAL_TCP_FRAME\"/>

REAL_TCP_FRAME 必须替换为法奥配置中的真实 TCP link。检查夹爪中心相对 TCP 的 xyz 偏移、闭合方向和碰撞模型。

### B.4 创建夹爪 ROS 2 Python 节点

    cd ~/fr3_ws/src
    ros2 pkg create fr3_gripper_manager --build-type ament_python \
      --dependencies rclpy control_msgs
    mkdir -p fr3_gripper_manager/fr3_gripper_manager
    touch fr3_gripper_manager/fr3_gripper_manager/__init__.py

创建文件：

    nano ~/fr3_ws/src/fr3_gripper_manager/fr3_gripper_manager/gripper_node.py

写入：

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from control_msgs.action import GripperCommand

    class GripperNode(Node):
        def __init__(self):
            super().__init__('gripper_manager')
            self.declare_parameter(
                'action_name',
                '/gripper_controller/gripper_cmd')
            self.action_name = self.get_parameter('action_name').value
            self.client = ActionClient(
                self, GripperCommand, self.action_name)

        def send_command(self, position, effort):
            if not self.client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('gripper action server unavailable')
                return False
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = float(effort)
            future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            handle = future.result()
            if handle is None or not handle.accepted:
                self.get_logger().error('gripper goal rejected')
                return False
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result
            self.get_logger().info(
                f'reached={result.reached_goal}, stalled={result.stalled}')
            return bool(result.reached_goal or result.stalled)

        def open(self):
            return self.send_command(0.08, 20.0)

        def close(self):
            return self.send_command(0.0, 20.0)

    def main():
        rclpy.init()
        node = GripperNode()
        node.open()
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

0.08、0.0 和 20.0 只是示例，必须按夹爪控制器单位、行程和最大力度修改。

### B.5 配置 Python 入口点

    nano ~/fr3_ws/src/fr3_gripper_manager/setup.py

确保 entry_points 包含：

    entry_points={
        'console_scripts': [
            'gripper_node = fr3_gripper_manager.gripper_node:main',
        ],
    },

编译和运行：

    cd ~/fr3_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install --packages-select fr3_gripper_manager
    source install/setup.bash
    ros2 run fr3_gripper_manager gripper_node

### B.6 配置关节式夹爪控制器

如果夹爪使用 ros2_control，在 ros2_controllers.yaml 中加入：

    left_gripper_controller:
      ros__parameters:
        joints:
          - left_gripper_joint
        command_interfaces:
          - position
        state_interfaces:
          - position
          - velocity

启动后检查：

    ros2 control list_controllers
    ros2 topic list | grep gripper
    ros2 action list | grep gripper

如果夹爪是 IO、Modbus 或厂家 SDK，则跳过这个控制器配置，保留 gripper_manager 节点并把 send_command() 替换为实际通信代码。

### B.7 重新生成 MoveIt 配置

    source /opt/ros/humble/setup.bash
    ros2 run moveit_setup_assistant moveit_setup_assistant

操作：

    1. Load Files：加载包含夹爪的完整 URDF。
    2. Self-Collisions：生成碰撞矩阵。
    3. Planning Groups：确认 fr3_arm。
    4. Planning Groups：增加 gripper。
    5. End Effectors：增加 fr3_gripper。
    6. Parent Link：选择实际 TCP link。
    7. ROS 2 Controllers：加入夹爪控制器。
    8. Configuration Files：生成新的 MoveIt 配置包。

如果夹爪由独立 SDK 节点控制，可以只把夹爪加入 URDF 和碰撞模型，不把它加入 MoveIt 可执行控制器。

### B.8 单臂抓取测试

不接视觉，先把零件位姿写死：

    part:
      frame: base_link
      position: [0.40, 0.10, 0.03]
      orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
      lift_height: 0.10

状态顺序：

    OPEN_GRIPPER
    → MOVE_PRE_GRASP
    → MOVE_APPROACH
    → CLOSE_GRIPPER
    → VERIFY_GRASP
    → MOVE_LIFT
    → MOVE_RETURN
    → OPEN_GRIPPER

测试时先在仿真执行，抬升 50 到 100 mm，保持 2 s，再回到原位。连续十次成功后，才接入第二台 FR3、相机和换手逻辑。

## 5. 主要依赖

    sudo apt install -y \
      ros-humble-rclpy \
      ros-humble-tf2-ros \
      ros-humble-tf2-geometry-msgs \
      ros-humble-control-msgs \
      ros-humble-trajectory-msgs \
      ros-humble-moveit-msgs

Python 库：

    python3 -m pip install --user numpy scipy PyYAML

不要在真实机器人上直接运行未经仿真验证的轨迹。

### 1. 检查已安装包

    source /opt/ros/humble/setup.bash
    source ~/moveit2_ws/install/setup.bash
    ros2 pkg list | grep -E 'fairino|moveit|controller'

应重点确认 fairino3_v6_moveit2_config、fairino_description、fairino_hardware、fairino_msgs 和 controller_manager 存在。若工作区不是 moveit2_ws，请替换路径。

### 2. 启动仿真 MoveIt

    ros2 launch fairino3_v6_moveit2_config demo.launch.py

然后检查：

    ros2 control list_controllers
    ros2 control list_hardware_interfaces
    ros2 topic echo /joint_states --once

拖动 RViz 末端标记，执行 Plan 和 Execute。记录实际规划组、TCP link、六个关节名和控制器名。不要直接使用 UR5 的 tool0 或 ur_manipulator。

### 3. 检查 TF

    ros2 run tf2_tools view_frames
    ros2 run tf2_ros tf2_echo base_link tool_frame

其中 tool_frame 只是示例，必须替换成 FR3 配置中的实际 TCP。后续需要预留 world、table 和 camera_link。

### 4. 检查实机切换

仿真一般使用 mock_components/GenericSystem，真实 FR3 使用 fairino_hardware/FairinoHardwareInterface。切换前备份 ros2_control xacro，确认 FR3 软件版本、硬件包版本和控制柜 IP 一致。真实启动后先只观察 joint_states，不执行运动；确认急停和低速模式后再执行安全命名姿态。

## 阶段二验收

- 夹爪模型安装到真实 TCP。
- 夹爪可打开、闭合、停止。
- 可检测夹持结果。
- 抬升测试零件后不掉落。
- 连续十次抓取和放回成功。

### 1. 选择夹爪接口

如果夹爪有位置控制接口，可以把主动夹爪关节加入 ros2_control；如果夹爪通过 IO、Modbus、TCP 或厂家 SDK 控制，建议作为独立 ROS 2 节点，不要伪装成机械臂关节。

推荐节点名：gripper_manager。

推荐接口：

    /gripper/open
    /gripper/close
    /gripper/stop
    /gripper/verify

可以使用 control_msgs/action/GripperCommand，或者定义自己的 GripperCommand.action。

### 2. 创建夹爪描述包

    cd ~/fr3_ws/src
    ros2 pkg create fr3_gripper_description --build-type ament_cmake

目录建议为：

    fr3_gripper_description/
    ├── urdf/gripper.urdf.xacro
    ├── meshes/
    ├── config/
    ├── CMakeLists.txt
    └── package.xml

第一版只使用简单 box 碰撞模型：夹爪底座、左右手指和测试零件。高精度 CAD 网格后续再加入。

### 3. 安装到 FR3 TCP

在 FR3 总 Xacro 中包含 gripper.urdf.xacro，并通过 fixed joint 连接到法奥实际 TCP link。必须实测夹爪中心相对 TCP 的 xyz 偏移、闭合方向、手指坐标轴和最大开口。

双臂阶段必须使用唯一名称，例如 left_gripper_joint、right_gripper_joint、left_gripper_base_link 和 right_gripper_base_link。

### 4. 配置 MoveIt

使用 MoveIt Setup Assistant 重新加载包含夹爪的完整 URDF，添加：

    Planning Groups:
      fr3_arm
      gripper

    End Effector:
      fr3_gripper
      parent group: fr3_arm
      parent link: tool_frame

实际的 parent link 仍以法奥 URDF 为准。未来双臂使用 left_fr3_arm、right_fr3_arm、left_gripper、right_gripper 和 dual_fr3。

### 5. 配置关节式夹爪控制器

示例：

    left_gripper_controller:
      ros__parameters:
        joints:
          - left_gripper_joint
        command_interfaces:
          - position
        state_interfaces:
          - position
          - velocity

如果左右手指是 mimic，只控制主动关节。URDF、控制器和 MoveIt 中的关节名必须完全一致。

### 6. 编写夹爪管理节点

Python 依赖：rclpy、rclpy.action、control_msgs、trajectory_msgs。节点实现 open、close、verify_grasp、stop 四个功能。close 返回成功不能只看 Action 结果，还应读取夹爪位置、电流、力或提前停止状态。

### 7. 单臂抓取测试

暂时不接视觉，使用固定零件位姿：

    OPEN_GRIPPER
    → MOVE_PRE_GRASP
    → MOVE_APPROACH
    → CLOSE_GRIPPER
    → VERIFY_GRASP
    → MOVE_LIFT
    → MOVE_RETURN
    → OPEN_GRIPPER

抬升高度可先设为 50 到 100 mm。连续十次成功后，才进入桌面零件场景和第二台 FR3。

## 参考

- https://github.com/FAIR-INNOVATION/frcobot_ros2
- https://blog.csdn.net/2301_78767880/article/details/147323555
