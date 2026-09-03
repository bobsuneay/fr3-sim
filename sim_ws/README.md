# Fairino DualArm Sim

这是一个自包含的 ROS 2 Humble 双臂仿真 MVP，不连接真实 Fairino、Orbbec 或 EPG50 硬件。

## 运行

在 Ubuntu + ROS 2 Humble 中：

```bash
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-xacro ros-humble-robot-state-publisher
source /opt/ros/humble/setup.bash
cd sim_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch fairino_dualarm_sim sim.launch.py
```

验证：

```bash
ros2 topic echo /vision/object_pose
ros2 topic echo /joint_states
```

启动命令会自动打开 RViz2，并加载 `RobotModel`、`TF` 和视觉目标 Marker；固定坐标系为 `world`。

## 当前能力

- 双臂 6-DOF 简化 URDF 模型
- 关节目标插值和 `/joint_states`
- 仿真视觉目标 `/vision/object_pose`
- 目标 Marker `/vision/object_marker`
- 预抓取、接近、虚拟闭合、抬升、放置状态机

这是第一版系统骨架，下一步可以把 `sim_controller` 替换为 MoveIt2/ros2_control，把 `sim_vision` 替换为 Gazebo 相机或真实检测节点，把 fake gripper 替换为 EPG50 接口。

## Gazebo 螺栓抓取场景

本仓库还提供一个 Gazebo Classic MVP：`gazebo.launch.py` 会加载桌面、5x8 个 25 mm x 6 mm 螺栓、侧装铝型材/法兰/平行夹爪，以及安装在工具顶部的深度相机。

Ubuntu + ROS 2 Humble：

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
cd sim_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch fairino_dualarm_sim gazebo.launch.py
```

相机话题为 `/head_depth/points`、`/head_depth/image_raw`、`/head_depth/depth/image_raw` 和 `/head_depth/camera_info`。相机坐标系为 `head_camera_optical_frame`。当前螺栓是固定在桌面上的视觉/规划测试件；要测试真实抓取碰撞，把每个螺栓的 fixed joint 改为 `floating`，再通过 Gazebo 接触动力学或 grasp attach 插件实现夹取。
