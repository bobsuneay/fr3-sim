# 法奥 FR3 螺栓工作台仿真

基于你提供的 `Desktop/FR3/src/fairino3_v6.urdf` 和七个 STL 网格构建，目标环境为 **Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11**。这是法奥 FR3，不是 Franka FR3，也不是用 UR5 几何替代。

## 这次搭建的内容

- 一根竖直长方体型材和底板，右侧安装 FR3 基座；没有移动底盘。
- FR3 六个关节 `j1`～`j6`，保留原始连杆几何、关节范围和惯量。
- 腕部平行夹爪：两个反向滑动手指，开口 0～60 mm，经 `ros2_control` 控制。
- 型材顶部固定 RGB-D 相机，模拟头部视角；输出 RGB、深度、相机内参和点云。
- 高 720 mm 的桌面，4 行 × 5 列独立动态螺栓；**总长含头部 25 mm**。
- Gazebo、控制器、MoveIt 2、RViz 顺序启动，以及 Python 开合/验收工具。

安装方向按“基座法兰侧装在型材侧面，工具法兰装夹爪”理解。相机不随手腕运动。若你指的是夹爪通过侧向转接板安装，需按实际转接件修改 `tool0_to_gripper`，不能把两种安装理解混用。

当前是**单臂环境基础包**。尚未实现自动抓取、缺陷检测、绕零件中心扫描、双臂交接和实机驱动；也没有使用虚拟吸附冒充物理夹取。

## 快速启动

把本仓库的整个 `sim_ws` 文件夹复制到 Ubuntu 的 `~/fr3_bolt_ws`，使 `~/fr3_bolt_ws/src/fr3_bolt_cell/package.xml` 存在。已安装 ROS 2 Humble 后：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-yaml python3-pytest \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-controllers ros-humble-moveit ros-humble-rviz2

cd ~/fr3_bolt_ws
rosdep update
rosdep install --from-paths src/fr3_bolt_cell --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-select fr3_bolt_cell
source install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch fr3_bolt_cell gazebo.launch.py
```

如果 `rosdep` 尚未初始化，先执行一次 `sudo rosdep init`。ROS_DOMAIN_ID 要与实机所用域不同，且本次仿真的所有终端保持一致。

第二个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/fr3_bolt_ws/install/setup.bash
export ROS_DOMAIN_ID=31
ros2 run fr3_bolt_cell check_sim --timeout 60
ros2 run fr3_bolt_cell gripper --width 0.050
ros2 run fr3_bolt_cell gripper --width 0.010
```

`--width` 单位是米，0.010 表示内侧净开口 10 mm。先在无物体的初始姿态测试开合；它不是夹持力命令。

## 主要入口

| 文件 | 用途 |
| --- | --- |
| [详细搭建与验收文档](docs/Ubuntu22.04.md) | 分步骤命令、参数修改、常见故障、下一阶段接口 |
| [场景参数](config/scene.yaml) | 型材、安装位姿、相机、桌面和螺栓阵列 |
| [整机 Xacro](urdf/cell.urdf.xacro) | 真实 FR3 模型、夹爪、固定相机、Gazebo 插件 |
| [启动文件](launch/gazebo.launch.py) | 唯一的状态发布器与控制管理器、按完成事件串联启动 |
| [物理场景生成器](fr3_bolt_cell/world.py) | 根据 YAML 生成桌面与自由螺栓 SDF |
| [测试说明](docs/VALIDATION.md) | 已执行的离线测试和仍待执行的运行测试 |
| [来源与授权记录](THIRD_PARTY.md) | 用户提供模型的来源、哈希、公开分发注意事项 |

## 重要边界

Gazebo Classic 已停止维护；此版本是为了兼容你已有的 Humble/FR3 工程，不能直接混入 Jazzy/Harmonic 插件。后续迁移应整体更换仿真后端。[官方兼容说明](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html)

本次在 Windows 完成代码生成和离线检查，**没有在 Ubuntu/Gazebo 中实际启动验证**。是否稳定抓住 25 mm 小件、点云质量和帧率，必须运行验收并进一步调参。现有位置控制模式不模拟夹爪真实电流/力限位。
