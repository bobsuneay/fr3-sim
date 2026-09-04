# Ubuntu 22.04：FR3 侧装相机工作台搭建与验收

## 0. 先确认版本和已有工程边界

本包直接使用用户 FR3 几何，但**不要求启动原来的 MoveIt demo**。原工程包含腕部相机、Robotiq、固定房间坐标、香蕉演示等配置；这些配置不能和新场景同时启动，否则容易出现双重 TF、两个控制管理器或错误的桌面坐标。

本次保持相同的六轴关节名和规划组名：`j1..j6`、`fairino3_v6_group`。工具改为 `gripper_tcp`，不继续使用原项目中的 `ur5e_gripper_tcp`。原 Robotiq 的单个旋转驱动关节被两个简单平移手指替代，因此原夹爪控制器 YAML 不能直接复用。

```bash
lsb_release -ds
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
gazebo --version
```

预期 Ubuntu 22.04、`humble`、Gazebo 11.x。`gz sim`/`ign gazebo` 对应另一条仿真栈，不是本包的启动命令。安装依赖见包首页；若 Gazebo Classic 的 apt 包不可用，请先检查 Ubuntu universe 与 ROS 官方软件源，不要用安装 Harmonic 来替代同名组件。

## 1. 复制、安装依赖、编译

如果终端仍显示 `(base)`，先执行 `conda deactivate`，再 source ROS。编译和运行 ROS Python 节点使用 Ubuntu 的系统 Python。不要仅在 Conda 中安装 NumPy来修复 ROS 消息依赖；Humble 的预编译扩展需要匹配系统 Python。

不要复制 Windows 的 `.tools`，也不要复制 `Desktop/FR3` 内的 `build`、`install`、`log` 和 YOLO 权重。新包需要的七个 STL 已包含在源码内，不依赖桌面绝对路径。

将仓库的 `sim_ws` 目录复制/重命名为 Ubuntu 的 `~/fr3_bolt_ws`。若你的 Ubuntu 里已有工作空间，也可只把 `sim_ws/src/fr3_bolt_cell` 放进该工作空间的 `src/`。不要同时放两份同名包。

```bash
source /opt/ros/humble/setup.bash
cd ~/fr3_bolt_ws
test -f src/fr3_bolt_cell/package.xml
rosdep update
rosdep install --from-paths src/fr3_bolt_cell --ignore-src -r -y --rosdistro humble
/usr/bin/python3 /usr/bin/colcon build --symlink-install --packages-select fr3_bolt_cell
source install/setup.bash
ros2 pkg prefix fr3_bolt_cell
ros2 pkg executables fr3_bolt_cell
```

应看到 `generate_world`、`gripper`、`publish_scene`、`check_sim`。本包不启动法奥 SDK、不配置机械臂 IP、不连接夹爪硬件。

## 2. 模型结构和坐标

世界坐标采用米/弧度：X 朝桌面前方、Y 朝左、Z 向上。固定相机属于 eye-to-hand，不是 eye-in-hand。

```text
world
└── support_link（竖直型材，固定在世界）
    ├── support_foot / side_mount_plate
    ├── head_bracket / head_camera_link
    │   └── head_camera_optical_frame
    └── base_link（FR3 基座侧装）
        └── shoulder → upperarm → forearm → wrist1 → wrist2 → wrist3
            └── tool0 → gripper_palm
                ├── left_finger（滑动关节）
                ├── right_finger（滑动关节）
                └── gripper_tcp

Gazebo 独立模型：ground、work_table、bolt_00_00 … bolt_03_04
```

| 项目 | 默认值 |
| --- | --- |
| 型材长方体 | 120 × 160 × 1360 mm |
| FR3 基座安装点 | world `[0, -0.10, 1.10]` m |
| 侧装旋转 | roll=π/2，基座 +Z 轴指向世界 -Y |
| 相机位置 | world `[0.06, 0, 1.44]` m |
| 相机俯角 | pitch=1.10 rad，约 63° 向下 |
| RGB-D 参数 | 640×480、15 Hz、水平视场角 70°、深度范围 0.05～3 m |
| 桌面 | 700×650 mm，顶面高度 720 mm |
| 螺栓 | 4×5，含头总长 25 mm，杆直径 5 mm，头直径 9 mm |
| 夹爪 | 两个滑块各 0～30 mm；内侧净开口为两关节位置之和 |

表中尺寸是仿真初值，不是经过承载设计的机械图纸。螺栓的杆和头采用两个圆柱近似，没有螺纹、六角棱或微小缺陷。螺栓横放，接触后可能轻微倾斜/滚动，不能把初始位姿永久当作测量真值。

## 3. 先启动 Gazebo 和控制器

先关掉同一 ROS 域中旧的 Gazebo、RViz demo 和实机驱动终端。选择未被实机使用的域；这里示例 31。

```bash
source /opt/ros/humble/setup.bash
source ~/fr3_bolt_ws/install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch fr3_bolt_cell gazebo.launch.py moveit:=false rviz:=false
```

启动过程为：生成临时 world → Gazebo 与 `robot_state_publisher` → 模型生成成功 → joint_state_broadcaster → fairino3_controller → gripper_controller。没有固定秒数的启动猜测；任何生成/控制器启动失败会停止本次 launch。world 保存在日志显示的临时目录，退出时清理，可按第 7 节单独生成供检查。

另开终端：

```bash
source /opt/ros/humble/setup.bash
source ~/fr3_bolt_ws/install/setup.bash
export ROS_DOMAIN_ID=31
ros2 control list_controllers -c /controller_manager
ros2 control list_hardware_interfaces -c /controller_manager
ros2 topic echo /joint_states --once
ros2 topic hz /clock
```

验收：三个控制器均为 `active`；六个手臂关节和两个手指有 position 命令接口；`/joint_states` 有八个关节；Gazebo 中机械臂没有整机掉落，螺栓在桌上自由接触。`/clock` 的频率显示完按 Ctrl+C。

注意初始姿态来自 `config/initial_positions.yaml`，Gazebo 插件读取 `initial_value` 设置初始关节。不依赖 ROS 1 式 `spawn_entity -J`，也不会发一条“自动归零”轨迹。

## 4. 验证头部相机

```bash
ros2 topic list
ros2 topic info /head_camera/points --verbose
ros2 topic echo /head_camera/camera_info --once --qos-reliability best_effort
ros2 topic hz /head_camera/image_raw
```

实际发布接口：

| 话题 | 类型 |
| --- | --- |
| `/head_camera/image_raw` | sensor_msgs/Image，RGB |
| `/head_camera/camera_info` | sensor_msgs/CameraInfo |
| `/head_camera/depth/image_raw` | sensor_msgs/Image，32FC1 深度，米 |
| `/head_camera/depth/camera_info` | sensor_msgs/CameraInfo |
| `/head_camera/points` | sensor_msgs/PointCloud2，XYZRGB |

```bash
timeout 5s ros2 run tf2_ros tf2_echo world head_camera_optical_frame
ros2 run fr3_bolt_cell check_sim --timeout 60
```

这些检查只读，不会移动手臂。`check_sim` 检查运行中的时钟、八关节、控制器、RGB/深度尺寸、光学 frame、有限点云与 TF，还通过 `/gazebo/model_states` 确认全部螺栓存在且位于桌面高度附近，失败退出码为 1。该脚本用于启动后的空载验收；物体已被抬升时，高度检查会按设计失败。

`/gazebo/model_states` 是物理仿真状态，不是视觉检测输出；可以用它记录螺栓沉降后的真实位置，但部署到实机时必须替换为真实感知。

光学坐标是 X 向右、Y 向下、Z 向前；相机传感器默认 +X 朝前，本包用固定旋转转换，不能只改消息里的 frame 名字而不配 TF。

640×480、约 0.8 m 工作距离的默认设置主要用于搭建环境与粗定位。25 mm 小件在图像中只占十余像素量级，细螺纹/微小缺陷不可据此保证分辨。后续可提高分辨率、缩小视场，或增加固定近距离检测相机；此处不包含任何缺陷算法或真实镜头自动对焦模型。

## 5. 验证夹爪

先保证手指处没有物体，只验证真实关节开合：

```bash
ros2 action list -t
ros2 run fr3_bolt_cell gripper --width 0.050 --seconds 3
ros2 run fr3_bolt_cell gripper --width 0.010 --seconds 3
ros2 run fr3_bolt_cell gripper --width 0.050 --seconds 3
```

命令使用 `/gripper_controller/follow_joint_trajectory`，它是 `FollowJointTrajectory`，**不是** `GripperCommand`。两指位置为 `[width/2, width/2]`。CLI 校验开口范围、等待仿真时钟和 action 结果，超时会请求取消。

存在 `/clock` 不是硬件隔离保证。务必使用隔离 ROS 域，绝不要在连接真实机械臂/夹爪的域执行这些测试命令。

此版采用 Gazebo position 接口，便于轨迹联动，但控制器不是力控夹爪，可能直接设置关节位置。惯量、接触、摩擦已提供，并不等价于已完成稳定小件夹取。后续需要调整物理步长、接触刚度、手指碰撞形状，并选用 effort/position_pid 力学控制或明确标为非物理的 attach 方案。不能拿“闭合成功”的返回值当作“夹到了螺栓”。

## 6. 接入已具备的 MoveIt 2 能力

结束第 3 节 launch 后，重新启动全链路，不要在旧进程旁再启动第二套：

```bash
ros2 launch fr3_bolt_cell gazebo.launch.py
```

本包自带与当前夹爪、相机和安装方向对应的 SRDF、KDL、OMPL、控制器映射。MoveIt 启动后将同一组桌面/桌腿盒体写入 PlanningScene，成功后才打开 RViz。型材、机器人、夹爪、相机的碰撞几何来自 URDF。

RViz 操作：

1. Fixed Frame 保持 `world`，MotionPlanning 的组选择 `fairino3_v6_group`。
2. 确认 PlanningScene 中可以看到桌面与四根桌腿，RobotModel 与 Gazebo 一致。
3. 选择 `ready`，或小幅移动末端交互标记，先点击 Plan，观察轨迹和碰撞反馈。
4. 确认轨迹可行后，在这个仿真域里点击 Execute；观察 Gazebo 实际关节跟随。
5. 夹爪也可选择 `gripper` 组的 `open`/`closed`，但空载验收优先用第 5 节 CLI。MoveIt 的 `closed` 留 0.5 mm 净间隙，避免两指恰好接触时被碰撞检测拒绝。

规划服务就绪检查：

```bash
ros2 service list | grep planning_scene
ros2 param get /move_group use_sim_time
ros2 action info /fairino3_controller/follow_joint_trajectory
ros2 action info /gripper_controller/follow_joint_trajectory
```

本包没有把 20 个会移动的螺栓硬编码成永久静态 MoveIt 障碍，也没有自动接入点云 Octomap。自动抓取前必须增加目标与邻近件感知、抓取位姿、动态 CollisionObject、夹持后 AttachedCollisionObject。不要因为桌子已经加入规划场景就认定所有小件避障已完成。

若只保留你原来的 MoveIt 配置，至少同步：URDF 全树、`gripper_tcp`、两个手指关节、SRDF、控制器 action 名、`use_sim_time` 和当前桌面坐标；不要同时启动两套 `move_group`/`robot_state_publisher`/controller_manager。这里的默认启动已替你完成一套独立匹配配置。

## 7. 修改布局、导出模型

编辑参数：

```bash
cd ~/fr3_bolt_ws
nano src/fr3_bolt_cell/config/scene.yaml
colcon build --symlink-install --packages-select fr3_bolt_cell
source install/setup.bash
ros2 launch fr3_bolt_cell gazebo.launch.py
```

也可以保留源码默认值，复制成自定义场景：

```bash
cp src/fr3_bolt_cell/config/scene.yaml /tmp/my_fr3_scene.yaml
nano /tmp/my_fr3_scene.yaml
ros2 launch fr3_bolt_cell gazebo.launch.py scene:=/tmp/my_fr3_scene.yaml
```

`rows` 沿世界 X 方向排列，`cols` 沿 Y 方向排列。修改 `first_xy`、`spacing_xy` 时要确认阵列仍落在桌面与机械臂工作空间内。生成器会拒绝长度 ≥30 mm、负尺寸、阵列越出桌边和间距导致的重叠；它不代替运动学可达性求解。

相机可先试 `width: 1280`、`height: 720`、`rate: 10.0`，再看帧率；改分辨率不能直接推断真实相机精度。修改安装高度/角度时同步检查支架连接、相机视野和初始机械臂碰撞。默认转接板位置是为当前右侧安装设计的，换左侧安装应同步改板件。

导出、检查离线文件：

```bash
sudo apt install -y liburdfdom-tools
ros2 run fr3_bolt_cell generate_world --output /tmp/fr3_bolts.world
xacro "$(ros2 pkg prefix fr3_bolt_cell)/share/fr3_bolt_cell/urdf/cell.urdf.xacro" > /tmp/fr3_cell.urdf
check_urdf /tmp/fr3_cell.urdf
gz sdf -p /tmp/fr3_cell.urdf > /tmp/fr3_cell.sdf
gz sdf -k /tmp/fr3_bolts.world
gz sdf -k /tmp/fr3_cell.sdf
```

命令中的 `gz sdf` 属于 Gazebo Classic 配套工具；如果提示不同命令集，检查是否被另一版本的 `gz` 覆盖。生成的 URDF/SDF 用于排查，不需要手动运行 `gzserver` 来替代正常 launch。

## 8. 无 GUI 验收与故障定位

<a id="python-environment-recovery"></a>

### spawn_entity.py 报 `No module named 'numpy'`

若第一条异常发生在 `spawn_entity.py → geometry_msgs → import numpy`，表示运行模型生成脚本的 Python 环境没有 NumPy。Conda `(base)` 会改变 PATH，而 Gazebo 的脚本使用 `#!/usr/bin/env python3`，因此可能选中 Conda Python；单凭日志不能排除系统 NumPy 本身未安装。

启动器收到模型生成失败后会关闭 Gazebo 和 robot_state_publisher，所以随后出现的 `SIGINT`、退出码 `-2` 和 `rcl node's context is invalid` 在该日志顺序下属于关闭过程的后续错误。应先解决模型生成脚本的依赖问题。

修复后的 launch 使用 `/usr/bin/python3` 启动 `spawn_entity.py`、控制器 spawner 和本包 `publish_scene`。先退出 Conda（如有多层环境，重复到提示符无 Conda 名称）：

```bash
conda deactivate
```

对于按 GitHub 使用说明克隆到 `~/fr3-sim` 的用户，执行：

```bash
cd ~/fr3-sim
git pull --ff-only origin main

sudo apt update
sudo apt install -y python3-numpy python3-yaml python3-lxml python3-colcon-common-extensions
source /opt/ros/humble/setup.bash

/usr/bin/python3 -c "import sys, numpy, rclpy; from geometry_msgs.msg import Pose; from lxml import etree; print(sys.executable); print(numpy.__version__); print('ROS Python imports OK')"

cd ~/fr3-sim/sim_ws
/usr/bin/python3 /usr/bin/colcon build --symlink-install --packages-select fr3_bolt_cell
source install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch fr3_bolt_cell gazebo.launch.py
```

导入检查应输出 `/usr/bin/python3` 和 `ROS Python imports OK`。若仍失败，先保留这条命令的完整异常，不要跳过检查继续启动。若工作空间采用前面文档的 `~/fr3_bolt_ws` 复制方式，将重新编译和 source 的路径相应调整。

在更新后的 launch 中，Python 子进程命令会带 `/usr/bin/python3` 前缀。此修复只验证了解释器选择逻辑；是否成功创建 Gazebo 模型及后续控制器，须在 Ubuntu 重启验证。

参照：[Gazebo 模型生成脚本的解释器声明](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/scripts/spawn_entity.py)、[ROS 2 官方 Python 环境说明](https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/Using-Python-Packages.rst)。

### 无 GUI 和其他故障

关闭 Gazebo 窗口不等于关闭深度相机的图形渲染依赖。已有显示环境时可：

```bash
ros2 launch fr3_bolt_cell gazebo.launch.py gui:=false rviz:=false
```

无显示服务器的 Ubuntu：

```bash
sudo apt install -y xvfb mesa-utils
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s "-screen 0 1280x720x24" \
  ros2 launch fr3_bolt_cell gazebo.launch.py gui:=false rviz:=false
```

软件渲染通常更慢；虚拟机需可用的 OpenGL。另一个同域终端运行 `check_sim`。

| 症状 | 首先检查 |
| --- | --- |
| spawn_entity.py 找不到 NumPy | 退出 Conda，安装系统 python3-numpy，拉取修复并用 /usr/bin/python3 调用 colcon 重编译，见上方恢复命令 |
| `/spawn_entity` 不存在 | Gazebo 插件是否加载；必须通过本包的 gazebo_ros launch，不能只启动裸 gzserver |
| 控制器一直等待 | `libgazebo_ros2_control.so` 是否安装、同域中是否存在重复 controller_manager、URDF 插件日志 |
| 找不到 FR3 STL | 是否完成 colcon 并 source 当前工作空间；URI 应为 `package://fr3_bolt_cell/meshes/...` |
| 没有深度或点云 | `libgazebo_ros_camera.so`、Gazebo 是否播放、显示/OpenGL、实际订阅话题、命名空间 |
| RViz 点云存在但不显示 | Fixed Frame=world、TF、QoS Best Effort、PointCloud2 的 Color Transformer=RGB8 |
| MoveIt 规划成功但不执行 | 两个 FollowJointTrajectory action 是否存在，关节名是否一致，所有节点是否 use_sim_time |
| MoveIt 报初始状态碰撞 | 修改安装位姿后重新检查真实碰撞，不要批量关闭所有碰撞对 |
| 螺栓漂浮、弹飞或穿透 | 单位是否米、惯量是否被删、是否把精细小件用 1 cm 接触层模拟、位置控制是否在强行挤压 |
| 性能不足 | 减小相机分辨率/帧率，减少阵列数量；不要先删除物体惯量来“加速” |

本场景没有外部在线模型下载，离线 Ubuntu 在依赖安装完成后也可运行。

## 9. 下一步接回你的检测任务

当前包完成的是环境和控制接口。下一阶段先做单个螺栓的“接近—夹持确认—抬升—放回”，稳定后才扩展全阵列。固定检测中心旋转应围绕**零件中心**生成 TCP 轨迹，而不是只改变腕关节角度。

用 `T_world_object × inverse(T_tcp_object)` 计算所需 TCP 位姿，并在改变零件朝向时保持选定的零件中心平移近似不变；还要评估可达性、碰撞和遮挡。Gazebo 的理想深度传感器不模拟真实焦深，需要实机相机工作距离/镜头参数另行标定。

双臂阶段需要给两份 FR3 和夹爪加 `left_`/`right_` 前缀、统一规划场景、规划组和控制器，并处理交接时的碰撞许可及附着物归属。本包没有声称已完成双臂或实机安全验证。

## 10. 参考与选择理由

- [法奥官方 fairino_gazebo](https://github.com/FAIR-INNOVATION/fairino_gazebo)：参考 FR 系列 Gazebo/ros2_control 集成；本包优先采用用户提供的真实 FR3 模型，避免混用型号。
- [jjh1214/fairino_sim](https://github.com/jjh1214/fairino_sim)：可参考仿真与 MoveIt 组织方式，其 Jazzy/Harmonic 环境不能直接套进本包的 Humble/Classic。
- [hellototoro/grasp_sim](https://github.com/hellototoro/grasp_sim)：参考抓取场景组织思路，不移植其 Panda 模型或假定其夹取接口可直接控制 FR3。
- [Gazebo ROS 相机官方源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_camera.cpp)：据此核对 ROS 2 插件名、snake_case 参数和实际话题路径。
- [Humble gazebo_ros2_control 官方说明](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html)：据此使用 GazeboSystem、initial_value 与唯一控制管理器，同时保留 Classic 停维提示。
