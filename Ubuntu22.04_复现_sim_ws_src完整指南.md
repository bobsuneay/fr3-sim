# Ubuntu 22.04 复现 `sim_ws/src` 指南

本文针对 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11 和 MoveIt 2。Windows 目录不能直接运行 ROS 2 或 Gazebo。

## 当前包

| 包 | 用途 |
|---|---|
| `fairino_dualarm_sim` | 双臂流程和视觉模拟 |
| `fr3_bolt_cell` | 单臂 FR3 螺栓 Gazebo 工作台 |
| `fr3_bolt_inspection_cell` | 双臂点云抓取和检测仿真 |
| `fr3_dual_bolt_cell` | 双臂 FR3、HKV、MoveIt 和真机后端 |
| `fr3_dual_bolt_cell_modified` | 左手指主驱动、右手指 mimic 的修改版 |
| `fr3_real_bringup` | 单台正装 FR3 真机或 mock |

`fr3_dual_bolt_cell_modified` 与原 `fr3_dual_bolt_cell` 的 `package.xml` 包名相同，不能在同一个 colcon 源空间同时构建。修改版请使用单独工作空间。

## 1. 系统和 ROS 2

```bash
lsb_release -a
uname -m
sudo apt update
sudo apt install -y curl wget git build-essential cmake pkg-config \
  software-properties-common lsb-release gnupg2 ca-certificates \
  python3-pip python3-dev python3-yaml python3-numpy python3-pytest \
  python3-colcon-common-extensions python3-rosdep ripgrep

sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update
sudo apt install -y \
  ros-humble-desktop ros-humble-moveit ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-rviz2 ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
  ros-humble-gazebo-ros2-control ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
  ros-humble-gripper-controllers ros-humble-position-controllers \
  ros-humble-kdl-parser ros-humble-moveit-kinematics \
  ros-humble-moveit-planners-ompl ros-humble-moveit-ros-move-group \
  ros-humble-moveit-ros-visualization

sudo rosdep init 2>/dev/null || true
rosdep update
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO
ros2 --version
gazebo --version
```

退出 Conda，确认 `which python3` 使用 `/usr/bin/python3`。

## 2. 获取和构建原始工作空间

```bash
cd ~
git clone https://github.com/bobsuneay/fr3-sim.git
cd ~/fr3-sim
git pull origin main
cd sim_ws
source /opt/ros/humble/setup.bash
colcon list
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
/usr/bin/colcon build --symlink-install \
  --packages-select fairino_dualarm_sim fr3_bolt_cell \
  fr3_bolt_inspection_cell fr3_dual_bolt_cell fr3_real_bringup \
  --event-handlers console_direct+
source install/setup.bash
```

不要复制 Windows 生成的 `build/`、`install/`、`log/` 或 `.tools/`。

## 3. 双臂流程和单臂仿真

```bash
source /opt/ros/humble/setup.bash
source ~/fr3-sim/sim_ws/install/setup.bash
export ROS_DOMAIN_ID=31

# 双臂流程模拟
ros2 launch fairino_dualarm_sim sim.launch.py

# 单臂螺栓 Gazebo
ros2 launch fr3_bolt_cell gazebo.launch.py
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source ~/fr3-sim/sim_ws/install/setup.bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic list | grep head_camera
ros2 run fr3_bolt_cell check_sim --timeout 60
```

## 4. 原始双臂包的 Gazebo 和 mock

```bash
source /opt/ros/humble/setup.bash
source ~/fr3-sim/sim_ws/install/setup.bash
export ROS_DOMAIN_ID=31

ros2 launch fr3_dual_bolt_cell bringup.launch.py \
  mode:=gazebo enable_execution:=true

# 停止上一套 launch 后，也可以运行无 Gazebo mock
ros2 launch fr3_dual_bolt_cell bringup.launch.py \
  mode:=mock enable_execution:=true
```

检查控制器和夹爪：

```bash
ros2 control list_controllers
ros2 action list -t | grep -E 'follow_joint_trajectory|gripper_controller/command'
ros2 run fr3_dual_bolt_cell check_feedback --timeout 30
ros2 run fr3_dual_bolt_cell gripper --arm left --width 0.020
```

## 5. 修改版双臂包

修改版目录在仓库的 `sim_ws/src/fr3_dual_bolt_cell_modified`，但不能和原包一起构建。推荐复制到干净工作空间：

```bash
mkdir -p ~/fr3_modified_ws/src
cp -a ~/fr3-sim/sim_ws/src/fr3_dual_bolt_cell_modified \
  ~/fr3_modified_ws/src/
cd ~/fr3_modified_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-select fr3_dual_bolt_cell
source install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch fr3_dual_bolt_cell bringup.launch.py mode:=mock enable_execution:=true
```

接口定义集中在：

```text
~/fr3_modified_ws/src/fr3_dual_bolt_cell_modified/urdf/my_robot.ros2_control.xacro
```

默认插件是 `mock_components/GenericSystem`。真机只替换 FR3/HKV 插件和硬件参数，不修改机械模型。夹爪只控制左手指：

```text
<side>_gripper_left_finger_joint   prismatic, X, 0~0.06 m
<side>_gripper_right_finger_joint  mimic, multiplier="-1"
```

## 6. `fr3_real_bringup` mock

```bash
source /opt/ros/humble/setup.bash
source ~/fr3-sim/sim_ws/install/setup.bash
export ROS_DOMAIN_ID=32
ros2 launch fr3_real_bringup mock.launch.py
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 run fr3_real_bringup check_feedback --seconds 5
```

mock 不连接 FR3 控制柜，也不打开 HKV 串口。

## 7. 真机准备和启动

真机需要匹配的法奥驱动、控制柜 IP、左右独立串口、固件版本、TCP/负载、安装位姿和现场安全检查。先编译独立驱动工作空间：

```bash
mkdir -p ~/fr3_dual_driver_ws/src
# 将适配后的 fairino_hardware_v3_9_7、fairino_msgs、ros2_hkv_gripper
# 放入 ~/fr3_dual_driver_ws/src/
cd ~/fr3_dual_driver_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-up-to fairino_hardware_v3_9_7 ros2_hkv_gripper
source install/setup.bash
```

先只验证反馈：

```bash
source /opt/ros/humble/setup.bash
source ~/fr3_dual_driver_ws/install/setup.bash
source ~/fr3-sim/sim_ws/install/setup.bash
export ROS_DOMAIN_ID=32
ros2 launch fr3_dual_bolt_cell bringup.launch.py mode:=real \
  hardware:=$HOME/fr3_dual.hardware.yaml enable_execution:=false
```

另开终端：

```bash
ros2 control list_controllers -c /left_controller_manager
ros2 control list_controllers -c /right_controller_manager
ros2 control list_hardware_interfaces -c /left_controller_manager
ros2 control list_hardware_interfaces -c /right_controller_manager
ros2 topic echo /joint_states --once
ros2 run fr3_dual_bolt_cell check_feedback --timeout 30
```

`enable_execution:=false` 只禁止 MoveIt 执行，不保证厂商驱动完全停止保持指令；它不是急停或物理只读模式。首次真实运动必须低速、空载，并确认现场急停有效。

## 8. 常见问题和清理

重复包名：把修改版放到 `~/fr3_modified_ws/src`，不要和原包一起放入同一个构建源空间。

找不到 Xacro 或控制器：

```bash
source /opt/ros/humble/setup.bash
sudo apt install -y ros-humble-xacro ros-humble-position-controllers \
  ros-humble-joint-trajectory-controller ros-humble-gazebo-ros2-control
```

重新构建单个包：

```bash
cd ~/fr3-sim/sim_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-select fr3_dual_bolt_cell
source install/setup.bash
```
