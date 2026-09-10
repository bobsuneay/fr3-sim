# FR3 双臂螺栓工作台：Gazebo / mock / 实机

`fr3_dual_bolt_cell` 是参照 `fr3_bolt_cell` 新建的独立 ROS 2 功能包。
它把原来立柱右侧的法奥 FR3 扩展成左右两台 FR3，左右各装一只 HKV TG-9801，
保留头部固定 RGB-D 相机、720 mm 桌面和 20 个总长 25 mm 的动态螺栓。
原有三个功能包不需要修改，也不要与本包同时启动。

初始姿态关于身体中线所在的 **XZ 平面（Y=0）** 镜像：左右关节中心满足
`(x,y,z) → (x,-y,z)`，工具 +Z 朝向也按该平面反射。配置中的左右关节角
不是简单正负取反；FR3 固定关节变换和腕部偏置要求第二、第四轴包含 π 偏移。
当前左右 TCP 约为 `[0.336058, ±0.551093, 0.853675]` 米，仅在 gazebo/mock 初始化时使用；
real 仍读取实机当前状态，不因更新初始配置而自动移动。

连接板和 HKV 外观现在均有显式灰色／金属色材质。之前持续红色的原因包括材质缺失：
[RViz Humble 源码](https://github.com/ros2/rviz/blob/humble/rviz_default_plugins/src/rviz_default_plugins/robot/robot_link.cpp)
在 `getMaterialForLink()` 中对缺失材质使用 `RVIZ/ShadedRed`，颜色本身不能证明碰撞。
本次没有新增碰撞排除或修改 MoveIt 的碰撞警告颜色。若更新后某个规划目标仍高亮红色，
需要检查该目标的实际碰撞或有效性，不能通过隐藏红色来判断安全。

更新后须完整停止旧 launch 并重新启动，才能加载新的 URDF 材质、SRDF ready 状态和初始角度。
如果启动时传了自定义 `arms:=...`，该文件仍会覆盖本包默认值，需同步其左右 `initial` 数组。

**用途：给双臂抓取、搬运、检测、交接提供统一的模型、规划、控制和实机接入基础。**
本包可在 RViz 中分别规划左右臂，或用 `both_arms` 做双臂联合关节规划。
它还不是自动 pick-and-place 程序：目标识别、抓取位姿、工件附着/释放、任务状态机、
双臂交接和失败恢复仍需在这个基础上实现。没有用虚拟吸附替代物理抓取。

## 一个开关，三种后端

目标平台沿用原项目：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。
必须先停止旧 launch 再切换；这不是运动中的热切换。

```bash
# Gazebo 物理仿真，允许 RViz 执行
ros2 launch fr3_dual_bolt_cell bringup.launch.py mode:=gazebo enable_execution:=true

# 无 Gazebo 的虚拟控制器，检查 MoveIt/控制器链路
ros2 launch fr3_dual_bolt_cell bringup.launch.py mode:=mock enable_execution:=true

# 现场完成配置和驱动编译后，启动实机控制
ros2 launch fr3_dual_bolt_cell bringup.launch.py mode:=real \
  hardware:=$HOME/fr3_dual.hardware.yaml enable_execution:=true
```

省略参数时是 `mode:=gazebo enable_execution:=false`，运动控制器处于 inactive。
实机模式必须先填写硬件配置并设置 `commissioned: true`；缺失 IP、重复 IP、串口重复、
固件不匹配或未安装适配过的驱动都会在启动前报错。
`enable_execution:=false` 禁止 MoveIt 执行且不激活运动控制器，
**但实机驱动自身仍可能发送 ServoJ 保持或夹爪保持命令，绝不是只读连接。**

## 安装和仿真

把本文件夹放进 Ubuntu 上 `~/fr3-sim/sim_ws/src/`，与 `fr3_bolt_cell` 并列。
以下命令在 Ubuntu 执行；Windows 目录不能直接运行 ROS/Gazebo。

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-yaml \
  python3-numpy python3-pytest ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-moveit ros-humble-rviz2
cd ~/fr3-sim/sim_ws
# rosdep 未初始化时先执行一次 sudo rosdep init
rosdep update
rosdep install --from-paths src/fr3_dual_bolt_cell --ignore-src -r -y --rosdistro humble
/usr/bin/python3 /usr/bin/colcon build --symlink-install --packages-select fr3_dual_bolt_cell
source install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch fr3_dual_bolt_cell bringup.launch.py mode:=gazebo enable_execution:=true
```

退出 Conda 后加载 ROS，避免系统 ROS 模块被 Conda Python 遮蔽。
仿真和实机使用不同 ROS_DOMAIN_ID；同一会话的所有终端必须一致。
本包自带模型资源，不依赖原 `fr3_bolt_cell` 的安装路径。

第二个终端加载相同环境：

```bash
source /opt/ros/humble/setup.bash
source ~/fr3-sim/sim_ws/install/setup.bash
export ROS_DOMAIN_ID=31
ros2 run fr3_dual_bolt_cell check_feedback --timeout 30
ros2 control list_controllers -c /controller_manager
ros2 run fr3_dual_bolt_cell gripper --arm left --width 0.030
ros2 run fr3_dual_bolt_cell gripper --arm right --width 0.010
```

`--width` 是内侧净开口，单位米；默认范围 0～30 mm。所有后端都使用同一个
单关节 `GripperCommand`，目标直接发送到 `/<side>_gripper_controller/command`。
real 后端再由 `ros2_hkv_gripper/GripperHardwareInterface` 转换为现场配置的寄存器/力参数。
例如：

```bash
ros2 run fr3_dual_bolt_cell gripper --arm left --width 0.020
```

所有后端的 URDF 都只导出 `left_gripper_left_finger_joint`/
`right_gripper_left_finger_joint` 两个夹爪硬件关节；对应的
`gripper_right_finger_joint` 是 `multiplier="-1"` 的 mimic 关节。
夹爪 CAD 只用于外观，碰撞体、质量、开口、TCP 是可调整的近似值，需现场测量。
实机夹持力使用硬件配置的 `target_force_percent`；位置控制器的 action `max_effort`
不能当作已标定的实际夹持力。action 返回 stalled 也不证明成功夹住工件。

RViz 的规划组：`left_arm`、`right_arm`、`both_arms`、`left_gripper`、`right_gripper`。
`both_arms` 支持联合关节目标；两只末端同时指定笛卡尔目标需要上层程序构造约束，
不是本包自动完成的交接动作。两台控制柜的同步也不是硬件级同步。

## 文件夹内容与接口

| 路径 | 用途 |
| --- | --- |
| `launch/bringup.launch.py` | 模式切换、驱动校验、按控制器成功事件启动 MoveIt/RViz |
| `config/arms.yaml` | 左右基座位姿、仿真初始角度、夹爪行程、法兰和 TCP |
| `config/scene.yaml` | 立柱、头部相机、桌面、螺栓、物理参数；旧 `mount` 项仅保留格式兼容，实际安装位姿读 arms.yaml |
| `config/hardware.example.yaml` | 两台控制柜 IP、固件、两个串口、夹爪寄存器/速度/力 |
| `fr3_dual_bolt_cell/model.py` | 生成机械模型、SRDF 和 MoveIt 配置，并调用独立 ros2_control Xacro |
| `urdf/my_robot.ros2_control.xacro` | 单独描述左右 FR3/HKV 的 ros2_control 接口；默认 mock，可替换插件和硬件参数 |
| `urdf/`、`meshes/` | 共用立柱/相机 Xacro、FR3 原始连杆和 HKV 外观资源 |
| `fr3_dual_bolt_cell/world.py` | 根据 scene.yaml 生成 Gazebo 桌面和独立动态螺栓 |
| `fr3_dual_bolt_cell/planning_scene.py` | 将同一桌面尺寸加入 MoveIt 碰撞场景 |
| `tools/prepare_driver.py` | 复制并适配指定版本的厂商驱动，生成审阅 diff，不改输入目录 |
| `test/` | 离线模型、接口、配置拒绝、几何和驱动适配测试 |
| `docs/COMMISSIONING.md` | 实机准备、编译、验证与排错 |
| `docs/VALIDATION.md` | 本次实际验证范围与尚未执行的运行验收 |

| 接口 | 用途 |
| --- | --- |
| `/left_arm_controller/follow_joint_trajectory` | 左臂六轴轨迹 action |
| `/right_arm_controller/follow_joint_trajectory` | 右臂六轴轨迹 action |
| `/left_gripper_controller/command`、`/right_gripper_controller/command` | 所有后端统一的 GripperCommand action |
| `/joint_states` | 两个互不重名的关节子集，合计 12 个臂关节和 2 个夹爪主关节 |
| `/left/gripper_registers`、`/right/gripper_registers` | 实机 HKV 插件寄存器反馈 |
| `/head_camera/image_raw`、`/head_camera/points` | 仅 Gazebo 自动提供；实机需另接真实 RGB-D 驱动与标定 |

Gazebo 只有一个 `/controller_manager`。mock/real 用独立进程
`/left_controller_manager` 和 `/right_controller_manager`，隔离厂商 SDK 与串口线程。
全局只有一个 robot_state_publisher 和一个 move_group。
三种模式的关节名、规划组和 action 地址相同，业务程序不必随模式改写。

## 本次优化与边界

- 左右前缀统一生成，实机 FR3 插件严格只接收各自按 j1～j6 排序的六轴；夹爪不会成为第七轴。
- 保留双臂之间的碰撞检查；只排除机械相邻/固定安装接触，不把所有碰撞关闭。
- 仿真和 MoveIt 桌面来自同一份配置；临时 URDF/控制器/world 隔离生成，退出清理。
- 控制器加载失败停止启动流程；必要 ROS 节点退出会结束本次 launch。ROS 退出不能替代硬件急停。
- 实机适配 IP 参数、六轴数量检查、反馈错误传播、第六轴有限值检查、初始状态同步和指针释放。
- 默认较低规划速度/加速度；真实速度、负载、TCP、安装角度和工作空间仍需在控制柜侧设置。

螺栓是动态 Gazebo 刚体，没有作为固定位置的 MoveIt 障碍物发布；真实/自动抓取前必须接入感知与工件状态管理。
Gazebo Classic 对小零件接触的稳定性、HKV 模型匹配程度和实机通信尚待现场验收。
本包提供的实机接入代码不等于已经在真实双臂上验证可用。

参考用户提供的[法奥机器人 ROS2 环境搭建文章](https://blog.csdn.net/2301_78767880/article/details/147323555)
中的固件匹配、Humble 与 MoveIt 联动步骤；具体接口以本次提供的厂商源码为准。
另核对了 [Humble 夹爪控制器源码](https://github.com/ros-controls/ros2_controllers/tree/humble/gripper_controllers)
和 [Gazebo ros2_control 文档](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html)。
