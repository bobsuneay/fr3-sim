# 双臂实机接入

以下流程是现场操作文档。本次没有连接任何机器人，没有发送机械臂或夹爪运动指令。

## 1. 驱动版本与网络

先分别在左右控制柜 WebApp 的“系统设置 → 关于”核对软件版本。
本次驱动适配仅支持用户提供的 `fairino_hardware_v3_9_7` 对应 3.9.7 源码。
如果某台机器人版本不同，不能只改配置中的版本字符串绕过校验；需评审对应版本接口。

提供的驱动在头文件硬编码 `192.168.58.2`，只加 ROS 命名空间不会改变它。
本包工具改为从 ros2_control 的 `robot_ip` 参数读取，并且缺少参数就拒绝初始化。
每臂单独的控制管理进程只加载一个 FR3 系统和一个夹爪系统，FRRobot 库的进程内状态彼此隔离。

同一主机上的两个控制柜应配置不同 IP，电脑网卡应能直接访问二者。
若两个控制柜保持相同默认 IP，须先解决网络隔离/路由问题；本入口明确拒绝相同 IP。
用虚拟机时配置可访问控制柜的桥接网络，并把两个 USB 串口分别直通到 Ubuntu。

## 2. 编译独立的驱动工作区

不要把官方归档的所有 `fairino_hardware*` 一起编译和 source；它们可能导出相同插件类和库名。
下面用 `~/vendor/frcobot_ros2-v3.0.0_robotV3.9.7` 表示在 Ubuntu 解压后的厂商目录，按实际路径替换。

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/fr3_dual_driver_ws/src
python3 ~/fr3-sim/sim_ws/src/fr3_dual_bolt_cell_modified/tools/prepare_driver.py \
  --source ~/vendor/frcobot_ros2-v3.0.0_robotV3.9.7/fairino_hardware_v3_9_7 \
  --destination ~/fr3_dual_driver_ws/src/fairino_hardware_v3_9_7
cp -r ~/vendor/frcobot_ros2-v3.0.0_robotV3.9.7/fairino_msgs ~/fr3_dual_driver_ws/src/
cp -r ~/vendor/ros2_hkv_gripper ~/fr3_dual_driver_ws/src/

# 审阅生成的补丁，然后在此工作区编译
cat ~/fr3_dual_driver_ws/src/fairino_hardware_v3_9_7/dual_cell_adapter.diff
cd ~/fr3_dual_driver_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-up-to fairino_hardware_v3_9_7 ros2_hkv_gripper
source install/setup.bash
```

工具只接受匹配 SHA256 的原始接口文件，生成全新目标目录；目标已存在时拒绝覆盖。
再次使用时选择新目标目录或自行保留并处理旧目录，不要对未知版本强行替换代码。
补丁保留厂商 `.008 s` ServoJ 周期，本包实机管理器使用 125 Hz；实际周期抖动需在 Ubuntu 测量。
本次没有编译过厂商 C++/SDK，也未验证实时性与通信断开时的真实停机行为。

如果厂商 SDK 链接失败，核对其 `libfairino.so.2.3.7` 是否完整，以及 `ldd` 输出是否存在 `not found`。
这是提供归档中的预编译厂商库，不应随意用其他版本的 `.so` 替换。

## 3. 配置现场数据

```bash
cp ~/fr3-sim/sim_ws/src/fr3_dual_bolt_cell_modified/config/hardware.example.yaml ~/fr3_dual.hardware.yaml
cp ~/fr3-sim/sim_ws/src/fr3_dual_bolt_cell_modified/config/arms.yaml ~/fr3_dual.arms.yaml
cp ~/fr3-sim/sim_ws/src/fr3_dual_bolt_cell_modified/config/scene.yaml ~/fr3_dual.scene.yaml
```

填写两个 `robot_ip`，两个实际 `serial_port`，优先使用 `/dev/serial/by-id/...` 固定设备名。
把账户加入 `dialout` 并重新登录，以获得串口访问权限。
两个串口名即使不同，解析成同一设备时也会被拒绝。

测量立柱和桌面相对位置、左右基座六自由度安装位姿、法兰转接、TCP、夹爪净开口/有效行程。
`arms.yaml` 的初始角度只供 gazebo/mock；real 模式从控制柜读当前角度，不自动回 ready。
默认 HKV 行程仅用于模型，不能替代真实测量；默认输入寄存器张开值 100、闭合值 0 来自提供源码。
确认夹爪型号/固件的反馈范围与此一致，再设定速度、力百分比。

所有 ros2_control 声明集中在 `urdf/my_robot.ros2_control.xacro`。默认插件是
`mock_components/GenericSystem`；`bringup.launch.py mode:=real` 只为该片段替换
FR3/HKV 插件和硬件参数，机械 URDF、碰撞模型和 MoveIt 结构不需要复制或修改。

控制柜配置要与模型相符：侧装重力方向、负载、工具中心、工作空间、速度限制。
现场核对急停可用、工作区域清空、两臂与安装架/桌面不干涉后，将硬件文件的 `commissioned` 改为 `true`。
这个字段只是启动配置记录，不代替风险评估，也不证明上述物理条件已被软件检测。

## 4. 先反馈，再执行

新终端仅加载这一套驱动 underlay 和本项目 overlay：

```bash
source /opt/ros/humble/setup.bash
source ~/fr3_dual_driver_ws/install/setup.bash
source ~/fr3-sim/sim_ws/install/setup.bash
export ROS_DOMAIN_ID=32
ros2 launch fr3_dual_bolt_cell_modified bringup.launch.py mode:=real \
  hardware:=$HOME/fr3_dual.hardware.yaml arms:=$HOME/fr3_dual.arms.yaml \
  scene:=$HOME/fr3_dual.scene.yaml enable_execution:=false
```

另一个相同环境的终端：

```bash
ros2 run fr3_dual_bolt_cell_modified check_feedback --timeout 30
ros2 control list_controllers -c /left_controller_manager
ros2 control list_controllers -c /right_controller_manager
ros2 control list_hardware_interfaces -c /left_controller_manager
ros2 control list_hardware_interfaces -c /right_controller_manager
ros2 topic echo /joint_states --once
```

应看到各臂 broadcaster active、运动控制器 inactive，12 个臂关节和 2 个夹爪主关节持续更新。
FR3 插件只导出六个 position 关节；每个 HKV 插件只导出对应的
`<side>_gripper_left_finger_joint`，右手指通过 URDF mimic 关节跟随，并提供
position/velocity 状态。
真机夹爪控制器是 `position_controllers/GripperActionController`，action 地址为
`/<side>_gripper_controller/command`；不要向真机发送仿真双指轨迹的四个 finger joint。
将 RViz 当前姿态与实物逐轴比对，确认左右连接与关节方向正确。
`check_feedback` 只检查 ROS 层新鲜、有限、重复的消息，不保证 SDK 未缓存旧数据，也不验证控制柜安全状态。

注意：即使上述 `enable_execution:=false`，驱动可能持续发送保持指令。
如只想离线预览，使用 `mode:=mock`。

确认反馈后停止本次 launch，保持相同配置重启，并将 `enable_execution` 设为 `true`。
先在低速下分别做小幅单臂运动和空载夹爪开合，再做双臂联合规划。
不要直接用 `ready` 作为未检查路径的实机回零命令。
停止 launch、RViz Stop 或 action cancel 都不能替代现场硬件急停。

## 5. 上层应用衔接

业务层通过统一 MoveIt 规划组和四个 action 控制后端，不应直接拼接厂商 ServoJ 指令。
联合规划采用统一的双臂碰撞模型；MoveIt 分发到两台控制柜并不保证硬件级严格同步。
精确交接需要进一步做控制时序、速度、夹持确认、共享工作区互锁和通信失效处理。
实时相机要自行启动匹配的硬件驱动，标定到 `world` 或 `head_camera_link`；本包没有猜测相机型号。
