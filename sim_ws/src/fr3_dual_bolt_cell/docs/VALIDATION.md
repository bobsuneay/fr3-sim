# 验证记录

本次在 Windows 上完成；没有可用的 Ubuntu/ROS 2/Gazebo 运行环境，没有连接真实机器人。

2026-09-09 当前修订：`pytest` 结果 **35 passed**（包含提供原始驱动的测试）；Python 语法检查通过。
工具目录的 Xacro 有一条 Python 弃用警告，未导致测试失败。
厂商适配修改可直接查看同目录 `driver_adapter.diff`。

本轮修正了此前的镜像检查：原检查只验证关节角符号，不能证明关于身体中线对称。
现在通过 URDF 正运动学检查全部关节中心、TCP 和工具朝向关于 `Y=0` 反射，坐标容差 `1e-8 m`。
左右 TCP 为 `[0.336058, ±0.551093, 0.853675] m`，保留同一厂商网格与原始六轴变换。
相同型号机械臂的铸件不是左右手版本，测试证明的是运动学姿态镜像，不把 CAD 网格反射成不存在的硬件。

新增检查：每个 visual 必须有显式材质，避免 RViz 缺省红色；默认初始姿态中所有非 SRDF 排除对，
分别用每个碰撞网格/碰撞体的局部包围盒，经 FK 转为定向包围盒，再用分离轴检查。
这些盒子包含实际碰撞几何，盒子分离可保守证明该姿态中的几何分离。
此检查覆盖安装板、机身、双臂所有非排除对以及左右夹爪，没有为通过测试新增碰撞豁免。
桌面/地面检查仍单独执行。没有运行真实 RViz、Gazebo 或全轨迹碰撞检测。
语法检查另修复了上次夹爪改动遗留的 `check_feedback` 括号缺失，恢复反馈验收命令的可解析性。

已执行的离线测试覆盖：

- Gazebo、mock、real 三种模式完整生成；32 个 link / 31 个 joint，无重名、环或断开的分支。
- Gazebo/mock 的 12 个 FR3 关节和 4 个独立手指关节；夹爪 position/velocity 状态接口。
- 每个实机管理器只包含本臂六轴硬件与一个夹爪，左右控制柜 IP 分别写入硬件参数。
- gazebo/mock 模型不含实机插件；real/mock 不含 Gazebo 控制/传感器插件。
- SRDF 的双臂规划组和跨臂碰撞检查，安装/相邻排除范围。
- 默认初始姿态的网格/碰撞体保守 AABB 检查：与地面、桌面、对侧臂不重叠。
- 所有碰撞连杆的惯量正定；模型 URI 对应的安装资源文件存在。
- 夹爪物理净开口方向、越界/NaN 拒绝、配置错误拒绝、20 个独立动态螺栓。
- 使用用户提供的 3.9.7 原始源码实际运行“复制后适配”，验证输入文件未修改；未知源码和现存目标被拒绝。

AABB 检查只覆盖默认初始姿态，不是全工作空间自碰撞、接触动力学或轨迹安全证明。
夹爪近似碰撞体通过测试也不表示 CAD 外观/TCP 已完成物理标定。

Ubuntu 复测：

```bash
cd ~/fr3-sim/sim_ws
export FR3_VENDOR_DRIVER=$HOME/vendor/frcobot_ros2-v3.0.0_robotV3.9.7/fairino_hardware_v3_9_7
PYTHONPATH=$PWD/src/fr3_dual_bolt_cell /usr/bin/python3 -m pytest src/fr3_dual_bolt_cell/test -q
colcon test --packages-select fr3_dual_bolt_cell
colcon test-result --verbose
```

未提供 FR3_VENDOR_DRIVER 时，与完整厂商归档有关的复制测试会跳过，其余离线测试仍可执行。

尚待在目标系统执行：colcon 构建、launch 真正加载控制器、Gazebo 接触稳定性、MoveIt 规划/执行、
运行中反馈验收、硬件驱动 C++/SDK 编译、实机限速运动、两串口并发、断线与急停处理、双臂同步。
因此交付状态是“代码与离线检查完成，目标系统和实机验收待执行”，不能描述为实机已验证。
