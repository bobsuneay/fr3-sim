# FR3 仿真工作空间

## 当前推荐：真实 FR3 + Gazebo 螺栓工作台

新入口为 [`fr3_bolt_cell`](src/fr3_bolt_cell/README.md)，采用用户提供的法奥 FR3 原始模型，搭建：侧装型材支架、可开合平行夹爪、固定头部 RGB-D 相机、桌面、20 个独立动态小螺栓，以及匹配的 ros2_control / MoveIt 2 / RViz 配置。

目标平台：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。完整安装、编译、启动、相机与夹爪验收命令见[实施文档](src/fr3_bolt_cell/docs/Ubuntu22.04.md)。

已编译并 source 本工作空间后：

```bash
export ROS_DOMAIN_ID=31
ros2 launch fr3_bolt_cell gazebo.launch.py
```

新包只做仿真，不连接实际法奥控制柜。此次是在 Windows 上生成并离线检查，Ubuntu 的实际运行还需按[验收清单](src/fr3_bolt_cell/docs/VALIDATION.md)完成。

## 保留的旧原型

`fairino_dualarm_sim` 保留不改，供阅读早期双臂状态机结构。它使用简化模型、插值关节状态和模拟视觉，不是本次真实 FR3 的 Gazebo/ros2_control 入口。不要同时运行旧包与新包。

旧 Gazebo 描述中的工具顶部相机、固定螺栓，与这次的固定头部视角和独立自由物体要求不一致，因此不再推荐该启动入口。自由螺栓应使用独立 SDF 动态模型，不能通过简单把 URDF fixed joint 改成 floating 就视为可靠的物理抓取仿真。

本目录的历史阶段文档只作为设计背景；本次包内 README、配置和实施文档是当前版本的使用依据。
