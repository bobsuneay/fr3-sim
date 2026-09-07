# 验证状态

## 已完成的静态检查

- 独立包目录和安装数据文件已创建。
- URDF 使用正常水平桌面、正装基座和 HKV 工具；没有 bolts/camera/Gazebo 元素。
- ros2_control 只声明 FR3 六个位置命令/位置状态接口。
- real 配置默认不能通过确认门槛，避免未知固件、IP 或驱动被误启动。
- 检查工具只订阅 /joint_states，不产生运动指令。

## 尚未在本机完成

当前开发环境是 Windows，没有 Ubuntu 22.04、ROS 2 Humble、Gazebo/MoveIt 运行时，
因此没有声称已经完成 colcon、RViz、controller_manager 或实机运动测试。
你需要在 Ubuntu 上按 COMMISSIONING.md 构建并逐项验收。

## 第三方驱动风险

官方不同固件对应不同硬件包；有些版本把 IP 写在源码里。选定版本后必须审查：
六轴名称/顺序、rad/deg 转换、激活时初始角同步、ServoJ 周期、错误返回、
SDK 二进制架构和停止行为。不能仅因为 /joint_states 有数据就认为实机安全。
