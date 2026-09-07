# FR3 桌面正装真机调试包

本目录是独立 ROS 2 Humble 包：桌面正装 FR3、HKV TG-9801 平行夹爪和 MoveIt 2。
原有 fr3_bolt_cell 保持不变。本包不启动 Gazebo，不包含螺栓、相机或侧装立柱。

夹爪当前作为固定工具显示，碰撞盒覆盖其完整开合范围；不加载夹爪串口驱动，
也不伪造夹爪反馈。六轴 FR3 通过 fairino_hardware/FairinoHardwareInterface
接入 ros2_control，实际驱动必须由你根据固件版本选择。

## Mock 预览

    source /opt/ros/humble/setup.bash
    cd ~/fr3-sim/sim_ws
    colcon build --symlink-install --packages-select fr3_real_bringup
    source install/setup.bash
    ros2 launch fr3_real_bringup mock.launch.py

MoveIt 规划组是 fairino3_v6_group，末端是 gripper_tcp。mock 模式只动虚拟模型。

## 真机前

完整步骤见 docs/COMMISSIONING.md。必须由现场人员填写固件版本、控制柜 IP、驱动包、
桌面/基座尺寸、TCP、负载和安全检查。enable_execution:=false 只禁止 MoveIt 轨迹执行，
不代表驱动只读；部分官方插件启动后会持续发送 ServoJ 保持指令。ROS 退出和 RViz Stop 不是急停。

主要文件：

- config/cell.yaml：桌面、正装基座、工具变换、碰撞包围盒
- config/real.example.yaml：真机人工验收配置
- launch/mock.launch.py：虚拟完整链路
- launch/bringup.launch.py：mock/real 通用入口
- docs/VALIDATION.md：当前验证范围

新编写代码为 MIT；第三方 FR3/HKV 网格的来源和许可见 THIRD_PARTY.md。
