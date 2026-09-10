# FR3 双臂螺丝抓取与多视角检测

这是基于 `fr3_dual_bolt_cell` 新建的独立 ROS 2 功能包。它复用原包的双臂模型、HKV 夹爪、控制器、MoveIt 配置和桌子，在自己的 launch 中增加点云识别、三个相机和完整任务流程。原包的启动方式不受影响；构建时必须同时保留原包。

**本版定位：Gazebo Classic / ROS 2 Humble 的第一版流程实现，含操作 UI。Windows 上已做离线测试，尚未在 ROS 2/Gazebo 中完成编译和整流程运行验收。不能把本版当成已经验证的实机抓取程序。**

## 完成什么

1. 原头部相机输出真实的仿真 PointCloud2。按标定桌面高度和 ROI 去掉背景，用欧式聚类找到单颗螺丝，PCA 求水平主轴，比较两端宽度识别螺丝头；连续三帧稳定才开始。
2. 默认右手张开，移动到螺丝上方 50 mm，以约 8 mm/s 的保守分段速度下降，闭合原始 HKV 夹爪，再抬高 50 mm。
3. 将螺丝移到腰部检测相机前下方。围绕**估计的零件中心**计算一系列姿态，不是只原地转第六轴；转动时保持中心位置，停稳后保存三路 RGB/深度图和相机外参。
4. 右手回到交接基准姿态，左手从相反方向靠近螺丝另一段，闭合并确认接管后，右手才松开、撤离、回到初始姿态。
5. 左手重复多视角检测，结束后保持夹持，状态为 `DONE_HOLDING_LEFT`。本版不自动松手、不循环、不自动放回桌面。

由于侧装 FR3 的关节范围、夹爪体积和双臂碰撞约束，18 个候选视角不一定全部可达。每个视角先完整规划，失败则记录原因；运动执行或夹持核验失败会终止任务。每只手至少要成功拍摄 3 个视角，否则报告失败。拍摄张数不等于六面完整覆盖，也不等于已完成表面缺陷检测或点云重建。

## 相机和夹爪

| 相机 | 安装位置与用途 | RGB / 深度 / 点云 |
|---|---|---|
| `head_camera` | 保留头部 RGB-D，用于桌面螺丝定位 | `/head_camera/image_raw`、`/head_camera/depth/image_raw`、`/head_camera/points` |
| `waist_camera` | 身体前侧，高 1.22 m；朝向前下方的检测区域 | 同名前缀的 `image_raw`、`depth/image_raw`、`points` |
| `left_d435i` | 左夹爪侧面支架 | 同上 |
| `right_d435i` | 右夹爪侧面支架 | 同上 |

两个腕部 D435i 使用约 90 × 25 × 25 mm 外壳、独立光学 TF 和 Gazebo 深度相机传感器，距离抓取中心约 15 cm。**这是外形/视场的简化仿真，不包含真实双目噪声、红外干扰、IMU或 RealSense USB 驱动。** 腰部相机使用可配置的通用 RGB-D 模型，不预设真实镜头型号。

所有新增视觉几何都有显式颜色；相机、支架和夹指都有碰撞几何。双臂沿用关于身体中线 `Y=0` 的镜像初始姿态，初始夹爪张开。

本包恢复原始 HKV 夹爪形态，不再生成沿螺丝方向 4 mm 宽的简化窄指尖，也不再用包围整个手指的长方体碰撞代理。HKV 的 `flange`、`base_body`、`rail_155`、`slider` 和 `finger` 网格同时作为 RViz/Gazebo 视觉和 MoveIt 碰撞几何，碰撞外形与显示外形一致。原始夹爪较宽，两只手同时夹持 25 mm 螺丝的可达性和夹持空间必须在仿真中重新确认。

## 构建和使用

在已有 **Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 + MoveIt 2** 环境中，从仓库的 `sim_ws` 目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/fr3_dual_bolt_cell src/fr3_bolt_inspection_cell --ignore-src -r -y
colcon build --symlink-install --packages-up-to fr3_bolt_inspection_cell
source install/setup.bash
colcon test --packages-select fr3_bolt_inspection_cell
colcon test-result --verbose
```

如果是从 2026-09-09 首次版本更新（日志中出现 `Skipping joint ... finger_joint` 或
`inspection_task not found`），先清掉这个包的旧构建产物再重建：

```bash
cd ~/fr3-sim
git pull
cd sim_ws
rm -rf build/fr3_bolt_inspection_cell install/fr3_bolt_inspection_cell
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to fr3_bolt_inspection_cell
source install/setup.bash
ros2 launch fr3_bolt_inspection_cell bringup.launch.py mode:=gazebo enable_execution:=true
```

不要删除整个 `install` 目录；上述命令只清理新功能包。更新后可先确认两个入口存在：

```bash
ls -l install/fr3_bolt_inspection_cell/lib/fr3_bolt_inspection_cell/
```

先预览模型，不执行任务：

```bash
ros2 launch fr3_bolt_inspection_cell bringup.launch.py mode:=mock
```

启动完整仿真与操作面板：

```bash
ros2 launch fr3_bolt_inspection_cell bringup.launch.py mode:=gazebo enable_execution:=true
```

控制器动作的超时按 `/clock` 仿真时间计算。四个深度相机使 Gazebo 实时率低于 1 时，机械臂会继续等待轨迹完成，不会因电脑上的墙上时间先到而在 `APPROACH` 后误报失败；如果 `/clock` 连续 30 秒完全不前进，任务仍会停止并保持夹持。终端每 10 秒会打印一次控制器进度。

每次笛卡尔规划都会打印请求目标、步长、跳变阈值、碰撞检查开关，以及返回的 `error_code`、路径比例、轨迹点数和最大关节步长。当前 `cartesian_step=0.003 m`、`joint_step_limit=0.20 rad` 是为默认俯抓位姿放宽的仿真参数；路径仍必须达到 100% 且保持碰撞检查，部分路径不会执行。

等待控制器和 MoveIt 启动，在面板中点击 **开始单次完整任务**。面板可切换四路相机图像，显示当前阶段，并提供 **停止运动并保持夹持**。启动 launch 不会自动运动。停止是 ROS 动作取消，不是实机急停；本版没有实机连接。

不使用面板时加 `panel:=false`，通过服务运行：

```bash
ros2 service call /inspection/start std_srvs/srv/Trigger '{}'
ros2 topic echo /inspection/status
ros2 service call /inspection/stop std_srvs/srv/Trigger '{}'
```

关闭面板只关闭窗口，后台任务仍在运行。需要停止时先点击停止按钮或调用停止服务。一次任务完成或失败后会锁定，检查日志后重启整个仿真才能再次运行，避免对仍被夹持的零件重复抓取。不要同时在 RViz 中发送执行命令或另开夹爪控制程序。

## 配置入口

主要参数在 [`config/inspection.yaml`](config/inspection.yaml)：

| 参数 | 默认值/含义 |
|---|---|
| `cloud_topic`、`roi_min/max` | 桌面识别输入和工作区域；默认只有一颗螺丝 |
| `approach_height`、`lift_height` | 0.05 m |
| `descent_speed` | 0.008 m/s；路径逐点停靠，实际平均速度更低 |
| `inspection_center`、`handover_center` | `[0.25, 0, 1.15]`，世界坐标，单位米 |
| `views_deg` | 绕零件局部 X/Y/Z 的角度，单位度；每次拍摄后回基准姿态 |
| `center_tolerance` | 0.003 m，规划路径中心误差阈值；仿真运动中也检查中心漂移 |
| `minimum_views` | 每只手至少 3 个成功视角 |
| `cameras` | 相机安装外参、分辨率、视场和近远裁剪面 |
| `output_directory` | 默认 `~/fr3_inspection_runs` |

`scene.yaml` 管桌面、螺丝和头部相机，`arms.yaml` 管双臂初始关节角。更改螺丝位置时同步改 ROI；若想左手先抓，除了 `first_arm: left`，还应把桌上螺丝和 ROI 移到左侧并重新检查可达性。改变零件尺寸需要同步修改指尖、感知参数与 C++ 夹持校验，当前配置会拒绝直接换尺寸。

## 输出结果

每次任务建立独立时间目录，包含：

- `report.json`：阶段、失败原因、完成/不可达视角、估计位姿、中心误差。
- `detection.npz`：实际分割出的螺丝点云、估计变换矩阵和包围尺寸。
- `left_view_XX/`、`right_view_XX/`：每个相机的 RGB 预览 `.ppm`、原始 RGB/深度字节 `.npz`、内参、拍摄时的 `world_optical` 外参。
- `metadata.json`：图像编码、行跨度、大小端和时间戳。读取深度时按编码解析，例如 `32FC1` 是米，不能把保存的 uint8 字节直接当深度值。

RGB/深度对在同一相机内限制时间差 ≤50 ms，三个相机都要求停稳后的新帧；三相机不是硬件同步采集。没有自动调焦算法：这里通过减少零件位移来保持成像距离，实际镜头的景深仍需实验确认。

## 仿真夹持的实际边界

本包提供自己的 `libfr3_inspection_grasp.so`，无需安装第三方 link-attacher。它先检查当前夹指开口、螺丝杆部是否位于指尖之间以及轴向对齐，再建立固定关节；交接时先验证接收手，在同一仿真更新中切换唯一持有者。不会把螺丝瞬移到夹爪上。

这属于**明确的夹持辅助**，并非靠真实摩擦力完成抓取。MoveIt 的 attached collision object 只服务于碰撞规划，Gazebo 固定关节才承担演示夹持。Gazebo 真值只用于包围检查、交接确认和误差核验，抓取目标来自相机点云；不会用模型坐标替代感知。扫描期间核验误差会考虑初次感知中心与模型中心的固定偏差。

当前感知是“桌面高度先验 + 水平 PCA + 头尾宽度”的近似位姿，螺丝绕自身轴的角度无法由这种轴对称模型唯一确定。它不处理堆叠、严重遮挡、密集多颗螺丝，也不宣称得到高精度完整六自由度位姿。

新包只提供 `gazebo/mock`。实机需要接入真实三相机及手眼标定、夹爪位置/力反馈、真实夹持与交接确认，并重新验证碰撞、可达性和速度。本包不会把 Gazebo 的固定关节成功响应用于实机放手。

详细验证记录和尚需执行的验收步骤见 [`docs/VALIDATION.md`](docs/VALIDATION.md)。

## 参考接口

- [MoveIt Humble GetCartesianPath](https://github.com/moveit/moveit_msgs/blob/humble/srv/GetCartesianPath.srv)：完整路径比例、碰撞检查、步长和关节跳变限制。
- [Gazebo ROS camera 实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_camera.cpp)：RGB、深度、CameraInfo、PointCloud2 话题。
- [D435i 官方产品资料](https://www.realsenseai.com/products/depth-camera-d435i/)：外壳尺寸参考；此处不模拟全部传感器特性。
- 复用资产与许可证遵循原包 [`fr3_dual_bolt_cell/THIRD_PARTY.md`](../fr3_dual_bolt_cell/THIRD_PARTY.md)，不重复复制 CAD。
