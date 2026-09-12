# FR3 双臂螺丝抓取与多视角检测

这是基于 `fr3_dual_bolt_cell` 新建的独立 ROS 2 功能包。它复用原包的双臂模型、HKV 夹爪、控制器、MoveIt 配置和桌子，在自己的 launch 中增加点云识别、三个相机和完整任务流程。原包的启动方式不受影响；构建时必须同时保留原包。

**本版定位：Gazebo Classic / ROS 2 Humble 的第一版流程实现，含操作 UI。Windows 上已做离线测试，尚未在 ROS 2/Gazebo 中完成编译和整流程运行验收。不能把本版当成已经验证的实机抓取程序。**

## 完成什么

1. 原头部相机输出真实的仿真 PointCloud2。按标定桌面高度和 ROI 去掉背景，用欧式聚类找到单颗螺丝，PCA 求水平主轴，比较两端宽度识别螺丝头；连续三帧稳定才开始。
2. 默认右手张开，先求抓取点 IK（最多 8 组种子），从抓取点向上验证 50 mm 路径，再规划到该路径上端的指定关节构型。接近成功后，反向执行这条已验证路径，以约 8 mm/s 的保守分段速度下降，闭合原始 HKV 夹爪，再抬高 50 mm。
3. 将螺丝移到腰部检测相机前下方。围绕**估计的零件中心**计算一系列姿态，不是只原地转第六轴；转动时保持中心位置，停稳后保存三路 RGB 图像（腕部相机另保存深度）和相机外参。
4. 右手回到交接基准姿态，左手从相反方向靠近螺丝另一段，闭合并确认接管后，右手才松开、撤离、回到初始姿态。
5. 左手重复多视角检测，结束后保持夹持，状态为 `DONE_HOLDING_LEFT`。本版不自动松手、不循环、不自动放回桌面。

由于侧装 FR3 的关节范围、夹爪体积和双臂碰撞约束，18 个候选视角不一定全部可达。每个视角先完整规划，失败则记录原因；运动执行或夹持核验失败会终止任务。每只手至少要成功拍摄 3 个视角，否则报告失败。拍摄张数不等于六面完整覆盖，也不等于已完成表面缺陷检测或点云重建。

## 相机和夹爪

Humble Gazebo 将从动指状态接口导出为 `*_right_finger_joint_mimic/position`。Gazebo 状态广播器使用此名称；任务节点和面板收到数据后映射回 URDF 关节名，保留实际测量值和接收时间，不用主动指数值伪造从动指反馈。mock 模式仍使用原始关节名。

每只 HKV 夹爪只有一个开合自由度。控制器只接收 `left_finger_joint` 的单关节目标（开口宽度的一半）；`right_finger_joint` 通过 URDF mimic 和 Gazebo ros2_control mimic 参数按 1:1 跟随。两关节轴方向相反，因此两指对称开合。任务仍读取两指反馈，联动位移差超过 1 mm 则报错。修改控制器和模型后需要重建并重启 launch。

| 相机 | 安装位置与用途 | RGB / 深度 / 点云 |
|---|---|---|
| `head_camera` | 保留头部 RGB-D，用于桌面螺丝定位 | `/head_camera/image_raw`、`/head_camera/depth/image_raw`、`/head_camera/points` |
| `waist_camera` | 身体前侧，高 1.22 m；朝向前下方的 RGB 拍照区域 | 仅 `image_raw`（不提供深度/点云） |
| `left_d435i` | 左夹爪侧面支架，可作为近距离 RGB-D 观测源 | `image_raw`、`depth/image_raw` |
| `right_d435i` | 右夹爪上侧支架；与左腕相机关于身体中线镜像，可作为近距离 RGB-D 观测源 | `image_raw`、`depth/image_raw` |

默认由 `head_camera` 发布的 `/head_camera/points` 完成桌面螺丝点云分割和位姿估计；需要近距离点云时可将 `point_cloud_camera` 切换为左/右腕部 D435i，并把 `cloud_topic` 改为对应点云来源。腰部工业相机只用于 RGB 拍照和视场检查，不能作为点云来源。两个腕部 D435i 使用约 90 × 25 × 25 mm 外壳、独立光学 TF 和 Gazebo 深度相机传感器，距离抓取中心约 15 cm。**这是外形/视场的简化仿真，不包含真实双目噪声、红外干扰、IMU或 RealSense USB 驱动。** 腰部相机使用通用 RGB 模型。

所有新增视觉几何都有显式颜色；相机、支架和夹指都有碰撞几何。双臂沿用关于身体中线 `Y=0` 的镜像初始姿态，初始夹爪张开。

当前待机姿态将双臂收在桌面作业区前方：右臂关节角为 `[2.0, -1.6, 1.6, -1.25, -0.93, 0.23] rad`，左臂按身体中线镜像生成，见 `config/arms.yaml`。TCP 约为 `(0.339, ±0.298, 0.963) m`；第五关节保持弯曲，减少从原来向两侧伸展的姿态转入作业区的需要。启动和重新夹取回位使用同一组配置。离线已检查关节限位、镜像关系、模型自碰撞和桌面碰撞；这些检查不能替代 Gazebo 中的完整路径规划验证。修改初始值后需要重启 launch。

首次下降高度由启动时读取的原始 `finger.stl` 顶点、URDF 安装变换及夹指开度计算。当前模型在竖直抓取时，桌面 `Z=0.720 m`、指尖间隙 `5 mm` 对应 TCP 高度约 `0.7227 m`。到位后再次使用实际关节反馈核对指尖最低点；不再通过抬高抓取目标来绕过逆解失败。关节闭合遇到杆部时允许停在几何接触开度，但仍必须通过辅助夹持校验和试抬确认；Gazebo 的固定关节辅助夹持不等于真实摩擦力验证。

仿真中四个夹指的实际位置由 Gazebo 插件以 30 Hz 发布到 `/inspection/sim/gripper_states`，任务和界面读取该反馈，避免将 `_mimic` 接口名送入 MoveIt，也不使用主动指位置代替从动指反馈。随机位置按钮在后台执行，界面显示 `RANDOMIZING`；只有读取到零件实际新位姿并核对后才算成功。

Gazebo 夹爪使用本包 `ContactSystem` 硬件插件：机械臂仍由原 GazeboSystem 控制，两个夹指从原插件的控制列表中移出。每只夹爪只接收一个开度目标；两个物理指由同一目标和同步反馈产生驱动力，以物理步频运行。只在生成模型时初始化位置，运动中不再调用 `SetPosition`。默认 `finger_max_force: 4.0` 限制每指驱动力为 4 N，`finger_max_speed: 0.006` 限制目标变化率为 6 mm/s（不是实际速度硬限制）。碰到零件时有限驱动力允许夹指停住，而不是强行达到空夹位置。该模型使用有限刚度模拟联动，偏差超过 1 mm 仍会中止；实机和 mock 后端不使用此仿真插件。

辅助夹持要求两指均在最近 0.15 秒仿真时间内接触到螺丝，且接触穿透深度不超过 1.5 mm；开度和杆部位置也必须符合几何校验。空夹、只有单指接触或明显穿透时不建立固定关节。日志 `Grasp check ... finger contact age` 显示两指接触距当前的时间；缺少接触会报 `Missing recent bolt contact on BOTH fingers`，随后按原流程清理、重试。固定关节辅助和试抬仍不能代替实机摩擦/力反馈验证。

更新此修复后必须重新编译 C++ 插件并完全退出旧 Gazebo 再启动：`colcon build --packages-select fr3_bolt_inspection_cell --symlink-install`，然后 `source install/setup.bash`。确认启动日志出现 `CONTACT GRIPPER: one target per pair`；否则新硬件插件没有加载。可用 `colcon test --packages-select fr3_bolt_inspection_cell` 运行包含驱动力上限、联动与接触证据的测试。还需在 Gazebo 实测空夹、桌上双指接触、试抬、交接和停止；当前 Windows 离线测试没有覆盖完整 ROS/Gazebo 插件加载。

本包保留原始 HKV 夹爪的视觉外形，但采用与 `fr3_dual_bolt_cell` 一致的稳定简化碰撞代理：法兰、主体、滑轨和手指滑块使用盒体，只有左右两根手指末端保留 `finger.stl` 形状碰撞。这样既保留真实指尖夹持轮廓，也避免完整 CAD 网格在法兰安装处产生数值抖动。演示螺丝已放大为总长 **45 mm**、杆径 **12 mm**、头部直径 **18 mm**、头长 **8 mm**。点云筛选和辅助夹持校验读取配置尺寸，质量与惯量由场景生成器重算；指尖离桌面仍为 5 mm。实际两手夹持的可达性和夹持空间仍需在仿真中重新确认。

为避免原始 CAD 网格之间的微小接触让 Gazebo 夹爪数值振荡，两个活动手指在仿真中关闭 `selfCollide`，使用较低刚度、较高阻尼的接触参数，并将关节阻尼/摩擦提高到 `15.0/0.40`；这只影响仿真手指的数值稳定性，不改变视觉网格、MoveIt 碰撞网格或夹持服务的几何门限。每次夹爪动作结束后，任务节点会记录目标开口和两个实际关节位置。

## 构建和使用

接近与下降现在保留并执行同一组已验证的阶段轨迹。接近结束后会检查关节反馈和当前场景碰撞，再执行保存的下降轨迹，避免预检成功后又重新求解失败。参考代码、适配范围及测试见 [抓取规划参考](docs/REFERENCE_PICK_PLANNING.md)。

若日志在 `DESCEND` 显示 `Cartesian path incomplete`，表示 MoveIt 没有规划出完整下降段，任务此前不会发送任何下降轨迹。现在单目标笛卡尔段会尝试逐点逆解备用规划：步长不超过 1 mm，每次使用上个解作为种子，逆解超时 1 秒（兼容只读取整秒的 Humble 实现）；在重采样前检查关节跳变，同时检查关节插值中的全机器人碰撞和 TCP 偏离。局部 IK 失败后会移除种子关节窗口重试一次，精确目标姿态、碰撞检查和原始关节跳变上限保持不变。完整通过后才发送轨迹。抓取前从精确抓取姿态向上推导接近构型，避免接近位置的姿态容差引入额外腕部转动。失败日志会指出采样位置、逆解错误码、诊断解与种子的最大关节差，或 `link <-> object` 碰撞对；不会执行部分路径。诊断阶段可能临时请求不带碰撞检查的逆解来查明碰撞对象，该解仅用于诊断，不用于执行。

下降执行完成后新增 `CHECK_GRASP_REACH`：从新鲜关节反馈计算实际 TCP，日志显示目标、实际位置、高度差和姿态误差。位置误差不超过 3 mm、姿态误差不超过 0.12 rad 才闭合。若仅有不超过 25 mm 的未完成下降，且横向与姿态误差在容差内，可完整规划并执行一次补降；仍未到位则保持张开并报错。参考本地 FR3 示例的具体取舍见 [抓取规划参考](docs/REFERENCE_PICK_PLANNING.md)。补降不能绕过接近阶段的规划失败。

闭合后不会立即进入拍摄。任务先检查夹指反馈和仿真夹持所有权，再进入 `VERIFY_GRASP`，将夹爪试抬 15 mm，并从 Gazebo 独立读取零件实际位姿。零件位置/姿态跟随误差满足 3 mm/0.12 rad 且所有权仍属于当前手，才发布 `GRASP_CONFIRMED`。任一条件失败会自动解除可能残留的固定关节和 MoveIt 附着体，张开、退离、回位、重新识别点云并重新夹取，默认最多 5 次；点击停止会保留当前夹持，不执行自动释放。

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

控制器动作的超时按 `/clock` 仿真时间计算。三个深度相机和一个 RGB-only 腰部相机使 Gazebo 实时率低于 1 时，机械臂会继续等待轨迹完成，不会因电脑上的墙上时间先到而在 `APPROACH` 后误报失败；如果 `/clock` 连续 30 秒完全不前进，任务仍会停止并保持夹持。终端每 10 秒会打印一次控制器进度。

每次笛卡尔规划都会打印请求目标、步长、跳变阈值、碰撞检查开关，以及返回的 `error_code`、路径比例、轨迹点数和最大关节步长。当前 `cartesian_step=0.003 m`、`joint_step_limit=0.20 rad` 是为默认俯抓位姿放宽的仿真参数；路径仍必须达到 100% 且保持碰撞检查，部分路径不会执行。

等待控制器和 MoveIt 启动，在面板中点击 **开始完整任务**。首次夹取失败或夹住前停止后，等待后台运动退出，点击 **重新夹取**，不必重启仿真。重试会确认双臂均未持有零件，等待关节停止，张开夹爪、从低处抬离、依次回到初始姿态，再重新识别三帧点云并夹取；成功后继续原检测和交接流程。若仍失败，可再次点击重试，每次生成独立结果目录，`report.json` 中的 `retry_of` 关联前次记录。

点击 **随机零件位置** 可在初始位置 `(0.50, -0.20) m` 周围半径 5 cm 的圆内均匀随机放置螺丝。只能在任务未运行且没有夹爪持有零件时使用；移动会清零零件速度、更新 MoveIt 场景并丢弃旧点云，下一次开始/重试必须重新识别。点云 ROI 已扩大到覆盖随机圆和完整螺丝，不会产生逐次点击向外漂移的随机游走。

面板以 10 Hz 刷新左右臂 J1–J6 关节角（rad 和 °）、左右夹指位移（mm）及开口宽度（两个夹指位移之和）。这些数值直接读取 `/joint_states`，不是轨迹目标值。未收到或超过 2 秒的关节数据显示 `--`；仿真辅助夹持状态单独轮询 `/inspection/sim/owner`，断开或过期显示未知，不由夹爪闭合推断抓取成功。任务状态服务失联时禁用开始和重试按钮。

面板还可切换四路相机图像、显示当前阶段，并提供 **停止运动并保持夹持**。启动 launch 不会自动运动。停止是 ROS 动作取消，不是实机急停；本版没有实机连接。

不使用面板时加 `panel:=false`，通过服务运行：

```bash
ros2 service call /inspection/start std_srvs/srv/Trigger '{}'
ros2 topic echo /inspection/status
ros2 service call /inspection/stop std_srvs/srv/Trigger '{}'
# 首次夹取失败或夹住前停止，等待后台任务退出后：
ros2 service call /inspection/retry_pick std_srvs/srv/Trigger '{}'
# 任务空闲且未夹持时随机放置零件：
ros2 service call /inspection/randomize_object std_srvs/srv/Trigger '{}'
```

关闭面板只关闭窗口，后台任务仍在运行。需要停止时先点击停止按钮或调用停止服务。任务执行中或尚未停止时拒绝重复启动；一旦首次夹持已经确认，后续抬升、扫描、交接失败或任务完成后不会开放“重新夹取”，也不会自动松开悬空零件。夹持状态无法确认、退出运动未停止、退离/回位规划失败时重试会中止并说明原因。如果零件已被碰出配置 ROI，需先调整场景或 ROI，重试不会用旧点云绕过定位。不要同时在 RViz 中发送执行命令或另开夹爪控制程序。

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
| `grasp_reach_tolerance` | 0.003 m，闭合前实际 TCP 的三维到位容差 |
| `grasp_recovery_max` | 0.025 m，只允许一次完整规划的补降；横向与姿态偏差也须合格 |
| `fingertip_table_clearance` | 0.005 m；根据完整指尖网格及安装变换反算首次抓取高度，使指尖最低点停在桌面上方 5 mm |
| `grasp_test_lift` | 0.015 m，闭合后的抓取确认试抬高度 |
| `max_grasp_attempts` | 5，自动重新识别和夹取的最大次数；范围 1–20 |
| `random_position_center` | `[0.50, -0.20] m`，随机圆心 |
| `random_position_radius` | 0.05 m，随机位置最大半径 |
| `minimum_views` | 每只手至少 3 个成功视角 |
| `cameras` | 相机安装外参、分辨率、视场和近远裁剪面 |
| `output_directory` | 默认 `~/fr3_inspection_runs` |

`scene.yaml` 管桌面、螺丝和头部相机，`arms.yaml` 管双臂初始关节角。更改螺丝位置时同步改 ROI；若想左手先抓，除了 `first_arm: left`，还应把桌上螺丝和 ROI 移到左侧并重新检查可达性。改变零件尺寸时，同步修改 `scene.yaml` 的 `bolts` 和 `inspection.yaml` 中对应的长度、半径参数；启动会检查两处一致。点云筛选及 C++ 辅助夹持尺寸自动跟随配置，仍需检查 ROI、张开宽度、抓取偏移和双手空间是否适合新尺寸。参数单位为米，`shaft_radius`、`head_radius` 填半径。

## 输出结果

每次任务建立独立时间目录，包含：

- `report.json`：阶段、失败原因、完成/不可达视角、估计位姿、中心误差。
- `detection.npz`：实际分割出的螺丝点云、估计变换矩阵和包围尺寸。
- `left_view_XX/`、`right_view_XX/`：每个相机的 RGB 预览 `.ppm`、原始 RGB/深度字节 `.npz`、内参、拍摄时的 `world_optical` 外参。
- `metadata.json`：图像编码、行跨度、大小端和时间戳。读取深度时按编码解析，例如 `32FC1` 是米，不能把保存的 uint8 字节直接当深度值。

腕部 RGB/深度对在同一相机内限制时间差 ≤50 ms；腰部只要求停稳后的 RGB 新帧，三路相机不是硬件同步采集。没有自动调焦算法：这里通过减少零件位移来保持成像距离，实际镜头的景深仍需实验确认。

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
