# 抓取规划参考与本包调整

查阅日期：2026-09-10。参考 ROS 2 Humble 对应的官方实现，未直接复制第三方源码。

## 参考代码

1. [MoveIt Task Constructor Humble 抓放示例](https://github.com/moveit/moveit_task_constructor/blob/humble/demo/src/pick_place_task.cpp)：将张开、接近、抓取姿态 IK、允许手指接触、闭合、附着、抬升组织为任务阶段；`ComputeIK` 为抓取候选寻找多个解，`Connect` 连接相容状态。`plan()` 生成任务解，`execute()` 执行选中的解。
2. [MTC Humble CartesianPath 实现](https://github.com/moveit/moveit_task_constructor/blob/humble/core/src/solvers/cartesian_path.cpp)：在规划场景副本上检查碰撞和约束，默认 `min_fraction=1.0`；跳变阈值是相对于平均关节运动的倍数。不能把不完整路径当成成功抓取，也不能直接照抄示例的 1 cm 步长用于本项目毫米级螺丝。
3. [MoveIt Humble Pick and Place 教程](https://moveit.picknik.ai/humble/doc/examples/pick_place/pick_place_tutorial.html)：区分抓取姿态、抓取前接近方向、抓取后撤离方向及夹爪开合姿态。该页已标注旧抓放接口弃用并推荐 MTC，此处仅参考坐标和阶段划分，不引入其旧 API。示例机器人坐标、夹爪关节和尺寸需要按 FR3/HKV 实际模型转换。

## 对本包发现的具体问题

`90185d7` 已经提前求出了可用的下降轨迹，但 `global_move()` 当时只把它用于检查，随后丢弃。接近执行完成后，任务再次调用 `cartesian()` 从头求解。第二次请求可能使用不同求解过程，预检成功并没有保证真正执行那条已验证的下降路径。

现在参考 MTC 的“选择阶段解，再执行该解”方式：

1. 先求精确抓取姿态的 IK（最多 8 组种子，探索 FR3 基座、肘部、腕部不同分支）。从成功抓取解向上规划 50 mm，再规划到这个上端的指定关节构型。这样不会把自由接近姿态的误差带入下降。
2. 将向上路径的关节序列和 FK 序列反向，保存为下降轨迹及其起始机器人状态。
3. 执行接近运动。
4. 核对所有关节反馈：机械臂允许起点误差 0.01 rad，手指 0.0015 m；反馈缺失、过期或超差时拒绝执行。
5. 用当前规划场景复查下降轨迹的已采样状态，包含另一只机械臂；新碰撞会打印碰撞对并中止。
6. 对保存的轨迹按既有慢速限制设置时间，再执行；不重新请求下降 IK 或 `GetCartesianPath`。

这只实现了接近和下降两个阶段之间的解传递，并未集成 MTC，也未宣称全流程已联合规划。闭合、辅助夹持、附着、抬升和交接仍沿用现有任务状态机。未引入新的 ROS 包依赖；操作界面和 launch 用法相同。

## 验证边界

规划与运动学测试共 19 项通过，包含抓取点优先求解、反向路径保留、直接执行、任一手臂/夹指反馈变化后拒绝，以及接近后新碰撞的拒绝。包含在日志目标位置用 FR3 URDF 和数值 IK 贯通 50 mm 升降段的回归；该回归只验证运动学，不验证场景碰撞或实际 KDL 求解行为。ROS 服务使用替身测试，无法代替 Ubuntu 上的 MoveIt/Gazebo 联调。完整离线包仍存在前次已确认的两项旧几何断言失败。

运行时应先看到 `Grasp-first preflight`、`Seeded Cartesian preflight complete`、`Grasp-first plan selected`，接近执行成功后出现 `Revalidating prepared descent`。正常下降直接执行保存的轨迹；若后续到位检查触发补降，则为补降请求 `Cartesian request`，抬升或扫描也仍可能使用该服务。

## 本地 FR3 抓取示例对照（2026-09-11）

另检查了用户提供的 `C:/Users/sun/Desktop/FR3` 工程，重点是：

- `src/fairino3_v6_moveit2_config/src/pick_banana_node.cpp` 的 `descend_eef_to_z()`、主下降段和闭合前高度检查。
- `src/fairino3_v6_moveit2_config/launch/pick_banana.launch.py` 的实际默认参数。
- `src/fairino3_v6_moveit2_config/config/kinematics.yaml` 和 `fairino3_v6_robot.urdf.xacro`。

该工程是单臂 FR3 加 Robotiq 2F-85，TCP、夹爪驱动和安装方向与本包双臂 HKV 不同，不能直接套用其目标高度或关节命令。它同样使用 KDL，并非更换了逆解求解器。这里只借鉴流程，未复制第三方源码或模型；也未运行该工程来验证其抓取成功率。

| 项目 | 本地参考工程 | 本包采用方式 |
| --- | --- | --- |
| 下降完成比例 | launch 默认 `min_fraction=0.5`，可先执行部分路径再补降 | 整段完整、连续并通过碰撞检查才执行 |
| 碰撞 | launch 默认 `avoid_collisions=false`、`descend_avoid_collisions=false` | 运动规划碰撞检查保持开启 |
| 失败重试 | 可放宽姿态，补降失败后还可用仅位置目标规划 | 精确 TCP 姿态不变，局部 IK 失败后只移除种子关节窗口重试一次；原始关节差仍受 0.20 rad 上限约束，且复查插值状态的碰撞和 FK |
| 闭合前检查 | 读取当前末端高度，尝试多次补降，部分失败分支仍继续 | 从新鲜关节反馈求 FK，检查三维位置和姿态；只允许一次有界补降，未到位则不闭合 |

新增 `CHECK_GRASP_REACH` 阶段位于下降成功之后、夹爪闭合之前。等待既有 `settle_seconds` 后，日志 `Grasp reach check` 输出目标/实际 TCP、三维误差、带符号高度差及姿态误差。位置误差不超过 `grasp_reach_tolerance=0.003 m` 且姿态误差不超过既有 `angular_tolerance=0.12 rad` 才进入 `GRASP`。

如果只是尚未降够，且横向误差在 3 mm 内、姿态在上述容差内、正向高度差不超过 `grasp_recovery_max=0.025 m`，会出现 `Supplemental grasp descent`，从当前反馈重新规划到同一个精确抓取目标，沿用 8 mm/s 默认下降速度。补降也必须完整通过检查，随后重新测量；补降规划失败、控制器中止、反馈无效或仍不到位，都中止任务并保留张开状态。超出补降范围、横向偏移过大或已经降得过低时不会盲目补降。

这项补降只处理控制器报告完成后仍存在的小幅到位误差。若任务仍停在 `Grasp-first preflight` 并最终耗尽种子，应继续分析逆解日志；不能据此宣称补降解决了所有“停在上方”的原因。新增的保留碰撞检查的 IK 重试，只有产生连续且完整验证的路径才能改善这类规划失败。

本次完整离线测试为 59 项通过、2 项既有几何断言失败。新增 12 项覆盖重试解的连续性、超限关节跳变拒绝、实际反馈 FK、到位不补降、小量补降、超高/过低/横向/姿态/非数值反馈拒绝，以及补降无效或控制器中止后的停止。ROS 服务使用替身，未进行实际 KDL、Gazebo 或实机测试。
