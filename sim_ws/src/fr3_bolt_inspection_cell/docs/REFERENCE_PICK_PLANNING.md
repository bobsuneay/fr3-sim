# 抓取规划参考与本包调整

查阅日期：2026-09-10。参考 ROS 2 Humble 对应的官方实现，未直接复制第三方源码。

## 参考代码

1. [MoveIt Task Constructor Humble 抓放示例](https://github.com/moveit/moveit_task_constructor/blob/humble/demo/src/pick_place_task.cpp)：将张开、接近、抓取姿态 IK、允许手指接触、闭合、附着、抬升组织为任务阶段；`ComputeIK` 为抓取候选寻找多个解，`Connect` 连接相容状态。`plan()` 生成任务解，`execute()` 执行选中的解。
2. [MTC Humble CartesianPath 实现](https://github.com/moveit/moveit_task_constructor/blob/humble/core/src/solvers/cartesian_path.cpp)：在规划场景副本上检查碰撞和约束，默认 `min_fraction=1.0`；跳变阈值是相对于平均关节运动的倍数。不能把不完整路径当成成功抓取，也不能直接照抄示例的 1 cm 步长用于本项目毫米级螺丝。
3. [MoveIt Humble Pick and Place 教程](https://moveit.picknik.ai/humble/doc/examples/pick_place/pick_place_tutorial.html)：区分抓取姿态、抓取前接近方向、抓取后撤离方向及夹爪开合姿态。该页已标注旧抓放接口弃用并推荐 MTC，此处仅参考坐标和阶段划分，不引入其旧 API。示例机器人坐标、夹爪关节和尺寸需要按 FR3/HKV 实际模型转换。

## 对本包发现的具体问题

`90185d7` 已经提前求出了可用的下降轨迹，但 `global_move()` 当时只把它用于检查，随后丢弃。接近执行完成后，任务再次调用 `cartesian()` 从头求解。第二次请求可能使用不同求解过程，预检成功并没有保证真正执行那条已验证的下降路径。

现在参考 MTC 的“选择阶段解，再执行该解”方式：

1. 规划接近运动，并从接近终点规划完整下降。
2. 保存成功下降的起始机器人状态、关节轨迹和 FK 结果。
3. 执行接近运动。
4. 核对所有关节反馈：机械臂允许起点误差 0.01 rad，手指 0.0015 m；反馈缺失、过期或超差时拒绝执行。
5. 用当前规划场景复查下降轨迹的已采样状态，包含另一只机械臂；新碰撞会打印碰撞对并中止。
6. 对保存的轨迹按既有慢速限制设置时间，再执行；不重新请求下降 IK 或 `GetCartesianPath`。

这只实现了接近和下降两个阶段之间的解传递，并未集成 MTC，也未宣称全流程已联合规划。闭合、辅助夹持、附着、抬升和交接仍沿用现有任务状态机。未引入新的 ROS 包依赖；操作界面和 launch 用法相同。

## 验证边界

规划单元测试共 16 项通过，包含选中下降轨迹的保留、直接执行、任一手臂/夹指反馈变化后拒绝，以及接近后新碰撞的拒绝。ROS 服务使用替身测试，无法代替 Ubuntu 上的 MoveIt/Gazebo 联调。完整离线包仍存在前次已确认的两项旧几何断言失败。

运行时应先看到 `Pick preflight`、`Seeded Cartesian preflight complete`，接近执行成功后出现 `Revalidating prepared descent`。这一抓取下降阶段不再出现重新请求 `Cartesian request` 的流程；之后抬升或扫描仍可能使用该服务。
