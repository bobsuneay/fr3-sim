# 验证记录与待验收项

## 本次实际执行

环境：Windows，Python 3.9.13；项目内临时 Xacro 2.1.1。测试依赖放在仓库 `.tools/python`，没有安装到用户的 ROS 或修改桌面 FR3 工程。`.tools` 不进入版本控制，不应复制到 Ubuntu。

执行 `pytest`：**18 项通过**，有一条 Xacro/Python 3.9 的 DeprecationWarning。Python 编译检查、`setup.py check`、安装数据文件存在性检查均通过；动态 world 生成成功。

后续用户运行日志暴露了 Gazebo 模型路径未导出的问题：原有测试只核对源目录里的网格文件，未覆盖 Gazebo 的 `model://` 搜索。现已补上路径导出，并新增 isolated/merged 两种 colcon 安装目录布局的资源查找回归测试。本轮针对该问题运行的两个回归测试和语法检查均通过（3 passed，17 deselected）；这些仍是离线检查，Ubuntu 中的 Gazebo 加载结果待复核。

覆盖范围：

- Xacro 展开无残留宏，URDF 单根树连通，link 名称唯一。
- 真实 FR3 七个 STL 的 SHA-256 与用户提供原件一致。
- 有碰撞几何的 link 均有正质量、正定且满足三角不等式的惯量。
- 六个旋转关节、两个平移关节的 ros2_control 与控制器列表一致。
- SRDF 的 link、关节、关节目标范围及 MoveIt 控制器映射一致。
- 初始姿态的每个碰撞几何包围盒均不与桌子相交、不低于地面。
- 相机针孔视锥覆盖全部螺栓中心，并留有图像边缘余量；这不是遮挡渲染测试。
- 20 个螺栓均为独立非 static 模型，有杆/头碰撞体和正确组合惯量，总长 25 mm。
- 非法长度、阵列间距、越出桌面、非法行列数、NaN/Inf 等配置被拒绝。
- Python AST、YAML、package.xml 语法及 RViz 固定坐标配置可解析。

几何检查记录（默认配置）：

```text
初始 gripper_tcp 世界坐标 ≈ [0.102000, -0.263451, 0.966451] m
螺栓中心投影范围 u ≈ 245.59 ～ 419.21 px
螺栓中心投影范围 v ≈ 229.42 ～ 319.53 px
图像尺寸 640 × 480
```

这些数值来自模型正运动学与针孔计算，不是 Gazebo 截图或实机测量。没有宣称完成全关节空间碰撞检测，也没有证明全部桌面位置可达。

## 在 Ubuntu 上复核离线检查

Humble 自带 Xacro 的版本可能与这里的离线版本不同，所以必须在目标系统再展开并检查一次。

```bash
source /opt/ros/humble/setup.bash
source ~/fr3_bolt_ws/install/setup.bash
cd ~/fr3_bolt_ws/src/fr3_bolt_cell
python3 -m pytest test -q
python3 -m compileall -q fr3_bolt_cell launch
```

URDF → SDF 检查命令见实施文档第 7 节。这里未运行 Linux 的 `check_urdf` / `gz sdf`，所以离线 XML 通过不等于 Gazebo 转换已经验证通过。

## 尚未执行：目标运行环境验收

当前电脑没有 Ubuntu ROS 2/Gazebo 可运行环境，以下项目须在用户 Ubuntu 完成。不要把未执行项写成“运行通过”。

- [ ] apt/rosdep 依赖解析与 colcon 编译。
- [ ] Humble Xacro 展开，urdfdom 和 Gazebo SDF 转换校验。
- [ ] Gazebo 创建整机，三个控制器均 active，机械臂稳定保持初始姿态。
- [ ] 螺栓在桌面沉降后不穿透、不明显弹飞，模型状态话题可读。
- [ ] `check_sim` 全部 PASS：时钟、关节、控制器、图像/点云、TF 与螺栓位置。
- [ ] 空载夹爪 50 → 10 → 50 mm 开合，动作结果成功。
- [ ] MoveIt 当前状态无碰撞，桌面可见，Plan 成功，Execute 在 Gazebo 跟随。
- [ ] 保存 Gazebo 总览、RViz 点云、RGB 图像与日志作为实际运行记录。

还未包含：接触夹取稳定性、任意布局的可达性、物体中心扫描轨迹、真实镜头对焦、双臂交接、硬件/安全验收。
