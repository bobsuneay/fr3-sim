# 来源记录

- 本包仿照用户提供的 fr3-sim 归档中的 fr3_bolt_cell 构建；共用场景 Xacro、world.py、planning_scene.py 和 RViz 设置经过适配。
- 七个法奥 FR3 STL 和原始六轴 URDF 来自该归档的 fr3_bolt_cell，URDF 只改资源 URI，运行时加左右名称前缀；连杆变换、原始限制和惯量保持。
- 五个 HKV TG-9801 STL 从该归档的 fr3_real_bringup 复制，其来源记录指向用户提供的 ros2_hkv_gripper。
- 机械安装板与夹爪惯量/碰撞体为集成近似值，不能作为产品参数。
- 外部驱动依据用户提供的 frcobot_ros2-v3.0.0_robotV3.9.7 和桌面的 ros2_hkv_gripper 源码核对；本包不包含厂商预编译 SDK，也不重分发其驱动实现。

新编写代码使用 MIT。FR3 资产原记录标注 BSD，但缺少完整权利人/许可正文；
HKV 原 README 标注 AGPL-3.0，package.xml 标注 Apache-2.0，存在声明不一致。
第三方资产不因复制到本目录而改成 MIT，公开分发时应沿用确认后的原授权。

`docs/assets.sha256` 记录本包资产哈希。`tools/prepare_driver.py` 对厂商接口源码做哈希检查，
仅在新目录生成修改版本和 diff，不覆盖提供的源目录。
