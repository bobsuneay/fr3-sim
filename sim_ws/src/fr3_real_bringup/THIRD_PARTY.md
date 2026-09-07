# 第三方资产说明

FR3 网格和 URDF 从本仓库原有 fr3_bolt_cell 复制，仅修改了 package URI。
HKV 网格来自用户本机 C:/Users/sun/Desktop/ros2_hkv_gripper 的上一版集成。
这些第三方资产不随本包重新授权；外部发布前请确认原始作者许可。

HKV 原 README 声明 AGPL-3.0，而 package.xml 声明 Apache-2.0，许可存在冲突，
因此本包保留来源记录，不替第三方资产选择许可。

真机模式依赖外部 FAIRINO frcobot_ros2 驱动和厂商 SDK；本包不复制二进制 SDK。
