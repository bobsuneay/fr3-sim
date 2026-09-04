# 模型来源与分发边界

模型由用户通过本地 `Desktop/FR3` 提供，本次未从不明下载站获取模型，也未修改桌面原工程。

- 原始 URDF：`src/fairino3_v6.urdf` → 本包 `urdf/fr3_arm.urdf`。
- 修改范围：仅将网格 URI 的包名 `fairino_description` 改为 `fr3_bolt_cell`，用于独立安装。六轴变换、限制、惯量、质量和几何保持原值。
- 七个 STL：来自 `src/fairino3_v6/*.STL`，逐字节复制。
- 来源包的 `package.xml` 声明 license 为 `BSD`，但所提供模型目录没有完整 BSD 授权正文和明确权利人文件。本包不替模型补造版权声明，也不将这些资产改为 MIT。
- 新写的场景生成器、夹爪、安装架、launch、配置和文档使用本包 LICENSE。第三方资产仍需遵守其原授权。

当前可在用户本地进行参考与集成。**再次公开上传这些模型前，请核对原始模型的再分发许可，并补齐原作者要求的授权文件。**此前用户已有的 GitHub 仓库不代表本批新导入模型已完成许可核对。本次不自动发布这些资产。

原始 SHA-256：

```text
fairino3_v6.urdf BEE48FD89106E2246E55F130BE5CA3DE4852D888213F58F631CFACDBA5298772
base_link.STL A4C94C0FCC939C6EA20BBC6E5548DF78194FC6F7BA9C405FB8885B64F4587929
forearm_link.STL 384388E6F6BDEE3749A1B732BEF1143AB35D1DC1B5BAB551829034E7438D3EE8
shoulder_link.STL 7DD0C503F694CE38F9E5950B925B081F8DBFECD828147788EF3A5586699573B1
upperarm_link.STL CEA67D53323AEA9881B81F1EBA55BAAF12799C26C07E902DA2CBEC084CF6CB5B
wrist1_link.STL 5BB16EE905EAD03D7C350C2BAEC64CB99744B7D7DC430A1D3B103201F189916D
wrist2_link.STL 4257EB654ACF51BFE52F2477C512C40CCCFD855FD3ED551C3826268E050BFB6D
wrist3_link.STL B4C06DD49A457BFFD4A6B73DD3DFA1803A61CDA8462A3B287CC7F9C9978F13B1
```

原 URDF 自带 SolidWorks-to-URDF 导出器注释，已保留；导出工具作者注释不能单独视为模型资产授权。资产哈希在离线测试中校验。
