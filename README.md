# Simulink Align Any Blocks

一个用于自动整理 Simulink 子连通网络的 MATLAB 脚本。

## 功能

- 查询普通信号线、Goto/From 和 Data Store 连接。
- 选择全图连接关系最多的最长有向路径作为主轴。
- 统一多端口模块主连接侧的端口节距。
- 将输入、输出模块分别放在左右边界，并沿 Y 方向错层排列。
- 使用拓扑列、子列和网格分配其他模块。
- 检查模块重叠、未连接线、斜线段和穿模块线路。

## 文件

- `alignAnyBlocks.m`：主要布局算法。
- `sl_customization.m`：向 Simulink 工具菜单注册命令。
- `startup.m`：将脚本目录加入 MATLAB 搜索路径。
- `test.slx`：演示模型。
- `layout_preview.png`：最终布局预览。
- `alignAnyBlocks_设计思路.md`：详细设计说明。

## 使用

1. 将当前目录加入 MATLAB 搜索路径。
2. 打开 `test.slx` 或其他 Simulink 模型。
3. 选中一个子系统作为布局锚点。
4. 执行：

```matlab
alignAnyBlocks
```

也可以刷新 Simulink 自定义菜单，然后使用：

```matlab
sl_refresh_customizations
```

菜单项名称为 `Align Blocks`，默认快捷键为 `Ctrl+Shift+Q`。

## 设计说明

详见 [alignAnyBlocks_设计思路.md](alignAnyBlocks_设计思路.md)。
