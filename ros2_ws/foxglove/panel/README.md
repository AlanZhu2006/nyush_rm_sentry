# NYUSH Sentry 操作台

使用 Foxglove 面板管理建图、导航、手动底盘和 RTT 日志。连接 Sentry 的 Foxglove WebSocket 后，在布局中添加 **NYUSH Sentry 操作台**。

在 3D 面板用 **2D Pose** 选定目标，然后回到操作台点击 **执行目标**。使用 **取消目标** 清除待执行的目标或取消正在执行的目标。

取得手动控制后，可按住方向按钮，或聚焦 **键盘控制** 后同时按住空格和 WASD 或 QE。结束操作时点击 **停止并释放**。

在本目录构建扩展包。

```sh
npm ci
npm run check
npm run package
```

后端安装与连接步骤见源码仓库中的 `ros2_ws/foxglove/README.md`。扩展包包含项目的 MIT 许可证和第三方声明。
