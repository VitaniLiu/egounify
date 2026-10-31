# manus_sharpa_teleop

一键启动 Manus → retargeting → Sharpa Wave 双手链路：

```bash
pixi install -e retarget-v4
ros2 launch manus_sharpa_teleop teleop.launch.py
```

安全的无硬件入口：

```bash
ros2 launch manus_sharpa_teleop fake_teleop.launch.py
```

`teleop.launch.py` 默认使用真实硬件和 Sharpa V4 optimizer sidecar；
`fake_teleop.launch.py` 显式使用 analytic retargeter。真实 Manus client 可通过
`MANUS_CLIENT_EXECUTABLE` 和
`MANUS_CLIENT_WORKING_DIRECTORY` 环境变量由 driver 自动启动。完整安装、参数及 topic
说明见仓库根目录 `README.md`。

Sharpa driver 默认发布完整的 22 维 position/velocity/effort，并在设备支持时发布
五指 tactile `raw`、`deformation` 和 `wrench`。可通过
`publish_tactile:=false` 跳过 tactile 初始化与发布。

V4 的两个 CPython 3.10 `.so` 位于本包的 `vendor/retarget_v4`，安装到
`share/manus_sharpa_teleop/retarget_v4`。左右手 URDF 不在本包重复保存，由独立
`sharpa_wave_description` 包提供。两类资产统一通过以下命令从只读 reference 更新：

```bash
pixi run stage-retarget-v4
```
