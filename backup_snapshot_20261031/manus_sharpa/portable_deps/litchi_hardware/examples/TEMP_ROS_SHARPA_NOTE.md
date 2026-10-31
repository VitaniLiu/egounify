# 临时笔记：ROS 2 控制并记录左手 Sharpa

当前设备：

- Sharpa IP：`192.168.26.11`
- Sharpa 序列号：`CD559038CD54`
- 手侧：左手
- MANUS 校准目录：
  `/home/user/workspace/litchi_hardware/calibration/metaglove`

启动前先退出正在运行的 `pilot_sdk`、`read_sharpa.py` 或其他 Sharpa
控制程序，避免多个进程同时控制硬件。

## 1. 构建 ROS 2 workspace

首次运行或 ROS 代码更新后执行：

```bash
pixi install -e retarget-v4
pixi run stage-retarget-v4
pixi run -e ros2 ros-build
```

## 2. 单独验证 MANUS

先退出正在运行的 teleop、`manus_driver` 和其他 MANUS SDK 程序。MANUS native SDK
生命周期是进程级单例，不要同时启动多个 MANUS reader。

### 2.1 绕过 ROS，直接读取 HAL

读取左手 10 帧：

```bash
pixi run python examples/read_manus.py \
  --backend native \
  --side left \
  --calibration-directory /home/user/workspace/litchi_hardware/calibration/metaglove \
  --timeout 1.0 \
  --count 10
```

读取右手：

```bash
pixi run python examples/read_manus.py \
  --backend native \
  --side right \
  --calibration-directory /home/user/workspace/litchi_hardware/calibration/metaglove \
  --timeout 1.0 \
  --count 10
```

读取双手并持续输出：

```bash
pixi run python examples/read_manus.py \
  --backend native \
  --side both \
  --calibration-directory /home/user/workspace/litchi_hardware/calibration/metaglove
```

正常时每一行都是一个 JSON frame，包含：

- `side`
- `sequence`
- `timestamp_s`
- 25 个 `keypoint_names`
- `positions_m`
- `orientations_xyzw`

如果这里持续出现 `no left/right Manus native frame received`，问题位于 MANUS
SDK、Dongle、手套连接或 calibration，尚未进入 ROS 层。

### 2.2 单独启动 MANUS ROS driver

终端一：

```bash
pixi run -e ros2 ros2 run manus_driver manus_driver --ros-args \
  -p backend:=native \
  -p publish_left:=true \
  -p publish_right:=false \
  -p native_host_discovery_s:=3 \
  -p frame_timeout_s:=0.1 \
  -p publish_rate_hz:=30.0 \
  -p calibration_directory:=/home/user/workspace/litchi_hardware/calibration/metaglove
```

正常启动时应看到类似日志：

```text
Manus driver connected via native; fake=False
```

终端二检查节点和 topic：

```bash
pixi run -e ros2 ros2 node list
pixi run -e ros2 ros2 topic list | grep manus
pixi run -e ros2 ros2 topic info -v /manus/left/keypoints
```

读取一条左手关键点消息：

```bash
pixi run -e ros2 ros2 topic echo \
  /manus/left/keypoints \
  geometry_msgs/msg/PoseArray \
  --once \
  --qos-reliability best_effort
```

查看发布频率：

```bash
pixi run -e ros2 ros2 topic hz /manus/left/keypoints
```

如果 HAL 示例能输出，但 ROS topic 没有消息，保留 `manus_driver` 终端的完整日志，重点检查
是否重复出现：

```text
no left Manus native frame received
failed to publish left Manus frame
```

### 2.3 验证结论

- HAL 示例没有 frame：检查 MANUS SDK/Dongle/手套/calibration。
- HAL 有 frame，ROS driver 没有：检查 `manus_driver` 参数、QoS 和节点日志。
- ROS `/manus/left/keypoints` 有数据，但 `/sharpa/left/command` 没有：检查
  `retarget_node`，并确认没有 `dropped stale left Manus frame` 日志。
- `/sharpa/left/command` 有数据，但手不动作：检查 `sharpa_driver` 和关节限位错误日志。

MANUS SDK 的原始 `publishTime` 使用设备自己的时钟域，不是 ROS 系统时间。
`manus_driver` 会在收到 frame 时改用 ROS 时钟填充 `PoseArray.header.stamp`，避免
`retarget_node` 将有效输入误判为过期。更新相关代码后必须重新执行：

```bash
pixi run -e ros2 ros-build
```

## 3. 启动 MANUS 到 Sharpa 遥操作

以下命令只启用左手：

```bash
pixi run -e ros2 teleop \
  enable_left:=true \
  enable_right:=false \
  left_sharpa_serial:=CD559038CD54 \
  retarget_backend:=v4 \
  manus_backend:=native \
  manus_calibration_directory:=/home/user/workspace/litchi_hardware/calibration/ljw \
  speed_coefficient:=0.15 \
  current_coefficient:=0.30 \
  stop_on_command_timeout_s:=0.5
```

```
pixi run -e ros2 teleop \
    enable_left:=true \
    enable_right:=false \
    left_sharpa_serial:=CD559038CD54 \
    retarget_backend:=v4 \
    manus_backend:=native \
    manus_calibration_directory:=/home/user/workspace/litchi_hardware/calibration/ljw
```

启动后会向真实 Sharpa 发送位置命令。运行前确保手周围无人员和障碍物，并准备好急停。

`stop_on_command_timeout_s:=0.5` 表示控制命令中断超过 0.5 秒后停机。触发停机后需要重启
launch。

真实 launch 默认使用
`reference/sharpa-manus-sdk/retargeting_alg_release_V4.0` 对应的厂商 V4
URDF/CasADi 优化器。上游扩展只支持 CPython 3.10，因此它在
`.pixi/envs/retarget-v4` sidecar 中运行；不要为真实手改成
`retarget_backend:=analytic`。25 点顺序为 wrist、thumb、index、middle、ring、
pinky，22 关节命令顺序与 V4 的 Wave `joint_names` 一致。

只验证 V4 retarget 节点、暂不连接 Sharpa：

```bash
pixi run -e ros2 ros2 run manus_sharpa_teleop retarget_node --ros-args \
  -p enable_left:=true \
  -p enable_right:=false \
  -p backend:=v4
```

## 4. 记录 rosbag

另开一个终端，在仓库根目录执行：

```bash
mkdir -p bags

pixi run -e ros2 ros2 bag record \
  -o bags/manus_sharpa_left \
  --compression-mode file \
  --compression-format zstd \
  -e '^/(manus/left/keypoints|sharpa/left/(command|joint_states|tactile/.*))$'
```

按 `Ctrl+C` 结束录制。

检查 bag：

```bash
pixi run -e ros2 ros2 bag info bags/manus_sharpa_left
```

如果需要重复录制，请换一个输出目录，或者先处理已有的
`bags/manus_sharpa_left`，否则 rosbag 会拒绝覆盖。

## 5. 检查 ROS topics

查看 MANUS 关键点：

```bash
pixi run -e ros2 ros2 topic echo /manus/left/keypoints
```

查看 retarget 后的 Sharpa 控制命令：

```bash
pixi run -e ros2 ros2 topic echo /sharpa/left/command
```

查看 Sharpa 关节反馈：

```bash
pixi run -e ros2 ros2 topic echo /sharpa/left/joint_states
```

查看关节反馈频率：

```bash
pixi run -e ros2 ros2 topic hz /sharpa/left/joint_states
```

查看拇指六维 tactile：

```bash
pixi run -e ros2 ros2 topic echo /sharpa/left/tactile/thumb/wrench
```

检查拇指原始 tactile 图频率：

```bash
pixi run -e ros2 ros2 topic hz /sharpa/left/tactile/thumb/raw
```

## 6. 仅启动 Sharpa ROS 驱动

不使用 MANUS，只启动左手 Sharpa：

```bash
pixi run -e ros2 ros2 run sharpa_driver sharpa_driver --ros-args \
  -p enable_left:=true \
  -p enable_right:=false \
  -p left_serial:=CD559038CD54 \
  -p discovery_timeout_s:=20.0 \
  -p speed_coefficient:=0.15 \
  -p current_coefficient:=0.30
```

另开终端发送一个全零的 22 关节位置命令：

```bash
pixi run -e ros2 ros2 topic pub --once \
  /sharpa/left/command \
  sensor_msgs/msg/JointState \
  '{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

## 7. 当前记录范围

当前 ROS 2 driver 提供：

- `/manus/left/keypoints`
- `/sharpa/left/command`
- `/sharpa/left/joint_states`
- `/sharpa/left/tactile/{thumb,index,middle,ring,pinky}/raw`
- `/sharpa/left/tactile/{thumb,index,middle,ring,pinky}/deformation`
- `/sharpa/left/tactile/{thumb,index,middle,ring,pinky}/wrench`

`joint_states` 同时包含 22 维 `position`、`velocity` 和 `effort`；其中
`effort` 当前是 SDK `State.torques` 的直接值，未做额外标定或缩放。触觉
`raw`/`deformation` 当前均为 `sensor_msgs/Image` 的 `mono8` 原始 SDK 数据；
`wrench` 为 `geometry_msgs/WrenchStamped`。
