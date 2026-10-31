# litchi_hardware

Manus MetaGlove → Sharpa Wave / Wuji Hand 2 双手 teleoperation，以及 Manus、Sharpa、
Wuji、Linker Hand、Xynova 灵巧手与机械臂硬件的可复用 HAL/ROS2 包装。纯 Python HAL 位于 `src/`，
没有 ROS 依赖；ROS2 workspace 位于 `ros2/`。

## 已交付的链路

```text
Manus SDK callback（native，默认）
  或 SharpaManusClient → ZMQ/Protobuf（可选隔离后端）
  → Manus HAL（双手各 25 个关键点）
  → manus_driver（geometry_msgs/PoseArray）
  → manus_sharpa_teleop/retarget_node
      → Sharpa V4.0 URDF/CasADi optimizer（Python 3.10 sidecar）
      → 22 关节、限位、滤波
  → sharpa_driver（sensor_msgs/JointState）
  → Sharpa Wave Python SDK
```

Wuji 链路复用相同的 Manus HAL 和 `manus_driver`，之后进入独立分支：

```text
manus_driver（25 点 PoseArray）
  → manus_wuji_teleop/retarget_node
      → 官方 wuji-sdk RetargetSession
      → Manus 25 点转 MediaPipe 21 点
      → 20 关节固件顺序命令
  → wuji_hand_driver（sensor_msgs/JointState）
  → 官方 wuji-sdk WujiHand2 API
```

对应 fake 链路使用完全相同的 HAL 接口和 ROS topics，只替换首尾硬件后端。核心设计与
依赖边界见 [docs/architecture.md](docs/architecture.md)。

## 作为 Python SDK 安装

仓库生成的是包含厂商头文件、动态库、Python extension、配置和 Manus client 的
Linux x86_64 私有 platform wheel。默认环境与 ROS2 环境共享 Python solve-group：

```bash
pixi install
pixi run wheel
python3 -m pip install 'dist/litchi_hardware-0.1.0-*.whl[sharpa,manus,linker,xynova]'
```

如果 wheel 已上传到公司的私有 index，同事只需：

```bash
python3 -m pip install 'litchi-hardware[sharpa]'
# 同时安装 Manus native + ZMQ 两种 backend
python3 -m pip install 'litchi-hardware[sharpa,manus]'
# 安装包含 Xynova 手/机械臂 SDK 与 x86_64/aarch64 runtime 的 platform wheel
python3 -m pip install 'litchi-hardware[xynova]'
# Linker G20/O30I，包含厂家 Python SDK、HOP 控制器和多架构 libcanbus
python3 -m pip install 'litchi-hardware[linker]'
```

调用入口：

```python
from litchi_hardware import DeviceAction, linker, manus, sharpa, xynova

# HAL
hand = sharpa.create_hand(sharpa.SharpaConfig(name="left"))
native_tracker = manus.create_tracker(
    manus.ManusNativeConfig(name="gloves"),  # 进程内直接调用 Manus SDK
)
zmq_tracker = manus.create_tracker(
    manus.ManusZmqConfig(name="isolated"),  # 启动/订阅 SharpaManusClient
)
xynova_hand = xynova.create_hand(
    xynova.XynovaConfig(
        name="xz_hand",
        can_interface="0.0",
        device_id=1,
    )
)
xynova_hand.connect()
state = xynova_hand.read_joint_state()  # core.types.JointState，23 关节弧度
xynova_hand.send_action(
    DeviceAction({"joint_position": state.position})
)

# Xynova 机械臂：连接默认不上电，运动前必须显式确认并上电
xynova_arm = xynova.create_arm(
    xynova.XynovaArmConfig(
        name="xynova_ar",
        remote_ip="192.168.1.160",
        local_ip="192.168.1.133",
        model=xynova.XynovaArmModel.XYNOVA_AR,
    )
)
xynova_arm.connect()
arm_state = xynova_arm.read_joint_state()
# 确认工作区安全后再调用：xynova_arm.power_on()
# xynova_arm.send_action(DeviceAction({"joint_position": arm_state.position}))

# 厂商原生 API 与 C/C++ 开发路径
native_sharpa = sharpa.load_sdk()
manager = sharpa.SharpaWaveManager.get_instance()  # lazy proxy，同样可用
native_manus = manus.load_library()
native_bridge = manus.load_bridge()
print(sharpa.include_dir(), sharpa.library_dir())
print(
    manus.include_dir(),
    manus.library_dir(),
    manus.bridge_include_dir(),
    manus.bridge_library(),
    manus.client_executable(),
)

# 仍可访问 Xynova 灵巧手原生 ctypes API（SDK 2.0.0 R3）
print(xynova.sdk_build_info())
print(xynova.bundled_library_path())
# 机械臂 SDK 0.7.1 的头文件、动态库和项目 C ABI bridge
print(xynova.arm_include_dir(), xynova.arm_sdk_library())
print(xynova.arm_bridge_library())

linker_hand = linker.create_hand(
    linker.LinkerConfig(
        name="right_o30i",
        model="O30I",
        side="right",
        transport="socketcan",
        channel="can0",
    )
)
print(linker.protocol_manual(), linker.bundled_libcanbus_path())
```

wheel tag 与构建它的解释器 ABI 一致，例如
`cp312-cp312-linux_x86_64`。Python 3.10–3.13 各自在对应解释器环境中构建一份
wheel；包内已经包含 Sharpa 对应 ABI 的 extension。

> 厂商资产是 proprietary material。只允许通过公司私有仓库/index 分发给协议授权的
> 内部人员，不要上传公开 PyPI。wheel 体积会随打包的厂商 SDK 架构而变化。

## 快速开始：无硬件

推荐使用仓库的 ROS2 Humble pixi 环境：

```bash
pixi install -e ros2
pixi install -e retarget-v4
pixi run stage-retarget-v4
pixi run -e ros2 ros-build
pixi run -e ros2 teleop-fake
```

另一个终端进入相同环境后可检查：

```bash
pixi shell -e ros2
source ros2/install/setup.bash
ros2 node list
ros2 topic echo /sharpa/left/command --once
ros2 topic echo /sharpa/right/joint_states --once
```

也可以在已有 ROS2 环境中构建。HAL 必须先以 editable package 安装：

```bash
python3 -m pip install -e '.[manus]'
source /opt/ros/humble/setup.bash
cd ros2
colcon build --symlink-install
source install/setup.bash
ros2 launch manus_sharpa_teleop fake_teleop.launch.py
```

## 真实硬件准备

### Sharpa Wave

不再要求安装厂商 deb。private wheel 已包含 Sharpa SDK 5.0.6 的头文件、动态库、
`pilot_sdk`、配置和 Python 3.10–3.13 extension。可先确认 loader：

```bash
pixi run -e ros2 python -c \
  'from litchi_hardware import sharpa; print(sharpa.load_sdk().__file__)'
```

设备与主机需位于可互通网络。`sharpa_driver` 会发现设备，并先读取持久化的 speed/current
coefficient、control source 和 control mode。已匹配的设置不会重复写入；不匹配时只写一次
并回读校验（系数使用 `1e-4` 绝对容差）。确认 source 为 SDK、mode 为 POSITION 后才会
读取独立的 enable state；仅在 disabled 时 enable 一次并等待生效、回读确认，然后调用
start。enable setter 不会重试，状态通过最长 1 秒的只读轮询确认。初始化不会无条件
disable、切换到 APP 或重复 enable，以兼容 firmware 3.0.6。
双手同时存在时建议传入 serial，避免发现顺序变化。

### Manus MetaGlove

每个硬件域自行管理厂商资产：MANUS 位于 `hardware/manus/_vendor`，Sharpa 位于
`hardware/sharpa/_vendor`，并各自维护 provenance manifest。MANUS 仅分发 Integrated
SDK 的头文件和动态库、左右手校准文件及 SharpaManusClient，不再包含 Remote/Core SDK
及其旧版 gRPC/Protobuf runtime。项目自有的 `liblitchi_manus_bridge.so`、C ABI 头文件
及源码位于 `hardware/manus/_native`。默认 `native` backend 在当前 Python 进程中直接
初始化 Integrated SDK、注册 raw skeleton/landscape callbacks，并通过线程安全的最新帧
缓冲区输出 25 个关键点；不经过 ZMQ 或 Protobuf。

`zmq` backend 仍完整保留，用于需要进程隔离或兼容既有 Sharpa-Manus 部署的场景。它
使用 wheel 内 `$ORIGIN/../lib` RUNPATH 的 `SharpaManusClient`，也可用团队维护的 client
覆盖：

```bash
export MANUS_CLIENT_EXECUTABLE=/absolute/path/to/client/SharpaManusClient
export MANUS_CLIENT_WORKING_DIRECTORY=/absolute/path/to/client
```

ZMQ 后端默认监督该进程，退出时终止它。若 client 已由其他服务运行，使用
`ManusZmqConfig(manage_client=False)`，tracker 将只订阅指定 `endpoint`。native 专属
设置（`native_library`、`calibration_directory`、`native_host_discovery_s`、
`native_loopback_only`）只存在于 `ManusNativeConfig`；ZMQ 专属设置只存在于
`ManusZmqConfig`，避免无效配置被静默接受。`ManusConfig` 仅承载两个后端共享的超时与
坐标系设置，传给 factory 时默认选择 native。Manus SDK 生命周期是进程级单例，因此
每个进程只创建一个 native tracker；该 tracker 本身同时提供左右手数据。

### Xynova Dexterous Hand

`hardware/xynova` 保留了独立 SDK workspace 中的纯 Python `ctypes` wrapper、C/C++
头文件，以及 Linux x86_64/aarch64 的 `libxynova` 和 `libcontrolcanfd`。默认从 wheel
内选择当前架构，也可用 `XYNOVA_LIB` 指向外部的匹配版本。在 wrapper 上层，
`XynovaSdkHand` 实现统一 `HandInterface`；配置、生命周期、action/observation、
23 关节 schema、异常转换和 Angle 标定换算均在无 ROS 依赖的 HAL 内完成。先运行只读
示例检查 CAN、设备 ID、序列号和 23 关节反馈：

```bash
python examples/read_xynova.py --can-interface 0.0
# 无硬件验证相同 interface
python examples/read_xynova.py --fake
```

运动示例默认不会发送命令；确认手周围安全后才显式启用：

```bash
python examples/control_xynova.py --execute --can-interface 0.0
```

core interface 和 SDK Angle API 的 position 单位均为弧度。可选的线性标定
通过 `XynovaConfig.servo_gains/servo_offsets` 配置：
`sdk_value = gain * radians + offset`。ROS YAML 的 `joint_calibration` 会生成同一份 HAL
配置，ROS 节点本身不做厂商单位换算。

ROS 2 驱动可在无硬件时 dry-run，也可连接真实设备：

```bash
cd ros2
colcon build --symlink-install --packages-select xynova_hand
source install/setup.bash
ros2 launch xynova_hand xynova_hand.launch.py dry_run:=true
```

在 YAML 的 `params` 下设置 `enable_tactile: true`，会发布 15 个区域的六分量触觉
wrench（Fx/Fy/Fz/Mx/My/Mz）到 `tactile_forces`。SDK 2.x 不再提供旧版 joint torque
反馈接口，`joint_states.effort` 因而为空。触觉默认关闭。

### Xynova Arm

`hardware/xynova/arm` 集成机械臂 SDK 0.7.1，包含厂商公共头文件、Linux x86_64/aarch64
运行库、项目自有 C ABI bridge，以及统一 `ArmInterface` 的真机和 mock 实现。当前支持
`XynovaRobot`（6 轴）、`XynovaCr5Robot`（5 轴）、`XynovaErRobot`/`XynovaArRobot`
（7 轴）、`StandardRobot`（6 轴工业）和 PCB3/PCB4。

HAL 提供关节位置/速度/力矩反馈、TCP 位姿、关节位置命令、停止和上下电。协作机型还可
通过 `read_wrench()` 读取工具坐标系下的三轴力与三轴力矩；工业机型的 SDK 不提供这一
查询。所有关节量均使用 SI 单位（rad、rad/s、Nm），TCP 平移为米、姿态转换为 xyzw
四元数。

连接机械臂不会自动上电。先只读检查机型、控制器版本和反馈，确认工作区、急停和目标
关节均安全后，再显式上电并发送命令：

```python
from litchi_hardware import DeviceAction, xynova

arm = xynova.create_arm(
    xynova.XynovaArmConfig(
        name="arm",
        remote_ip="192.168.1.160",
        local_ip="192.168.1.133",
        model=xynova.XynovaArmModel.XYNOVA_AR,
    )
)
arm.connect()
try:
    state = arm.read_joint_state()
    print(arm.info, state, arm.read_tcp_pose())
    arm.power_on()
    arm.send_action(DeviceAction({"joint_position": state.position}))
    arm.stop()
    arm.power_off()
finally:
    arm.disconnect()
```

`default_speed` 和 `default_zone` 默认为 `-1`，即使用控制器默认轨迹参数；也可在
`XynovaArmConfig` 中显式设置。`auto_power_on=False` 是安全默认值。
`XYNOVA_ARM_BRIDGE_LIB` 可覆盖 wheel 内的 bridge（外部 bridge 需自行解析匹配的 SDK）。
无硬件测试使用相同接口：

```python
arm = xynova.create_arm(config, use_fake_hardware=True)
```

### Linker Hand G20 / O30I

`hardware/linker` 支持厂家交付中的两代协议：G20 使用 Python SDK 3.1.0 和经典
SocketCAN；O30I 使用 ROS2 SDK 3.0.1 中提取的 HOP 0.0.3 控制器，可选透明 USB-CANFD
的 `socketcan` 或金属 CAN-FD 盒的 `libcanbus`。wheel 内含 x86_64 Ubuntu 20/22、
aarch64 和 armv7l 的 `libcanbus.so`，运行时按架构选择；宿主机仍需提供
`libusb-1.0`。

HAL 为 G20 和 O30I 分别定义显式 20 关节 schema，因为两者的厂家数组顺序不同。
HAL position 是弧度，默认使用 `vendor_tick = 255 * radians` 将 0–1 rad 映射到厂家
0–255；这是可运行的初始映射，不是每只手的物理标定。真机运动前必须按实物填写
`position_gains`/`position_offsets` 并核对方向。连接过程不会发送初始姿态或运动命令。

先用 fake 或只读示例验证：

```bash
python examples/read_linker.py --model O30I --fake --samples 3
python examples/read_linker.py --model O30I --side right --channel can0
python examples/read_linker.py --model G20 --side left --channel can0
```

运动示例默认只预览；确认标定和环境安全后才显式发送一次平滑往返：

```bash
python examples/control_linker.py --model O30I
python examples/control_linker.py --model O30I --fake --execute
python examples/control_linker.py --model O30I --side right --channel can0 --execute
```

ROS2 驱动同时支持两个型号及无硬件调试：

```bash
cd ros2
colcon build --symlink-install --packages-select linker_driver
source install/setup.bash
ros2 launch linker_driver linker_hand.launch.py use_fake_hardware:=true
```

真实 O30I 默认读取 `linker_driver/config/o30i.yaml`。透明 CAN-FD 盒使用
`transport: socketcan`；金属盒改为 `transport: libcanbus`。默认
`auto_setup_can: false`，请预先配置 CAN 接口；只有配置好免密 `sudo ip` 后才应开启自动
配置。G20 使用 `config/g20.yaml`。

## 启动真实 teleoperation

> 启动后会向真实灵巧手发送位置命令。先保证手周围无人员和障碍物，并准备硬件急停。

```bash
pixi install -e retarget-v4
pixi run stage-retarget-v4
pixi run -e ros2 ros-build
pixi run -e ros2 teleop \
  left_sharpa_serial:=WAVE-L-XXXX \
  right_sharpa_serial:=WAVE-R-XXXX
```

等价的原生 ROS 命令：

```bash
source ros2/install/setup.bash
ros2 launch manus_sharpa_teleop teleop.launch.py \
  use_fake_hardware:=false \
  left_sharpa_serial:=WAVE-L-XXXX \
  right_sharpa_serial:=WAVE-R-XXXX
```

常用 launch 参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `use_fake_hardware` | `false` | 同时切换 Manus 与 Sharpa fake backend |
| `manus_backend` | `native` | `native` 进程内 SDK；`zmq` 为隔离/兼容后端 |
| `enable_left` / `enable_right` | `true` | 启用对应手 |
| `manus_endpoint` | `tcp://127.0.0.1:2044` | 仅 ZMQ backend 使用的 client 数据地址 |
| `manus_manage_client` | `true` | ZMQ backend 是否启动并监督 client；外部服务设为 `false` |
| `manus_native_loopback_only` | `false` | native backend 只发现本机 Manus host |
| `left_sharpa_serial` / `right_sharpa_serial` | 空 | 指定硬件；空表示按手侧发现 |
| `retarget_backend` | `v4` | 真实链路默认使用上游 V4；`analytic` 仅用于 fake/回退 |
| `v4_filter_alpha` | `1.0` | 上游 V4 一阶关节滤波参数 |
| `v4_worker_python` | 自动 | V4 所需 Python 3.10；也可设 `LITCHI_RETARGET_V4_PYTHON` |
| `smoothing_alpha` | `0.35` | 仅 `analytic` backend 使用 |
| `flexion_scale` / `abduction_scale` | `1.0` | 仅 `analytic` backend 使用 |
| `speed_coefficient` | `0.3` | Sharpa 速度系数 |
| `current_coefficient` | `0.6` | Sharpa 电流系数 |
| `stop_on_command_timeout_s` | `0.0` | 大于 0 时丢命令后停机；停机后需重启节点 |

只测单手：

```bash
ros2 launch manus_sharpa_teleop teleop.launch.py \
  enable_left:=true enable_right:=false left_sharpa_serial:=WAVE-L-XXXX
```

### Wuji Hand 2

Wuji Hand 2 路径不使用 Sharpa V4 optimizer。retargeting 和硬件控制都直接调用官方
`wuji-sdk>=2026.8.3,<2026.9`。官方 Linux wheel 要求 glibc 2.34 或更新版本；Pixi 会在
安装时检查。手的静态 IP 和主机网卡必须在同一子网。

右手单手默认启动：

```bash
pixi install -e ros2
pixi run -e ros2 ros-build
pixi run -e ros2 teleop-wuji \
  manus_calibration_directory:=/absolute/path/to/manus/calibration
```

双手并显式指定序列号：

```bash
pixi run -e ros2 teleop-wuji \
  enable_left:=true enable_right:=true \
  left_wuji_serial:=LEFT_SERIAL right_wuji_serial:=RIGHT_SERIAL \
  manus_calibration_directory:=/absolute/path/to/manus/calibration
```

`manus_wuji_teleop` 从 Manus 25 点骨架中删除食指到小指各自多出的 metacarpal base，
得到官方 retargeter 要求的 MediaPipe 21 点顺序。`RetargetSession.step()` 输出已经是 Wuji
Hand 2 的 20 关节固件顺序，不再进行关节角拟合或重排。

retarget 输出按左右手分别滤波。默认 `smoothing_alpha:=0.35`，小指四关节使用更严格的
`pinky_max_joint_velocity_rad_s:=5.0` 限速；若小指仍过于敏感，可单独降到 `3.0`，不会拖慢
其他手指。

## ROS 接口

| Topic | 类型 | 方向 |
|---|---|---|
| `/manus/{left,right}/keypoints` | `geometry_msgs/PoseArray` | Manus driver 发布，固定 25 点厂商顺序 |
| `/sharpa/{left,right}/command` | `sensor_msgs/JointState` | retargeter 发布，22 关节弧度 |
| `/sharpa/{left,right}/joint_states` | `sensor_msgs/JointState` | 22 维 position/velocity/effort |
| `/sharpa/{left,right}/tactile/{finger}/raw` | `sensor_msgs/Image` | 指尖原始 tactile 图 |
| `/sharpa/{left,right}/tactile/{finger}/deformation` | `sensor_msgs/Image` | 指尖形变图 |
| `/sharpa/{left,right}/tactile/{finger}/wrench` | `geometry_msgs/WrenchStamped` | 指尖六维力/力矩 |
| `/wuji/{left,right}/command` | `sensor_msgs/JointState` | 官方 retargeter 输出的 20 关节弧度命令 |
| `/wuji/{left,right}/joint_states` | `sensor_msgs/JointState` | Wuji Hand 2 position/velocity/effort 反馈 |
| `/xynova_hand/joint_command` | `sensor_msgs/JointState` | Xynova 23 关节弧度命令 |
| `/xynova_hand/joint_states` | `sensor_msgs/JointState` | Xynova 23 关节角；effort 为空 |
| `/xynova_hand/tactile_forces` | `sensor_msgs/Image` | 可选 15 区域 × 六维 wrench，`32FC6` |
| `/linker/{side}/joint_command` | `sensor_msgs/JointState` | Linker G20/O30I 20 关节弧度命令 |
| `/linker/{side}/joint_states` | `sensor_msgs/JointState` | Linker 20 关节位置/速度反馈 |
| `/linker/{side}/tactile/{finger}` | `sensor_msgs/Image` | 可选 Linker 指尖矩阵触觉 |

Sharpa command 可以不带 names（必须是 SDK 的 22 元素顺序），也可以使用通用 names 或
`left_`/`right_` 前缀 names；驱动会重排并执行长度与关节限位检查。

## 测试

```bash
pixi run test
pixi run lint
pixi run wheel
pixi run -e ros2 ros-build
pixi run -e ros2 ros-test
```

纯 HAL 测试不需要 ROS 或硬件。端到端 fake launch 会贯通 25 点输入、22 关节
retargeting、命令订阅和状态回读。真实 launch 使用 V4 sidecar；fake launch 显式使用
`analytic`，避免测试依赖厂商优化器。

本地开发可安装 Git hooks：

```bash
pixi run hooks-install
```

提交前 hook 会对暂存的 Python 文件运行 Ruff，推送前 hook 会运行纯 HAL 测试。也可以
用 `pixi run hooks` 手动检查全部文件。GitLab CI 是最终质量门禁：它会在 CPython
3.10–3.13 上测试和构建 ABI wheel，并单独构建、测试 ROS2 workspace。

创建与 `pyproject.toml` 版本一致的 `vX.Y.Z` tag 后，pipeline 会提供手动的内部 PyPI
发布作业。GitLab 项目需要配置 protected、masked CI variables：
`TWINE_REPOSITORY_URL`、`TWINE_USERNAME` 和 `TWINE_PASSWORD`。执行发布作业的 Runner
必须能够访问该内部 PyPI 地址。

## 常见问题

- `Sharpa SDK is unavailable`：确认安装的是匹配当前 CPython ABI 的 Linux x86_64
  platform wheel；用 `sharpa.load_sdk()` 查看底层 loader 错误。
- wheel 被识别为 `py3-none-any`：这是不安全的旧构建，重新使用当前 `hatch_build.py`
  构建，文件名应含 `cp310-cp310-linux_x86_64` 等 platform tag。
- `Manus native SDK connection failed`：确认 Manus host/手套可发现、SDK 依赖完整，必要时
  设置 `manus_native_loopback_only:=true`；用 `manus.load_bridge()` 检查 loader。
- `no Manus frame received`（ZMQ）：确认 client 正在运行、endpoint 一致、Manus
  Core/手套已连接且校准文件可读。
- `wuji-sdk` 没有兼容 wheel：官方 2026.8.3 Linux wheel 要求 x86_64/aarch64、
  CPython 3.10–3.14 和 glibc 2.34+。
- Wuji Hand 2 无法发现：确认主机网卡与手的静态 IP 同网段；多只同侧手时通过
  `left_wuji_serial` / `right_wuji_serial` 消除发现歧义。
- 找错左右手：为两个 Sharpa 节点显式提供 serial。
- 真实手在丢帧后停止：若启用了 `stop_on_command_timeout_s`，这是终止式安全行为；
  排除数据问题后重启对应节点或整个 launch。
