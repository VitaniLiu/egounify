# litchi-hardware 架构:模块边界与职责划分

本仓库是一个**内部可复用的硬件封装层(HAL)**。目标设备:Franka 臂、Linker Hand G20/O30、Sharpa hand、Xynova hand。

核心原则:**HAL 抽象的是"设备",而不是"控制器"。**

---

## 1. 顶层结构

```
litchi-hardware/
├── src/litchi_hardware/       # 纯 Python HAL,零 ROS 依赖
│   ├── core/                  #   接口 + 值对象 + 状态机(纯净圈层)
│   └── hardware/              #   厂商耦合后端 + mock
│       ├── franka/
│       ├── linker/
│       ├── sharpa/
│       └── xynova/
│
└── ros2/                      # 单一 ROS workspace,面向 ROS 生态
    └── src/                   #   按"设备/功能域"分子目录(colcon 递归扫描,不影响构建)
        ├── linker_hand/       #   域:Linker 手集成
        ├── sharpa_hand/       #   域:Sharpa 手集成
        ├── xynova_hand/       #   域:Xynova 手集成
        ├── franka_bridge/     #   域:Franka 与 HAL 的读侧桥接
        └── vendor/            #   上游/外部只读依赖(franka_ros2 等)
```

> `ros2/src/` 的子目录详细划分见 [第 5 节](#5-ros2--面向-ros-生态的单一-workspace)。

### 两条硬边界

1. **`src/`(HAL)永不依赖 `ros2/`** —— 方向单向,这是唯一不可协商的约束。纯 HAL 必须能在无 ROS 环境下使用与测试。
2. **`ros2/` 内每个包按需依赖 `src/`** —— 当 ROS 只是设备传输/状态桥时依赖并实现 core 接口;当它是自治 C++ controller 时就不依赖。**同一目录,自由选择。**

---

## 2. `src/litchi_hardware/` —— 纯 Python HAL

分为两个圈层:纯净的核心契约(`core/`,零外部依赖)与厂商耦合的实现(`hardware/`)。

```
src/litchi_hardware/
├── core/                      # ── 纯净圈层:零外部依赖,不 import 任何 SDK/rclpy ──
│   ├── interfaces.py          #    DeviceInterface → ArmInterface / HandInterface
│   │                          #    + TactileCapable(可选能力协议)
│   ├── types.py               #    JointSchema / HandSkeletonSchema + 采样值对象
│   │                          #    + FeatureSpec / DeviceAction / DeviceObservation
│   ├── config.py              #    DeviceConfig(基) / DeviceInfo
│   ├── lifecycle.py           #    ConnectionState 枚举 + 状态机 guard(不变量)
│   └── errors.py              #    DeviceError / ConnectionError / DependencyUnavailableError
│
├── hardware/                  # ── 厂商耦合圈层:依赖 core,永不反向 ──
│   ├── franka/                #    臂:ArmInterface
│   │   ├── base.py            #      FrankaArmBase(矩阵→位姿、共享观测/动作组装)
│   │   ├── config.py          #      FrankaConfig(后端无关)
│   │   ├── libfranka.py       #      backend(extras: franka-libfranka)
│   │   ├── franky.py          #      backend(extras: franka-franky)
│   │   ├── mock.py            #      进程内 mock,无硬件
│   │   └── factory.py         #      create real/mock
│   │
│   ├── linker/                #    手:HandInterface (+ TactileCapable)
│   │   ├── base.py            #      LinkerHandBase
│   │   ├── config.py          #      LinkerConfig(型号、手侧、传输与标定)
│   │   ├── schema.py          #      G20/O30I 各自的 20 关节顺序契约
│   │   ├── sdk_driver.py      #      G20 SocketCAN / O30I HOP CAN-FD 后端
│   │   ├── mock.py
│   │   └── factory.py         #      create_hand / create_pair
│   │                          #      ROS2 仅桥接统一 HAL，不承载厂家协议
│   │
│   ├── sharpa/                #    手:HandInterface(base/config/driver/mock/factory)
│   └── xynova/                #    手:HandInterface(结构同 sharpa)
│
└── registry.py               #    轻量后端注册表(entry-point 发现,无 ROS import)
```

### core 与 hardware 的分工

| 核心业务逻辑(`core/`,纯 Python) | 外部依赖(`hardware/`,厂商耦合) |
|---|---|
| 抽象接口、值对象、FeatureSpec、状态机不变量、统一异常 | libfranka/franky/Linker SDK 绑定、CAN 比特率/节点 ID、型号解析细节、厂商错误类型 |
| **不 import** 任何 SDK、rclpy、ROS 消息 | 具体 backend、Factory、SDK 适配 |

**依赖规则**:`hardware/*` 依赖 `core`,永不反向。一族 = `base.py` + `config.py` + `factory.py` + `mock.py` + 后端。

---

## 3. 核心领域实体

领域分为两个圈层:纯净的核心契约与厂商耦合的实现。

| 实体 | 圈层 | 说明 |
|---|---|---|
| **Device**(`DeviceInterface`) | core | 一切可控硬件的共同基底:连接生命周期 + 关节 + 观测/动作 + 急停 |
| **Arm**(`ArmInterface`) | core | Device 扩展,新增 TCP 位姿(`read_tcp_pose`)、外力/力矩 |
| **Hand**(`HandInterface`) | core | Device 扩展,手掌语义。**保持小而精** |
| **TactileCapable** | core | *可选能力协议* —— 只有带触觉/矩阵触觉的手(Linker)才实现 |
| **JointSchema / HandSkeletonSchema** | core | 冻结、具名且带版本的顺序契约；区分“同样长度、不同语义”的关节或骨架 |
| **JointState / HandTrackingFrame / Pose / TactileFrame** | core | 带 schema 的采样值对象(position/velocity/effort、关键点、位置+四元数、taxel 阵列) |
| **DeviceObservation / DeviceAction / FeatureSpec** | core | 载荷映射 + schema 声明,后端据此校验命令 |
| **ConnectionState** | core | 生命周期枚举 `DISCONNECTED/CONNECTING/CONNECTED/FAILED` |
| **DeviceInfo / DeviceConfig** | core | 设备身份 + 配置基协议 |
| **HAL Errors** | core | `DeviceError / ConnectionError / DependencyUnavailableError` 等统一异常 |
| **FrankaArmBase / LinkerHandBase / …** | hardware | 设备族共享逻辑(矩阵→位姿、型号解析) |
| **Backend**(libfranka/franky/g20_can/mock…) | hardware | 具体传输实现,翻译 SDK 异常,拥有真实状态转移 |
| **VendorConfig**(`FrankaConfig`/`LinkerConfig`/…) | hardware | 型号、传输、标定和厂商特定连接参数 |
| **Factory** | hardware | 创建 real/mock/pair 实例 |

### 关键领域判断:Device / Arm / Hand 三层拆分

**手不是"没有 TCP 的坏机器人",它是一等设备。** 若让手臂共用单一接口,手会被迫实现 `read_tcp_pose()` 这类臂专属方法(只能返回 `None` 或抛异常 —— 语义污染)。因此:

```
DeviceInterface(共性:生命周期 + 关节 + 观测/动作 + 急停)
  ├── ArmInterface   (+ read_tcp_pose / read_wrench)
  └── HandInterface  (+ 手掌抓握语义)

TactileCapable(可选能力协议:read_tactile → TactileFrame)
  由 Linker 等带触觉的手 mixin,不进 HandInterface 强制方法
```

触觉做成**能力协议**而非强制方法,因为 Linker 矩阵触觉 / Sharpa / Xynova 差异巨大。

---

## 4. 连接生命周期状态机

每个设备持有一个 `ConnectionState`:

```
DISCONNECTED ─connect()→ CONNECTING ─成功→ CONNECTED ─disconnect()→ DISCONNECTED
                             └─失败→ FAILED ──reset──→ DISCONNECTED
```

- **不变量归 core**:不得跳过 `CONNECTING` 直达 `CONNECTED`;失败必落到 `FAILED`。core 提供轻量 lifecycle guard。
- **具体转移归 backend**:什么算"失败"、如何清理 CAN/socket/SDK 句柄、能否重连、SDK 异常翻译 —— 都是传输相关的,由各后端实现。

**行为契约(`DeviceInterface`)**:`connect / disconnect / is_connected / connection_state / read_joint_state / observation_features / action_features / get_observation / send_action / stop`。

- `ArmInterface` 增:`read_tcp_pose()`、(后续)`read_wrench()`。
- `HandInterface` 增:手掌级抓握语义。
- `TactileCapable` 增:`read_tactile() -> TactileFrame`。

---

## 5. `ros2/` —— 面向 ROS 生态的单一 workspace

`ros2/` 不按"依不依赖 HAL"拆分 —— 依赖与否是**包级别的自由**,不体现在目录划分上。ROS 相关的一切(Python 节点 + C++ controller)在**同一个** workspace 下平级共存,这是 ROS workspace 的常态。

### "ROS2" 的两种性质

必须区分两种性质完全不同的 ROS2 用法:

| | **A. ROS2 作为"传输后端"** | **B. ROS2 作为"实时控制平面"** |
|---|---|---|
| 例子 | Linker ROS bridge 调用纯 HAL CAN/CAN-FD 后端 | Franka C++ `ros2_control` controller(零力示教、阻抗、力控) |
| 本质 | ROS2 只是一条到设备的"线" | ROS2 + C++ 本身就是控制器,跑在 1kHz 实时环 |
| 谁实现 HAL 接口 | 实现 `HandInterface`,让上层无感知 | **不实现** —— 它比 HAL 更底层 |
| 依赖 `src/` | 是(实现 core 接口) | 否 |
| 契约边界 | Python 抽象类 | ROS topic/action |
| 为什么 | 需被 Python 侧统一编排 | 经 Python 中转会毁掉实时性(GIL、序列化、抖动),C++ 直连 FCI 才有力控效果 |

### 集成方式:用 ROS 接口而非 Python 继承

Python 侧要用到 C++ controller 时,**不是调用它,而是订阅/发布它的 ROS 接口**。若需要 HAL 视图,写一个薄薄的 *state-only* 读适配器(`franka_bridge_server`):内部只 `subscribe` controller 广播的 `franka_robot_state` 来实现 `get_observation`,高频命令仍交给 C++ controller。这是**读侧适配**,不把实时环塞进 Python,`src/` 依旧零 ROS。

### 子包划分:按"设备/功能域"分子目录

`colcon` 递归扫描 `package.xml`,子目录纯粹是**给人看的组织手段,不影响构建**。按**设备/功能域纵切**(而非按语言或类型横切),内聚最高。这也与团队现有惯例一致(既有的 `franka_spine/` 已用子目录容纳 `franka_spine_server` / `franka_spine_msgs` / `franka_spine_examples` 三件套)。

```
ros2/src/
├── linker_hand/                # 域:Linker 手的 ROS2 集成
│   ├── linker_hand_msgs/       #   自定义消息(手指位姿/触觉)
│   ├── linker_driver/          #   G20/O30I ROS bridge(依赖 src/ 的 HandInterface)· A 类
│   └── linker_hand_examples/   #   使用示例
│
├── sharpa_hand/                # 域:Sharpa 手(结构同 linker_hand)
│   ├── sharpa_hand_msgs/
│   └── sharpa_driver/          #   依赖 src/,实现 HandInterface · A 类
│
├── xynova_hand/                # Xynova ROS 包
│                               #   依赖 src/,仅通过 HandInterface 控制 · A 类
│
├── franka_bridge/              # 域:Franka 与 HAL 的桥接(读侧)
│   ├── franka_bridge_server/   #   订阅 franka_robot_state → 实现 ArmInterface 读侧 · A 类
│   └── franka_bridge_examples/
│
└── vendor/                     # 外部/上游只读依赖(不改动)
    └── franka_ros2 -> ...      #   指向兄弟目录 franka_ros2_0624 或 git submodule · B 类
```

**划分规则**:

| 层级 | 是什么 | 命名 |
|---|---|---|
| **子目录** = 功能/设备**域** | 逻辑分组,`colcon` 递归扫描,不影响构建 | `linker_hand/`、`franka_bridge/` |
| **域内包** = `<域>_<角色>` | 真实 `package.xml` 包 | `_msgs`(消息)、`_driver`/`_server`(逻辑)、`_examples`(示例) |
| **`vendor/`** | 上游/外部只读依赖 | `franka_ros2` 归此,自治不被改 |

**为什么这样最合适**:

1. **镜像 HAL 边界**:每个设备域一个子目录,正好对应 `src/hardware/<vendor>/` —— ROS 侧与 Python HAL 侧对称,谁实现哪个接口一目了然。
2. **A/B 类物理隔离**:`vendor/franka_ros2`(B 类自治 C++)与 `franka_bridge/`(A 类读侧)分属不同子目录,依赖边界肉眼可辨。
3. **`_msgs` 独立成包**:ROS2 最佳实践 —— 消息接口包必须与实现分离,才能被 C++/Python 双侧和外部复用。
4. **反模式提醒**:**不要**按"语言"(`python/`、`cpp/`)或"类型"(`msgs/`、`drivers/`)横切分子目录,那会把同一设备的东西打散到多处;按设备/功能域纵切,内聚最高。

### 外部依赖 `franka_ros2_0624` 的定位

兄弟目录 `../franka_ros2_0624/` 是**上游官方 [frankarobotics/franka_ros2](https://github.com/frankarobotics/franka_ros2) 栈 + 团队自定义包**(`franka_hardware` ros2_control 硬件接口、`franka_example_controllers`、`franka_spine`、`moveit_dual`、`olvx_descriptions_module` 等)的自包含 ROS2 workspace。全仓库**零处**依赖本 HAL。

它是本架构 **"B 类实时控制平面"的现成实例**:自治 C++ `ros2_control` 栈,契约边界是 ROS topic/action。集成时:

- **归入 `ros2/src/vendor/`**(符号链接或 git submodule),保持独立只读,**绝不进 `src/`**,也**绝不被 `src/` 依赖**。
- Python 侧需要 Franka 状态的 HAL 视图时,由 `franka_bridge/franka_bridge_server` 订阅其 topic 实现读侧 `ArmInterface`;高频命令仍走它的 C++ controller,实时性不受损。

---

## 6. 边界规则总表

| 单元 | 语言 | 依赖方向 | 契约边界 | 何时用 |
|---|---|---|---|---|
| `src/litchi_hardware`(core+hardware) | Python | 圈层内 `hardware → core`,永不反向 | Python 抽象类 | 所有非实时设备编排的公共契约 |
| `ros2/src/<device>/` 内驱动/桥接包(linker、sharpa、xynova、franka_bridge) | Python | `→ src/`(调用 core 接口) | Python 抽象类 | ROS2 只是"到设备的线"或状态桥 · A 类 |
| `ros2/src/vendor/franka_ros2`(C++ controller) | C++ | **零依赖 src/** | ROS topic/action | ROS2+C++ 本身即控制器(实时力控) · B 类 |

**四条硬边界**:

1. `core/` **绝不** import 任何 SDK、`rclpy`、ROS 消息。
2. `hardware/*` 依赖 `core`,**永不反向**。
3. `src/`(HAL)**永不**依赖 `ros2/` —— 方向单向。
4. Franka 实时控制**不进 HAL** —— 独立 C++ 包,Python 侧通过订阅其 topic 获得 HAL 视图,命令仍走 C++,实时性不受损。

---

## 7. 一句话总结

> HAL 抽象的是"设备",不是"控制器"。当 ROS2 只是通往设备的线时,把它藏进后端(实现 core 接口);当 ROS2+C++ 本身就是控制器时,让它自治,用 ROS 接口而非 Python 继承来集成。

---

## 8. Manus → Sharpa teleoperation 纵切

本节把上述通用边界落实到 Manus MetaGlove 与 Sharpa Wave。参考实现分别位于
`reference/SDKMinimalClient_Linux`、`reference/sharpa_extracted` 和
`reference/sharpa-manus-sdk`，它们只用于确认厂商契约，**不是运行时依赖**。

### 8.1 领域实体与职责

| 实体 | 圈层 | 职责、状态与行为 |
|---|---|---|
| **HandTracker** (`HandTrackerInterface`) | core | 稳定的输入设备概念：连接生命周期、按左右手读取一帧手部关键点；不伪装成可接受动作的机器人手 |
| **HandTrackingFrame** | core | 冻结值对象：时间戳、坐标系、左右手、关键点位姿及显式 `HandSkeletonSchema` |
| **Manus tracker backend** | hardware/manus | native 进程内 SDK callback 或 ZMQ 隔离传输，将两者统一翻译成 `HandTrackingFrame` |
| **Sharpa hand backend** | hardware/sharpa | 管理 Sharpa SDK 生命周期，将统一弧度关节动作翻译成 SDK 调用，并将 SDK 错误翻译成 HAL 错误 |
| **Sharpa V4 retargeter** | ros2/manus_sharpa_teleop | 将 25×7 Manus 位姿交给上游 URDF/CasADi 优化器，输出 Sharpa 的 22 个关节目标 |
| **Analytic fallback** | ros2/manus_sharpa_teleop | 无厂商运行时的 fake 测试/显式回退实现，不用于默认真实硬件链路 |
| **Teleop ROS node** | ros2/manus_sharpa_teleop | 订阅关键点、运行控制策略并发布目标关节 |

`HandTracker` 是新增实体而不是 `HandInterface` 的特例：Manus 是只产生观测的穿戴式
输入设备，强迫它实现 `send_action()`、`stop()` 等执行器语义会污染既有 `Hand` 契约。
Retargeting 是跨设备控制策略，不属于 HAL，因此 `src/litchi_hardware` 不定义
`HandRetargeter` 接口，也不包含 optimizer、URDF 或 teleop 实现。

### 8.2 厂商契约与核心逻辑的分界

| 核心业务逻辑 | 外部依赖 |
|---|---|
| HAL：schema、动作校验、设备生命周期；ROS teleop：V4 优化、滤波和失帧策略 | Manus 的 25 点契约、Sharpa 的 22 关节契约、V4 optimizer/URDF、SDK 回调、ZMQ/Protobuf、设备发现和网络 |

官方 Sharpa-Manus V4 优化器是面向 Python 3.10 发布的原生扩展，而本仓库 ROS2
环境使用不同的 Python ABI。因此 ROS 包内的 `SharpaV4Retargeter` 通过本机 Unix
socket 调用隔离的 Python 3.10 sidecar；sidecar 使用上游
`HandRetargetingOptimizer`、Wave URDF 和
`JointAngleFilter`。ROS 进程不加载该扩展，也不继承 ROS `PYTHONPATH`。解析实现保留为
fake 测试和显式回退，不再作为真实 launch 的默认实现。

### 8.3 文件结构与依赖方向

```text
src/litchi_hardware/
├── core/
│   ├── interfaces.py          # 设备接口（含 HandTrackerInterface）
│   └── types.py               # + HandSide / HandTrackingFrame
├── manus.py / sharpa.py       # public facade + lazy native loader/path API
├── hardware/
│   ├── manus/
│   │   ├── _vendor/           # MANUS Integrated SDK 资产和独立 manifest
│   │   ├── schema.py          # Manus 25 点顺序实体
│   │   ├── config.py          # shared/native/ZMQ 三类配置
│   │   ├── native_driver.py   # 只做 HAL 生命周期与值对象适配
│   │   ├── zmq_driver.py      # ZMQ 后端与可选 client 进程管理
│   │   └── _native/           # C ABI、ctypes FFI、纯 Python bridge model
│   └── sharpa/
│       ├── _vendor/           # Sharpa SDK 资产和独立 manifest
│       ├── schema.py          # Sharpa 22 关节顺序、单位与限位实体
│       └── config/base/sdk_driver/mock/factory

ros2/src/
├── manus_glove/manus_driver/  # HandTrackingFrame ↔ PoseArray
├── sharpa_hand/sharpa_driver/ # JointState command/state ↔ Sharpa HAL
└── manus_sharpa_teleop/
    ├── manus_sharpa_teleop/   # V4 .so + retarget node/worker + launch
    └── sharpa_wave_description/ # 左右手 URDF 独立 ROS package
```

依赖只能沿下列方向：

```text
manus_sharpa_teleop → litchi_hardware（只读取类型/schema）
manus_driver / sharpa_driver → hardware backends → core
```

所有真实后端都有同契约的 fake 实现。ROS launch 的 `use_fake_hardware` 同时切换
Manus 与 Sharpa，确保无 SDK、无 ROS 硬件时仍能贯通关键点、retargeting、命令和状态
整条链路。真实模式默认使用 `ManusNativeTracker`：C++ C-ABI bridge 在进程内初始化
Integrated SDK，注册 raw skeleton/landscape callback，并把最新帧放入固定大小缓冲区；
Python 阻塞读取该缓冲区，不会在 SDK callback 线程执行 Python。`ManusZmqTracker`
作为显式可选后端，可管理 wheel 内的 Sharpa client 子进程，也可只订阅外部服务。
两种后端分别接受 `ManusNativeConfig` 与 `ManusZmqConfig`；共同字段留在
`ManusConfig`。`_native/ffi.py` 独占 ctypes/C ABI 布局，向 `native_driver.py` 暴露的
边界是普通的 `NativeSkeletonFrame`，因此 HAL adapter 与桥接 ABI 可独立测试和演进。
`reference/` 只在维护者运行 `tools/vendor_sdks.py` 或
`tools/stage_retarget_v4_ros.py` 更新私有资产时读取，安装后的 HAL/ROS 包都不依赖
仓库下的 reference 路径。

### 8.4 SDK 分发边界

`litchi_hardware` 的 Python 代码仍保持零 ROS 依赖，但私有 Linux platform wheel 会把
厂商 runtime 作为 package data 纳入：

- Sharpa：C++ headers、两套 `.so`、`pilot_sdk`、config、CPython 3.10–3.13 extensions。
- Manus 厂商资产：C++ headers、Integrated `.so`、校准文件和编译后的 ZMQ client，
  位于 `hardware/manus/_vendor`；不分发 Remote/Core SDK 及旧版 gRPC/Protobuf runtime。
- Sharpa 厂商资产位于 `hardware/sharpa/_vendor`。
- 项目自有 Manus 适配层：C-ABI bridge 的源码、头文件和 `.so`，全部位于
  `hardware/manus/_native`，不计入 vendor manifest。
- 每个硬件 `_vendor/manifest.json` 分别固定本域资产的 size/SHA-256；大二进制由
  Git LFS 管理。

这些二进制属于 `hardware` 圈层的外部依赖实现，不进入 `core`。wheel 必须按构建
解释器标为 `cpXY-cpXY-linux_x86_64`，不能发布为 `py3-none-any`。由于厂商许可限制，
该 wheel 仅通过组织内部私有 index 分发。

V4 retargeting 不进入 `litchi_hardware` wheel：两个 CPython 3.10 optimizer `.so`
由 `manus_sharpa_teleop` ROS 包安装，左右手 URDF 由
`sharpa_wave_description` ROS 包安装，数值依赖由独立 `retarget-v4` pixi 环境提供。
