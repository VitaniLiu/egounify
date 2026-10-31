> 历史记录：下文76维方案已废止，当前以[统一78维转换实施与使用说明](统一78维转换实施与使用说明.md)为准。旧数据检查结果保留作对照。

# Apex 同步数采与 76 维转换操作说明

核验日期：2026-10-28（北京时间）。适用机器：av-pipeline 与 ms-thor；MANUS → Sharpa 双手；Gento 机器人。

## 1. 当前交付与边界

本轮承接的任务是：将双手和身体的录制统一接入 Apex Record/Stop，并说明 Raw 与后续转换契约。

| 项目 | 实际状态 |
|---|---|
| 原 Gento Record/Stop 接入 | 已安装 launch remap 与录制协调器，保留原界面、原生 MCAP writer |
| 同包记录 | 双手 state、SDK 成功目标、context、health、capture_event 与身体进入同一数值录制器 |
| 统一时间轴 | 使用机器人 MCAP log 时间，`robot_receive`、int64 纳秒 |
| 失败处理 | 缺流拒绝开始；断流触发停止；收尾失败保留事务并重试；不完整样本不标 COMPLETE |
| Raw 完整性 | manifest、completion、逐文件 SHA256、流覆盖及同包来源核验 |
| 软件验证 | 7 项单元测试通过；ROS domain 199 原生 writer 隔离测试通过；手部专测360条，完整数值profile另测356条手部样本 |
| 正式硬件验收 | 未完成；当前 `apex-teleop.service` 未运行，双手遥操未运行；本轮没有启动运动模块 |
| 现有手部 HDF5 提取 | 已实现，从认证 Raw 中提取；不再必须提供外置 JSONL |
| 完整 LeRobot76 导出 | **尚未实现**；现有公共导出器仍支持 78/27 维，旧手部 derive 仍写 78 维槽位 |
| 身体＋双手联合 replay | **尚未实现统一起点握手**；不能把独立手部 replay 与身体 replay 同时手动启动称为同步回放 |

同步录制的软件接入已具备；只有经过真实身体、双手与相机完整短录制验证后，才能认定正式数采通过。本文不会将隔离假设备样本作为硬件采集结果。

## 2. 部署位置与控制路径

在本机访问主机：

```bash
ssh av-pipeline-lan
# 当前地址：descfly@192.168.31.27
# 从 av-pipeline 访问机器人：
ssh user@6.6.7.100
```

本轮使用局域网别名；原 `av-pipeline` VPN 地址连接关闭，未改动 SSH 配置。

| 所在机器 | 路径 | 作用 |
|---|---|---|
| av-pipeline | `/home/descfly/apex_n_sharpa` | 原数采与转换项目 |
| av-pipeline | `/home/descfly/apex_n_sharpa/integrations/manus_sharpa` | 手部模块入口，软链接 |
| av-pipeline | `/mnt/apex-data/apex_n_sharpa/manus_sharpa` | 手部模块实际目录 |
| ms-thor | `/home/user/manus_sharpa/robot_receiver.py` | 手部数据进入机器人录制时钟域的接收器 |
| ms-thor | `/home/user/apex_record_sync` | 协调器、配置、审计器、测试及部署备份 |
| ms-thor | `/home/user/gento_replay_bags` | 原生 Raw 录制目录 |

实际调用链：

```text
Apex 的 Gento Record 按钮
  → 8012 端口 /api/gento_replay/record
  → /tj/control/gento_replay/record_named（同步协调器）
  → /tj/control/gento_replay/record_named_native（原生 writer）
```

同时保留 `/tj/control/gento_replay/record` 的录制兼容入口。状态由 `/tj/info/gento_replay/status` 发布；原生状态 remap 到 `native_status`。不要绕过协调器直接调用 `*_native`，否则不能生成认证完成包。

安装涉及：

- `/opt/kernelmind/apex/install/gento_replay/share/gento_replay/launch/gento_replay.launch.py`
- `/opt/kernelmind/apex/install/gento_replay/share/gento_replay/config/gento_replay.yaml`

配置已写入，**下次正常启动 Apex 遥操模块时加载**。本轮未强制启动、重启身体控制或手部服务。

## 3. 实际操作顺序

1. 按现有现场流程启动机器人与 MANUS→Sharpa 遥操，确认设备持续反馈正常。
2. 左手必须绑定 `CE50903ECE53`，右手 `CC50933FCC53`。手部在 ROS domain 198；机器人录制在 domain 29；通过 SSH FIFO 转发事件。
3. 使用原 Apex Gento Record 按钮开始录制。协调器会检查必需流、手部 READY、序列号、live 标记、相机、原生 writer 状态及空间。
4. 如返回 missing/stale 或 hands not READY，先排查所列流，不能关闭检查来冒充完整采集。
5. 使用原 Stop 按钮结束。结束响应会等待停止、写入队列排空、相机退出录制和落盘审计。
6. 以 `capture_completion.json` 的 `state=COMPLETE` 为准，并运行完整性校验；仅有目录、MCAP、UI 停止状态或 `capture_time.json` 都不表示成功。
7. 将整个 Raw 目录复制到主机，再离线转换。录制结束不要求结束手部遥操，会话可用于下一条录制。

Record/Stop **只控制录制**。手部实体启动命令会连接和使能硬件，属于已有遥操流程，不由 Record 自动执行。录制失败也不负责代替硬件急停或控制器的停止保护。

## 4. 开始、停止与失败的定义

### 开始

- 所有必需话题最近到达时间不超过 0.5 秒；相机也需持续到达。
- 手部 health schema 正确、READY、live=true、左右序列号正确，并有同一 source session。
- 原生 recorder 状态新鲜，没有正在录制或播放；剩余空间至少 1 GiB。
- 生成唯一 `capture_id`，调用原生开始，定位唯一的新目录，写入初始 manifest。
- 等待 event 订阅匹配并重复发布开始标记。标记未真正落盘时，最终审计拒绝 COMPLETE。
- 开始标记中保留最近 context 历史，供已存在目标的事件关联使用；这些历史项仍保留原始事件身份和时间，不是新执行命令。

### 停止

- 发布停止标记；即使标记发布异常，也必须调用原生停止。
- 等待新的原生状态确认 recording=false、camera_recording=false、write_queue_size=0、queued_messages=written_messages。
- 检查文件大小稳定，实际读取 MCAP，核对必需流、事件、来源、首尾覆盖和数据间断；读取相机 MCAP 检查帧数、覆盖与最大间隔。
- 写入最终 manifest，再写 completion；两者通过哈希相互关联。
- 原生停止失败保留 active 事务并进入 STOP_FAILED，watchdog 后续重试；完成标记写盘失败也保留事务，不能假成功。
- 一旦本次事务发生错误，即使重试成功完成物理落盘，也仍标记 FAILED，保留故障原因。

### 阈值含义

| 检查 | 当前阈值 | 含义 |
|---|---:|---|
| 开始/录制中的流新鲜度 | 0.5 s | 协调器最近实际收到消息的年龄 |
| 落盘数值流最大间隔 | 0.25 s | 原频率 MCAP 内相邻记录间距 |
| 数值流窗口覆盖 | 0.25 s | 第一条相对 start_ack、末条相对 stop_request 的允许差距 |
| 相机首尾覆盖 | 0.10 s | 相机相对数值录制首尾覆盖误差 |
| 相机落盘最大间隔 | 0.25 s | 防止只有首尾帧、中途长期断帧仍通过 |
| 原生状态超时 | 2 s | 录制过程中丢失 writer 状态时失败停止 |
| 训练 20 Hz 匹配误差 | 50 ms | **转换阶段**逐帧判定，不等同于录制阈值 |

因此 COMPLETE 代表通过 Raw 录制契约，不等于已经满足训练转换的更严格时间匹配要求。很短的录制可能达不到每流至少两条的条件。

## 5. Raw 的预期目录格式

以下使用已实测原生 writer 布局；数值 MCAP 位于 episode 根目录，不能强行假定有 `data/`：

```text
/home/user/gento_replay_bags/<录制名>/
├── <录制名>_0.mcap               # 数值、context、health、capture_event
├── metadata.yaml                # rosbag2 元数据，可能有多个 MCAP 分卷
├── video/                       # 相机 writer 自己的 MCAP/metadata 层级
│   └── ...
├── capture_time.json            # UTC 起始时间、capture_id、clock_domain
├── capture_manifest.json        # 最终来源、窗口、审计与文件哈希
└── capture_completion.json      # 最后写入的完成凭证
```

不要只复制主 `.mcap`；必须保留全部分卷、相机文件、metadata 和 sidecar。隔离测试禁用相机，因此测试包没有 video，这不适用于正式配置。

### capture_manifest.json

| 字段 | 类型/含义 |
|---|---|
| schema | `apex_capture_manifest_v1` |
| capture_id | 32 位 UUID hex，本次 Record 的唯一身份 |
| source_session_id | 手部桥会话 ID；不是 capture_id；一个遥操会话可包含多次 Record |
| state | `RECORDING` / `COMPLETE` / `FAILED`；以最终 state 为准 |
| diagnostic | bool；测试包 true，禁止进入正式训练 |
| clock_domain | 最终包为 `robot_receive` |
| start_request_ns / start_ack_ns | 机器人请求/收到开始响应的本机时刻 |
| stop_request_ns / stop_ack_ns | 机器人停止事务的本机时刻 |
| serials | `left`、`right` 的 Sharpa 序列号 |
| required_topics | 话题名 → ROS 消息类型 |
| audit | 各流数量、首末 log 时间、最大间隔、来源验证数量、问题列表、writer 状态及相机统计 |
| failure_reason | null 或具体故障；失败包不得进入正式训练 |
| raw_files | 相对路径 → `{size, sha256}`，包括 MCAP、视频和原生 metadata 等 |

`bag` 是生成时的原机路径，搬迁后仅用于来源审计；访问文件应使用当前 Raw 根目录与 raw_files 的相对路径。

### capture_completion.json

```json
{
  "schema": "apex_capture_completion_v1",
  "capture_id": "与manifest一致",
  "state": "COMPLETE",
  "diagnostic": false,
  "manifest_sha256": "capture_manifest.json文件的SHA256"
}
```

校验时要求两个文件均 COMPLETE、capture_id 和 diagnostic 一致、manifest 哈希正确、所有登记文件大小和哈希正确；默认拒绝 diagnostic。拷贝工具自己的 manifest 只证明拷贝完整，不能代替 capture completion。

## 6. 必需话题与内容

| 话题 | ROS 类型 | 用途 |
|---|---|---|
| `/tj/joint_states` | sensor_msgs/msg/JointState | 身体、头部、双臂实测关节；头部 EEF FK 来源 |
| `/tj/info/eef_left`、`/tj/info/eef_right` | geometry_msgs/msg/PoseStamped | 左右臂实测 EEF |
| `/tj/control/target_poseL`、`/tj/control/target_poseR` | geometry_msgs/msg/PoseStamped | 左右臂目标 EEF |
| `/tj/control/vr_body` | marvin_msgs/msg/VrBody | 包含头部目标位姿 |
| `/tj/control/ik_request` | marvin_msgs/msg/IKRequest | 原生 IK 输入及 replay 需要的全字段 |
| `/tj/control/joint_cmd_body` | marvin_msgs/msg/JointcmdBody | 升降/腰部三个关节的控制目标 |
| `/tj/control/joint_cmd_head` | marvin_msgs/msg/JointcmdHead | 两个头部关节的控制目标 |
| `/recording/chassis/state` | move/msg/State | 底盘真实状态 |
| `/recording/chassis/manual_move_cmd` | geometry_msgs/msg/TwistStamped | 底盘实际命令记录副本 |
| `/hand_left/joint_states`、`/hand_right/joint_states` | sensor_msgs/msg/JointState | 双手真实反馈 |
| `/hand_left/joint_commands`、`/hand_right/joint_commands` | sensor_msgs/msg/JointState | 双手 SDK 成功提交的最终目标 |
| `/recording/sharpa/context` | std_msgs/msg/String | JSON 事件证据，必须与手部样本同包 |
| `/recording/sharpa/health` | std_msgs/msg/String | READY、session、serials、live、年龄及计数 |
| `/recording/capture_event` | std_msgs/msg/String | start/stop、capture_id、窗口及初始历史 |
| `/recorder/quad_tile/compressed_undistorted` | sensor_msgs/msg/CompressedImage | 相机录制；实际编码按 format 与现有解码器解释 |

原录制白名单中的额外身体、IK、底盘话题继续保留。夹爪历史 topic 可能仍出现在通用白名单里，但没有进入新 76 维目标向量。

## 7. 手部消息与来源证据

每手 22 关节，固定名称顺序见随附 `天机76维目标契约.json`；位置为有限浮点数、单位 rad。

JointState 使用：

- `name[22]`：标准关节名称。
- `position[22]`：state 是读取反馈，commands 是 SDK 成功提交后的最终目标。
- `header.frame_id = manus_sharpa/<session_id>/<seq>`：与 context 的唯一关联键。
- `header.stamp`：**机器人 receiver 收到该事件的时刻**，不是原 MANUS header。

context 的 String.data 为 JSON：

| 字段 | 含义 |
|---|---|
| schema | `manus_sharpa_event_v1` |
| session_id、seq | 事件身份，seq 为严格连续递增整数 |
| kind | state / request / accepted / rejected；只有 accepted 可成为手部 action |
| side、serials、live | 侧别、设备绑定、真实/诊断来源 |
| names、position | state / accepted 的标准名称及值 |
| source_stamp_ns | 主机侧源事件时间；反馈通常来自反馈 header，accepted 来自 SDK 结果时间 |
| received_host_ns | 主机 bridge 进入队列时间 |
| source_monotonic_ns | 主机单调时钟，用于本机队列年龄检查，不能跨主机比较 |
| received_robot_ns | 机器人 receiver 到达时间 |
| clock_domain | `robot_receive`，不改变各 source 字段原有的来源含义 |
| sdk_result | accepted/rejected 的调用结果；accepted 要求 status=`sdk_call_succeeded` |

SDK 成功只表示接口成功提交，不证明关节已经物理到位。MANUS request 不能代替 SDK 成功后的 action；更不能将 feedback 同时作为 action。

核验器要求同包 context 或开始快照能解释每个 JointState 的 session/seq，且角色、序列号、时间、名称和值一致。外置 receiver JSONL 仍可作为诊断日志保留，但新的认证 Raw 不依赖它。

## 8. 时间对齐规则

1. 所有训练源使用机器人 MCAP 的 `log_time`，存储为 int64 纳秒。机器人 receiver 与 MCAP writer 使用同一机器人时间域，但时间数值可能因排队不同。
2. 保存源 header/SDK 时间、receiver 时间和 log 时间；不得直接把主机时间重命名为 robot_receive。
3. MANUS/Sharpa 先在主机生成事件，再传入机器人。因此统一的是接收时间轴和录制事务，**不是硬件同时采样**；仍包含跨机与排队延迟。
4. 正式转换取所有必需流的公共可用区间，再建立 `t[k] = t0 + k × 50,000,000 ns`。公共 t0、有效区间和源索引必须保存，不能各分支各自生成不同起点。
5. 手部 state 选择最近反馈；action 选择不晚于 t[k] 的最近 accepted 目标。两者最大匹配误差均为 50 ms；未来命令禁止用于当前 action。
6. 缺流、重复时间戳、时间回退、过期或 SDK 关联冲突必须失败或明确排除，不能填零后标有效。
7. 录制开始前的历史目标仅能作为带原始身份与时间的初始条件。当前手部提取器提取包内实际 JointState，不自动把快照历史伪造成新 MCAP 样本。
8. replay 应根据同一个 episode 起点计算相对 ns 调度。两进程“差不多同时启动”不能保证整体同步。

## 9. 当前可执行的 Raw 校验与手部 HDF5

以下在 av-pipeline 执行；`RAW` 必须是已复制完成、包含 capture_manifest.json 的具体录制目录。

```bash
cd /home/descfly/apex_n_sharpa/integrations/manus_sharpa
PY=/home/descfly/apex_n_sharpa/.venv/bin/python
RAW=/替换为实际完整Raw目录

# 只读校验：不接触设备，不启动录制。
"$PY" - "$RAW" <<'PY'
import json,sys
sys.path.insert(0,'conversion')
from capture_contract import verify_completion
m=verify_completion(sys.argv[1])
print(json.dumps({'capture_id':m['capture_id'],'state':m['state'],
                  'clock_domain':m['clock_domain']},ensure_ascii=False,indent=2))
PY

# 输出必须是本手部模块 data 下不存在的新文件。
"$PY" conversion/capture.py \
  --raw "$RAW" \
  --output "$PWD/data/任务名/录制名/hands.hdf5"
```

`--allow-diagnostic` 只用于隔离样本验证，禁止在正式采集操作中添加。提取过程中生成的 `.embedded_context.jsonl` 来自同包 MCAP，并不是需要人工提供的外部依赖。

当前 `hands.hdf5`：

```text
attrs: schema=manus_sharpa_hdf5_v1, capture_id, session_id,
       clock_domain=robot_receive, unit=rad, serials, complete, provenance
state/left/   state/right/   action/left/   action/right/
  timestamp_ns          int64[N]    MCAP log_time
  source_timestamp_ns   int64[N]    主机源事件时间
  receive_timestamp_ns  int64[N]    机器人receiver时间
  seq                   int64[N]    context关联序号
  position              float64[N,22]
  attrs.joint_names     JSON名称数组
```

每个流保留原频率，N 可以不同。附属 `hands.hdf5.commit.json` 保存 HDF5 哈希和 capture_id；`.capture.json` 保存 Raw 认证来源及 diagnostic 标志。手部 HDF5 不是完整机器人 HDF5。

## 10. 身体 HDF5 与转换的实际差距

原项目保留这三个入口：`read_raw_data`、`apex2hdf5`、`hdf52lerobot`。前者只导入已有录制，不会开始录制。

原 Raw 导入布局是 `data/raw_data/apex_data/<task>/<episode>/payload/`；payload 保留原目录全部文件，外层 manifest 是传输校验。后续验证 capture completion 应指向 payload。

原身体 HDF5 的主要结构：

- `observations/left`、`observations/right`、`observations/active_camera`：实测 EEF，含 timestamp_ns 和 eef。
- `actions/left`、`actions/right`、`actions/active_camera`：目标 EEF。
- `observations/body_joints/value[N,5]`：五个原生身体/头部关节。
- `actions/body_controller/value[N,3]` 与 `actions/head_controller/value[N,2]`：控制器目标。
- `observations/chassis`、`actions/chassis`：原底盘状态与动作。
- `images/.../jpeg`：变长 JPEG 字节，伴随真实时间戳。
- `time/sample_timestamp_ns`：原身体公共采样网格。
- `raw_files/000000...`：嵌入原 Raw 字节，relative_path 属性保留来源路径。
- 各流还保留 source/receive 时间、source_topic、unit、source_role、source_stage、coordinate_frame 等属性。

**当前默认脚本不能直接完成统一 LeRobot76：**

1. `hdf52lerobot/main.py` 只接受 `apex_sharpa_78`、`apex_nonhand`，主维数分别 78、27；旧手部 derive 也强制校验 78。
2. 旧手部槽位为左 `[32:54]`、右 `[55:77]`；新格式右手必须 `[54:76]`。不能只改 shape 或删 metadata 字段。
3. 身体 HDF5 的 capture_id 当前读取 `episode_metadata.json`；新录制器写 capture_manifest/capture_time。需要补全认证 capture_id 的传递，不能把空 ID 视为关联成功。
4. 更正（2026-10-28复核）：有效配置在 `/mnt/apex-data/apex_n_sharpa/data/config`，已包含完整IK、full_joints及实际 `/recording/chassis/manual_move_cmd`；代码目录中的data/config是旧副本，不能据此判断有效配置缺字段。仍需将新的手部输入和76维契约接入这套已有身体配置。
5. 新手部 HDF5 应与身体使用相同 capture_id、同一认证 Raw 和同一 20 Hz 网格，再合并写出 76 维。
6. metadata、有效性、统计、特征名、各辅助列、原频率 replay 数据和完成哈希都需一起更新。

本文提供目标契约和已有可执行的手部提取命令，不提供一个会误生成旧格式的“LeRobot76 一键命令”。这一步是后续转换器实现与验收范围。

## 11. 76 维目标数据契约

零起始、半开区间：

| 范围 | 维数 | 内容 | 单位/来源 |
|---|---:|---|---|
| 0:9 | 9 | 左臂 EEF：XYZ + rot6d | XYZ:m，旋转无量纲；state实测、action目标 |
| 9:18 | 9 | 右臂 EEF：XYZ + rot6d | 同上 |
| 18:27 | 9 | 头部 EEF：XYZ + rot6d | state经实测关节FK到eyes_link，action头部目标 |
| 27 | 1 | waist_joint_1 | m；state body_joints[0]，action body_controller[0] |
| 28 | 1 | waist_joint_2 | rad；索引1 |
| 29 | 1 | waist_joint_3 | rad；索引2 |
| 30 | 1 | Head_J1_Joint | rad；state body_joints[3]，action head_controller[0] |
| 31 | 1 | Head_J2_Joint | rad；state body_joints[4]，action head_controller[1] |
| 32:54 | 22 | 左手绝对关节角 | rad；真实反馈 / SDK成功目标 |
| 54:76 | 22 | 右手绝对关节角 | rad；真实反馈 / SDK成功目标 |

`observation.state`、`action` 为 float32[76]；`validity` 为 uint8[76] 的结构有效性。真正齐备且逐帧核验通过后才可全部置1。身体5维是真实数据，不能沿用旧样例无效mask；夹爪两维彻底删除。

EEF 位姿存储在 base_link。已核对原项目 convert_pose：rot6d 是旋转矩阵前两列按列拼接，即 `[R00,R10,R20,R01,R11,R21]`。76维实现必须保持这一约定并做往返测试。不同坐标系的 action 必须经过明确变换。

建议沿用现有 LeRobot v2.1 目录组织：

```text
data/chunk-000/episode_000000.parquet
meta/info.json
meta/tasks.jsonl
meta/episodes.jsonl
meta/stats.json
meta/episodes_stats.jsonl
meta/source_timestamps.json
meta/completion.json
```

保留 `timestamp`（相对秒）、`frame_index`、`episode_index`、`index`、`task_index`；准确时间以 sidecar 的机器人 int64 ns 为准。保留现有图像列及其真实编码、尺寸，不凭旧示例强行改成三路 PNG。保留原频率 IK、底盘、身体和手部目标及来源信息；76维本身不包含底盘速度，不能仅靠它恢复整体 replay。

## 12. replay 应如何实现

当前独立手部 replay 和机器人原生 replay 各自存在；本轮没有实现联合执行器，也没有运行实体回放。

整体 replay 的实现需满足：

1. 验证认证 Raw、76维数据、各原频率计划的 capture_id、时钟、哈希、序列号和关节名称。
2. 关闭实时遥操控制源，先读取实际初态并检查偏差；不通过重命名topic或重启进程绕过控制权检查。
3. 身体 IK/底盘与双手执行器分别预加载并报告 READY，再由共同协调器选择同一未来单调时钟起点。
4. 使用同一 origin_ns 的原频率相对偏移；跨机执行需先做起点握手与时钟误差估计，不能直接比较两台机子的单调时间。
5. 一侧故障时通知所有执行器，按照各自已验证的停止接口终止；停止、失联、迟到和完成要有整体结果。
6. 先用假设备验证同步调度，再由现场人员核验初态、停止和小幅动作。

原始关节/IK/底盘数据的保留是实现整体 replay 的前提；同步录制本身不等于已经实现同步运动。

## 13. 验证证据、备份与下一次现场检查

本轮结果详见随附 `同步数采续接验收.json`。

- 单元测试：7/7，包含重复开始、完整性、诊断拒绝、来源校验、停止标记故障、停止RPC故障、写盘故障与相机中途断帧检查。
- ROS 原生 writer 测试：domain199、localhost；360条手部标准消息，context 同包、统一开始停止、断流失败、过期拒绝全部通过。
- 完整数值profile另测16个必需topic，身体、EEF、IK、底盘、双手及context同包通过；该轮356条手部标准消息。
- 两轮测试均为模拟数据且禁用相机实际录制，不启动硬件；没有验证实体身体、相机或手部运动效果。
- ms-thor 本轮第一份修复前备份：`/home/user/apex_record_sync/backups/resume-1793167420695563264/`。
- 第二次补充相机审计前备份：`/home/user/apex_record_sync/backups/resume-1793167499205926180/`。
- 原安装备份与哈希：`/home/user/apex_record_sync/installation.json`；本轮代码哈希：`resume-deployment.json`。
- 原 launch/YAML 回滚工具：`install.py --rollback`，会核对安装后哈希并恢复原录制配置；不会重启服务。恢复前应确保没有活动录制，并先核对是否存在后续独立修改。

下一次现场操作应先按正常流程启动身体与手部，确认持续数据就绪，再用原按钮录制一条短样本。停止后核对 manifest 中身体、双手、context、相机的数量与覆盖，运行默认完整性校验，再检查50ms转换匹配误差。未通过之前不要把该样本标为正式训练数据。
