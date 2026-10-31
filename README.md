# EgoUnify：Sharpa 灵巧手适配代码备份

本分支是 2026-10-31 对真机数采、转换与回放相关代码的备份。以下说明本次备份的目录结构和范围；原项目使用说明保留在后半部分。

## 本次备份的文件结构

分支：`adapt/sharpa-hands`。本分支保存灵巧手适配相关的真机采集、数据转换、回放代码，以及 2026-10-31 可取得的代码备份。

```text
仓库根目录/
├── src/                         当前正式数据导入、转换、client及replay实现
├── recording/                   同步录制与完整性契约
├── config/                      首次入库的配置副本
├── tests/                       正式代码测试
├── docs/                        使用说明、契约和验收记录
├── apex2hdf5 / hdf52lerobot      数据转换入口
├── read_raw_data                Raw导入入口
├── convert_unified              统一转换入口
├── client_lerobot               身体和底盘client入口
├── replay_lerobot / replay_unified
└── backup_snapshot_20261031/     只作备份，不作为新的运行入口
    ├── manus_sharpa/            灵巧手模块实际文件副本，解除了原入口符号链接依赖
    │   ├── scripts/            手部启动、环境准备、回放等脚本
    │   ├── recording/          手部桥接与机器人接收器
    │   ├── conversion/         手部提取和转换
    │   ├── config/             手部设备配置
    │   ├── portable_deps/      HAL、ROS源码及保留下来的SDK文件
    │   ├── tests/              手部测试
    │   └── backups/            原目录中保留的代码历史副本
    ├── effective_config/        外置盘当前有效配置及任务固定配置
    ├── calibration/             标定脚本、模型和负载资料
    ├── deployment/              部署脚本、历史源码副本及报告
    ├── controller_stage/        未合入的控制器集成实现
    ├── replay_stage/            统一replay暂存实现
    ├── conversion_stage/        统一转换暂存实现
    ├── chassis_test/            底盘专项诊断、回放和测试脚本
    ├── robot_local_snapshot/
    │   ├── module/             本地保存的录制协调器、审计和安装代码
    │   └── robot_sources/      本地保存的机器人启动、launch及配置快照
    └── manifest.json           源路径、备份清单、SHA256及排除记录
```

机器人 `6.6.7.100` 本轮不可达，`robot_local_snapshot` 来自此前保存在维护电脑的副本，不代表机器人当前版本。主机正式代码和各stage分别保留，stage不代表部署完成或验收通过。

备份不包含采集数据、运行环境、缓存、构建输出、运行日志、压缩归档和大于20MB的单文件；逐文件排除见manifest。第三方依赖的原始许可证随文件保留。本仓库是代码与配置备份，不是完整系统镜像。

原有 `integrations/manus_sharpa` 运行链接未改动，其目标内容已复制到备份目录；`test/` 的专项代码也已复制，运行输出不入库。部署路径与设备信息按备份原样保留。

---

## 原项目使用说明

# APEX + Sharpa 分步数据转换

本项目使用独立 `.venv`。三个步骤各自位于 `src` 的独立目录，源码互不 import、不调用其他步骤，只通过磁盘数据衔接。第三个命令内部完成转换与清洗。

**当前状态：离线实现与合成数据测试；未连接机器人，未导出任何宣称来自真实机器人+Sharpa的训练数据。** Sharpa 尚未采集，时钟标定、VR坐标变换、头部真实EEF与四宫格相机区域尚未确认。默认配置故意不会在这些条件不满足时生成可训练的完整78维数据。

## 路径统一：代码在主机，数据在WD硬盘

生产运行仅在av-pipeline上执行。代码与独立虚拟环境位于 `/home/descfly/apex_n_sharpa`，所有运行时配置、参考规范、原始副本、转换结果、锁和断点记录位于外置WD硬盘：

```text
/media/descfly/My Passport/apex_n_sharpa/data/
├── config/{apex,lerobot}.json
├── references/
├── raw_data/
│   ├── apex_data/<task>/episode_00000000/{payload/,manifest.json}
│   └── sharpa_data/<task>/episode_00000000/{payload/,manifest.json}
├── hdf5/<task>/hdf5_00000000.hdf5
├── lerobot/<task>/episode_00000000/{data/,meta/}
└── test_sources/                  # 仅测试生成，用后清理
```

代码目录中的旧 `data` 不再由任何转换命令或测试读取/写入；保留其中旧文件，不自动删除。下文 `data/...` 均指WD硬盘上的上述目录。

三个命令启动时检查实际挂载和设备序列号 `WD-WX12A367D1A2`，断开硬盘、挂错盘、数据路径越界时直接失败，不会回退到系统盘。若以后将同一硬盘挂载到 `/mnt/apex-data`，统一设置 `APEX_STORAGE_MOUNT=/mnt/apex-data` 即可，数据仍位于其下 `apex_n_sharpa/data`；该环境变量不会自动挂载或格式化硬盘。

当前沿用已有exFAT，未格式化硬盘、未清理已有备份。代码中的单步读写逻辑彼此独立，无跨步骤import。

## 安装与三个命令

av-pipeline已建立Python 3.12独立环境。重新部署可执行：

```bash
cd /home/descfly/apex_n_sharpa
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

Python 3.9及以上；已在av-pipeline Linux / Python 3.12及WD exFAT上测试。文件锁使用 POSIX `fcntl`，支持 macOS/Linux，不直接支持 Windows。

按顺序**分别**执行，不提供一键串联命令：

```bash
./read_raw_data --task folding
./apex2hdf5 --task folding
./hdf52lerobot --task folding
```

1. 第一条命令默认通过SSH连接 `user@6.6.7.100`，从机器人 `/home/user/gento_replay_bags` 读取录制。源根目录应包含多个 episode 目录。原始 `my-bag-26-10-23-18-38-53` 名称按采集时间排序。其他命名必须在每个目录提供 `capture_time.json`，内容例如 `{"utc":"2026-10-23T10:38:53Z"}`；带时区的时间统一转UTC后排序；my-bag文件名默认解释为Asia/Shanghai，可用 `--source-timezone` 指定采集主机时区。不能用拷贝后的mtime冒充采集时间。一个调用处理一个任务，任务名仅允许字母、数字、下划线、连字符。
2. 根目录直属文件也保存在任务 `_collection/payload`，不静默丢弃。episode下所有文件按原目录结构完整复制至 `payload`，包括未知文件；原有robot/video等分类保持不变，MCAP混合topic不拆包。`manifest.json`记录每个文件的路径、大小和SHA256。符号链接和特殊文件会显式报错，需先实体化。
3. `--sharpa-source /路径` 可同时采集Sharpa；默认留空。APEX与Sharpa各自按时间排序，但**不能仅凭同序号认为是同一次录制**，必须提供共同 `capture_id`，并确保同任务两侧episode集合一一对应；转换会校验标识、共同时间范围和偏差。不应在缺失中间episode的情况下强行按编号配对。
4. `--ssh user@host` 可更改机器人地址，使用密钥/agent和已有known_hosts，不自动信任陌生主机、不保存密码。仅显式加 `--local-source --apex-source /本地源` 才切换为本地导入，用于离线测试。Sharpa源留空时不读取；指定 `--sharpa-source` 默认读取本地源，另加 `--sharpa-ssh user@host` 才连接独立Sharpa采集机。本次仅修改默认来源，未运行真实录制拷贝。
5. 只复制已停止录制的目录。拷贝时验证文件尺寸/mtime及目录成员不变；未完成目录不会发布。需要录制端完成标志来做到更强的一致性保证，本接口目前不会控制录制器。空目录也会保留，后续episode的文件遍历推迟到前一episode提交之后。

## 断点与事务

每一步均串行处理episode，显示tqdm进度；复制进度条显示字节数和B/s，逐episode输出完成提示。转换还显示消息读取进度。完成后原子重命名，完整的episode、HDF5或LeRobot目录才会出现在最终位置。

Ctrl-C、SIGTERM、普通异常：记录当前episode/文件和已复制字节数，删除当前episode临时目录或临时HDF5；下次从该episode重新处理，已完成episode跳过。复制步骤以**episode为恢复单位**，不进行半文件续传。完整目录含校验清单；后两步记录输入/配置摘要，禁止配置变更后悄悄复用旧结果。

SIGKILL、断电无法被Python捕获，因此不能承诺退出瞬间删除。下次同命令启动清除`.partial`，利用此前已落盘进度和完整结果恢复。每个步骤有独占锁，避免同一步两个进程同时写入。关键文件和提交父目录执行fsync；仍不能承诺存储硬件故障下绝对不丢数据。HDF5的同名 `.commit.json` 和LeRobot的 `meta/completion.json` 保存输出SHA256，恢复时检测成品损坏，不只比较输入摘要。

修改源数据/配置后，请使用新任务，或手工备份并移走对应旧输出后重新运行；程序不自动覆盖已完成结果。

## APEX → HDF5

`data/config/apex.json` 是基于本地既有采集审计构建的topic映射候选，并非本次联网验证结果。

- 左/右真实state：`/tj/info/eef_left`、`/tj/info/eef_right`。
- 左/右action：`/tj/control/target_poseL`、`/tj/control/target_poseR`，来源是VR控制器，绝不使用反馈或下一帧state伪造action。
- 头部action候选：`/tj/control/vr_body.head_pose`。
- 底盘state：`/recording/chassis/state.speed`；action：`/recording/gamepad/command.twist`；均提取vx、vy、wz。
- 视频：`/recorder/quad_tile/compressed_undistorted`，支持H.264或JPEG，H.264使用解码PTS保留每帧时间。

MCAP需要内嵌完整ROS2消息定义，支持自定义类型。当前不支持只有ROS2 sqlite3 `.db3` 的直接解码；这类数据会完整复制，但第二步提示缺少受支持输入，应先通过明确适配器导出标准流。一个episode不要同时混放重复MCAP和JSONL，存在MCAP时以MCAP为输入。

HDF5沿用样例的 `observations/<role>/eef`、`images/<role>/jpeg` 分组，并扩展为：

- `actions/<role>/eef`：独立遥操位姿；xyz米，四元数xyzw。
- 各分组 `timestamp_ns`：映射到统一时钟域的int64纳秒；另存 `source_timestamp_ns` 原设备时刻和 `receive_timestamp_ns` 接收时刻（JSONL适配输入没有独立接收时刻时保留适配时间）。
- `observations/chassis/value`、`actions/chassis/value`。
- `time/sample_timestamp_ns`：统一20Hz网格。
- `raw_files/*`：逐文件原始字节和relative_path，包含全部topic、侧车元数据和原视频，未被选入训练字段的数据也能恢复。
- 根属性 `complete`、`clock_domain`、`capture_id`、`metadata_json`、`provenance`。

**这是一份明确扩展的HDF5 schema v4，并非逐字段克隆示例。** 示例 `episode1.hdf5` 是另一套机械臂的677帧反馈记录，仅有observation，没有action；其monotonic时间和各臂独立坐标系不适用于直接拼接APEX。项目保存了结构摘要，没有把该文件转换为虚假的APEX数据。流保留各自采样率，第三步再对齐，原始异步测量不会在第二步被丢弃。

底盘header与机器人时钟已知不一致，默认用统一机器人MCAP接收时间 `clock=log` 对齐，并保存原包供审计。这只是接收时刻同步，不是硬件曝光/采集同步，也不能消除传输延迟。若切换header时间，必须在stream配置中声明 `source_clock_domain`；与统一时钟不同则必须提供 `clock_map`。程序不再将header时间直接重标为机器人时钟。

## 必须补齐的真实采集配置

1. **头部真实EEF**：新增 `observations/active_camera` 映射，来源须为真实末端反馈或使用正确URDF/FK/外参计算的头部位姿。不能把三维头关节角当作七维位姿。本项目不猜测机器人几何参数，也未内置该机器人的FK。
2. **VR到公共坐标系**：在 `lerobot.json.frame_transforms` 填写实际 `tracking_base_link` 到 `base_link` 的4×4变换；变换方向为 `p_base = T @ p_source`。仅支持已标定的静态刚体变换，若VR存在重锚定、尺度转换或动态变换，应先提供校正后的流；不得填单位阵绕过验证。state已在base_link时不变换。
3. **四宫格分区**：将 `images/quad` 映射替换为 `images/active_camera`、`images/left`、`images/right`，每条可以使用同一视频topic，但必须填写实际确认的 `crop:[x0,y0,x1,y1]` 和可选 `size:[640,480]`。四宫格中头部双目要明确选择哪一路；不会自动猜左右和相机位置。原四路视频仍完整保留在raw_files。
4. **Sharpa与手套适配**：设备格式尚未给出；已定义下面的显式JSON适配接口，不宣称能读取未知厂家文件。原始厂家文件应全部一起保留在payload。

未采集手时，可在 `allow_missing` 中显式加入 `state/left,state/right,action/left,action/right` 来导出无手数据；缺失维度填0且validity=0。类似地，头部允许项是 `observations/active_camera,actions/active_camera`。默认严格要求完整数据。共享validity要求state和action该分组同时存在；仅一侧存在时双方训练槽位均无效，但HDF5/原始文件仍保留原测量。

## Sharpa适配文件

存于 `data/raw_data/sharpa_data/<task>/episode_00000000/payload/sharpa.json`。推荐在采集导出端写入，再通过第一步完整复制。格式：

```json
{
  "capture_id": "本次共同采集的唯一标识",
  "streams": {
    "state/left": {
      "source_role": "state",
      "clock_domain": "robot_receive",
      "timestamp_ns": [1792750000000000000, 1792750000050000000],
      "unit": "rad",
      "joint_names": ["此处填写参考info.json中索引32:54的22个完整关节名"],
      "values": [["22个真实反馈数值"], ["22个真实反馈数值"]]
    }
  }
}
```

示意中的字符串占位符须替换成实际数据。APEX episode源目录必须提供 `episode_metadata.json`，内容为 `{"capture_id":"本次共同采集的唯一标识"}`；Sharpa JSON顶层须包含相同capture_id。标识须由采集组织端显式关联，不允许从编号推断。必须提供 `state/left,state/right,action/left,action/right` 四条流，每条N×22，各条流的 `source_role` 必须分别为 `state` / `action`。state来自Sharpa真机反馈，action来自手套控制目标；允许各流独立时间戳，关节名按名称重排，单位支持rad或deg。右手名对应参考规范55:77。需要手套到Sharpa关节的重定向/标定时，必须在适配器中完成，不直接将原始手套传感器编码当弧度。

不同设备时钟必须逐流提供 `clock_map`：

```json
{"target_domain":"robot_receive","source_origin_ns":1000000000,"target_origin_ns":1792750000000000000,"scale":1.0}
```

映射为 `target_origin + round((source-source_origin)*scale)`，使用整数原点与有理数比例计算，避免epoch纳秒精度丢失和int64中间减法溢出；拒绝非正/非有限比例、浮点时间戳、重复/回退及溢出。原点和漂移比例必须来自实际同步测量，不能从相近文件编号推断。没有映射则拒绝转换。

## 清洗与LeRobot输出

- state/图像选择最近时刻；action选择不晚于目标帧的最近时刻，避免未来action泄漏。
- 每条流与网格偏差，以及所有被选源样本之间总时间跨度，均不得超过 `max_skew_ms`（默认50ms）。保存对齐偏差统计和原始纳秒时间轴。
- 拒绝非递增/重复时间戳；过滤NaN/Inf、退化/明显非单位四元数、损坏图像、缺帧和超限不同步。
- 小幅四元数归一化后转换为旋转矩阵**前两列**，顺序与78维样例一致。
- 保留**最长连续有效段**，不将删帧两端强行拼接、压缩成虚假连续20Hz。保留帧数、丢弃帧数、原始起止时间写入cleaning_report。这个策略会舍弃其他有效短段；可审阅报告后调整阈值或将源episode分段重跑。
- 不按“动作没变化”删帧、不做任意离群值裁剪、不平滑真实动作、不将零速度静止段视作坏数据。

每对HDF5+Sharpa生成一个**独立单episode LeRobot v2.1数据集**：目录名保持源8位episode编号，内部按照给定规范使用6位 `episode_000000.parquet` 和局部episode_index=0。它不是把整任务合成一个多episode数据集；任务目录本身不能直接作为一个LeRobot数据集加载。源编号保存在meta中。

输出含样例的11个字段、JPEG bytes/path结构、Hugging Face图像schema元数据，以及 `observation.chassis`、`action.chassis` 两个额外三维字段。身体预留位27:32以及两个夹爪位保持0且validity=0，不偷偷塞底盘量。训练配置须显式使用底盘特征；使用78维action的模型不会自动学到底盘动作。

meta包括info、tasks、episodes、episodes_stats、stats、cleaning_report、source_timestamps、provenance、completion。source_timestamps包含每条流逐帧匹配的时间和原流索引，cleaning_report分别报告清洗前与保留段最大时间跨度。无效维度的零统计不是有效测量，训练时需使用validity掩码。没有声称通过完整LeRobot训练加载器测试；已按提供的v2.1样例检查Arrow类型、维数、图像结构和元数据。

## 测试

```bash
.venv/bin/python -m pytest tests -q
```

测试需要指定WD硬盘已经挂载，在硬盘data子目录中生成独立命名的合成数据并在测试后清理；所有复制测试显式使用 `--local-source`，默认机器人连接参数通过假Source对象验证，不连接机器人。exFAT不支持符号链接，因此符号链接逃逸测试会跳过，普通路径越界测试仍执行。覆盖三个真实CLI串行执行、state/action区分、78维布局、恢复跳过、时钟异常拒绝与清理、MCAP自定义消息、H.264时间戳、复制中断恢复、非法四元数、时间戳重复、连续段和路径隔离。尚未进行真实机器人/Sharpa整段联调和大规模内存/性能验证。解码后的episode流会驻留内存，长episode应分段采集。

## 离线审查修订（2026-10-24）

详细检查和测试范围见 [AUDIT.md](AUDIT.md)。新版本不自动复用旧版缺少commit/来源标注的转换结果；请保留原始副本，将旧HDF5/LeRobot输出备份到任务输出之外后重建。

`apex.json`的每条observations映射必须声明 `source_role: "state"`，每条actions映射声明 `source_role: "action"`，位姿单位必须显式为m。禁止将同一topic+字段同时映射为state和action；pose消息实际frame_id必须与配置frame一致。MCAP多个分卷按log_time合并，而非按文件名字典序串接。源时钟回退会报错，不再通过排序掩盖。

离线标准流 `payload/streams.jsonl` 的每行必须包含topic、整数timestamp_ns、clock_domain、source_role和value；pose另需coordinate_frame，图像可用jpeg_base64代替value。若不同于目标时钟域，须在对应stream配置提供clock_map。适配器负责准确填写这些语义信息，程序无法从数值本身证明它确实来自真机或手套。

第三步拒绝HDF5外部/软链接和外部存储，防止HDF5绕过data目录输入限制。所有虚拟环境、三命令、代码分离和统一data路径约定保持不变。

## 本次外置存储更新

已在WD上初始化config、references、raw_data/apex_data、raw_data/sharpa_data、hdf5、lerobot和test_sources。配置及参考规范已部署到硬盘；默认 `--config` 也指向硬盘，不能继续传主机项目旧data路径。本次没有格式化、没有复制真实录制或启动真实转换。

## ext4格式化准备（待主机sudo认证）

目标WD序列号已固定为 `WD-WX12A367D1A2`。`deployment/format-wd-ext4.sh` 是主机上已准备的管理员执行脚本：校验设备和配置备份、非强制卸载、格式化原唯一数据分区为ext4、挂载 `/mnt/apex-data`、通过UUID写入fstab、重建目录并还原config/references、执行离线测试。此脚本将清空WD上的其他现有文件；只备份了约26KB项目配置和参考规范，并未备份用户的大体积备份文件。尚未通过sudo执行，不能视为已格式化。

三个运行命令在未显式设置APEX_STORAGE_MOUNT时，优先使用已挂载的 `/mnt/apex-data`；否则尝试原 `/media/descfly/My Passport`，两种情况都严格校验WD序列号和挂载状态，不回退到系统盘。格式化成功后的数据根目录为 `/mnt/apex-data/apex_n_sharpa/data`。

## eyes_link 接入（2026-10-24）

上述格式化准备状态已更新：WD 已完成 ext4 格式化并挂载。运行数据根目录为 `/mnt/apex-data/apex_n_sharpa/data`。

`apex2hdf5` 的 pose 流可配置 `fk`：包含 data 内的 `model` 路径、模型 `sha256`、`root`、`target`。输入必须是带关节名的原始 JointState 实测 state，缺失关节、模型校验失败或不支持的模型表示会停止转换。使用独立、显式弧度制 MJCF，关节类型为 hinge/slide，body 旋转使用 quat。模型默认关节参数不应省略，必须使用已核验的模型。HDF5 保留原始采样时间和 FK 来源属性。

av-pipeline 的配置已将 `/tj/joint_states` 经 Gento_Sky 模型计算的 `eyes_link` 接入 `observations/active_camera`；`actions/active_camera` 仍来自 `/tj/control/vr_body`。LeRobot 配置使用已核实的 `tracking_base_link` 到 `base_link` 单位变换。`allow_missing` 保持空列表，不填充缺失 Sharpa。三集新 HDF5 已重建，旧版保存在 data/hdf5_archive；头部结果与独立计算记录核对一致。

相机修复脚本 `deployment/repair_robot_camera.sh` 只针对 ms-thor 的指定内核：备份并移除两个相机驱动的黑名单条目，加载驱动、重启 Argus 和相机服务。需在机器人上经 sudo 执行，不能仅凭脚本存在视为相机已恢复。

## 不含夹爪/灵巧手的实测数据导出

`profile: apex_nonhand` 输出 27 维双臂/eyes_link 位姿，而非带缺失占位的 78 维。非手部模式禁止 allow_missing；所有位姿必须存在，validity 的 27 个维度全部有效。额外流由 `extra_streams` 显式声明来源、维数、名称和来源阶段，使用相同时间容差，控制器命令只做因果采样。命名 JointState 可用 joint_names 选取反馈，固定数组可用 indices 选取真实使用的槽位。

已部署配置 `data/config/apex_nonhand.json`、`data/config/lerobot_nonhand.json`。在 av-pipeline 项目根目录运行：

```bash
./apex2hdf5 --task robot_import_20261024 --config /mnt/apex-data/apex_n_sharpa/data/config/apex_nonhand.json
./hdf52lerobot --task robot_import_20261024 --config /mnt/apex-data/apex_n_sharpa/data/config/lerobot_nonhand.json
```

状态/动作位姿使用 base_link，旋转表示是矩阵前两列。独立保存底盘速度、3腰部+2头部实测关节、原生身体/头部控制器命令以及 VR 躯干/骨盆目标；原生控制器槽位不冒充文档中尚未定义的5个身体模型维度。右头为 head_color，左腕和右腕作为另外两路图像。保留源纳秒时间戳及来源阶段，不复制夹爪/灵巧手到训练字段。

交接文档描述的是推理接口：单头部RGB、78维输入、32步动作、chunk首帧相机系。该协议不能自动推导训练数据的坐标预处理。当前导出明确标记 egoav78_wire_compatible=false；没有右头相机光学外参、身体5维定义及对应client处理代码时，不生成伪装成该接口的输出，也不启动文档中的真机推理命令。

## EgoAV 78维映射（字段说明文档修订）

78维采用零起始索引：左臂0–8、右臂9–17、eyes_link头部18–26、身体27–31、左手32–53、左夹爪54、右手55–76、右夹爪77。仅 `egoav_task: drink` 启用27腰角；28–31永远未启用。G2使用双手、3B使用夹爪，协议未启用槽位为0且validity=false；启用字段缺失一律报错，禁止以allow_missing伪造手部数据。Skye三腰两头不直接填入27–31。

`scalar_streams` 明确指定腰角/夹爪的state和action各自HDF5 path及index；读取source_role并校验单位，腰角输出rad、夹爪输出mm。混合单位向量按选定通道解释，例如m,rad,rad。现有HDF5的native_driver_position仍不能直接推定为rad；需通过经过核实的上游单位配置重新生成，禁止只改标签掩盖来源。

`output_frame: chunk_first_frame_camera` 需要显式 `chunk_size`（1–32）。清洗后按固定非重叠块划分，用每块第一帧**实测**头部EEF作为冻结坐标系，将state/action的三个EEF均转换到该系；标量关节角不变。`meta/chunk_frames.json`记录每块边界与回到base_link的4×4变换。action仍是绝对目标，不再叠加server已还原的相对量。训练时不能跨块直接拼接动作窗口；若使用任意起点的重叠窗口，必须先恢复到base_link，再按新窗口首帧重新表达。附加pose流仍在base_link，元数据显式区分。

默认base_link存储继续兼容已有数据；新的配置模板data/config/lerobot_egoav78.json启用32帧chunk及eyes_link约定，缺失手部数据时会停止，不覆盖已有非手部输出。字段/坐标满足文档不等于完整在线G2兼容：Skye机器人形态、任务及相机图像约定仍须由模型消费方确认，因此egoav78_wire_compatible保持false。不启动推理。

## 显式未采集手部占位（用户授权）

78维模式可通过 `uncollected_fields: ["left_hand", "right_hand", "left_gripper", "right_gripper"]` 声明尚未采集的字段。这些槽位为0、validity/state_mask/action_mask为false，info.json和cleaning_report.json记录not_collected与is_measurement=false；missing_data_mask区别未采集字段与协议未启用的身体槽位。有效数据数量记录在valid_sample_counts.json，训练必须排除无效维度的归一化统计和loss。不得把这些0解释为测量值。若声明缺失的手部已有数据则报错，不静默丢弃。EEF和drink腰角仍不允许缺失占位。
