# 统一 LeRobot Replay：离线检查与隔离模拟修改报告

日期：2026-10-28。

本轮已实现并部署 **LeRobot离线计划校验＋四进程隔离模拟**，验证身体、左手、右手、底盘的共同起点、原频率调度、故障传播和停止确认。196项测试及15个模拟场景通过预期断言。没有执行真实硬件回放；真实执行入口明确不可用。

## 一、修改范围

在av-pipeline项目`/home/descfly/apex_n_sharpa`新增以下文件：

| 文件 | 实现内容 |
|---|---|
| `replay_unified` | 统一离线/模拟命令入口，使用项目现有虚拟环境 |
| `src/unified_replay/contract.py` | LeRobot完整性、身份、78维布局、时间、原频率流、训练值与replay来源一致性检查；生成计划 |
| `src/unified_replay/simulator.py` | 四个spawn子进程、共同启动协议、单调时钟映射、心跳、独立看门狗及15种场景 |
| `src/unified_replay/isolation.py` | 主进程与子进程禁止网络连接/监听、ROS及硬件SDK导入、硬件设备文件和Raw/HDF5读取 |
| `src/unified_replay/main.py` | 参数校验、报告落盘、场景集合运行、源数据未变验证 |
| `src/unified_replay/__init__.py` | 模块声明 |
| `tests/test_unified_replay.py` | 新增30项离线拒绝、因果选样、进程隔离和停止协议回归测试 |

本轮没有修改现有转换器、旧身体/底盘回放器或双手硬件驱动，没有修改机器人文件或重启服务。Raw、HDF5、LeRobot目录及旧episode02均保持原样；新报告写入独立的`data/replay_runs/`。

## 二、离线检查

入口只读取选定LeRobot episode目录，检查：

- completion文件清单及SHA256；拒绝数据文件增删、改写和符号链接。
- task定位的episode、内部编号、capture_id、COMPLETE状态、diagnostic标记及设备序列号一致。
- observation.state、action、validity始终78维，身体和双手名称顺序正确，54/77两个夹爪槽位保持零值及无效mask。
- 20Hz训练时间戳为整数纳秒；原频率数据递增、有效窗口覆盖、初始样本索引及相对偏移正确。
- 完整11组IK姿态、availability和19维seed；旋转列正交、数值有限。
- 双手22关节名称、SDK成功提交语义、序列号、session、seq、安装版Sharpa关节范围；身体与底盘流的单位及来源topic。
- 底盘速度边界；各流断档不能超过回放契约要求。
- 20Hz身体/双手值逐项对应replay原频率流的来源索引和时间；action不得取未来值，匹配偏差不超过50ms。
- `unified_replay.json`与`source_timestamps.json`重复保存的native replay数据必须一致。
- 生成计划后及运行后重新核对源episode完整性。

关节范围快照来自已安装的`integrations/manus_sharpa/portable_deps/litchi_hardware/src/litchi_hardware/hardware/sharpa/schema.py`，通过读取源码获取，没有导入SDK。该检查不等于碰撞、可达性或真实设备验收。

## 三、共同时间轴及执行语义

本轮计划固定为`backend=mock_only`、`hardware_execution_allowed=false`。

- 身体通道装载原生完整IK输入；腰/头控制器目标及19关节反馈保留用于检查，不额外向同一个身体控制通道发送第二套命令。
- 左右手装载SDK成功提交的原频率22关节目标；底盘装载原频率速度命令。
- 执行范围为LeRobot已导出的共同`origin_ns..end_ns`窗口。本轮样本为1.45秒；并非自动播放Raw全部首尾历史。
- 每个通道在共同起点只恢复一次“起点之前最近的有效目标”，保留其原始来源时间与索引。负偏移历史不集中补发。
- 起点后的事件保留原始偏移和频率，既不统一降采样到20Hz，也不修改源数据。
- 模拟进程时钟分别注入0、+35.45秒、−12.345秒、+7秒偏移及轻微速率偏差；多次往返探测估计时钟映射和误差界限，不依赖Unix系统时间相等。
- 四端完成BOOT与初态检查后，执行ARM/ARMED、COMMIT/COMMITTED共同启动协议。未准备齐全或计划哈希不一致时不发送任何轨迹事件。
- 执行期间持续检查时钟映射、反馈新鲜度和心跳。超期事件直接触发停止，不突发补发积压命令。

所用阈值是**协议模拟的验收参数**，不是对真实硬件安全性的认证：心跳周期20ms，本地心跳超时200ms，模拟反馈超时100ms，调度迟到上限50ms，时钟映射不确定度上限5ms，故障触发后的停止确认上限400ms。真实控制器接入后仍需独立校验这些值。

## 四、停止协议与修复

任一通道报告错误，主协调器向其他通道发出STOP。每个子进程独立保留看门狗，因此主协调器停止心跳时不依赖中央STOP消息也能停止模拟输出。

模拟停止动作：身体及双手disable，底盘zero_velocity；分别返回STOPPED及日志。报告只有收到所有通道的模拟停止回执才标记`mock_stop_confirmed=true`。设备动态未建模，`physical_stop_verified`始终为false。

第一轮故障测试发现发送管道关闭与接收回执之间存在竞态：对子进程再次写入失败时，主进程可能提前停止读取已在管道中的STOPPED。已分离写端关闭和读端关闭状态，并通过子进程故障回归验证。

进程退出不等于停止已确认。完整链路隔离、进程异常退出、停止回执丢失均保留`STOP_UNCONFIRMED`，不得把实验按预期检出故障解释为“全部设备成功停止”。

## 五、验收结果

测试素材为前一轮诊断`episode_00000042`，capture_id为`25949b07dfbc4dda941d3c47547a1403`。没有使用或改写正式编号3。

- **196项pytest通过**，其中本轮新增30项；覆盖原有转换/回放功能回归。
- **15个四进程模拟场景通过预期断言**，详见下表。
- 正常场景发送294条模拟事件：身体74、底盘74、左手73、右手73。
- payload哈希、来源索引及事件顺序一致；本地停止后发送事件数为0。
- 正常场景四端启动时间跨度约**0.553ms**；最大本地调度迟到约**0.869ms**；相对于共同主时间轴的最大事件时间误差约**1.025ms**。
- 主协调器失联场景，各端依靠本地看门狗停止，测得从注入失联到最后一个模拟停止约**186.9ms**。

以上时延是本机子进程及管道测试的观测结果，不能用于承诺真实跨主机网络、ROS调度或执行器时延。

| 场景 | 预期及实际结果 |
|---|---|
| 正常调度 | COMPLETED，全部事件和模拟停止回执完整 |
| 左手SDK拒绝 | 联动停止，四端模拟停止确认 |
| 右手SDK拒绝 | 联动停止，四端模拟停止确认 |
| 身体反馈中断 | 联动停止，四端模拟停止确认 |
| 底盘反馈中断 | 联动停止，四端模拟停止确认 |
| 左手心跳中断 | 左手本地超时，其他通道联动停止 |
| 左手链路完全隔离 | 其余通道停止；左手停止回执不可得，STOP_UNCONFIRMED |
| 主协调器心跳丢失 | 四端本地看门狗停止，不依赖中央STOP |
| 右手调度阻塞 | 迟到事件未发送，触发联动停止 |
| 左手时钟跳变 | 检出映射变化并联动停止 |
| 准备阶段计划哈希不一致 | 启动前中止，轨迹事件0条 |
| 右手未就绪 | 启动前中止，轨迹事件0条 |
| 初始手姿不匹配 | 启动前中止，轨迹事件0条，不自动移动 |
| 身体进程异常退出 | 其他三端停止；身体STOP_UNCONFIRMED |
| 底盘停止回执丢失 | 保留缺失回执，STOP_UNCONFIRMED |

## 六、使用方法

在av-pipeline执行，只做离线检查：

```bash
cd /home/descfly/apex_n_sharpa
./replay_unified --task unified78_validation_1793171077641728617 --episode 42 \
  --plan-only --allow-diagnostic
```

运行正常隔离模拟：

```bash
./replay_unified --task unified78_validation_1793171077641728617 --episode 42 \
  --simulate --allow-diagnostic
```

运行全部故障场景：

```bash
./replay_unified --task unified78_validation_1793171077641728617 --episode 42 \
  --simulate --scenario all --allow-diagnostic
```

`--allow-diagnostic`只为上述诊断样本显式开启。正式episode的离线检查不需要该选项。`--execute`会在输入读取、报告创建和执行之前明确报错退出；当前没有真实硬件backend，也没有把该选项转发到旧硬件回放脚本。

每次运行创建独立目录：

```text
data/replay_runs/<task>/episode_XXXXXXXX/offline-<run-id>/
├── plan.json
├── report.json
├── normal.json                    运行相应场景时生成
└── <故障场景>.json                 逐端计划/实际时间、来源哈希、停止原因与回执
```

本轮完整场景报告路径：
`/mnt/apex-data/apex_n_sharpa/data/replay_runs/unified78_validation_1793171077641728617/episode_00000042/offline-1793172446921105134-7b2767db/`。

源episode校验前后未改变。运行报告的“experiment_passed”表示该场景符合预期；要判断实际运行是否完成，应同时查看status、mock_stop_confirmed及missing_stop_ack。

## 七、部署、回退及后续边界

部署清单位于：
`/home/descfly/apex_n_sharpa/deployment/unified-replay-offline-1793172524250934622/manifest.json`。

部署前核对暂存文件SHA256，并确认目标文件均不存在；本轮只新增7个文件，无旧源码覆盖。清单包含各文件哈希及回退说明。回退仅移除清单中的新增入口、模块和测试，保留数据及验收报告；无需回退机器人服务。

**本轮完成的是离线与协议模拟阶段。** 未连接ROS、原生QP控制器、Sharpa SDK或底盘接口，未测物理轨迹、碰撞、真实跨主机同步及硬件停止。本机进程模拟中的初始反馈和反馈新鲜度由mock生成，不是现场预检查。下一阶段仍需实现真实传输与执行适配、实际反馈和停止回执，再先做控制器隔离集成；真实硬件短片段回放不在本次执行范围内。

机器可读验收与部署信息见[统一Replay离线与隔离模拟验收.json](统一Replay离线与隔离模拟验收.json)。
