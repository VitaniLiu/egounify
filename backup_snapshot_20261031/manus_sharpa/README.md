> 2026-10-28 同步录制更新：Apex 原 Gento Record/Stop 已接入统一录制协调器；认证 Raw 包内保存手部 context，新增 `conversion/capture.py`，无需人工寻找独立 JSONL。详细格式、操作与验收见 `/home/descfly/apex_n_sharpa/docs/Apex同步数采与76维转换操作说明.md`。下文旧 `hands_data.py derive` 仍为78维独立派生，不能作为76维导出器；完整76维转换和联合replay尚未实现。

# 天机 MANUS / Sharpa 独立模块

本目录只提供手部启动、记录、HDF5、LeRobot手部槽位派生和独立手部replay。
现有机器人、头部、双臂、底盘、相机、公共转换与replay文件不修改。

## 部署位置

- 项目入口：`/home/descfly/apex_n_sharpa/integrations/manus_sharpa`
- 实际目录：`/mnt/apex-data/apex_n_sharpa/manus_sharpa`（外置盘）
- 机器人仅新增：`/home/user/manus_sharpa/robot_receiver.py`
- MANUS权限：`/etc/udev/rules.d/99-manus-sensor-dongle.rules`
- 左手 `CE50903ECE53`；右手 `CC50933FCC53`。
- 独立ROS域198、本机通信；机器人记录桥通过SSH进入domain29，避免改变原DDS/ROS配置。

使用本目录 `pixi`，锁定环境在 `portable_deps/litchi_hardware/.pixi`。
V4是Python3.10 sidecar；ROS环境由原pixi.lock锁定。不要移动编译环境；移动时重新setup。

## 已安装与运行方式

```bash
cd /home/descfly/apex_n_sharpa/integrations/manus_sharpa
./scripts/start_hands.sh                    # 默认只打印计划，不连接设备
./scripts/start_hands.sh --fake --duration 4 # MANUS及Sharpa全部假设备；不连接机器人
./scripts/start_hands.sh --execute          # 实体：连接并使能MANUS/双手，开始跟随
```

机器人仍通过原有入口启动；本模块不启动、重启任何机器人服务。
实体入口会占用双手并下发目标。先核对佩戴者MANUS标定和左右手绑定，确保其他手部程序退出。
USB规则安装后需要拔插MANUS接收器使权限生效；安装脚本不全局触发USB设备。
启动先等待双侧连续有限映射目标，再连接Sharpa；数值库限制为单线程，停止阈值不变。
Ctrl+C依次停生产者、等待命令watchdog、退出驱动调用SDK stop/disconnect。
命令超时0.2s、源消息最大年龄0.25s、反馈过期0.5s、记录ACK超时2s。
watchdog停止后不自动恢复，需退出并重新启动；启动不提供自动回零/起点过渡。
SDK停止为set_enable_state(False)，不能当作已验证的实体硬件急停。

## 动作与时间规定

- 手部输出每侧22维绝对关节角rad，名称顺序对应 `reference/schema78.json`。
- MANUS映射结果是request；只有SDK成功回执里的最终裁剪目标作为accepted action。
- state只取Sharpa真实反馈；SDK调用成功不代表物理到位。
- 主机bridge读取源header，记录请求、SDK结果、序列号、会话及源时间。
- 机器人receiver给每个事件赋本机ROS时间，保存context JSONL，并发布已有四个手部录制topic。
- `/hand_left/joint_commands` / `/hand_right/joint_commands` 是SDK成功提交目标的记录副本；实体驱动只订阅本机域198的 `/sharpa/<side>/request`。
- 反馈发布至 `/hand_left/joint_states` / `/hand_right/joint_states`。
- 最终训练时间使用机器人原MCAP **log_time**（robot_receive），不是主机时间，也不是MANUS设备时钟。
- 原header、机器人receiver时间与MCAP时间分开保留；receiver时间不假冒MCAP接收时间。
- 单一FIFO、严格事件序号、队列满/积压/ACK丢失拒绝继续；跨机仍含网络时延，不宣称硬件同步。
- 主机源事件年龄最大0.5s；MCAP与receiver差异最大0.5s；20Hz训练匹配误差最大50ms。

机器人现有录制入口须另外正常录制上述topic。Sharpa启动不自动开始/结束机器人episode。
手部receiver日志在 `/home/user/manus_sharpa/recordings/<session_id>.jsonl`。
先正常停止手部会话再获取完整日志：

```bash
python3 scripts/fetch_hand_recording.py <32位session_id>
```

## Raw -> HDF5 -> LeRobot 手部派生

使用项目已有转换Python，只读导入依赖，不修改该环境：

```bash
PY=/home/descfly/apex_n_sharpa/.venv/bin/python
$PY conversion/hands_data.py extract \
  --mcap /path/to/finished/robot_episode.mcap \
  --journal "$PWD/data/journals/<session_id>.jsonl" \
  --capture-id <本次录制唯一ID> --output "$PWD/data/<任务>/hands.hdf5"

$PY conversion/hands_data.py derive \
  --source /path/to/existing/lerobot/episode_directory \
  --hdf5 "$PWD/data/<任务>/hands.hdf5" --capture-id <同一录制唯一ID> \
  --output "$PWD/data/<任务>/derived_episode"
```

extract验证MCAP消息的session/seq、原receiver日志、设备序列号、SDK结果和逐值一致性。
有多个分卷时重复 `--mcap`。不混合不同session；源消息重复或时间回退报错。
HDF5保存state/action原频率、源header时间、receiver时间、MCAP log时间、seq、来源哈希。
capture-id必须由操作者给同一录制一致命名；不能把不同动作仅按episode编号拼接。

derive校验源completion和手部HDF5的hash，读取源meta/source_timestamps.json的原机器人时间网格。
action因果保持此前最近目标，state取最近反馈；任一行超过50ms即拒绝，不移位、不插值、不删原行。
仅替换state/action的 `[32:54]`、`[55:77]` 和对应结构validity、手部mask/统计。
其他列（包括JPEG字节、索引、时间）及其他34维逐值验证不变。
`validity`为结构可用性，不表示置信度或实时质量；匹配质量记录在hands/alignment_report.json。
输出必须在本模块data下、新目录，原数据不覆盖；失败保留.partial供检查，不静默重试覆盖。
原频率手部回放计划存 `hands/replay.json`，并有独立hash与完整completion。

附件的11列和非手部坐标/分辨率若与源数据不同，本模块不会修改它们，也不会将完整数据标成已兼容在线模型。
现有 `egoav78_wire_compatible=false` 保留。只保证手部44维与附件一致。
已有无手部历史episode没有SDK回执和真实反馈，不会为其制造手部数据。

## 手部replay

```bash
./scripts/replay_hands.sh "$PWD/data/<任务>/derived_episode"            # 仅离线计划
./scripts/replay_hands.sh --execute "$PWD/data/<任务>/derived_episode"  # 实体手部回放
```

回放校验schema、hash、序列号、关节限位、起始姿态最大偏差0.1rad、反馈新鲜度和单发布者。
保留原频率相对时间，补充episode起点前最后一个目标；调度迟到超过50ms或原始目标缺口超过0.2s拒绝。
结束不附加运动/回零，supervisor执行停止清理。
**不控制机器人IK和底盘。现有公共replay没有在此修改，也未实现两进程共同起点握手，因此不宣称完整联合同步回放。**

## 验证与边界

```bash
/home/descfly/apex_n_sharpa/.venv/bin/python -m unittest discover -s tests -v
sha256sum -c reports/public_before.sha256
```

离线覆盖：附件顺序、因果匹配、MCAP/SDK证据关联、HDF5、派生非手部内容不变、capture/时间错误拒绝、replay hash。
ROS测试使用全假设备；V4 sidecar及MANUS/Sharpa共享库只加载不连接硬件。
真实MANUS双侧帧已读取；随后实体遥操尝试中，两只Sharpa均连接并收到过SDK成功目标，
但MANUS输入间断且SDK报告quantum sensor错误-6，导致原0.2s watchdog停止。
该会话已退出并断开双手，不应将其视为持续遥操成功。尚未验收实体运动效果、正式episode转换及联合回放。
