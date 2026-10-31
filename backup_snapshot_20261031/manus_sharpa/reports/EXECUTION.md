# Sharpa手部迁移执行报告

日期：2026-10-27（Asia/Shanghai）。范围：仅手部模块；没有启动实体MANUS、Sharpa或机器人运动。

## 部署位置与来源

- 源：`real:/home/descfly/code/egoav/sharpa/`，Git提交 `b252fe8d78acc13d98d504d3a7c5fad653b8b349`，复制包含源工作区内容。
- 项目入口：`av-pipeline:/home/descfly/apex_n_sharpa/integrations/manus_sharpa/`。
- 实际文件：`av-pipeline:/mnt/apex-data/apex_n_sharpa/manus_sharpa/`，项目入口为链接。系统盘剩余约4.6GB，因此环境和缓存均放外置盘。
- 原依赖文件复制711个，来源及逐文件SHA256见 `reports/source_manifest.json`。
- 部署清单：`migration_manifest.json`。完整使用说明：`README.md`。
- 复制独立Pixi可执行文件，按原pixi.lock安装ros2、retarget-v4两套环境，并构建manus_driver、sharpa_driver、manus_sharpa_teleop、sharpa_wave_description四个ROS包。
- 未部署FR3/GELLO运行链路。HAL保留原共享包依赖，但不构建或运行其他设备ROS驱动。

## 新增手部文件及作用

以下路径相对于实际部署目录：

| 文件 | 实现内容 |
|---|---|
| `config/hands.json` | 左手CE50903ECE53、右手CC50933FCC53；手部域、时钟域、目标机、频率与超时配置 |
| `hands_common.py` | 附件规定的22关节顺序、具名重排、有限值检查、因果/最近时间匹配、文件hash |
| `runtime.py` | 独立手部进程管理与互斥锁；默认plan-only，显式execute才连接真机；源新鲜度、单发布者、watchdog停止锁存、子进程异常与退出清理 |
| `scripts/setup_hands.sh` | 在外置盘独立重建环境及四个手部ROS包，不修改公共Python环境，不启动设备 |
| `scripts/start_hands.sh` | 本机域198运行手部链路；隔离ROS/Python环境；支持全假设备测试 |
| `scripts/replay_hands.sh` | 默认离线检查；显式execute通过同一互斥入口启动手部回放 |
| `recording/bridge.py` | 订阅手部请求、SDK回执和反馈，通过有界队列及SSH转发；记录source header、序列号、会话和事件序号；拒绝积压与断链 |
| `recording/robot_receiver.py` | 机器人侧手部记录接收端，保存独立JSONL并发布原有四个手部录制topic；不发布IK/底盘控制命令 |
| `scripts/fetch_hand_recording.py` | 获取已经停止会话的完整机器人手部JSONL，检查录制锁和复制前后大小 |
| `conversion/hands_data.py` | 手部MCAP+SDK日志证据关联→独立HDF5→新目录LeRobot手部派生；不修改原转换器或原数据 |
| `replay.py` | 校验hash/schema/序列号/限位/起点；按原频率相对时间发送双手目标；不控制机器人其他部位 |
| `tests/test_hands.py` | 6项离线测试，覆盖schema、匹配、MCAP证据、HDF5、派生完整性、错误拒绝及replay校验 |
| `tests/fake_source_loss.py` | 隔离域假设备断流测试，验证native watchdog停止及进程失败退出 |
| `reference/info.json`、`schema78.json`、`training_schema.json` | 从用户ZIP提取的只读格式参考，不复制600MB图像Parquet |
| `README.md` | 环境、启动、记录、转换、回放、时间语义与已知边界 |
| `migration_manifest.json`、`reports/*` | 来源hash、部署hash、公共文件基线及验证报告 |

本地 `.hands_migration/` 为传输中转与手部适配源码副本，不参与机器人运行。

## 修改的复制文件

唯一修改的原依赖源码为：

`portable_deps/litchi_hardware/ros2/src/sharpa_hand/sharpa_driver/sharpa_driver/node.py`

将SDK结果发布者QoS从 `BEST_EFFORT/depth=100` 改为 `RELIABLE/depth=1000`，使作为action标签证据的SDK回执在本机可靠传输。没有修改原映射算法、关节范围或SDK发送逻辑。

额外保护通过新的 `runtime.py` 子类实现，不修改源主机文件：检查具名22维、源年龄、发布者数量，watchdog停止后拒绝自动恢复；停止生产者后等待原生watchdog，再退出驱动。

## 两处目录之外的手部专用新增文件

1. `av-pipeline:/etc/udev/rules.d/99-manus-sensor-dongle.rules`：安装MANUS接收器3325:0049的USB/hidraw权限规则，已reload；未全局触发USB设备，当前接收器需拔插使新权限生效。
2. `ms-thor:/home/user/manus_sharpa/robot_receiver.py`：部署独立手部接收程序。未创建/修改机器人systemd服务，也未运行接收器。

## 时间、动作及存储实现

- MANUS→V4→Sharpa生成22维绝对关节目标rad；左右手按附件名称匹配，不将手套角度当真实反馈。
- SDK成功回执的最终裁剪目标作为action；真实Sharpa反馈作为state；请求和拒绝回执只作为证据。
- 主机域198仅做手部控制；SSH记录桥进入机器人域29发布四个已有录制topic。
- 机器人receiver记录自身接收时刻；正式HDF5和训练对齐仍使用原MCAP log_time，统一为robot_receive int64纳秒。
- 原源header时间、receiver时间及MCAP log时间分别保存。跨主机传输包含延迟，不宣称硬件同步。
- extract核对MCAP与receiver日志的session/seq、设备号、SDK结果、关节值和header；不允许用历史无手部数据制造action/state。
- derive读取原source_timestamps网格，20Hz对齐：action取此前最近目标，state取最近反馈，误差最大50ms；任一行失败即拒绝输出，不删除原行或插值。
- 派生仅替换state/action的32:54、55:77，以及手部validity/mask/统计；其他34维、图像字节、列和时间/索引保持不变，生成新目录。
- validity为结构可用性。质量另写alignment_report，不把无效反馈包装成结构有效数据。
- HDF5及派生结果有hash；原频率手部replay放于hands/replay.json及独立commit中。

## 已完成验证

- 独立锁定环境安装和4个手部ROS包构建成功。
- MANUS native bridge、Sharpa共享库无硬件加载成功。
- V4优化器sidecar无硬件加载与退出成功。
- 6项离线测试全部通过。
- 全假MANUS+假Sharpa的ROS进程链测试通过；记录到左右手state和accepted回执，未连接机器人记录端。
- 假输入进程挂起后，native command watchdog停止，supervisor以失败退出；未初始化实体设备。
- `reports/public_before.sha256`列出的13个公共源码/说明文件及2份默认转换配置校验全部通过，没有内容变化。
- 尚未进行真实手套帧、Sharpa动作、实体停止、正式数据采集转换和实体replay验收。

## 使用命令

```bash
ssh av-pipeline
cd /home/descfly/apex_n_sharpa/integrations/manus_sharpa
./scripts/start_hands.sh
./scripts/start_hands.sh --fake --duration 4
```

上述默认入口是plan-only。实体启动命令在README中单独列明，需要先完成USB拔插、MANUS标定/左右侧确认和现场工作区检查。

## 保留的限制

- 不改公共文件，因此不调整其他部位坐标、图像分辨率、现有额外Parquet列或模型协议；若源数据与附件11列有其他差异，保留并报告，不能宣称整个数据集已完全符合附件。
- 原有 `egoav78_wire_compatible=false` 保留；本次只保证手部44维契约。
- 手部replay独立可用，但未修改机器人replay加入共同起点握手，因此不宣称IK/底盘/手部联合严格同步。
- SDK stop为失能调用，不是已经验收的硬件急停。

## 未修改范围

源主机源码、现有机器人启动/systemd配置、双臂/头部/底盘/相机文件、`src/apex2hdf5`、`src/hdf52lerobot`、`src/lerobot_client`、`src/lerobot_replay`、原训练数据及默认apex.json/lerobot.json均未修改。
