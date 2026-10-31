底盘 20 Hz 回放测试（2026-10-31）
==============================
位置：real_tianji:/home/descfly/apex_n_sharpa/test/
只新增测试文件；不更改原遥操、采集、手套、控制服务或数据。
本次仅离线验证；没有驱动机器人，实机预检/运动效果尚未验收。

1. 进入主机
   ssh real_tianji
   cd /home/descfly/apex_n_sharpa

2. 离线检查（默认，不连接机器人）
   .venv/bin/python test/replay_chassis.py --episode 2

3. 实机只读预检（不发速度，不切换模式）
   .venv/bin/python test/replay_chassis.py --episode 2 --check

4. 确认周围空旷、人在旁可急停，底盘静止，关闭其他底盘控制发送端；
   预检通过后，由操作者执行：
   .venv/bin/python test/replay_chassis.py --episode 2 --execute

   可先测试前 2 秒，保留原速度及时间，不加速：
   .venv/bin/python test/replay_chassis.py --episode 2 --seconds 2 --execute
   注意：前缀可能是静止帧，不一定能看到运动。

显式指定任务与数据编号：
   .venv/bin/python test/replay_chassis.py --task robot_import_20261024_egoav_v4 --episode 2 --execute
--task 默认 robot_import_20261024_egoav_v4，--episode 默认 2。
目前第 0/1 条旧导出缺少坐标系/单位声明，脚本会拒绝；第 2 条元数据完整，
离线检查通过。旧条目需要核实后重新导出元数据，脚本不会猜测坐标系。
目录：/mnt/apex-data/apex_n_sharpa/data/lerobot/<task>/episode_<8位编号>/
需要本仓库已有 .venv（numpy、pyarrow）和到 user@6.6.7.100 的免密 SSH；
机器人使用已有 /etc/apex/apex_ros_env.sh。不会自动改机器人模式或解除互锁。

读什么、发什么
------------
读取 Parquet 的 action.chassis，不是 observation.chassis，也不是 native_replay。
action=[vx, vy, wz]，单位=[m/s, m/s, rad/s]，坐标系=base_link（机体系）。
state=observation.chassis，是实测速度，仅作为记录参考，不作为控制命令。
发送 /move/ManualMoveCmd（ROS domain 0），头、双臂和手不发送控制命令。
当前已有数据 vy 全为 0；原控制接口只实现 vx/wz，因此明确拒绝非零 vy，
不把它静默丢掉，也不声称已验证横移能力。

频率和边界
---------
Parquet 为 20 Hz（整数时间戳间隔 50ms）；原始流约 50 Hz，不混用。
第 2 条：314 帧，首末采样跨度 15.65 秒；最后一帧保持一个 50ms 周期，
随后发送独立零速，命令序列为 15.70 秒，停止确认另需时间。
保留静止帧、原始数值和顺序，不修改数据、不插值、不加速、不恢复中途回放。
调度容许最多 10ms 迟到（相邻帧至少约 40ms）；超过则停止，不追赶补发。
指令范围 |vx|<=0.4m/s、|wz|<=0.5rad/s（仅留 float32 舍入容差）。
数据完整性 SHA256、频率、维度、有限值、时序及因果对齐均先校验。
最多 120 秒；较长数据需用 --seconds 选择 <=120 秒的前缀。

保留原底盘保护：反馈 200ms 超时、底盘模式/障碍/故障互锁、其他控制端竞争、
转向准备与速度跟踪检查。独立底盘回放不创建 ROS domain 29 观察节点，
不订阅或检查上身关节、上身控制模式、VR 连接状态；不需要启动上身反馈服务。
Host 心跳 100ms，丢失超过 500ms、SSH EOF、Ctrl+C 均请求停止。
停止发送零速，要求新鲜 vx/vy/wz 连续 200ms 在静止阈值内；最长等 4 秒。
Ctrl+C 后等清理结束；没有停止回执绝不报成功，请现场确认，必要时硬件急停。
预检失败且机器人明确返回 motion_started=false 时，不显示“停止未确认”；
已经尝试发送或因连接断开无法确认是否发送时，仍保留停止未确认提示。
这是软件保护，不是硬件急停；掉电、进程卡死或控制链故障不能由它保证停车。
速度开环回放不保证复现相同位置轨迹，勿将“发送完成”当成轨迹精度验收。

日志
----
主机 test/runs/<时间-随机编号>/：plan.json（来源/哈希/动作）、robot.log、report.json。
机器人执行时先输出 robot_report 路径：/tmp/apex-chassis-parquet-*/report.json；
结束/异常停止后先保存实测、发送记录、诊断及停止结果，再尝试通过 SSH 返回。
SSH 断开可恢复连接后按日志中的准确路径取回。/tmp 可能在重启后被清理。
若进程被强杀/机器掉电，报告可能只留下 starting；不能据此确认已停止。

文件说明
--------
replay_chassis.py：主机入口、数据校验、20 Hz 动作计划、SSH 心跳与结果保存。
chassis_endpoint.py：机器人临时内存执行入口、定时发送、保护和停车，不安装服务。
test_parquet_chassis_replay.py：离线自动测试，模拟传输/时钟，不连接机器人。
PLAN.md：本次实施与验证记录。
复用 src/chassis_replay/main.py 的原底盘保护，SHA256 固定校验；原文件若更新，
需复核后再更新哈希，不能删除校验来绕过。

验证命令（不运动）：
   .venv/bin/python -m pytest -q tests test/test_parquet_chassis_replay.py

初版验证：224 项测试通过（原 196 + 新 28），第 2 条真实数据离线通过。
原代码 git diff 为空，原底盘后端 SHA256 未改变。只读实机检查未通过网络阶段：
user@6.6.7.100: No route to host。因此还不能确认实机能启动，需先恢复连接。

2026-10-31 后续调整：按用户要求移除所有上身/VR 反馈依赖；保留底盘检查。
网络已恢复；实机只读预检通过（sent_frames=0、motion_started=false），
229 项测试通过（原 196 + 回放测试 33）；未实际执行运动回放。
详细结果与修改前备份位置见 PLAN.md 的后续记录。
