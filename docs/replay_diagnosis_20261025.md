# 2026-10-25 回放诊断

## 已确认

- 已部署客户端的原生固定宽度消息修复：body 6 槽有效 3，head 3 槽有效 2；验证保留槽位为零。
- 新增 --max-frames、QP/mux 命令计数、初始化过程目标/反馈采样和独立的映射预检标志。
- execution_ready 只在实际初始化通过后置为 true；physical_replay_verified 不把回零运动计入回放运动。
- 原运行状态下，回零 QP 四组命令各 251 条，mux 输出 0。独立命名空间的同版 mux 通过 UDP 能正常输出并完成 2 秒平滑。
- 通过 Apex 官方 host-control socket 正常重启遥操模块后，默认通信仍异常；UDP 临时用户服务恢复 QP→mux→实际运动。底层共享内存异常是怀疑原因，尚未直接证实；不删除 /dev/shm 文件。
- 单纯延长回零到 3 秒未解决跟踪残差，实验启动副本已停用，当前恢复原生 1 秒参数。未修改原厂文件、驱动增益或负载配置。

## 当前阻塞

回零完整时序记录：右臂 Joint6_R 的 QP/mux 目标达到 -0.147 rad，实际位置约 -0.13278 rad，相差约 0.01422 rad（0.815 度）。最新检查仍不满足 0.01 rad 回零阈值；没有放宽阈值，也没有发送 LeRobot 轨迹。

驱动配置含 left_pd/right_pd，不能据此单独认定当前 SDK 的控制模式。尚需确认驱动实际接受目标、模式枚举、PD 跟踪允许误差、末端无灵巧手时的负载配置。不能仅凭 ROS mux 目标认定底层 SDK 收到完全相同目标，不能直接断言机械故障或擅改增益。

最近有全过程采样的主机报告：
`/mnt/apex-data/apex_n_sharpa/data/replay_runs/robot_import_20261024_replay_v1/episode_00000002/1792937479706087629/report.json`
包含 home_samples、QP/mux/actual 对照、stop_confirmed=true、original_source_restored=true、sent_frames=0、physical_replay_verified=false。

## 当前服务状态

机器人 ms-thor：原 apex-teleop.service 停止；仅运行用户服务 apex-teleop-udp-audit.service，使用原 /etc/apex/apex-launch-teleop.sh 和 FASTDDS_BUILTIN_TRANSPORTS=UDPv4。无双控制器。该服务是临时的，Apex 系统模块面板可能显示遥操已停止；不要从面板重复启动。

## 将通信修复写入原服务

已经放置 `/home/user/codex-robot-test/repair_teleop_udp.sh`，本地来源 deployment/repair_teleop_udp.sh。脚本先核对静止、VR、原生回放/录制状态，再停止临时服务，备份已有同名配置，写入 UDP systemd drop-in，重启原服务。它不会启动轨迹回放，也不会修改任何驱动增益、录制文件或安全标志。

需在机器人终端运行：
```bash
sudo bash /home/user/codex-robot-test/repair_teleop_udp.sh
```
当前 SSH 没有免密 sudo，官方 host-control 只支持模块启动/停止/重启，不能写服务配置，所以持久化尚未执行。完成后仍需解决上述回零残差，再做前 20 帧和完整 314 帧测试。
