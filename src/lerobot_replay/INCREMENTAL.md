# 增量目标导出

在 av-pipeline 项目独立虚拟环境中运行：

```bash
cd /home/descfly/apex_n_sharpa
./export_incremental --task robot_import_20261024_egoav_v4 --episode 2
```

只读取指定 LeRobot episode；保留原始数据，拒绝 raw_data、MCAP 和 HDF5 输入。
输出为 WD 硬盘 `data/replay_incremental/<task>/episode_<编号>/`，包含 `targets.npz`、`report.json` 和 `completion.json`。这是回放目标文件，不是更改了语义却沿用原元数据的 LeRobot 数据集。

增量统一在恢复后的 base_link 坐标系计算，相对整条 episode 的首帧，不在 32 帧相机 chunk 边界重置：

- 位移：`dp(t) = p(t) - p(0)`。
- 旋转：`dR(t) = R(t) @ R(0).T`；恢复时 `R(t) = dR(t) @ R(0)`。
- 关节：`dq(t) = q(t) - q(0)`，保持 m/rad 单位，不做角度取模。

delta_matrix 的平移与旋转分别按上述公式定义，不能将其当成完整 SE(3) 相对变换直接与锚点矩阵相乘。首帧位移和关节增量为零；旋转是单位阵，rot6 编码为 `[1,0,0,0,1,0]`，不是六个零。

NPZ 同时保存 `action_delta78`、`state_delta78`、各自的绝对锚点、身体/头部控制增量及锚点、纳秒时间戳和 mask。state 与 action 各用自己的首帧锚点，不能互换。78 维无效槽保持零和 false mask，灵巧手与夹爪不生成任何有效命令。底盘速度不是位姿，因此未作为位置增量导出。

最新 episode_00000002 已转换：314 帧、20 Hz；25 项测试通过，保存后的数组与内存计算一致，从增量恢复原绝对目标的最大元素误差约 `4.49e-8`。

原生 Apex 控制器内部增量锚点和旋转组合公式尚未验证，输出明确标记 `native_apex_incremental_wire_verified=false`、`execution_ready=false`。本次未执行机器人运动，也未修改客户端的绝对位姿执行入口或放宽其检查。将轨迹重新锚定到机器人当前姿态会改变绝对路径，不能据此声称原轨迹已精确回放。
