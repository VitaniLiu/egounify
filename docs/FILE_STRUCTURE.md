# 仓库文件结构

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
