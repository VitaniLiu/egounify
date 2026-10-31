# Sharpa遥操启动检查

2026-10-27，按用户要求仅启动手部MANUS→V4→Sharpa，未启动机器人其他部分。

## 结果

两只Sharpa连接成功，实际设备序列号为左CE50903ECE53、右CC50933FCC53，左右设备型号与绑定一致。
会话曾收到SDK成功回执，但未能持续运行；命令流中断后，原生0.2秒watchdog停止，supervisor退出，双手断开。
没有通过放大超时、自动重新使能或切换映射算法维持运行。

## 检查依据

- 机器人apex-robot、apex-teleop、apex-camera启动前后均inactive，未调用任何启动/重启命令。
- 原先已active的gamepad/chassis记录辅助服务保持原样。
- 在不连接Sharpa的15秒持续MANUS+V4诊断中，左侧740次、右侧739次得到有限目标；分别出现56、57次50ms等待内没有新MANUS帧。
- 有帧时V4预热后计算通常5–6ms，最大约68ms。
- MANUS SDK反复出现failed to set quantum sensor enabled / running，error -6。
- V4还出现Inf目标函数告警。短时有限输出不等于整个输入链路稳定。
- 原命令watchdog和新的supervisor均按设计停止；最终没有残留手部遥操或记录接收进程。

## 本次仅手部文件调整

- `scripts/start_hands.sh`：OPENBLAS/OMP/MKL单线程，降低优化器启动与计算抖动。
- `recording/bridge.py`：统计双侧请求数与最近到达时间，供启动预热检查。
- `runtime.py`：先启动MANUS/V4，等双侧连续有效目标后才连接Sharpa，保持原停止阈值。
- `scripts/check_mapping.py`：新增真实手套+V4、不连接Sharpa的持续诊断入口。
- `README.md`：更新当前实机验证边界。

调整后6项离线测试、全假设备预热流程、假输入断流watchdog测试通过。公共源码和配置hash未变。

当前状态：**手部已停止，未保持遥操**。需先处理MANUS持续数据间断及SDK传感器错误，再进行实体持续遥操验证。
