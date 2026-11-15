# 仿真候选策略（未烧录）

模型来自 `update_000050.pt`（update 50）。网络结构和 31 维输入不变。
此包的轮速度 Kd 为 **0.1**，必须与训练同步，PD 2 ms、估计器 5 ms、策略 10 ms。
若有 `physical_state_guard.patch`，动作目标使用带余量的 `rl_knee_limits`，状态保护使用
独立的 `rl_physical_knee_limits`；需配套 `chassis_candidate.c`，不能只替换权重文件。
轮半径 55 mm / 总减速比 15.88 / 电机空间上限腿 20 Nm、轮 2 Nm。

`model.c/.h` 与 `actor31.pt` 是可复核候选产物；原固件、放行标志与实物均未修改。
主机 C 与 Torch 的 256 个输入对拍通过，不能替代 MCU 时序、启动或实车平衡验证。
实测支撑姿态直接释放仍是独立失败项；右轮反馈和有效力矩需要现场复核。
