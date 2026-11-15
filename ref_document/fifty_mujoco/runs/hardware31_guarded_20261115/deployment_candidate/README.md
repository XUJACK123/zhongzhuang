# 仿真候选策略（未烧录）

模型来自 `update_000100.pt`（update 100）。网络结构和 31 维输入不变。
训练时轮 Kd=0.1，部署候选 Kd=0.14；若不同，属于训练后的控制参数整定，验收按后者进行。
此包的轮速度 Kd 为 **0.14**，必须与训练同步，PD 2 ms、估计器 5 ms、策略 10 ms。
若有 `physical_state_guard.patch`，动作目标使用带余量的 `rl_knee_limits`，状态保护使用
独立的 `rl_physical_knee_limits`；需配套 `chassis_candidate.c`，不能只替换权重文件。
`robot_config_candidate.h` / `evaluated_command_caps.patch` 把遥控上限限定为前进0.15 m/s、
yaw±0.30 rad/s；高速度/高角速边界扫描存在跟踪超标，未接受原固件全部遥控范围。
候选配置保留原校准与输出标志，未把未验证标志改为已验证。
轮半径 55 mm / 总减速比 15.88 / 电机空间上限腿 20 Nm、轮 2 Nm。

`model.c/.h` 与 `actor31.pt` 是可复核候选产物；原固件、放行标志与实物均未修改。
主机 C 与 Torch 的 256 个输入对拍通过，不能替代 MCU 时序、启动或实车平衡验证。
实测支撑姿态直接释放仍是独立失败项；右轮反馈和有效力矩需要现场复核。

`simulation_snapshot.xml` / `simulation_physics.json` 与 `simulation_code/` 保存本次几何、惯量、气弹簧、电机和控制实现快照。
XML 本身不施加外部气弹簧/电机力，也不包含训练 reset；复现需配套环境代码，不能仅用 XML 默认 qpos 播放。
