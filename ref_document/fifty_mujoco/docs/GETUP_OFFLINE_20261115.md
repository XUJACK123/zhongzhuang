# 失能后起身的离线实现与运行入口

2026-11-15，Asia/Hong_Kong。

本次完成协议源码、合成数据测试、训练与评估入口、MuJoCo适配源码及记录格式。动态可行性、接触几何、PyTorch初始化和PPO更新需要在运行环境继续验证。第一版任务从车身正放、腿部失能停位开始。

## 文件与职责

| 文件 | 职责 |
|---|---|
| [训练规格](../configs/hardware31_next_training_20261115.json) | 时序、判据、奖励初值、课程、接触名单及种子 |
| [getup31.py](../src/fifty_mujoco/contracts/getup31.py) | STOP/GETUP/HANDOFF/BALANCE/FAILED、目标混合、奖励分项 |
| [getup_curriculum.py](../src/fifty_mujoco/learning/getup_curriculum.py) | 参数采样、开发集晋级、恢复及留出统计 |
| [getup_trace.py](../src/fifty_mujoco/transfer/getup_trace.py) | JSONL协议重放、同步遥测字段和新鲜度检查 |
| [MuJoCo适配](../src/fifty_mujoco/envs/sample_env31_getup.py) | 复用GuardedHardwareEnv31，增加零力矩静置、支撑、起身奖励和自主平衡验证 |
| [训练入口](../tools/sample_getup_train.py) | 复用SamplePPO，actor初始化、优化器/RNG恢复、预算和逐回合参数记录 |
| [评估入口](../tools/sample_getup_evaluate.py) | 开发/留出种子、逐场结果和课程报告 |
| [离线命令](../tools/sample_getup_offline.py) | check、replay、curriculum、acceptance |
| [离线测试](../tests/test_getup31_offline.py) | 时序、机械/通信失败、交接、课程、输入格式和适配层辅助逻辑 |
| [检查报告](../reports/sample_weight/hardware_getup_offline_readiness_20261115.json) | 实际测试结果、源码哈希和待运行验证项 |

## 现在可以运行的命令

从项目根进入工程目录后执行。三个命令使用Python标准库，预检查模式在导入Torch和MuJoCo之前返回。

```powershell
Set-Location D:\robomaster_material\electric_contol\zhongzhuang\ref_document\fifty_mujoco
python tools/sample_getup_offline.py check
python tools/sample_getup_train.py --prepare-only
python tools/sample_getup_evaluate.py --prepare-only --split heldout_nominal
```

检查报告中的 `physics_validation_completed=false` 随本次结果保留。适配层测试使用替身对象检查控制缩放和支撑计时。

## 起身协议

STOP输出零主动电机力矩。首次启动需观察到一次停机输入，故障或超时进入FAILED并保持零输出，再经过停机输入才可重启。协议按同一单调时钟更新时间，重复/倒退时间戳报错，运行期间超过25 ms的采样间隔触发失败。

GETUP采用使能力矩斜坡，稳定窗口满足规格并连续保持1 s后进入HANDOFF。HANDOFF使用三次平滑函数混合两个actor裁剪后的动作，再调用原有 `action_targets` 和膝目标保护。下一帧观测接收混合后的裁剪动作。连续髋角按现有观测契约计算角度差。

HANDOFF期间失稳即停车。进入BALANCE后保留原平衡失败判据，连续10 s通过后返回整段起身成功。仿真适配器在一次末端起身决策中自主推进这段平衡验证，使训练器收集的每个动作都对应GETUP或HANDOFF。奖励包含这段验证的累计成本和最终结果。

关节、气弹簧、分支、估计器和非法接触失败贯穿全流程。起身阶段可接受明确名单内的静置接触，成功窗口与平衡阶段要求机身离地。碰撞名单和40°起身倾角、2%体重轮支撑阈值属于初始工程配置，需要CPU动态检查。

## 课程和随机化

近站姿阶段从已接近站立的初始化学习平衡接入。支撑测量姿态阶段加入竖直虚拟支撑。这两个准备阶段的零力矩静置时间为0。

随后在随机撤支撑阶段执行1–3 s失能静置，支撑释放计时从使能后开始。无支撑自然停位及联合随机化阶段也执行真实零力矩静置，保留弹簧被动力并持续推进物理状态。

虚拟支撑采用竖直位置PD并限制在2倍模型体重内，0.5 s淡出。参数初值为5000 N/m和400 N·s/m。起身成功判据要求辅助力归零。初值和奖励权重在规格JSON中带工程假设标签。

联合阶段按回合固定四腿gain、地面摩擦和恒力弹簧倍率，名义参数保留概率20%。轮gain保持1。每个回合记录seed和实际采样参数。最终reset类别为near_stand、disabled_rest和asymmetric_rest，非对称初值采用左腿±0.03 rad扰动，所有状态须经过物理接触、行程和分支验证。

晋级需要当前阶段完整100个开发seed连续三份不同评估ID达标，并且平衡回归报告通过。相同报告重复应用保持幂等，相同ID内容变化报错。保存的课程状态包含计划哈希和评估摘要哈希。

## 运行环境恢复后的顺序

1. 在CPU运行环境中创建适配器，检查默认与自然姿态、允许接触名单、静置后的机械有效性、转速—力矩包络和弹簧行程，保存失败与轨迹。
2. 用CPU完成actor初始化和1–2次PPO更新，核对新critic/优化器、终止前观测、宏步平衡验证和检查点恢复。
3. 确认存在满足20 Nm请求上限、0.8兑现率等条件的起身路径，再开始课程训练。
4. 逐级开发评估，晋级后带课程状态恢复同一PPO实验。
5. 冻结权重、控制参数、reset分布和阈值，执行200个名义及500个联合留出seed，同时检查旧平衡回归。

运行环境按工程pyproject使用Python 3.12或3.13及其固定依赖。本次标准库检查运行于本机Python，实际Torch/MuJoCo兼容性在上述CPU冒烟中核对。

以下是后续运行环境中的首段训练命令，先设置较短预算做CPU冒烟，再扩大运行预算。

```powershell
python tools/sample_getup_train.py --factory fifty_mujoco.envs.sample_env31_getup:make_env --out runs/getup31_cpu_smoke_20261115 --device cpu --envs 1 --steps 16 --updates 2 --max-seconds 120
```

正式训练和恢复使用相同接口。每段输出目录使用新路径，检查点按update编号原子保存。修改课程阶段时，通过 `--curriculum-state` 指定开发评估晋级后的状态，通过 `--resume` 传入已有检查点，保留当前任务actor、critic、优化器和RNG。恢复时仿真器用新seed重置。

评估命令中的 `PATH_TO_GETUP_CHECKPOINT` 和 `PATH_TO_REPORT` 需要替换为选定文件路径。

```powershell
python tools/sample_getup_evaluate.py --checkpoint PATH_TO_GETUP_CHECKPOINT --split development --balance-regression-report reports/sample_weight/hardware_final_acceptance_20261115.json --out PATH_TO_REPORT
python tools/sample_getup_offline.py curriculum --report PATH_TO_REPORT --state runs/getup_curriculum_state.json
```

`--balance-regression-report`读取已有冻结平衡验收报告，检查候选SHA、Kd、膝余量、200场与3个切换协议。输出会保留该报告来源和哈希。平衡源码或物理模型发生变化后需重新生成相应回归证据。

留出评估在joint_randomization阶段执行。名义留出固定名义动力学参数，联合留出按规格采样。评估入口逐场保存 `.rows.json`，预算结束后保留部分结果并标记 `complete=false`。完整性、总体及逐reset类成功率和硬限位失败共同决定通过。每类同时报告Wilson95%区间。

## 记录格式与单位

JSONL第一行是header，随后每行一个sample。每个sample提供当前测量和本次控制更新使用的两个actor动作，重放会返回该时刻的后续目标。单个文件包含单次起身试验，并结束于首次成功或失败。

header字段包括schema、record_type、evidence_kind、plan_sha256、balance_checkpoint_sha256、getup_checkpoint_sha256、seed和reset_class。schema取 `hardware_getup31_protocol_v1`。evidence_kind取synthetic_protocol、simulation、hardware_synced或hardware_debugger。

sample字段包括record_type、enabled、signals、getup_action和balance_action。signals字段与 `getup31.Sample` 相同，包含单调秒时间、六关节位置、roll/pitch角度、三轴角速度、前向速度、净空、水平位移、双轮支撑、机身接触、辅助力及机械/遥测有效性。

同步实机记录另含raw_telemetry，保留以下量及其测量来源。

| 字段 | 单位/顺序 |
|---|---|
| mcu_tick_ms / can_last_tick_ms / imu_tick_ms / remote_tick_ms | MCU毫秒时钟及各反馈时间 |
| can_received | 六路CAN累计计数，右髋/右膝/右轮/左髋/左膝/左轮 |
| joint_velocity_rad_s / requested_motor_torque_nm | 六路规范顺序，rad/s和Nm |
| dm_reported_torque_nm | DM上报值，右髋/右膝/左髋/左膝 |
| wheel_reported_current_a | 两只轮的上报电流，右/左，A |
| imu_quaternion_wxyz / projected_gravity | 机体姿态与投影重力 |
| estimated_xy_m | 轮里程计估计位置，m |
| mode / inhibit | 实际控制模式与禁止输出位 |
| host_monotonic_s | 主机单调时间，s |
| battery_voltage_v | 测得电压，未测时填null |

规范延用固件30 ms电机、5 ms IMU、100 ms遥控新鲜度。姿态/接触真值、外部支撑状态、外部距离或负载读数需保留来源，未测量项目保留缺失标签。调试器串行采样已有约1 s实际间隔，适合稳态核对。新协议的10 ms时序验证需同步遥测。

`replay`结果保留evidence_kind。留出评分入口接受单一来源的simulation或hardware_synced数据，检查同一计划与两份权重哈希。合成协议结果在检查报告中单独标明来源。

## 当前验证范围

离线检查运行Python标准库测试、源码语法检查及命令行预检查，结果写入[检查报告](../reports/sample_weight/hardware_getup_offline_readiness_20261115.json)。现有站立候选仍沿用原来的有限仿真验收范围。新的起身适配器还需要CPU动态回归，实际起身训练、双actor MCU部署计时和实物起立验收按后续关卡执行。
