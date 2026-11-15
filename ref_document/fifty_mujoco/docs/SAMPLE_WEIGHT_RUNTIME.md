# sample 权重：本机运行与证据范围

更新：2026-11-12 22:58（Asia/Shanghai）。用户已解除环境/训练暂停，并同意原训练工程缺失时按现有材料重建。旧站立入口和默认模型未切换。以下第一部分保留前轮诊断范围；最新训练/40秒评估见文末。

## 已实际运行

- `transfer/sample_checkpoint.py`：独立31维ELU actor；`weights_only=True`、CPU读取，完整检查17个模型张量的键、形状、float32和有限值，std须正；actor严格恢复，critic/优化器不恢复。
- `tools/sample_actor_validate.py`：四份权重的批量1/32/256、零输入和观测裁剪端点；与source-files哈希匹配的原ActorCritic、NumPy float64及CPU/CUDA比较，atol1e-5/rtol1e-4全部通过。裁剪端点最大绝对差可大于atol，但仍满足绝对＋相对联合容差。原环境黄金输入/输出不存在，这只是固定输入网络对拍。
- `transfer/sample_source.py`：直接从归档URDF生成独立MJCF，不使用旧corrected模型。保留惯量坐标旋转、COM、碰撞几何、膝限位和连续髋；相邻刚体排除自碰撞，其余可碰撞。模型nu=0，归档耦合主动电机力矩与被动弹簧力矩通过qfrc_applied唯一施加。
- `tools/sample_source_probe.py`：质量23.259630515567018kg，初始净空0.1000001066m，仅左右轮接地、无初始穿透；10项检查及0.25s限幅PD通过。
- `tools/sample_rollout_diagnostic.py`：冻结3050，五类各5种子×10s，25/25完成；记录首次失败，不自动reset。后退只测−0.05m/s，前进+0.15m/s、左右±0.3rad/s；这是源模型诊断，未正式验收。
- `tests/test_sample_runtime.py`：14项通过，包括坏权重拒绝、电机变换功率守恒/可逆、转速包络与增益后再限幅、弹簧有限差分虚功、惯量只旋转一次、六通道方向、接触路径、reset与回放可重复性。标准库离线测试另21项通过。

## 尚未解决的证据边界

V0归档当前为34项字节匹配、12项换行等价、2项缺失，严格审计退出码仍为1；未修改原归档、manifest或冻结哈希。缺失项是artifacts/console.log与metrics/scalars.csv，本地同名检索和Git路径历史均未发现。186项源码仍只有127项能映射，六项关键原版本缺失。

诊断只执行哈希逐字节匹配的核心输入：env_cfg、URDF、actuator_model/hardware、gas_spring/resolved及权重。完整归档失败仍单独保留，不提升为V0完整通过。

源外部PD为5ms，每次两个MuJoCo2.5ms子步；策略周期10ms。关节速度按外部PD间隔对关节位置差分并绕回，机身速度按策略周期位置差分转入body坐标，上一动作取刚执行的分通道裁剪动作。参考轨迹先积分本步applied命令，再推进命令斜坡，供下一次观测。这些细节依据已匹配的归档派生类和**不匹配的本地底层候选**重建，未声称恢复了原环境全部时序。

MuJoCo使用implicitfast、滑动摩擦0.75、扭转/滚动摩擦0、solref=[0.01,1]等**暂定接触参数**；源PhysX接触响应尚未对拍。源450N/80mm弹簧安装本身仍geometry_confirmed=false，不覆盖本机450N/75mm资料。

回放按每次首次硬失败停止：非有限状态、膝超限超过1e-4rad、源弹簧行程检查异常、合倾角>60°、根高<0.15m或机身地面合接触力>10N。源配置的持续失败时间没有复现，诊断采用即时失败。速度误差在启动2s后按applied命令统计；空窗口保留null。根前向速度尚未转换为偏置命令点速度，合倾角P95也不替代正式roll/pitch分别统计。

诊断种子10000/10100/10200/10300/10400各连续5个，不占用既定开发与保留种子。未运行40×40s评估、过渡/弧线协议、目标迁移、随机化或PPO。本轮报告明确V2_accepted=false、V3_accepted=false；25/25仅是短回放完成次数。

## 复核命令

在`ref_document/fifty_mujoco`执行。GPU需使用可访问NVIDIA设备的终端；本次在沙箱外运行CUDA对拍。

```bash
.venv/bin/python -I -S tools/sample_weight_offline.py --out reports/sample_weight/offline_audit_20261112.json
# 当前归档有缺失/换行差异，以上命令应返回1，不能解释为通过。
.venv/bin/python -I -S tests/test_sample_offline.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_sample_runtime.py
.venv/bin/python tools/sample_actor_validate.py --out reports/sample_weight/actor31_cpu_20261112.json
.venv/bin/python tools/sample_actor_validate.py --cuda --out reports/sample_weight/actor31_cuda_20261112.json
.venv/bin/python tools/sample_source_probe.py --out reports/sample_weight/source_probe_20261112.json --xml assets/mjcf/sample_source_diagnostic.xml
.venv/bin/python tools/sample_rollout_diagnostic.py --seconds 10 --seeds 5 --out reports/sample_weight/rollout_diagnostic_3050_20261112.json
.venv/bin/python -I -S tools/report_index.py --reports reports/sample_weight
.venv/bin/python -I -S tools/report_index.py --reports reports/sample_weight --check
```

所有JSON/NPZ在[reports/sample_weight](../reports/sample_weight/index.json)；NPZ每个数组对应一个场景/seed，每行含步末状态、步初观测/动作及最后一次PD更新的力矩/弹簧长度，列布局及时刻写在JSON中。旧11-11报告原样保留。[离线说明](SAMPLE_WEIGHT_OFFLINE.md)记录的是前一轮历史。

## 从原.pt初始化的迁移训练（2026-11-12 冒烟）

可以继续利用原.pt的actor权重。`envs/sample_env31.py`提供独立31维重建环境，`learning/sample_ppo.py`提供新的31维critic和PPO，`tools/sample_train_smoke.py`验证初始化、critic预热、actor更新及保存恢复。原147维critic和原优化器不参与新实验；旧25/125训练器也不用于此路径。

重建奖励为`sample_transfer_tracking_reward_v1`：每策略步dt乘前向/航向/净空三个指数跟踪奖励，减动作变化与roll/pitch平方惩罚，真实失败再扣1。初期actor lr=1e-6、critic lr=3e-4；gamma=.99、GAE lambda=.95、clip=.2、entropy系数=.001；两轮训练epoch。更新后全rollout解析Gaussian KL>0.02即回退actor、std和Adam状态并减半lr。保存的source探索std不被0.01下限突然放大。

终止与timeout不自动reset。训练器先计算终止观测value，再显式reset；真实失败零bootstrap，timeout用终止value，并截断跨episode GAE。GPU负责网络和PPO，CPU MuJoCo负责物理；不是Warp并行训练。无episode结束时只报数量0，不计算空均值NaN。

最终v3从原3050初始化：2次critic预热＋2次actor更新，1024样本；再从新checkpoint恢复网络、actor/critic两套优化器和Torch随机状态，更新第5/6次，512样本。合计1536样本，无物理失败，最大更新全rollout KL0.00180744。恢复起点update4的网络、两套优化器逐项完全一致。**模拟器会重置，未声称物理轨迹逐步一致续跑**。所有原.pt哈希仍匹配。

v1探索std下限问题、v2只有更新前KL跳过而仍能接受单次过大提议的问题都已修正；原记录分别保留在training_v1/v2报告，最终使用v3与resume_v3。当前产物`runs/sample31_gpu_resume_v3_20261112/update_000006.pt`只验证迁移训练链路，没有宣称比原3050更好。

```bash
# 使用新目录运行，原权重不覆盖；需要GPU可访问的终端。
.venv/bin/python tools/sample_train_smoke.py --device cuda --out runs/my_sample31_smoke --updates 4 --warmup-updates 2 --envs 8 --steps 32 --max-seconds 60
.venv/bin/python tools/sample_train_smoke.py --device cuda --resume runs/my_sample31_smoke/update_000004.pt --out runs/my_sample31_resume --updates 2 --warmup-updates 2 --envs 8 --steps 32 --max-seconds 60
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_sample_runtime.py tests/test_sample_training.py
```

这是有上限的源重建训练冒烟：updates≤20、envs≤32、steps≤128、wall budget≤120s；正式目标模型训练入口仍待实现。resume校验模型/环境/奖励和代码哈希；环境或算法变化后应新建迁移实验，不能强行当作同实验恢复。

## 40秒重建源模型评估（2026-11-12 稍早）

`tools/sample_reconstructed_evaluate.py`对原3050评估命令点世界位移在当前heading框架的前向速度、绕回后的heading角差分、roll/pitch分别绝对值P95，补齐横向速度、参考位置/航向误差、停车漂移、电机速度与饱和、弹簧最小余量、Wilson95%区间。

五类各5次×40s，后退为−0.15m/s；再测−0.30后退、停车反向协议、左右转/弧线协议各1次×40s，28/28完成且均满足本轮指标。种子71000起按类间隔100，属于重建开发诊断，不用原预留的正式保留集。每类5/5的Wilson区间约[0.5655,1]，不足以宣称高置信度完成率≥90%。原PhysX接触对拍和本机目标模型仍未通过；本轮没有随机化。

```bash
.venv/bin/python tools/sample_reconstructed_evaluate.py --out reports/sample_weight/reconstructed_commands_20261112.json --seeds-per-class 5 --duration 40
```

最新证据：[继续训练核验](../reports/sample_weight/training_continuation_20261112.json)、[40秒命令评估](../reports/sample_weight/reconstructed_commands_20261112.json)、[43项测试](../reports/sample_weight/training_tests_20261112.json)。下一步建立本机目标profile并逐项回归，再决定正式微调或随机化训练；不再等待缺失原工程才能推进重建。

## 目标 profile（V5 第一步，2026-11-12）

`configs/sample_target_profile.json` 与 `transfer/sample_target.py` 把本机参数叠加在冻结的源模型上：
轮半径 0.060 m、M3508 内置 3591/187 × 16T→19T 链 = 22.80381、弹簧 450 N / 自由长 250 / 行程
75 mm。MJCF 由后处理生成（轮径、出生高度 +5 mm、膝压缩硬限位右 −3.72383 / 左 +3.72467 rad），
**不修改** `transfer/sample_source.py` 或 `envs/sample_env31.py`（哈希属于运行中训练契约）。
弹簧 175 mm 硬限位距默认低姿态仅 2.8 mm 销距，是该配置的紧边界。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_sample_target.py
.venv/bin/python tools/sample_target_probe.py --out reports/sample_weight/target_probe_20261112.json \
  --xml assets/mjcf/sample_target_diagnostic.xml
.venv/bin/python tools/sample_target_evaluate.py --out reports/sample_weight/target_commands_20261112.json \
  --seeds-per-class 1 --duration 10
```

13/13 静态与短时 PD 检查通过；冻结 3050 在目标 profile 上 1 seed × 10 s × 8 场景全部完成。
这是 V5 诊断，不是目标验收：轮惯量未重算、M3508 包络为 P19 代理×链比、弹簧为 450 N 常量、
URDF effort/velocity 限幅仍是源值，随机化关闭。

## 1 小时迁移微调与目标 profile A/B（2026-11-12→11-13 最新）

新增 `tools/sample_transfer_train.py`：正式分段入口，可恢复模型、两套优化器、Torch/CUDA RNG、
update 号和契约/超参哈希；每 20 updates 保存 checkpoint 并做一次 5 类 ×10 s 确定性评估。

实跑 `runs/sample31_formal_1h_20261112`：413 updates / 422,912 样本 / 3607 s（32 envs×32 steps，
20 s episode，前 2 次 critic 预热）。192 段 episode 全部走满，物理失败 0；全 rollout KL 最大
0.01469（无回退），actor 参数最大变化 0.001758。checkpoint `update_000413.pt`，
SHA-256 `5387fcccd4a8574fa03906b9c8f1046886109857204ee19d2cff6c0d20f86195`。
训练中 20 次评估全部 5/5 类达标，episode 回报均值 58.94→59.08。

目标 profile 同种子 A/B（五类×5 seed + 三个协议，各 10 s，共 28 场）：

| 指标 | 原 3050 | ft413 |
|---|---:|---:|
| 达标场景 | 28/28 | 28/28 |
| vx RMSE (m/s) | 0.0070 | **0.0029** |
| yaw RMSE (rad/s) | 0.0231 | **0.0048** |
| 净空 MAE (m) | 0.00922 | 0.00867 |
| 倾角 P95 (°) | 2.74 | 2.80 |
| 最小弹簧余量 (mm) | 2.4 | 2.5 |

```bash
.venv/bin/python tools/sample_transfer_train.py --device cuda \
  --out runs/my_segment --envs 32 --steps 32 --max-seconds 3600 \
  --save-every 20 --eval-every 20 --eval-seconds 10
.venv/bin/python tools/sample_transfer_train.py --device cuda --resume runs/sample31_formal_1h_20261112/update_000413.pt \
  --out runs/my_resume --envs 32 --steps 32 --max-seconds 1800
.venv/bin/python tools/sample_target_evaluate.py --seeds-per-class 5 --duration 10 \
  --checkpoint runs/sample31_formal_1h_20261112/update_000413.pt \
  --out reports/sample_weight/target_commands_ft413_5seed_20261112.json
```

边界：重建源物理、10 s×5 seed，不是 V4 正式验收；轮惯量、弹簧力曲线与 URDF effort 限幅仍
未处理，随机化关闭。


## 2026-11-15 当前实机候选

以上2026-11-12记录保留为历史。当前用户确认轮半径55mm、总减速比15.88，使用独立guarded31维环境；最终权重为`runs/hardware31_guarded_20261115/update_000100.pt`，评估/部署Kd0.14、膝目标余量0.03。名义独立保留203/203、DM偏差500/500达标。原60mm/22.8038 profile不再代表当前实车。

完整证据、40秒视频、控制配套要求和未通过项见[本轮最终结果](HARDWARE_TRANSFER_20261115.md)。未烧录，实车与自然姿态启动仍未验收。
