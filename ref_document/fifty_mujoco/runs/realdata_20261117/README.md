# 2026-11-17 实测数据离线训练

本轮从01:53开始，总预算5小时，最晚06:53停止并保存。目标是有支撑起身、完全撤支撑及独立平衡接管。最新训练状态在`latest_status.json`，按任务分别绘制的曲线在`training_progress.png`；项目根目录`progress.md`保存阶段记录。本轮已结束，独立结果未通过整体验收；结论见`final_summary.json`、`heldout_audit.json`。

原始采集文件没有改写。`input_manifest.json`保存输入文件哈希；`step_audit.json`记录指定264 MB STEP的复核；`evidence_and_spring_calculation.json`区分实测、CAD、厂家参考和假设。质量21.9 kg、有效轮半径55.704230 mm采用用户认可记录，左轮采用对称假设。粗测70°大腿/120°膝内角用于有支撑姿态分布，不能当成精确编码器标定。

BG450-080优先于CAD旧型号。450 N初始力、80 mm行程、250 mm自由销距、1.7倍进程比、15 N密封滞回、20 Ns/m阻尼、转子折算惯量和齿轮摩擦均是候选假设。厂家BKB0.45参考与BG450-080对应未确认，150 bar是标签上限，实际压力未知。STEP只能支持几何/力臂计算，不能独立确定轴力、滞回或实际充气状态。旧参考位与粗测方向冲突仍未解决。

每个训练目录保存`contract.json`、所用源码副本、`rows.jsonl`、周期检查点和`training_report.json`。停止或变差的实验全部保留。监督拟合批次、PPO学习转移、物理策略周期、GPU回合成功和CPU开发成功分别记录，不相互替代。GPU采用MuJoCo Warp物理与Torch控制图；实际固件C观测器用于CPU权威复测。GPU物理与CPU闭环对拍仍有误差，GPU成功不能直接作为验收。

仿真流程先在虚拟夹具上零主动输出静置1秒，再渐增主动输出、转移竖直负载并撤去水平力与力矩。成功要求全部辅助力/力矩为零、双轮各承担至少2%体重、无机身接地，满足倾角8°、角速度0.5 rad/s、净空误差20 mm、腿角误差0.15 rad、累计位移0.3 m等门槛，并完成1秒稳定等待、0.3秒交接和10秒冻结平衡策略验证。20 Nm腿电机/2 Nm轮电机上限和原物理膝边界没有提高。虚拟夹具并非实物纸箱辨识结果，零输出有支撑静置不等于自然无辅助停位。

课程有两项显式区别：旧课程起身超时6秒，后续慢速课程为12秒；部分课程在稳定等待和交接期间均要求速度≤0.035 m/s，entry课程只在交接前的1秒等待要求≤0.035 m/s，0.3秒交接与之后10秒仍用主协议≤0.08 m/s。`configs/`中分别留存配置，报告必须注明课程，不能直接混合成功率。30秒开发压力测试单独记录。后续的时长奖励只改变学习目标，失败回合仍判失败。

解析反馈/力矩分配教师用于建立可行路径，标准31维MLP再通过监督学习和PPO训练。`media/full_survey_teacher_development.*`是从开发集成功回合中选取的连续轨迹/视频，使用解析教师，不能当作网络独立验收。`export_near_u20/`是部分成功的网络推理导出与C对拍，未注册或安装为实车起身actor。

从`ref_document/fifty_mujoco`目录执行，例如：

```bash
.venv/bin/python tools/realdata_status.py
.venv/bin/python tools/realdata_evaluate.py --checkpoint runs/realdata_20261117/getup_feedback18_near/update_000025.pt --balance-checkpoint runs/realdata_20261117/feedback_grid_near/balance_rank00.pt --task getup --reset-range 0 .2 --settle-seconds 1 --seeds 8 --seed-base 615000 --out /tmp/realdata_cpu_replay.json
.venv/bin/python -m pytest tests/test_realdata_candidate.py tests/test_getup31_offline.py tests/test_sample_hardware_fast.py
```

独立种子710000–719999在冻结选型前不使用；开发种子610000–619999不用于宣称独立通过。本轮没有访问板卡、烧录、改实车零位、清失锁标志或使能电机。离线推理一致性、单次成功和仿真验收均不能替代实车T1/T3/T4/T8。


## 最终结果（权重冻结后）

| 冻结候选 | 近站姿8种子 | 中间姿态8种子 | 完整粗测姿态8种子 |
|---|---:|---:|---:|
| 近站姿update25 | 7/8 | 1/8 | 0/8 |
| 完整姿态update20 | 0/8 | 2/8 | 2/8 |

18个假设参数敏感性回合全部失败。66/66回合均完成，来源/种子/轨迹逐帧复算一致；没有把失败或提前结束回合剔除。开发集完整姿态6/8下降到独立完整姿态2/8，说明泛化不足。**两套模型均未通过所声明范围的整体验收，不能当作可直接上车的起身模型。**

`frozen_candidates/near`和`frozen_candidates/full`各保存匹配的`getup.pt`、`balance.pt`、配置和物理快照。`heldout_cpu/`保存所有分组报告、逐回合增量结果与原始轨迹；`heldout_audit.json`复核全部66条。`final_export_*`为两套起身/平衡C推理副本，各1024组C/Torch对拍通过；没有安装到生产固件。

`media/near_u25_development.mp4`、`media/frozen_full_u20_development.mp4`是最终权重的成功开发回放，均有逐帧轨迹和审计；它们不是整体成功率的替代。`heldout_results.png/.pdf`及`training_progress.png`可单独查看。所有未选中的实验和失败日志仍保留在本目录。

结束后没有根据独立集继续训练。下一轮需优先缩小弹簧力/滞回、惯量、摩擦、参考位和真实时序的不确定性，再用新的独立种子验收。当前T3/T4/T8实车标志均未因此变为通过。
