# 2026-11-18 双腿后靠坐地起身训练

从双轮位于髋轴后方、机身底部接地开始，无辅助起身，经过 1 s 稳定驻留、0.3 s 平滑交接和 10 s 独立平衡。本轮从 03:53:41 开始，冻结后于 05:11:42 完成验收审计，约 78 分钟，低于 6 小时预算。

训练使用最新 11 月 17 日采集汇总、用户选定的 STEP、21.9 kg 质量和 55.704230 mm 有效轮半径，CPU/GPU 均使用 15 MPa 名义弹簧曲线。静态陀螺仪实测标准差用于仿真噪声。九轮电流探针用于审计电流跟随与运动响应，机械有效增益、自由销距、摩擦、惯量与滞回仍有假设。15 个仿真初态逐一通过 3 s 无主动电机输出静置。精确实车坐地姿态和接触载荷没有同步真值。

| 冻结候选 | 名义通过 | 联合组通过 | 实际受扰样本通过 |
|---|---:|---:|---:|
| 31 维观测触发的起身策略及平衡接管 | 100/100 | 21/100 | 0/79 |
| 标准 31 维 MLP 起身策略及平衡接管 | 49/100 | 14/100 | 0/79 |

主候选名义条件达到 95% 目标，联合扰动未通过。联合分布保留 20% 名义成分，本次实际抽到 21 个名义样本。主候选扰动失败主要发生于起身超时和位移超过 0.3 m。两套候选共 400 场，未剔除失败回合，成功场均有 1000 个平衡周期，逐轨迹验证全程辅助力与力矩为零。标准 MLP 有 2 场非法接触。针对 0.97 倍腿增益的动作优化在训练回放得到 8/8 次完整接管，工作范围局限于调好的工况。联合拟合未提升通用网络，最终权重以开发集选型为准。

可复现入口（工作目录为 `ref_document/fifty_mujoco`，输出使用新路径）：

```bash
run=runs/getup_seated_20261118_035341
.venv/bin/python tools/realdata_evaluate.py   --checkpoint "$run/frozen/primary.pt"   --balance-checkpoint "$run/frozen/balance.pt"   --config "$run/frozen/configuration.json"   --task getup --settle-seconds 1 --seed-base 911500 --seeds 1   --out /tmp/fifty_seated_development_replay.json
```

- `frozen/manifest.json` 保存权重、配置、来源代码、固件 C 观测器和保留集规范的哈希。标准网络另存 `frozen/standard_mlp.pt`。
- `final_summary.json` 和 `heldout_cpu/` 保存独立验收、逐场结果与连续轨迹。 `summarize_holdout.py` 可复核哈希、阶段、辅助力和样本数量。
- `primary_CAD_development.mp4` 为成功的 MuJoCo 回放。CAD 网格用于显示，动力学依然使用仿真碰撞模型。 `training_progress.png` 为训练与验收曲线。
- `input_manifest.json`、`input_audit.json` 和 `validated_reset_bank.json` 记录数据用途与初态证据。
- 86 项针对性回归通过。 `standard_mlp_export/` 保存隔离 C 推理副本，1024 组主机对拍通过，最大绝对误差约 1.96e-5。物理真机起身和控制部署仍未验证。

宽范围联合训练和低出力专项试验完成后，独立扰动分布仍表现为起身超时或离地后漂移。后续使用这些结果时应保留名义与联合扰动的适用范围。
