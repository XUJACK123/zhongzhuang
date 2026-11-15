# 2026-11-15 实机参数迁移结果

最终候选为 `runs/hardware31_guarded_20261115/update_000100.pt`，SHA-256：
`9fc23553c3d3ad00960c16c80aad5ef5bccd3e6d19a047967aeadf5097e4f4c3`。
部署与评估使用 **轮速度 Kd=0.14、膝目标角余量0.03 rad**。权重训练时 Kd=0.10，0.14 是在开发集上完成的控制参数整定；保留集只评估一次，没有用于再训练或选参数。

本轮五段 PPO/冒烟累计 **66.08 分钟、332,544 样本**，小于用户4小时训练上限。
全流程另设北京时间06:23:45的保守截止。旧权重和旧模型保留，未烧录或操作实物。

## 仿真验收证据

状态：`passed_limited_reconstruction_scope`。下表速度和yaw是所有有效统计步加权的RMSE，净空是加权MAE；启动后前2秒不计跟踪误差，但启动期间失败仍算失败。

| 数据集 | 五类达标 | 附加协议达标 | vx RMSE m/s | yaw RMSE rad/s | 净空 MAE mm | 最差电机饱和率 |
|---|---:|---:|---:|---:|---:|---:|
| 开发集 | 200/200 | 3/3 | 0.01279 | 0.03524 | 4.88 | 0.025% |
| 独立名义保留集 | 200/200 | 3/3 | 0.01279 | 0.03524 | 4.88 | 0.025% |
| 独立DM增益保留集 | 500/500 | — | 0.01520 | 0.04030 | 5.62 | 0.025% |

名义集每类40 seed×40s，五类为站立、+0.15/-0.15 m/s、±0.3 rad/s；附加协议是完整40秒高速后退、停车反向、左右弧线。
名义保留种子52000–52199；DM保留种子62000–62499，每类100×40s。
DM测试仅随机四个腿电机兑现率0.8–1.0，轮为1，每个episode保持固定，20%为名义环境。该范围来自工程假设，不是已标定的真实概率分布。
每类Wilson95%区间、逐场首次失败及指标在机器报告中；不把这些区间推广成真实世界成功率。

- [最终验收报告](../reports/sample_weight/hardware_final_acceptance_20261115.json)
- [名义保留集](../reports/sample_weight/hardware_final_heldout_nominal_20261115.json)
- [DM保留集](../reports/sample_weight/hardware_final_heldout_dm_20261115.json)
- [训练预算与分段索引](../reports/sample_weight/hardware_campaign_20261115.json)

## 实机参数与方法变化

用户确认当前轮半径 **55 mm**、总减速比 **15.88**，取代旧CAD推算的60 mm/22.8038。
接入当前固件的2ms PD、5ms差分速度保持/无滑移里程计、10ms actor；物理步1ms。
保持31维输入、原关节默认角与动作尺度；使用450N恒力弹簧、250mm自由长、75mm行程。

旧控制配置允许膝目标超过弹簧压缩端，完整弧线协议出现限位失败。新配置只收紧目标角，不放宽物理限位或失败门槛。
目标角范围为右[-3.693829953,-2.559268358]、左[2.559268358,3.694667691]rad；物理压缩端为右-3.723829953、左3.724667691rad。
训练新增轮饱和惩罚以及弧线/每5秒换命令课程。GPU用于策略和PPO，物理使用CPU MuJoCo；没有冒充Warp训练。
标量电机运算与参考Torch版本500组数据、1秒闭环及40秒同种子回放对拍通过，保留参考实现。

## 部署候选与验证

[候选目录](../runs/hardware31_guarded_20261115/deployment_candidate/README.md)包含：

- `actor31.pt` 和生成的 `model.c/.h`；训练恢复使用原 `update_000100.pt`，actor31是部署用精简权重。
- `chassis_candidate.c` / `physical_state_guard.patch`：把动作目标限位和物理状态保护限位分开。
- `robot_config_candidate.h` / `evaluated_command_caps.patch`：候选遥控上限前进0.15m/s、yaw±0.30rad/s；保留原校准与输出标志，不把未验证项改成已验证。
- `manifest.json`：源训练与部署控制配置、生成文件哈希和C对拍结果。

主机C actor的256输入与Torch对拍通过；另200个状态的观测、参考推进、目标和电机指令对拍通过。
最大观测误差8.8e-8、目标角误差2.1e-7rad、电机指令误差4.3e-6Nm。
测试：全项目 **365 passed / 11 skipped**，独立标准库21项通过；41项针对性测试也通过。详情见[测试报告](../reports/sample_weight/hardware_final_tests_20261115.json)。
本机没有可用`arm-none-eabi-gcc`，**未完成MCU交叉构建、时序或实机平衡验证**。
当前固件树未修改；候选源码需单独审阅并由用户明确发起硬件验证。

复现命令（工程目录）：

```bash
.venv/bin/python tools/sample_hardware_batch_evaluate.py \
  --checkpoint runs/hardware31_guarded_20261115/update_000100.pt \
  --backend guarded --knee-margin .03 --wheel-kd .14 \
  --suite nominal --scope heldout --seeds-per-class 40 --duration 40 \
  --workers 4 --deadline-epoch YOUR_AUTHORIZED_DEADLINE --out NEW_REPORT.json --extra-protocols
```

生成相同控制配置的候选（输出到新目录，不写入活动固件）：

```bash
.venv/bin/python tools/sample_hardware_export.py \
  --checkpoint runs/hardware31_guarded_20261115/update_000100.pt \
  --wheel-kd-override .14 --out NEW_CANDIDATE_DIR
.venv/bin/python tools/sample_hardware_package_snapshot.py --bundle NEW_CANDIDATE_DIR
```

旧25/125维CLI不能直接加载本次31维checkpoint；回放使用`sample_hardware_video.py --wheel-kd .14`。

不要对已经用过的保留集反复调参；上面用于说明接口，新实验应另设并冻结保留种子。
不要直接运行原固件默认`make export-policy`去覆盖候选限位；使用本工程`sample_hardware_export.py`及相应控制参数生成候选。

## 视频与轨迹

三个录像均连续40秒，首次失败即停止，没有拼接reset；渲染显示碰撞模型。
各视频同名JSON记录采样语义，同名NPZ包含逐步观测、动作、状态和指标。

- [站立](../reports/sample_weight/media/final_stand40.mp4)
- [停车反向](../reports/sample_weight/media/final_reversal40.mp4)
- [弧线与左右转](../reports/sample_weight/media/final_turning40.mp4)

[图表](../reports/sample_weight/figures/)包含控制参数扫描、训练证据和名义保留集指标。

## 附加敏感性复核

最终配置在7种固定条件、每类5 seeds×40s，共 **175/175** 达标：名义、DM统一兑现率0.8/0.9、有效接触摩擦0.6/0.9、恒定弹簧力405/495N。此前旧候选的405N失败已在此配置回归通过。
这仍是开发敏感性诊断，不是弹簧力曲线实测或质量/摩擦/惯量联合随机化验收。

## 明确未解决的边界

高速度/高角速扫描共63场，全部走满，只有51场跟踪达标。
负向2.5rad/s与部分高速弧线仍超标，所以不接受当前固件完整遥控范围。
这是开发边界证据，不是通过后可无限扩展的命令域。

实测参考姿态来自支架支撑，并非无支撑平衡点。最终候选在两种腿兑现率下释放支撑，
**0/2完成**，约0.08–0.10s机身触地。
另54组简单启动脉冲搜索也未找到成功，不把这些失败称为硬件不可能起立的证明。
架空仅平衡模型弹簧约需18.13Nm/电机；若有效上限只有16Nm可能不足，但自重、地面反力和摩擦会改变需求，不能替代台架测力。

质量/惯量沿用归档，等效串链、恒定弹簧力与接触仍有暂定项；未验证全部摩擦、质量、惯量、弹簧曲线与联合随机化。
原Isaac/PhysX黄金时序仍缺失；真实CAN时延、量化、MCU时序与地面里程计未由仿真证明。
最近实机记录曾有右轮反馈丢失，需要现场确认。
因此`target_model_accepted=false`、`real_robot_accepted=false`，本轮通过范围仅为上述MuJoCo重建验证。
