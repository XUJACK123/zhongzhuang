# sample 权重离线工作交付

更新：2026-11-11 14:29（Asia/Hong_Kong）。当前仅为标准库数学与文件检查；不包含Torch推理、MuJoCo/GPU仿真或训练。

## 已交付

- [离线审计工具](../tools/sample_weight_offline.py)：48文件校验、4个检查点结构与参数有限值、源码哈希与候选副本检索、网格依赖、URDF静态FK/质量/惯量、配置冻结校验。
- [31维数学契约](../src/fifty_mujoco/contracts/sample_actor31.py)：观测31维、额外6维参考特征、连续髋周期误差、分通道裁剪、膝目标限幅、净空、指令斜坡、SE(2)参考积分。纯Python标准库；调用方负责传入同一时刻的状态，不自行读取仿真器或重置环境。
- [actor配置](../configs/sample_actor31.json)、[源参数输入及网格映射](../configs/sample_source_nominal.json)、[评估协议](../configs/sample_command_evaluation.json)、[随机化课程规范](../configs/sample_randomization.json)。这些JSON是独立规范，不是已接入现有Config/CLI的运行配置；运行/随机化开关保持关闭。
- [测试](../tests/test_sample_offline.py)：18项通过。[审计报告](../reports/sample_weight/offline_audit.json)与[测试原始输出](../reports/sample_weight/offline_tests.json)可复核。

## 实际核查结论

归档48/48长度和SHA-256一致；2975/3000/3025/3050四个快照各17个模型张量形状正确、全部参数有限，std为正；无encoder，结构为31维前馈actor。检查没有执行网络forward，不能叫Torch恢复或数值推理对拍通过。

源码清单186项：当前目录95项原始字节一致、8项仅换行归一化后一致、14项不同、69项缺失。再使用归档中的24个匹配快照建立引用映射，合计127项有可用来源，**没有覆盖当前副本**。

7个训练网格均在本地找到，SHA-256与source-files清单一致。但归档URDF直接从artifacts目录加载时，其../meshes路径无效；后续应读取sample_source_nominal中的显式网格映射，不能只复制URDF就宣称资产齐全。

源URDF采用env_cfg覆盖后的低姿态：两轮最低点约-6.1e-15 m（浮点误差），机身底部净空0.1000001066 m；总质量23.259630515567018 kg，7个刚体质量正、惯量正定并满足主惯量三角约束。只验证静态几何；未检查自碰撞、接触求解、弹簧力方向、机构完整可行域或站立稳定性。

## 仍缺少的来源

以下6项没有找到哈希匹配的本地/归档版本，训练提交019a4b63f2e33f2473a656ae0cbda1a1b3a6e4e0也不在当前本地Git对象中：

- 原legged_robot.py：检查了50个同名候选，均不匹配。
- 原ppo.py、on_policy_runner.py、task_registry.py：各检查了2个候选，均不匹配。
- 原evaluate_tracking.py、moving_yaw_rewards.py：当前缺失。

完整路径、预期哈希、候选检索结果在offline_audit.json的source_comparison/source_resolution中。其余未恢复项也逐一列出，不能把127/186写成整份训练工程恢复完成。即便补齐这些文件，归档脚本还依赖原package布局，不应从artifacts目录直接运行train_fifty.py。

## 复核命令（在fifty_mujoco根目录）

仅用已有Python解释器；-I -S排除外部PYTHONPATH和site-packages，不需要安装仿真依赖：

~~~powershell
./.venv/Scripts/python.exe -I -S tools/sample_weight_offline.py --out reports/sample_weight/offline_audit.json
./.venv/Scripts/python.exe -I -S tests/test_sample_offline.py
./.venv/Scripts/python.exe -I -S tools/report_index.py --reports reports/sample_weight
./.venv/Scripts/python.exe -I -S tools/report_index.py --reports reports/sample_weight --check
~~~

其他系统可将解释器前缀改为已有Python 3.12+。这些命令不经过fifty_mujoco.cli，不执行完整pytest，不导入Torch、MuJoCo、Isaac Gym、NumPy或Warp。

## 暂缓的环境工作

Torch严格加载与原输出对拍、CPU MuJoCo源模型复现、目标模型迁移、GPU/Warp并行验证、PPO微调及随机化训练均尚未执行。V0仍为来源部分核实；V1仅完成静态结构/有限值；V2仅完成源URDF的部分静态准备，所有动力学放行仍未通过。后续恢复原底层代码或取得黄金输入输出后，才可确认时间对齐、速度差分和reset细节。
