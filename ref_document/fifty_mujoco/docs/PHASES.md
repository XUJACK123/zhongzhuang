# 关卡状态（P0–P10）

> **2026-10-31 最新覆盖说明**：下表的仿真/测试计数是旧串联模型或旧骨架历史证据，本轮未重跑。旧模型 GPU 对拍及 64×60 s 评估已见 progress.md §22（策略 0/64）；新闭链须重新过关。平面杆长已纠正为 210/250 mm，末端 96.600 mm；弹簧三维标称销距 227.760939 mm，已压缩 22.239061 mm、剩余可压缩 52.760939 mm。当前运行仍为旧锚点 300 N。完整检查顺序以 progress.md §25 为准。

> 验证级别只使用方案 §9.2 的六档，且不能跨级标记：
> 静态检查通过 / CPU仿真通过 / GPU仿真通过 / 训练工程通过 / 策略评估通过 / 实机验证通过。

| 关卡 | 内容 | 状态 | 证据 |
|---|---|---|---|
| P0 来源冻结 | 源 URDF、7 个 STL 哈希、关节树、已知缺陷复现 | ✅ 静态检查通过（5/5） | `reports/assets_audit.json` |
| P1 坐标与物理资产 | 单次刚体变换重算 visual/COM/惯量、重生成碰撞、两个 MJCF、§3.6 放行 | ✅ 静态检查通过（18/18） | `reports/model_validation.json` |
| P2 正立初始化 | SpawnSpec、根四元数、轮底离地、顶部标记、无穿地 | ✅ CPU仿真通过（8/8） | `reports/spawn_report.json` |
| P3 安装几何与力模型 | 气弹簧两端安装点、行程、力方向、虚功 | ⚠️ **接口通过（9/9），关卡未关闭** | `reports/spring_report.json`；锚点已从 STEP 确认（上=大腿板、下=小腿板），名义力 450 N、自由长度 ≈250 mm、行程 75 mm 均已确认，标称已压缩 22.239061 mm、剩余可压缩 52.760939 mm；新锚点与模型尚未接入（`docs/STEP_GEOMETRY_AUDIT.md` §6.10–§6.11、`mechanism-check` 的 `gas_spring` 段） |
| P4 电机与被动力组合 | 六通道方向/单位、A/B/C 组合、断使能仍有被动支撑 | ✅ CPU仿真通过（12 + 3） | `reports/joint_test.json`、`reports/spring_sweep.json` |
| P5 接触与策略契约 | 25/125/149、PD、五帧历史、terminal/reset | ✅ CPU仿真通过（4 + 12）；**2026-10-29 重开修复后重跑** | `reports/contact_test.json`、`reports/contract_test.json`、`tests/test_env_contract_reset.py` |
| P6 教师复现 | ONNX 权重恢复、原模型复现、通道语义 | ⚠️ **权重恢复通过，迁移失败** | `reports/teacher_report.json`、`reports/teacher_eval.json` |
| P7 GPU 后端 | mjlab + MuJoCo Warp、CPU/GPU 对拍 | ⚠️ **仅物理批量通过，端到端未通过**；2026-10-29 的 GPU 侧重置/终端/弹簧特征修复**未在 CUDA 上执行** | `reports/gpu_smoke.json` |
| P8 训练冒烟 | 多环境、完整 PPO 更新、保存/恢复 | ✅ 训练工程通过 | `runs/*`、`tests/test_learning_pipeline.py`、`tests/test_checkpoint.py` |
| P9 正式迁移训练 | critic 预热 → 教师辅助 → 独立 PPO | ⏳ 未开始（教师阶段已被证据否决） | — |
| P10 导出与回放 | ONNX 数值对拍、普通 MuJoCo 独立回放 | ✅ 训练工程通过 | `runs/export/*.onnx` + `play` |
| 新增：独立站立评估 | 64 独立初态 × 60 s、逐环境首次失败、不混入 reset 状态 | 🟡 **评估器已实现并实测（CPU）**；demo_run4 在 6 s 窗口 8/8 存活、vx MAE 0.201（`reports/stand_eval_demo_run4_6s.json`）；**64×60 s 未跑完，门槛仍空缺** | `tests/test_stand_eval.py` |
| 新增：LQR 教师 | 当前模型 LQR/PD 教师 + 动作适配 | ⏸ **路线暂停**（用户决定不用作教师）；原型 6 s 内 0/4 失败，保留为诊断工具 | `reports/lqr_teacher_eval.json`、`tests/test_lqr_teacher.py` |
| 新增：STEP 结构与轴线 | 产品/实例/变换、圆柱归属、同零件两轴距离、平面机构三点 | 🟡 **G01–G07 复现且带零件名；平面三点已推出**；锚点/固连变体/驱动路径已在 SolidWorks 复核；仍无质量信息 | `docs/STEP_GEOMETRY_AUDIT.md`、`reports/step_geometry_audit.json`、`tools/step_audit_standalone.py` |
| 新增：驱动与弹簧信息（S2b） | 六路电机型号/实装数量/所属装配、气弹簧轴向与销距、轮侧行星减速比 | ✅ **静态提取完成，并已由操作者在 SolidWorks 逐项确认**：4×DM-8009（固定在髋关节大铝件）+ 2×M3508（自带减速器直接驱动轮）；销距 194.950 mm（装配姿态）；链传动 16→19 齿 = 1.1875；Mk4 行星 14.88:1 **但不在轮驱动链上**；旧 4500 N / 195 mm 读法已撤回；平面杆长按垂直铰轴投影为 210 / 250 mm，三维销距（含 11 mm 侧向错位）装配 194.949622、标称 227.760939 mm；动力学尚未接入 | `docs/STEP_GEOMETRY_AUDIT.md` §6.6–§6.10、`configs/gas_spring.yaml: real_spring`、`reports/step_geometry_audit.json` |
| 新增：闭链拓扑（S2c） | 两种固连候选的可行域、A 销承载件 | ✅ **已确认**：A 销固定在大腿板 → `link_a_on_thigh`，961/961 闭合 | `configs/mechanism.yaml`、`reports/mechanism_check_*.json` |
| 新增：闭链 MJCF 骨架（S2c→C） | 七刚体/腿的真实环链、两个 `<connect>` 闭合、六通道输入 | 🟡 **骨架已重新生成并编译通过**：nq 21 / nv 20 / nu 6 / neq 4、两环残差 1.1e-16 m、总质量与串联资产相等；带 URDF 侧向偏移（三维 0.210315 / 平面 0.210000 m）；**无网格/无碰撞/无弹簧/不是训练模型** | `assets/mjcf/fifty_closed_skeleton.xml`、`docs/CLOSED_CHAIN.md`、`tests/test_closed_chain.py` |
| 新增：关节约定与参考姿态（S3） | 六通道基座轴向/符号/零位、参考站立姿态、串联↔闭链映射与 Jacobian | ✅ **静态检查通过**：轴向与 MJCF 一致、参考姿态弦长 0.404751 m 且非奇异 | `configs/joint_conventions.yaml`、`docs/JOINT_CONVENTIONS.md`、`reports/joint_conventions_*.json` |

## 命令与关卡的对应

```text
audit-assets      P0
build-assets      P1（内部顺序：修正资产 → 两个 MJCF → §3.6 校验）
validate-mjcf     P1（只重跑校验）
inspect           P2（模型摘要，可选 —viewer）
spawn-audit       P2（正立/离地/顶部标记）
spring-audit      P3（锚点、长度/Jacobian、力符号、断使能、虚功）
render            固定视角截图（显示/碰撞两组）
joint-test        P4
spring-sweep      P4
contact-test      P5
contract-test     P5
prepare-teacher   P6
evaluate-teacher  P6
smoke             P7
benchmark         P8
train             P8/P9
train-gpu         P9（--resume / --init-from 为 2026-10-29 新增）
evaluate-stand    独立站立验收（2026-10-29 新增，CPU）
evaluate-lqr      LQR 教师门槛（2026-10-29 新增）
step-audit        S2 STEP 装配结构审计（2026-10-29 新增）
mechanism-check   S2c 闭链拓扑、闭合误差与可行域（2026-10-29 新增）
joint-conventions S3 关节符号/零位/轴向与参考姿态（2026-10-30 新增）
export            P10
play              P10
```

未实现的命令以退出码 2 结束，并打印所属关卡。当前**所有命令均已实现**，
`GATED_COMMANDS` 为空；`tests/test_cli.py` 会持续断言这一点，防止再出现
"看起来跑通了但其实没有实现"的路径。

退出码约定：`0` = 关卡通过；`1` = 检查失败**或**只完成了部分范围
（`smoke --backend mjlab` 即为后者：跑了的三项都 PASS，但端到端 GPU 训练未验证，
所以故意返回 1，避免被流水线误读成"GPU 闸门已开"）。

## 本次执行中发现、方案里没有预设的问题

1. **轮电机力矩上限必须单独限幅。** 源资产给轮关节 `effort=30 N·m`，但轮子
   （`I≈3.4e-4 kg·m²`）在这个力矩下 5 ms 速度环数值发散。已按复旦资产改为
   **腿 30 N·m / 轮 5 N·m**（`configs/robot.yaml: joints.effort_nm.wheel`）。
2. **轮关节 armature 缺失会让速度环发散。** 源 URDF 没有 `armature`，5 ms 下
   `Kd=0.2` 的轮速环等效环路增益远大于 2。已给轮关节补 **0.01 kg·m²** 的
   折算转子惯量（**暂定值**，`joints.status.armature: provisional`），
   `audit-assets` 仍把该项列为"缺依据"。
3. **显式 PD 在"钉住且空载"的单关节上不稳定。** `joint-test` 因此把腿通道
   窗口缩到 3 步、轮通道 1 步，并关掉气弹簧（300 N×力臂 ≈ 9 N·m 远大于
   约 1 N·m 的 PD 输出）。详见 `reports/joint_test.json` 的 `note`。
4. **双轮受载必须用"垂直滑块"支撑。** 两轮倒立摆在无平衡控制器时 0.2–0.3 s
   内必然倾倒，钉死机身又会把重量吃在约束上。现做法只钉姿态与水平位置、
   放开 z：销轴无法承担竖向力，故两轮法向力之和必须等于重力。实测
   235.9 N vs 228.2 N（1.034，接触软约束的正常偏差）。
5. **教师不可迁移。** 见 `docs/MIGRATION_STATUS.md` 的教师一节；`train` 已按
   `policy_contract.teacher.distillation_enabled=false` 拒绝挂载教师。

## P3 需要补齐的数据

1. 每支气弹簧的左右编号与实际数量。
2. 两端所在 link（缸体端 / 活塞杆端）与销轴中心在各自 link 局部坐标中的位置，单位 m。
3. 最短 / 最长安装长度与正常站姿长度（区分总长与有效行程）。
4. 推力依据：型号、参考长度下的推力、力—长度曲线、温度/公差资料。
5. 气弹簧与连接件质量是否已并入 CAD 刚体（避免重复计重）。

当前旧串联 MJCF 已有参考 300 N 弹簧，闭链骨架尚无弹簧。真实参数记录不能自动生成新锚点与力模型，须完成集成再重验 P3/P4。力曲线可后补，首轮允许显式 450 N 常力近似。
