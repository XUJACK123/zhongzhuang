# fifty_mujoco

> **2026-10-31 当前状态**：平面杆长改为**垂直铰轴投影**，恢复 **210 / 250 mm**（三维距离
> 210.315 / 253.947 mm 含侧向安装偏移，不是杆长），末端连杆回到 **96.600 mm**，23.1 mrad 的
> 参考姿态移动**撤回**；气弹簧改用三维销距（含 11 mm 侧向错位）。闭链骨架
> `assets/mjcf/fifty_closed_skeleton.xml` **已重新生成**并编译，两环残差 1.1e-16 m。
> 只跑了不依赖仿真库的检查（`mechanism-check`、`joint-conventions`、`closed-mjcf` 与四个纯
> pytest 模块）；完整 pytest、MuJoCo 动力学、CUDA、训练均未执行。
> 完整启动顺序见 `../../progress.md` §25 —— 其中明确指出：`audit-assets`、`build-assets`、
> `spring-*`、`train*`、`evaluate-*`、`gpu-parity`、`export`、`play` **仍绑定旧串联模型**，
> 必须完成新模型适配后才能用于验收。

五十版轮腿机器人的 MuJoCo 迁移工程。对应方案文档
`zhongzhuang/轮腿机器人_本机迁移方案2.0注意事项.md` 的 **P0–P10 阶段**。

**当前范围**：源资产审计 → 用"同一物理姿态"重算 visual/质心/惯量 → 生成共享物理参数的
两个 MJCF → 正立初始化与几何验收 → 气弹簧安装几何 → 六通道电机与被动力验收 →
接触与策略契约 → 教师 ONNX 恢复与物理语义检查 → CPU/GPU 后端冒烟 → PPO 训练 →
ONNX 导出与独立回放。

**关卡状态**：P0–P2 ✅、P4–P5 ✅、P8/P10 ✅（训练工程）；
P3 ⚠️ 接口通过但行程未实测、P6 ⚠️ 权重恢复通过但教师不可迁移、P7 ⚠️ 物理批量已上 GPU
但端到端训练未验证；P9（正式训练）未开始。详见 `docs/PHASES.md`。

**2026-10-29 补充（S1/S2/S4 软件部分）**：重置/历史/终止/命令/特权特征契约已修复并有
CPU 回归；新增独立评估器 `evaluate-stand`（64 独立初态 × 60 s，逐环境首次失败统计）、
版本化 checkpoint（编号快照 + 原子写 + best 排序 + resume）、STEP 审计（装配结构 +
**圆柱轴线提取**，G01–G07 可复现并带零件名）。GPU 侧同批修复**未在 CUDA 上执行**，
`tests/test_warp_env.py` 在本机全部 skip。

**LQR 教师路线暂停**（2026-10-29 决定）：原型 6 s 内 0/4 失败，且属结构性问题；
`control/lqr_teacher.py` 与 `evaluate-lqr` 保留为诊断工具，不再作为示教/预训练前置。
当前主线改为 STEP 机构建模（`docs/STEP_GEOMETRY_AUDIT.md`）。

**2026-10-30 补充（机构确认与关节约定）**：
- 固连变体已在 SolidWorks 确认（A 销固定在大腿板 → `link_a_on_thigh`），
  闭链现在可实例化，`±0.15 rad` 内 961/961 闭合。
- 新增 `configs/joint_conventions.yaml` + `mechanism/conventions.py`：六个通道的
  基座轴向、左右符号、零位与**参考站立姿态**，全部与 MJCF/策略契约/`robot.yaml` 交叉核对。
  核对过程中纠正了一处：左轮有效轴与右轮相同（+Y），不是腿关节那样的镜像。
- 新增 `mechanism/bridge.py`：等效串联模型 ↔ 真实闭链（曲柄）的映射与 Jacobian，
  过滤 ±π 绕回和可行域边界造成的假根。详见 `docs/JOINT_CONVENTIONS.md`。

**2026-10-31 补充（四项 CAD 事实 + 弹簧数据）**：
- 曲柄角是**地面参考**：两台 DM-8009 都固定在 `【复用交C件】髋关节大铝件`（车体架）上，
  所以 `crank = 电机角`，不带大腿项。
- 轮由 **M3508 自带减速器直接驱动**；Mk4 行星（行星架固定，14.88:1）在 CAD 里但**不在动力路径上**。
- 链传动齿数由齿座几何数出并用标准节距验证：**16 → 19 齿、12.7 mm 链、传动比 1.1875**。
- 气弹簧两端确认（上=大腿板、下=小腿板）。最初的 4500 N / 195 mm 两个读数都已撤回
  （前者是电机上限的 6 倍，后者其实是装配姿态画出来的销距），改按 **450 N / ≈250 mm**，
  这一组与机构自洽。记录在 `configs/gas_spring.yaml: real_spring` 与
  `docs/STEP_GEOMETRY_AUDIT.md` §6.10。

## 快速开始

```bash
uv sync --extra gpu                      # Python 3.12 + mujoco/torch(cu128)/mjlab/warp
uv run python -m fifty_mujoco.cli audit-assets
uv run python -m fifty_mujoco.cli build-assets
uv run python -m fifty_mujoco.cli inspect --backend cpu --viewer        # 静态展示初始姿态
uv run python -m fifty_mujoco.cli inspect --backend cpu --viewer --animate  # 开始物理仿真
uv run python -m fifty_mujoco.cli spawn-audit
uv run python -m fifty_mujoco.cli spring-audit --profile reference_300n
uv run python -m fifty_mujoco.cli joint-test
uv run python -m fifty_mujoco.cli spring-sweep
uv run python -m fifty_mujoco.cli contact-test
uv run python -m fifty_mujoco.cli contract-test
uv run python -m fifty_mujoco.cli render --variant visual --prefix fifty  # 固定视角截图
uv run python -m fifty_mujoco.cli mechanism-check        # 闭链拓扑、闭合与可行性
uv run python -m fifty_mujoco.cli closed-mjcf            # 生成并校验闭链 MJCF 骨架
uv run python -m fifty_mujoco.cli joint-conventions      # 关节符号/零位/轴向 + 参考姿态
uv run python tools/report_index.py                      # 重建 reports/index.json
uv run pytest -q
```

> `--viewer` 默认**不步进物理**，保证看到的是稳定初始姿态；
> `--animate` 会在一秒内倒地（还没有平衡控制器）。

训练与导出：

```bash
uv run python -m fifty_mujoco.cli benchmark
uv run python -m fifty_mujoco.cli train --smoke-updates 2 --num-envs 16
export CKPT=$(ls -td runs/run_* | head -1)/model_last.pt
uv run python -m fifty_mujoco.cli export --checkpoint "$CKPT"
uv run python -m fifty_mujoco.cli play --onnx runs/export/model_last.onnx --seconds 2
uv run python -m fifty_mujoco.cli smoke --backend mjlab
uv run python -m fifty_mujoco.cli doctor --device cuda:0
```

评估与检查（新增）：

```bash
# 独立的 60 s 站立验收：64 个与训练不同的初态，逐环境首次失败统计
uv run python -m fifty_mujoco.cli evaluate-stand --checkpoint runs/demo_run4/model_best.pt \
    --steps 6000 --episodes 64
uv run python -m fifty_mujoco.cli evaluate-stand --onnx runs/export/model_best.onnx
uv run python -m fifty_mujoco.cli evaluate-stand --zero-policy --steps 60 --episodes 2  # 对照

# STEP 装配结构与圆柱轴线审计（不需要 numpy / mujoco / torch）
python tools/step_audit_standalone.py
uv run python -m fifty_mujoco.cli step-audit            # 完整环境下的等价入口
uv run python -m fifty_mujoco.cli step-audit --no-geometry

# LQR 教师门槛（路线已暂停，仅作诊断）
uv run python -m fifty_mujoco.cli evaluate-lqr --steps 600 --episodes 4

# 训练检查点：--resume 继续同一实验，--init-from 只加载权重
uv run python -m fifty_mujoco.cli train-gpu ... --resume runs/<run>/model_last.pt
uv run python -m fifty_mujoco.cli train-gpu ... --init-from runs/<run>/model_best.pt
```

## 目录

```text
configs/      robot.yaml（单一物理参数真源）、spawn.yaml、gas_spring.yaml、
              policy_contract.yaml、teacher_manifest.yaml
assets/       corrected/（修正后 URDF + 网格硬链接）、mjcf/（两个 MJCF）
src/fifty_mujoco/
  assets/     urdf_io、kinematics、stl、audit、build_corrected、colliders、
              build_mjcf、validate_mjcf
  contracts/  关节顺序、符号、维度契约（25 / 125 / 141+8 / 152）
  control/    spawn、gas_spring、spring_audit、mixed_pd
  mdp/        observations、rewards、terminations、commands
  envs/       cpu_env（单环境）、fifty_env（向量化 + 历史 + 延迟）、warp_env（GPU）
  learning/   actor_critic、storage、ppo、runner（三阶段状态机）、
              gpu_ppo（GPU PPO）、checkpoint（编号快照/原子写/best/resume）
  control/    spawn、gas_spring、spring_audit、mixed_pd、
              lqr_teacher（教师原型）、teacher_adapter（教师动作↔关节力矩）
  transfer/   teacher（ONNX→PyTorch）、export（ONNX + 对拍）
  evaluation/ render、channel_suite、spring_suite、contact_suite、contract_suite、
              teacher_eval、gpu_smoke、benchmark、playback、
              stand_eval（独立站立评估）、lqr_suite（教师门槛）
  assets/     …、step_audit（STEP 装配结构审计）
  cli.py      子命令入口
reports/      每个关卡的 JSON 报告
runs/         训练输出、export/ ONNX + manifest
tests/        pytest 套件
docs/         MIGRATION_STATUS.md、PHASES.md、STEP_GEOMETRY_AUDIT.md
```

## 源包只读

`solidwork_graph/fifty_version_with_meshes`、`solidwork_graph/fifty_version_training`、
`ref_document/fudan_rl_wheel_leg-main`、`ref_document/mjlab_upkie-main` 全部只读。
所有修正只发生在本目录。

## 已知未核实项

- 轮半径 0.06 m、轮距、质量与惯量按用户约定沿用，未新增实测证据。
- 关节限位是围绕装配姿态的外接框，不是五连杆真实可行域。
- 关节速度上限 1000 rad/s 是占位值。
- 力矩上限按分组：腿 30 N·m（占位）/ 轮 5 N·m（取自复旦资产）。
- 轮关节 `armature=0.01 kg·m²` 是**没有厂商数据的暂定值**，但没有它轮速环在
  5 ms 下会发散；腿关节 armature 仍为 0。
- base_link 占总质量约 91%；沿用已接受惯量，闭链转换仍需正确分组和坐标变换。
- 气弹簧按复旦数据设为每侧 300 N（每侧 1 支、共 2 支）。**锚点已从 STEP 确认**
  （上=大腿板、下=小腿板，装配姿态销距 194.95 mm），名义力 **450 N**、自由长度 **≈250 mm**、
  行程 75 mm 已记录：三维标称销距 227.760939 mm，已压缩 22.239061 mm、剩余可压缩 52.760939 mm。
  **P3 尚未关闭**：真实锚点、三维力方向与行程检查尚未接入；首轮允许明确标注的 450 N 常力近似，见
  `configs/gas_spring.yaml: real_spring`、`docs/STEP_GEOMETRY_AUDIT.md` §6.10。

**2026-10-31 离线纠正**：保留 URDF 坐标与侧向安装偏移，平面投影杆长为 **210 / 250 mm**；三维原点距离不是平面杆长。末端连杆保持 **96.600 mm**，撤回节点移动与 23.1 mrad 姿态修正结论。旧默认姿态轮心在髋前方 22.865 mm，不是正下方；本轮未改变运行默认角与策略契约。

**闭链骨架尚不能训练**：旧 XML 及其历史编译报告保留，未按这次数值修正重新生成。源码生成器与运动学已共用投影解析入口；完整候选模型还缺地面、碰撞、正确惯量、真实弹簧、轮轴/连续关节修正及 CPU/GPU 环境名称与初始化适配。详见 docs/CLOSED_CHAIN.md。

- 车身原先按 G1 记录的基座旋转是倒的，现在用 `base.mesh_correction` 单独修正
  base_link；整条链的规范化推导留待车头方向确认后重做。
- **默认站立姿态下 `Kp=20` 抱不住车身**：维持 `h0` 需约 14/21 N·m，而
  `Kp=20 × 0.5 rad` 只有 10 N·m。策略必须输出 `|a| ≈ 2`（动作不裁剪，
  与复旦 `CLIP_ACTIONS=100` 一致），否则后续应把 `Kp` 提到 150 左右。
- **复旦教师不可迁移到本机**：教师对应 `infantry_V4`（20.68 kg、腿长 0.494 m、
  默认角 ±0.2/±0.4），本项目 23.26 kg、0.460 m、默认角 ±2.055/−2.559，
  关节零位约定不同。`policy_contract.teacher.distillation_enabled=false`。
- **历史状态已更新**：旧串联模型已有端到端 GPU 训练与 CPU/GPU 对拍记录（progress.md §22）；旧策略 64×60 s 为 0/64。新闭链模型尚未验证，这些旧报告不迁移为新模型通过。
