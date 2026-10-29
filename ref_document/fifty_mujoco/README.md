# fifty_mujoco

五十版轮腿机器人的 MuJoCo 迁移工程。对应方案文档
`zhongzhuang/轮腿机器人_本机迁移方案2.0注意事项.md` 的 **P0–P10 阶段**。

**当前范围**：源资产审计 → 用"同一物理姿态"重算 visual/质心/惯量 → 生成共享物理参数的
两个 MJCF → 正立初始化与几何验收 → 气弹簧安装几何 → 六通道电机与被动力验收 →
接触与策略契约 → 教师 ONNX 恢复与物理语义检查 → CPU/GPU 后端冒烟 → PPO 训练 →
ONNX 导出与独立回放。

**关卡状态**：P0–P2 ✅、P4–P5 ✅、P8/P10 ✅（训练工程）；
P3 ⚠️ 接口通过但行程未实测、P6 ⚠️ 权重恢复通过但教师不可迁移、P7 ⚠️ 物理批量已上 GPU
但端到端训练未验证；P9（正式训练）未开始。详见 `docs/PHASES.md`。

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
  envs/       cpu_env（单环境）、fifty_env（向量化 + 历史 + 延迟）
  learning/   actor_critic、storage、ppo、runner（三阶段状态机）
  transfer/   teacher（ONNX→PyTorch）、export（ONNX + 对拍）
  evaluation/ render、channel_suite、spring_suite、contact_suite、contract_suite、
              teacher_eval、gpu_smoke、benchmark、playback
  cli.py      子命令入口
reports/      每个关卡的 JSON 报告
runs/         训练输出、export/ ONNX + manifest
tests/        pytest 套件
docs/         MIGRATION_STATUS.md、PHASES.md
```

## 源包只读

`solidwork_graph/fifty_version_with_meshes`、`solidwork_graph/fifty_version_training`、
`ref_document/fudan_rl_wheel_leg-main`、`ref_document/mjlab_upkie-main` 全部只读。
所有修正只发生在本目录。

## 已知未核实项

- 轮半径 0.06 m 只是网格半径，可能缺胎面。
- 关节限位是围绕装配姿态的外接框，不是五连杆真实可行域。
- 关节速度上限 1000 rad/s 是占位值。
- 力矩上限按分组：腿 30 N·m（占位）/ 轮 5 N·m（取自复旦资产）。
- 轮关节 `armature=0.01 kg·m²` 是**没有厂商数据的暂定值**，但没有它轮速环在
  5 ms 下会发散；腿关节 armature 仍为 0。
- base_link 占总质量 91%，惯量未复核。
- 气弹簧按复旦数据设为每侧 300 N（每侧 1 支、共 2 支），**锚点借自复旦、
  行程未实测**，P3 关卡未关闭。
- 车身原先按 G1 记录的基座旋转是倒的，现在用 `base.mesh_correction` 单独修正
  base_link；整条链的规范化推导留待车头方向确认后重做。
- **默认站立姿态下 `Kp=20` 抱不住车身**：维持 `h0` 需约 14/21 N·m，而
  `Kp=20 × 0.5 rad` 只有 10 N·m。策略必须输出 `|a| ≈ 2`（动作不裁剪，
  与复旦 `CLIP_ACTIONS=100` 一致），否则后续应把 `Kp` 提到 150 左右。
- **复旦教师不可迁移到本机**：教师对应 `infantry_V4`（20.68 kg、腿长 0.494 m、
  默认角 ±0.2/±0.4），本项目 23.26 kg、0.460 m、默认角 ±2.055/−2.559，
  关节零位约定不同。`policy_contract.teacher.distillation_enabled=false`。
- **端到端 GPU 训练未验证**：只验证了 MuJoCo Warp 物理批量（133k env-steps/s
  @1024 worlds），PD/观测/奖励仍在 NumPy 里。
