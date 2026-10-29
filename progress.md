# 轮腿机器人迁移进度

> 更新时间：2026-10-29
> 收口位置：`ref_document/fifty_mujoco/`（本机验证工程）、`solidwork_graph/`（CAD→URDF 资产）
> 本文件只回答"现在到哪了、还差什么、下一步做什么"。证据与细节在 `ref_document/fifty_mujoco/docs/`。

---

## 0. 一句话结论

**机器人已经能在本机 GPU 上被训练到"站住 10.7 秒"，并且策略能导出 ONNX 在普通 MuJoCo 里独立回放；但离真机还差两件硬骨头 —— 五连杆并联映射（P11）和气弹簧安装几何核实（P3）。**

- ✅ 模型能跑：质量/惯量/关节/接触/PD/契约全部逐项对拍通过
- ✅ 训练能跑：8192 并行世界，305k env-steps/s，10 分钟出站立策略
- ✅ 部署链能跑：train → export ONNX（误差 9.5e-7）→ play
- ❌ 真机还上不了：当前模型是**串联等效链**，真机是**五连杆并联**

---

## 1. 文档导航（先看哪个）

| 想了解 | 看这个 |
|---|---|
| 整体文件关系、模型修复记录、部署数据定义 | `轮腿机器人_文件关系与训练部署实施说明.md` |
| 本机迁移的分阶段方案 A–F 与放行标准 | `轮腿机器人_本机MuJoCo迁移方案.md` |
| 2.0 版补充注意事项 | `轮腿机器人_本机迁移方案2.0注意事项.md` |
| 关卡 P0–P10 的逐项状态与证据 | `ref_document/fifty_mujoco/docs/PHASES.md` |
| 实测数据、踩过的坑、训练崩塌复盘 | `ref_document/fifty_mujoco/docs/MIGRATION_STATUS.md` |
| 怎么敲命令 | `ref_document/fifty_mujoco/README.md` |
| **进度全景（本文件）** | `progress.md` |

`ref_document/` 下的 `chassis_of_hero/`、`chassis_ref_new/`、`chassis_ref_origin/`、
`fudan_rl_wheel_leg-main/`、`mjlab_upkie-main/` 等均为**只读参考源**，不修改。

---

## 2. 关卡进度

验证级别严格按方案 §9.2 六档，不跨级标记。

| 关卡 | 内容 | 状态 | 证据 |
|---|---|---|---|
| P0 | 来源冻结：URDF、7 个 STL 哈希、关节树、缺陷复现 | ✅ 静态检查通过 5/5 | `reports/assets_audit.json` |
| P1 | 坐标与物理资产：visual/COM/惯量重算、碰撞重生成、双 MJCF | ✅ 静态检查通过 18/18 | `reports/model_validation.json` |
| P2 | 正立初始化：根四元数、轮底离地、无穿地 | ✅ CPU 仿真通过 8/8 | `reports/spawn_report.json` |
| P3 | 气弹簧安装几何与力模型 | ⚠️ **接口 9/9，关卡未关闭** | 锚点借自复旦，**行程未实测** |
| P4 | 电机通道与被动力组合 | ✅ CPU 仿真通过 12 + 3 | `reports/joint_test.json`、`spring_sweep.json` |
| P5 | 接触与策略契约（25/125/149、PD、历史、terminal） | ✅ CPU 仿真通过 4 + 12 | `contact_test.json`、`contract_test.json` |
| P6 | 教师复现 | ⚠️ **权重恢复通过，迁移失败** | 见 §5 |
| P7 | GPU 后端 | ⚠️ **物理批量通过，端到端未验** | `reports/gpu_smoke.json` |
| P8 | 训练冒烟 | ✅ 训练工程通过 | `runs/*`、`tests/test_learning_pipeline.py` |
| P9 | 正式迁移训练 | 🟡 **自研 GPU PPO 已跑通，mjlab 路线未走** | `runs/stand_3h`、`runs/demo_run4` |
| P10 | 导出与回放 | ✅ 训练工程通过 | `runs/export/model_best.onnx` |
| **P11** | **五连杆映射（电机角 ↔ 虚拟关节角 + 雅可比）** | ❌ **未开始** | 需要 4 个连杆长度 + 曲柄相位 |

> P3、P7、P9、P11 是当前唯一挡住真机的四项。

---

## 3. 机器人关键参数（已核实）

| 项目 | 值 |
|---|---|
| 连杆数 / 关节数 | 7 links / 6 joints（串行等效） |
| 质量 | 23.2596305 kg |
| MuJoCo 自由度 | nq=13, nv=12, nu=8, ntendon=2, neq=0 |
| 关节顺序 | rf0, rf1, r_wheel, lf0, lf1, l_wheel |
| 默认角 | rf0 +2.0554215512, rf1 −2.5592683580, lf0 −2.0554215512, lf1 +2.5592683580, 轮 0 |
| 出生高度 | z = 0.474751372 m；标称站立高 h0 = 0.464751 m |
| 腿 PD | Kp = 20, Kd = 1 |
| 轮 PD | Kp = 0（速度环）, Kd = 0.2 |
| **力矩上限** | **腿 30 N·m / 轮 5 N·m（每关节，必须分开限幅）** |
| 轮 armature | 0.01 kg·m²（**暂定**，缺依据） |
| 气弹簧 | 2 根，恒力 300 N，`SPRING_CTRL_SIGN = +1.0` |
| 物理步长 / 策略周期 | 5 ms / 10 ms（decimation 2） |
| 动作缩放 | 腿 ±0.5 rad，轮 ±10 rad/s |

**策略契约维度**：obs 25 / history 125（5×25，最旧在前）/ privileged 149（141+8）/ critic 输入 152 / actions 6。

> ⚠️ `Kp = 20` 下静态站立需要 13.9 / 21.3 N·m，而腿只有 30 N·m、轮只有 5 N·m —— 策略必须输出 |a| ≈ 1.4–2.1 才撑得住。试过 `Kp = 150`，站立时长从 0.90 s 掉到 0.20 s，**已否决**，保留复旦兼容的 Kp = 20。

---

## 4. 训练吞吐与结果

### 4.1 GPU 吞吐（RTX 5060 Laptop 8 GB，MuJoCo Warp + torch + CUDA Graph）

| 配置 | env-steps/s | 每 update | rollout | PPO 更新 | Python 迭代数 |
|---|---|---|---|---|---|
| 2048 × 24, mb4 ep5 | 54 979 | 894 ms | 246 ms | 624 ms | 20 |
| 8192 × 24, mb4 ep5 | 206 409 | 952 ms | 374 ms | 555 ms | 20 |
| 8192 × 48, mb2 ep3 | 305 653 | 1287 ms | 795 ms | 447 ms | 6 |
| 8192 × 96, mb2 ep3 | 344 584 | 2282 ms | 1541 ms | 652 ms | 6 |
| 16384 × 48, mb2 ep3 | 368 146 | 2136 ms | 1387 ms | 710 ms | 6 |

**核心结论：瓶颈是 Python 迭代次数（`epochs × minibatches`），不是 GPU 算力。**
把 20 次迭代压到 6 次，吞吐 55k → 305k，GPU 利用率 9% → 95%。

### 4.2 已跑通的训练

| 运行 | 配置 | 结果 |
|---|---|---|
| `runs/stand_3h` | 8192×96×3ep，3 小时 | update 600–2300 学到站立（20 s 上限），之后**自己毁掉**（翻车） |
| `runs/demo_run4` | 同上 + best-checkpoint + early-stop | **第 600 次 update 达标即早停，存住站立策略** ✅ |
| `runs/gpu_10min_eval` | 默认 2048×24，10 分钟 | 前 700 次 update 全 0 s，第 750 次跳到评估窗满（3.00 s），高度仅 0.363 m |

### 4.3 交付策略实测（`runs/demo_run4/model_best.pt`）

| 指标 | 值 |
|---|---|
| 站立时长 | **10.74 s**（0–9 s 干净直立） |
| 终止原因 | `illegal_contact`（腿/机身擦地） |
| 终止时高度 / up·Z | 0.360 m / +0.834 |
| 前向漂移 | 0.209 m/s（指令 vx = 0）⚠️ |
| ONNX 导出误差 | 9.537e-07 |
| ONNX 独立回放 | 10.00 s / 10.00 s，末态高 0.4609 m |

产物：`reports/demo_stand.mp4`（11.9 s，358 帧）、`reports/demo_frames.png`。

**两个待修缺陷**：① 0.21 m/s 前向漂移；② 10 s 后高度缓慢下沉导致腿擦地。都不是物理/契约问题，是奖励项要调。

### 4.4 3 小时训练崩塌复盘（重要经验）

update 600–2300 表现完美（episode 20 s 满、return +28、up·Z = 1.000），之后塌到 0.20 s、up·Z = −0.07。三个原因叠加：

1. **adaptive-LR 被推到地板**：262k 样本的 batch 让 KL 只有 0.001–0.02，远小于 `0.5 × desired_kl`，于是每轮 ×1.5，一路降到 `min_lr = 2e-5`。
2. **entropy 奖励把 σ 吹大**：`entropy_coef = 0.005` 让可学习 σ 从 ~0.1 涨到 `max_std = 1.0`，动作噪声 ±0.5 rad（腿）/ ±10 rad/s（轮），均值策略被拖向翻车。
3. **没有 best checkpoint**：`save_every = 100` 只覆盖 `model_last.pt`，好策略被覆盖。

**单独的修法都无效**（关 adaptive-LR 或降 entropy，训练就卡在 0.5–0.9 s，demo_run 1/2/3 三次验证）。
**有效修法**：保留这套组合，额外加 `--stop-on-survive 1 --save-every 25`，用早停把峰值扣下来。

> 教训：`episode length` 衡量的是"噪声容忍度"，不是"技能"。必须看**确定性评估**下的站立时长。

---

## 5. 已知未关闭的问题

按"挡住真机的严重度"排序。

### 5.1 【P11】真机是五连杆并联，模型是串联等效链 ❌

- 当前 MJCF：nq=13 / nv=12 / nu=8 / neq=0，7 个连杆
- 真机：五连杆闭链，电机角与虚拟关节角之间是非线性映射
- 训练出的力矩是**虚拟关节力矩**，必须经雅可比映射到实际电机
- **需要你提供**：4 个连杆长度 + 曲柄相位角（CAD 里量或上台架量）
- 迁移方案第 44 行明确这是分阶段策略，当前阶段只导出过 7 个 STL（base_link、l0_link、l1_link、left_wheel、r0_link、r1_link、right_wheel），**推杆/摇臂零件从未导出**

### 5.2 【P3】气弹簧行程未实测 ⚠️

- 两个锚点坐标是从复旦资产借来的，没有按本机 CAD 核实
- `length_min / length_max` 未实测 → 力-位移曲线在行程端点是外推的
- **风险**：如果真机弹簧装在推杆/摇臂上而不是大腿上，当前 300 N 的力臂就是错的，P3 要重开，前面所有训练结论在真机上都可能不成立

### 5.3 【P7】端到端 GPU 闸门未开 ⚠️

- MuJoCo Warp 物理批量已验（133 913 env-steps/s @1024 worlds）
- 但走的是**自研** `WarpEnv` + `GpuPpoTrainer`，没有构建 mjlab 的 `ManagerBasedRlEnv`
- `smoke --backend mjlab` 因此故意返回退出码 1

### 5.4 【P6】复旦教师不可迁移 ❌

- 权重能精确恢复（14 个张量，最大误差 1.14e-5）
- 但部署时存活 0.30–0.49 s，**比零动作基线（0.29 s）还差**
- 根因：复旦部署资产 nq=21 / nv=20 / nu=10 / neq=4（五连杆闭链，16 个 body），与我们的 nq=13 / nv=12 / nu=8 / neq=0 拓扑不同；腿长 0.494 vs 0.460 m、质量 21.06 vs 23.26 kg、默认角也不同
- 通道顺序已逐项核对，与复旦源码完全一致；试过左右腿置换，没用
- 结论：`configs/policy_contract.yaml: teacher.distillation_enabled = false`

### 5.5 轮关节 armature 是暂定值 ⚠️

- 源 URDF 没有 `armature`，5 ms 步长下轮速环会发散
- 补了 0.01 kg·m² 的折算转子惯量，标为 `provisional`
- 需要真机电机转子惯量 × 减速比² 才能定死

### 5.6 电机侧待确认

- 减速比未知
- **连续力矩未知** —— 站立需要 ~21 N·m，如果电机连续力矩不够，策略再好也站不住

---

## 6. 下一步（按性价比排序）

| 序 | 任务 | 前置条件 | 预估 |
|---|---|---|---|
| 1 | 拧掉前漂 + 下沉：加 vx 误差惩罚到 0，`base_height` 从软目标改死盯 nominal，重跑 15 min | 无 | 2 h |
| 2 | 命令跟踪套件：stand / 前进 +0.3 / 后退 −0.3 / 转向 ±0.3，各 10 s → `reports/policy_eval.json` | 任务 1 | 3 h |
| 3 | 【P11】五连杆映射 `five_bar.py`：FK / IK / 雅可比 | **需要 4 个连杆长度 + 曲柄相位角** | 1–2 天 |
| 4 | 【P3】核实气弹簧两个锚点挂在哪 | **需要你看 CAD 或上台架** | 半天 |
| 5 | 域随机化：摩擦 0.6–1.4、质量 ±10%、动作延迟 0–10 ms、初始扰动 | 任务 2 | 1 天 |
| 6 | 真机：电机角映射 + 雅可比力矩分配 + CubeAI 固件 | 任务 3、4，**并确认电机减速比与连续力矩** | 1 周+ |

**任务 1 和 2 我可以直接做；任务 3、4 卡在缺数据上。**

---

## 7. 复现命令

工程根目录：`ref_document/fifty_mujoco/`，依赖用 `uv` 管理。

```bash
# 关卡自检
uv run python -m fifty_mujoco.cli audit-assets        # P0
uv run python -m fifty_mujoco.cli build-assets        # P1
uv run python -m fifty_mujoco.cli spawn-audit         # P2
uv run python -m fifty_mujoco.cli spring-audit        # P3（接口）
uv run python -m fifty_mujoco.cli joint-test          # P4
uv run python -m fifty_mujoco.cli contact-test        # P5
uv run python -m fifty_mujoco.cli contract-test       # P5
uv run python -m fifty_mujoco.cli prepare-teacher     # P6
uv run python -m fifty_mujoco.cli smoke               # P7
uv run pytest -q                                      # 94 项

# GPU 训练（推荐配置，约 305k env-steps/s）
uv run python -m fifty_mujoco.cli train-gpu \
  --minutes 15 --num-envs 8192 --steps 48 --minibatches 2 --epochs 3 \
  --lr 1.5e-3 --entropy-coef 0.005 --max-std 1.0 \
  --eval-every 50 --eval-steps 600 --save-every 25 --stop-on-survive 1 \
  --log-every 25 --plot reports/train.png --name my_run

# Demo / 导出 / 回放
uv run python tools/demo_policy.py --checkpoint runs/<name>/model_best.pt \
  --seconds 20 --out reports/demo_frames.png --video reports/demo.mp4
uv run python -m fifty_mujoco.cli export --checkpoint runs/<name>/model_best.pt
uv run python -m fifty_mujoco.cli play --onnx runs/export/model_best.onnx --seconds 10
```

---

## 8. 训练运行索引（`ref_document/fifty_mujoco/runs/`）

| 目录 | 说明 |
|---|---|
| `stand_3h` | 3 小时长跑，学到站立又自己毁掉（**反面教材**，见 §4.4） |
| `demo_run4` | 同一组超参 + 早停，**当前唯一可用的站立策略** |
| `demo_run` / `demo_run2` / `demo_run3` | 三次失败的调参尝试（关 adaptive-LR 或降 entropy 都卡在 0.5–0.9 s） |
| `gpu_10min_eval` | 默认配置 10 分钟基线 |
| `gpu_5min` / `gpu_demo` / `gpu_kp150_10min` | 早期吞吐/Kp 实验 |
| `export` | ONNX 与 manifest |

---

## 9. 环境

Ubuntu 22.04.5 / RTX 5060 Laptop 8151 MiB / driver 595.99.02
Python 3.12.13 · uv 0.12.1 · mujoco 3.17.0 · torch 2.11.0+cu128 ·
warp-lang 1.12.0 · mujoco_warp 3.8.1 · mjlab 1.3.0 · onnxruntime 1.31.1
