# 迁移状态（2026-10-28）

## 当前验证级别

**静态检查通过 + CPU 仿真通过（仅几何、正立与接触前状态）**

不能标记为 GPU 仿真通过、训练工程通过、策略评估通过或实机验证通过：本阶段没有
观测/奖励/PPO/教师代码，也没有在 GPU 上运行任何东西。

## 环境

| 项目 | 值 |
|---|---|
| 系统 | Ubuntu 22.04.5 LTS |
| GPU | NVIDIA GeForce RTX 5060 Laptop GPU，8151 MiB |
| 驱动 | 595.99.02 |
| Python | 3.12.13（uv 管理的独立解释器） |
| numpy | 2.5.4 |
| mujoco | 3.17.0 |
| pyyaml | 6.0.3 |
| 包管理器 | uv 0.12.1 |

GPU 依赖（独立的 `gpu` extra）已安装并通过 `doctor` 验证：

| 包 | 版本 | 验证 |
|---|---|---|
| torch | 2.11.0+cu128 | `cuda.is_available() = True`，GPU 张量运算通过 |
| warp-lang | 1.12.0 | 识别 `cuda:0` RTX 5060 Laptop，`sm_120`，CUDA Toolkit 12.9 |
| mjlab | 1.3.0 | 可导入 |
| onnxruntime | 1.31.1 | 已安装（P6/P10 使用） |

结果见 `reports/doctor.json`。**这只能证明张量运算与设备识别可用**；还没有任何
MuJoCo Warp 批量环境，因此不能标记为 GPU 仿真通过。它也不参与 P0–P2 的放行判定。

## P0 资产审计（`audit-assets`）

| 检查 | 结果 |
|---|---|
| 源 URDF SHA-256 与 `model_parameters.json` 记录一致 | 通过 `63d0936a…3b71` |
| 7 个 STL 的哈希与三角面数与记录一致 | 通过 |
| 关节原点对齐（源 q=0 与训练 q=默认角） | 最大误差 `1.78e-15 m` |
| 关节轴平行（允许 ±z 符号差） | 最大偏差 `0` |

### 车身倒置：根因与修正（2026-10-28 追加）

用户目视反馈“车身是倒的”。离线渲染证实了两个事实：

1. 把 `fifty_version_with_meshes` 按源坐标、`q=0`、世界 +Z 向上渲染，**车身是正的**
   （云台在上、履带在下），但**两条腿是向上收起的运输姿态**
   （轮轴在源坐标 z=0.5088，髋轴 z=0.104）。
2. `model_parameters.json` 记录的基座旋转是 `Rz(90°)·Rx(180°)`，那个额外的 180° 翻滚
   把整体翻了过来：腿被放下（这是对的），但**车身也被翻成倒立**。

由于腿的关节系与基座 link 绑在一起，直接改基座旋转会同时带动腿的网格。
最终处理：`configs/robot.yaml` 新增 `base.mesh_correction.base_link = Rx(180°)`，
**只作用于 base_link 的网格与惯量**，腿与其他 link 完全不变。校正后：

- 车身上方 = 训练 +Z（`dot = 1.0`），车头 = 腿的前进方向 +X；
- 髋关节安装点位于校正旋转轴上，位置不变，因此腿仍精确对齐；
- 基座顶部标记由 z≈0.650 m 升到 z≈0.746 m，仍高于轮轴 0.070 m。

代价（已记录为待办）：base_link 的 link 系相对于网格不再“规范”，
等真实车头方向确认后应重新推导整条链（见 `docs/PHASES.md`）。

复现出的已知缺陷（P1 必须消除）：

| link | 训练 URDF 与重算值的 visual 偏差 |
|---|---:|
| `right_wheel` / `left_wheel` | `0.242832 m`（≈ 文档记录的 `[-0.2386, 0.0452, 0]`） |
| `r1_link` / `l1_link` | `0.359570 m` |
| `base_link` / `r0_link` / `l0_link` | `0` |
| 全部 link | `inertial/origin/rpy` 非零 → 惯量被二次旋转 |

## P1 修正资产与 MJCF（`build-assets`）

做法：对每个 link 只做一次刚体变换

$$T_{\text{new}\leftarrow\text{old}}=T_{\text{new}}(q_{\text{default}})^{-1}\;T_{\text{base}\leftarrow\text{source}}\;T_{\text{old}}(0)$$

基座变换的平移是 $-R\,p_{\text{origin}}$（记录值是**源坐标中**新基座原点的位置）。
把它当成 `+p_origin` 会让所有 link 平移两倍该向量——本轮实现中曾出现，已修正并写入测试。

| 放行项（§3.6） | 结果 |
|---|---|
| 网格点 / 质心 / 惯量在同一装配姿态下自洽 | 通过（visual origin 由变换直接生成并回验） |
| 轮外观、碰撞圆柱、旋转轴对齐 | 通过，三者偏移 `0`，轴与 link z 点积 `1.0` |
| ≥200 组合法姿态 URDF↔MJCF 正运动学对拍 | 通过，最大位置误差 `2.3e-16 m`（要求 ≤1e-6） |
| 惯量正定 + 主惯量三角不等式 | 通过（逐 link） |
| 默认姿态无穿地 | 通过，轮底 `0.010 m`，`ncon = 0` |
| 可视版本与训练版本物理参数一致 | 通过，质量/质心/惯量/关节逐项相同 |
| 源 STL 哈希不变 | 通过 |
| 气弹簧 | **已生成** 2 支，见下节（P3 接口通过，关卡未关闭） |

关键数值：

```text
nq = 13, nv = 12, nu = 8 (6 关节电机 + 2 气弹簧 tendon motor), nbody = 8
总质量            = 23.259630516 kg
生成高度          = 0.474751372 m   （与文档记录一致）
轮轴高度（世界）  = 0.070 m         = 轮半径 0.06 + 余量 0.01
最低机器人点      = 0.010 m
```

## 被动气弹簧（Fudan 参考数据，`reference_300n`）

用户确认每侧一支气弹簧，数据按复旦模型设置：

| 项目 | 值 | 来源 |
|---|---|---|
| 数量 | 每侧 1 支，共 2 支 | 用户确认 |
| 推力 | 恒定 300 N/支 | 复旦 `d.ctrl = 300`（`onnx_mj_binglian.py:106-110`） |
| 安装点 | 髋关节原点前 15.5 mm、内 8 mm、下 44.5 mm；膝关节原点前 5 mm、内 5 mm、下 46 mm | 复旦 site `*_qitanhuang`（`mjmodel.xml:114-121/133-140`） |
| tendon | `<spatial>` 连接两端 site | 复旦 `Left/Right_loop1_tendon` |
| 施力通道 | `<motor tendon=... gear=1>` | 复旦 `*_loop1_motor` |
| 补偿 | 关闭（未复制复旦 `300×1.23` 解析补偿） | 方案 §11.3/§12.6 |
| 行程 | **未知** | 待实测 |

物理约定经实测确定：MuJoCo tendon motor 的**正 ctrl 会把两端推开**
（`qfrc_actuator = J_l^T·force`）。零力矩 1 s 自由仿真验证：
`+300 N` 时膝关节从 −2.559 rad 伸到限位 −1.854 rad 且机身抬高，
`−300 N` 时直接塌陷 —— 与复旦 `ctrl=+300` 的用法一致。

`spring-audit` 的 9 项接口检查全部通过：
锚点所在 body、长度与 tendon 长度、Jacobian 与有限差分、tendon 速度、
力的符号、无净外力、电机断使能仍有被动力、左右独立、虚功一致性。

默认姿态：弹簧长度 0.21656 m（右）/ 0.21658 m（左），各 300 N。

**关卡仍未关闭**：锚点是从复旦按“相对关节的偏移”转移过来的，不是本机实测；
行程上下限为空。理由与待补数据见 `docs/PHASES.md`。

## P2 正立与初始化（`spawn-audit`）

| 检查 | 结果 |
|---|---|
| 根四元数 `[1,0,0,0]` wxyz 且模长误差 ≤1e-8 | 通过 |
| 基座 +Z 与世界 +Z 点积 ≥0.999 | 通过（=1.0） |
| 投影重力 ≈ `[0,0,-1]` | 通过（误差 <1e-6） |
| 顶部标记高于轮轴 | 通过（标记 z≈0.650 m，轮轴 0.070 m） |
| 左右轮离地 0.010 m | 通过 |
| 左右轮等高 | 通过（差 <1e-9 m） |
| 源→训练旋只应用一次 | 通过（根四元数即单位四元数） |

顶部标记取自校正后 `base_link` 网格在基座系中的最高 CAD 顶点，不是任意 +Z 点。

## 本轮实现的文件

```text
src/fifty_mujoco/
  math3d.py            齐次变换、rpy/四元数、惯量校验
  paths.py             路径解析（FIFTY_WORKSPACE 可覆盖）
  config.py            配置加载、基座变换、气弹簧可用性判定
  cli.py               audit-assets / build-assets / validate-mjcf / inspect / spawn-audit / doctor + 桩
  assets/urdf_io.py    URDF 读写
  assets/kinematics.py 完整关节树正运动学
  assets/stl.py        二进制 STL 扫描、AABB、哈希
  assets/mesh_export.py STL→OBJ（MuJoCo STL 解码器上限 20 万面）
  assets/audit.py      P0 审计
  assets/build_corrected.py P1 单次变换重建
  assets/colliders.py  碰撞基本体重生成
  assets/build_mjcf.py 两个 MJCF 生成器
  assets/validate_mjcf.py §3.6 放行检查
  contracts/names.py   顺序/符号/维度契约
  control/spawn.py     SpawnSpec、高度求解、正立审计
  envs/cpu_env.py      单环境加载、名称映射、reset、step
tests/                 60 个测试，全部通过
```

## 已知限制（不要当成已解决）

1. **气弹簧完全不存在于物理模型中。** `configs/gas_spring.yaml` 的锚点、行程、
   推力全为 `null`，MJCF 里没有 site/tendon/motor，`ntendon = 0`。
   P3/P4 关卡保持未通过；缺数据清单见 `docs/PHASES.md`。
2. **视觉网格使用 OBJ 派生文件。** MuJoCo 的 STL 解码器拒绝超过 200 000 面的文件，
   而 `base_link.STL` 有 1 020 702 面。STL 原文件未改动，哈希仍在校验；OBJ 由
   `assets/mesh_export.py` 以 1e-6 m 精度生成。
3. **机器人自碰撞关闭。** 机身的 AABB 盒在几何上包住了大腿与小腿，第一版按方案
   §3.4 关闭自碰撞（`contype=2, conaffinity=1`），只保留与地面的接触。
   碰撞体仍是粗近似：机身一个盒、腿一根圆柱、轮一个圆柱。
4. **全部物理参数仍是 provisional**：轮半径 0.06 m 来自网格、
   速度上限 1000 rad/s、关节限位是外接框、base_link 惯量未复核。
   力矩上限已按关节分组：**腿 30 N·m / 轮 5 N·m**（轮值取复旦资产，
   因为 30 N·m 在 5 ms 步长下会让轮速环数值发散）。
5. **`body_mass` 是 float32。** 比较质量时使用相对容差，不能要求逐位相等。
6. **轮关节 armature 为暂定值 0.01 kg·m²。** 源 URDF 完全没有 `armature`，
   缺少折算转子惯量时 `Kd=0.2` 的轮速环在 5 ms 下等效环路增益远大于 2，
   实测会发散发。该值不是厂商数据，`audit-assets` 仍把
   `rotor_armature` 列为"缺依据"。
7. **未做人工目视检查。** 本会话无桌面环境，`inspect --viewer` 未运行；
   需要在有显示器的终端里执行并保存截图（方案 §10.6 要求固定视角截图）。

## 复现命令

```bash
cd ref_document/fifty_mujoco
uv sync --extra gpu
uv run python -m fifty_mujoco.cli audit-assets
uv run python -m fifty_mujoco.cli build-assets
uv run python -m fifty_mujoco.cli spawn-audit
uv run python -m fifty_mujoco.cli spring-audit
uv run python -m fifty_mujoco.cli joint-test
uv run python -m fifty_mujoco.cli spring-sweep
uv run python -m fifty_mujoco.cli contact-test
uv run python -m fifty_mujoco.cli contract-test
uv run python -m fifty_mujoco.cli prepare-teacher
uv run python -m fifty_mujoco.cli evaluate-teacher
uv run python -m fifty_mujoco.cli benchmark
uv run python -m fifty_mujoco.cli smoke --backend mjlab
uv run python -m fifty_mujoco.cli train --smoke-updates 2 --num-envs 16
uv run python -m fifty_mujoco.cli export --checkpoint runs/<run>/model_last.pt
uv run python -m fifty_mujoco.cli play --onnx runs/export/model_last.onnx --seconds 2
uv run pytest
```

报告输出：`reports/assets_audit.json`、`reports/model_validation.json`、
`reports/spawn_report.json`、`reports/spring_report.json`、
`reports/joint_test.json`、`reports/spring_sweep.json`、
`reports/contact_test.json`、`reports/contract_test.json`、
`reports/teacher_report.json`、`reports/teacher_eval.json`、
`reports/benchmark.json`、`reports/gpu_smoke.json`、`reports/run_manifest.json`。

## P4–P10 实测结果

### P4 电机通道（`joint-test`，12/12）

| 通道 | 指令 | 实测 | 峰值力矩 |
|---|---|---|---|
| `rf0_Joint` | ±0.050 | ±0.0028 rad | 0.50 N·m |
| `rf1_Joint` | ±0.050 | ±0.0055 rad | 0.50 N·m |
| `r_wheel_Joint` | ±1.000 | ±0.967 rad/s | 2.00 N·m |
| `lf0_Joint` | ±0.050 | ±0.0029 rad | 0.50 N·m |
| `lf1_Joint` | ±0.050 | ±0.0055 rad | 0.50 N·m |
| `l_wheel_Joint` | ±1.000 | ±0.967 rad/s | 2.00 N·m |

轮通道现在能跟到目标值的 97%（补上 armature 之前只有 0.5% 并且振荡）。

### P4 被动力组合（`spring-sweep`，3/3）

| 组合 | 电机 | 气弹簧 | 结果 |
|---|---|---|---|
| A | 关 | 关 | 机身从 0.475 m 塔到 0.224 m（腿完全折叠） |
| B | 关 | 开 | 稳定在 0.236 m，弹簧提供被动支撑 |
| C | 开 | 关 | 需要 τ_rms 22.87 N·m，44.58% 时间饱和 |
| D | 开 | 开 | 稳定在 0.342 m，所需力矩明显下降 |

结论：气弹簧确实抬高了能级，但 **`Kp=20` 的气缸根本抱不住默认站立姿态**：
在垂直滑块约束下，维持 `h0` 需要约 14 N·m（髀）/21 N·m（膝），而
`Kp=20 × 0.5 rad` 最多只有 10 N·m。所以策略必须输出 `|a| ≈ 2`
（动作不裁剪，与复旦 `CLIP_ACTIONS=100` 一致），或者后续把 `Kp` 提到 150。

### P5 接触（`contact-test`，4/4）

| 检查 | 结果 |
|---|---|
| 双轮受载 | 235.9 N vs 重力 228.2 N（比例 1.034） |
| 自由落体 | 从 spawn+0.3 m 落下，接触力有限、状态有限 |
| 正反滚动 | +2 → +1.66 rad/s，0 → +0.32，−2 → −1.03（符号与排序均正确） |
| 倒地判定 | `illegal_contacts` 捕获 `base_link, r0_link, r1_link` |

滚动只测量前 0.075 s：两轮倒立摆在无平衡控制器时 0.2–0.3 s 必然倾倒，
之后关节速度被机身俯仰主导，不再是滚动信号。

### P6 教师（`prepare-teacher` 通过、`evaluate-teacher` 失败）

`prepare-teacher`：ONNX → PyTorch 按形状贪心匹配 14 个张量，
`max abs err 1.14e-05`、`max rel err 1.01e-03`。

`evaluate-teacher`（目标模型、10 s、5 个指令）：存活 0.30–0.54 s，全部失败。
零动作基线也是 0.29 s，教师只多活了 0.2 s。排查结论：

1. **不是通道顺序问题。** 复旦训练代码
   `compute_proprioception_observations` 拼出的正是
   `[ang_vel*0.25, projected_gravity, cmd*[3,0.25,5],
   (q[[0,1,3,4]]-default)*1, qd*0.05, actions]` = 25 维，
   与本项目逐通道一致 → 该悬置项已关闭
   （`policy_contract.obs.channel_order_verified: true`）。
2. **不是单纯左右置换。** 复旦 DOF 顺序是左腿在前
   `[lf0,lf1,l_wheel,rf0,rf1,r_wheel]`，本项目是右腿在前；
   加上动作/观测四个置换组合后存活仍是 0.24–0.49 s。
3. **是机器人本身不同。** `actor/yuntai` 教师对应复旦
   `infantry_V4`：总质量 20.68 kg、髀到轮 0.494 m、`effort=50`、
   `default_dof_pos=[0.2,0.4,0,-0.2,-0.4,0]`；
   本项目 23.26 kg、0.460 m、`effort=30`、默认角
   `[2.055,-2.559,0,-2.055,2.559,0]`。
   关节零位约定不同 ⇒ `q* = default + 0.5a` 的动作语义不同，
   无法在不做角度标定的前提下直接迁移。

因此 `policy_contract.teacher.distillation_enabled: false`，
`train` 会拒绝挂载教师并打印原因。蒸馏链路代码保留（`Ppo.set_teacher` 等
和单测仍然覆盖），但 P9 只能走“从零 PPO”路线。

### P7 GPU（`smoke`，partial）

| 检查 | 结果 |
|---|---|
| CPU 批量 | 1844 env-steps/s @16 环境 |
| `MjSpec` 编译 | 8 bodies / 6 joints / 8 actuators |
| **MuJoCo Warp 物理批量** | **133 913 env-steps/s @1024 worlds**（RTX 5060, sm_120） |

但 PD、观测、奖励仍跑在 NumPy 里，也没有构建 `ManagerBasedRlEnv`，
**端到端 GPU 训练未验证，GPU 闸门仍未打开**。要真开 GPU 训练，下一步是
把 `FiftyVecEnv` 的控制/观测层搬到 warp 数组上（或直接用 mjlab 的
`ManagerBasedRlEnv` 重建）。

### P8/P10 训练链路（训练工程通过）

- `benchmark`：1/16/64 环境 = 1883 / 1844 / 1179 env-steps/s（纯 Python 循环）。
- `train --smoke-updates 2 --num-envs 16`：两次完整 PPO 更新，4.2 s，
  正确停在 `INDEPENDENT_PPO` 阶段，产出 `model_last.pt`。
- `export`：opset 18，`max abs err 6.7e-08`，生成 manifest。
- `play`：ONNX 在普通 MuJoCo 里独立回放（未训练策略 0.30 s 倒地，符合预期）。
- `pytest`：全绿，新增 `tests/test_learning_pipeline.py` 覆盖 PD 分组限幅、
  弹簧补偿顺序、25 维通道切片、特权维数、历史顺序、奖励裁剪、
  rollout 形状、PPO 更新真的改了参数、checkpoint 往返、ONNX 往返、
  垂直滑块只钉姿态。

## GPU 端到端训练（2026-10-29 追加）

`GpuPpoTrainer` + `WarpEnv`：MuJoCo Warp 物理 + torch 批量控制/观测/奖励，
整步进 CUDA Graph 捕获。实测吞吐（RTX 5060 Laptop, 8151 MiB）：

| 配置 | env-steps/s | 每 update | rollout | PPO upd | Python 迭代数 |
|---|---|---|---|---|---|
| 2048×24 mb4 ep5 | 54 979 | 894 ms | 246 ms | 624 ms | 20 |
| 8192×24 mb4 ep5 | 206 409 | 952 ms | 374 ms | 555 ms | 20 |
| 8192×48 mb2 ep3 | 305 653 | 1287 ms | 795 ms | 447 ms | 6 |
| 8192×96 mb2 ep3 | 344 584 | 2282 ms | 1541 ms | 652 ms | 6 |
| 16384×48 mb2 ep3 | 368 146 | 2136 ms | 1387 ms | 710 ms | 6 |

结论：**瓶颈不是算力而是 Python 迭代次数**（`epochs × minibatches`），
把 20 次迭代降到 6 次即可从 55k 提到 305k env-steps/s，GPU 利用率 9%→95%。

### 3 小时训练：学到了站立，然后又自己毁掉

`runs/stand_3h`（8192×96×3 epochs，adaptive-LR，entropy 0.005）：

- update 600–2300（15–30 min）：episode length 19.15→19.97 s（20 s 上限），
  return +28.17，eval 站立 6.00 s（评估窗满），`up·Z = 1.000` —— **成功**
- update 2300 之后：塌到 0.20 s，`up·Z = −0.07`（翻过去了），return −0.35

三个原因叠加：

1. **adaptive-LR 被推到地板**：262k 样本的 batch 让 KL 只有 0.001–0.02，
   远小于 `0.5 × desired_kl`，于是每轮 ×1.5，一路降到 `min_lr = 2e-5`。
2. **entropy 奖励把 σ 吹大**：`entropy_coef = 0.005` 是可学习 σ 的净正收益，
   σ 从 ~0.1 涨到 `max_std = 1.0`（entropy −1.8 → +1.7）。动作噪声变成
   ±0.5 rad（腿）/ ±10 rad/s（轮），episode 变短，均值策略被拖向"翻车"。
3. **没有 best checkpoint**：`save_every = 100` 只覆盖 `model_last.pt`，
   18 s 的好策略被后来的坏策略覆盖。

**已验证无效的修法**：单独关掉 adaptive-LR 或单独降 entropy，都会让训练
卡在 0.5–0.9 s（demo_run 1/2/3 三次都失败）。

**有效修法**：保留这套组合，另外加上 best-checkpoint 与 early-stop 把峰值
扣下来。`runs/demo_run4` 用同一组超参 + `--stop-on-survive 1 --save-every 25`，
第 600 次 update 达标（eval 站立 6.00 s，`up·Z +0.99`）即早停，
`model_best.pt` 存住了这个策略。

### 策略实测（`runs/demo_run4/model_best.pt`）

`tools/demo_policy.py`（视觉 MJCF + 跟随相机）与 `play --onnx` 双重确认：

| 指标 | 值 |
|---|---|
| 站立时长 | **10.74 s**，终止原因 `illegal_contact` |
| 终止时高度 / `up·Z` | 0.360 m / +0.834（沉下去后腿擦地） |
| 前向漂移 | 0.209 m/s（指令 vx = 0，即 `eval_vx_mae ≈ 0.21`） |
| ONNX 导出误差 | 9.537e-07 |
| ONNX 独立回放 | 10.00 s / 10.00 s，末态高度 0.4609 m |

产物：`reports/demo_frames.png`（接触表）、`reports/demo_stand.mp4`（11.9 s）。

**两个待修缺陷**：① 前向塑性漂移 0.21 m/s（奖励里没有 vx 误差项惩罚到 0，
或 `hold_bias` 给了恒定前倾）；② 10 s 后高度缓慢下沉导致腿擦地。
二者都不是物理/契约问题，需要在奖励与终止上继续调。
