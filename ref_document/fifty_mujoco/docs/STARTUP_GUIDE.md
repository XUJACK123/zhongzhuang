# 启动指南（从离线数值到正式训练）

> 从 `backups/progress_history_20261031_204938.md` 的 §25 抽出，作为独立文档保留。
> **所有需要运行环境的命令都只写在文档里**；本文不表示它们已被执行。
> 当前状态、下一步与注意事项见根目录 `progress.md`。

## 0. 定位

> 本章覆盖 §24.13 关于平面杆长、23.1 mrad 和末端连杆调整的旧结论。
> **本章所有需要运行环境的命令都只写在文档里，本轮一条都没执行。**

## 1 本轮实际改了什么、复核到什么程度

被改动的文件在动笔前已整体备份到仓库根 `backups/20261031_offline_numerics_115653/`
（含 `manifest.json` 与逐文件 SHA-256）。

| 组 | 改动 | 复核结果 |
|---|---|---|
| 几何 | 平面杆长改为**垂直铰轴投影**：`planar = ‖d − (d·a)a‖`；恢复 210 / 250 mm；节点不移动；末端连杆回到 96.600 mm；左右侧向偏移分别保存 | 投影 vs CAD 节点跨距偏差 **3.46e-10 mm** |
| 姿态 | 撤销 23.1 mrad；正下方参考姿态仍为 thigh −2.115162642 / shin −1.120740360；另存"旧串联默认姿态"（轮心前 22.865346 mm）以免混称 | 映射 vs 模型 FK 偏差 **3.6e-15 rad** |
| 弹簧 | 计算改用**三维销距**（含 11 mm 侧向错位）；装配 194.949622、标称 227.760939；已压缩 22.239061、剩余可压缩 52.760939；全伸展 249.126126 mm | `L3D² − Lplanar² = 121.000000000` |
| 载荷解释 | 24.815 N·m 是整车载荷杠杆估算；对称均分示例 12.408 N·m/侧；两者都不是完整静力学结论 | — |
| 哈希 | 文本资产按 LF 归一化后哈希（`stl.sha256_text`），保留 `sha256_raw`；**未跳过、未删断言、未把检查放宽到恒真** | `sha256_text == recorded` → True |
| 运行配置 | `gas_springs` **仍是旧 300 N**；真实弹簧 450 N / ≈250 mm / 75 mm 只作为记录 | 未把 450 N 施加到旧锚点上 |

**已运行**（全部不依赖仿真库）：`mechanism-check`、`joint-conventions`、
`closed-mjcf`（只编译 + 关节零位残差）、`report_index`，以及
`pytest tests/test_spring.py test_resolve.py test_joint_conventions.py test_closed_chain.py`。

**没有运行**：完整 `pytest tests`（含 `mujoco`/`torch` 的用例）、任何动力学/接触/`mj_step`、
`train*`、`gpu-parity`、SolidWorks。

## 2 先看清楚：现有部分入口仍绑在旧模型上

照抄命令之前必须知道这件事，否则会把旧模型的通过当成新模型的通过：

| 入口 | 现状 | 会不会误判 |
|---|---|---|
| `mechanism-check` / `joint-conventions` / `closed-mjcf` / `step-audit` | 已按本轮几何与解析层更新 | 不会：只做静态几何与编译 |
| `audit-assets` / `build-assets` / `validate-mjcf` / `inspect` / `spawn-audit` | 仍生成与读取**等效串联模型** `assets/mjcf/fifty_train.xml` | **会**：全部通过也只说明旧模型没问题 |
| `spring-audit` / `spring-sweep` / `joint-test` / `contact-test` / `contract-test` | 同上，且弹簧是 Fudan 300 N 占位 | **会** |
| `train` / `train-gpu` / `evaluate-stand` / `gpu-parity` / `export` / `play` | 同上 | **会**：跑出来的权重属于旧模型 |

所以第 ② 阶段的主体工作不是"跑命令"，而是**把入口指向新模型**：共同的模型版本选择、
刚体/执行器/弹簧名称映射、spawn 与重置。在它完成之前，任何"训练成功"的记录都不能算新模型的验收。

## 3 阶段表（顺序不可跳）

| 阶段 | 进入下一阶段的条件 |
|---|---|
| ① 离线数值检查 | 几何、参考姿态、弹簧与映射所有派生值一致，无未解释偏差 |
| ② 补齐训练模型接入 | 地面、碰撞、正确惯量、弹簧、轮轴方向、连续轮关节、电机限制都进模型；**控制配置配套，运行入口不再指向旧模型** |
| ③ 环境与编译检查 | 已有训练机器依赖核对；编译候选模型无错，尺寸/惯量/名称映射/闭链约束检查通过 |
| ④ 初始化检查 | 完整设置主动与被动关节；参考姿态与重置闭链闭合、轮地关系正确、弹簧未越程 |
| ⑤ CPU 短时仿真 | 逐通道扰动、弹簧方向、接触与有限时长运动无 NaN、无明显穿透、无异常约束发散或符号错误 |
| ⑥ 契约与 GPU 对拍 | 新模型上观测/动作/历史/终止/重置/图重放的 CPU/GPU 数值符合容差 |
| ⑦ 训练冒烟 | 固定高度站立跑 20 updates，更新/保存/恢复/评估正常 |
| ⑧ 正式训练 | R0 纯 PPO；H0 仅在兼容检查通过后加入；**总预算最多 3 小时**，保存配置与独立评估报告 |
| ⑨ 站立验收 | 独立 64 个初态、每次 60 s，**≥61/64** 成功，速度 MAE ≤ 0.03 m/s、高度 MAE ≤ 0.02 m，并满足其余既定门槛 |

## 4 ① 离线数值检查（**本轮已完成**，值为复现基准）

- **前置**：无。
- **命令**：

```bash
cd ref_document/fifty_mujoco
uv run python -m fifty_mujoco.cli mechanism-check
uv run python -m fifty_mujoco.cli joint-conventions
uv run python -m fifty_mujoco.cli closed-mjcf        # 需要 mujoco 才能编译
uv run python tools/report_index.py
```

- **预期报告**：`reports/mechanism_check_<UTC>.json`、`reports/joint_conventions_<UTC>.json`、
  `reports/closed_chain_<UTC>.json`、`reports/index.json`。
- **通过标准**：`mechanism-check` 打印 `verdict : all attempted checks within 1e-9 m` 且
  `link length : planar thigh 210.000000 mm / shin 250.000000 mm`；
  `at nominal : 3D pin 227.760939 mm  compressed 22.239061 mm  remaining 52.760939 mm`；
  `joint-conventions` 打印 `status : passed`；`closed-mjcf` 两环残差 < 1e-9 m 且总质量与串联资产相等。
- **失败处理**：几何不一致时**不要改数去凑**。`resolve.py` 的报错会区分
  "投影与 CAD 不符" 和 "配置 `lengths` 与投影不符"，两者只能改一处。

## 5 ② 补齐训练模型接入（**未开始，工作量最大**）

- **前置**：① 通过。
- **待实现的入口**（当前不存在，下面是规格不是命令）：

```bash
uv run python -m fifty_mujoco.cli build-closed-model --variant train   # 生成候选训练模型
uv run python -m fifty_mujoco.cli model-summary --model closed          # 名称映射与尺寸核对
```

- **必须包含**：地面与接触几何；正确质量/惯量与网格安装变换（骨架现在是连杆公式惯量，
  **不能**因为质量相等就认为惯量已继承）；弹簧真实锚点 + 11 mm 侧向错位且三维力一致；
  轮轴方向与 `policy_contract.yaml` 一致（左轮 +Y）；连续轮关节（去掉 ±3.2 rad 限位）；
  电机连续/峰值力矩、速度上限、`armature`；共同模型版本选择 + 名称映射，
  并让 `spawn`、`spring-audit`、`contact-test`、`contract-test`、`gpu-parity`、训练与评估入口全部指向它。
- **通过标准**：所有旧入口不再指向 `fifty_train.xml`；两个模型能在同一进程里被区分。
- **失败处理**：某个入口无法适配时**记录为已知限制并在训练配置里关掉该特性**，
  不要用"附近的值"顶替。

## 6 ③ 环境与编译检查

- **前置**：② 完成；依赖在**已有训练机器**上核对，本机不重装。

```bash
uv sync --extra gpu                  # 仅当依赖清单变化
uv run python -m fifty_mujoco.cli doctor --device cuda
uv run python -m fifty_mujoco.cli build-closed-model --variant train
uv run python -m fifty_mujoco.cli model-summary --model closed
```

- **通过标准**：无编译错误；`nq/nv/nu/neq` 与设计一致；约束 Jacobian 在初态**满秩**
  （`nv − 2·neq = 12` 只是数出来的，不是秩检查）；名称映射 100% 命中。
- **失败处理**：编译错误先查 `<connect>` 是否挂在 site 上；秩不足通常是两环共线
  或某环退化成同一 body 内的自约束。

## 7 ④ 初始化检查

```bash
uv run python -m fifty_mujoco.cli spawn-audit --backend cpu     # 适配后指向新模型
uv run python -m fifty_mujoco.cli inspect --backend cpu
```

- **通过标准**：参考姿态轮心正下方、轮底离地；闭链闭合残差 < 1e-9 m；被动关节残差一致；
  弹簧三维长度在 `[free − stroke, free]` 内；主动/被动关节、基座姿态、速度、控制量与历史状态
  **全部显式设置**。
- **失败处理**：被动关节落到另一支路时，检查初值是否与 `mechanism/bridge.py` 同支；
  不要靠加大迭代次数掩盖分支错误。

## 8 ⑤ CPU 短时仿真

```bash
uv run python -m fifty_mujoco.cli joint-test --backend cpu
uv run python -m fifty_mujoco.cli spring-sweep --backend cpu --duration 5
uv run python -m fifty_mujoco.cli contact-test --backend cpu
```

- **通过标准**：逐通道正扰动方向正确；断使能仍有被动支撑；无 NaN、无穿透；
  约束残差不发散；弹簧推离而非拉近。
- **失败处理**：约束发散先降 `timestep`、查 `solref/solimp`；
  **不要**用放大 `damping` 让症状消失——那会把机构问题变成参数问题。

## 9 ⑥ 契约与 GPU 对拍

```bash
uv run python -m fifty_mujoco.cli contract-test --backend cpu
uv run python -m fifty_mujoco.cli gpu-parity --num-envs 256 --steps 200
```

- **通过标准**：25/125/6 语义在新模型上仍成立（需要真实坐标转换，不是只保留向量长度）；
  CPU/GPU 在容差内；终止与重置逐环境一致。
- **失败处理**：先确认 mjlab 与 CPU 的积分器/时间常数是否不同，再查 `armature` 与执行器顺序。

## 10 ⑦⑧⑨ 训练与验收

```bash
# ⑦ 冒烟
uv run python -m fifty_mujoco.cli stage-plan --stage 0
uv run python -m fifty_mujoco.cli train-gpu --num-envs 1024 --steps 20 --task stand --stand-command 0

# ⑧ 正式（参数以 stage-plan 输出的基线为准；总预算 <= 3 h）
uv run python -m fifty_mujoco.cli train-gpu --num-envs 2048 --task stand     --stand-command 0 --seed 0 --minutes 90
uv run python -m fifty_mujoco.cli evaluate-stand --checkpoint runs/<run>/model_best.pkl

# ⑨ 验收
uv run python -m fifty_mujoco.cli evaluate-stand --checkpoint runs/<run>/model_best.pkl     --episodes 64 --seconds 60
```

- **⑦ 通过标准**：20 次更新无异常；`checkpoints.json` 最佳指标方向正确（MAE 越低越好）；
  中断后 `--resume` 保持累计 update 编号与最佳指标。
  失败处理：最佳模型被一次较差评估覆盖时，查 `metric_is_better` 的方向表。
- **⑧ 通过标准**：保存配置快照与独立评估报告；R0/H0 同预算可比；中途改奖励或课程后
  不得仍当作同一组对照。失败处理：预算用完未达验收就**停下来记录**，不延长预算重跑同一配置。
- **⑨ 通过标准**：独立 64 初态 × 60 s，≥61/64；速度 MAE ≤ 0.03 m/s；高度 MAE ≤ 0.02 m；
  评估不混入 reset 后状态。失败处理：失败集中在少数初态时先看是否同一支路或同一弹簧行程边界，
  不用加长训练掩盖结构性问题。

## 11 旧哈希失败是怎么处理的

`tests/test_audit.py` 的两个失败已查明并修复（§24.14.5）：不是文件被改动，而是 Windows 检出把
URDF 变成 CRLF 而记录值是 LF。改法是文本资产按 LF 归一化后哈希，并保留 `sha256_raw` 供核对。
**没有跳过测试、没有删断言、没有把哈希检查放宽到恒真**——测试里同时断言"只改行尾不变"
与"改内容必变"。

## 12 本章明确没有做的事

- 没有安装或升级依赖，没有运行 MuJoCo（除 `closed-mjcf` 的编译）、CUDA、训练、SolidWorks。
- 没有生成候选训练模型；② 的两个入口仍是待实现规格。
- 没有把 `robot.yaml` 默认角或 `policy_contract.yaml` 切到新模型。
