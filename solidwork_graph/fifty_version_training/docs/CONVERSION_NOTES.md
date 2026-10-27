# fifty_version_training 转换说明

由 `tools/build_training_urdf.py` 生成，源模型是
`solidwork_graph/fifty_version_with_meshes`（六个活动关节已修复、坐标系仍是 SolidWorks 原始值）。
本目录**不修改源包**，`meshes/` 是指向源 STL 的硬链接，构建时会校验 SHA256。

## 1. 这次做了什么

| 项目 | 做法 | 结果 |
|---|---|---|
| 基座坐标系 | 原点 = 两髋轴心中点；X = +旧 Y（前）、Y = +旧 X（左）、Z = −旧 Z（上） | 旋转阵行列式 +1，左右髋对称于 ±0.2277 m |
| 零位 | 每个关节绕自身横轴重新对零，使 q=0 时**大腿水平朝前、小腿竖直向下** | 与复旦公式 $\theta_1=q_{hip}$、$\theta_2=q_{knee}+\pi/2$ 严格一致，200 组随机姿态最大误差 6e-15 m |
| 关节顺序 | URDF 内按**右腿先**声明 | `rf0, rf1, r_wheel, lf0, lf1, l_wheel`，与 `legged_robot.py` 的硬编码索引 `[0,1,3,4]` / `2,5` 对齐 |
| 关节符号 | 右腿局部 z = +Y，左腿 = −Y；两轮都取 +Y（正转前进） | 见 `docs/PARAMETER_TABLE.md` 的符号表 |
| 网格 | 不改 STL 一个字节；visual 用"旧 link 系 → 新 link 系"的刚性变换写 origin | 惯量同步做 $R I R^\top$，质心同步平移 |
| 碰撞 | visual 保留 STL；collision 换成基本体 | 机身 box、大腿/小腿 cylinder、轮 cylinder（半径 0.060 m 参数化） |
| 限位/驱动 | 腿限位取默认角 ±0.6 / ±0.7 rad，effort 30 N·m、velocity 1000 rad/s | **全部是 provisional**，见 `model_parameters.json` 的 `provisional` 段 |

推导出的关键数值：

- 平面腿长 0.210 / 0.250 m，轮距 0.529 m，髋距 0.455396 m。
- 默认姿态（= SW 装配姿态在复旦坐标下的表示）：
  `rf0=+2.0554`、`rf1=−2.5593`、`lf0=−2.0554`、`lf1=+2.5593`。
- 该姿态下 $\theta_0\approx-0.056$ rad、髋轴离地 ≈ 0.4648 m、生成高度 0.4748 m（含 10 mm 余量）。
- 整机质心在新坐标系中约 `(0.0785, 0.0249, 0.0855)` m，即**在轮轴前方约 5.6 cm**。
  因此零动作站立会缓慢前倾，冒烟测试只验证数值与接触，不验证自稳。

## 2. 没有做的事（不要当成已完成）

- **没有运行 Isaac Gym / MuJoCo**：`config/build_report.json` 里 `isaacgym_runtime` 仍是 `NOT RUN`。
- **没有建模五连杆闭链与气弹簧**：本模型是"等效串联六自由度"，
  真实机构是髋部双电机并联驱动（见上一级 `README.md` 的机构说明），
  电机角→虚拟关节角的映射、雅可比和力矩映射属于 G4，不在这里。
- **第二电机转子惯量没有折算**：`dof_armature` 目前全 0，等电机/减速比数据。
- **轮组质量与半径未与实物核对**：`wheel_radius_m=0.060` 只是网格半径，
  CSV 里轮 link 只有 M3508 与端盖类零件，可能缺胎面。
- **base_link 惯量未复核**：base 占 91% 质量，需回 SolidWorks 核对质量属性。
- **碰撞体是几何近似**：机身用一个盒、腿用圆柱，未做凸分解；
  自碰撞在训练配置中关闭（`self_collisions=1`）。
- **前向方向是 provisional**：取旧 +Y 为前。若与实物相反，需要
  `--forward-sign -1` 重新生成，并同时交换左右标签。

## 3. 已知风险

1. **零位是一个"L 形"姿态**（大腿水平、小腿竖直）。它可能超出真实机构可达范围，
   但没有关系：它只是坐标定义，实际训练围绕默认角（SW 装配姿态）进行。
   关节限位是围绕默认角给的 provisional 区间。
2. **五连杆可行域不是矩形**：真实 $(q_{hip},q_{knee})$ 可行集是曲边区域，
   URDF 的 lower/upper 只是外接框。部署侧（G4）必须做投影钳位，
   否则策略可能给出机构到不了的角度组合。
3. **轮 link 包含电机定子**：continuous 关节会让整个轮 link 旋转，
   如果实物定子固定在小腿上，则转动惯量与外观都不完全正确。需要 CAD 确认零件归属。
4. **default 角很大（±2 rad）**：因为零位在 L 形姿态，SW 装配姿态在该坐标图里
   就是 ±2 rad。这是坐标选择的结果，不是机械异常；观测里用的是相对默认角的偏差。

## 4. 重新生成与校验

```bash
cd solidwork_graph/fifty_version_training
python3 tools/build_training_urdf.py --force     # 重新生成 URDF / 参数 / 参数表
python3 integration/validate_model.py            # 17 项静态检查，退出码 0 表示通过
python3 integration/train_fifty.py --validate-only
```

Isaac Gym 可用之后：

```bash
python3 integration/train_fifty.py --smoke-test --fudan-root <path> --headless
python3 integration/train_fifty.py --joint-test --fudan-root <path> --headless
python3 integration/train_fifty.py --fudan-root <path> --headless --num_envs 1024
```

`--smoke-test` 会断言：DOF 顺序、7 个刚体、两个轮子接触体、加载质量等于
23.2596 kg（用来证明 Isaac Gym 真的用了 URDF 惯量而不是 `density`）。
`--joint-test` 会逐个关节给 ±小指令，记录 `dof_pos` / `dof_vel`，
用于确认"索引 → 关节 → 物理方向"这一条链。

## 5. 文件清单

| 文件 | 作用 |
|---|---|
| `urdf/fifty_training.urdf` | 生成的训练模型（7 link / 6 joint / 基本体碰撞） |
| `config/model_parameters.json` | 变换矩阵、逐 link 惯量与网格变换、符号表、默认角、限位、provisional 标记 |
| `config/joint_names_fifty_training.yaml` | 控制器关节名（右先） |
| `config/build_report.json` | 构建时的校验结果与网格哈希 |
| `docs/PARAMETER_TABLE.md` | 人读参数表（自动生成） |
| `tools/build_training_urdf.py` | 转换脚本（纯标准库，可重复运行） |
| `integration/validate_model.py` | 无依赖静态校验 |
| `integration/train_fifty.py` | Isaac Gym 任务适配器（validate / smoke / joint-test / train） |
| `meshes/*.STL` | 指向源 STL 的硬链接，构建时校验哈希 |
