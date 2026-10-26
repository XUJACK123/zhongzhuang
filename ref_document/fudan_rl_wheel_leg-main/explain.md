# fudan_rl_wheel_leg-main 工程结构与核心训练代码说明

更新日期：2026-10-25。本文依据当前本地文件和调用关系整理，属于静态代码说明；本次没有安装依赖、启动训练、运行仿真或修改训练源码。

本工程的目标是：在 Isaac Gym 中训练轮腿机器人的运动策略，保存 PyTorch checkpoint，导出推理网络，再在 MuJoCo 中验证策略和机构适配。它不是 STM32 电机驱动工程。

你目前需要重点学习 `plane`，因为当前目标是站立、低速前后移动和转向。`jump` 用于跳跃，`mujoco` 用于训练后的仿真验证。具体迁移方案见 [process.md](process.md)；本文解释现有文件，不表示方案中的新增功能已经实现。

## 1. 先看整体目录与数据流

```text
fudan_rl_wheel_leg-main/
├── README.md                  原项目安装和基本使用说明
├── isaacgym脚本.txt            作者保存的训练、播放、续训、导出命令示例
├── process.md                 新机器人迁移训练与分段续训方案，尚待代码实现
├── explain.md                 本文件：现有工程结构和阅读指南
├── .gitignore                 Git 忽略规则
├── plane/                     常规运动训练工程
│   ├── setup.py / pyproject.toml
│   ├── wheel_legged_gym/       训练、环境、网络、PPO 及工具
│   ├── resources/robots/      机器人 URDF、网格及 CAD 导出附件
│   ├── logs/wheel_legged/     历史实验源码和配置快照
│   └── export_onnx/           checkpoint 转 ONNX 的脚本
├── jump/                      跳跃训练工程，目录组织与 plane 基本相同
└── mujoco/                    ONNX 在 MuJoCo 中运行的验证工程
    ├── actor/yuntai/          两个已有 ONNX 模型
    ├── python_tools/         串联坐标、并联机构两套运行脚本
    └── assert_now/            MuJoCo XML、STL 网格、地形图片
```

注意原目录拼写就是 `assert_now`，不是 `assets`。不要只改目录名而不改脚本中的资源路径。

训练和验证是两条不同流程：

```text
训练：
URDF + 任务配置
       ↓
train.py → task_registry → 环境 LeggedRobot + OnPolicyRunner
                                   ↓
                  策略采样动作 → 仿真推进 → 奖励/结束标志
                                   ↓
                  RolloutStorage → PPO.update() 更新权重
                                   ↓
                             model_N.pt

部署模型准备与验证：
model_N.pt → export_onnx.py → encoder + actor 的 ONNX
                                   ↓
                MuJoCo XML + onnx_mj_*.py
                                   ↓
                    观测 → 模型动作 → PD/机构映射 → 力矩
```

`train.py` 是入口，但“怎么学”主要在 PPO；“学什么、身体怎样运动”主要在环境和配置中。修改新机器人的重量分布，首先看模型和动力学配置，不是先改 PPO。

## 2. 最核心的训练文件

以下路径相对于 `plane/`。`jump/` 下存在同名文件，但不是所有内容都相同。

| 优先级 | 文件 | 负责什么 | 什么时候需要改 |
|---|---|---|---|
| 核心 | `wheel_legged_gym/scripts/train.py` | 解析启动参数，创建环境和训练器，保存配置，调用 learn | 增加新的训练启动流程 |
| 核心 | `wheel_legged_gym/envs/wheel_legged/wheel_legged_config.py` | 当前轮腿机器人的模型路径、默认角度、PD、动作缩放、腿长、实验名 | 换机器人、核对动作接口 |
| 核心 | `wheel_legged_gym/envs/base/legged_robot_config.py` | 观测维度、历史长度、指令范围、奖励权重、随机化、仿真参数、PPO 超参数 | 调整任务与训练配置 |
| 核心 | `wheel_legged_gym/envs/base/legged_robot.py` | 加载机器人、执行动作、计算观测与奖励、结束与重置、随机化和课程 | 改观测、动作含义、奖励或环境行为 |
| 核心 | `wheel_legged_gym/rsl_rl/modules/actor_critic_sequence.py` | 当前默认的历史编码器、actor、critic 和探索分布 | 网络结构、模型恢复、训练与部署接口 |
| 核心 | `wheel_legged_gym/rsl_rl/algorithms/ppo.py` | PPO 损失、价值损失、熵、KL 调节、优化器更新、编码器辅助训练 | 加教师约束、改训练算法 |
| 核心 | `wheel_legged_gym/rsl_rl/runners/on_policy_runner.py` | 组织采样与更新循环，日志、保存、恢复、推理接口 | 分段续训、训练阶段和保存规则 |
| 核心 | `wheel_legged_gym/rsl_rl/storage/rollout_storage.py` | 暂存本轮轨迹，计算 GAE/returns，产生训练 mini-batch | 增加教师标签或其他训练数据字段 |
| 连接入口 | `wheel_legged_gym/utils/task_registry.py` | 根据任务名创建环境和 runner，选择恢复路径，保存配置 | 新增独立任务、改 runner 选择 |
| 连接入口 | `wheel_legged_gym/envs/__init__.py` | 注册任务名 `wheel_legged` 与环境/配置的对应关系 | 注册 `wheel_legged_custom` |
| 物理核心 | `resources/robots/infantry_V4/urdf/infantry_V4_increase.urdf` | 当前 plane 配置实际选中的机器人结构和动力学参数 | 对照模型；新机器人另建目录 |

这些文件共同决定训练结果，不能只读神经网络文件就认为理解了整个控制系统。

## 3. 一次训练是怎样运行的

### 3.1 从命令行进入环境

在 `plane` 已正确安装、Isaac Gym 环境可用时，原有启动方式是：

```bash
python wheel_legged_gym/scripts/train.py --task=wheel_legged --headless
```

这条命令启动原任务，不是 process.md 中尚未实现的迁移任务；它不会自动读取现有 ONNX 开始微调。本次没有执行该命令。

调用顺序：

1. `utils/helpers.py::get_args()` 解析 task、resume、load_run、checkpoint、num_envs、max_iterations 等参数。
2. 导入 `envs` 时，`envs/__init__.py` 把 `wheel_legged` 注册到任务表。
3. `task_registry.make_env()` 取得配置、应用命令行覆盖、设置随机种子并创建 `LeggedRobot`。
4. `BaseTask` 建立 Isaac Gym 仿真、GPU/CPU 数据缓冲、可选 viewer。
5. `LeggedRobot` 加载 URDF、创建多份并行环境、初始化传感/状态数据。
6. `task_registry.make_alg_runner()` 创建网络、PPO、轨迹缓冲和 runner；resume 时加载 `.pt`。
7. `train.py` 调用 `runner.learn()` 进入训练。

任务配置存在继承关系：`WheelLeggedCfg` 继承 `LeggedRobotCfg`，专用配置未覆盖的项继续使用基类值。因此只查看其中一个文件会漏掉实际生效参数。

### 3.2 收集一批经验

`on_policy_runner.py::learn()` 在每轮更新前采样多步：

```text
PPO.act(obs, history, privileged_obs)
    ↓
encoder(history) → latent
actor(obs + latent) → 动作分布 → 采样动作
critic(训练用观测 + latent) → 当前价值估计
    ↓
env.step(actions)
    ↓
动作截断 → PD 力矩 → Isaac Gym 物理推进
    ↓
更新姿态、关节反馈、奖励、终止标志和历史
    ↓
PPO.process_env_step() → RolloutStorage
```

当前默认配置每个环境每轮采样 48 个策略步；`sim.dt=0.005`、`control.decimation=2`，策略步长为 0.01 s。并行环境数当前默认较大，实际可被命令行覆盖，不能把它当作适合所有显卡的设置。

### 3.3 使用经验更新网络

- `RolloutStorage.compute_returns()` 用奖励、价值和结束标志计算 GAE 优势及回报目标。
- `PPO.update()` 将数据分为 mini-batch，计算策略裁剪损失、critic 损失与熵项，进行多轮更新。
- 主优化器更新 actor、critic 和可学习探索标准差。
- 当前 sequence 网络还使用独立 `extra_optimizer` 更新 encoder，利用仿真真值训练状态估计。
- runner 记录 TensorBoard 指标，并按保存间隔写 checkpoint。

这里的轨迹缓冲用于当前 on-policy 采样批次，不是无限累积所有历史经验的长期数据库。

## 4. 环境、配置和网络具体负责什么

### 4.1 `envs/base/legged_robot.py`：机器人与任务规则

这个文件体积大，是最值得按函数阅读的核心文件。

| 函数/函数组 | 作用 |
|---|---|
| `create_sim()`、`_create_envs()` | 建立仿真、加载 URDF、创建机器人、取得刚体/关节信息 |
| `_init_buffers()` | 分配角度、速度、动作、历史、随机化参数等张量 |
| `step()` | 一个策略周期的完整推进入口，内部包含多个物理子步 |
| `_compute_torques()` | 将六维动作变成关节位置目标/轮速目标，再通过 PD 计算力矩并限幅 |
| `post_physics_step()` | 汇总仿真结果，更新状态、奖励、结束条件和观测 |
| `compute_proprioception_observations()` | 按固定顺序拼接策略输入 |
| `compute_observations()` | 生成 actor/critic 观测、噪声和历史缓冲 |
| `compute_reward()`、`_prepare_reward_function()` | 按配置选取非零权重奖励并累加 |
| `_reward_*()` | 速度跟踪、姿态、高度、能耗、动作平滑、越限等具体评分公式 |
| `check_termination()` | 判断失败、时间结束及边界等重置条件 |
| `reset_idx()`、`_reset_dofs()`、`_reset_root_states()` | 重置指定环境、机器人状态及相关缓冲 |
| `_resample_commands()` | 生成前进速度、转向速度和高度等任务指令 |
| `_process_rigid_body_props()` | 质量、重心和惯量随机化 |
| `_process_rigid_shape_props()` | 接触摩擦、恢复系数等处理 |
| `_process_dof_props()` | 处理关节属性和限制 |
| `_update_terrain_curriculum()`、`update_command_curriculum()` | 调整地形或指令难度 |
| `_get_heights()` | 获取用于环境/训练的地形高度信息 |

当前 `_compute_torques()` 实际执行的是位置/速度 PD 路径。不能只看函数通用注释或 `control_type` 字段，就认为已经实现了任意动作模式切换。

### 4.2 两个配置文件的分工

`legged_robot_config.py` 包含两类配置：

- `LeggedRobotCfg`：环境、地形、初始状态、指令、动作控制、模型加载、随机化、奖励、观测缩放、噪声、仿真参数。
- `LeggedRobotCfgPPO`：网络层宽、历史编码维度、学习率、PPO 迭代参数、保存间隔和恢复设置。

`wheel_legged_config.py` 在这些基础上设置具体轮腿模型的参数，例如：

- 选中哪个 URDF。
- 六个关节默认角度。
- 关节和轮子的 PD。
- `pos_action_scale=0.5`、`vel_action_scale=10.0`。
- 腿长、实验名、最大训练迭代数等。

“奖励公式”在 `legged_robot.py`，“各奖励的重要程度”在配置文件的 `rewards.scales`。两边需要对应；有函数不代表该奖励当前启用。

### 4.3 `actor_critic_sequence.py`：三个网络的关系

当前默认网络是 `ActorCriticSequence`，不是同目录中的普通 ActorCritic 或 RNN 版本。

```text
125 维历史 → encoder → latent
                         ├── 与当前 25 维拼接 → actor → 6 维动作均值
                         └── 与训练用特权观测拼接 → critic → 价值
```

当前默认 latent 为 3 维，用于机身线速度相关估计；另有历史实验使用 10 帧和 16 维 latent，不能混用 checkpoint 或导出脚本。

actor 的训练动作由均值和可学习标准差组成的高斯分布采样；推理接口使用动作均值。critic 提供训练用的价值估计，不是第二个电机控制器。

当前 actor 输入中的 latent 使用 `detach()`，PPO 的策略损失不会通过该路径更新 encoder；encoder 由独立辅助损失训练。这会影响后续教师辅助损失的实现。

### 4.4 和电控直接相关的接口

当前 25 维观测：

| 部分 | 数量 |
|---|---:|
| 机身角速度 | 3 |
| 重力方向 | 3 |
| 前进/转向/高度指令 | 3 |
| 四关节相对默认姿态的位置 | 4 |
| 六关节/轮速度 | 6 |
| 上一周期动作 | 6 |

动作顺序为 `左关节0、左关节1、左轮、右关节0、右关节1、右轮`。当前配置中：

```text
四关节：目标角度 = 默认角度 + 动作 × 0.5 rad
两轮：目标角速度 = 动作 × 10 rad/s
```

然后由 PD 转为力矩。模型输出既不是直接 CAN 命令，也不是直接六维 N·m；默认角度、方向、动作缩放和历史周期都属于部署接口的一部分。

## 5. `plane/wheel_legged_gym` 其余文件职责

### 5.1 启动与演示脚本

| 文件 | 作用与当前限制 |
|---|---|
| `scripts/train.py` | 真正的训练入口；创建环境和 runner，然后 learn |
| `scripts/play.py` | 加载 `.pt`，在 Isaac Gym 中运行策略；含键盘前后、转向、高度控制；默认不导出 JIT |
| `scripts/play_export.py` | 较早的播放/日志/绘图和 JIT 导出脚本；含固定机器人索引、人工指令与速度积分补偿，不是标准独立评估器 |
| `scripts/export_encoder_jit.py` | 单独导出 encoder 的旧脚本；硬编码旧服务器路径和 135 维输入，不能直接用于当前 125 维历史模型 |
| `tests/test_env.py` | 原意是用零动作运行仿真检查环境；不是纯 CPU 单元测试，且当前 step 返回值解包已与环境接口不一致 |

`play.py`/`play_export.py` 运行的是 Isaac Gym 环境，不是 MuJoCo。`play_export.py` 的自动补偿会改变实际指令，不能用其结果直接代替 process.md 设计的独立评估。

### 5.2 基类与包入口

| 文件 | 作用 |
|---|---|
| `envs/base/base_task.py` | 仿真和 viewer 基础设施，观测/奖励/结束缓冲，以及 reset/render 基础接口 |
| `envs/base/base_config.py` | 将嵌套配置类递归实例化，提供该项目的配置组织方式 |
| `envs/__init__.py` | 导入环境并注册 task 名称，影响 `--task` 能找到什么任务 |
| `wheel_legged_gym/__init__.py` | 计算工程根目录和环境目录常量，用于资源路径定位 |
| `plane/__init__.py` | 外层包标记，不是训练入口 |
| 各子目录 `__init__.py` | Python 包初始化及类/函数导出；是否有额外行为应看具体文件 |

### 5.3 通用工具 `utils/`

| 文件 | 作用 |
|---|---|
| `task_registry.py` | 任务名到环境/配置的映射，创建环境和训练器，恢复 checkpoint、保存配置 |
| `helpers.py` | 配置转字典、命令行参数、随机种子、仿真参数、checkpoint 路径选择，以及 JIT 导出工具 |
| `terrain.py` | 高度场/三角网格地形生成、难度分布、台阶/坑等地形工具；当前 mesh_type 为 plane 时不会生成复杂地形 |
| `math.py` | 四元数、角度及随机采样等数学辅助函数 |
| `logger.py` | 播放阶段的状态/奖励收集与绘图，区别于 runner 的训练 TensorBoard 写入 |
| `keyboardctrl.py` | 独立键盘控制辅助类；当前 play.py 另有自己的键盘实现，不能认为所有键盘输入都走这里 |

### 5.4 本地训练库 `rsl_rl/`

这里是项目自带的训练库源码。入口导入的是 `wheel_legged_gym.rsl_rl`，不应假定修改另一个 pip 安装的 rsl_rl 就会影响本项目。

| 子目录/文件 | 作用 |
|---|---|
| `algorithms/ppo.py` | PPO 训练更新及 encoder 辅助训练 |
| `runners/on_policy_runner.py` | 采样、训练、保存恢复、TensorBoard 和推理入口 |
| `storage/rollout_storage.py` | 保存观测、历史、动作、奖励、done、价值、旧 log probability、均值/标准差；计算 GAE 和 batch |
| `modules/actor_critic_sequence.py` | 当前使用的历史编码器策略 |
| `modules/actor_critic.py` | 普通前馈 Actor-Critic 备选实现 |
| `modules/actor_critic_recurrent.py` | 带 RNN 记忆的备选实现；与“拼接 5 帧历史”不是同一种结构 |
| `modules/normalizer.py` | 经验均值/方差归一化模块；当前主训练链未因此自动替代配置中的固定缩放 |
| `env/vec_env.py` | 向训练算法提供向量化环境的接口约定 |
| `utils/utils.py` | 轨迹分段、补齐/去补齐和代码状态存档工具 |
| `utils/wandb_utils.py` | 可选 Weights & Biases 日志适配器 |
| `utils/neptune_utils.py` | 可选 Neptune 日志适配器 |

当前 runner 使用 PyTorch `SummaryWriter` 写 TensorBoard；目录中存在 WandB/Neptune 适配器，不代表本次训练已经启用在线日志。

## 6. `plane/resources`、`logs` 和导出目录

### 6.1 机器人资源 `resources/robots/`

| 机器人目录 | 作用 |
|---|---|
| `infantry_V1_old` | 旧版模型资源，URDF 名为 `old7.urdf` |
| `infantry_V1` | 另一版步兵模型资源 |
| `infantry_V4` | 当前 plane 任务选用的模型家族；同时有普通 URDF 与 increase 变体 |
| `hero_V1` | 英雄机器人模型资源，含 `hero_V1.urdf` 和 `hero_V1_wuyuntai.urdf` 变体 |
| `hero_2_V1` | 另一套英雄机器人模型资源 |

文件夹存在不等于训练会同时使用所有模型。当前任务通过 `asset.file` 选择一份 URDF。变体之间的准确差别应对比文件参数，不能只凭文件名确定。

每种机器人目录中常见附件：

| 文件/目录 | 作用 | 当前训练是否直接需要 |
|---|---|---|
| `urdf/*.urdf` | 刚体质量、惯量、关节连接、轴、限制以及视觉/碰撞资源引用 | 是，使用被配置选中的文件 |
| `meshes/*.STL` | 基座、左右腿、轮等部件的几何网格 | 被 URDF 引用时需要 |
| `urdf/*.csv` | SW2URDF 导出的参数表，含质量、重心、惯量、坐标和关节信息 | 不直接作为 Isaac Gym 加载入口 |
| `config/joint_names_*.yaml` | ROS 控制器关节名配置 | 当前训练不直接用它决定策略通道顺序 |
| `launch/display.launch` | ROS/RViz 展示启动文件 | 原训练入口不调用 |
| `launch/gazebo.launch` | Gazebo 展示/仿真启动文件 | 原训练入口不调用 |
| `package.xml` | ROS 包元数据 | 不属于 PPO 算法 |
| `CMakeLists.txt` | ROS 包构建/安装说明 | 不用于编译神经网络 |
| `export.log` | SolidWorks 导出工具的日志 | 用于追查模型导出，不是训练日志 |

不要仅修改 CSV 就认为仿真质量已改变；实际运行取决于加载的 URDF 和运行时随机化。也不要用 ROS joint_names YAML 中的顺序替代实际 DOF 映射核对。

新机器人按 process.md 放入 `plane/resources/robots/custom_wheel_leg/`，本次没有创建该模型。

### 6.2 历史实验 `logs/wheel_legged/`

本次检查到 plane 下有 47 个实验目录，jump 下有 1 个；共 144 个文件，都是历史环境/配置快照。当前整个工程没有 `.pt/.pth/.ckpt`，也没有 TensorBoard event 文件。

每个实验目录通常包含三份文件：

| 文件 | 保存什么 |
|---|---|
| `legged_robot.py` | 当时的环境、动作处理和奖励公式 |
| `legged_robot_config.py` | 当时的环境/PPO 基础配置 |
| `wheel_legged_config.py` | 当时的机器人专用配置 |

这些文件是历史对照材料，不是当前 `train.py` 默认导入的代码。`--resume` 也不会自动把这些旧源码替换到当前环境中，必须主动核对配置兼容性。

目录名称可用于查找实验线索：

- `linx_scales=...`、`pd...`：观测缩放、PD 或随机化相关实验。
- `history10latent16...`：10 帧历史、16 维 latent 的实验。
- `recover_from_lied_on_ground...`：恢复/起身相关实验。
- `上台阶...`：台阶或相关运动任务实验。
- `腿熊...`、`25_20...`、`6010.2...`：不同机器人版本或参数尝试。

这些名称是作者备注，不能当作性能排名或完整配置定义。迁移时重点对照 `上台阶3_angz+` 的源码配置，但仍须做教师复现。

以后真正训练时，logs 通常还会产生 `model_N.pt` 和 TensorBoard 数据；当前没有这些产物，不能直接按 README 的示例 checkpoint 编号恢复。

### 6.3 导出脚本 `export_onnx/`

| 文件 | 用途 |
|---|---|
| `export_onnx.py` | 从 `model_state_dict` 读取 encoder/actor，导出 25 维当前观测、5 帧历史、latent=3 的两输入 ONNX |
| `export_onnx_his10.py` | 同类导出流程，但历史为 10 帧、latent=16，历史输入 250 维 |

导出 wrapper 执行 `encoder(history)`，再把 latent 与 obs 拼接送入 actor。导出时不包含 critic、训练探索分布和优化器。

它们都是 `.pt → .onnx` 工具，不能拿现有 ONNX 作为 checkpoint 恢复 PPO，也不负责训练。网络结构参数当前硬编码，需与源 checkpoint 严格一致。

## 7. `jump/` 与 `plane/` 的区别

两者是两个目录形式的训练版本，使用相同 Python 包名 `wheel_legged_gym`，并各自提供 setup.py。不是一个入口里自动切换的两个 task。

本次逐文件比较确认：

| 部分 | 关系 |
|---|---|
| `rsl_rl/algorithms/ppo.py` | 两份当前内容相同 |
| `rsl_rl/runners/on_policy_runner.py` | 两份当前内容相同 |
| `rsl_rl/modules/actor_critic_sequence.py` | 两份当前内容相同 |
| `envs/base/legged_robot.py` | 不同，jump 增加跳跃/腾空及腿部状态相关逻辑 |
| 两级机器人配置 | 不同，控制参数与任务奖励不同 |
| `utils/terrain.py` | 不完全相同 |

因此主要区别在于任务环境、奖励和参数，而不是换了另一种 RL 算法。

jump 的环境中可看到：

- `check_jump()`：根据接触信息维护腾空状态。
- `leg_post_physics_step()`、`forward_kinematics()`：计算腿部长度、摆角等状态。
- `_reward_encourage_jump()`、`_reward_base_height_flight()`、`_reward_leg_tuck()`、`_reward_takeoff_extend()` 等：鼓励起跳、腾空、收腿及伸展。

哪些奖励真正参与训练仍由 `rewards.scales` 决定。jump 的 `_compute_torques()` 同样把关节位置目标和轮速目标经 PD 转为力矩，不因存在 VMC 状态计算就意味着网络直接输出腿端力。

`jump/resources/robots/` 只包含 infantry 相关模型，没有 plane 中的两个 hero 目录；`jump/export_onnx/` 只有普通导出脚本。其余同名基础设施的职责参考上文，但修改时必须确认正在编辑哪个版本。

两版不应在同一个 Python 环境反复 editable install 后混用。建议为实验固定环境和工作目录，并检查 Python 实际导入的包路径。

## 8. `mujoco/`：模型推理与 sim2sim

此目录不执行 PPO 反向传播，不更新网络权重。它将训练好的策略放到另一个物理引擎中运行，用于检查模型、观测、动作接口及机构映射。

### 8.1 `actor/yuntai/`

当前有两个推理文件：

| 文件 | 当前脚本中的角色 |
|---|---|
| `上台阶3_angz+.onnx` | plane 模式策略，也是 process.md 选定的候选教师 |
| `p60.50.2把urdf中力矩限制为50.onnx` | jump 模式策略 |

目录名叫 actor，但这两份 ONNX 实际封装了 encoder 和 actor，不是只有 actor；两者接口均为 obs 25、history 125、actions 6。

### 8.2 `python_tools/onnx_mj_chuanlian.py`

按训练使用的串联关节坐标组织观测和控制：

1. 加载 MuJoCo XML、ONNX 和运行参数。
2. 从传感器/关节状态读取姿态、角速度、位置和速度。
3. `build_obs()` 拼接与训练一致的观测，更新历史。
4. ONNX 推理得到六维动作。
5. `compute_torque()` 将关节位置/轮速目标经 PD 转为力矩。
6. 施加控制、推进仿真、处理键盘和策略切换。

脚本也包含腿部雅可比和跳跃阶段力缩放相关辅助函数，并非只有最简推理循环。默认 XML 路径指向现有共享模型资源，不能只按脚本名推断加载的全部机构约束。

### 8.3 `python_tools/onnx_mj_binglian.py`

在 `BinglianRuntime` 中组织并联机构运行流程：

- 几何求解和虚拟腿状态计算。
- 训练坐标下的观测、历史与 ONNX 推理。
- 策略动作先通过 PD 得到虚拟关节力矩。
- 数值或解析雅可比将力矩映射到并联机构执行关节。
- 气弹簧相关补偿、可选跳跃力缩放、执行器限幅。
- 将结果写入 MuJoCo 执行器并推进仿真。

这是理解“训练动作如何落到真实并联腿电机”的重要参考，但当前几何、符号、补偿和限幅都针对参考机器人，不能直接认定适用于你的机器人。

### 8.4 `assert_now/infantry_binglian_yuntai/infantry_V2/meshes/`

| 文件类别 | 作用 |
|---|---|
| `mjmodel.xml` | MuJoCo 的 MJCF 场景和机器人定义，根标签为 `<mujoco>` |
| 基座、腿部、轮子 `.STL` | 被模型引用的几何资源 |
| `l20/l21/l22/l23`、`r20/r21/r22/r23` 等网格 | 机构中相应连杆的几何资源，具体连接由 XML 决定 |
| `terrain_hfield.png` | 地形高度场图片资源，是否生效取决于 XML 引用和配置 |

该目录中的 XML 有跨目录引用 plane 下网格的情况，不能只拷贝一个 XML 就认为模型资源已经完整。

训练与验证的格式关系：

```text
SolidWorks → URDF + meshes → Isaac Gym 训练
                    └────→ 转换并补齐 MJCF → MuJoCo 验证
```

URDF 和 MJCF 都使用 XML 语法，但不是同一种模型格式，不能只改后缀。

## 9. 包安装与根目录说明文件

| 文件 | 作用 |
|---|---|
| `plane/setup.py`、`jump/setup.py` | 安装本地 Python 包；当前仅使用 find_packages 等基本打包配置，不等于完整依赖安装脚本 |
| 两处 `pyproject.toml` | 指定 setuptools/wheel 构建后端，不是经过验证的 CUDA/PyTorch/Isaac Gym 依赖锁文件 |
| `README.md` | 作者的环境安装和基本训练/验证说明 |
| `isaacgym脚本.txt` | 训练、play、resume、ONNX 导出命令备忘；目录名和 checkpoint 编号是历史示例 |
| `.gitignore` | 控制哪些文件不进入 Git，不参与仿真和网络推理 |
| `process.md` | 新机器人迁移、教师辅助、退出门槛、三小时 checkpoint 的计划 |
| `explain.md` | 当前目录和文件职责说明 |

目前没有 process.md 中规划的 `train_transfer.py`、`prepare_transfer.py`、`evaluate_transfer.py`、`export_transfer.py`、`validate_robot.py`，也未注册 `wheel_legged_custom`。文档中的未来命令不能当作当前已实现接口。

## 10. 已发现的历史接口限制

以下是本次和此前静态检查确认的问题，记录在说明中，不在本次文档任务中修复：

1. **checkpoint 计数与保存不完整。** runner 循环内周期保存使用未及时更新的 current_learning_iteration；最终保存未正确累加续训起点；漏存 encoder 的 extra_optimizer 状态。分段续训前需要处理。
2. **旧环境测试返回值数量不匹配。** `tests/test_env.py` 按 5 项解包，而当前 `LeggedRobot.step()` 返回 6 项，额外包含历史观测；该脚本不能视为已经通过的自动化测试。
3. **旧 encoder 导出脚本不匹配当前模型。** `export_encoder_jit.py` 固定 135 维输入和旧绝对路径，当前默认为 125 维历史。
4. **JIT 导出不一定含历史编码器。** `helpers.export_policy_as_jit()` 的非 RNN 分支只导出 actor；对当前 sequence 模型，它需要已经拼好 latent 的输入，不能把该文件直接当成接受 obs/history 的完整策略。
5. **播放脚本不是独立验收工具。** play_export 含人工控制逻辑、速度补偿和固定日志索引；缩小环境数或改变配置后需重新核对。
6. **logs 源码不会自动恢复。** 加载权重仍使用当前导入的环境和配置。网络维度相同也不保证观测缩放、默认角度和动作含义相同。
7. **未验证运行环境。** 本次没有在 GPU 上执行训练或仿真；静态阅读不能代替 Python 版本、依赖、GPU 和动力学运行验证。

## 11. 针对你的目标，应该先读和修改哪里

### 推荐阅读顺序

1. `scripts/train.py`：先知道启动入口与对象如何创建。
2. `envs/wheel_legged/wheel_legged_config.py`：知道现在训练的是哪台机器人。
3. `envs/base/legged_robot_config.py`：理解观测、奖励、随机化和训练设置。
4. `envs/base/legged_robot.py`：重点读观测、动作转力矩、奖励、重置。
5. `rsl_rl/modules/actor_critic_sequence.py`：理解 encoder/actor/critic。
6. `rsl_rl/runners/on_policy_runner.py`：理解采样、更新和保存顺序。
7. `rsl_rl/algorithms/ppo.py` 与 `storage/rollout_storage.py`：理解实际优化和数据流。
8. `export_onnx/export_onnx.py`：理解训练模型与部署模型的边界。
9. `mujoco/python_tools/onnx_mj_binglian.py`：理解机构适配；与串联脚本对照。

### 按需求定位文件

| 你想做什么 | 应该看哪里 |
|---|---|
| 换成自己的机器人 | 新建 URDF/meshes/参数清单；新增机器人专用任务配置 |
| 修改质量、重心和惯量 | URDF inertial + 环境运行时刚体随机化 |
| 改动作表示和电机控制 | `_compute_torques()`、动作缩放、关节映射，并同步部署代码 |
| 改模型输入 | 观测拼接、历史、噪声、维度配置、网络、导出、部署全部一致修改 |
| 让机器人更重视站稳 | 奖励公式和权重；另外设置独立稳定性评估，不能只看总回报 |
| 增加教师约束 | PPO 损失、冻结教师、迁移 runner，按 process.md 实现 |
| 三小时后保存再继续 | runner 保存/恢复、信号处理、时间预算、配置存档 |
| 判断何时取消教师 | 新增独立评估和训练阶段状态机；当前没有现成实现 |
| 放到 H723 控制电机 | 部署推理 + 观测适配 + 动作/机构力矩转换；训练目录不负责 CAN |

当前应优先推进 `plane` 的模型和接口核对，保留原任务作对照。先把身体模型、观测和动作含义对应正确，再进行教师辅助训练；不要通过调整 PPO 来掩盖关节方向、重心或减速比错误。
