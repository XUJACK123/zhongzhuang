# 训练环境检查报告（G0）

检查日期：2026-10-28。主机：`xy-Legion-Y7000-IRX10`。

## 1. 硬件与系统

| 项目 | 值 |
|---|---|
| 发行版 | Ubuntu 22.04.5 LTS |
| 内核 | 6.8.0-138-generic |
| GPU | NVIDIA GeForce RTX 5060 Laptop GPU，8151 MiB |
| 驱动 | 595.99.02（`nvidia-smi` 可用） |
| GPU 架构 | Blackwell，compute capability **sm_120** |

## 2. 已安装的环境

| 组件 | 状态 |
|---|---|
| conda | `~/miniconda3`，base 环境 Python 3.10，无 `leg` 类环境 |
| **`leg38` 环境** | **已创建**：conda-forge，Python 3.8.20，numpy 1.23.5，pip 24.3.1 |
| PyTorch | **已安装** `2.4.1+cpu`（CPU 版；见第 4 节为什么不是 CUDA 版） |
| TensorBoard | **已安装** 2.14.0（`on_policy_runner` 需要） |
| rsl_rl | 使用仓库自带的 `plane/wheel_legged_gym/rsl_rl`，不需要 pip 安装 |
| Isaac Gym | **缺失**，需要手动下载（见第 3 节） |

说明：`pip` 在 PATH 上被系统 `/usr/bin/pip`（Python 3.10）遮蔽，环境内请用
`python -m pip`。另外本次排障过程中，系统 Python 3.10 的用户站点被装入了
`torch 2.4.1+cpu`（`~/.local/lib/python3.10/site-packages`）。它不影响 `leg38`，
如不需要可以执行 `/usr/bin/pip uninstall torch` 移除。

## 3. 还缺 Isaac Gym Preview 4

Isaac Gym 的安装包需要 **NVIDIA 开发者账号登录**后从
<https://developer.nvidia.com/isaac-gym> 下载（`IsaacGym_Preview_4_Package.tar.gz`），
无法用脚本自动获取。拿到之后的步骤：

```bash
tar -xf IsaacGym_Preview_4_Package.tar.gz -C ~/
source ~/miniconda3/etc/profile.d/conda.sh && conda activate leg38
cd ~/isaacgym/python && python -m pip install -e .
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
cd ~/isaacgym/python/examples && python 1080_balls_of_solitude.py   # 或 headless 验证
```

装好后即可运行：

```bash
cd solidwork_graph/fifty_version_training
python integration/train_fifty.py --smoke-test --fudan-root ../../ref_document/fudan_rl_wheel_leg-main --headless
python integration/train_fifty.py --joint-test --fudan-root ../../ref_document/fudan_rl_wheel_leg-main --headless
```

## 4. 重要的兼容性风险（RTX 5060 + Isaac Gym Preview 4）

`nvidia-smi` 实测：`NVIDIA GeForce RTX 5060 Laptop GPU, compute_cap 12.0, driver 595.99.02`
（compute_cap 12.0 = sm_120，Blackwell）。

问题不是显卡性能，而是**版本链闭合、交集为空**：

| 环节 | 事实 | 来源 |
|---|---|---|
| Isaac Gym Preview 4 | 2021–2022 年发布，已停止更新；Python 绑定只提供 **cp38** | fudan README 要求 Python 3.8 |
| Python 3.8 | 能装的最后一个 PyTorch 是 **2.4.1** | `pip index versions torch`（leg38 环境实测） |
| PyTorch 2.4.1 | 官方轮子最高 **cu121**，SASS 只到 **sm_90** | PyTorch 发布说明 |
| RTX 5060 | **sm_120**，需要 CUDA 12.8+ 的 kernel | `nvidia-smi` |
| 支持 sm_120 的 PyTorch | 从 **2.7 + cu128** 开始 | PyTorch 2.7 发布说明 |
| PyTorch 2.7 | 要求 **Python ≥ 3.9** | PyPI 元数据 |
| Isaac Gym 的 Python 3.9+ 绑定 | 不存在 | — |

也就是说：Isaac Gym → py3.8 → torch≤2.4.1 → CUDA≤12.1 → 无 sm_120；
而 RTX 5060 → sm_120 → CUDA≥12.8 → torch≥2.7 → Python≥3.9 → Isaac Gym 装不上。
**两条链无法在同一个环境里同时满足。**

即使 PyTorch 那一关能过，Isaac Gym 的 **PhysX GPU 管线**是另一套闭源 CUDA 二进制（2021 年编译），
同样没有 Blackwell 的机器码，所以 GPU 仿真这条路依然不通。

因此**在这台笔记本上无法 GPU 运行 Isaac Gym**（未实测，本机没有 Isaac Gym；装好后三条命令即可证实）。
可行路径：

1. **CPU 管线冒烟**（推荐先做，够用）：Isaac Gym 支持纯 CPU 仿真，
   ```bash
   python integration/train_fifty.py --smoke-test --fudan-root ../../ref_document/fudan_rl_wheel_leg-main \
     --headless --sim_device=cpu --pipeline=cpu
   ```
   单环境 100 步，验证加载、DOF 顺序、接触、观测和 PD 语义；训练速度不适合正式 PPO。
2. **正式训练放到远程 GPU**：RTX 3090（sm_86）、4090（sm_89）、A100（sm_80）都在 torch 2.4.1 的支持表内，
   按第 3 节重装 Isaac Gym 即可。fudan 作者自己也是在租用的 GPU 服务器上训练的。
3. **迁移到 Isaac Lab**（官方替代，支持 Blackwell、Python 3.10 + torch 2.7+）：
   但 fudan 的 legged_gym 代码（观测拼接、奖励、runner）需要移植，属于数天到数周的工作，
   不建议在第一阶段做。

不受影响的部分：MuJoCo sim2sim 验证、ONNX 导出、PyTorch 后处理在这台机器上都能正常跑
（MuJoCo 物理在 CPU，渲染走 OpenGL）。

以上是静态判断，未实测（本机没有 Isaac Gym）。装好之后先跑第 3 节的官方示例即可确认。


## 5. 结论

- G1（训练资产生成与静态校验）**已完成**，不依赖 Isaac Gym。
- G0 只差 Isaac Gym 安装包（需要你下载）；`leg38` 与 torch/tensorboard 已就绪。
- G2 冒烟命令已写好，**尚未运行**，不能声称加载/站立已验证。
