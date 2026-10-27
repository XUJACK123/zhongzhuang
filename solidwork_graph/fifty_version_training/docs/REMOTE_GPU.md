# 远程 GPU 训练：准备与执行清单

本机（RTX 5060 Laptop，sm_120）只能跑 CPU 冒烟，正式训练建议放到远程 GPU。
本文件是那台机器上的完整步骤，按顺序执行即可。

## 1. 选卡

Isaac Gym Preview 4 只能配 Python 3.8（torch ≤ 2.4.1，CUDA ≤ 12.1），
所以**必须选 torch 2.4.1 支持表内的架构**：

| GPU | compute capability | 能否用 |
|---|---|---|
| V100 | 7.0 | ✅ 可用但太老 |
| A100 / A30 | 8.0 / 8.0 | ✅ 推荐 |
| RTX 3090 / A40 / A6000 | 8.6 | ✅ 推荐（性价比） |
| RTX 4090 / L40S / L4 | 8.9 | ✅ 推荐（最快） |
| H100 | 9.0 | ✅ 可用 |
| **RTX 50 系（5060/5070/5080/5090）** | **12.0** | ❌ 需要 torch ≥ 2.7 + Python ≥ 3.9，与 Isaac Gym 冲突 |

其它要求：Ubuntu 20.04/22.04、驱动 ≥ 525（cu118 运行时要求）、至少 8 核 CPU、
磁盘 ≥ 20 GB（Isaac Gym 约 1 GB，日志与检查点另计）。

显存与并行环境数（经验值，先小后大）：1024 环境约 3–5 GB，4096 环境约 10–16 GB。
fudan 配置默认 4096，显存不足时用 `--num_envs 1024` 起步。

## 2. 把文件传到远程机器

只需要两个目录（`fifty_version_training` 里的 `meshes/` 是硬链接，传输后会变成普通副本，约 57 MB）：

```bash
# 在本地 zhongzhuang 目录下执行
rsync -av --progress --exclude '__pycache__' \
  solidwork_graph/fifty_version_training \
  user@host:/home/user/work/zhongzhuang/solidwork_graph/

rsync -av --progress --exclude '__pycache__' --exclude 'logs' \
  ref_document/fudan_rl_wheel_leg-main/plane \
  ref_document/fudan_rl_wheel_leg-main/mujoco/actor \
  user@host:/home/user/work/zhongzhuang/ref_document/fudan_rl_wheel_leg-main/
```

`mujoco/actor/yuntai/*.onnx` 是后面 G3 教师迁移要用的教师网络，体积很小，建议一起传。

## 3. 在远程机器上装环境

```bash
cd /home/user/work/zhongzhuang/solidwork_graph/fifty_version_training
bash tools/setup_remote_env.sh
```

脚本会：检查 GPU 与 compute capability → 建 `leg38`（Python 3.8 + numpy 1.23.5）→
装 `torch 2.4.1+cu118` 与 tensorboard → 打印 CUDA 可用性 → 检查 Isaac Gym。

Isaac Gym 需要手动下载（脚本会打印步骤）：

```bash
# 1) 在 https://developer.nvidia.com/isaac-gym 登录并下载 IsaacGym_Preview_4_Package.tar.gz
tar -xf IsaacGym_Preview_4_Package.tar.gz -C ~/
source ~/miniconda3/etc/profile.d/conda.sh && conda activate leg38
cd ~/isaacgym/python && python -m pip install -e .
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
cd ~/isaacgym/python/examples && python 1080_balls_of_solitude.py
```

## 4. 执行顺序与验收

```bash
cd /home/user/work/zhongzhuang/solidwork_graph/fifty_version_training
conda activate leg38

# 4.1 静态校验（不需要 Isaac Gym）
python integration/validate_model.py

# 4.2 单环境冒烟
python integration/train_fifty.py --smoke-test --fudan-root ../../ref_document/fudan_rl_wheel_leg-main --headless

# 4.3 逐关节方向检查
python integration/train_fifty.py --joint-test --fudan-root ../../ref_document/fudan_rl_wheel_leg-main --headless

# 4.4 训练链路小规模验证（不是正式训练）
python integration/train_fifty.py --fudan-root ../../ref_document/fudan_rl_wheel_leg-main \
  --headless --num_envs 64 --max_iterations 20
```

| 步骤 | 通过标准 |
|---|---|
| 4.1 | 退出码 0，17 项检查全绿 |
| 4.2 | `resets=0`、无 NaN、加载质量 23.2596 kg、每个轮地接触力 ≈ 114 N（半车重）、基座高度接近 0.4748 |
| 4.3 | 每个关节的 `sign_ok=true`（腿看 `delta_q` 符号，轮看 `delta_qd` 符号） |
| 4.4 | 奖励有限、不出现 NaN、`plane/logs/fifty_wheel_legged/<时间>_six_dof/` 里出现 `model_*.pt` 与 `env_cfg.json`/`train_cfg.json` |

4.4 通过只说明“训练通路是通的”，**不代表策略可用**。

## 5. 之后才是正式训练

按 `process.md` 的路线执行，不要从零 PPO：

1. 恢复教师 ONNX 的 actor/encoder 权重（`mujoco/actor/yuntai/上台阶3_angz+.onnx`），
   比较相同输入的 PyTorch/ONNX 输出误差；
2. 实现 `wheel_legged_custom` 任务、教师辅助损失与权重退火；
3. 独立 PPO 与评估；这些目前都还没有实现（见主说明文档 G3）。

## 6. 常见问题

- **`no kernel image is available`**：GPU 架构超出 torch 2.4.1 的支持范围，换卡或降级到受支持的架构。
- **`ImportError: libpython3.8m.so.1.0` / 动态库找不到**：`export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"`。
- **无显示器时段错误**：必须加 `--headless`；仍失败时用 `python -m pip install -e .` 重新编译 Isaac Gym 绑定。
- **OOM**：先 `--num_envs 1024`，再根据 `nvidia-smi` 显存占用往上加。
- **日志位置**：`ref_document/fudan_rl_wheel_leg-main/plane/logs/fifty_wheel_legged/`。
