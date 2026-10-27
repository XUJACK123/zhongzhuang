# fifty_version_training

从 `../fifty_version_with_meshes`（SolidWorks 导出 + 关节修复）派生的**训练用等效串联六自由度模型**，
目标是直接接复旦 `fudan_rl_wheel_leg-main/plane` 的 25 维观测 / 125 维历史 / 6 维动作接口。

源包不改动；本目录由 `tools/build_training_urdf.py` 生成，可重复构建。

## 当前状态

| 项目 | 状态 |
|---|---|
| 坐标系、零位、命名、DOF 顺序 | 已完成，17 项静态检查通过 |
| 网格（STL） | 与源包逐字节一致（SHA256 校验） |
| 基本体碰撞 | 已生成（机身 box、腿 cylinder、轮 cylinder） |
| Isaac Gym 加载 / 接触 / 站立 | **未运行**（本机无 Isaac Gym） |
| 五连杆闭链、气弹簧、电机映射 | **未建模**，属于 G4/G5 |

## 快速开始

```bash
# 1) 静态校验（无需 Isaac Gym）
python3 integration/validate_model.py

# 2) 重新生成（改了轮径、限位、前向符号之后）
python3 tools/build_training_urdf.py --force

# 3) 在 Isaac Gym 环境里冒烟（需要 Preview 4 + torch）
#    本机 RTX 5060 是 sm_120，Isaac Gym Preview 4 只能用 CPU 管线，见 docs/ENV_REPORT.md
python3 integration/train_fifty.py --smoke-test --fudan-root ../../ref_document/fudan_rl_wheel_leg-main \
  --headless --sim_device=cpu --pipeline=cpu

# 4) 逐关节方向检查
python3 integration/train_fifty.py --joint-test --fudan-root ../../ref_document/fudan_rl_wheel_leg-main \
  --headless --sim_device=cpu --pipeline=cpu

# 5) 正式训练
python3 integration/train_fifty.py --fudan-root ../../ref_document/fudan_rl_wheel_leg-main --headless
```

## 机构说明（为什么是 6 个"虚拟"关节）

真实腿是**五连杆并联**：髋部两个同轴电机，一个直驱大腿，一个经推杆/摇臂驱动小腿，
小腿铰链本身是被动关节，另有气弹簧。本模型用等效串联 6 关节描述同样的 2 自由度/腿：

- 训练、观测、动作、PD 都作用在虚拟关节上（与复旦权重兼容）；
- 电机角 → 虚拟关节角、雅可比、力矩映射、气弹簧补偿由部署侧（G4）完成；
- MuJoCo 详细模型（G5）才用 `<equality><connect>` 表达闭链。

详见 `docs/CONVERSION_NOTES.md`。

## 目录

```
fifty_version_training/
├─ urdf/fifty_training.urdf          训练模型
├─ meshes/*.STL                      指向源 STL 的硬链接
├─ config/model_parameters.json      变换/惯量/符号/限位/默认角
├─ config/joint_names_fifty_training.yaml
├─ config/build_report.json          构建校验结果
├─ tools/build_training_urdf.py      转换脚本
├─ integration/validate_model.py     无依赖校验
├─ integration/train_fifty.py        Isaac Gym 适配器
└─ docs/                             参数表与转换说明
```

## 安全边界

`model_parameters.json` 的 `provisional` 段列出的值（轮半径、关节限位、effort/velocity、
机身碰撞盒、base 惯量、前向方向）**都还没有实物依据**。在拿到轮径实测、电机参数、
关节可达范围之前，不要把这些数值当成标定结果，也不要把仿真结果当成实车验证。
