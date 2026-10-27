# 带真实网格版本：关节修正（fifty_version_with_meshes）

本目录独立修复，不替换成简化几何的 `fourth_version`，也不修改 `fifty_version_with_meshes copy`（该目录仍是逐字节相同的原始 SW 导出，可作对照）。

## 修复前的问题

- 六个 joint 全部为 `type="fixed"`，`<axis xyz="0 0 0" />`；`export.log` 中每个 joint 都有 `Creating joint ... failed`。
- `controller_joint_names` 为 `['', ]`；`display.launch` 含未使用且无默认值的 `model` 参数，并引用不存在的 `urdf.rviz`。
- 关节名为 `l0_joint`、`l1_joint`、`ri_joint`（字母 i 与数字 1 混用）、`r0_joint`、`left_wheel`、`right_wheel`，与 fudan / `fourth_version` 的训练命名不一致。

## 修改表

| 导出关节名 | 现关节名 | 类型 | 局部轴 | 基座坐标系中的轴向（q=0） |
|---|---|---|---|---|
| l0_joint | lf0_Joint | revolute | 0 0 1 | 1 0 0 |
| l1_joint | lf1_Joint | revolute | 0 0 1 | -1 0 0 |
| left_wheel | l_wheel_Joint | continuous | 0 0 1 | 1 0 0 |
| r0_joint | rf0_Joint | revolute | 0 0 1 | -1 0 0 |
| ri_joint | rf1_Joint | revolute | 0 0 1 | -1 0 0 |
| right_wheel | r_wheel_Joint | continuous | 0 0 1 | -1 0 0 |

- 四个腿关节沿用 `lower=-100`、`upper=100` rad 的宽泛占位范围；轮关节改为 `continuous` 后删除上下限。六个关节统一 `effort=30` N·m、`velocity=1000` rad/s。
- **这些限位与驱动参数是占位值，不是本机标定结果。** 参考值：fudan `infantry_V4_increase.urdf` 大腿 `effort=50/velocity=1000`、小腿 `effort=50/velocity=1000` 且限位 -1.1..0.65 / -0.65..1.1、轮 `effort=5/velocity=1500`。正式训练前要按电机型号、减速比与机械限位重新给出。
- link 名（`base_link`、`l0_link`、`l1_link`、`left_wheel`、`r0_link`、`r1_link`、`right_wheel`）和 STL 文件名保持 SW 原名，便于和 CSV 中的 SW 零件清单对照；只改 joint 名。

## 为什么六个轴都是 0 0 1

本包保留 SW 坐标，没有做 fudan 的前后左右上下转换。按原始装配位置算出的坐标关系是：**X 为左右，+Z 向下，YZ 平面为腿摆动的矢状面**。

- 腿：`lf0_Joint`/`rf0_Joint` 的 origin rpy 为 `1.5707963267949 0 ±1.5707963267949`，把关节坐标系的 z 轴映射到基座的 ±X；`lf1_Joint`（rpy `-π 0 0`）与 `rf1_Joint`（rpy `0 0 π`）的 z 轴同样落在 ±X。因此腿关节局部轴取 `0 0 1` 即得到左右方向的俯仰轴。
- 轮：把 `left_wheel.STL`、`right_wheel.STL` 的共面三角形按面积分组，最大一组的法向为 `0 0 -1`（面积约 0.034 m²，即轮盘端面），说明轮子绕 link 局部 z 轴转动；轮关节 origin 的 rpy 为 `π 0 0`（左）和 `0 0 0`（右），局部轴 `0 0 1` 换算到基座后同样落在 ±X，与轮盘法向一致。
- 与 `fourth_version_with_meshes` 的差异：那一版右轮 origin 带 -0.0688 rad 偏转，需要写成 `0.0687283917617354 0 -0.997635408436291`；本版左右轮 origin 没有这种偏转，六个轴都可以直接写 `0 0 1`。

左右两侧的轴在基座坐标系里符号相反（左 +X、右 -X），这是 SW 导出时左右 joint 坐标系互为镜像造成的，不代表某一侧装反。它意味着**同号指令会让两侧关节朝世界坐标中的相反方向转动**，训练接入和电控必须使用同一张符号表，不能仅凭关节名假定同号同向。

## 几何与零位（q=0，基座坐标系）

| 项 | 数值 |
|---|---|
| 左右轮中心 | `±0.2645, 0.0229, 0.5088` m |
| 轮距（沿 X） | 0.5290 m |
| 轮半径（网格包围盒推得） | 0.0600 m |
| 六关节全部 q=0 时整体包围盒 | x ±0.2870、y ±0.3110、z -0.0710..0.5688 m |
| 总质量 | 23.259630516 kg |

轮半径来自网格，没有和实物测量核对；轮组是否完整、是否包含全部胎面也要另行确认。不要拿 `fourth_version` 里 0.075 m 的代理轮半径覆盖这里的数值。

## 保持不变的项

- 七个 link 的名称、质量、质心、惯量矩阵，以及 visual/collision 的 origin 与网格引用。
- 全部 joint 的 parent/child、origin xyz 与 rpy。
- 七个 STL 的每一个字节（SHA256 见修复报告）。
- q=0 时与 SW 装配一致的外形与位置。

## 写回了哪些文件

| 文件 | 修改内容 |
|---|---|
| `urdf/fifty_version_with_meshes.urdf` | joint 名称、类型、有效轴向、限位 |
| `urdf/fifty_version_with_meshes.csv` | 同步 Joint Name、Joint Type、Joint Axis Z、Limit Effort/Velocity/Lower/Upper（共 38 个单元格，其余单元格逐字段未变） |
| `config/joint_names_fifty_version_with_meshes.yaml` | 填入六个关节名，顺序按 fudan（右侧在前） |
| `launch/display.launch` | 删除无用 `model` 参数，改为引用实际存在的 `config/display.rviz` |
| `config/display.rviz` | 新增 Grid、RobotModel、TF 显示 |
| `config/joint_fix_report.json` | 轴向证据、网格哈希与三角形数、运动检查、CSV 变更清单、备份路径 |
| `tools/fix_fifty_joints.py` | 本次修复脚本（备份、改写、校验）；检测到已有活动关节时会直接退出，避免重复运行破坏模型 |

没有修改 SW 文档内部的 URDF Export Configuration。之后再次从 SW 导出时应检查插件里保存的关节类型与轴向，否则会覆盖本次修正。

## 已做的检查

1. XML 可解析，7 个 link、6 个 joint，单根树结构，六个关节不再是 fixed。
2. 六个轴均为单位向量，换算到基座后都落在 ±X。
3. 7 个 STL 都是完整二进制 STL（`84 + 50 × 三角形数` 等于文件大小），顶点数值有限，SHA256 与修复报告一致。
4. 各 link 质量为正，惯量矩阵正定且满足主惯量三角不等式。
5. link 定义与 joint/origin 与备份逐项比对一致，q=0 装配外形未变。
6. 每个关节取 0.2 rad，用正运动学验证子 link 网格顶点确实移动（位移 0.012–0.056 m）。这是运动定义检查，不是物理仿真步进。
7. 写回后重新解析并核对报告，`post_install_validation_passed=true`。

## 未做的验证与仍存在的限制

- 未运行 ROS/RViz、Gazebo、Isaac Gym；`runtime_tested=false`。
- 未验证碰撞导入性能：`base_link.STL` 约 102 万个三角形（51 MB），且当前 visual 与 collision 使用同一份 STL。训练前应另建简化碰撞体，但不能用改质量的方式代替碰撞精简。
- 云台仍刚性并在 `base_link` 内，没有 yaw/pitch 自由度；已合并进 link 的连杆与气弹簧随该 link 刚性运动。
- 本版仍是 SW 坐标与 SW 零位，没有做 fudan 的 X 前 / Y 左 / Z 上转换，也没有重定义零位。接入训练时仍需按名称、顺序、坐标、零位、正方向逐项对齐，不能直接把 `fourth_version` 的训练配置拿来用。

## 查看

ROS 1 环境把本包放进工作空间并 source 后：

```text
roslaunch fifty_version_with_meshes display.launch
```

在 joint_state_publisher_gui 里给小角度（如 0.1 或 0.2 rad）逐个检查六个关节；输入 0 回到 SW 导出时的装配姿态。
