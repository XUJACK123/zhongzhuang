# 第四版：复旦六自由度训练用简化模型

本版按用户要求修改 URDF，采用简化几何体，现有质量、质心和惯量作为输入依据。没有下载、安装软件，没有修改原 STEP、SLDASM 或复旦源码。

## 已完成与验证边界

- 7 个 link，4 个 revolute 腿关节、2 个 continuous 轮关节；轴均为各自局部 Z。
- 采用复旦名称；期望加载顺序：lf0_Joint、lf1_Joint、l_wheel_Joint、rf0_Joint、rf1_Joint、r_wheel_Joint。
- XML 和树结构检查、100 组随机姿态与复旦平面运动学公式比较通过。
- 质量总计 25.69460514566614 kg。每个 link 的质量、主惯量保持不变；惯量矩阵元素随坐标变换而改变。
- 原 CAD 姿态下，变换后的各 link 质心与惯量空间姿态可重建，已数值验证。
- 未进行 Isaac Gym 加载、接触、站立、PPO 训练或实机验证。本模型不表示现有 ONNX 策略可以直接迁移。

## 坐标、零位与站姿

机身原点取原 URDF 两髋轴心的中点。新 X=-旧 Y，新 Y=-旧 X，新 Z=-旧 Z，因此右侧在负 Y、左侧在正 Y。该前方选择符合用户允许任选前向的要求。

大腿零位沿机身前方；小腿在零位时向下，与大腿成直角。不是简单把原 URDF 的 RPY 清零，而是同时转换各 link 的质心、惯量和坐标。髋间距 0.4554 m，左右大腿平面长度均 0.21 m，小腿平面长度均 0.25 m。原始横向轴心偏移保留。

为满足复旦 theta1/theta2 公式，右、左关节符号相反。右轮的关节坐标系按平面腿轴重新对齐，原惯量方向同步转换。独立训练入口设左右髋初始角 +0.6986598247 / -0.6986598247 rad，膝关节初始角 0；轮心在对应髋轴正下方。初始髋高度约 0.41150 m，含 0.01 m 离地余量。

原 CAD 装配姿态角、原 link 到新 link 的变换、质量主惯量及具体参数记录在 config/model_parameters.json。若以后导入按原 link 坐标导出的 STL，必须使用其中 old_mesh_frame_transforms，不能把所有 mesh origin 清零。

## 明确的几何近似

这些尺寸不是从 STEP 测量得到的，不参与重算质量或惯量：

| 部位 | 代理外形/碰撞 |
|---|---|
| 机身 | 0.32 × 0.40 × 0.18 m 长方体，中心位于现有机身质心 |
| 大腿 | 跨度采用实际两轴心距离，截面 0.040 × 0.025 m |
| 小腿 | 跨度采用实际两轴心距离，截面 0.030 × 0.022 m |
| 轮子 | 半径 0.075 m、宽 0.035 m 的圆柱 |

质量与上述均匀几何体的理论惯量不要求一致：模型使用的是原 URDF 惯性数据，几何体仅用于外观和接触近似。轮半径会直接影响速度与接触，后续获得真实值必须更新。

目前不需要任何 STL。未从 320 MB 的 STEP 分割或生成精确网格。云台合并于机身，未增加 yaw/pitch，维持复旦的六维动作及七刚体观测结构。传动杆和气弹簧继续作为既有质量分组的近似，不模拟闭环杆件、弹簧伸缩或弹簧力。

## 限位与已有数据

按“默认现有数据正确”的要求，沿用原有每关节 effort=30 N·m、velocity=1000 rad/s，四个腿关节角度范围 [-100,100] rad。轮关节改成 continuous，移除角度上下限。此处保留不代表这些参数已被实机校准；尤其宽泛的腿限位不能模拟真实机械挡块。没有用参考机器人的质量或电机参数覆盖本机数据。

原 SW CSV 是修改前的导出配置快照，未改写；不能把它当作本次转换后的插件配置再次加载后覆盖当前 URDF。原文件的逐字节备份在 backups 子目录。

## 使用与训练

单独替换参考 URDF 不够：复旦默认使用另一台机器的腿长、关节初始角和模型路径。本目录 integration/fudan_train.py 是独立适配入口，不修改复旦原有任务。

先在任意 Python 3 环境进行静态检查（不需要第三方依赖）：

```text
python integration/fudan_train.py --validate-only
```

在已有 Isaac Gym / PyTorch / 复旦依赖的训练环境，将整个 fourth_version 文件夹复制过去，再运行（将路径替换为真实路径）：

```text
python /path/fourth_version/integration/fudan_train.py --fudan-root /path/fudan_rl_wheel_leg-main --headless --smoke-test
```

此检查创建 1 个环境、关闭已知随机化、执行 100 次零动作步进，打印自由度顺序、PD 参数、重置次数，检查状态有限性。不把完成检查等同于站稳。

确认加载与接触正常后，训练命令：

```text
python /path/fourth_version/integration/fudan_train.py --fudan-root /path/fudan_rl_wheel_leg-main --headless --num_envs 1024
```

入口使用 25 维观测和原有历史接口；更新本机腿长、初始高度、轮接触名称，关闭圆柱转胶囊，默认以低速指令开始。保留原 PD 增益和 PPO 算法，不声称训练一定收敛。实际自由度顺序不符时立即报错，避免错接动作。已有策略权重并未在本次验证，不能把本适配入口视作已完成的教师迁移流程。

## 查看

ROS 1 环境安装本包后可运行 roslaunch fourth_version display.launch，先由 joint_state_publisher_gui 拖动关节检查轴向。该 launch 已配置站姿初始角及 RViz 显示文件；若所用 joint_state_publisher_gui 版本未采用 zeros 参数，请手动设置上述髋关节角。Gazebo launch 只按站姿生成模型，未配置电机控制器，机器人可能倒下，不等于复旦训练入口。ROS / RViz / Gazebo 启动均未在本机实测。
