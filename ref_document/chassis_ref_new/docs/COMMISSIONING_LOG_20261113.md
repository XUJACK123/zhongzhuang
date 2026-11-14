# 轮腿底盘改动记录（实时更新）

本文件按时间顺序记录每一次源码／工具改动、目的与验证结果。它只记录**实际发生的改动**；
未验证的推断、待现场测量的数据不会写成已验证结论。放行标志的当前值仍以
[robot_config.h](../user/robot_config.h) 和 [实机联调记录](BOARD_BRINGUP_20261113.md) 为准。

状态标记：`已修改`（代码已改，未验证）→ `主机通过`（`make test`／`py_compile`）→
`实机通过`（板卡上读取到对应行为）。

## 当前目标

让轮腿可以真正上手操作：按“方向 → 参考位 → IMU → 里程计 → 受限输出”的顺序完成标定，
并把需要现场配合的步骤压缩到最少。

## 2026-11-13

### 23:00 板卡状态复核（只读）

- `make monitor`（`python tools/monitor_can.py --interval 1`）确认：
  - 六个反馈 ID 全部有新帧：`0x304/0x303/0x203/0x301/0x302/0x202`。
  - `chassis.inhibit=33`（1=未配置放行，32=无关节几何），`nonzero_torque_frames=0`，`enable_commands=0`。
  - `imu.calibrated=true`，`max_gap_ms=1`，`errors=0`；遥控器 `valid_frames` 持续增长。
  - 证据：[diagnostics/live_can_20261113_230001/result.json](../diagnostics/live_can_20261113_230001/result.json)。
- 结论：板上仍是 `build/rl/wheel_leg_rl.elf`（BIN SHA256 `7665ba7f…4789`），只发零输出。

### 23:05 `tools/monitor_can.py` 重构为可复用模块

- 改动：把 OpenOCD 采样与解码拆成模块级函数 `elf_symbols()`、`openocd_argv()`、`dump_ram()`、
  `decode()`、`snapshot()`；`main()` 行为、`live_can_*` 目录结构、`result.json` 字段保持不变。
- 目的：让标定工具复用同一套“读 RAM + 向量表指纹校验”逻辑，避免第二份实现走样。
- 状态：`已修改`；`py_compile` 通过，`make monitor` 待复跑确认。

### 23:05 `tools/calibration_values.py` 提取宏计算函数

- 改动：新增 `arm_from_joint()`、`macros()`、`define_line()`；`measured_pose()` 与命令行输出格式不变
  （`--torch-check` 之外的既有用法已回归验证：`--joint-deg`、`--measured-leg-angles` 两种输入各自输出三条宏）。
- 目的：`commission.py` 复用同一份装配分支／雅可比公式，防止两处公式漂移。
- 状态：`主机通过`（`tests/test_host.py` 仍从本模块导入 `measured_pose`）。

### 23:05 新增 `tools/commission.py`（标定流程工具）

- 子命令：
  - `capture --label X`：只读抓一次 RAM 快照，按策略顺序打印原始计数／展开计数／温度／状态字节，
    写入 `diagnostics/commission_<时间>/`，供后续 `delta` 比较。
  - `delta --before X --after Y`：输出每个电机的原始计数变化、跨 8191/0 的回绕差和展开计数差，
    并给出 `+1/-1/?` 的方向提示。
  - `apply --signs … --counts … (--angles 4 | --joint-deg 6 | --at-training-default) [--dry-run]`：
    计算三条标定宏并只改写 `user/robot_config.h` 里的 `ROBOT_ENCODER_SIGNS`、
    `ROBOT_ENCODER_REFERENCE_RAD`、`ROBOT_ARM_REFERENCE_RAD`。
- 安全边界：工具不烧录、不使能电机、不发送任何运动命令、不改动五个放行标志；
  `apply` 前会打印旧／新宏，`--dry-run` 只打印不落盘。
- 状态：`已修改`；`py_compile` 通过。

### 23:08–23:15 人工动作录制（六路编码器方向）

- 工具：`python tools/commission.py watch --seconds 180 --interval 0.5`。一次 OpenOCD 会话内用生成的
  `watch.tcl` 循环 `dump_image`，避免 Windows 命令行过长（`WinError 206`）；
  采样后自动按“峰值偏离中位数”给出每次人工动作的方向与幅度。
- 数据：[watch_signs/watch.json](../diagnostics/commission_20261113_230207/watch_signs/watch.json)。
- 现场确认：用户把**左右大腿都向上抬**（这是规范正向的反方向），小腿因气弹簧支撑**无法用手单独扳动**。

|索引|电机|测到的动作|展开计数变化|折合电机角|说明|
|---|---|---|---|---|---|
|0|右髋 DM4|大腿向上抬|`+594`|+26.1°|规范正向应为向下，故 `sign=-1`|
|3|左髋 DM1|大腿向上抬|`-472`|-20.7°|规范正向应为向下，故 `sign=+1`|
|2|右轮 M3508 ID3|按前进方向滚动|`+50322`|+2211°（轮轴约+139°）|待确认滚动方向后 `sign=+1`|
|5|左轮 M3508 ID2|按前进方向滚动|`-37947`|-1668°（轮轴约-105°）|待确认滚动方向后 `sign=-1`|
|1|右膝 DM3|—|`+599`|+26.3°|与右髋同时变化，是闭链连杆被动带动，不是小腿独立运动|
|4|左膝 DM2|—|`-449`|-19.7°|与左髋同时变化，同上|

- 结论：**髋与轮的方向已测到，膝没有独立测到**。膝读数与髋读数几乎 1:1 联动，说明当前姿态下
  小腿被气弹簧顶在伸展端，抬大腿时只是连杆带动曲柄，不能用它推膝的编码器方向。
- 因此膝方向需要一次“受载折叠”测试：轮子着地，用手/体重把车体向下压，让腿真正折叠。

### 23:26–23:29 右膝独立折叠（第二次录制成功）

- 数据：[watch_fold_right2/watch.json](../diagnostics/commission_20261113_230207/watch_fold_right2/watch.json)。
- 过程：0–46 秒完全静止（`hip=4068, knee=3718`）；47–50 秒先把右大腿抬起约 130°（`hip→6546`，
  `knee→6185`，`knee−hip` 基本不变，属整条腿刚性转动）；50–65 秒保持大腿不动、反复把右小腿折起来，
  `knee` 在 4715…6391 之间变化，而 `hip` 只在 6546…7359 小幅浮动。
- 取 49 秒（`hip=6546, knee=6185`）到 64 秒（`hip=7142, knee=4715`）：
  `Δm_hip=+596`、`Δm_knee=−1470`。
- 由 `alpha = arm_knee − arm_hip`、`arm = arm_ref + sign·(m − m_ref)`：
  `Δalpha = sign_knee·Δm_knee − sign_hip·Δm_hip = sign_knee·(−1470) + 596`。
  - `sign_knee=+1` → `Δalpha=−874`（折叠，膝内夹角变小）✔ 与人工动作一致
  - `sign_knee=−1` → `Δalpha=+2066`（伸展）✘
- 结论：**右膝 `sign=+1`**（用户已确认 50–65 秒确实是“折起来”）。
- 左膝尚未做同样的独立折叠，仍缺。

### 23:35–23:38 左膝折叠 + 两轮方向

- 数据：[watch_leftknee_wheels/watch.json](../diagnostics/commission_20261113_230207/watch_leftknee_wheels/watch.json)。
- 左腿：2 秒内整体抬起（`hip 826→−1728`，`knee 3233→685`，`knee−hip` 仍约 2410，刚性转动）；
  12–28 秒大腿基本保持，小腿被折动，`knee−hip` 从 `2417`（10 秒）降到 `743`（18 秒）。
  取 10 秒→18 秒：`Δm_hip=+471`、`Δm_knee=−1203`；
  `Δalpha = sign_knee·(−1203) − sign_hip·(+471)`，左髋 `sign=+1`：
  - `sign_knee=+1` → `−1674`（折叠）✔　`sign_knee=−1` → `+732`（伸展）✘
  - 结论：**左膝 `sign=+1`**。
- 两轮：本轮右轮 `+624165`、左轮 `−309113` 计数；与 23:08 那轮的 `+50322 / −37947` 方向一致。
  即：右轮按前进方向滚动时计数增大，左轮按前进方向滚动时计数减小（**待用户最终确认两次都是“前进方向”**）。
- 数据质量问题：采样中出现过一次 `counts=4294964075`（64 位读被撕裂的低 32 位值），属调试器读 RAM 的瞬时撕裂，
  不代表固件计数错误；分析时按“超出 ±2^31 即丢弃”处理。

### 目前得到的方向（策略索引顺序）

|索引|电机|sign|依据|
|---|---|---|---|
|0|右髋 DM4|**−1**|大腿向上抬 → 计数 +594|
|1|右膝 DM3|**+1**|抬起后独立折叠 → `knee−hip` 下降|
|2|右轮 M3508 ID3|**+1**|前进方向滚动 → 计数增大（两轮实验一致）|
|3|左髋 DM1|**+1**|大腿向上抬 → 计数 −472 / −2554|
|4|左膝 DM2|**+1**|抬起后独立折叠 → `knee−hip` 下降|
|5|左轮 M3508 ID2|**−1**|前进方向滚动 → 计数减小（两轮实验一致）|

**还缺的是绝对参考位**：`ROBOT_ENCODER_REFERENCE_RAD` 与 `ROBOT_ARM_REFERENCE_RAD` 需要同一个真实姿态的
四个机械角。方向只解决“正负”，不解决“零点”。

### 23:45 新增 `commission.py pose`：用两个长度测量代替量角器

- 动机：机械臂被气弹簧顶住，人工摆角+量角器既费力又难复现。改为测两个**长度**并用固件同款正运动学反解。
- 运动学（与 [state_estimator.c](../user/state_estimator.c) 完全一致）：
  `X = 0.21cos(a) + 0.25cos(b)`，`Z = −0.21sin(a) − 0.25sin(b)`，
  其中 `a = s·q_hip`，`b = s·(q_hip+q_knee) + π/2`，大腿 0.21 m、小腿 0.25 m。
- 输入：髋轴离地高度 `--hip-height`（mm）、轮心相对髋轴的前后偏移 `--wheel-x`（mm，前为正）、轮半径
  `--wheel-radius`（默认 55 mm）。工具按站姿算出轮心相对髋轴的 `(x, z)`，反解 4 个关节角，
  再用归档装配分支 `alpha ∈ (−2π, 0)` 选解，最后输出三条标定宏；`--write` 才落盘。
- 自检：`ik_leg` 对 `q=[2.7251,−3.6012]` 等样本往返误差 0.0，不可达目标返回 `None`。
- 状态：`主机通过`（数值往返），待现场测量后实际写入。

### 23:55 写入标定并放行 MOTOR_CALIBRATED

- 现场测量（用户用卷尺量，两轮着地、车身用支架托住）：**髋轴离地 300 mm、髋轴心到轮轴心直线距离 350 mm、轮心在髋轴后方**。
  反解得 `q = [+3.1501, −2.9896, 0, −3.1501, +2.9896, 0]`，膝内夹角约 99°，在归档范围 24°–123° 内。
- 写入 [user/robot_config.h](../user/robot_config.h)：
  - `ROBOT_ENCODER_SIGNS = {−1, 1, 1, 1, 1, −1}`
  - `ROBOT_ENCODER_REFERENCE_RAD = {2.7511945431, 2.4643401357, 0.1742633086, 1.0385049934, 2.8731460157, 0.1278478320}`
  - `ROBOT_ARM_REFERENCE_RAD = {3.1500981078, 1.5822922061, 0, 3.1500981078, 1.5822922061, 0}`
  - `ROBOT_MOTOR_CALIBRATED` 0 → **1**；`ROBOT_ENABLE_OUTPUT` 仍为 **0**，其余三个标志仍为 0。
- 构建：`make -j4` 通过，FLASH 101,080 B、DTCM 15,496 B；`make test` 通过。
- 烧录：[flash 记录](../diagnostics/flash_*/result.json)，OpenOCD 返回 `Verified OK` 并复位运行。
- 实机回读（[live_can_20261114_000202](../diagnostics/live_can_20261114_000202/result.json)）：
  - `joint_position_rad = [3.1501, −2.9896, 0, −3.1501, 2.9896, 0]`，与反解姿态完全一致；
  - `policy_runs` 持续增长（影子推理），`inference_us≈408 µs`、`max_loop_us≈446 µs`、`overruns=0`；
  - `inhibit=257` = `1`（输出未放行）+ `256`（当前姿态离训练默认角超过 0.30 rad），无其它故障位；
  - 六路 CAN 全部有新帧，IMU 已校准、最大间隔 1 ms。

### 00:05 主机测试改为不再依赖“占位标定”

- 原因：`tests/test_logic.c` 原来假设 `ROBOT_ENCODER_SIGNS` 全为 +1；`tools/test_host.py` 的假 HAL 用文本替换
  假设 `ROBOT_ARM_REFERENCE_RAD` 是 `{0,0,0,0,0,0}`。真实标定写入后两处都失效。
- 改动：
  - `tests/test_logic.c` 新增 `cfg_signs[6]=ROBOT_ENCODER_SIGNS`，编码回读和臂角增量都用配置里的符号校验；
  - `tools/test_host.py` 用正则把三条标定宏替换成**合成标定**（signs 全 +1、编码器参考 0、臂参考=默认姿态），
    使假 HAL 的“零编码器计数 = 训练默认姿态”假设与真实配置解耦。
- 状态：`主机通过`（`make test` 全绿），生产 `robot_config.h` 不被测试修改。

### 00:20 方向端到端复核（实机）

- 数据：[watch_verify_signs/watch.json](../diagnostics/commission_20261113_230207/watch_verify_signs/watch.json)，
  采样含 `g_measurement.joint_position_rad`。
- 右腿：人工把右大腿抬起约 28°，`q[0]` 由 `3.150 → 2.668`（**减小**），与“规范正向=大腿朝下”一致 ✔。
- 左腿：人工抬起左大腿约 139°，`q[3]` 由 `−3.150 → −0.713`（即规范臂角减小 2.44 rad）✔。
- 结论：符号与参考位在固件里端到端自洽，关节角随真实姿态按预期方向和幅度变化。
- 备注：采样中 `q[3]` 有一次单点跳变（同一计数值下读到相反符号），属调试器读 RAM 的撕裂，
  不是固件计算问题；分析时按单点异常剔除即可。

## 当前放行状态（2026-11-14 00:50）

|标志|值|依据|
|---|---|---|
|`ROBOT_ENABLE_OUTPUT`|**1**|已按用户要求简化为单一开关，S1 下档=零电流；实机已出过力（见 00:50）|
|`ROBOT_MOTOR_CALIBRATED`|**1**|六路方向实测 + 参考位反解 + 实机回读与反解一致|
|`ROBOT_IMU_FRAME_VERIFIED`|**1**|三轴方向人工核对 + 整圈 yaw 复核（见下）|
|`ROBOT_POLICY_CONTRACT_VERIFIED`|0|原训练 `legged_robot.py` 仍缺，观测/参考推进时序未与原环境对拍|
|`ROBOT_ODOMETRY_VERIFIED`|0|方向已正确、尺度与轮转角一致，但缺一次低打滑的地面真值推行|

### 00:10 yaw 整圈复核 → `IMU_FRAME_VERIFIED=1`

- 数据：[watch_yaw360/watch.json](../diagnostics/commission_20261113_230207/watch_yaw360/watch.json)。
- 人工从地面参照线左转**整一圈**回到同一线：0.5 s 采样逐点解缠后累计 **+362.24°**（理想 360°），
  比例偏差 **+0.62%**；最大单步 82°（远小于 180°，无混叠）。
- 静止段（12–59 s）yaw 漂移约 **0.02°/s**（此前 5.5 s 窗口测到 0.006°/s，短窗口不代表长期上界）。
- 判定：方向与比例都经过外部参照复核，比例误差在人工对线误差量级内，因此写入 `ROBOT_IMU_FRAME_VERIFIED=1`
  并重新烧录（`Verified OK`，回读 `inhibit=257`、影子推理 828 次、`overruns=0`）。

### 00:12 里程计测试揭出轮子符号错误

- 第一次推行（[watch_odometry1m](../diagnostics/commission_20261113_230207/watch_odometry1m/watch.json)）：
  人工前推，右轮计数 **减少** −275 376、左轮计数 **增加** +268 978；
  `root_xy` 走 **−0.7165 m**，方向反了。幅值 0.7165 m 与轮子转角折算的 0.731 m 相差 2%，
  反过来验证了 15.88 齿比与 0.055 m 轮径。
- 结论：先前按“手滚轮子”判断的轮子符号不可靠，**以地面推行方向为准**。
  `ROBOT_ENCODER_SIGNS` 由 `{−1,1,1,1,1,−1}` 改为 **`{−1,1,−1,1,1,1}`**，重新构建、烧录、`make test` 全绿。
- 注意：改符号同时改变**力矩指令方向**（反馈与输出共用同一 sign），这是符号统一的要求。

### 00:14 里程计复测（方向已正确，尺度待定）

|轮次|`root_xy` ΔX|右轮转角折算|左轮转角折算|yaw 变化|判断|
|---|---:|---:|---:|---:|---|
|b|[watch_odometry1m_b](../diagnostics/commission_20261113_230207/watch_odometry1m_b/watch.json) +0.5735 m|0.967 m|1.090 m|+8.4°|方向✔，轮子明显打滑（轮转过量）|
|c|[watch_odometry1m_c](../diagnostics/commission_20261113_230207/watch_odometry1m_c/watch.json) +0.5325 m|0.835 m|0.238 m|−5.3°|方向✔，两轮转角差 3.5 倍，打滑/离地|

- 两次都是人手推行：轮子转速与车体真实位移不一致，无法作为无滑移真值；因此
  **`ODOMETRY_VERIFIED` 保持 0**。
- 现有证据：方向正确；尺度与轮转角在第一次（两轮转速接近时）吻合到 2%。缺的是**低滑移的地面真值一次**——
  最可靠的做法是等策略使能后用自身轮速匀速走 1 m 复核，或把车放在能承重的推车上推行。

### 00:19 监控工具增加 `g_rl` 输出解码

- `tools/monitor_can.py` 符号表加入 `g_rl`（ARM 上 1164 B），新增 `rl.action / command / reference_yaw /
  target_q / target_qd / joint_torque / motor_torque` 解码，可直接看策略算出的电机力矩。
- 实机验证（[live_can_20261114_001959](../diagnostics/live_can_20261114_001959/result.json)）：
  `mode=0`、`inhibit=257`、`nonzero_torque_frames=0`、`enable_commands=0`，
  而 `motor_torque=[12,−12,−2,12,−12,−2]` 已饱和——影子策略在算、力矩没有发出去，
  原因是当前姿态远离训练分布（`action` 打满 ±1）。

### 00:50 左侧腿不出力：卡在气弹簧力矩，不在通信

录制 [watch_left_leg_under_torque](../diagnostics/commission_20261113_230207/watch_left_leg_under_torque/)：
51 个采样、0.2 s 间隔，全程 `mode=3`、`inhibit=0`。同时读六个电机的**指令力矩**、**DM 反馈力矩**
（`MotorFeedback.current` 取自 DM 反馈字节 4–5）和编码器展开计数。

|关节|指令（电机空间）|DM 反馈力矩|12 s 内计数变化|
|---|---:|---:|---:|
|右髋|−0.4 … −1.1 Nm|+0.45 Nm|0|
|右膝|−6.1 Nm|−4.84 Nm|0|
|左髋|+2.8 Nm|+2.4 Nm|+8|
|左膝|−7.4 Nm|−5.88 Nm|+8|

- **左侧确实在出力**：左膝报出 5.88 Nm、左髋 2.4 Nm，符号与指令一致。所以既不是"左侧没输出"，
  也不是 DM `0x0D` 通信（`inhibit=0`、`mode=3`、`policy_runs=1810`、`nonzero_torque_frames=18076`）。
- **两条腿的编码器 12 s 内基本不动**（±8 计数 = 0.35°），而关节力矩有 6–8.5 Nm
  → 机构被外部力矩顶住，不是电机没劲。
- 断使能瞬间左侧腿**弹回**：`arm_hipL 2.816→3.153`、`arm_kneeL 1.221→1.587`，
  正好回到 `ROBOT_ARM_REFERENCE_RAD`。无电时机构停在这个"站立参考位"，正是气弹簧把小腿顶在伸展端的位置；
  有电时策略把它压出去 19°，一松又弹回。
- 力矩链自检（[rl.c](../user/algorithm/rl.c)）：膝 `kp=60 Nm/rad`，误差 0.17 rad → 关节 8.5 Nm
  → 电机空间 `motor[k]=r·tau[k]`（`r≈0.863`）→ 7.37 Nm，与 `g_rl.motor_torque` 逐位一致。
  符号、雅可比、限幅都对，卡住的是**幅值**。
- 训练默认膝角 ±3.661 rad，实车自然位 ±3.0 rad，差 0.65 rad。策略一上电就 `action` 打满 ±1
  （`target_q` 顶到 ∓3.161），要求把膝再折 0.17–0.65 rad，必须压缩气弹簧；之前 10 Nm（电机空间）不够。
- 附注：DM 反馈力矩比指令低约 20%（5.88 / 7.37），提示真实 `Kt` 更接近 1.0 Nm/A，即指令力矩实际只兑现八成。
  本次不动 `ROBOT_DM_KT_NM_A`（来自 DM8009J_test 组控标定），只把上限开到训练额定。

改动：[user/robot_config.h](../user/robot_config.h) `ROBOT_LEG_TORQUE_CAP_NM` 10.0 → **20.0**（训练额定），
`ROBOT_WHEEL_TORQUE_CAP_NM` 保持 2.0。`make test` 全绿、交叉构建通过；**尚未烧录**（用户已断电充电）。

现场动作：上电 → 烧录 → 整车架空、四腿悬空、人手不碰腿，S1 上档看左右膝能否折向默认姿态；
用 `commission.py watch` 记 `arm/current` 判断是否仍被弹簧顶住。若 20 Nm 仍不动，
说明有效力矩仍小于气弹簧力矩或存在硬限位，需要分级给定实测拐点。

## 下一步（需要现场）

### 00:40 按用户要求简化输出授权

- 用户要求："只需要有一个最下档的是 0 电流的就行了"。据此改写控制入口：
  - [user/control_state.c](../user/control_state.c)：**S1 下档 = STOP（零电流）**，其余位置直接 ENABLING→ACTIVE；
    去掉 S2 中档要求、摇杆居中要求、300 ms 驻留和 READY 档。故障/遥控丢失后需再经过一次下档才重新允许运行。
  - [user/chassis.c](../user/chassis.c)：`configured` 只由 `ROBOT_ENABLE_OUTPUT` 决定，删掉四个标志的串联门禁；
    删掉"起始姿态离默认角 0.30 rad"的 `inhibit` 位 256；`inhibit` 位 16 不再包含 `rx_overflow`（反馈超时已覆盖丢帧）。
  - 保留的硬件保护：遥控 100 ms、电机反馈 30 ms、IMU 5 ms、倾角、超温、CAN bus-off、几何/膝限位、策略自检、周期超时。
  - `robot_config.h` 删除临时 `ROBOT_BENCH_TEST`；`ENABLE_OUTPUT=1`，力矩上限保持台架值 **腿 3 Nm / 轮 0.3 Nm**。
- 主机测试同步更新：[tests/test_logic.c](../tests/test_logic.c) 改为"下档=停机/其余=运行/故障与遥控丢失需重新下档"，
  [tests/test_chassis.c](../tests/test_chassis.c) 的解锁序列改为下档→上档，[tools/test_host.py](../tools/test_host.py)
  给两个假 HAL 变体各自生成 fixture（禁用变体强制 `ENABLE_OUTPUT=0`，不再依赖生产开关）。
- 状态：`主机通过`（`make test` 全绿）、构建零警告、烧录 `Verified OK`；烧录后 `--fast` 回读 `inhibit=0`。

### 01:00 烧录 20 Nm 版后：右轮 M3508 无反馈

- `make flash` 成功（`Verified OK`），但随后的 [watch_freeair_cap20](../diagnostics/commission_20261113_230207/watch_freeair_cap20/)
  只录到 15 个采样（OpenOCD 报 `external reset detected`），且全程 `mode=0`、`inhibit=36`
  = 位 4（电机不健康）+ 位 32（几何未算出）。
- 逐节点读 `MotorFeedback`：`wheelR(0x203) recv=0`，`tick=0`、温度 0、状态 0x00；
  同一时刻 `wheelL(0x202) recv=3880`，四个 DM 全部正常（`recv` 3700–3900，24–26 ℃）。
  → **右侧 M3508 完全没有上电/上报**，不是解码或滤波问题（左轮在同一条总线上正常）。
- 因为 `motor_feedback_healthy()` 要求六路全新鲜，位 4 置位后 `estimator_update` 被跳过，
  几何随之置位 32，`inhibit!=0` 使状态机永远停在 STOP——这就是这一轮"又不动"的直接原因。
- 所有节点的 `position_lost` 都是 0，说明调试器读 RAM 没有触发圈数丢失，问题纯在右轮。

### 01:12 监控工具会把 MCU 停在 halt 状态（已修）

- 现象：连续两次 `--brief`/`--fast` 读到**完全相同**的 `tick`、`received`、`valid_frames`
  （例如 `tick=4306 received=20641 valid_frames=308` 反复出现），看起来像固件挂了。
- 直接读内核寄存器（`halt` + `reg pc/sp`）得到 `pc=0x08000a8c`、`lr=0x08000a8b`，
  用 `addr2line` 落在 [main.c](../Core/Src/main.c:52) 的 `for(;;){...__WFI();}` 空闲循环——
  **不是 HardFault，就是被调试器 halt 在 WFI**。
- 原因：`openocd_argv` 的 TCL 序列在 `dump_image` 之后直接 `shutdown`，没有 `resume`，
  于是每次读 RAM 都把 CPU 留在 halted。这也解释了之前反复出现的"看完监控就动不了、要重新上电"。
- 修复：[tools/monitor_can.py](../tools/monitor_can.py) `dump_ram` 结尾加 `catch {resume}`；
  [tools/commission.py](../tools/commission.py) `watch.tcl` 每个采样点后也 `catch {resume}`；
  顺带在两个摘要输出里加 `tick`，方便一眼看出板子是不是真的在跑。
- 验证：连续两次 `--fast`，`tick` 从 13332 涨到 18186（间隔 4 s，符合毫秒计时）。

## 下一步（需要现场）

1. **里程计地面真值**：需要一次低打滑的推行——轮子承重、匀速、不打滑；比较 `root_xy` ΔX 与量出的距离。
   通过后才设 `ODOMETRY_VERIFIED`。
2. **策略契约**（`POLICY_CONTRACT_VERIFIED`）：本机只有 `ref_document/fifty_mujoco/contracts/sample_actor31.py`
   这个离线规范，以及 `sample_weight_file/artifacts/` 里被引用的若干训练模块；
   原训练提交 `019a4b63…` 的 `legged_robot.py` 仍不存在。需要提供原仓库或一段黄金输入/输出轨迹，
   才能核对“先构造观测再推进参考”“5 ms 位置差分”等时序细节。
3. **受限输出验证**：以上两项完成后才谈 `ENABLE_OUTPUT=1`；首次运动必须受支撑、低力矩、有超时停止。
   另外当前姿态的膝角离训练默认角 0.67 rad，会触发 `inhibit` 位 256，启动策略前需先把腿摆到默认姿态附近。
4. **受限输出**：上述全部通过后，先受支撑、低力矩、有超时停止地验证方向，再放开整机。

## 待办（按依赖顺序）

1. 六个编码器方向：逐个电机小幅人工移动，比较 `capture` 前后计数得到 `sign`。
2. 参考位：实测四个机械角 + 同一姿态的六个计数，用 `apply --angles` 写入。
3. `MOTOR_CALIBRATED=1`（输出仍为 0）后复核关节角、膝分支和限位。
4. IMU：已知转角（整圈对齐）复核 yaw 比例，`IMU_FRAME_VERIFIED`。
5. 里程计：地面已知距离推动，`ODOMETRY_VERIFIED`。
6. 原训练时序对拍：`POLICY_CONTRACT_VERIFIED`（需要原环境代码或黄金轨迹）。
7. 受支撑低力矩验证后才考虑 `ENABLE_OUTPUT=1`。
