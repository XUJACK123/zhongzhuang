# H723 强化学习轮腿控制

本工程部署已有策略，不在单片机上训练。权重来自 `sample_weight_file/artifacts/model_3050.pt`，actor 为 `31 → 128 → 64 → 32 → 6`，隐藏层 ELU，输出层线性。已接入真实权重，未填随机权重或占位网络。默认禁止力矩输出；电机方向和绝对参考位尚未标定。

## 接线与目录

2026-11-13 用户确认：DM-MC02 / STM32H723，六个电机全部接 CAN2（PB5 RX / PB6 TX），1 Mbps；遥控器 DR16/DT7 接 UART5 PD2；使用板载 BMI088。

|策略索引|训练关节|实际电机|反馈 ID|命令槽位|
|---|---|---|---|---|
|0|rf0_Joint 右髋|DM8009J ID4|0x304|0x3FE 第4槽|
|1|rf1_Joint 右小腿|DM8009J ID3|0x303|0x3FE 第3槽|
|2|r_wheel_Joint 右轮|M3508 ID3|0x203|0x200 第3槽|
|3|lf0_Joint 左髋|DM8009J ID1|0x301|0x3FE 第1槽|
|4|lf1_Joint 左小腿|DM8009J ID2|0x302|0x3FE 第2槽|
|5|l_wheel_Joint 左轮|M3508 ID2|0x202|0x200 第2槽|

DM 和 DJI 同号可以共线，反馈和组控 ID 各自独立。DM 电流组帧为有符号 int16 **小端**，DJI 为有符号 int16 **大端**。DM 的 `0.06 度/秒/LSB`、8192 位置计数、`Kt=1.261575 Nm/A` 与 `Imax=41.044776 A` 沿用 DM8009J_test 已使用的组控转换。不要套用 MIT 的位置/力矩量化，也不要将 DM 输出轴角度再除以9。

```text
user/
  algorithm/rl.c, rl.h       观测、推理、目标解码、PD、耦合映射、速度包络
  algorithm/model/          导出的实际权重 model.c/model.h 和来源 manifest
  remote_control.c/.h       遥控器解析与指令映射
  motor_io.c/.h             六电机反馈、编码器展开、组帧
  state_estimator.c/.h      连杆坐标转换、位置差分速度、轮式里程计
  imu.c/.h                  BMI088、静止偏置校准、四元数融合
  board.c/.h                电源、UART5、看门狗、DWT
  control_state.c/.h         停机、准备、使能、运行、故障状态
  chassis.c/.h              采样与控制调度、统一输出管理、调试变量
  robot_config.h            标定、安装坐标、放行标志、实际限制
tools/                      权重导出、标定值计算、主机测试、ARM 构建核验
tests/                      纯算法及假 HAL 控制循环测试
```

原 CAN 扫描工程备份在 `archive/can_scan_before_rl_20261113.zip`。旧Keil/CubeMX和旧VS Code配置已归档到 `archive/legacy_configs_*.zip`，活动目录仅保留Makefile与RL调试入口。逐步标定流程见 [COMMISSIONING.md](COMMISSIONING.md)。

## 模型契约

观测按以下顺序拼接，裁剪到 ±100：

|索引|内容|缩放|
|---|---|---|
|0..2|机体角速度 rad/s|0.25|
|3..5|机体系投影重力 Rᵀ·(0,0,-1)|1|
|6..8|前进速度、yaw 角速度、底部净空指令|2 / 0.25 / 5|
|9..12|右髋、右膝、左髋、左膝相对训练默认角|1；髋使用周期误差|
|13..18|六个广义关节速度|0.05|
|19..24|上次经分通道限幅的 action|1|
|25..26|机体系参考位置误差|5，再限幅 ±2|
|27..28|sin(参考 yaw−实测 yaw)、cos(误差)−1|1|
|29..30|机体系根部 vx、vy|2|

本权重没有 history encoder，也不输入 critic 的147维特权状态。参考工程 XYEGA 的25维观测、历史缓存、模型默认角和力矩系数不能直接套用。

策略周期为10 ms（训练 `sim.dt=5 ms`、`decimation=2`）；IMU/遥控状态采样按1 ms主循环处理，广义关节速度按5 ms位置差分更新，PD与CAN输出每2 ms更新。差分测量会受反馈刷新率和调度抖动影响，必须在板上量测；主机测试不证明实时周期。

指令斜坡为 `(1.5 m/s², 2π rad/s², 0.12 m/s)`。action 的四个腿通道裁剪 ±1、轮通道裁剪 ±2。腿目标为 `q_default + 0.5*action`；轮目标为 `20*action rad/s`。膝目标限制在归档机械范围内。腿 PD 为髋 `40/2`、膝 `60/2`，轮速度系数为0.3。默认净空指令0.10 m是机身底部净空，归档默认根部高度约0.2749762 m，二者不能混用。

训练默认角：

```text
右髋 +3.1063109025，右膝 -3.6606010515，右轮 0
左髋 -3.1063109025，左膝 +3.6606010515，左轮 0   [rad]
```

两个腿电机都固定在车身，不能把小腿电机角当作训练膝关节角。对每条腿，令 `s=右+1/左−1`，`r=113.4/135`：

```text
alpha = m_knee - m_hip                         (-2π,0) 装配分支
beta  = alpha/2 + asin(r*sin(alpha/2))
q_hip = s*m_hip
q_knee = s*(beta - π/2)
k = d(beta)/d(alpha)
tau_motor_hip  = s*(tau_joint_hip - k*tau_joint_knee)
tau_motor_knee = s*k*tau_joint_knee
```

角度、速度与力矩使用同一雅可比，主机测试检查逆变换与功率一致性。力矩最终在**电机空间**施加训练额定速度包络，再限制到调试上限（`ROBOT_LEG_TORQUE_CAP_NM` 腿 20 Nm = 训练额定、`ROBOT_WHEEL_TORQUE_CAP_NM` 轮 2 Nm，略低于训练额定约 2.48 Nm）。腿已开到训练额定，是因为实车气弹簧把小腿顶在伸展端，低于该值无法把膝折向训练默认姿态（见调试日志 00:50）；轮仍留 20% 余量，会轻微改变策略表现，不能据此宣称训练能力已在实物复现。真实气弹簧由机构自然产生，不额外移植其他机器人策略的弹簧补偿。

M3508 总减速比15.88已于2026-11-13由用户确认，与训练归档一致。转矩常数仍按 P19 比例折算，尚非实物转矩标定值。后续若更换减速箱，需要同步修改反馈、输出系数及策略对应硬件约定。

## 遥控器操作

右开关 **S1 只有两档语义：下档 = 零电流停机，其余位置 = 运行**（上电、遥控丢失或故障后，需要先经过一次下档）。
左开关 S2 不再参与使能判断。右摇杆竖直方向控制前进0..0.30 m/s，水平方向控制 yaw ±2.5 rad/s；反向拨前进杆产生0速度请求，当前权重未按倒车指令训练。左摇杆和高度调节暂不使用。

进入运行时先维持零电流，每20 ms向四个 DM 发送 FC，使能反馈全部为1后才发非零力矩。遥控100 ms超时、电机30 ms超时、IMU5 ms超时、超温、过大倾角、机构分支/范围错误、推理失败、周期超时都会立即停机并置故障，故障后必须再经过一次 S1 下档。反馈断档后编码器圈数可能丢失，会锁定 `position_lost`，需要检查参考位并重启。

## 方向与参考位标定

`user/robot_config.h` 的三组数组使用**策略索引顺序**，不是 ID1..6顺序：

```text
canonical_arm = arm_reference + encoder_sign * unwrap(raw_angle - encoder_reference)
```

`ROBOT_ENCODER_SIGNS` 同时用于反馈和力矩发送，必须按虚功方向统一。`ROBOT_ENCODER_REFERENCE_RAD` 是已测参考姿态的编码器角；`ROBOT_ARM_REFERENCE_RAD` 是同一姿态下的规范输出臂角。上电不会把当前姿态自动当作零位。轮反馈先按实际齿比转换到轮轴。

调试器读取 `g_motor_snapshot.motor[i].raw` 得到六个原始计数。已知实际广义关节参考姿态后，可计算配置值：

```powershell
# 数字仅示范命令格式；必须替换成实测计数、方向和参考姿态！
python tools/calibration_values.py --counts 100 200 300 400 500 600 `
  --signs 1 -1 1 -1 1 -1 `
  --joint-deg 178 -210 0 -178 210 0
```

也可在**确实摆到训练默认姿态时**使用 `--at-training-default` 替代 `--joint-deg`。工具只打印宏，不修改配置、不连接电机。摆位与上电姿态应使参考位附近单圈编码器能够唯一确定装配分支；单圈传感器不能恢复断电期间任意多圈转动。

BMI088 使用 SPI2，PC0加速度片选、PC3_C陀螺仪片选、PB13时钟、PC1 MOSI、PC2_C MISO；PC2/PC3模拟开关均闭合。PE12的真实DRDY上升沿指示新样本，主循环约1ms读取，不再轮询瞬态中断状态位。启动需2000个静止样本完成陀螺偏置校准；移动或超出5ms失鲜门槛时重新累计。当前暂定映射为 `(-chip_x,-chip_y,+chip_z)`，由本次确认姿态得出；三轴方向已人工复核，整圈左转实测累计 yaw +362.24°（比例偏差 0.62%），因此 `IMU_FRAME_VERIFIED=1`；长期温漂仍按 0.02°/s 量级观察，不作为精度上界。数据见 [实机联调记录](BOARD_BRINGUP_20261113.md)。

配置中的五个标志默认均为0：`ENABLE_OUTPUT`、`MOTOR_CALIBRATED`、`IMU_FRAME_VERIFIED`、`POLICY_CONTRACT_VERIFIED`、`ODOMETRY_VERIFIED`。其中 `MOTOR_CALIBRATED` 与 `IMU_FRAME_VERIFIED` 已于 2026-11-14 用实测方向、实测参考位和整圈 yaw 复核置1（见 [改动记录](COMMISSIONING_LOG_20261113.md)），其余三个仍为0。只有实际完成对应工作才能置1。电机标定完成后，保持 `ENABLE_OUTPUT=0` 可进行真实传感器的影子推理，观察 `g_rl`，CAN仍然只发零输出。未标定时，仅采集原始反馈并执行网络启动自检，不用虚构关节状态冒充实时推理。

## 来源边界与实物验证状态

观测数学以本地 `fifty_mujoco/contracts/sample_actor31.py` 离线规范、归档 env_cfg/train_cfg、actuator_model 和模型参数为依据。原训练提交对应的 `legged_robot.py` 尚未完整恢复，参考轨迹更新先后、位置差分精确时序、reset细节还没有原运行轨迹对拍。本实现先构造当前观测再推进下一参考，源码已注明该选择。因此 `POLICY_CONTRACT_VERIFIED` 默认0；不能把数学对拍等同于原训练环境完全一致。

额外六维参考观测需要根部位置和速度。本工程使用训练 URDF 的轮中心几何，结合腿运动和机体角速度补偿的无滑移里程计；会受打滑、离地、轮径、装配误差及陀螺 yaw 漂移影响。它不具有外部定位的绝对精度，也没有实物接触检测，当前只适用于后续验证地面接触状态，故 `ODOMETRY_VERIFIED` 默认0。参考高度固定0.10 m，不加入未训练的跳跃、自救或变高模式。

当前已通过C权重逐位检查、网络/观测数值对拍、协议/耦合测试、默认禁止输出及假HAL故障测试、ARM交叉编译、启动向量和FLASH权重地址检查。另通过H723 CAN2硬件FIFO确认六个反馈ID有新帧，记录在 `diagnostics/fifo_can_20261113_215157/result.json`。这不等于RL固件已烧录或实物平衡完成；烧录、传感器及周期结果以 `diagnostics/` 中相应操作记录为准。假HAL测试配置不改生产标志，`build/host` 为可再生成的测试产物。

物理CAN断线时，固件无法保证零帧送达；电调自身通信超时停机行为需要实测。六个电机共线还需核对反馈频率与总线利用率，避免各节点反馈过密。软件发送成功计数只代表提交给FDCAN，不代表电机执行或实测平衡。

## 构建、测试与烧录

在本目录执行，使用现有 Arm GNU Toolchain、GNU make、Python，无需 CubeAI 或 Torch：

```powershell
make -j4
make test
python tools/verify_build.py --fresh
```

生成 `build/rl/wheel_leg_rl.elf`、`.hex`、`.bin`。`verify_build.py` 保留完整日志、检查八个权重符号都位于FLASH，并报告编译器、内存和单函数静态栈占用；单函数栈大小不等于整个中断嵌套调用栈。`docs/validation.json` 保存本次结果快照。

重新导出同一模型的权重：

```powershell
make export-policy
# 仅在已有PyTorch的环境，可额外核对零向量：
python tools/export_policy.py --torch-check
```

另一个检查点只能在确认同一31维契约、关节顺序和归一化后替换。普通make使用已生成的C权重，构建不依赖外部归档目录。

烧录命令沿用OpenOCD + Makefile方式，只有显式运行时才会访问板卡：

```powershell
make flash
make monitor
# 仅检查命令和镜像，不连接板卡：
make flash-check
```

默认使用本机已实测的 `${USERPROFILE}/.eide/tools/openocd_7a1adfbec_mingw32/bin/openocd.exe` 和 `openocd_dap.cfg`，指定Horco CMSIS-DAP v2的VID/PID、USB bulk接口0及100kHz SWD。可用 `OPENOCD=完整路径`、`OPENOCD_CONFIG=配置文件` 覆盖。烧录脚本记录镜像SHA256、放行标志和完整OpenOCD日志。普通构建与flash-check均不连接硬件。

调试变量：`g_chassis.mode/inhibit/fault`、`g_chassis.inference_us/max_loop_us/overruns`、`g_chassis.actor_selftest`、`g_motor_snapshot`、`g_remote`、`g_imu`、`g_measurement`、`g_rl.obs/action/motor_torque`。`inhibit` 位定义见 `user/chassis.h`；调试耗时只测推理及部分控制段，真实完整循环应另用示波器或DWT量测。

Cortex-Debug 的 Live Watch 请使用下列当前符号（旧固件的 `motor_can_debug`、`bench_test_status`、`hfdcan4` 等已不存在）：

|用途|表达式|
|---|---|
|状态机|`g_chassis.mode`、`g_chassis.inhibit`、`g_chassis.fault`|
|是否真出力|`g_can_monitor.nonzero_torque_frames`、`g_can_monitor.enable_commands`|
|策略输出|`g_rl.action`、`g_rl.target_q`、`g_rl.motor_torque`|
|遥控器|`g_remote.s1`、`g_remote.s2`、`g_remote.channel`、`g_remote.count`、`g_remote.valid`|
|电机反馈|`g_motor_snapshot.motor[0].raw`、`.counts`、`.arm`、`.velocity`、`.temperature`、`.state`|
|CAN 节点|`g_can_monitor.node[0].received`、`g_can_monitor.node[0].data`|
|运动学|`g_measurement.q`、`g_measurement.qd`、`g_measurement.xy`|
|IMU|`g_imu.calibrated`、`g_imu.max_gap_ms`、`g_imu.gravity`|

电机索引固定为 0 右髋 DM4、1 右膝 DM3、2 右轮 ID3、3 左髋 DM1、4 左膝 DM2、5 左轮 ID2。
Live Watch 每次采样都要读内存；输出使能期间高频轮询会使 `g_can_rx_overflow` 增加并让 DM 上报通信丢失
（`state=0x0D`），从而重新置位 `inhibit` 的 16/4 位。出力测试时把 `samplesPerSecond` 降到 1 或关闭
Live Watch，改用 `python tools/monitor_can.py --fast` 事后核对计数。
