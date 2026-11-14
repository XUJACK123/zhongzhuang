# STM32H723 强化学习轮腿底盘

真实 `model_3050.pt` 权重的单片机部署工程。四个DM8009J和两个M3508共用CAN2；使用板载BMI088和UART5 DR16/DT7遥控器。

**当前状态（2026-11-14 01:20）：已烧录允许输出的版本（腿力矩上限 20 Nm、轮 2 Nm），六路方向、绝对参考位、IMU 整圈 yaw 已标定；策略能算出并真的发出力矩，但整机被右轮 M3508 反馈丢失挡在 STOP，尚未实现站立或平衡。**

|放行标志|值|含义|
|---|---|---|
|`ROBOT_ENABLE_OUTPUT`|1|唯一输出总开关；S1 下档=零电流|
|`ROBOT_MOTOR_CALIBRATED`|1|六路方向实测 + 参考位反解，实机回读一致|
|`ROBOT_IMU_FRAME_VERIFIED`|1|三轴方向人工核对 + 整圈 yaw +362.24°（偏差 0.62%）|
|`ROBOT_POLICY_CONTRACT_VERIFIED`|0|缺原训练 `legged_robot.py`，观测/参考推进时序未对拍|
|`ROBOT_ODOMETRY_VERIFIED`|0|方向已对，缺一次低打滑地面真值|

## 文档地图

|文件|作用|什么时候看|
|---|---|---|
|本文件|入口：当前状态、文档索引、问题点、下一步、遥控操作|每次上机前|
|[AGENTS.md](AGENTS.md)|给 AI/协作者的硬约束：总线与索引约定、标定与放行纪律、禁止项|改映射或改初始化前|
|[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)|**权威技术规范**：接线与 ID 表、31 维观测契约、运动学与雅可比、遥控语义、标定公式、构建烧录命令、Cortex-Debug 变量表|查协议、查公式、查调试变量|
|[docs/COMMISSIONING.md](docs/COMMISSIONING.md)|从零输出到放行的**操作手册**：方向标定、参考位、IMU 坐标、里程计、受限出力六步法|每次重新标定|
|[docs/COMMISSIONING_LOG_20261113.md](docs/COMMISSIONING_LOG_20261113.md)|**本轮改动与实机记录的流水账**：每个决定的动机、实测数值、数据目录链接|回溯"为什么这样改"|
|[docs/BOARD_BRINGUP_20261113.md](docs/BOARD_BRINGUP_20261113.md)|2026-11-13 首次联调的原始证据：CAN/遥控/IMU 逐项数据与 JSON 链接|核对历史基线|
|[docs/validation.json](docs/validation.json)|主机测试与交叉编译的机器可读快照（含 `pending` 列表）|确认构建层面通过到什么程度|
|[docs/board_bringup_20261113.json](docs/board_bringup_20261113.json)|联调汇总：固件哈希、源文件哈希、姿态标签|复核证据链|
|[user/robot_config.h](user/robot_config.h)|**唯一配置入口**：方向、参考位、放行标志、力矩上限、超时、周期|改任何数值|
|[user/algorithm](user/algorithm)|`rl.c/.h` 观测/推理/PD/耦合 + `model/` 由 `make export-policy` 生成的权重|改控制律|
|[user/remote_control.c](user/remote_control.c), [user/control_state.c](user/control_state.c)|遥控解码与请求映射、停机/使能/运行状态机|改操作手感|
|[tools/](tools)|`monitor_can.py` 只读抓 RAM、`commission.py` 录制与分析、`calibration_values.py` 算宏、`export_policy.py` 导权重、`verify_build.py` 核构建、`test_host.py` 假 HAL 测试|标定、监控、验证|
|[tests/](tests)|纯算法与假 HAL 控制循环的主机测试|改控制后回归|
|`diagnostics/`|每次实测的 `result.json` + RAM 二进制 + OpenOCD 日志，按时间戳分目录|查现场证据|
|`archive/`|旧 Keil/CubeMX/VS Code 配置与旧 CAN 扫描代码，仅存档|只读参考|

```powershell
make -j4
make test
make flash-check
# 确认放行标志与力矩上限后，由你显式执行：
make flash
make monitor
```

唯一产物：`build/rl/wheel_leg_rl.elf`、`.hex`、`.bin`。`make flash-check` 不访问硬件；`make flash` 使用已验证的 Horco DAP 配置烧录、校验并复位运行。普通 `make` 不访问硬件。主机验证使用 `make test`，完整构建核验使用 `python tools/verify_build.py --fresh`。

左腿：DM1髋/DM2小腿；右腿：DM4髋/DM3小腿；左轮M3508 ID2，右轮ID3。策略内部顺序为右髋、右膝、右轮、左髋、左膝、左轮。

旧配置已归档到 `archive/legacy_configs_*.zip`；旧CAN扫描代码备份为 `archive/can_scan_before_rl_20261113.zip`。当前只维护Makefile和VS Code的RL入口，详细清理记录在 `diagnostics/cleanup_*/manifest.json`。

## 目前的问题点

### 一、挡住"能动"的（现场/硬件，优先）

1. **右轮 M3508（0x203）完全无反馈**。同一时刻其余五节点各收到 2–3 万帧，右轮 `recv=0`、温度 0、状态 0x00。
   因为 `motor_feedback_healthy()` 要求六路全新鲜，位 4 一置位就跳过状态估计 → 位 32（几何）跟着置位 →
   `inhibit=36` ≠ 0 → 状态机永远停在 STOP。**这是当前唯一的硬阻塞**，先查电调供电、CAN 线、与左轮对调信号线定位。
2. **左腿被气弹簧顶住**。实机自然停位就在 `ROBOT_ARM_REFERENCE_RAD`（气弹簧把小腿顶在伸展端），
   台架 10 Nm 压不动；已把腿力矩上限开到训练额定 20 Nm 待验证。
3. **调试器会打断控制**。OpenOCD 读 RAM 会 halt CPU，DM 组控因此在状态字节报 `0x0D`（通信丢失），
   且过去读完不 `resume` 会让板子停在 `__WFI()`（已修：工具链已补 `catch {resume}`）。

### 二、模型层面（训练与实物的差距）

1. **默认姿态差 0.65 rad**：训练默认膝角 ±3.661，实车自然位 ±3.0；策略一上电就要把膝再折 0.65 rad，
   `action` 直接饱和 ±1（四个腿通道全打满）。需要重新训练带气弹簧负载的模型，或在训练侧把默认角/气弹簧补偿建模进去。
2. **气弹簧没进模型**：真实气弹簧由机构自然产生，固件不做额外补偿，因此策略必须靠自己"顶过"弹簧死点；
   训练环境若无弹簧，策略学到的力矩分布与实物不符。
3. **力矩常数未实物标定**：DM 反馈力矩比指令低约 20%（5.88 / 7.37 Nm），`Kt` 可能更接近 1.0 而非 `1.261575`，
   即当前 20 Nm 上限实际只兑现约 16 Nm。需做一次静态力矩标定后统一换算系数。
4. **轮力矩上限 2 Nm < 训练 2.48 Nm**，会改变轮子行为。
5. **`POLICY_CONTRACT_VERIFIED=0`**：缺原训练 `legged_robot.py`，观测构造/参考推进先后、5 ms 位置差分、
   reset 时序都没有黄金轨迹对拍，无法断言与原训练环境完全一致。

### 三、电控/固件层面（可提升）

1. **单点故障全身阻断**：任一电机丢帧即 `inhibit` 位 4，并连带几何位 32 把策略也停掉。
   可以考虑按关节重要性分级降级（例如轮子丢反馈时保留腿部支撑），但这是安全取舍，需要单独评估并测试。
2. **监控方式侵入性太强**：靠 halt CPU 读 RAM。长期应改为 DWT/mailbox 环形缓冲或 CAN 侧旁路记录，
   做到"看数据不影响控制"。
3. **`rx_overflow` 是锁存计数**，一次溢出后永久为真，缺少按窗口统计的丢帧率指标。
4. **缺单电机标定模式**：`COMMISSIONING.md` 第 6 节要求先做单电机低力矩验证，但当前固件只能整车上电，
   没有单关节小幅运动入口（不能加"任意 ID 扫描/脉冲运动"，需要受限、带超时的专用模式）。
5. **里程计假设接地无滑移**：实车推行打滑时尺度偏差可达 40%，`ODOMETRY_VERIFIED=0`。

## 下一步操作

1. **修右轮反馈**：电调上电、CAN 线插紧；把左右轮信号线对调——故障跟着线走是线/接口问题，仍固定在右轮就是电调或供电问题。
2. **确认六路在跑**：`python tools/monitor_can.py --brief`，要求六个 `recv` 都在增长、`inhibit=0`。
3. **架空验证 20 Nm**：整车支撑、四条腿全部悬空、手不碰腿，S1 上档，看左右膝是否都折向默认姿态。
   ```powershell
   python tools/commission.py watch --label freeair_cap20 --seconds 30 --interval 0.2
   ```
   看 `arm` 与 DM 反馈力矩：若 20 Nm 仍纹丝不动，说明有效力矩仍小于气弹簧，需要分级给定实测弹簧拐点。
4. **标定力矩常数**：静止堵转给定已知电流，用反馈力矩/外部测力计反算 `Kt`，再统一 `ROBOT_DM_KT_NM_A`。
5. **落地受支撑站立**，先低高度、低速，逐步放开。
6. **里程计真值**（低打滑推行 1 m）→ 置 `ODOMETRY_VERIFIED`。
7. **原训练对拍**（提供 `legged_robot.py` 或黄金轨迹）→ 置 `POLICY_CONTRACT_VERIFIED`。

> 纪律：`make test` 与交叉编译通过不代表板上时序、台架行为或平衡能力；不把调试器看到的数值当作平衡证据。

## 当前遥控器操作

DR16/DT7，UART5 PD2。**右开关 S1 是唯一使能开关**：

|操作|结果|
|---|---|
|S1 下档|零电流停机（唯一的"关"）|
|S1 中档/上档|进入 ENABLING → ACTIVE，开始出力|
|上电后 / 故障后 / 遥控丢失后|必须**先经过一次下档**才允许重新运行|
|左开关 S2|不参与判断，任意位置都可|
|右摇杆竖直|前进 0…0.30 m/s（**反向拨 = 0 速度请求**，该权重未训练倒车）|
|右摇杆水平|yaw ±2.5 rad/s|
|左摇杆、高度|暂不使用（高度固定 0.10 m 底部净空）|

自动保护（任一触发即停机并置故障，需重新下档）：遥控 100 ms 超时、电机反馈 30 ms 超时、IMU 5 ms 超时、
超温 75 ℃、倾角 > 0.70 rad、CAN bus-off、膝限位越界、推理失败、控制周期超时（2 ms）。

出力测试时**不要开 Live Watch**（每次采样都要 halt CPU），改用 `python tools/monitor_can.py --fast` 事后核对。
