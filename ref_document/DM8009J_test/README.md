# DM8009：先识别 CAN ID，再由遥控器触发 +20°

默认应用是 STM32H723 / DM-MC02 + DM8009 + DBUS。当前控制目标 CAN_ID=2，MST_ID=0（2026-11-11 参数回复确认），默认只核对 ID=2。CAN2/P3，PB5=RX、PB6=TX，经典 CAN 1 Mbps。
默认构建产物为 build/dm8009_step.elf/.hex/.bin，VS Code 的 STLink/DAP/JLink 调试配置均指向该 ELF。

## 上电流程

1. 等待 2 秒。
2. 默认对已知 ID=2 读取确认两次，每次等待 100 ms；专用 ID_SCAN=1 固件扫描 1..2046 两轮，每个 ID 等待至少 10 ms，实测完整扫描约 51 秒。
3. 扫描仅向 0x7FF 发送 RID7（MST_ID）读取。根据回复中的完整电机 ID 与反馈 CAN ID 匹配，两个轮次均回复才确认。
4. 找不到、只确认一轮、出现多个 ID、记录溢出或 CAN 错误时，停止；不会挑选任意一个电机启动。
5. 默认运动应用只接受一只电机。唯一识别成功后先失能，读取 ESC_ID、CTRL_MODE、PMAX、VMAX、TMAX。
6. 不再等待已失能反馈。保留 MIT 参数/状态读回通道；默认运动发送改为 HKU 组控协议（DM_USE_MULTI=1）：0x3FE，ID=2 使用第2个 little-endian int16 电流槽（字节 2..3），其余槽位为零。目标电机上传 0x302 组控反馈，读回 CTRL_MODE=1 不代表 MIT 力矩帧被功率级采用。不保存到电机 Flash，不修改 ID，不设置机械零位。
7. 参数与反馈有效后等待遥控器解锁。不会仅因扫描成功自动使能。

仅在隔离的单电机台架上使用运动应用。两只电机都要识别时，用下面的专用扫描固件。
同 ID 的两只设备可能发生报文冲突，扫描不能可靠区分它们；需分别连接后读取。

## 专用 ID 扫描（不控制电机）

构建：mingw32-make -j4 all DEBUG=1 ID_SCAN=1

产物：build_dm8009_idscan/dm8009_idscan.elf/.hex/.bin。
烧录扫描固件：mingw32-make -j4 download_stlink DEBUG=1 ID_SCAN=1

此固件只发送上述读参请求；不发送使能、失能、运动、参数写入或保存命令。
找到一只时 stage=5；找到多只时 stage=4、scan_result=3，但每只的记录都保留在 g_dm.scan 中。
扫描后停止发送。扫描不会改变电机原本的使能状态。
若因尚未连接而进入 bus-off，完成接线后重新复位主控，重新扫描。不会自动恢复 bus-off。

注意：默认 F5 加载运动应用；观察扫描固件时应使用下面的 STLink-IDScan-Attach 配置，避免加载错 ELF。

## 遥控器与指示灯（默认运动应用）

板载 WS2812：双下档蓝色、双上档红色、无有效信号或超过 100 ms 未更新为绿色。中档或拨杆不同档显示蓝色并禁止动作。灯色只表示遥控状态，不表示电机已经准备好。PA7/SPI6、6 MHz、GRB、低亮度；发送等待 TXC 后显式挂起，不使用 DMA。

20° 指 DM8009 输出轴角度。默认组控以 0x302 的 8192-count 单圈编码器累计相对角度，速度字段为 0.01 rpm/count；MIT 状态保留为独立诊断，按 PMAX 映射为弧度。内置减速比已包含在输出轴反馈中，不再次除以 9，也不套用 M3508 的减速比。单圈编码器跨 ±pi 时按 2*pi 解包，PMAX 并不是机械回绕周期。

- 启动和遥控器恢复后，两个拨杆先在下档保持 100 ms 解锁。
- 两个拨杆切上档，稳定 40 ms，触发一次运动。普通 `make all` 固定目标为绝对 20°并在到位后保持；`DIRECT_TEST=1` 默认从触发时的位置相对移动 +20°。
- 一直双上档不重复触发。普通 `make all` 默认保持目标；Direct Test 默认到位并稳定 150 ms 后停止。
- 离开双上档立即取消动作、取消待发目标并发送失能。Direct Test 再触发时从当时位置重新走 +20°。
- 每次动作先发送全零组控电流，再使能；收到新的已使能反馈才输出控制力矩。所有停止路径均先覆盖全零组控帧，再请求失能；不能依赖 FD 改变使能状态。
- 遥控器丢失、过温、超速、周期超时、单次位移超过 24°，或未在对应编译配置的动作超时内到位时停止并取消解锁。
- 电机反馈超过 100 ms、CAN 故障、发送失败或使能超时会锁定应用；排除问题后复位重启。

主控以位置 P -> 限速目标 -> 速度 PI 计算输出轴力矩；参考 HKU 的 Kt=1.261575 Nm/A 和 41.044776 A/16384 换算组控电流。编码采用向零截断，不超过力矩限幅。位置和速度闭环使用组控反馈；MIT 状态仍用于诊断和使能确认。
相对角度从有效反馈开始累计；断联期间的运动不做推算。
运动参数与限制已扩大两倍。普通 `make all` 档的速度目标上限为 4°/s、力矩指令限幅为 ±28 Nm（按现有 Kt 换算约 ±22.2 A）、力矩爬升为 0.8 Nm/s、超速阈值为 40°/s、未到位超时为 90 秒；头文件/无额外覆盖的 Direct Test 默认档为 10°/s、±11 Nm、1 Nm/s、20°/s 和 12 秒，启动前速度门槛为 multi<6°/s、MIT<3°/s。两档行程限制均为 24°，过温保护仍为 75°C。`LOAD_TEST=1..6` 的专用速度/力矩参数与 10°目标、12°行程、原启动速度门槛保持不变。组控零电流使用 int16=0，无 MIT 中点量化残差。闲置时周期性覆盖零电流并请求失能；反馈 state=1 时不能声称电机已经失能。普通档的 28 Nm 是命令限幅，不是实测力矩，且未经过这次参数调整后的实机验证；运行前需确认机构行程和防护条件。

参数：Core/Inc/dm8009_step_config.h。DM_FIXED_CAN_ID 默认 2；设为 0 可全范围识别。专用 ID_SCAN=1 构建强制使用 0。
固定 ID 仍读取并确认两次，但不会检查其它 ID 是否存在，因此同样要求单电机连接。
增益和限幅尚未经实机运动整定，小力矩可能不能克服摩擦；不要据此直接提高到大力矩。

## Live Watch

| 变量 | 含义 |
| --- | --- |
| g_dm.stage | 0启动等待、1扫描、2参数准备、3运动应用就绪、4锁定/停止、5只读识别完成 |
| g_dm.error | 0正常、1扫描失败、2准备超时、3总线异常、4反馈失效、5电机状态异常、6发送失败、7使能超时、8参数异常 |
| g_dm.scan_result | 0未完成、1唯一、2无回复、3多只/溢出、4仅一轮确认 |
| g_dm.scan_id / scan_pass | 当前候选 ID 与轮次（0或1） |
| g_dm.scan_count / scan[] | 发现记录数；motor_id、master_id、pass_mask（3=两轮）、replies |
| g_dm.motor_id / master_id | 唯一确认后的电机命令 ID / 反馈 ID |
| g_dm.parameter_mask / mode | 参数读取掩码（31=完整）/控制模式（1=MIT） |
| g_dm.pmax_rad / vmax_rad_s / tmax_Nm | 电机读回的 MIT 量程 |
| g_dm.feedback / feedback_count / feedback_fresh | 原始状态、位置/速度/力矩和两处温度、计数、新鲜度 |
| g_dm.position_deg / speed_deg_s | MIT 查询通道的相对角度与速度（独立诊断） |
| g_dm.multi_position_deg / multi_speed_deg_s | 当前闭环实际采用的组控角度与速度 |
| g_dm.multi_count / multi_ms / multi_state / multi_current_mA | 组控反馈计数、时间、状态、实测电流 mA |
| g_dm.step | 解锁、动作状态、目标/误差、完成次数、力矩目标和故障 |
| g_dm.step.fault | 1遥控、2反馈、3总线、4温度、5速度/数值、6周期、7动作超时、8单次位移 |
| g_dm.drive | 0失能/空闲、1等待使能确认、2已确认使能 |
| g_dm.torque_quantized_Nm | 最近编码的实际力矩目标；失能时归零，并非实测力矩 |
| g_can_rx_total / g_can_last_rx / g_can_rx_ring | CAN原始接收计数和帧记录 |
| g_can_tx_queued / tx_failed | 提交计数/失败计数，不能当成总线送达确认 |
| g_can_bus_off / tx_error_count / rx_error_count | CAN总线状态与硬件错误计数 |

## 构建与验证边界

默认：mingw32-make -j4 all DEBUG=1
被动监听与内部回环：mingw32-make -j4 all DEBUG=1 DIAGNOSTIC=1
保留的旧小力矩诊断：DM8009=1，输出独立目录 build_dm8009_diag；不属于默认遥控器程序。

主机测试入口：powershell -ExecutionPolicy Bypass -File tools/test.ps1。覆盖自动 ID 与默认固定 ID 两种配置，以及只读扫描和协议。
测试覆盖扫描读参、非零反馈ID、高位电机ID、多电机/无回复/一轮回复拒绝、模式读回、遥控触发与取消、各类停止，以及简化闭环。
测试不等于真实负载、角度精度或通信看门狗验证。2026-11-10 已完成一次 0.50 Nm 组控实机尝试，有电流响应，但没有有效角度位移；10°运动尚未验证成功。
主控暂停、CAN断线或失电时，不能保证停止报文送达；应核实电机自身通信超时保护。

协议来源：[达妙官方 SDK](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)。

最新实机诊断见 diagnostics/MULTI10_TEST_20261110.md；旧状态见 diagnostics/DM8009_REMOTE_VERIFICATION_20261109.md；不能从“已编译/已烧录”推断真实转动已经通过验证。

2026-11-09追加：目标速度2°/s；力矩目标变化率0.10 Nm/s；反馈速度超过10°/s或单次位移超过12°停止。反馈超时后，恢复有效反馈并双下100ms可重新读取参数，不能直接自动恢复运动。

直接单次测试使用 DIRECT_TEST=1，独立产物在build_dm8009_direct；仅收到g_direct_start_request=1后运行一轮并自行停止，不属于默认遥控固件。2026-11-09实机尝试结果：指令有效发出，12秒内反馈位移0°，峰值编码力矩约0.488Nm；见diagnostics/DIRECT10_TEST_RESULT.md。

2026-11-10：默认已改为 HKU 0x3FE 组控，0.50 Nm试验无有效位移；用户授权1.50 Nm上限试验在20秒内只走约0.176°，实际力矩峰值1.214 Nm。之后用户授权逐步提高电流。3 Nm独立候选（DIRECT_TEST=1 LOAD_TEST=2）烧录后启动前CAN bus-off，没有触发动作。最新现场状态以diagnostics/MULTI10_TEST_20261110.md为准。

后续实机逐级升流与调参的完整结果见 diagnostics/MULTI10_TEST_20261110.md。当前组控闭环的状态、温度和新鲜度使用multi字段；MIT字段作为独立诊断。专用DIRECT_TEST通过g_direct.stop_step_fault保留故障码，完成后冻结最后32帧电机原始报文。诊断记录中的旧默认遥控限幅为0.50 Nm；此次参数更新后，普通 `make all` 档限幅为28 Nm，Direct Test 默认档为11 Nm。加载试验1/2/3仍分别为1.50/3/5 Nm，须单独编译且显式写一次启动请求。新目标和限幅未经过实机验证；仅编译不代表硬件当前加载了该镜像。
