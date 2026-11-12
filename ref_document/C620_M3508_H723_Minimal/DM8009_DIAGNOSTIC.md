> 2026-11-09：默认构建已恢复为 C620 + M3508。本文件为保留的 DM8009 测试记录。
> 单独构建 DM8009 使用 `mingw32-make -j4 all DEBUG=1 DM8009=1`，调试时需另行指定对应 ELF。

# DM8009 CAN2 小力矩 MIT 测试

当前默认构建是 `build/dm8009_diag.elf`，DAP/STLink/JLink 都加载它。
旧 C620 源码保留，但不参与默认构建，不再发送 C620 电流指令。

## 操作

1. 仅连接一只待测 DM8009，固定好电机/机构，确认独立供电、共参考地、CANH/CANL 和终端电阻。
2. `Core/Inc/dm8009_config.h` 中默认 `DM_MOTOR_CAN_ID=1`、`DM_MASTER_CAN_ID=0`，已由本次实机回帧验证。
3. VS Code 选择 DAP，F5 构建/烧录、停在 main；继续后会执行下面的初始化和输出流程。
4. 程序等待 2 秒，先失能；读取 KT_Value、TMAX、Master ID、控制模式，确认失能反馈。
5. 必要时临时把 CTRL_MODE 写为 MIT=1，并读取确认。不会保存参数到电机 Flash。
6. 发送 Kp=Kd=0 的零力矩目标（存在协议量化误差），再使能；收到 state=1 后进入小力矩输出。
7. 默认目标 0.05 N·m，每 10 ms 发送 MIT 力矩报文；每 100 ms 查询反馈及轮询参数。

代码不是直接电流环：当前使用明确的 MIT 力矩目标，默认 0.05 N·m，限幅 ±0.10 N·m。
本次实机 RID1 返回 `01 00 33 01 00 00 00 00`，KT_Value=0；RID23 返回 TMAX=54。
原版本依赖 KT 换算电流，因此在 stage=1 检查失败后进入 stage=5/error=1，没有发出小电流目标。
现已修正为默认直接力矩控制，只要求有效 TMAX。KT=0 时 `g_dm_parameter_warning=1`，并非致命错误。
`g_dm_current_estimate_valid=0` 表示无法给出可靠的等效电流，电流字段为0不能理解为实测零电流。
如未来已正确配置非零 KT，可以将 DM_USE_CURRENT_EQUIVALENT 改为1，启用电流等效输入和原 ±0.10 A 限幅。
MIT 力矩只有12位，TMAX=54时 0.05 N·m 落在离散网格上；本程序四舍五入编码为2049，实际目标约0.03956 N·m。
查看 `g_dm_torque_quantized_Nm` 可确认真正编码值。KT=0时不会猜测电流。
TMAX 读取失败或不合理、模式切换未确认或设置超时，程序不会进入输出。

恒定小力矩没有速度上限，空载仍可持续加速；本程序没有增加限速闭环。停止测试应请求失能。
暂停 CPU 会中断控制发送，但能否及时失能取决于电机通信看门狗，不应把暂停当作急停。

## Live Watch

| 变量 | 意义 |
| --- | --- |
| `g_init_stage` | 1 HAL，2 系统时钟，3 CAN 时钟，4 CAN/GPIO，5 CAN 启动，6 主循环 |
| `g_error` | 初始化错误，正常为 0 |
| `g_dm_control_stage` | 0 等待；1 失能/读参数；2 切 MIT；3 使能；4 小力矩输出；5 已停止 |
| `g_dm_control_error` | 0 无；1 参数/目标/模式异常；2 设置超时；3 反馈丢失或退出使能；4 电机故障；5 bus-off；6 TX 提交失败 |
| `g_dm_torque_target_Nm` | 默认 0.05，可在线修改，目标限幅 ±0.10 N·m |
| `g_dm_current_command_A` | 限幅后的目标；停止时归零，不代表电机实测值 |
| `g_dm_current_estimate_valid` | 当前 KT=0，此值为0，不提供可靠电流换算 |
| `g_dm_kt_Nm_per_A / g_dm_tmax_Nm` | 从电机读取的转矩系数和 MIT 力矩映射范围 |
| `g_dm_torque_command_Nm / g_dm_torque_quantized_Nm` | 力矩目标和编码后的实际给定值 |
| `g_dm_control_mode` | 电机读回模式；1 MIT；2 位置速度；3 速度；0xFFFFFFFF 未知 |
| `g_dm_enabled` | 仅新鲜反馈且 state=1 时为 1 |
| `g_dm_feedback` | 反馈计数、原始位置/速度/力矩、MOS/绕组温度和状态 |
| `g_dm_feedback.fields.state` | 0 失能；1 使能；8 超压；9 欠压；A 过流；B MOS过温；C 绕组过温；D 通信丢失；E 过载 |
| `g_dm_feedback_stale / g_dm_feedback_age_ms` | 500 ms 没有匹配反馈则过期 |
| `g_dm_master_id / g_dm_param_rx_count` | 反馈 ID 和成功读参次数 |
| `g_can_rx_total / g_can_last_rx` | 总接收数和最近帧（ID、DLC、时间戳、前 8 字节） |
| `g_can_rx_ring[32]` | 最近 32 帧的循环记录；远程帧拒收，标准/扩展数据帧接收 |
| `g_can_tx_queued / g_can_tx_failed` | FIFO 提交计数，不等于总线送达 |
| `g_can_bus_off / g_can_error_passive` | 总线错误状态 |
| `g_can_tx_error_count / g_can_rx_error_count` | 硬件收发错误计数 |
| `g_can_rx_lost / g_can_rx_failed` | FIFO 丢帧事件及读失败计数 |

停止：把 `g_dm_disable_request` 改为 1，请求失能并停止输出；仍应确认反馈 state=0。
重新开始：显式把 `g_dm_enable_request` 改为 1，重新执行失能、读参、模式确认和使能流程。
在 stage=1..3 时把 enable_request 清零可取消启动；已在 stage=4 时要用 disable_request 停止。
程序不会自动清故障或恢复 bus-off。失能报文如果因断线/bus-off 未送达，应依赖经过确认的电机超时保护或断电停止。

## 本次实机读取与代码修正

2026-11-08，通过 DAP 只读访问运行中的 STM32H723，没有发送运动指令或重烧程序。
读取旧诊断固件 Flash 与当时本地 BIN 的 SHA256 一致，排除了错误 ELF 地址。
直接读取 FDCAN2 的消息 RAM，看到：

- 反馈 CAN ID=0，数据 `01 00 33 07 00 00 00 00`：对 CAN_ID=1 的 RID7 查询回复，Master ID=0。
- 反馈 CAN ID=0，数据 `01 00 33 0A 03 00 00 00`：电机 CAN_ID=1，CTRL_MODE=3（速度模式）。
- 反馈 CAN ID=0，数据 `11 97 D4 7F F7 FF 1D 18`：状态高半字节=1（已使能），电机 ID 低半字节=1，MOS 29℃、绕组24℃。
- FDCAN2 正常模式、收发错误计数0、无 bus-off。

发现并修复旧诊断程序错误：本工程 HAL 返回未移位的 DLC=8，旧代码再次右移16位后得到0，导致数据复制与反馈解析被跳过。
新增 DLC 回归测试、MIT 编码测试和反馈状态测试。Flash/RAM 读取二进制保存在 diagnostics/dm8009_*readback.bin、dm8009_canram*.bin。
当前修正后的力矩固件只完成交叉编译和主机协议测试，尚未烧录；力矩输出与模式切换未做实机验证。

## 构建与来源

默认：`mingw32-make -j4 all DEBUG=1`。
旧 C620：`mingw32-make -j4 all DEBUG=1 LEGACY_C620=1`，对应 ELF 为 build/c620_m3508.elf，需调整调试路径。
旧被动诊断：`mingw32-make -j4 all DEBUG=1 DIAGNOSTIC=1`。

[达妙官方 SDK](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)：MIT 编码、参数读写、使能、查询和反馈布局。
[原厂 DM-J8009P-2EC 手册](https://damiao.enactic.ai/en/assets/files/dm-j8009p-2ec-v1.0-zh-976b6fed966540041c0378bef48fb178.pdf)：状态灯与基本协议；具体固件以读回参数为准。
