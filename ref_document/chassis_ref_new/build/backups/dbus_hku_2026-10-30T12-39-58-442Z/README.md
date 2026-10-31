# DM-MC02 / STM32H723 六电机力矩底座与单电机台架测试

更新日期：2026-10-25。本文件是本工程唯一维护的使用说明；完成情况和后续路线见 [PROGRESS.md](PROGRESS.md)。

当前完成了源码修改、交叉编译和主机逻辑测试，尚未刷写或验证真实电机、绿灯、蜂鸣器。硬件为用户确认的达妙 DM-MC02 / MC-Board02。

## 1. 目标与两种固件

长期目标是提供一个带保护的六电机力矩执行底座，由后续 RL 适配层生成六维力矩。当前先使用临时台架控制器，逐个核对接线、方向、单位和小角度运动。

| 项目 | 正式底座 | 单电机台架 |
|---|---|---|
| 构建开关 | `TEST_MOTOR=0`，命令行默认 | `TEST_MOTOR=1` |
| 控制器 | 当前没有；等待外部代码提交力矩 | 小角度轨迹 + MCU 上的 PD |
| 在线要求 | 六电机和四关节状态正常 | 只检查目标电机和目标关节状态 |
| 公共保护 | DR16、IMU、命令超时、CAN 和故障恢复 | 相同，未绕过遥控和 IMU |
| 输出目录 | `build/default/` | `build/bench/` |
| 无有效运动命令 | 零力矩请求 | 零力矩请求 |
| 启动指示 | 绿色常亮、三声蜂鸣 | 相同 |

未选中的达妙电机不发使能/控制帧；3508 必须使用组帧，其中非目标槽填零。不向未使用总线周期发送。未给非目标达妙发送帧，不等于替其他控制器关闭它。

## 2. 先构建，再选择正确镜像

在 VS Code 中直接打开 **chassis_ref_new 文件夹**，终端也进入这一目录。需要 GNU Make、Arm GNU Toolchain；主机测试另需主机 GCC。Windows 建目录和清理由 PowerShell 脚本完成，不再依赖 `rm`、`mkdir -p`。

```powershell
make -j32 TEST_MOTOR=1 BENCH_MOTOR=LF
make test
make -j32 TEST_MOTOR=0
```

第一条生成默认台架：LF 通道、CAN1、命令 ID 0x003、反馈 ID 0x003、相对 +5°。第三条生成不主动运动的正式底座。`make` 本身等价于正式底座构建；VS Code 默认构建任务则是台架任务，请区分。

两套目录分别包含 `chassis_torque_base.elf/.hex/.bin/.map`。`make clean` 清除整个本工程 build 目录，随后可重新并行构建；不要把 clean 和 all 同时作为并行目标执行。`make size TEST_MOTOR=1` 查看台架占用。

工具链不在 PATH 时：
```powershell
make -j32 TEST_MOTOR=1 TOOLCHAIN_DIR=C:/path/to/arm-toolchain/bin/
make test HOST_CC=C:/path/to/mingw/bin/gcc.exe
```

工具路径建议使用正斜杠且不带空格，`TOOLCHAIN_DIR` 保留末尾斜杠。`.build-config` 跟踪编译参数：切换电机、总线、ID、角度会自动重编；不需要每次 clean。两种固件隔离存放，不会因 TEST_MOTOR 切换复用另一模式的对象文件。只执行 `make test` 不会更新或烧录固件。

此前 PowerShell 下“找不到 rm”和“命令语法不正确”源于 Make 配方依赖 Unix shell 命令；现已由 `tools/build_helper.ps1` 处理 Windows 文件操作，并分别验证 MSYS Make 与原生 Windows Make。若仍调用了另一份 Make，用 `where.exe make` 和 `where.exe arm-none-eabi-gcc` 核对实际路径。

## 3. 指定 CAN 与电机

唯一配置表位于 `application/chassis_motor_config.c`：

| 名称 | 六维索引 | 类型 | 默认总线 | 命令 ID | 反馈 ID |
|---|---:|---|---:|---|---|
| LF 左前 | 0 | DM8009 关节 | CAN1 | 0x003 | 0x003 |
| LR 左后 | 1 | DM8009 关节 | CAN1 | 0x004 | 0x004 |
| RF 右前 | 2 | DM8009 关节 | CAN2 | 0x002 | 0x002 |
| RR 右后 | 3 | DM8009 关节 | CAN2 | 0x001 | 0x001 |
| LW 左轮 | 4 | 3508 | CAN1 | 0x200 | 0x201 |
| RW 右轮 | 5 | 3508 | CAN2 | 0x200 | 0x202 |

台架参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `BENCH_MOTOR` | LF | LF/LR/RF/RR 选择关节；LW/RW 选择轮电机 |
| `BENCH_CAN_BUS` | 0 | 0 用表；1/2 指定总线 |
| `BENCH_CAN_ID` | 0 | 0 用表；关节填实际命令 CAN ID；3508 为 0x200 |
| `BENCH_RX_ID` | -1 | -1 用表；关节填实际 Master ID；3508 填 0x201..0x204 |
| `BENCH_TARGET_DEG` | 5 | 本次使能起点的相对角度，负数反向；绝对值不得超过 15° |

达妙 CAN ID 与 Master ID 是两件事，不能假定必然相同；Master ID 可显式配置为 0。默认表沿用原接线，不代表已核实驱动器设置。覆盖路由后，LF 等名字仅表示数组通道，代码不会判断真实机械安装位置。

示例：使用第 0 通道测试 CAN2 上命令 ID=5、Master ID=0x15 的关节，反转 5°：
```powershell
make -j32 TEST_MOTOR=1 BENCH_MOTOR=LF BENCH_CAN_BUS=2 BENCH_CAN_ID=0x005 BENCH_RX_ID=0x015 BENCH_TARGET_DEG=-5
```

示例：CAN2 上电调 ID=4 的 3508：
```powershell
make -j32 TEST_MOTOR=1 BENCH_MOTOR=LW BENCH_CAN_BUS=2 BENCH_RX_ID=0x204 BENCH_TARGET_DEG=5
```

当前不支持 3508 的 5..8 号电调组。要改变正式底座接线，修改统一配置表，不要只改发送端或只改接收端。

## 4. bench_motor_test 如何起作用

`tests/bench_motor_test.c` 是编入 STM32 的临时任务，只有 TEST_MOTOR=1 才创建。它不是 PC 测试程序，也不直接访问 CAN。`tests/test_*.c` 才是由 make test 执行的主机程序。

```text
电机反馈 / BMI088 / DR16
          ↓
chassis_task：快照 + 状态机 ← 后续 RL 适配器（正式版，尚未实现）
          ↓ chassis_get_feedback()
bench_motor_test：允许条件与目标选择
          ↓
bench_controller：参考轨迹 → PD → 限幅/力矩变化率限制
          ↓ chassis_submit_torque(六维 N·m)
chassis_task：500 Hz 最终保护、限幅、唯一 CAN 输出
          ↓
chassis_protocol 编码 → CAN_receive 发送 → 驱动器
```

测试从当次允许运动时的位置开始，默认相对 +5°，达到后保持目标；退出允许条件就清零，下次允许时重新记录起点，不会自动返回原位置。关节驱动 MIT 帧中的 Kp/Kd 都为零，MCU 算出的力矩放入前馈力矩字段。

| 测试参数 | 关节 | 轮电机 |
|---|---:|---:|
| 默认相对目标 | +5° | +5°输出轴角度 |
| 参考速度 / 加速度上限 | 5°/s、10°/s² | 相同 |
| MCU PD：Kp / Kd | 5 / 0.2 | 0.5 / 0.02 |
| 台架最终力矩限幅 | ±1 N·m | ±0.2 N·m |
| 正常力矩变化率上限 | 2 N·m/s | 0.5 N·m/s |

默认参考轨迹约 1.5 s；真实到位取决于负载和参数。5 s 内误差未小于 1°且速度未小于 0.05 rad/s 并持续 300 ms，则锁存不到位故障。实测速度超过 0.5 rad/s、相对位移超过“目标绝对值 +5°”、不合理反馈跳变、非法浮点数或运行间隔超过 10 ms，也会锁存测试故障。停止直接请求零力矩，不受缓升速率限制。

**允许流程：**

1. 上电先让协议 S1 下档，等待目标反馈和 IMU 正常、关节限次初始化完成。
2. 台架运动要求两只拨杆都上档；底座自身使用 S1 管理“下档 → 上档 → 新命令”。
3. 底座进入待机后，台架任务先提交零命令完成握手，获准后才开始轨迹。
4. 任一拨杆退出上档即撤销台架运动；测试故障需 S1 下档复位，底座故障也需健康恢复后重新经过下档。
5. S1 是代码中的 `switch_left` 协议字段；上/下/中数值为 1/2/3。首次连接用遥控快照核对实际拨杆对应关系。

其他电机缺失不阻止台架；目标缺失、DBUS/IMU 不正常仍会阻止运动。绿灯亮不代表已允许运动。零力矩请求也不等于机械抱闸；调试器暂停不能当作急停。

## 5. VS Code 构建、下载和调试

图中的“运行和调试”是 VS Code 面板；本项目通过 Cortex-Debug + OpenOCD + ST-LINK 使用它。配置已经提供，不必手工新建 launch.json。

1. 打开 chassis_ref_new 根目录，确保 Cortex-Debug 已启用。
2. 接好 ST-LINK 的 SWDIO、SWCLK、GND 和目标电压参考，按实际接口核对 NRST；不在此推断连接器针序。
3. “运行和调试”选择 **台架：单电机慢速测试 (ST-LINK)**，按 F5。
4. 构建前会询问目标通道、总线、命令 ID、反馈 ID、角度。成功后使用 build/bench 中的 ELF 下载并停在 main；继续运行后才执行外设初始化和启动指示。
5. 正式模式选择 **正式底座：等待力矩命令 (ST-LINK)**，对应 build/default。
6. 只下载可运行任务 `flash: bench (ST-LINK)` 或 `flash: default (ST-LINK)`；这些任务会先构建、下载校验，再复位运行。

`.vscode/tasks.json` 和 `.vscode/launch.json` 当前使用本机 EIDE 安装的 OpenOCD / GDB 绝对路径。换电脑时更新 serverpath、gdbPath、searchDir 和 flash 任务的路径。这里配置的是 ST-LINK，不适用于未修改配置的其他探针。已验证 JSON 和镜像对应关系，未连接硬件验证 F5 或下载。

调试观察 `bench_test_status`：
- `motor_index / bus / tx_id / rx_id`：是否真正选中目标接线。
- `chassis_state / chassis_faults`：底座是否允许输出。
- `controller.active / reached / faults`：台架是否启动、到位或停止。
- `controller.position_rad / reference_rad / speed_rad_s / torque_nm`：实际相对位置、参考位置、实测速度、测试请求力矩。

如果没有运动，先确认台架 ELF 和 TEST_MOTOR=1，再查双上档、目标反馈、Master ID、IMU 和两个故障字段，避免直接增大力矩。

## 6. 保护、接口与单位

公开接口在 `Task/chassis_task.h`：

| 接口 | 行为 |
|---|---|
| `chassis_submit_torque(torque_nm)` | 一次提交 LF/LR/RF/RR/LW/RW 六个 N·m，内部记录接收时间；拒绝 NaN/Inf 并置非法命令故障 |
| `chassis_get_feedback(&feedback)` | 获取已发布快照：电机、IMU、遥控、时间戳、有效标志、状态、故障和发送请求 |
| `chassis_stop()` | 清除当前命令并请求停止；旧命令不能重新生效，后续需新命令 |

各反馈源异步采样，快照提供一致的已复制数据结构及各自时间戳，不代表六电机和 IMU 在物理上同步采样。位置/速度用 rad、rad/s，轮侧换算到输出轴；方向采用驱动器原生正方向，不隐含腿部镜像或零点变换。

| 保护项 | 默认阈值 |
|---|---:|
| 命令年龄 | 30 ms |
| DBUS 年龄 | 100 ms |
| 活动电机反馈年龄 | 20 ms |
| IMU 年龄 | 10 ms |

状态：0 STOPPED、1 STANDBY、2 ARMED、3 LOCKED。底座故障位：bit0 DBUS、bit1 电机反馈、bit2 IMU、bit3 命令超时、bit4 非法命令、bit5 CAN 发送失败、bit6 关节状态、bit7 需重新恢复。台架另有故障位：bit0 数据、bit1 时序、bit2 超速、bit3 位移越界、bit4 不到位；台架故障时底座可能仍显示 ARMED，因此必须同时查看。

底座只允许一个任务实际下发 CAN。发送成功指 HAL 接受排队，不代表电机已经确认或执行。`applied_torque_nm` 是软件限幅后的发送请求，真实反馈力矩在 motor 字段；协议量化也会引入误差。CAN 断线时软件能撤销非零请求并尝试发零帧，不能保证断线电机收到停止帧。

正式底座保留 ±54 N·m 关节、±3.3 N·m 轮软件限幅，台架收紧到 ±1/±0.2 N·m。均不是已经验证的硬件额定安全值。关节初始化为下档触发的非阻塞、限次清错/使能，不循环自动恢复故障电机。

## 7. 硬件接口与启动指示

| 功能 | 当前实现 | 需要核对 |
|---|---|---|
| DR16 | UART5 RX / PD2，100000 baud，8 有效数据位 + 偶校验 + 1 停止位（HAL 9 位字长） | 接收通道是否已有硬件反相 |
| 板载 RGB 灯 | PA7 / SPI6 驱动 WS2812，S1 上绿、中红、下蓝，未连接白色 | 颜色、波形、真实板版次 |
| 蜂鸣器 | PB15 / TIM12_CH2，4 kHz PWM，100 ms 响、150 ms 间隔，共三声 | 实物声音 |
| IMU 温控 | PB1 / TIM3_CH4 | 保持原温控职责 |
| CAN | 沿用工程 FDCAN1/2 和 CAN 接线 | 电源、终端电阻、波特率、驱动器模式及收发 ID |

DBUS 默认沿用硬件反相接收；没有板级反相时，需核实接线后通过 `EXTRA_CFLAGS=-DDBUS_RX_INVERTED=1` 启用 UART RX 反相。不要同时硬件与软件重复反相，不凭 H723 芯片型号推断连接座针序。

灯在调度器启动前设为低亮度白色，随后由低优先级任务约每 10 ms 检查遥控快照，仅在颜色变化时写入。使用协议 S1（`switch_left`）：上档绿色、中档红色、下档蓝色；未收到有效帧、接收错误或超过 100 ms 未更新时为白色。三声启动音结束后仍持续更新灯色。灯色只表示遥控连接和档位，不表示保护已解除。蜂鸣由低优先级任务推进。厂家接线依据：[DM-MC02 SPI6 示例](https://raw.githubusercontent.com/dmBots/DM-MC02/master/examples/CtrBoard-H7_WS2812/Core/Src/spi.c)、[TIM12 示例](https://raw.githubusercontent.com/dmBots/DM-MC02/master/examples/CtrBoard-H7_ALL/Core/Src/tim.c)。

### DBUS 白灯诊断

在 Cortex Live Watch 添加并展开全局变量 `remote_debug`（需要烧录包含诊断变量的新固件）。这些计数自启动累积，不直接参与电机允许判断。

| 字段 | 用途 |
|---|---|
| `dma_start_count / dma_start_fail_count / last_dma_status` | 接收启动尝试、失败次数和最近 HAL 返回值；0 表示 HAL_OK |
| `rx_event_count / last_rx_size` | 收到数据的回调次数与最近字节数；完整帧为 18 字节 |
| `valid_frame_count / last_valid_time_ms` | 已通过校验的帧数与最近有效帧时间；持续增加才说明持续收到有效数据 |
| `length_error_count / decode_error_count` | 长度不符合要求，或 18 字节数据未通过通道/开关校验 |
| `uart_error_count / last_uart_error / uart_error_bits` | 串口错误计数、最近错误码和累计错误位；在 HAL 恢复清零前保留 |
| `last_raw / last_decoded` | 最近一次回调的原始数据（不足 18 字节时补零），以及最近一次 18 字节数据的解码结果，包括未通过校验的数据 |

只有 `rx_event_count` 增加而 `valid_frame_count` 不增加时，检查长度和解码结果。现有解析要求五个通道均在 -660..660，两个拨杆均为 1/2/3；应先用原始数据确认失败原因，不能仅为消除白灯放宽校验。所有接收/错误计数均为零也不能单独证明接线错误，还需检查接收启动、串口/DMA 和中断状态。多个字段的实时读取可能跨越中断更新，以多次观察的变化趋势为准。

## 8. 文件框架与阅读顺序

建议依次阅读 main → freertos → chassis_task → bench_motor_test → bench_controller → motor_config/protocol/CAN_receive。

| 文件或目录 | 负责什么 |
|---|---|
| `Makefile` | 选择源码与固件模式，参数依赖跟踪，交叉编译及主机测试 |
| `tools/build_helper.ps1` | Windows 建目录、仅内容变化才写配置戳、限定 build 范围的清理 |
| `.vscode/tasks.json / launch.json` | VS Code 参数输入、构建、两模式下载和调试 |
| `STM32H723VGTx_FLASH.ld` | Flash/DTCM/AXI 内存段和链接符号 |
| `Drivers/.../gcc/startup_stm32h723xx.s` | 向量表、data 初始化、bss/AXI 清零、进入 main |
| `Core/Src/main.c` | 时钟和外设启动、板载指示初始化、DBUS 初始化、启动 RTOS |
| `Core/Src/freertos.c` | 创建 IMU、底盘、遥测任务；台架模式额外创建 bench 任务 |
| `Core/Inc/FreeRTOSConfig.h` | RTOS 配置，当前堆 24 KiB；仍需实测任务栈余量 |
| `Core/Src/gpio.c / dma.c / fdcan.c / spi.c / tim.c / usart.c` | 各外设底层配置 |
| `Core/Src/stm32h7xx_it.c / stm32h7xx_hal_msp.c` | 中断入口和外设 GPIO/DMA 适配 |
| `Core/Src/stm32h7xx_hal_timebase_tim.c / system_stm32h7xx.c` | HAL 时基与芯片系统配置 |
| `Core/Src/syscalls.c` | C 库底层接口，不用半主机作为调试输出路径 |
| `Task/chassis_task.c/.h` | 500 Hz 唯一电机输出，接收命令、汇总快照、保护和限幅 |
| `application/chassis_motor_config.c/.h` | 唯一通道表、活动掩码、台架路由覆盖和配置检查 |
| `application/chassis_protocol.c/.h` | 纯逻辑：DBUS 解析、路由、反馈健康检查、MIT/3508 编码和换算 |
| `application/chassis_safety.c/.h` | 纯逻辑安全状态机、超时及重新拨档 |
| `application/CAN_receive.c/.h` | HAL 收发、反馈发布、关节限次初始化；不自行生成控制力矩 |
| `application/remote_control.c/.h` | DBUS DMA + 空闲中断、合法帧更新时间、错误恢复 |
| `tests/bench_motor_test.c/.h` | STM32 台架任务、允许条件、六维力矩提交及调试状态 |
| `tests/bench_controller.c/.h` | 可在主机测试的轨迹、PD、限幅、变化率和台架故障 |
| `Task/INS_task.c/.h` | BMI088 姿态估计、IMU 快照、温控 |
| `boards/BMI088driver.* / BMI088Middleware.* / bsp_delay.*` | BMI088 寄存器、SPI/GPIO 适配及初始化延时 |
| `algorithm/kalman_filter.* / PID.*` | IMU 卡尔曼矩阵流程与温控 PID |
| `algorithm/leg_position.*` | 保留腿部正运动学工具，当前底座不调用，不输出电机力矩 |
| `boards/board_indicator.c/.h` | DM-MC02 遥控档位 RGB 指示和三声启动音 |
| `Task/send_task.c/.h` | 100 Hz 低优先级遥测及非阻塞蜂鸣推进 |
| `application/USART7_Receive.c/.h` | UART7 二进制遥测；沿用旧文件名，当前不接收控制命令 |
| `tests/test_chassis_logic.c` | 协议、单位换算、编码回绕、DBUS 与安全状态机主机测试 |
| `tests/test_bench_controller.c` | 轨迹和台架控制保护主机测试 |
| `tests/test_motor_routing.c` | 六目标/自定义 ID 编码、只依赖目标在线的组合测试 |
| `Drivers / Middlewares` | HAL、CMSIS-DSP 所需函数、FreeRTOS 10.3.1 与 GCC M7 r0p1 移植层 |

RTOS 优先级从高到低为 IMU、底盘输出、台架控制器、遥测。底盘和台架各 500 Hz，互相传递快照/命令而不跨任务共享 CAN 发送缓冲。

历史裁判、超电、图传、RS485、detect_task 等源码保留但未纳入默认构建和启动。旧 LQR/MPC/KNN、跳跃、爬阶、自救及旧 VMC 力矩生成链已移除。Makefile 是验收入口；CubeMX 的 .ioc 不应被视为已完整反映所有手工修改，重新生成前需保留并对比修改。

## 9. UART7 遥测 v2

低优先级 100 Hz 尝试发送，DMA 忙时跳过。包长 100 字节，小端；float 为 32 位。旧 v1 上位机必须更新。此包无 CRC，接收端按 magic、version、长度重新同步；调试数据不可当作控制确认。

| 偏移 | 字段 | 类型 |
|---:|---|---|
| 0 | magic，字节为 RLMT | uint32，0x544D4C52 |
| 4 | version=2 | uint16 |
| 6 / 7 | state / reserved | uint8 / uint8 |
| 8 / 12 | sample_time_ms / fault_flags | uint32 / uint32 |
| 16 | 六个反馈力矩 | float[6] |
| 40 | 六个软件下发请求力矩 | float[6] |
| 64 / 68 | active_motor_mask / bench_motor_index | uint32 / uint32 |
| 72 / 76 / 80 | 台架相对位置 / 参考位置 / 目标角度 | float，rad |
| 84 / 88 | 台架实测速度 / 请求力矩 | float，rad/s / N·m |
| 92 / 96 | bench_faults / test_motor_enabled | uint32 / uint32 |

正式版 bench_motor_index=UINT32_MAX、台架浮点字段为零。更完整的原始位置、IMU 和遥控信息通过 chassis_get_feedback() 或调试器查看。

## 10. 已修复问题与软件验收

已处理：
- Windows Make 的清理/建目录兼容，模式与参数切换复用旧镜像问题。
- 单电机测试被其他五电机在线条件阻塞；非目标总线和关节也被初始化的问题。
- 命令 ID 与反馈 ID 混用，3508 反馈数组错误按总线寻址。
- 3508 正反换算参数不一致、整数除法、编码器首帧与回绕问题。
- CAN 按总线、标准 ID、经典数据帧和 8 字节长度共同校验。
- DBUS 停止位、DMA 重启和错误恢复；只让合法帧刷新在线时间。
- 掉线后旧命令继续输出、故障恢复重放旧命令、共享发送缓冲竞争。
- 测试任务启动延时后补跑旧周期、读取时间早于快照导致年龄下溢。
- 启动指示空实现；新增独立灯/蜂鸣驱动和可观察台架故障。

2026-10-25 软件验证：
- Windows MSYS GNU Make 3.81 与原生 GNU Make 4.4.1 的干净构建、清理及主机测试已通过。
- 最新正式/台架构建无编译告警；生成 ELF、HEX、BIN、MAP。
- 同参数重复构建对象/ELF 时间戳不变；修改电机、CAN、ID、角度触发重编且不污染正式版。
- 六个选择和两个自定义路由共八组测试，各枚举 64 种连接组合；只接目标可通过反馈健康检查，缺目标不能通过。
- 轨迹正负方向、零位移、限幅/变化率、NaN/Inf、超速、位移、时序、不到位、回绕、协议和底座故障测试通过。
- 正式 ELF 不含 bench_test_task，台架 ELF 包含；启动向量 0x08000000，DMA 区 0x24000000，RTOS 中断已链接，无可写且可执行装载段。

| 固件 | Flash | DTCM（含链接保留区） | AXI SRAM |
|---|---:|---:|---:|
| 正式 | 54,144 B | 41,672 B | 160 B |
| 台架（默认 LF +5°） | 56,432 B | 41,736 B | 160 B |

已恢复交付台架参数为 LF / CAN1 / tx=rx=0x003 / +5°。主机测试未覆盖真实中断时序、CAN 电气故障、DMA 外设恢复或电机动力学，这些仍需板上验证。

## 11. 下一阶段与 RL 的关系

先逐个记录电机类型、总线、命令 ID、Master ID、方向、零点、输出轴单位、实测位移/速度和故障。DM 的 P_MAX=3.14、V_MAX=45、T_MAX=54 必须与驱动器设置一致；3508 减速比暂为 13.94，尚待实物核对。错误量程或减速比会同时影响反馈和控制，不能靠调 PD 掩盖。

在固定、卸载或悬空的单电机条件下，先核对启动指示、下档反馈和初始化，再验证 +5°、-5°、停止及反馈/遥控中断；确认后才逐个扩展到六电机。首次参数无需增大；不到位先查模式、ID、方向、反馈和负载。

XYEGA 参考应用的模型输出不能直接当作本底座六维 N·m：其动作还经过关节位置/轮速目标、PD、雅可比和气弹簧补偿等步骤。后续须明确训练自由度与六物理通道的映射、观测单位/零点、动作缩放以及必要补偿，再接独占 RL 控制器；不要让台架任务与 RL 同时提交力矩。

本轮软件交付完成。灯和蜂鸣、实际缓慢运动、停止时延、任务栈水位及整车效果尚未验收，见 PROGRESS.md 的下一步清单。
