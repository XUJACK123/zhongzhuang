# STM32H723 / C620(M3508) CAN ID 自动枚举

本工程只保留 STM32H723 启动、时钟、HAL 和 FDCAN，用于**寻找 C620 电调的 CAN ID**，找到后可让电机轻微动作确认。

## 1. 固件行为

`Core/Src/main.c` 每轮依次尝试 8 组候选引脚，每组：

1. 回环自检（验证 FDCAN 内核和时钟，**不能**证明外部收发器）。
2. 正常模式：向 `0x200` 和 `0x1FF` 发送**全零电流**（电机不输出力矩），并监听所有标准帧。
3. 把 RX 引脚设为输入并下拉读取电平，判断该引脚是否被外部器件拉高。

电调收到合法控制帧后会回报 `0x201..0x208`，命中即记录 ID。

候选引脚：

| # | 外设 | TX | RX | AF |
|---|------|----|----|----|
| 0 | FDCAN1 | PD1 | PD0 | 9 |
| 1 | FDCAN2 | PB6 | PB5 | 9 |
| 2 | FDCAN3 | PD13 | PD12 | 9 |
| 3 | FDCAN1 | PB9 | PB8 | 9 |
| 4 | FDCAN1 | PA12 | PA11 | 9 |
| 5 | FDCAN1 | PH13 | PH14 | 9 |
| 6 | FDCAN2 | PB13 | PB12 | 9 |
| 7 | FDCAN2 | PG1 | PG0 | 9 |

识别成功后 `CONFIRM_PULSE_ENABLE=1` 会常驻该总线：每 2 s 给 150 ms 的 `+300` 原始电流，电机会轻抖一下；其余时间发零电流。不需要动作时把该宏改为 0。

## 2. 结果变量（调试器 Watch / GDB）

| 变量 | 含义 |
|---|---|
| `g_scan.found` | 0=总线上没有任何节点；1=收到非 C620 帧；2=已识别电调 |
| `g_scan.instance` | 1/2/3 = FDCAN1/2/3 |
| `g_scan.port` | ASCII 端口字母，如 68='D' |
| `g_scan.tx_pin` / `rx_pin` | HAL 引脚掩码，2=PIN_1，1=PIN_0 |
| `g_scan.esc_id` | **电调 ID（1..8）** |
| `g_scan.command_id` | 该电调的组帧 ID：`0x200`（ID1..4）或 `0x1FF`（ID5..8） |
| `g_scan.slot` | 组帧内的槽位（0..3） |
| `g_probe[i].rx_count` | 该组合收到的帧数 |
| `g_probe[i].tec` | 发送错误计数，>0 且无接收说明没有对端 ACK |
| `g_probe[i].rx_idle` | RX 下拉后仍为高 = 该引脚被外部器件拉高 |

反馈 ID 与电调 ID 的对应：`0x201 -> 1`、`0x202 -> 2` … `0x208 -> 8`；命令帧中第 n 个电调占 `2*(n-1)` 起的两字节。

## 3. 构建与烧录

```powershell
make
```

产物在 `build/minimal/`。用 ST-Link + OpenOCD 烧录：

```powershell
openocd -s <openocd>/share/openocd/scripts `
  -f interface/stlink.cfg -f target/stm32h7x.cfg `
  -c "program build/minimal/m3508_can_test.elf verify reset exit"
```

Keil 也可直接打开 `MDK-ARM/Balanced-infantry.uvprojx` 后 F8 下载。

## 4. 当前实测结论

在 ST-Link 连接的这块 H723 上烧录并读取（`g_probe` 数组）：

| 组合 | 总线 | 引脚 | `echo_high`/`echo_low` | `tec` | `rx_count` |
|---|---|---|---|---|---|
| 0 | FDCAN1 | PD1/PD0 | 1 / 0（收发器正常） | 248，bus-off | 0 |
| 1 | FDCAN2 | PB6/PB5 | 1 / 0（收发器正常） | 128 | 0 |
| 2 | FDCAN3 | PD13/PD12 | 1 / 0（收发器正常） | 0 | 0 |
| 3 | FDCAN1 | PB9/PB8 | 1 / 1 | 248 | 0 |
| 4 | FDCAN1 | PA12/PA11 | 0 / 0 | 248 | 0 |
| 5 | FDCAN1 | PH13/PH14 | 0 / 0 | 0 | 0 |
| 6 | FDCAN2 | PB13/PB12 | 1 / 1 | 252 | 0 |
| 7 | FDCAN2 | PG1/PG0 | 0 / 0 | 0 | 0 |

判定方法：把 TX 引脚用开漏拉低，若 RX 跟着变低（`echo_high=1, echo_low=0`），说明该引脚上有**已供电并在工作的 CAN 收发器**。

结论：

- 板上三路 CAN 收发器（FDCAN1 PD0/PD1、FDCAN2 PB5/PB6、FDCAN3 PD12/PD13）都正常，MCU 侧无问题。
- 但三路总线上**都没有任何其它节点应答**（无 ACK，`rx_count` 全为 0）。CAN 的 ACK 与 ID 无关——任何已上电的节点都会对任意合法帧应答，因此可以确定：电调在电气上并没有参与这三路任何一路的通信。
- 已排除：MCU、时钟（24 MHz 已用 25 MHz 反证法确认）、引脚复用、电源使能（PC14/PC15/PA0）、比特率（1 Mbps / 500 kbps / 250 kbps）、CAN_H/CAN_L 极性（两种都试过）、物理通断（板子单端 120 Ω，接电调后 60 Ω）。
- 追加实测：把 CAN2 槽位 0..7 依次各通 1.5 s 微小电流（用外部回环发送，不依赖 ACK），电机**完全无反应**，说明电调连合法的电流帧都没有收到，不是 ID 不匹配。

因此问题在电调一侧：其 CAN 收发器未工作、未处于 CAN 控制模式，或接线接到了非 CAN 端口。

需要检查的物理连接：C620 的 CAN_H/CAN_L 是否接到本板的 CAN1/CAN2/CAN3 接口（**H/L 接反同样会造成“收发器正常但无 ACK”**）、电调 24 V 与共地、总线 120 Ω 终端电阻、线缆是否接在正确的接插件上。修好后无需改代码，固件会自动识别并给出 ID。
