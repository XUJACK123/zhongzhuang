# STM32F407 + C620 + M3508：缓慢转动 10°

## 默认行为

本例控制 CAN1 上 ID=1 的 C620。上电后先发送零电流，等待至少 500 ms；
收到有效反馈且电机接近静止时，把当前位置记为零点，自动执行一次输出轴 +10° 转动，随后持续闭环保持。
复位会以新的当前位置为起点再执行一次。没有绝对机械零点，也不会自动往返。

默认减速比按 19:1 近似计算，角度是减速箱输出轴角度。
目标轨迹以 5°/s 增加，2 秒走完目标轨迹，实际到位还需要闭环收敛时间。
速度指令也限制为 5°/s；这不代表实物速度在所有负载下都绝对不超限。
位置 P 环产生转速目标，速度 PI 环产生 C620 电流指令。
误差小于 0.2° 且转子转速绝对值不超过 3 rpm，持续 200 ms 后状态变为 HOLDING。
0.2° 是软件到位判据，实际精度还受减速比、齿隙和调参影响。

## 接线与工程前提

- 保留原工程 STM32F407、12 MHz HSE、168 MHz SYSCLK、42 MHz APB1 配置。
  若你的开发板晶振不是 12 MHz，需要先修改时钟配置，不能仅凭 F407 型号直接套用。
- CAN1：PD0=RX，PD1=TX，必须通过板载或外置 CAN 收发器连接 C620 的 CANH/CANL；
  MCU 引脚不能直接连接 CANH/CANL。总线两端正确配置 120 Ω 终端电阻。
- CAN 位速率：42 MHz / [3 × (1+10+3)] = 1 Mbps。
- C620 设置为 ID 1，连接 M3508 的三相线和 7-pin 反馈线，并正确供电。
- 发送 0x200，接收 0x201；同组 ID 2、3、4 的电流字段始终为零。
  此例独占该组命令，不要与另一个发送 0x200 的控制任务同时使用。

## 文件与参数

- `bsp/motor_position.h`：目标角度、速度上限、减速比、PI 参数、电流限制。
- `bsp/motor_position.c`：约 1 ms 更新，等待/运动/保持/故障状态机。
- `bsp/bsp_can.c`：标准数据帧过滤、反馈保存、8192 编码器跨零累计、原子快照。
- `Src/main.c`：当前实际入口，保留裸机主循环，调用控制模块。
- `Src/freertos.c`：备用任务入口也已改为调用相同模块；当前调度器未启用。
- `backup_before_10deg/`：修改前文件备份。

默认参数：

```c
#define MOTOR_MOVE_DEG         10.0f  // 改为 -10.0f 可反向
#define MOTOR_MAX_DEG_S        5.0f   // 降低此值可更慢；同步增加运动超时
#define MOTOR_GEAR_RATIO       19.0f  // 如果要控制转子本身 10°，改为 1.0f
#define MOTOR_CURRENT_LIMIT    2000.0f
```

C620 的 ±16384 命令对应 ±20 A 转矩电流，2000 约对应 2.44 A，
不是 2000 mA。反馈电流也保留为协议原始值 `current_raw`。
PI 参数是待实机调试的初始值；低速摩擦可能导致爬行、偏差或振荡。

## 故障与观察

启动 3 秒未满足条件、运行中反馈超过 50 ms 未更新、控制周期超过 20 ms、
CAN 错误/发送失败、反馈温度达到 70°C，或 10 秒内未首次到位，会锁定 MOTOR_FAULT。
故障状态丢弃待发帧并尝试持续发送零电流，复位才重新启动；总线物理断开时无法保证零电流帧送达。
故障时不保持位置，机械负载可能自行移动。

调试器观察 `motor_state`（0 等待、1 运动、2 保持、3 故障）、
`motor_angle_deg`（相对输出轴角度）、`motor_reference_deg`（缓慢增加的目标）。
首次验证应固定电机、卸载或解除危险负载；烧录后启动程序会自动转动。
停止在断点上会中断控制更新，不应在带电运动时依赖断点控制停机。

## 构建与验证范围

```powershell
make -j4 BUILD_DIR=build_10deg
```

交叉编译、链接和 HEX/BIN 生成已通过。新固件在 `build_10deg/`，旧 `build/` 没有更新。
没有烧录，没有验证实物方向、速度、角度精度或 PI 稳定性。

协议参考：[DJI C620 使用说明](https://cdn-hz.robomaster.com/robomasters/public/document/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.0.pdf)。
减速比参考：[DJI M3508 P19 使用说明](https://cdn-hz.robomaster.com/robomasters/public/document/RoboMaster%20M3508%20P19%20Brushless%20DC%20Gear%20Motor%20V1.0.pdf)。
