# STM32H723 六电机力矩执行底座

本工程保留四个 DM8009 关节电机、两个 3508 轮电机、BMI088 姿态估计、DT7/DR16 急停和 UART7 遥测。旧 LQR、MPC、VMC、KNN、跳跃、爬阶和自救控制链已经移除。工程当前不包含强化学习模型，也不会自行生成运动力矩。

## 数据流

```text
FDCAN1/2 电机反馈 + BMI088 + DR16
                 │
                 ▼
          chassis_get_feedback()
                 │
        后续 RL/控制器计算力矩
                 │
                 ▼
          chassis_submit_torque()
                 │
      500 Hz 保护、限幅、故障锁存
                 │
                 ▼
       DM8009 力矩帧 + 3508 电流帧
```

公开接口位于 `Task/chassis_task.h`：

- `chassis_submit_torque()` 一次提交 6 个 N·m 力矩；NaN/Inf 会被拒绝并锁存故障。
- `chassis_get_feedback()` 获取同一时刻的电机、IMU、遥控、状态、故障和实际下发力矩快照。
- `chassis_stop()` 清除当前命令。再次运行必须提交新命令。

数组顺序固定为：左前关节、左后关节、右前关节、右后关节、左轮、右轮。所有方向均为电机驱动器原生正方向；策略坐标和机械零位转换应放在后续 RL 适配层。

## 代码框架与文件职责

### 构建、启动和调度

| 文件 | 职责 |
|---|---|
| `Makefile` | 默认构建清单和 `all/clean/size/test` 入口；只链接当前底座实际使用的任务、协议、HAL、CMSIS-DSP 和 FreeRTOS 源码。 |
| `STM32H723VGTx_FLASH.ld` | 定义 Flash、DTCM、AXI SRAM 和 D2/D3 SRAM；普通 `.data/.bss` 放 DTCM，DMA 对象放 `.AXI_SRAM`。 |
| `Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h723xx.s` | H723 GCC 启动向量，复制普通数据、清零普通 BSS 和 `.AXI_SRAM`，再进入 `main()`。 |
| `Core/Src/main.c` | 初始化时钟、GPIO、DMA、FDCAN1/2、UART5、UART7、SPI2 和 TIM3，启动 DBUS/遥测后进入 FreeRTOS。 |
| `Core/Src/freertos.c` | 只创建 3 个业务任务：IMU、500 Hz 底盘输出、低优先级遥测；不再创建裁判、检测或旧控制任务。 |
| `Core/Src/syscalls.c` | 明确禁用标准输入输出的隐式半主机依赖；调试数据不走 `printf`，统一走 UART7。 |
| `Core/Src/gpio.c`, `dma.c`, `fdcan.c`, `spi.c`, `tim.c`, `usart.c` | CubeMX 生成的底层外设配置；其中 UART5 是 DR16，UART7 是遥测，SPI2/TIM3 服务 BMI088 和温控。 |
| `Core/Src/stm32h7xx_it.c` | DMA、FDCAN、UART、SPI 和系统时基中断入口。 |

### 六电机接口和安全链

| 文件 | 职责 |
|---|---|
| `Task/chassis_task.h` | 对后续 RL 暴露唯一公共接口和统一反馈结构。 |
| `Task/chassis_task.c` | 500 Hz 唯一电机输出任务；读取一致性快照、计算超时、运行安全状态机、下发或清零力矩、发布反馈。这里不生成任何控制力矩。 |
| `application/chassis_protocol.h/.c` | 与 HAL 无关的纯逻辑：DBUS 解包、六通道编号、CAN 路由校验、力矩限幅、3508 电流/力矩/速度换算和编码器回绕。主机测试直接覆盖这一层。 |
| `application/chassis_safety.h/.c` | STOPPED/STANDBY/ARMED/LOCKED 状态机和故障位；处理四类超时、非法命令、CAN 发送失败、关节状态及“下档→上档→新命令”解锁序列。 |
| `application/CAN_receive.h/.c` | FDCAN1/2 接收和发送；按总线、标准 ID、数据帧和 8 字节长度过滤反馈；解析 4 个 DM8009 和 2 个 3508；使用函数内局部发送缓冲；执行限次、非阻塞关节清错/使能。 |
| `application/remote_control.h/.c` | UART5 的 DBUS DMA+空闲接收、18 字节帧校验、合法帧时间戳和串口错误恢复；DMA 缓冲固定在 AXI SRAM。 |

`chassis_task()` 每 2 ms 的执行顺序固定为：

1. 复制电机、IMU、遥控和最新力矩命令快照。
2. S1 下档时清除旧命令，并推进一次非阻塞关节初始化状态机。
3. 计算 DBUS、六电机、IMU 和命令数据年龄，检查关节状态及上一次 CAN 发送结果。
4. 安全状态机决定本周期是否允许非零输出。
5. 允许时逐通道限幅并下发；否则六电机全部发送零命令。
6. 发布包含实际下发量、时间戳、状态和故障位的反馈快照。

### 传感、滤波和调试

| 文件 | 职责 |
|---|---|
| `Task/INS_task.h/.c` | 读取 BMI088，运行四元数 EKF，维护 IMU 有效标志和时间戳，并执行加热片温控。 |
| `algorithm/kalman_filter.h/.c` | INS 使用的通用卡尔曼矩阵流程，内存改由 FreeRTOS 堆分配。 |
| `algorithm/PID.h/.c` | 仅保留 IMU 温控 PID；云台 PID 和旧底盘控制器依赖已删除。 |
| `algorithm/leg_position.h/.c` | 保留腿部正运动学工具。底座本身不调用它，因为原生电机方向到策略/机构坐标的映射必须由后续 RL 适配层明确给出。 |
| `boards/BMI088driver.c`, `BMI088Middleware.c`, `bsp_delay.c` | BMI088 寄存器驱动、SPI/GPIO 适配和传感器初始化延时。 |
| `Task/send_task.h/.c` | 低优先级、100 Hz 调试遥测任务，不参与电机控制。 |
| `application/USART7_Receive.h/.c` | 生成 UART7 二进制反馈包；历史文件名保留，但现在只发送，不接收控制命令。 |
| `tests/test_chassis_logic.c` | 主机单元测试，覆盖 DBUS、六路映射、异常 CAN 元数据、正负限幅、换算、回绕、各类超时、非法浮点数和重新解锁。 |

`application/referee_*`、`RS485_Receive.*`、`Image_Trans_Receive.*`、`Task/detect_task.*` 等历史协议源码仍留在目录中，但不在 Makefile 默认源清单，也不会在 `main()` 初始化。

## 构建和测试

要求 GNU Make、Arm GNU Toolchain 和主机 GCC：

```sh
make clean
make -j
make test
```

如果工具链不在 `PATH`：

```sh
make TOOLCHAIN_DIR="C:/path/to/arm-gnu-toolchain/bin/"
```

固件输出在 `build/`：ELF、HEX、BIN、MAP 和逐源文件栈占用文件。链接脚本把普通数据放在 DTCM，把带 `.AXI_SRAM` 标记的 DMA 缓冲区放在 `0x24000000`，启动代码会清零两类 BSS。

`make test` 在主机上检查 DBUS 解包、通道映射、非法帧、力矩限幅、轮电机换算、编码器回绕、命令超时、CAN 故障和重新拨档解锁。

## DBUS 与使能

DR16 接收信号使用 UART5 RX（MCU 引脚 PD2），配置为 100000 baud、8 位有效数据、偶校验、1 个停止位。DBUS 电平与普通 UART 相反；默认假定 H 板接收接口已经完成硬件反相。若所用接口没有反相，可在构建时添加 `EXTRA_CFLAGS=-DDBUS_RX_INVERTED=1`，但必须先核对 MCU 和板级电路是否支持该配置。

S1 拨杆状态：

- 下档：停止并清除旧命令，同时允许非阻塞地尝试关节电机初始化。
- 中档：待机，持续发送零力矩。
- 上档：仅在六电机、IMU、DBUS、CAN 和新力矩命令均正常时允许输出。

上电或任意故障后，必须先将 S1 拨到下档；健康状态恢复后再拨到上档，并在底座观察到上档以后提交新的有效力矩。命令序号保证拨档前的旧力矩不能重新生效。DBUS 超时 100 ms、任一电机反馈超时 20 ms、IMU 超时 10 ms、力矩命令超时 30 ms 都会立即零输出并要求重新拨档。

## CAN 映射

| 电机 | 总线 | 反馈/命令 ID |
|---|---|---|
| 左前关节 | FDCAN1 | `0x003` |
| 左后关节 | FDCAN1 | `0x004` |
| 右前关节 | FDCAN2 | `0x002` |
| 右后关节 | FDCAN2 | `0x001` |
| 左轮 | FDCAN1 | 反馈 `0x201`，命令 `0x200` 第 1 通道 |
| 右轮 | FDCAN2 | 反馈 `0x202`，命令 `0x200` 第 2 通道 |

接收端同时校验 FDCAN 实例、标准 ID、数据帧类型和 8 字节长度。每次发送使用独立局部缓冲区，避免旧工程多个任务共享发送数组造成的数据竞争。

## 已修复的主要逻辑问题

- 工程自有源码和说明已统一为 UTF-8，乱码中文注释已恢复；旧控制算法文件随算法链一并删除。
- 3508 的发送、反馈和速度统一使用集中配置的 `13.94` 减速比，去掉旧代码中的整数除法和正反换算不一致。
- 电机掉线、命令超时或 CAN 发送失败后不再保留上周期非零力矩；输出任务会立即尝试发送六路零命令。
- CAN 接收不再只按 ID 判断，避免两个总线上相同 ID、扩展帧、远程帧或错误长度污染反馈。
- 3508 第一帧反馈只建立编码器基准，不再把任意上电角度误判为跨圈。
- UART5 从 2 停止位修正为 DBUS 所需的 1 停止位；接收 DMA 改为与“逐帧重新启动”流程匹配的普通模式。
- 使能必须发生在进入上档之后的新命令上，拨档前缓存的旧力矩不会在恢复后重放。
- DM8009 的人工恢复流程按原工程协议改为先发 `0xFB` 清故障、再发 `0xFC` 使能；每次重新进入下档只执行一轮限次、非阻塞尝试。
- FDCAN 只保留实际使用的发送 FIFO，并为“2 个关节特殊帧 + 3 个零力矩帧”的最坏突发配置 8 个槽位，避免初始化周期产生伪发送故障。

## 软件验收结果

- `make clean && make all`：通过，生成 ELF、HEX、BIN 和 MAP。
- 再次执行 `make all`：仅运行镜像大小检查，增量依赖有效。
- `make test`：通过，输出 `chassis logic tests passed`。
- 当前镜像占用：Flash 52,112 B，DTCM 32,232 B，AXI SRAM 96 B。
- 向量表位于 `0x08000000`，DMA 缓冲区位于 `0x24000000`；Reset、SVC、PendSV 和 SysTick 入口均已链接，镜像没有可写且可执行的装载段。

## 参数和验收边界

- DM8009 软件限幅：±54 N·m。
- 3508 软件限幅：±3.3 N·m。
- 3508 减速比暂按旧工程实际输出路径使用 `13.94`，电流/力矩使用同一组互逆公式。
- 上述参数仅用于保持当前代码基准，不代表已经验证的实车安全额定值。上车前必须核对实际减速箱、电调电流定义、电机方向和零位。
- 裁判系统、超级电容、FDCAN3 云台控制和功率分配不进入默认构建；原协议文件保留，后续可作为独立模块接回。
- 当前验证只覆盖源码、交叉编译和主机逻辑测试；未刷写、未使能电机、未做整车测试。
