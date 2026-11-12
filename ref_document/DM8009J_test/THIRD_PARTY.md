# 来源与许可证

本例从同目录的 HKU_Balancing_2026 提取并精简：

- Core/Src/board.c：原 Core/Src/main.c 的系统时钟、PLL2 FDCAN 时钟，以及 Core/Src/fdcan.c 的 FDCAN2 配置和 PB5/PB6 引脚。
- Core/Src/c620_can.c：原 Robot/BSP/fdcan/bsp_fdcan.c 的 CAN_3508_2006_CMD 和 BSP_FDCAN_SendStd 的标准帧打包/发送思路。
- startup_stm32h723xx.s、Core/Src/system_stm32h7xx.c：直接复制，保留原文件版权说明。
- Core/Inc/stm32h7xx_hal_conf.h：复制后仅启用 RCC / FLASH / PWR / GPIO / CORTEX / FDCAN。
- Drivers：复制原工程中的 CMSIS/HAL 头文件和本例需要的 HAL 源文件，不依赖原工程的相对路径。

HKU 应用代码许可证见 LICENSE.HKU；ST HAL/设备头文件许可证位于 Drivers 对应目录的 LICENSE.txt。
Core/Src/board.c 的 ST 原始文件声明：

Copyright (c) 2026 STMicroelectronics. All rights reserved.
This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.

本例是独立手写示例，不是新的 CubeMX 自动生成工程；如在 CubeMX 工程中使用，应通过 CubeMX 配置时钟、GPIO 和 FDCAN，再移入 c620_can.c/.h 及主循环。
