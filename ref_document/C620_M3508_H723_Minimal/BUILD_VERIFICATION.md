# 构建与静态验证记录（2026-11-06）

- 命令：mingw32-make -j4 all DEBUG=1
- GNU Arm GCC 14.3.1；Cortex-M7 / Thumb / FPv5-D16 / hard-float。
- 最终完整构建退出码 0，无编译或链接警告。
- Flash 使用 11520 B；RAM 使用 4824 B（包含预留 4096 B 栈及 512 B 堆）。
- 已生成 build/c620_m3508.elf、.hex、.bin、.map。
- 从最终 ARM ELF 和 BIN 检查：初始 MSP = 0x24050000；Reset 向量与 ELF 入口/Reset_Handler 一致；SysTick 向量指向本例的 HAL_IncTick handler。
- ELF 中 g_c620_current 的初始值确认为 1000；无同时可写和可执行的加载段。
- 源码核对：经典标准数据帧、8 字节、大端电流、1~4 号 0x200 / 5~8 号 0x1FF、电流夹限 ±16384、约 2 ms 轮询发送。

本记录只证明交叉构建和上述静态检查通过。未连接调试器、未烧录、未验证 CAN 实际帧或电机动作。

## VS Code / Make 验证

- 新增 Makefile 完整构建成功，无编译/链接警告；ELF/HEX/BIN 已更新。
- 已处理本机继承 DEBUG=release 的情况：Make 默认 DEBUG=1，VS Code 构建任务也明确传入 DEBUG=1，保留 -Og/-g3。
- 离线 GDB 能识别 g_c620_current 的地址、g_c620_feedback 的字段类型，以及 main 的源码行。命令行 GDB 在打印类型时出现 CP1252 转 UTF-32 的字符集提示，但完成了类型和源码行查询；未在 VS Code 中连接硬件验证其界面表现。
- 五份 .vscode JSON 可解析，Cortex-Debug 字段与本机 1.12.1 扩展声明一致，preLaunchTask 引用和工具/SVD 路径均存在。
- ST-Link / CMSIS-DAP / J-Link 的 OpenOCD 配置均在未执行 init 的情况下解析后 shutdown，退出码为 0。
- make -n download_stlink download_dap download_jlink 已核对烧录命令，仅打印、未执行。
- build.ps1 已改为 Make 的兼容入口，增量调用验证通过。
- 未连接、复位、烧录或运行硬件，未实际启动 VS Code 调试会话。
