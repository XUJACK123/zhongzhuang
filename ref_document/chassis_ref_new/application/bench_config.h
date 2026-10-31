#ifndef BENCH_CONFIG_H
#define BENCH_CONFIG_H

/* 台架参数只在这里修改，VS Code / Make 不再传入这些参数。
 * 索引：0 LF、1 LR、2 RF、3 RR 为达妙关节，4 LW、5 RW 为 3508。
 * CAN_ID 是命令 ID，RX_ID 是反馈 Master ID，二者不一定相同。
 * 下列 ifndef 仅供主机路由测试覆盖，固件使用本文件值。
 */
#ifndef BENCH_MOTOR_INDEX
#define BENCH_MOTOR_INDEX 4
#endif
#ifndef BENCH_CAN_BUS
#define BENCH_CAN_BUS 1       /* 1 = CAN1，2 = CAN2 */
#endif
#ifndef BENCH_CAN_ID
#define BENCH_CAN_ID 0x200    /* 达妙命令 ID；3508 固定 0x200 */
#endif
#ifndef BENCH_RX_ID
#define BENCH_RX_ID 0x201     /* 达妙 Master ID；3508 为 0x201..0x204，对应电调 ID 1..4 */
#endif
#define BENCH_TARGET_DEG 10.0f /* 每次上档，相对当前位置 +10 度 */

#endif
