#ifndef CHASSIS_MOTOR_CONFIG_H
#define CHASSIS_MOTOR_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 正式版保留六路；台架版只使能、检查和控制所选一路。 */
#ifndef TEST_MOTOR_ENABLE
#define TEST_MOTOR_ENABLE 0
#endif
#include "bench_config.h"

#if TEST_MOTOR_ENABLE && (BENCH_MOTOR_INDEX < 0 || BENCH_MOTOR_INDEX > 5)
#error "BENCH_MOTOR_INDEX must be 0..5"
#endif
#if TEST_MOTOR_ENABLE && (BENCH_CAN_BUS < 0 || BENCH_CAN_BUS > 2)
#error "BENCH_CAN_BUS must be 0 (table), 1 or 2"
#endif
#if TEST_MOTOR_ENABLE && (BENCH_CAN_ID < 0 || BENCH_CAN_ID > 0x7ff || BENCH_RX_ID < -1 || BENCH_RX_ID > 0x7ff)
#error "CAN IDs must fit 11 bits (RX -1 means table)"
#endif
#if TEST_MOTOR_ENABLE && (BENCH_MOTOR_INDEX >= 4) && (BENCH_CAN_ID != 0) && (BENCH_CAN_ID != 0x200)
#error "3508 command ID must be 0x200"
#endif
#if TEST_MOTOR_ENABLE && (BENCH_MOTOR_INDEX >= 4) && (BENCH_RX_ID != -1) && (BENCH_RX_ID < 0x201 || BENCH_RX_ID > 0x204)
#error "3508 feedback ID must be 0x201..0x204"
#endif

/* 必须与达妙调试助手内的 P_MAX/V_MAX/T_MAX 一致。 */
#define CHASSIS_DM_POSITION_MAX 3.14f
#define CHASSIS_DM_SPEED_MAX 45.0f
#define CHASSIS_DM_TORQUE_MAX 54.0f
#define CHASSIS_BENCH_JOINT_LIMIT_NM 1.0f
#define CHASSIS_BENCH_WHEEL_LIMIT_NM 0.60f

typedef struct
{
    uint8_t bus;
    uint16_t tx_id;
    uint16_t rx_id;
    bool is_wheel;
} chassis_motor_route_t;

bool chassis_motor_route(size_t index, chassis_motor_route_t *route);
uint32_t chassis_active_motor_mask(void);
bool chassis_motor_active(size_t index);
bool chassis_motor_configuration_valid(void);

#endif
