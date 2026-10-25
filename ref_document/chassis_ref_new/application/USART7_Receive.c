#include "USART7_Receive.h"

#include <stdint.h>

#include "chassis_task.h"
#include "usart.h"
#if TEST_MOTOR_ENABLE
#include "bench_motor_test.h"
#endif

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint8_t state;
    uint8_t reserved;
    uint32_t sample_time_ms;
    uint32_t fault_flags;
    float feedback_torque_nm[CHASSIS_MOTOR_COUNT];
    float applied_torque_nm[CHASSIS_MOTOR_COUNT];
    uint32_t active_motor_mask;
    uint32_t bench_motor_index;
    float bench_position_rad;
    float bench_reference_rad;
    float bench_target_rad;
    float bench_speed_rad_s;
    float bench_requested_nm;
    uint32_t bench_faults;
    uint32_t test_motor_enabled;
} chassis_telemetry_packet_t;

__attribute__((section(".AXI_SRAM"), aligned(32))) static chassis_telemetry_packet_t tx_packet;

void USART7_receive_init(void)
{
    /* UART7 当前只用于低优先级遥测发送，不接收控制命令。 */
}

void user_send_data(void)
{
    chassis_feedback_t feedback;

    if (huart7.gState != HAL_UART_STATE_READY || !chassis_get_feedback(&feedback))
    {
        return;
    }

    tx_packet.magic = 0x544d4c52UL; /* ASCII: RLMT */
    tx_packet.version = 2U;
    tx_packet.state = (uint8_t)feedback.state;
    tx_packet.reserved = 0U;
    tx_packet.sample_time_ms = feedback.sample_time_ms;
    tx_packet.fault_flags = feedback.fault_flags;
    tx_packet.active_motor_mask = chassis_active_motor_mask();
    tx_packet.bench_motor_index = UINT32_MAX;
    tx_packet.test_motor_enabled = TEST_MOTOR_ENABLE;
#if TEST_MOTOR_ENABLE
    bench_test_status_t bench;
    bench_test_get_status(&bench);
    tx_packet.bench_motor_index = bench.motor_index;
    tx_packet.bench_position_rad = bench.controller.position_rad;
    tx_packet.bench_reference_rad = bench.controller.reference_rad;
    tx_packet.bench_target_rad = BENCH_TARGET_RAD;
    tx_packet.bench_speed_rad_s = bench.controller.speed_rad_s;
    tx_packet.bench_requested_nm = bench.controller.torque_nm;
    tx_packet.bench_faults = bench.controller.faults;
#endif
    for (size_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i)
    {
        tx_packet.feedback_torque_nm[i] = feedback.motor[i].torque_nm;
        tx_packet.applied_torque_nm[i] = feedback.applied_torque_nm[i];
    }

    (void)HAL_UART_Transmit_DMA(&huart7, (uint8_t *)&tx_packet, sizeof(tx_packet));
}
