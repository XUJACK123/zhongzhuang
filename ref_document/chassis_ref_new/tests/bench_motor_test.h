#ifndef BENCH_MOTOR_TEST_H
#define BENCH_MOTOR_TEST_H
#include "bench_controller.h"
typedef struct
{
    bench_controller_t controller;
    uint32_t motor_index, bus, tx_id, rx_id;
    uint32_t chassis_faults, chassis_state;
} bench_test_status_t;
/* GDB 可直接查看；其他任务通过 getter 取得一致快照。 */
extern volatile bench_test_status_t bench_test_status;
void bench_test_get_status(bench_test_status_t *status);
void bench_test_task(void const *argument);
#endif
