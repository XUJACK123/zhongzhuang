#include "chassis_motor_config.h"
#include "chassis_protocol.h"

/* 唯一硬件映射表：总线、命令 ID、反馈 ID、类型。不要把 Master ID 当成 CAN ID。 */
static const chassis_motor_route_t routes[CHASSIS_MOTOR_COUNT] = {
    {1U, 0x003U, 0x003U, false}, /* 左前 */
    {1U, 0x004U, 0x004U, false}, /* 左后 */
    {2U, 0x002U, 0x002U, false}, /* 右前 */
    {2U, 0x001U, 0x001U, false}, /* 右后 */
    {1U, 0x200U, 0x201U, true},  /* 左轮，0x200 的第 1 槽 */
    {2U, 0x200U, 0x202U, true},  /* 右轮，0x200 的第 2 槽 */
};

bool chassis_motor_route(size_t index, chassis_motor_route_t *route)
{
    if (index >= CHASSIS_MOTOR_COUNT || route == NULL) return false;
    *route = routes[index];
#if TEST_MOTOR_ENABLE
    if (index == BENCH_MOTOR_INDEX)
    {
#if BENCH_CAN_BUS != 0
        route->bus = BENCH_CAN_BUS;
#endif
#if BENCH_CAN_ID != 0
        route->tx_id = BENCH_CAN_ID;
#endif
#if BENCH_RX_ID >= 0
        route->rx_id = BENCH_RX_ID;
#endif
    }
#endif
    return true;
}

uint32_t chassis_active_motor_mask(void)
{
#if TEST_MOTOR_ENABLE
    return 1UL << BENCH_MOTOR_INDEX;
#else
    return (1UL << CHASSIS_MOTOR_COUNT) - 1UL;
#endif
}

bool chassis_motor_active(size_t index)
{
    return index < CHASSIS_MOTOR_COUNT &&
           (chassis_active_motor_mask() & (1UL << index)) != 0U;
}

bool chassis_motor_configuration_valid(void)
{
    for (size_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i)
    {
        chassis_motor_route_t a;
        if (!chassis_motor_active(i)) continue;
        (void)chassis_motor_route(i, &a);
        if (a.bus < 1U || a.bus > 2U || a.tx_id == 0U || a.tx_id > 0x7ffU || a.rx_id > 0x7ffU)
            return false;
        /* 当前 3508 支持电调 ID 1..4，统一命令帧 0x200。 */
        if (a.is_wheel && (a.tx_id != 0x200U || a.rx_id < 0x201U || a.rx_id > 0x204U))
            return false;
        for (size_t j = 0; j < i; ++j)
        {
            chassis_motor_route_t b;
            if (!chassis_motor_active(j)) continue;
            (void)chassis_motor_route(j, &b);
            if (a.bus == b.bus && (a.rx_id == b.rx_id ||
                (a.tx_id == b.tx_id && !(a.is_wheel && b.is_wheel)))) return false;
        }
    }
    return true;
}
