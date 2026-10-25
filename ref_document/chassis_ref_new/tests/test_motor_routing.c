#include <assert.h>
#include <math.h>
#include <string.h>
#include "chassis_protocol.h"

int main(void)
{
    chassis_motor_route_t route;
    chassis_can_frame_t frames[CHASSIS_MOTOR_COUNT];
    float requested[CHASSIS_MOTOR_COUNT], applied[CHASSIS_MOTOR_COUNT];
    size_t index = 99U;
    bool wheel = false;
    motor_feedback_t motors[CHASSIS_MOTOR_COUNT] = {0};
    uint32_t max_age;
    bool enabled;
    assert(chassis_motor_configuration_valid());
    assert(chassis_active_motor_mask() == (1UL << BENCH_MOTOR_INDEX));
    assert(chassis_motor_route(BENCH_MOTOR_INDEX, &route));
    /* 对全部 64 种接线数量/组合验证：只要求目标在线，其他故障不阻止台架。 */
    for (uint32_t connected = 0U; connected < 64U; ++connected)
    {
        for (size_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i)
        {
            motors[i].valid = (connected & (1UL << i)) != 0U;
            motors[i].status = motors[i].valid ? 1U : 0U;
            motors[i].update_time_ms = 100U;
        }
        const bool target_present = (connected & (1UL << BENCH_MOTOR_INDEX)) != 0U;
        assert(chassis_feedback_health(motors, 102U, &max_age, &enabled) == target_present);
        assert(max_age == 2U);
        assert(enabled == (route.is_wheel || target_present));
    }
    motors[BENCH_MOTOR_INDEX].update_time_ms = 0U;
    assert(chassis_feedback_health(motors, 102U, &max_age, &enabled));
    assert(max_age == 102U); /* 将超时交给统一安全状态机锁存。 */
    motors[BENCH_MOTOR_INDEX].position_rad = NAN;
    assert(!chassis_feedback_health(motors, 102U, &max_age, &enabled));
#ifdef TEST_CUSTOM_JOINT
    assert(route.bus == 2U && route.tx_id == 5U && route.rx_id == 0x15U);
    assert(!chassis_route_can_feedback(route.bus, route.tx_id, &index, &wheel));
#endif
#ifdef TEST_CUSTOM_WHEEL
    assert(route.bus == 2U && route.tx_id == 0x200U && route.rx_id == 0x204U);
#endif
    assert(chassis_validate_can_feedback(route.bus, route.rx_id, true, true, 8U, &index, &wheel));
    assert(index == BENCH_MOTOR_INDEX && wheel == route.is_wheel);
    assert(!chassis_validate_can_feedback(route.bus, route.rx_id, false, true, 8U, &index, &wheel));
    assert(!chassis_validate_can_feedback(route.bus, route.rx_id, true, false, 8U, &index, &wheel));
    assert(!chassis_validate_can_feedback(route.bus, route.rx_id, true, true, 7U, &index, &wheel));
    assert(!chassis_route_can_feedback(route.bus == 1U ? 2U : 1U, route.rx_id, &index, &wheel));
    for (int sign = -1; sign <= 1; ++sign)
    {
        for (size_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i) requested[i] = sign * 100.0f;
        assert(chassis_encode_torques(requested, applied, frames) == 1U);
        assert(frames[0].bus == route.bus && frames[0].id == route.tx_id);
        for (size_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i)
            if (i != BENCH_MOTOR_INDEX) assert(applied[i] == 0.0f);
        if (route.is_wheel)
        {
            const size_t offset = (route.rx_id - 0x201U) * 2U;
            const int16_t current = (int16_t)((frames[0].data[offset] << 8) | frames[0].data[offset + 1U]);
            assert(fabsf(chassis_wheel_current_to_torque(current) - applied[BENCH_MOTOR_INDEX]) < 0.001f);
            for (size_t j = 0; j < 8U; ++j)
                if (j != offset && j != offset + 1U) assert(frames[0].data[j] == 0U);
        }
        else
        {
            const uint16_t code = ((frames[0].data[6] & 15U) << 8) | frames[0].data[7];
            const float torque = code * (108.0f / 4095.0f) - 54.0f;
            assert(fabsf(torque - applied[BENCH_MOTOR_INDEX]) < 0.027f);
            for (size_t j = 0; j < 6U; ++j) assert(frames[0].data[j] == 0U);
            assert((frames[0].data[6] & 0xf0U) == 0U); /* 驱动 Kp/Kd 始终为 0。 */
        }
    }
    const size_t count = chassis_encode_joint_command(0xfcU, frames);
    assert(count == (route.is_wheel ? 0U : 1U));
    if (count)
    {
        assert(frames[0].bus == route.bus && frames[0].id == route.tx_id);
        for (size_t j = 0; j < 7U; ++j) assert(frames[0].data[j] == 0xffU);
        assert(frames[0].data[7] == 0xfcU);
    }
    requested[0] = NAN;
    assert(chassis_encode_torques(requested, applied, frames) == 0U);
    return 0;
}
