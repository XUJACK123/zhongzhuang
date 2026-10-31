#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "bench_controller.h"
#include "chassis_motor_config.h"

static void test_trajectory(void)
{
    const float goals[] = {BENCH_TARGET_RAD, -BENCH_TARGET_RAD, 0.01f, 0.0f};
    for (unsigned int j = 0; j < sizeof(goals) / sizeof(goals[0]); ++j)
    {
        float previous_p = 0.0f, previous_v = 0.0f;
        for (unsigned int k = 0; k <= 3000U; ++k)
        {
            float p, v;
            bench_trajectory(k * 0.002f, goals[j], &p, &v);
            assert(fabsf(p) <= fabsf(goals[j]) + 1e-6f);
            assert(fabsf(v) <= BENCH_SPEED_RAD_S + 1e-6f);
            assert(fabsf(v - previous_v) <= BENCH_ACCEL_RAD_S2 * 0.002f + 1e-5f);
            assert(fabsf(p - previous_p) <= BENCH_SPEED_RAD_S * 0.002f + 1e-6f);
            if (goals[j] > 0.0f) assert(p >= previous_p - 1e-6f);
            if (goals[j] < 0.0f) assert(p <= previous_p + 1e-6f);
            previous_p = p; previous_v = v;
        }
        assert(fabsf(previous_p - goals[j]) < 1e-6f && previous_v == 0.0f);
    }
}

static void test_control(void)
{
    bench_controller_t c;
    bench_controller_reset(&c);
    assert(fabsf(BENCH_TARGET_RAD - 0.1745329252f) < 1e-6f);
    assert(bench_controller_step(&c, 0U, false, false, false, 1.0f, 0.0f) == 0.0f);
    assert(!c.active);
    assert(bench_controller_step(&c, 1000U, true, false, false, 1.0f, 0.0f) == 0.0f);
    const uint32_t start = c.start_ms;
    for (uint32_t t = 2U; t <= 9000U; t += 2U)
    {
        float p, v;
        bench_trajectory(t * 0.001f, BENCH_TARGET_RAD, &p, &v);
        const float torque = bench_controller_step(&c, 1000U + t, true, false, false, 1.0f + p, v);
        assert(fabsf(torque) <= CHASSIS_BENCH_JOINT_LIMIT_NM);
        assert(c.faults == 0U && c.start_ms == start); /* 持续上档不反复增加 10 度 */
    }
    assert(c.reached);
    assert(fabsf(c.position_rad - BENCH_TARGET_RAD) < 1e-5f);
    assert(fabsf(c.reference_rad - BENCH_TARGET_RAD) < 1e-5f);
    assert(bench_controller_step(&c, 7002U, false, false, false, 1.1f, 0.0f) == 0.0f);
    assert(!c.active);
    assert(bench_controller_step(&c, 8000U, true, false, false, 2.0f, 0.0f) == 0.0f);
    assert(c.start_ms == 8000U && c.position_rad == 0.0f && c.previous_position == 2.0f);
    bench_controller_step(&c, 8011U, true, false, false, 2.0f, 0.6f);
    assert(c.faults == 0U && c.active); /* 不再锁存间隔/超速故障 */
    bench_controller_step(&c, 8013U, true, false, false, 2.5f, 0.0f);
    assert(c.faults == 0U && c.active); /* 不再锁存行程/跳变故障 */

    bench_controller_reset(&c);
    bench_controller_step(&c, 0U, true, false, false, NAN, 0.0f);
    assert(c.faults == BENCH_FAULT_DATA && c.torque_nm == 0.0f);
    bench_controller_step(&c, 2U, true, false, false, 0.0f, 0.0f);
    assert(c.active && c.faults == 0U); /* 数据恢复即恢复，不要求下档 */
    bench_controller_step(&c, 4U, true, false, false, 0.0f, INFINITY);
    assert(c.faults == BENCH_FAULT_DATA && c.torque_nm == 0.0f);

    bench_controller_reset(&c);
    bench_controller_step(&c, 0U, true, false, false, CHASSIS_DM_POSITION_MAX - 0.001f, 0.0f);
    bench_controller_step(&c, 2U, true, false, false, -CHASSIS_DM_POSITION_MAX + 0.001f, 0.0f);
    assert(fabsf(c.position_rad - 0.002f) < 1e-5f);
    bench_controller_reset(&c);
    bench_controller_step(&c, UINT32_MAX - 1U, true, false, true, 0.0f, 0.0f);
    bench_controller_step(&c, 0U, true, false, true, 0.0f, 0.0f);
    assert(c.faults == 0U);

    for (unsigned int wheel = 0U; wheel < 2U; ++wheel)
    {
        bench_controller_reset(&c);
        float previous = 0.0f;
        bench_controller_step(&c, 0U, true, false, wheel != 0U, 0.0f, 0.0f);
        for (uint32_t t = 2U; t <= 6000U; t += 2U)
        {
            const float torque = bench_controller_step(&c, t, true, false, wheel != 0U, 0.0f, 0.0f);
            const float limit = wheel ? CHASSIS_BENCH_WHEEL_LIMIT_NM : CHASSIS_BENCH_JOINT_LIMIT_NM;
            const float slew = wheel ? BENCH_WHEEL_SLEW_NM_S : BENCH_JOINT_SLEW_NM_S;
            assert(fabsf(torque) <= limit);
            assert(fabsf(torque - previous) <= slew * 0.002f + 1e-6f);
            previous = torque;
        }
        assert(c.active && c.faults == 0U); /* 不再有 5 秒不到位锁存 */
        assert(bench_controller_step(&c, 6002U, false, false, wheel != 0U, 0.0f, 0.0f) == 0.0f);
    }
}

int main(void)
{
    test_trajectory();
    test_control();
    puts("bench 10-degree trajectory and stop tests passed");
    return 0;
}
