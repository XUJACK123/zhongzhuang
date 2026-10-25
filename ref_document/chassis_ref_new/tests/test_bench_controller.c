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
    assert(bench_controller_step(&c, 1250U, false, false, false, 0.0f, 0.0f) == 0.0f);
    assert(!c.active);
    assert(bench_controller_step(&c, 5000U, true, false, false, 1.0f, 0.0f) == 0.0f);
    float previous = 0.0f;
    for (uint32_t t = 2U; t <= 2000U; t += 2U)
    {
        float p, v;
        bench_trajectory(t * 0.001f, BENCH_TARGET_RAD, &p, &v);
        const float torque = bench_controller_step(&c, 5000U + t, true, false, false, 1.0f + p, v);
        assert(fabsf(torque) <= 1.0f);
        assert(fabsf(torque - previous) <= 2.0f * 0.002f + 1e-5f);
        assert(c.faults == 0U);
        previous = torque;
    }
    assert(c.reached);
    assert(bench_controller_step(&c, 7002U, false, false, false, 1.1f, 0.0f) == 0.0f);
    assert(!c.active);
    assert(bench_controller_step(&c, 8000U, true, false, false, 2.0f, 0.0f) == 0.0f);
    assert(c.position_rad == 0.0f && c.start_ms == 8000U);
    assert(bench_controller_step(&c, 8011U, true, false, false, 2.0f, 0.0f) == 0.0f);
    assert(c.faults & BENCH_FAULT_TIMING);
    assert(bench_controller_step(&c, 8013U, true, false, false, 2.0f, 0.0f) == 0.0f);
    assert(!c.active); /* 故障后持续上档不得重启。 */
    bench_controller_step(&c, 8015U, false, true, false, 2.0f, 0.0f);
    assert(c.faults == 0U);
    bench_controller_step(&c, 8017U, true, false, false, 2.0f, 0.6f);
    assert(c.faults & BENCH_FAULT_OVERSPEED);

    bench_controller_reset(&c);
    bench_controller_step(&c, 0U, true, false, false, NAN, 0.0f);
    assert(c.faults & BENCH_FAULT_DATA);
    bench_controller_reset(&c);
    bench_controller_step(&c, 0U, true, false, false, 0.0f, INFINITY);
    assert(c.faults & BENCH_FAULT_DATA);

    /* 关节跨编码零点仍是一个小正向位移。 */
    bench_controller_reset(&c);
    bench_controller_step(&c, 0U, true, false, false, CHASSIS_DM_POSITION_MAX - 0.001f, 0.0f);
    bench_controller_step(&c, 2U, true, false, false, -CHASSIS_DM_POSITION_MAX + 0.001f, 0.0f);
    assert(fabsf(c.position_rad - 0.002f) < 1e-5f && c.faults == 0U);
    bench_controller_reset(&c);
    bench_controller_step(&c, UINT32_MAX - 1U, true, false, true, 0.0f, 0.0f);
    bench_controller_step(&c, 0U, true, false, true, 0.0f, 0.0f);
    assert(c.faults == 0U); /* 毫秒计数回绕。 */

    for (unsigned int wheel = 0U; wheel < 2U; ++wheel)
    {
        bench_controller_reset(&c);
        previous = 0.0f;
        bench_controller_step(&c, 0U, true, false, wheel != 0U, 0.0f, 0.0f);
        for (uint32_t t = 2U; t <= 5002U; t += 2U)
        {
            const float torque = bench_controller_step(&c, t, true, false, wheel != 0U, 0.0f, 0.0f);
            assert(fabsf(torque) <= (wheel ? 0.20f : 1.0f));
            if (t <= 5000U) assert(fabsf(torque - previous) <= (wheel ? 0.5f : 2.0f) * 0.002f + 1e-6f);
            previous = torque;
        }
        assert(c.faults & BENCH_FAULT_NOT_REACHED);
        assert(c.torque_nm == 0.0f);
    }
    bench_controller_reset(&c);
    for (uint32_t t = 0U; t <= 1000U; t += 2U)
        bench_controller_step(&c, t, true, false, true, t * 0.0002f, 0.2f);
    assert(c.faults & BENCH_FAULT_TRAVEL);
}

int main(void)
{
    test_trajectory();
    test_control();
    puts("bench trajectory and protection tests passed");
    return 0;
}
