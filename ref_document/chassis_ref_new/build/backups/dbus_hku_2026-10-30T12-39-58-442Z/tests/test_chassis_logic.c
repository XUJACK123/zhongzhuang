#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "chassis_protocol.h"
#include "chassis_safety.h"

static void make_dbus_frame(uint8_t frame[18], uint16_t raw_channel, uint8_t s1, uint8_t s2)
{
    uint64_t packed = 0U;
    memset(frame, 0, 18U);
    packed |= (uint64_t)raw_channel;
    packed |= (uint64_t)raw_channel << 11;
    packed |= (uint64_t)raw_channel << 22;
    packed |= (uint64_t)raw_channel << 33;
    packed |= (uint64_t)s1 << 44;
    packed |= (uint64_t)s2 << 46;
    for (size_t i = 0; i < 6U; ++i)
    {
        frame[i] = (uint8_t)(packed >> (8U * i));
    }
    frame[16] = (uint8_t)raw_channel;
    frame[17] = (uint8_t)(raw_channel >> 8);
}

static chassis_safety_input_t healthy_input(void)
{
    chassis_safety_input_t input = {0};
    input.dbus_valid = true;
    input.motors_valid = true;
    input.imu_valid = true;
    input.command_valid = true;
    input.command_sequence = 1U;
    input.can_tx_ok = true;
    input.joints_enabled = true;
    input.switch_left = 2U;
    return input;
}

static void test_protocol(void)
{
    uint8_t frame[18];
    chassis_dbus_frame_t decoded;
    float torque[CHASSIS_MOTOR_COUNT] = {1.0f, -2.0f, 3.0f, -4.0f, 1.5f, -1.5f};

    make_dbus_frame(frame, 1024U, 2U, 3U);
    assert(chassis_dbus_decode(frame, &decoded));
    for (size_t i = 0; i < 5U; ++i)
    {
        assert(decoded.channel[i] == 0);
    }
    assert(decoded.switch_left == 2U);
    assert(decoded.switch_right == 3U);

    make_dbus_frame(frame, 0U, 2U, 3U);
    assert(!chassis_dbus_decode(frame, &decoded));
    assert(chassis_torque_vector_valid(torque));
    torque[2] = NAN;
    assert(!chassis_torque_vector_valid(torque));
    torque[2] = INFINITY;
    assert(!chassis_torque_vector_valid(torque));

    assert(fabsf(chassis_limit_torque(CHASSIS_MOTOR_LEFT_FRONT, 80.0f) - 54.0f) < 0.001f);
    assert(fabsf(chassis_limit_torque(CHASSIS_MOTOR_RIGHT_REAR, -80.0f) + 54.0f) < 0.001f);
    assert(fabsf(chassis_limit_torque(CHASSIS_MOTOR_LEFT_WHEEL, 8.0f) - 3.3f) < 0.001f);
    assert(fabsf(chassis_limit_torque(CHASSIS_MOTOR_LEFT_WHEEL, -8.0f) + 3.3f) < 0.001f);
    for (float value = -3.3f; value <= 3.3f; value += 0.3f)
    {
        int16_t current = chassis_wheel_torque_to_current(value);
        assert(fabsf(chassis_wheel_current_to_torque(current) - value) < 0.001f);
    }

    assert(chassis_update_encoder_turns(100U, 8100U, 0) == 1);
    assert(chassis_update_encoder_turns(8100U, 100U, 0) == -1);
    assert(chassis_update_encoder_turns(2000U, 1000U, 7) == 7);

    {
        size_t motor_index = 99U;
        bool is_wheel = false;
        const struct { uint8_t bus; uint32_t id; size_t index; bool wheel; } routes[] = {
            {1U, 0x003U, CHASSIS_MOTOR_LEFT_FRONT, false},
            {1U, 0x004U, CHASSIS_MOTOR_LEFT_REAR, false},
            {2U, 0x002U, CHASSIS_MOTOR_RIGHT_FRONT, false},
            {2U, 0x001U, CHASSIS_MOTOR_RIGHT_REAR, false},
            {1U, 0x201U, CHASSIS_MOTOR_LEFT_WHEEL, true},
            {2U, 0x202U, CHASSIS_MOTOR_RIGHT_WHEEL, true},
        };
        for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i)
        {
            assert(chassis_validate_can_feedback(routes[i].bus, routes[i].id, true, true, 8U,
                                                  &motor_index, &is_wheel));
            assert(motor_index == routes[i].index && is_wheel == routes[i].wheel);
        }
        assert(!chassis_validate_can_feedback(1U, 0x202U, true, true, 8U, &motor_index, &is_wheel));
        assert(!chassis_validate_can_feedback(3U, 0x003U, true, true, 8U, &motor_index, &is_wheel));
        assert(!chassis_validate_can_feedback(1U, 0x003U, false, true, 8U, &motor_index, &is_wheel));
        assert(!chassis_validate_can_feedback(1U, 0x003U, true, false, 8U, &motor_index, &is_wheel));
        assert(!chassis_validate_can_feedback(1U, 0x003U, true, true, 4U, &motor_index, &is_wheel));
    }
}

static void test_safety(void)
{
    chassis_safety_t safety;
    chassis_safety_input_t input = healthy_input();

    chassis_safety_init(&safety);
    assert(!chassis_safety_step(&safety, &input));
    assert(safety.state == CHASSIS_STATE_STOPPED);
    assert(safety.down_seen);

    input.switch_left = 3U;
    assert(!chassis_safety_step(&safety, &input));
    assert(safety.state == CHASSIS_STATE_STANDBY);

    input.switch_left = 1U;
    assert(!chassis_safety_step(&safety, &input));
    assert(safety.state == CHASSIS_STATE_STANDBY);
    assert(!chassis_safety_step(&safety, &input));
    input.command_sequence++;
    assert(chassis_safety_step(&safety, &input));
    assert(safety.state == CHASSIS_STATE_ARMED);

    input.command_age_ms = CHASSIS_COMMAND_TIMEOUT_MS + 1U;
    assert(!chassis_safety_step(&safety, &input));
    assert((safety.fault_flags & CHASSIS_FAULT_COMMAND_TIMEOUT) != 0U);
    assert(!safety.down_seen);

    input.command_age_ms = 0U;
    assert(!chassis_safety_step(&safety, &input));
    assert((safety.fault_flags & CHASSIS_FAULT_RECOVERY_REQUIRED) != 0U);

    input.switch_left = 2U;
    assert(!chassis_safety_step(&safety, &input));
    assert(safety.state == CHASSIS_STATE_STOPPED);
    assert(safety.fault_flags == CHASSIS_FAULT_NONE);

    input.can_tx_ok = false;
    assert(!chassis_safety_step(&safety, &input));
    assert((safety.fault_flags & CHASSIS_FAULT_CAN_TX) != 0U);
    input.can_tx_ok = true;
    assert(!chassis_safety_step(&safety, &input));
    assert(safety.state == CHASSIS_STATE_STOPPED);

    input.switch_left = 1U;
    input.motor_max_age_ms = CHASSIS_MOTOR_TIMEOUT_MS + 1U;
    assert(!chassis_safety_step(&safety, &input));
    assert((safety.fault_flags & CHASSIS_FAULT_MOTOR_TIMEOUT) != 0U);

    input = healthy_input();
    chassis_safety_init(&safety);
    input.dbus_age_ms = CHASSIS_DBUS_TIMEOUT_MS + 1U;
    assert(!chassis_safety_step(&safety, &input));
    assert((safety.fault_flags & CHASSIS_FAULT_DBUS_TIMEOUT) != 0U);

    input = healthy_input();
    chassis_safety_init(&safety);
    input.imu_age_ms = CHASSIS_IMU_TIMEOUT_MS + 1U;
    assert(!chassis_safety_step(&safety, &input));
    assert((safety.fault_flags & CHASSIS_FAULT_IMU_TIMEOUT) != 0U);

    input = healthy_input();
    chassis_safety_init(&safety);
    input.command_invalid = true;
    assert(!chassis_safety_step(&safety, &input));
    assert((safety.fault_flags & CHASSIS_FAULT_COMMAND_INVALID) != 0U);

    input = healthy_input();
    chassis_safety_init(&safety);
    input.dbus_valid = false;
    assert(!chassis_safety_step(&safety, &input));
    assert((safety.fault_flags & CHASSIS_FAULT_DBUS_TIMEOUT) != 0U);

    input = healthy_input();
    chassis_safety_init(&safety);
    input.motors_valid = false;
    assert(!chassis_safety_step(&safety, &input));
    assert((safety.fault_flags & CHASSIS_FAULT_MOTOR_TIMEOUT) != 0U);

    input = healthy_input();
    chassis_safety_init(&safety);
    input.imu_valid = false;
    assert(!chassis_safety_step(&safety, &input));
    assert((safety.fault_flags & CHASSIS_FAULT_IMU_TIMEOUT) != 0U);

    input = healthy_input();
    chassis_safety_init(&safety);
    input.joints_enabled = false;
    assert(!chassis_safety_step(&safety, &input));
    assert((safety.fault_flags & CHASSIS_FAULT_JOINT_STATUS) != 0U);

    /* 故障清除后旧命令序号不能重新使能，必须先下档再上档并提交新命令。 */
    input = healthy_input();
    chassis_safety_init(&safety);
    assert(!chassis_safety_step(&safety, &input));
    input.switch_left = 1U;
    assert(!chassis_safety_step(&safety, &input));
    assert(!chassis_safety_step(&safety, &input));
    input.command_sequence++;
    assert(chassis_safety_step(&safety, &input));
}

int main(void)
{
    test_protocol();
    test_safety();
    puts("chassis logic tests passed");
    return 0;
}
