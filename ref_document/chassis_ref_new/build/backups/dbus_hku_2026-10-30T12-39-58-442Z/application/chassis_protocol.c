#include "chassis_protocol.h"

#include <math.h>
#include <string.h>

#define DBUS_CHANNEL_OFFSET 1024
#define DBUS_CHANNEL_MIN (-660)
#define DBUS_CHANNEL_MAX 660
#define DBUS_SWITCH_UP 1U
#define DBUS_SWITCH_DOWN 2U
#define DBUS_SWITCH_MID 3U

/*
 * 该换算沿用本车现有标定，并把散落在旧控制器中的 13.94 减速比集中到一处。
 * 量产或上车前必须用实际减速箱和测功数据重新核对。
 */
#define WHEEL_TORQUE_TO_CURRENT \
    (20.0f * 16384.0f / 0.3f / (3591.0f / 187.0f) / CHASSIS_WHEEL_GEAR_RATIO)

static bool switch_valid(uint8_t value)
{
    return value == DBUS_SWITCH_UP || value == DBUS_SWITCH_DOWN || value == DBUS_SWITCH_MID;
}

bool chassis_feedback_health(const motor_feedback_t motor[CHASSIS_MOTOR_COUNT],
                             uint32_t now_ms, uint32_t *max_age_ms, bool *joints_enabled)
{
    if (motor == NULL || max_age_ms == NULL || joints_enabled == NULL) return false;
    bool valid = chassis_motor_configuration_valid();
    *max_age_ms = 0U;
    *joints_enabled = valid;
    for (size_t i = 0U; i < CHASSIS_MOTOR_COUNT; ++i)
    {
        if (!chassis_motor_active(i)) continue;
        const bool finite = isfinite(motor[i].position_rad) && isfinite(motor[i].speed_rad_s) &&
                            isfinite(motor[i].torque_nm);
        if (!motor[i].valid || !finite) valid = false;
        const uint32_t age = now_ms - motor[i].update_time_ms;
        if (age > *max_age_ms) *max_age_ms = age;
        if (i < CHASSIS_MOTOR_LEFT_WHEEL && (!motor[i].valid || !finite || motor[i].status != 1U))
            *joints_enabled = false;
    }
    return valid;
}

bool chassis_dbus_decode(const uint8_t frame[18], chassis_dbus_frame_t *decoded)
{
    if (frame == NULL || decoded == NULL)
    {
        return false;
    }

    decoded->channel[0] = (int16_t)(((frame[0] | (frame[1] << 8)) & 0x07ffU) - DBUS_CHANNEL_OFFSET);
    decoded->channel[1] = (int16_t)((((frame[1] >> 3) | (frame[2] << 5)) & 0x07ffU) - DBUS_CHANNEL_OFFSET);
    decoded->channel[2] = (int16_t)((((frame[2] >> 6) | (frame[3] << 2) | (frame[4] << 10)) & 0x07ffU) - DBUS_CHANNEL_OFFSET);
    decoded->channel[3] = (int16_t)((((frame[4] >> 1) | (frame[5] << 7)) & 0x07ffU) - DBUS_CHANNEL_OFFSET);
    decoded->switch_left = (uint8_t)((frame[5] >> 4) & 0x03U);
    decoded->switch_right = (uint8_t)((frame[5] >> 6) & 0x03U);
    decoded->mouse_x = (int16_t)(frame[6] | (frame[7] << 8));
    decoded->mouse_y = (int16_t)(frame[8] | (frame[9] << 8));
    decoded->mouse_z = (int16_t)(frame[10] | (frame[11] << 8));
    decoded->mouse_left = frame[12];
    decoded->mouse_right = frame[13];
    decoded->keyboard = (uint16_t)(frame[14] | (frame[15] << 8));
    decoded->channel[4] = (int16_t)((frame[16] | (frame[17] << 8)) - DBUS_CHANNEL_OFFSET);

    for (size_t i = 0; i < 5U; ++i)
    {
        if (decoded->channel[i] < DBUS_CHANNEL_MIN || decoded->channel[i] > DBUS_CHANNEL_MAX)
        {
            return false;
        }
    }

    return switch_valid(decoded->switch_left) && switch_valid(decoded->switch_right);
}

bool chassis_torque_vector_valid(const float torque_nm[CHASSIS_MOTOR_COUNT])
{
    if (torque_nm == NULL)
    {
        return false;
    }
    for (size_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i)
    {
        if (!isfinite(torque_nm[i]))
        {
            return false;
        }
    }
    return true;
}

float chassis_limit_torque(size_t motor_index, float torque_nm)
{
#if TEST_MOTOR_ENABLE
    const float limit = motor_index < CHASSIS_MOTOR_LEFT_WHEEL
                            ? CHASSIS_BENCH_JOINT_LIMIT_NM
                            : CHASSIS_BENCH_WHEEL_LIMIT_NM;
#else
    const float limit = motor_index < CHASSIS_MOTOR_LEFT_WHEEL
                            ? CHASSIS_JOINT_TORQUE_LIMIT_NM
                            : CHASSIS_WHEEL_TORQUE_LIMIT_NM;
#endif
    if (torque_nm > limit)
    {
        return limit;
    }
    if (torque_nm < -limit)
    {
        return -limit;
    }
    return torque_nm;
}

int16_t chassis_wheel_torque_to_current(float torque_nm)
{
    float current = chassis_limit_torque(CHASSIS_MOTOR_LEFT_WHEEL, torque_nm) * WHEEL_TORQUE_TO_CURRENT;
    if (current > CHASSIS_WHEEL_CURRENT_LIMIT)
    {
        current = CHASSIS_WHEEL_CURRENT_LIMIT;
    }
    else if (current < -CHASSIS_WHEEL_CURRENT_LIMIT)
    {
        current = -CHASSIS_WHEEL_CURRENT_LIMIT;
    }
    return (int16_t)lroundf(current);
}

float chassis_wheel_current_to_torque(int16_t current)
{
    return (float)current / WHEEL_TORQUE_TO_CURRENT;
}

float chassis_wheel_rpm_to_rad_s(int16_t rpm)
{
    return (float)rpm * (2.0f * 3.14159265358979323846f / 60.0f) / CHASSIS_WHEEL_GEAR_RATIO;
}

int32_t chassis_update_encoder_turns(uint16_t current, uint16_t previous, int32_t turns)
{
    if (current > previous && (uint16_t)(current - previous) > 4096U)
    {
        return turns - 1;
    }
    if (previous > current && (uint16_t)(previous - current) > 4096U)
    {
        return turns + 1;
    }
    return turns;
}

bool chassis_route_can_feedback(uint8_t bus_number, uint32_t identifier,
                                size_t *motor_index, bool *is_wheel)
{
    if (motor_index == NULL || is_wheel == NULL)
    {
        return false;
    }

    for (size_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i)
    {
        chassis_motor_route_t route;
        if (!chassis_motor_active(i)) continue;
        (void)chassis_motor_route(i, &route);
        if (bus_number == route.bus && identifier == route.rx_id)
        {
            *motor_index = i;
            *is_wheel = route.is_wheel;
            return true;
        }
    }
    return false;
}

bool chassis_validate_can_feedback(uint8_t bus_number, uint32_t identifier,
                                   bool standard_id, bool data_frame, uint8_t length,
                                   size_t *motor_index, bool *is_wheel)
{
    if (!standard_id || !data_frame || length != 8U)
    {
        return false;
    }
    return chassis_route_can_feedback(bus_number, identifier, motor_index, is_wheel);
}

size_t chassis_encode_torques(const float requested[CHASSIS_MOTOR_COUNT],
                             float applied[CHASSIS_MOTOR_COUNT],
                             chassis_can_frame_t frames[CHASSIS_MOTOR_COUNT])
{
    size_t count = 0U;
    if (applied == NULL || frames == NULL) return 0U;
    memset(applied, 0, sizeof(float) * CHASSIS_MOTOR_COUNT);
    memset(frames, 0, sizeof(*frames) * CHASSIS_MOTOR_COUNT);
    if (!chassis_torque_vector_valid(requested) || !chassis_motor_configuration_valid()) return 0U;
    for (size_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i)
    {
        chassis_motor_route_t route;
        if (!chassis_motor_active(i)) continue;
        (void)chassis_motor_route(i, &route);
        applied[i] = chassis_limit_torque(i, requested[i]);
        size_t frame_index = count;
        if (route.is_wheel)
        {
            for (size_t j = 0; j < count; ++j)
                if (frames[j].bus == route.bus && frames[j].id == route.tx_id) frame_index = j;
        }
        if (frame_index == count)
        {
            frames[count].bus = route.bus;
            frames[count++].id = route.tx_id;
        }
        uint8_t *data = frames[frame_index].data;
        if (route.is_wheel)
        {
            const uint16_t current = (uint16_t)chassis_wheel_torque_to_current(applied[i]);
            const size_t offset = 2U * (route.rx_id - 0x201U);
            data[offset] = (uint8_t)(current >> 8);
            data[offset + 1U] = (uint8_t)current;
        }
        else
        {
            const uint16_t torque = (uint16_t)((applied[i] + CHASSIS_DM_TORQUE_MAX) *
                                               4095.0f / (2.0f * CHASSIS_DM_TORQUE_MAX));
            /* MIT 的 Kp/Kd 为 0；上位控制器计算力矩，不在电调叠加 PD。 */
            data[6] = (uint8_t)(torque >> 8);
            data[7] = (uint8_t)torque;
        }
    }
    return count;
}

size_t chassis_encode_joint_command(uint8_t command,
                                   chassis_can_frame_t frames[CHASSIS_MOTOR_COUNT])
{
    size_t count = 0U;
    if (frames == NULL || !chassis_motor_configuration_valid()) return 0U;
    for (size_t i = 0; i < CHASSIS_MOTOR_LEFT_WHEEL; ++i)
    {
        chassis_motor_route_t route;
        if (!chassis_motor_active(i)) continue;
        (void)chassis_motor_route(i, &route);
        frames[count].bus = route.bus;
        frames[count].id = route.tx_id;
        memset(frames[count].data, 0xff, 7U);
        frames[count++].data[7] = command;
    }
    return count;
}
