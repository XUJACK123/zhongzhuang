#include "chassis_protocol.h"

#include <math.h>

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
    const float limit = motor_index < CHASSIS_MOTOR_LEFT_WHEEL
                            ? CHASSIS_JOINT_TORQUE_LIMIT_NM
                            : CHASSIS_WHEEL_TORQUE_LIMIT_NM;
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

    if (bus_number == 1U)
    {
        if (identifier == 0x003U) { *motor_index = CHASSIS_MOTOR_LEFT_FRONT; *is_wheel = false; return true; }
        if (identifier == 0x004U) { *motor_index = CHASSIS_MOTOR_LEFT_REAR; *is_wheel = false; return true; }
        if (identifier == 0x201U) { *motor_index = CHASSIS_MOTOR_LEFT_WHEEL; *is_wheel = true; return true; }
    }
    else if (bus_number == 2U)
    {
        if (identifier == 0x002U) { *motor_index = CHASSIS_MOTOR_RIGHT_FRONT; *is_wheel = false; return true; }
        if (identifier == 0x001U) { *motor_index = CHASSIS_MOTOR_RIGHT_REAR; *is_wheel = false; return true; }
        if (identifier == 0x202U) { *motor_index = CHASSIS_MOTOR_RIGHT_WHEEL; *is_wheel = true; return true; }
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
