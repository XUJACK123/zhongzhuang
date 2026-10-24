#ifndef CHASSIS_PROTOCOL_H
#define CHASSIS_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CHASSIS_MOTOR_COUNT 6U
#define CHASSIS_JOINT_TORQUE_LIMIT_NM 54.0f
#define CHASSIS_WHEEL_TORQUE_LIMIT_NM 3.3f
#define CHASSIS_WHEEL_CURRENT_LIMIT 16384
#define CHASSIS_WHEEL_GEAR_RATIO 13.94f

typedef enum
{
    CHASSIS_MOTOR_LEFT_FRONT = 0,
    CHASSIS_MOTOR_LEFT_REAR,
    CHASSIS_MOTOR_RIGHT_FRONT,
    CHASSIS_MOTOR_RIGHT_REAR,
    CHASSIS_MOTOR_LEFT_WHEEL,
    CHASSIS_MOTOR_RIGHT_WHEEL
} chassis_motor_index_t;

typedef struct
{
    int16_t channel[5];
    uint8_t switch_left;
    uint8_t switch_right;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint16_t keyboard;
} chassis_dbus_frame_t;

bool chassis_dbus_decode(const uint8_t frame[18], chassis_dbus_frame_t *decoded);
bool chassis_torque_vector_valid(const float torque_nm[CHASSIS_MOTOR_COUNT]);
float chassis_limit_torque(size_t motor_index, float torque_nm);
int16_t chassis_wheel_torque_to_current(float torque_nm);
float chassis_wheel_current_to_torque(int16_t current);
float chassis_wheel_rpm_to_rad_s(int16_t rpm);
int32_t chassis_update_encoder_turns(uint16_t current, uint16_t previous, int32_t turns);
bool chassis_route_can_feedback(uint8_t bus_number, uint32_t identifier,
                                size_t *motor_index, bool *is_wheel);
bool chassis_validate_can_feedback(uint8_t bus_number, uint32_t identifier,
                                   bool standard_id, bool data_frame, uint8_t length,
                                   size_t *motor_index, bool *is_wheel);

#endif
