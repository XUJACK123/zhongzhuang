#ifndef CHASSIS_PROTOCOL_H
#define CHASSIS_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "chassis_motor_config.h"

#define CHASSIS_MOTOR_COUNT 6U
#define CHASSIS_JOINT_TORQUE_LIMIT_NM 54.0f
#define CHASSIS_WHEEL_TORQUE_LIMIT_NM 3.3f
#define CHASSIS_WHEEL_CURRENT_LIMIT 16384
#define CHASSIS_WHEEL_MOTOR_GEAR_RATIO (3591.0f / 187.0f) /* C620 内置 ~19.2:1 */
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
    float position_rad;
    float speed_rad_s;
    float torque_nm;
    uint32_t update_time_ms;
    uint8_t status;
    bool valid;
} motor_feedback_t;

/* 只汇总当前配置要求的电机；非有限反馈不能作为有效控制输入。 */
bool chassis_feedback_health(const motor_feedback_t motor[CHASSIS_MOTOR_COUNT],
                             uint32_t now_ms, uint32_t *max_age_ms, bool *joints_enabled);

typedef struct
{
    int16_t channel[5];
    uint8_t switch_left;  /* HKU rc.s[1], frame[5] bits 6..7 */
    uint8_t switch_right; /* HKU rc.s[0], frame[5] bits 4..5；现有 S1 灯色/停止拨杆 */
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint16_t keyboard;
} chassis_dbus_frame_t;

typedef struct
{
    uint8_t bus;
    uint16_t id;
    uint8_t data[8];
} chassis_can_frame_t;

/* HAL 无关的真实发送编码，主机测试与固件使用同一实现。 */
size_t chassis_encode_torques(const float requested[CHASSIS_MOTOR_COUNT],
                             float applied[CHASSIS_MOTOR_COUNT],
                             chassis_can_frame_t frames[CHASSIS_MOTOR_COUNT]);
size_t chassis_encode_joint_command(uint8_t command,
                                   chassis_can_frame_t frames[CHASSIS_MOTOR_COUNT]);

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
