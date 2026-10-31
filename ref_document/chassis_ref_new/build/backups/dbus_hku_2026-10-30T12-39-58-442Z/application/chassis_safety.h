#ifndef CHASSIS_SAFETY_H
#define CHASSIS_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_COMMAND_TIMEOUT_MS 30U
#define CHASSIS_DBUS_TIMEOUT_MS 100U
#define CHASSIS_MOTOR_TIMEOUT_MS 20U
#define CHASSIS_IMU_TIMEOUT_MS 10U

typedef enum
{
    CHASSIS_STATE_STOPPED = 0,
    CHASSIS_STATE_STANDBY,
    CHASSIS_STATE_ARMED,
    CHASSIS_STATE_LOCKED
} chassis_control_state_t;

enum
{
    CHASSIS_FAULT_NONE = 0U,
    CHASSIS_FAULT_DBUS_TIMEOUT = 1U << 0,
    CHASSIS_FAULT_MOTOR_TIMEOUT = 1U << 1,
    CHASSIS_FAULT_IMU_TIMEOUT = 1U << 2,
    CHASSIS_FAULT_COMMAND_TIMEOUT = 1U << 3,
    CHASSIS_FAULT_COMMAND_INVALID = 1U << 4,
    CHASSIS_FAULT_CAN_TX = 1U << 5,
    CHASSIS_FAULT_JOINT_STATUS = 1U << 6,
    CHASSIS_FAULT_RECOVERY_REQUIRED = 1U << 7
};

typedef struct
{
    uint32_t now_ms;
    uint32_t dbus_age_ms;
    uint32_t motor_max_age_ms;
    uint32_t imu_age_ms;
    uint32_t command_age_ms;
    uint32_t command_sequence;
    uint8_t switch_left;
    bool dbus_valid;
    bool motors_valid;
    bool imu_valid;
    bool command_valid;
    bool command_invalid;
    bool can_tx_ok;
    bool joints_enabled;
} chassis_safety_input_t;

typedef struct
{
    chassis_control_state_t state;
    uint32_t fault_flags;
    uint32_t up_entry_command_sequence;
    bool down_seen;
    bool up_seen;
} chassis_safety_t;

void chassis_safety_init(chassis_safety_t *safety);
bool chassis_safety_step(chassis_safety_t *safety, const chassis_safety_input_t *input);

#endif
