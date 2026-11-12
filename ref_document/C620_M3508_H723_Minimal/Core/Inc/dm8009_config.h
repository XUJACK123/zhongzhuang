#ifndef DM8009_CONFIG_H
#define DM8009_CONFIG_H

/* One isolated DM8009 on CAN2. Change this to the motor's actual CAN_ID. */
#define DM_MOTOR_CAN_ID 2U
/* Feedback CAN ID (MST_ID). A successful RID=7 read replaces this default. */
#define DM_MASTER_CAN_ID 0U
/* Modern firmware uses CAN_ID for special commands. Older firmware may
 * require the documented mode-specific offset. Do not guess or scan enables. */
#define DM_SPECIAL_ID_OFFSET 0U
#define DM_STARTUP_DELAY_MS 2000U
#define DM_QUERY_PERIOD_MS 100U
#define DM_ENABLE_RETRY_MS 500U
#define DM_ENABLE_MAX_ATTEMPTS 3U
#define DM_FEEDBACK_TIMEOUT_MS 500U
/* MIT torque command = desired current * motor-reported KT_Value.
 * Requested/quantized current are diagnostic estimates, not measured phase current. */
#define DM_SMALL_CURRENT_A 0.05f
#define DM_CURRENT_LIMIT_A 0.10f
/* This motor returns KT_Value=0, so current conversion is unavailable.
 * Default to explicit torque units, never substitute a guessed KT. */
#define DM_USE_CURRENT_EQUIVALENT 0U
#define DM_SMALL_TORQUE_NM 0.10f
#define DM_TORQUE_LIMIT_NM 0.50f
/* Each explicitly armed test is bounded on the MCU, independent of the host. */
#define DM_TEST_DURATION_MS 250U
#define DM_TEST_MAX_DURATION_MS 500U
#define DM_TEST_MAX_POSITION_COUNTS 64
#define DM_TEST_MAX_VELOCITY_COUNTS 16
#define DM_CONTROL_PERIOD_MS 10U
#define DM_SETUP_TIMEOUT_MS 8000U

#if DM_MOTOR_CAN_ID < 1 || DM_MOTOR_CAN_ID > 0x7FE
#error "DM_MOTOR_CAN_ID must be 1..0x7FE"
#endif
#if DM_MASTER_CAN_ID > 0x7FF || (DM_MOTOR_CAN_ID + DM_SPECIAL_ID_OFFSET) > 0x7FF
#error "DM CAN IDs must fit standard CAN"
#endif
#endif
