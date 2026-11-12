#ifndef M3508_STEP_CONFIG_H
#define M3508_STEP_CONFIG_H
/* Output shaft angle, not rotor angle. Adjust if an external transmission is used. */
#define M3508_GEAR_RATIO (3591.0f / 187.0f)
#define STEP_DEGREES 20.0f
#define STEP_SPEED_DEG_S 50.0f
#define STEP_CURRENT_LIMIT 1500 /* C620 units, approximately 1.83 A */
#define STEP_SPEED_KP 10.0f     /* rotor rpm error -> current units */
#define STEP_SPEED_KI 35.0f
#define STEP_POSITION_KP 4.0f   /* output degree error -> output degree/s */
#define STEP_POSITION_TOL_DEG 0.3f
#define STEP_SETTLE_MS 150U
#define STEP_TIMEOUT_MS 5000U
#define STEP_SWITCH_DEBOUNCE_MS 40U
#define STEP_ARM_DOWN_MS 100U
#define STEP_REMOTE_TIMEOUT_MS 100U
#define STEP_MOTOR_TIMEOUT_MS 100U
#define STEP_MAX_TEMPERATURE 75U
#define STEP_OVERSPEED_DEG_S 60.0f
#endif
