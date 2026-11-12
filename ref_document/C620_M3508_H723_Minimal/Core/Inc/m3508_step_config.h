#ifndef M3508_STEP_CONFIG_H
#define M3508_STEP_CONFIG_H
/* Effective feedback-turns per visible shaft turn, NOT automatically the
 * motor's nominal gearbox ratio. The user's shaft moved ~180deg for the
 * previous 10deg command, so remove the extra ~19.2x compensation.
 * Calibrate against a shaft mark; do not reinsert nominal gearing blindly. */
#define M3508_FEEDBACK_RATIO 1.0f
#define M3508_GEAR_RATIO M3508_FEEDBACK_RATIO
#define STEP_DEGREES 10.0f
#define STEP_SPEED_DEG_S 3.0f
#define STEP_ACCEL_DEG_S2 2.0f
#define STEP_SPEED_FILTER_SECONDS 0.08f
#define STEP_CURRENT_LIMIT 1000 /* C620 units, approximately 1.22 A */
#define STEP_SPEED_KP 80.0f     /* measured shaft degree/s error -> current units */
#define STEP_SPEED_KI 120.0f
#define STEP_POSITION_KP 2.0f
#define STEP_POSITION_TOL_DEG 0.2f
#define STEP_SETTLE_MS 150U
#define STEP_TIMEOUT_MS 12000U
#define STEP_SWITCH_DEBOUNCE_MS 40U
#define STEP_ARM_DOWN_MS 100U
#define STEP_REMOTE_TIMEOUT_MS 100U
#define STEP_MOTOR_TIMEOUT_MS 100U
#define STEP_MAX_TEMPERATURE 75U
#define STEP_OVERSPEED_DEG_S 12.0f
#endif
