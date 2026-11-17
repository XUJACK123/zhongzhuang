#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#include "algorithm/model/model.h"

/* Legacy raw exports do not carry a profile header. */
#ifndef RL_PROFILE_MODEL_VERSION
#define RL_PROFILE_MODEL_VERSION 100U
#define RL_PROFILE_HAS_GETUP 0
#define RL_PROFILE_ALLOW_REVERSE 0
#define RL_PROFILE_WHEEL_RADIUS_M 0.055f
#define RL_PROFILE_HEIGHT_COMMAND_M 0.10f
#define RL_PROFILE_FORWARD_CAP_M_S 0.15f
#define RL_PROFILE_YAW_CAP_RAD_S 0.30f
#define RL_PROFILE_GETUP_TIMEOUT_MS 5000U
#define RL_PROFILE_HANDOFF_REANCHOR 0
#define RL_PROFILE_ENTRY_SPEED_M_S 0.08f
#define RL_PROFILE_BLEND_SPEED_M_S 0.08f
#endif

/* Physical wiring confirmed 2026-11-13. Policy order is RIGHT first.
 * ID2/ID3 may coexist: DM feedback 0x30x, DJI feedback 0x20x.
 * ROBOT_ENABLE_OUTPUT is the single output master switch; the flags below it
 * record what has actually been verified and no longer gate the motors. */
/* User-authorized supported startup trial. */
#define ROBOT_ENABLE_OUTPUT 1
#define ROBOT_STAND_ENTRY_ONLY 1 /* operator places and supports the robot for entry */
#define ROBOT_STAND_ENTRY_JOINT_TOL_RAD 0.7853981634f /* 45 deg, supported entry trial */
#define ROBOT_STAND_ENTRY_TILT_RAD 0.2617993878f /* 15 deg, supported entry trial */
#define ROBOT_STAND_ENTRY_DWELL_MS 500U
#define ROBOT_MOTOR_CALIBRATED 1
#define ROBOT_IMU_FRAME_VERIFIED 1
#define ROBOT_POLICY_CONTRACT_VERIFIED 0
#define ROBOT_ODOMETRY_VERIFIED 0

#define ROBOT_CONTROL_MS 2U
#define ROBOT_POLICY_MS 10U /* actor 10ms, estimator 5ms, external PD 2ms */
#define ROBOT_CONTROL_VERSION 0x2026111BU /* supported standing-entry revision; not a date */
#define ROBOT_MODEL_VERSION RL_PROFILE_MODEL_VERSION
#define ROBOT_FEEDBACK_TIMEOUT_MS 30U
#define ROBOT_REMOTE_TIMEOUT_MS 100U
#define ROBOT_IMU_TIMEOUT_MS 5U
#define ROBOT_ARM_HOLD_MS 300U
#define ROBOT_ENABLE_TIMEOUT_MS 500U
#define ROBOT_MAX_TILT_RAD 0.70f
#define ROBOT_MAX_TEMPERATURE_C 75U
#define ROBOT_LEG_TORQUE_CAP_NM 20.0f /* training nominal; the gas spring needs the full range */
#define ROBOT_WHEEL_TORQUE_CAP_NM 2.0f
#define ROBOT_DM_KT_NM_A 1.261575f /* DM8009J_test actual group-mode conversion */
#define ROBOT_DM_CURRENT_FULL_SCALE_A 41.044776f
#define ROBOT_WHEEL_GEAR_RATIO 15.88f /* actual gearbox confirmed 2026-11-13 */
#define ROBOT_WHEEL_KT_NM_A (0.3f * ROBOT_WHEEL_GEAR_RATIO / (3591.0f / 187.0f))
#define ROBOT_REMOTE_MAX_FORWARD RL_PROFILE_FORWARD_CAP_M_S
#define ROBOT_REMOTE_MAX_YAW RL_PROFILE_YAW_CAP_RAD_S
#define ROBOT_WHEEL_RADIUS_M RL_PROFILE_WHEEL_RADIUS_M
#define ROBOT_REFERENCE_MAX_ERROR_M 0.40f
#define ROBOT_ODOMETRY_RESIDUAL_M_S 0.30f
#define ROBOT_WHEEL_PREDICTION_ACCEL_RAD_S2 100.0f /* wheel-axis uncertainty bound, pending measurement */
#define ROBOT_IMU_ADAPTIVE 1
#define ROBOT_HEIGHT_COMMAND_M RL_PROFILE_HEIGHT_COMMAND_M /* bottom clearance */

/* Raw encoder -> canonical output arm at a surveyed reference pose.
 * canonical = arm_at_reference + sign * unwrap(raw - encoder_at_reference).
 * Surveyed with external support on 2026-11-13; repeatability remains to measure.
 * Stored reference selects initial alpha branch in (-2pi,0).
 * Wheels are rotor encoder angles / 15.88, no DM gearbox division. */
#define ROBOT_ENCODER_SIGNS {-1, 1, -1, 1, 1, 1}
#define ROBOT_ENCODER_REFERENCE_RAD {2.7511945431f, 2.4643401357f, 0.1742633086f, 1.0385049934f, 2.8731460157f, 0.1278478320f}
#define ROBOT_ARM_REFERENCE_RAD {3.1500981078f, 1.5822922061f, 0.0000000000f, 3.1500981078f, 1.5822922061f, 0.0000000000f}

/* Chip -> policy body, x forward, y left, z up. Direction checked with
 * level/nose-up/left-roll and one full left turn; dynamic drift pending. */
#define ROBOT_IMU_ROTATION {{-1,0,0}, {0,-1,0}, {0,0,1}}
#endif
