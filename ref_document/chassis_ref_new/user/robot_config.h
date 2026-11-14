#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Physical wiring confirmed 2026-11-13. Policy order is RIGHT first.
 * ID2/ID3 may coexist: DM feedback 0x30x, DJI feedback 0x20x.
 * ROBOT_ENABLE_OUTPUT is the single output master switch; the flags below it
 * record what has actually been verified and no longer gate the motors. */
#define ROBOT_ENABLE_OUTPUT 1
#define ROBOT_MOTOR_CALIBRATED 1
#define ROBOT_IMU_FRAME_VERIFIED 1
#define ROBOT_POLICY_CONTRACT_VERIFIED 0
#define ROBOT_ODOMETRY_VERIFIED 0

#define ROBOT_CONTROL_MS 2U
#define ROBOT_POLICY_MS 10U /* sim.dt .005 * decimation 2 */
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
#define ROBOT_REMOTE_MAX_FORWARD 0.30f /* training has stand and forward .1.. .3 */
#define ROBOT_REMOTE_MAX_YAW 2.50f
#define ROBOT_HEIGHT_COMMAND_M 0.10f /* bottom clearance, NOT hip height */

/* Raw encoder -> canonical output arm at a surveyed reference pose.
 * canonical = arm_at_reference + sign * unwrap(raw - encoder_at_reference).
 * These are PLACEHOLDERS, never inferred from the position at power-on.
 * Fill by measuring the marked pose; choose initial alpha branch in (-2pi,0).
 * Wheels are rotor encoder angles / 15.88, no DM gearbox division. */
#define ROBOT_ENCODER_SIGNS {-1, 1, -1, 1, 1, 1}
#define ROBOT_ENCODER_REFERENCE_RAD {2.7511945431f, 2.4643401357f, 0.1742633086f, 1.0385049934f, 2.8731460157f, 0.1278478320f}
#define ROBOT_ARM_REFERENCE_RAD {3.1500981078f, 1.5822922061f, 0.0000000000f, 3.1500981078f, 1.5822922061f, 0.0000000000f}

/* Chip -> policy body, x forward, y left, z up. Provisional physical mapping
 * from level and user-confirmed nose-up captures 2026-11-13; roll/yaw pending.
 * Keep ROBOT_IMU_FRAME_VERIFIED=0 until the independent motions are checked. */
#define ROBOT_IMU_ROTATION {{-1,0,0}, {0,-1,0}, {0,0,1}}
#endif
