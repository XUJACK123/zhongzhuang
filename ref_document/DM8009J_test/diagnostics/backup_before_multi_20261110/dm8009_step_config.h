#ifndef DM8009_STEP_CONFIG_H
#define DM8009_STEP_CONFIG_H
#ifndef DM_ID_SCAN_ONLY
#define DM_ID_SCAN_ONLY 0U
#endif
/* 0 = discover all IDs. Current motor verified on 2026-11-09: CAN_ID=2, MST_ID=0.
 * Only connect ONE DM motor on this isolated CAN2 bus during commissioning. */
#ifndef DM_FIXED_CAN_ID
#define DM_FIXED_CAN_ID 2U
#endif
#define DM_SCAN_FIRST_ID 1U
#define DM_SCAN_LAST_ID 0x7FEU
#define DM_SCAN_PERIOD_MS 10U
#define DM_FIXED_SCAN_PERIOD_MS 100U /* allow slower parameter replies on a busy bus */
#define DM_SCAN_CAPACITY 16U
#define DM_BOOT_DELAY_MS 2000U
#define DM_SETUP_LIMIT_MS 8000U
#define DM_RX_TIMEOUT_MS 100U
#define DM_STATUS_PERIOD_MS 20U
#define DM_TX_PERIOD_MS 2U
#define DM_ENABLE_LIMIT_MS 500U
/* Feedback is already the DM8009 joint/output angle; no M3508 reduction. */
#define DM_STEP_DEGREES 10.0f
#define DM_STEP_SPEED_DEG_S 2.0f
#define DM_STEP_TORQUE_LIMIT_NM 0.50f
#define DM_STEP_TORQUE_SLEW_NM_S 0.10f
#define DM_STEP_POSITION_KP 4.0f
#define DM_STEP_SPEED_KP 0.015f /* deg/s error -> Nm */
#define DM_STEP_SPEED_KI 0.030f
#define DM_STEP_POSITION_TOL_DEG 0.3f
#define DM_STEP_SETTLE_MS 150U
#define DM_STEP_TIMEOUT_MS 12000U
#define DM_STEP_ARM_MS 100U
#define DM_STEP_DEBOUNCE_MS 40U
#define DM_STEP_OVERSPEED_DEG_S 10.0f
#define DM_STEP_TRAVEL_LIMIT_DEG 12.0f
#define DM_STEP_MAX_TEMPERATURE 75U
/* DM8009 joint encoder gives output-shaft single-turn radians. PMAX describes
 * wire scaling; the mechanical wrap is 2*pi, not 2*PMAX. */
#define DM_OUTPUT_POSITION_PERIOD_RAD 6.283185307179586f
#if DM_FIXED_CAN_ID > 0x7FEU
#error "DM_FIXED_CAN_ID must be 0 (auto) or 1..0x7FE"
#endif
#endif
