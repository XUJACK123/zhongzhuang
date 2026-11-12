/* DM8009 CAN2 small-torque MIT test with raw-frame diagnostics.
 * Default is explicit torque units because this motor reports KT=0.
 * Reads parameters, disables, confirms MIT mode, enables with Kp=Kd=0.
 * Isolate one motor and secure the mechanism: even small torque can accelerate it.
 * Official reference: https://github.com/dmBots/motor-sdk/blob/main/Python例程/u2can/DM_CAN.py
 */
#include "main.h"
#include "dm8009_config.h"
#include "dm8009_protocol.h"

_Static_assert(FDCAN_DLC_BYTES_8 == 8U, "Review HAL DLC encoding before using DM_DlcToLength");

typedef struct {
    uint32_t sequence, tick_ms, id, id_type, frame_type, dlc, length;
    uint8_t data[8];
} CAN_RawFrame;
typedef struct {
    DM_FeedbackFields fields;
    uint32_t frame_count, last_rx_ms;
} DM_Feedback;

volatile uint32_t g_init_stage; /* 1 HAL,2 SYSCLK,3 CAN clock,4 GPIO/CAN,5 start,6 loop */
volatile uint32_t g_error; /* 1 init failure,2 filter failure,3 start failure */
volatile uint32_t g_can_tx_queued, g_can_tx_failed, g_can_rx_total;
volatile uint32_t g_can_rx_lost, g_can_rx_failed;
volatile uint32_t g_can_bus_off, g_can_error_passive, g_can_last_error;
volatile uint32_t g_can_tx_error_count, g_can_rx_error_count, g_can_tx_free;
volatile uint32_t g_can_last_hal_error, g_can_status_failed;
volatile uint32_t g_dm_enable_request = 0; /* explicitly write 1 for ONE bounded test */
volatile uint32_t g_dm_disable_request;   /* write 1 to request disable */
volatile uint32_t g_dm_enable_attempts;
volatile uint32_t g_dm_master_id = DM_MASTER_CAN_ID;
volatile uint32_t g_dm_control_mode = 0xFFFFFFFFU; /* RID10 read, unknown until response */
volatile uint32_t g_dm_param_rx_count;
volatile uint32_t g_dm_feedback_age_ms = 0xFFFFFFFFU;
volatile uint32_t g_dm_feedback_stale = 1;
volatile uint32_t g_dm_enabled; /* true only for fresh feedback with state=1 */
volatile uint32_t g_dm_enable_result; /* 0 pending,1 enabled,2 no confirmation,3 fault,4 bus-off,5 disabled */
volatile uint32_t g_dm_control_stage; /* 0 delay,1 disable/read,2 set MIT,3 enable,4 current output,5 stopped */
volatile uint32_t g_dm_control_error; /* 1 bad parameters/current,2 setup timeout,3 feedback lost,4 motor fault,5 bus-off,6 TX failure */
volatile uint32_t g_dm_parameter_mask; /* bit0 KT read, bit1 TMAX read */
volatile float g_dm_kt_Nm_per_A;
volatile float g_dm_tmax_Nm;
volatile float g_dm_current_target_A = DM_SMALL_CURRENT_A;
volatile float g_dm_torque_target_Nm = DM_SMALL_TORQUE_NM;
volatile uint32_t g_dm_command_unit = DM_USE_CURRENT_EQUIVALENT; /* 0 Nm, 1 equivalent A */
volatile uint32_t g_dm_current_estimate_valid;
volatile uint32_t g_dm_parameter_warning; /* 1 KT unavailable; use explicit torque units */
volatile float g_dm_pmax_rad;
volatile float g_dm_vmax_rad_s;
volatile uint32_t g_dm_test_duration_ms = DM_TEST_DURATION_MS;
volatile uint32_t g_dm_test_start_ms;
volatile uint32_t g_dm_test_elapsed_ms;
volatile uint32_t g_dm_test_start_position_raw;
volatile int32_t g_dm_test_position_delta_raw;
volatile int32_t g_dm_test_max_abs_position_delta;
volatile int32_t g_dm_test_max_abs_velocity_delta;
volatile uint32_t g_dm_test_stop_reason; /* 1 duration,2 position limit,3 velocity limit,4 feedback lost,5 other */
volatile float g_dm_current_command_A; /* after clamp */
volatile float g_dm_current_quantized_A; /* encoded torque / KT; not measured current */
volatile float g_dm_torque_command_Nm;
volatile float g_dm_torque_quantized_Nm;
volatile DM_Feedback g_dm_feedback;
volatile CAN_RawFrame g_can_last_rx;
volatile CAN_RawFrame g_can_rx_ring[32]; /* sequence lets debugger detect overwritten slots */
volatile uint32_t g_can_rx_write_index;
volatile uint32_t g_dm_last_tx_id;
volatile uint8_t g_dm_last_tx_data[8];

static HAL_StatusTypeDef Send(uint32_t id, const uint8_t data[8]) {
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier = id;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    g_dm_last_tx_id = id;
    for (unsigned i = 0; i < 8; ++i) g_dm_last_tx_data[i] = data[i];
    HAL_StatusTypeDef status = HAL_BUSY;
    if (!g_can_bus_off && HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) != 0U)
        status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx, (uint8_t *)data);
    if (status == HAL_OK) ++g_can_tx_queued;
    else ++g_can_tx_failed;
    return status;
}
static void Special(uint8_t tail) {
    uint8_t data[8];
    DM_SpecialPayload(tail, data);
    (void)Send(DM_MOTOR_CAN_ID + DM_SPECIAL_ID_OFFSET, data);
}
static void Query(uint8_t command, uint8_t rid) {
    uint8_t data[8];
    DM_QueryPayload(DM_MOTOR_CAN_ID, command, rid, data);
    (void)Send(0x7FFU, data);
}
static uint32_t ReadLE32(const uint8_t *b) {
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}
static float ReadFloat(const uint8_t *b) {
    union { uint32_t bits; float value; } v;
    v.bits = ReadLE32(b);
    return v.value;
}
static void SetMITMode(void) {
    uint8_t data[8];
    DM_QueryPayload(DM_MOTOR_CAN_ID, 0x55U, 10U, data);
    data[4] = 1; /* CTRL_MODE=MIT, volatile runtime write; no AA/save command */
    (void)Send(0x7FFU, data);
    g_dm_control_mode = 0xFFFFFFFFU; /* require a readback, not just a write ACK */
}
static int ParametersValid(void) {
    return DM_ControlParametersValid(DM_USE_CURRENT_EQUIVALENT, g_dm_parameter_mask,
                                      g_dm_kt_Nm_per_A, g_dm_tmax_Nm);
}
static int SendOutput(float requested) {
    uint8_t data[8];
    float actual_torque;
    float limit = DM_USE_CURRENT_EQUIVALENT ? DM_CURRENT_LIMIT_A : DM_TORQUE_LIMIT_NM;
    if (!ParametersValid() || !(requested >= -limit && requested <= limit)) return 0;
    float torque = DM_USE_CURRENT_EQUIVALENT ? requested * g_dm_kt_Nm_per_A : requested;
    if (!DM_MitTorquePayload(torque, g_dm_tmax_Nm, data, &actual_torque)) return 0;
    g_dm_current_estimate_valid = (g_dm_parameter_mask & 1U) &&
                                 g_dm_kt_Nm_per_A > 0.0f && g_dm_kt_Nm_per_A <= 100.0f;
    float quantized_current = g_dm_current_estimate_valid ? actual_torque / g_dm_kt_Nm_per_A : 0.0f;
    if (DM_USE_CURRENT_EQUIVALENT) {
        if (!(quantized_current >= -DM_CURRENT_LIMIT_A && quantized_current <= DM_CURRENT_LIMIT_A)) return 0;
    } else if (!(actual_torque >= -DM_TORQUE_LIMIT_NM && actual_torque <= DM_TORQUE_LIMIT_NM)) return 0;
    g_dm_current_command_A = g_dm_current_estimate_valid ? torque / g_dm_kt_Nm_per_A : 0.0f;
    g_dm_torque_command_Nm = torque;
    g_dm_torque_quantized_Nm = actual_torque;
    g_dm_current_quantized_A = quantized_current;
    return Send(DM_MOTOR_CAN_ID, data) == HAL_OK;
}
static void Stop(uint32_t error) {
    g_dm_control_error = error;
    g_dm_control_stage = 5;
    g_dm_enable_request = 0;
    g_dm_current_command_A = 0;
    g_dm_torque_command_Nm = 0;
    if (!g_can_bus_off) Special(0xFDU);
}
static void Receive(void) {
    if (__HAL_FDCAN_GET_FLAG(&hfdcan2, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST)) {
        ++g_can_rx_lost;
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan2, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
    }
    uint32_t pending = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0);
    while (pending--) {
        FDCAN_RxHeaderTypeDef rx = {0};
        uint8_t data[64] = {0}; /* HAL supports FD DLCs; record first eight bytes only. */
        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
            ++g_can_rx_failed;
            break;
        }
        uint32_t length = DM_DlcToLength(rx.DataLength);
        CAN_RawFrame raw = {0};
        raw.sequence = ++g_can_rx_total;
        raw.tick_ms = HAL_GetTick();
        raw.id = rx.Identifier;
        raw.id_type = rx.IdType;
        raw.frame_type = rx.RxFrameType;
        raw.dlc = rx.DataLength;
        raw.length = length;
        for (unsigned i = 0; i < 8 && i < length; ++i) raw.data[i] = data[i];
        g_can_last_rx = raw;
        g_can_rx_ring[g_can_rx_write_index] = raw;
        g_can_rx_write_index = (g_can_rx_write_index + 1U) % 32U;
        if (rx.IdType != FDCAN_STANDARD_ID || rx.RxFrameType != FDCAN_DATA_FRAME ||
            rx.FDFormat != FDCAN_CLASSIC_CAN || length != 8U) continue;
        /* Parameter replies are distinct from status frames. No parameter writes. */
        if (data[0] == (DM_MOTOR_CAN_ID & 0xFFU) &&
            data[1] == (DM_MOTOR_CAN_ID >> 8) && data[2] == 0x33U) {
            if (data[3] == 7U && ReadLE32(&data[4]) <= 0x7FFU) {
                g_dm_master_id = ReadLE32(&data[4]);
                ++g_dm_param_rx_count;
            } else if (data[3] == 10U) {
                g_dm_control_mode = ReadLE32(&data[4]);
                ++g_dm_param_rx_count;
            } else if (data[3] == 1U) {
                g_dm_kt_Nm_per_A = ReadFloat(&data[4]);
                g_dm_parameter_warning = !(g_dm_kt_Nm_per_A > 0.0f && g_dm_kt_Nm_per_A <= 100.0f);
                g_dm_parameter_mask |= 1U;
                ++g_dm_param_rx_count;
            } else if (data[3] == 23U) {
                g_dm_tmax_Nm = ReadFloat(&data[4]);
                g_dm_parameter_mask |= 2U;
                ++g_dm_param_rx_count;
            } else if (data[3] == 21U) {
                g_dm_pmax_rad = ReadFloat(&data[4]);
                ++g_dm_param_rx_count;
            } else if (data[3] == 22U) {
                g_dm_vmax_rad_s = ReadFloat(&data[4]);
                ++g_dm_param_rx_count;
            }
            continue;
        }
        DM_FeedbackFields decoded;
        if (DM_DecodeFeedback(rx.Identifier, g_dm_master_id, DM_MOTOR_CAN_ID,
                              length, data, &decoded)) {
            g_dm_feedback.fields = decoded;
            g_dm_feedback.last_rx_ms = raw.tick_ms;
            ++g_dm_feedback.frame_count;
        }
    }
}
static void ReadBusStatus(void) {
    FDCAN_ProtocolStatusTypeDef status = {0};
    FDCAN_ErrorCountersTypeDef errors = {0};
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &status) == HAL_OK) {
        g_can_bus_off = status.BusOff;
        g_can_error_passive = status.ErrorPassive;
        g_can_last_error = status.LastErrorCode;
    } else ++g_can_status_failed;
    if (HAL_FDCAN_GetErrorCounters(&hfdcan2, &errors) == HAL_OK) {
        g_can_tx_error_count = errors.TxErrorCnt;
        g_can_rx_error_count = errors.RxErrorCnt;
    } else ++g_can_status_failed;
    g_can_tx_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
    g_can_last_hal_error = HAL_FDCAN_GetError(&hfdcan2);
}
int main(void) {
    SCB_EnableICache();
    g_init_stage = 1;
    if (HAL_Init() != HAL_OK) Error_Handler();
    g_init_stage = 2;
    SystemClock_Config();
    g_init_stage = 3;
    FDCAN2_Clock_Config();
    g_init_stage = 4;
    MX_FDCAN2_Init();
    g_init_stage = 5;
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    /* Mask zero: all standard IDs. Global filter also accepts extended data. */
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        g_error = 2; Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) { g_error = 3; Error_Handler(); }
    g_init_stage = 6;
    uint32_t boot = HAL_GetTick(), last_query = boot, last_special = boot;
    uint32_t last_control = boot, setup_start = boot, query_phase = 0, mode_writes = 0;
    while (1) {
        Receive();
        ReadBusStatus();
        uint32_t now = HAL_GetTick();
        g_dm_feedback_age_ms = g_dm_feedback.frame_count ? now - g_dm_feedback.last_rx_ms : 0xFFFFFFFFU;
        g_dm_feedback_stale = g_dm_feedback_age_ms > DM_FEEDBACK_TIMEOUT_MS;
        g_dm_enabled = !g_dm_feedback_stale && g_dm_feedback.fields.state == 1U;
        if (g_dm_disable_request) {
            Stop(0);
            g_dm_disable_request = 0;
            g_dm_enable_result = 5; /* disable requested, check feedback state for confirmation */
        }
        if (!g_dm_enable_request && g_dm_control_stage > 0U && g_dm_control_stage < 4U) {
            Stop(0); /* cancel a pending startup sequence */
        }
        if (g_dm_control_stage == 5U && g_dm_enable_request) {
            /* Explicit debugger request is needed to restart after a stop/fault. */
            g_dm_control_stage = 0;
            g_dm_control_error = 0;
            g_dm_enable_attempts = 0;
            mode_writes = 0;
            g_dm_parameter_mask = 0;
            g_dm_control_mode = 0xFFFFFFFFU;
            g_dm_test_elapsed_ms = 0;
            g_dm_test_position_delta_raw = 0;
            g_dm_test_max_abs_position_delta = 0;
            g_dm_test_max_abs_velocity_delta = 0;
            g_dm_test_stop_reason = 0;
            boot = now;
        }
        if (g_dm_control_stage != 5U && now - boot >= DM_STARTUP_DELAY_MS) {
            if (g_can_bus_off) {
                g_dm_enable_result = 4; Stop(5);
            } else if (!g_dm_feedback_stale && g_dm_feedback.fields.state >= 8U) {
                g_dm_enable_result = 3; Stop(4);
            } else if (g_dm_control_stage > 0U && g_dm_control_stage < 4U && now - setup_start > DM_SETUP_TIMEOUT_MS) {
                g_dm_enable_result = 2; Stop(2);
            } else if (g_dm_control_stage == 0U && g_dm_enable_request) {
                Special(0xFDU);
                g_dm_control_stage = 1;
                setup_start = now;
                last_special = now;
            } else if (g_dm_control_stage == 1U) {
                if (g_dm_parameter_mask == 3U && !ParametersValid()) Stop(1);
                else if (!g_dm_feedback_stale && g_dm_feedback.fields.state == 0U && ParametersValid() &&
                         g_dm_control_mode != 0xFFFFFFFFU) {
                    g_dm_control_stage = 2;
                    last_special = now - 1000U;
                } else if (now - last_special >= DM_ENABLE_RETRY_MS) {
                    Special(0xFDU);
                    last_special = now;
                }
            } else if (g_dm_control_stage == 2U) {
                if (g_dm_control_mode == 1U) {
                    if (!SendOutput(0.0f)) Stop(6);
                    else {
                        Special(0xFCU);
                        g_dm_enable_attempts = 1;
                        g_dm_control_stage = 3;
                        last_special = now;
                        last_control = now;
                    }
                } else if (now - last_special >= 1000U && mode_writes < 3U) {
                    SetMITMode();
                    ++mode_writes;
                    last_special = now;
                }
            } else if (g_dm_control_stage == 3U) {
                if (g_dm_enabled) {
                    g_dm_enable_result = 1;
                    g_dm_enable_request = 0;
                    g_dm_control_stage = 4;
                    g_dm_test_start_ms = now;
                    g_dm_test_start_position_raw = g_dm_feedback.fields.position_raw;
                    g_dm_test_stop_reason = 0;
                } else if (now - last_special >= DM_ENABLE_RETRY_MS) {
                    if (g_dm_enable_attempts >= DM_ENABLE_MAX_ATTEMPTS) {
                        g_dm_enable_result = 2; Stop(2);
                    } else {
                        Special(0xFCU); ++g_dm_enable_attempts; last_special = now;
                    }
                }
                if (g_dm_control_stage == 3U && now - last_control >= DM_CONTROL_PERIOD_MS) {
                    if (!SendOutput(0.0f)) Stop(6);
                    last_control = now;
                }
            } else if (g_dm_control_stage == 4U) {
                g_dm_test_elapsed_ms = now - g_dm_test_start_ms;
                int32_t delta = (int16_t)(g_dm_feedback.fields.position_raw - g_dm_test_start_position_raw);
                int32_t abs_delta = delta < 0 ? -delta : delta;
                int32_t vel_delta = (int32_t)g_dm_feedback.fields.velocity_raw - 2048;
                int32_t abs_vel = vel_delta < 0 ? -vel_delta : vel_delta;
                g_dm_test_position_delta_raw = delta;
                if (abs_delta > g_dm_test_max_abs_position_delta) g_dm_test_max_abs_position_delta = abs_delta;
                if (abs_vel > g_dm_test_max_abs_velocity_delta) g_dm_test_max_abs_velocity_delta = abs_vel;
                uint32_t duration = g_dm_test_duration_ms;
                if (duration > DM_TEST_MAX_DURATION_MS) duration = DM_TEST_MAX_DURATION_MS;
                if (g_dm_feedback_stale || !g_dm_enabled) { g_dm_test_stop_reason = 4; Stop(3); }
                else if (g_dm_test_elapsed_ms >= duration) { g_dm_test_stop_reason = 1; Stop(0); }
                else if (abs_delta >= DM_TEST_MAX_POSITION_COUNTS) { g_dm_test_stop_reason = 2; Stop(0); }
                else if (abs_vel >= DM_TEST_MAX_VELOCITY_COUNTS) { g_dm_test_stop_reason = 3; Stop(0); }
                else if (g_dm_control_mode != 1U) Stop(1);
                else if (now - last_control >= DM_CONTROL_PERIOD_MS) {
                    float requested = DM_USE_CURRENT_EQUIVALENT ? g_dm_current_target_A : g_dm_torque_target_Nm;
                    float limit = DM_USE_CURRENT_EQUIVALENT ? DM_CURRENT_LIMIT_A : DM_TORQUE_LIMIT_NM;
                    if (!(requested >= -1000.0f && requested <= 1000.0f)) Stop(1);
                    else {
                        if (requested > limit) requested = limit;
                        if (requested < -limit) requested = -limit;
                        if (!SendOutput(requested)) Stop(6);
                    }
                    last_control = now;
                }
            }
        }
        if (!g_can_bus_off && now - boot >= DM_STARTUP_DELAY_MS && now - last_query >= DM_QUERY_PERIOD_MS) {
            static const uint8_t rids[6] = {7U, 10U, 1U, 23U, 21U, 22U};
            Query(0x33U, rids[query_phase++ % 6U]);
            Query(0xCCU, 0U);
            last_query = now;
        }
        HAL_Delay(1);
    }
}
void SysTick_Handler(void) { HAL_IncTick(); }
void Error_Handler(void) {
    if (!g_error) g_error = 1;
    __disable_irq();
    while (1) {}
}
