/* Transport-independent discovery, setup and remote-controlled DM8009 steps.
 * Scan uses ONLY RID7 reads on 0x7FF; never enables candidate IDs. */
#include "dm8009_app.h"
#include <string.h>
#define RAD_TO_DEG 57.295779513f
#define PARAM_ALL 31U /* ESC_ID, CTRL_MODE, PMAX, VMAX, TMAX */
static uint32_t LE32(const uint8_t *b) {
    return (uint32_t)b[0] | (uint32_t)b[1]<<8 | (uint32_t)b[2]<<16 | (uint32_t)b[3]<<24;
}
static float Float32(const uint8_t *b) {
    uint32_t bits = LE32(b); float value; memcpy(&value, &bits, sizeof(value)); return value;
}
static uint32_t FirstID(void) { return DM_FIXED_CAN_ID ? DM_FIXED_CAN_ID : DM_SCAN_FIRST_ID; }
static uint32_t LastID(void) { return DM_FIXED_CAN_ID ? DM_FIXED_CAN_ID : DM_SCAN_LAST_ID; }
static uint32_t ScanPeriod(void) { return DM_FIXED_CAN_ID ? DM_FIXED_SCAN_PERIOD_MS : DM_SCAN_PERIOD_MS; }
static int RangesValid(const DM_App *a) {
    return a->pmax_rad > 0 && a->pmax_rad <= 1000 &&
           a->vmax_rad_s > 0 && a->vmax_rad_s <= 10000 &&
           a->tmax_Nm > 0 && a->tmax_Nm <= 1000;
}
static void Fail(DM_App *a, uint32_t error, const DM_IO *io) {
    if (a->stage != DM_FAULT) {
        a->stage = DM_FAULT; a->error = error;
        io->cancel_pending(io->context);
    }
    a->drive = a->step.active = a->step.armed = 0;
    a->step.torque_Nm = a->step.integral = 0;
}
static int Emit(DM_App *a, const DM_IO *io, uint16_t id, const uint8_t data[8]) {
    if (io->send(io->context, id, data)) return 1;
    Fail(a, DM_ERR_TX, io); return 0;
}
static int Special(DM_App *a, const DM_IO *io, uint8_t tail) {
    uint8_t data[8]; DM_SpecialPayload(tail, data);
    return Emit(a, io, a->motor_id, data);
}
static int Query(DM_App *a, const DM_IO *io, uint16_t id, uint8_t command, uint8_t rid) {
    uint8_t data[8]; DM_QueryPayload(id, command, rid, data);
    return Emit(a, io, 0x7FFU, data);
}
static int Torque(DM_App *a, const DM_IO *io, float requested) {
    uint8_t data[8]; float actual;
    if (!DM_MitTorquePayload(requested, a->tmax_Nm, data, &actual)) {
        Fail(a, DM_ERR_PARAMETER, io); return 0;
    }
    /* Round toward a representable command INSIDE the configured torque bound. */
    uint32_t raw = ((uint32_t)(data[6] & 15U) << 8) | data[7];
    if (actual > DM_STEP_TORQUE_LIMIT_NM && raw) --raw;
    else if (actual < -DM_STEP_TORQUE_LIMIT_NM && raw < 4095U) ++raw;
    actual = raw * (2.0f * a->tmax_Nm / 4095.0f) - a->tmax_Nm;
    if (!(actual >= -DM_STEP_TORQUE_LIMIT_NM && actual <= DM_STEP_TORQUE_LIMIT_NM)) {
        Fail(a, DM_ERR_PARAMETER, io); return 0;
    }
    data[6] = (uint8_t)(raw >> 8); data[7] = (uint8_t)raw;
    a->torque_quantized_Nm = actual;
    return Emit(a, io, a->motor_id, data);
}
void DM_AppInit(DM_App *a, uint32_t now) {
    memset(a, 0, sizeof(*a));
    a->stage_since = now; a->scan_id = FirstID(); a->mode = 0xFFFFFFFFU;
}
static void ScanReply(DM_App *a, uint32_t now, uint16_t can_id, const uint8_t *data) {
    if (!a->scan_waiting || now - a->scan_sent_ms > ScanPeriod() ||
        data[0] != (uint8_t)a->scan_id || data[1] != (a->scan_id >> 8) ||
        data[2] != 0x33U || data[3] != 7U || LE32(data+4) != can_id) return;
    for (uint32_t i = 0; i < a->scan_count; ++i) {
        DM_ScanEntry *entry = &a->scan[i];
        if (entry->motor_id != a->scan_id) continue;
        if (entry->master_id != can_id) { a->scan_overflow = 1; return; }
        entry->pass_mask |= 1U << a->scan_pass; ++entry->replies;
        return;
    }
    if (a->scan_count == DM_SCAN_CAPACITY) { a->scan_overflow = 1; return; }
    a->scan[a->scan_count++] = (DM_ScanEntry){(uint16_t)a->scan_id, can_id, 1U << a->scan_pass, 1U};
}
void DM_AppReceive(DM_App *a, uint32_t now, uint16_t can_id, const uint8_t data[8]) {
    if (a->stage == DM_SCAN) { ScanReply(a, now, can_id, data); return; }
    if (!a->motor_id || can_id != a->master_id) return;
    if (data[0] == (uint8_t)a->motor_id && data[1] == (a->motor_id >> 8) &&
        (data[2] == 0x33U || data[2] == 0x55U)) {
        /* Only a matching, outstanding READ can satisfy setup; write ACKs cannot. */
        if (a->stage != DM_SETUP || data[2] != 0x33U || !a->parameter_pending ||
            data[3] != a->pending_rid || now - a->parameter_sent_ms > 100U) return;
        uint32_t value = LE32(data+4);
        switch (data[3]) {
        case 8: if (value == a->motor_id) a->parameter_mask |= 1U; break;
        case 10: a->mode = value; a->parameter_mask |= 2U; break;
        case 21: a->pmax_rad = Float32(data+4); a->parameter_mask |= 4U; break;
        case 22: a->vmax_rad_s = Float32(data+4); a->parameter_mask |= 8U; break;
        case 23: a->tmax_Nm = Float32(data+4); a->parameter_mask |= 16U; break;
        case 13: a->hw_version = value; a->diagnostic_mask |= 1U; break;
        case 14: a->sw_version = value; a->diagnostic_mask |= 2U; break;
        case 9: a->comm_timeout = value; a->diagnostic_mask |= 4U; break;
        case 20: a->internal_gear_ratio = Float32(data+4); a->diagnostic_mask |= 8U; break;
        default: return;
        }
        ++a->parameter_replies; a->parameter_pending = 0;
        return;
    }
    DM_FeedbackFields fields;
    if (!DM_DecodeFeedback(can_id, a->master_id, a->motor_id, 8U, data, &fields)) return;
    if (fields.state != 0 && fields.state != 1 && !(fields.state >= 8 && fields.state <= 14)) return;
    if (RangesValid(a) && (a->parameter_mask & 28U) == 28U) {
        /* PMAX maps the wire value; the physical single-turn encoder wraps at pi. */
        if (a->position_initialized && now - a->feedback_ms <= DM_RX_TIMEOUT_MS) {
            a->position_deg += DM_OutputPositionDeltaDeg((uint16_t)a->previous_raw,
                fields.position_raw, a->pmax_rad, DM_OUTPUT_POSITION_PERIOD_RAD);
        }
        a->previous_raw = fields.position_raw; a->position_initialized = 1;
        a->speed_deg_s = (fields.velocity_raw * (2.0f * a->vmax_rad_s / 4095.0f) - a->vmax_rad_s) * RAD_TO_DEG;
    }
    a->feedback = fields; a->feedback_ms = now; ++a->feedback_count;
}
static void ScanTick(DM_App *a, uint32_t now, const DM_IO *io) {
    if (a->scan_waiting) {
        if (now - a->scan_sent_ms < ScanPeriod()) return;
        if (++a->scan_id > LastID()) {
            a->scan_id = FirstID();
            if (++a->scan_pass == 2U) {
                if (a->scan_count == 1 && !a->scan_overflow && a->scan[0].pass_mask == 3U) {
                    a->scan_result = DM_SCAN_UNIQUE;
                    a->motor_id = a->scan[0].motor_id; a->master_id = a->scan[0].master_id;
                    a->stage = DM_ID_SCAN_ONLY ? DM_IDENTIFIED : DM_SETUP; a->stage_since = now;
                    a->last_query_ms = now - DM_STATUS_PERIOD_MS;
                    a->last_service_ms = now - DM_STATUS_PERIOD_MS;
                    return;
                }
                /* A motor may boot later than the MCU after a power cycle.
                 * Retry ONLY reads of the known ID before latching no-response. */
                if (DM_FIXED_CAN_ID && !a->scan_overflow && a->scan_count <= 1U &&
                    now-a->stage_since < DM_SETUP_LIMIT_MS) {
                    a->scan_pass=0; a->scan_waiting=0;
                    return;
                }
                a->scan_result = a->scan_overflow || a->scan_count > 1 ? DM_SCAN_MULTIPLE :
                                 (a->scan_count ? DM_SCAN_UNCONFIRMED : DM_SCAN_NONE);
                Fail(a, DM_ERR_SCAN, io); return;
            }
        }
    }
    if (Query(a, io, (uint16_t)a->scan_id, 0x33U, 7U)) {
        a->scan_sent_ms = now; a->scan_waiting = 1;
    }
}
static void SetupTick(DM_App *a, uint32_t now, const DM_IO *io) {
    static const uint8_t rids[] = {8,10,21,22,23,13,14,20,9};
    if (now - a->stage_since > DM_SETUP_LIMIT_MS) { Fail(a, DM_ERR_SETUP, io); return; }
    if (a->feedback_fresh && a->feedback.state >= 8U) { Fail(a, DM_ERR_MOTOR, io); return; }
    if (now - a->last_service_ms < DM_STATUS_PERIOD_MS) return;
    a->last_service_ms = now;
    if (!Special(a, io, 0xFDU)) return;
    if (a->parameter_mask == PARAM_ALL) {
        if (!RangesValid(a)) { Fail(a, DM_ERR_PARAMETER, io); return; }
        /* A disabled-state ACK is not a startup prerequisite. Some firmware
         * keeps reporting state=1 after FD; retain live feedback/mode checks. */
        if (a->feedback_fresh && a->feedback.state <= 1U && a->position_initialized) {
            if (a->mode == 1U) { a->stage = DM_READY; return; }
            /* Disable has been requested above. Volatile mode write + readback;
             * do not wait for a disabled-state ACK and do not save to Flash. */
            uint8_t data[8]; DM_QueryPayload(a->motor_id, 0x55U, 10U, data); data[4] = 1;
            if (!Emit(a, io, 0x7FFU, data)) return;
            a->parameter_mask &= ~2U; a->mode = 0xFFFFFFFFU;
            a->setup_index = 1; a->parameter_pending = 0;
        }
    }
    if (!a->parameter_pending || now - a->parameter_sent_ms >= 100U) {
        uint8_t rid = rids[a->setup_index++ % (sizeof(rids)/sizeof(rids[0]))];
        if (!Query(a, io, a->motor_id, 0x33U, rid)) return;
        a->pending_rid = rid; a->parameter_sent_ms = now; a->parameter_pending = 1;
    }
}
void DM_AppTick(DM_App *a, uint32_t now, const DBUS_State *remote, int bus_ok, const DM_IO *io) {
    a->feedback_fresh = a->feedback_count && now - a->feedback_ms <= DM_RX_TIMEOUT_MS;
    a->remote_fresh = remote->valid && now - remote->last_rx_ms <= DM_RX_TIMEOUT_MS;
    if (!bus_ok) Fail(a, DM_ERR_BUS, io);
    if (a->stage == DM_BOOT) {
        if (now - a->stage_since < DM_BOOT_DELAY_MS) return;
        a->stage = DM_SCAN; a->stage_since=now;
    }
    if (a->stage == DM_SCAN) { ScanTick(a, now, io); return; }
    /* Dedicated diagnostic image has no path to any write or special command. */
    if (DM_ID_SCAN_ONLY) return;
    if (a->stage == DM_FAULT) {
        if (a->motor_id && bus_ok && now - a->last_service_ms >= DM_STATUS_PERIOD_MS) {
            a->last_service_ms = now; (void)Special(a, io, 0xFDU);
            (void)Query(a, io, a->motor_id, 0xCCU, 0U);
        }
        /* A transient RX timeout must not require reflashing to recover.
         * Recheck parameters ONLY after fresh normal feedback and deliberate
         * double-down for 100 ms. Never recover directly into an active step. */
        int recover = a->error == DM_ERR_FEEDBACK && bus_ok && a->feedback_fresh &&
                      a->feedback.state <= 1U && a->remote_fresh &&
                      remote->right_switch == DBUS_DOWN && remote->left_switch == DBUS_DOWN;
        if (recover) {
            if (!a->recovery_timing) { a->recovery_timing=1; a->recovery_since=now; }
            if (now-a->recovery_since >= DM_STEP_ARM_MS) {
                a->stage=DM_SETUP; a->error=0; a->stage_since=now;
                a->parameter_mask=0; a->parameter_pending=0; a->setup_index=0;
                a->position_initialized=0; a->recovery_timing=0; ++a->recovery_count;
                memset(&a->step,0,sizeof(a->step));
            }
        } else a->recovery_timing=0;
        return;
    }
    /* Status queries keep disabled feedback fresh too; don't rely on auto-upload. */
    if (now - a->last_query_ms >= DM_STATUS_PERIOD_MS) {
        a->last_query_ms = now;
        if (!Query(a, io, a->motor_id, 0xCCU, 0U)) return;
    }
    if (a->stage == DM_SETUP) { SetupTick(a, now, io); return; }
    if (!a->feedback_fresh) { Fail(a, DM_ERR_FEEDBACK, io); (void)Special(a, io, 0xFDU); return; }
    if (a->feedback.state >= 8U) { Fail(a, DM_ERR_MOTOR, io); (void)Special(a, io, 0xFDU); return; }
    DM_StepInput input = {
        .now_ms = now, .remote_ok = a->remote_fresh, .motor_ok = 1, .bus_ok = 1,
        .drive_enabled = a->drive == 2U, .right_switch = remote->right_switch,
        .left_switch = remote->left_switch,
        .temperature = a->feedback.mos_temperature > a->feedback.rotor_temperature ?
                       a->feedback.mos_temperature : a->feedback.rotor_temperature,
        .position_deg = a->position_deg, .speed_deg_s = a->speed_deg_s
    };
    DM_StepUpdate(&a->step, &input);
    if (!a->step.active) {
        uint32_t was_driving = a->drive;
        if (was_driving) { io->cancel_pending(io->context); a->drive = 0; }
        a->torque_quantized_Nm = 0;
        if (was_driving || now - a->last_service_ms >= DM_STATUS_PERIOD_MS || a->feedback.state == 1U) {
            a->last_service_ms = now; (void)Special(a, io, 0xFDU);
        }
        return;
    }
    if (!a->drive) {
        /* Establish a neutral MIT target BEFORE enabling; never reuse an old target. */
        io->cancel_pending(io->context);
        if (!Torque(a, io, 0)) return;
        if (!Special(a, io, 0xFCU)) return;
        a->drive = 1; a->enable_since = now; a->enable_feedback_count = a->feedback_count;
        return;
    }
    if (a->drive == 1U) {
        if (now - a->enable_since > DM_ENABLE_LIMIT_MS) { Fail(a, DM_ERR_ENABLE, io); (void)Special(a, io, 0xFDU); return; }
        if (a->feedback.state == 1U && a->feedback_count > a->enable_feedback_count) a->drive = 2;
        (void)Torque(a, io, 0); return;
    }
    if (a->feedback.state != 1U) { Fail(a, DM_ERR_MOTOR, io); (void)Special(a, io, 0xFDU); return; }
    (void)Torque(a, io, a->step.torque_Nm);
}
