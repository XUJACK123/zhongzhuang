#ifndef DM8009_APP_H
#define DM8009_APP_H
#include "dm8009_protocol.h"
#include "dm8009_step.h"
#include "dbus_protocol.h"
enum { DM_BOOT, DM_SCAN, DM_SETUP, DM_READY, DM_FAULT, DM_IDENTIFIED };
enum { DM_SCAN_PENDING, DM_SCAN_UNIQUE, DM_SCAN_NONE, DM_SCAN_MULTIPLE, DM_SCAN_UNCONFIRMED };
enum { DM_ERR_NONE, DM_ERR_SCAN, DM_ERR_SETUP, DM_ERR_BUS, DM_ERR_FEEDBACK,
       DM_ERR_MOTOR, DM_ERR_TX, DM_ERR_ENABLE, DM_ERR_PARAMETER };
typedef struct { uint16_t motor_id, master_id; uint32_t pass_mask, replies; } DM_ScanEntry;
typedef struct {
    void *context;
    int (*send)(void *context, uint16_t id, const uint8_t data[8]);
    void (*cancel_pending)(void *context);
} DM_IO;
typedef struct {
    uint32_t stage, error, stage_since;
    uint32_t scan_result, scan_id, scan_pass, scan_count, scan_overflow;
    uint32_t scan_waiting, scan_sent_ms;
    DM_ScanEntry scan[DM_SCAN_CAPACITY];
    uint16_t motor_id, master_id;
    uint32_t parameter_mask, parameter_replies, mode, setup_index;
    uint32_t parameter_pending, pending_rid, parameter_sent_ms;
    float pmax_rad, vmax_rad_s, tmax_Nm;
    DM_FeedbackFields feedback;
    uint32_t feedback_count, feedback_ms, feedback_fresh;
    uint32_t position_initialized, previous_raw;
    float position_deg, speed_deg_s;
    uint32_t last_query_ms, last_service_ms, drive, enable_since, enable_feedback_count;
    uint32_t remote_fresh;
    float torque_quantized_Nm;
    DM_StepController step;
    /* Read-only firmware diagnostics; never used to guess protocol offsets. */
    uint32_t hw_version, sw_version, comm_timeout;
    float internal_gear_ratio;
    uint32_t diagnostic_mask;
    uint32_t recovery_timing, recovery_since, recovery_count;
} DM_App;
void DM_AppInit(DM_App *a, uint32_t now);
/* Only pass validated standard, classic, eight-byte CAN data frames here. */
void DM_AppReceive(DM_App *a, uint32_t now, uint16_t can_id, const uint8_t data[8]);
void DM_AppTick(DM_App *a, uint32_t now, const DBUS_State *remote, int bus_ok, const DM_IO *io);
#endif
