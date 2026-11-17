#ifndef TRIAL_TRACE_H
#define TRIAL_TRACE_H
#include <stdbool.h>
#include "telemetry.h"
#define TRIAL_TRACE_CAPACITY 512U
typedef struct {
    uint32_t begin,tick,mode,inhibit,fault,entry_block,stable_ms,inference_us,loop_us;
    uint32_t motor_tick[6];
    uint16_t raw[6];
    float q[6],qd[6],motor_velocity[6],torque[6],feedback[6],action[6];
    float omega[3],gravity[3],confidence,residual;
    uint32_t end;
} TrialFrame;
typedef struct {
    uint32_t schema,frame_bytes,capacity,published,overwritten,frozen;
    uint32_t entry_counts[10],last_entry_block;
    TrialFrame frame[TRIAL_TRACE_CAPACITY];
} TrialTrace;
extern volatile TrialTrace g_trial_trace;
#define TRACE_TRANSFER_BYTES 8192U
typedef struct {
    uint32_t request,ack,command,offset,length,status;
    uint32_t header[17];
    uint8_t data[TRACE_TRANSFER_BYTES];
} TraceTransfer;
extern volatile TraceTransfer g_trace_transfer;
/* STOP-only diagnostic commands: 1 freeze, 2 read frozen bytes, 3 clear trace. */
void trial_trace_transfer(bool stopped);
void trial_trace_reset(void);
void trial_trace_boot(uint32_t reset_flags);
void trial_trace_entry(uint32_t reason);
void trial_trace_publish(const TelemetryFrame *source, const float gravity[3],
    uint32_t stable, uint32_t inference, uint32_t loop);
#endif
