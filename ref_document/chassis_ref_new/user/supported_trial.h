#ifndef SUPPORTED_TRIAL_H
#define SUPPORTED_TRIAL_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint32_t started, running, reason; float origin[6]; } SupportedTrial;
/* Reasons: 1 duration, 2 leg travel, 3 wheel travel, 4 wheel speed, 5 nonfinite. */
extern SupportedTrial g_supported_trial;
typedef struct {
    uint32_t request,ack,accepted,status,permit,down_seen,last_stop,has_stopped;
    float requested_wheel_cap,applied_wheel_cap;
} SupportedSession;
extern volatile SupportedSession g_supported_session;
bool supported_session_step(uint32_t now,bool remote_ok,uint8_t s1,bool centered,bool ready,
    bool stopped,bool bounded_stop,bool other_fault,bool trace_ready);
bool supported_trial_stop(uint32_t now, bool active, const float q[6], const float speed[6]);
#endif
