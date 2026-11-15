#ifndef STARTUP_CONTROL_H
#define STARTUP_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { STARTUP_STOP, STARTUP_GETUP, STARTUP_HANDOFF, STARTUP_BALANCE, STARTUP_FAILED } StartupPhase;
typedef enum { STARTUP_OK, STARTUP_MODEL_MISSING, STARTUP_TIMEOUT, STARTUP_UNSTABLE } StartupFailure;
typedef struct {
    StartupPhase phase;
    StartupFailure failure;
    uint32_t since, stable_since, handoff_since, balance_since;
    float torque_scale, balance_weight;
    bool stable, completed;
} StartupControl;
void startup_step(StartupControl *s, uint32_t now, bool active, bool ready, bool balance_safe, bool actor_available);
#endif
