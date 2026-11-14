#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { CONTROL_STOP, CONTROL_READY, CONTROL_ENABLING, CONTROL_ACTIVE, CONTROL_FAULT } ControlMode;
typedef struct { ControlMode mode; bool down_seen; uint32_t since, neutral_since; bool neutral; } ControlState;
void control_state_step(ControlState *s, uint32_t now, bool remote_ok, uint8_t s1,
                        uint8_t s2, bool centered, bool healthy, bool enabled);
#endif
