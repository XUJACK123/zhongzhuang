#ifndef LIMITED_CURRENT_H
#define LIMITED_CURRENT_H
#include <stdbool.h>
#include <stdint.h>
#define IDENT_CURRENT_LIMIT_A 0.5f
#define IDENT_CURRENT_DIRECTION (-1.0f) /* fold right knee, away from extension */
#define IDENT_RUN_MS 500U
#define IDENT_ENABLE_MS 100U
#define IDENT_MAX_DELTA_RAD 0.00872664626f
#define IDENT_MAX_SPEED_RAD_S 0.05f
#define IDENT_SELECTED_INDEX 1U /* RIGHT knee DM3; other five torque slots zero */
enum { IDLE, ARMED, ENABLING, RUNNING, DONE };
enum { REASON_NONE, COMPLETE, REMOTE_STOP, FEEDBACK_STOP, MOVEMENT_STOP, BUS_STOP, TIME_STOP, INVALID_INPUT };
typedef struct {
    uint32_t phase, reason, since, last_token;
    bool down_seen;
    float origin[6];
} LimitedCurrent;
typedef struct {
    uint32_t now, token, remote_age;
    uint8_t s1;
    bool remote_valid, healthy;
    float arm[6], velocity[6];
} IdentInput;
void ident_step(LimitedCurrent *c, const IdentInput *in, float *current_a);
void ident_stop(LimitedCurrent *c, uint32_t reason);
#endif
