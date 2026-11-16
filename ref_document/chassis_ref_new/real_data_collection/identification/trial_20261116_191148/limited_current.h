#ifndef LIMITED_CURRENT_H
#define LIMITED_CURRENT_H
#include <stdbool.h>
#include <stdint.h>
#define IDENT_CURRENT_LIMIT_A 6.0f
#define IDENT_TRIAL_COUNT 3U
#define IDENT_CURRENT_LEVELS_A {2.0f,4.0f,6.0f}
#define IDENT_COOLDOWN_MS 1000U
#define IDENT_ARM_DELAY_MS 2000U
#define IDENT_CURRENT_DIRECTION (-1.0f) /* fold selected DM3/DM2 knee, away from extension */
#define IDENT_RUN_MS 500U
#define IDENT_ENABLE_MS 100U
#define IDENT_MAX_DELTA_RAD 0.1745329252f
#define IDENT_MAX_SPEED_RAD_S 1.0f
#define IDENT_WHEEL_MAX_DELTA_RAD 0.7853981634f
#define IDENT_WHEEL_MAX_SPEED_RAD_S 5.0f
#define IDENT_SELECTED_INDEX 4U /* LEFT knee DM2; other five torque slots zero */
enum { IDLE, ARMED, ENABLING, RUNNING, DONE, COOLDOWN };
enum { REASON_NONE, COMPLETE, REMOTE_STOP, FEEDBACK_STOP, MOVEMENT_STOP, BUS_STOP, TIME_STOP, INVALID_INPUT };
typedef struct {
    uint32_t phase, reason, since, last_token;
    float origin[6];
    uint32_t trial_index;
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
