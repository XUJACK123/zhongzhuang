#ifndef MOTOR_IO_H
#define MOTOR_IO_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint16_t raw;
    int64_t counts;
    float arm, velocity, current;
    uint32_t tick, received;
    uint8_t temperature, state;
    bool position_lost;
} MotorFeedback;
typedef struct { MotorFeedback motor[6]; uint32_t rejected; } MotorBank;
extern const uint8_t motor_id[6];
bool motor_receive(MotorBank *bank, uint16_t id, const uint8_t data[8], unsigned len, uint32_t now);
bool motor_feedback_healthy(const MotorBank *bank, uint32_t now);
bool motor_all_enabled(const MotorBank *bank);
bool motor_encode(const float torque[6], uint8_t dm[8], uint8_t dji[8]);
#endif
