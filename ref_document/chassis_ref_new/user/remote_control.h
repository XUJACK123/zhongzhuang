#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    int16_t channel[5];
    uint8_t s1, s2;
    uint32_t tick, count, rejected;
    bool valid;
} RemoteControl;
bool remote_decode(RemoteControl *r, const uint8_t *data, unsigned size, uint32_t now);
bool remote_fresh(const RemoteControl *r, uint32_t now);
void remote_request(const RemoteControl *r, float command[3]);
bool remote_sticks_centered(const RemoteControl *r);
#endif
