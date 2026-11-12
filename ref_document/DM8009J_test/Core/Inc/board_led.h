#ifndef BOARD_LED_H
#define BOARD_LED_H
#include "remote_indicator.h"
int BoardLED_Init(void);
void BoardLED_Update(const DBUS_State *remote, uint32_t now);
#endif
