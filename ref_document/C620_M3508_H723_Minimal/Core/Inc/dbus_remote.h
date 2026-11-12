#ifndef DBUS_REMOTE_H
#define DBUS_REMOTE_H
#include "dbus_protocol.h"
extern volatile DBUS_State g_remote;
extern volatile uint32_t g_remote_uart_errors, g_remote_bad_frames, g_remote_rx_bytes;
int DBUS_Init(void);
DBUS_State DBUS_Snapshot(void);
#endif
