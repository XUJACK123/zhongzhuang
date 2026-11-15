#include "telemetry.h"
#include <stddef.h>
#include <string.h>
volatile TelemetryRing g_telemetry;
void telemetry_reset(void) {
    memset((void *)&g_telemetry,0,sizeof(g_telemetry));
    g_telemetry.schema=TELEMETRY_SCHEMA; g_telemetry.frame_bytes=sizeof(TelemetryFrame);
    g_telemetry.capacity=TELEMETRY_CAPACITY;
}
void telemetry_publish(const TelemetryFrame *frame) {
    if (g_telemetry.frozen) return;
    uint32_t count=g_telemetry.published,sequence=(count+1)*2;
    volatile TelemetryFrame *slot=&g_telemetry.frame[count%TELEMETRY_CAPACITY];
    slot->begin=sequence|1U;
    __asm volatile("" ::: "memory");
    memcpy((void *)&slot->tick,&frame->tick,offsetof(TelemetryFrame,end)-offsetof(TelemetryFrame,tick));
    slot->end=sequence;
    __asm volatile("" ::: "memory");
    slot->begin=sequence;
    g_telemetry.published=count+1;
    if (count>=TELEMETRY_CAPACITY) ++g_telemetry.overwritten;
}
void telemetry_freeze(void) { g_telemetry.frozen=1; }
