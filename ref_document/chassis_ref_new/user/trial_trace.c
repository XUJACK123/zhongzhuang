#include "trial_trace.h"
#include <stddef.h>
#include <string.h>
volatile TrialTrace g_trial_trace __attribute__((section(".TRACE_RETAIN"),aligned(32)));
volatile TraceTransfer g_trace_transfer;
_Static_assert(sizeof(TrialTrace)<=128*1024,"Trace must fit AXI SRAM");
void trial_trace_boot(uint32_t reset_flags) {
    memset((void *)&g_trace_transfer,0,sizeof(g_trace_transfer));
    /* H723 RSR PORRSTF/BORRSTF invalidate retained SRAM. Other resets may
     * preserve a coherent trace; freeze before any new startup can overwrite it. */
    if (reset_flags && !(reset_flags&0x00a00000U) && g_trial_trace.schema==1 &&
        g_trial_trace.frame_bytes==sizeof(TrialFrame) && g_trial_trace.capacity==TRIAL_TRACE_CAPACITY &&
        g_trial_trace.published && g_trial_trace.frozen<=1) { g_trial_trace.frozen=1; return; }
    trial_trace_reset();
}
void trial_trace_transfer(bool stopped) {
    uint32_t token=g_trace_transfer.request;
    if (token!=g_trace_transfer.ack) {
        uint32_t command=g_trace_transfer.command,offset=g_trace_transfer.offset,length=g_trace_transfer.length;
        uint32_t status=0;
        if (!stopped) status=3;
        else if (command==1) g_trial_trace.frozen=1;
        else if (command==3) trial_trace_reset();
        else if (command!=2) status=4;
        else if (!g_trial_trace.frozen) status=2;
        else if (!length || length>TRACE_TRANSFER_BYTES || offset>sizeof(TrialTrace) || length>sizeof(TrialTrace)-offset) status=1;
        else memcpy((void *)g_trace_transfer.data,(const uint8_t *)&g_trial_trace+offset,length);
        g_trace_transfer.status=status;
        __asm volatile("" ::: "memory"); g_trace_transfer.ack=token;
    }
    memcpy((void *)g_trace_transfer.header,(const void *)&g_trial_trace,sizeof(g_trace_transfer.header));
}
void trial_trace_reset(void) {
    memset((void *)&g_trial_trace,0,sizeof(g_trial_trace));
    g_trial_trace.schema=1; g_trial_trace.frame_bytes=sizeof(TrialFrame);
    g_trial_trace.capacity=TRIAL_TRACE_CAPACITY;
}
void trial_trace_entry(uint32_t reason) {
    if (g_trial_trace.frozen) return;
    g_trial_trace.last_entry_block=reason;
    for (unsigned i=0;i<10;++i) if (reason&(1U<<i)) ++g_trial_trace.entry_counts[i];
}
void trial_trace_publish(const TelemetryFrame *s,const float gravity[3],
                         uint32_t stable,uint32_t inference,uint32_t loop) {
    if (g_trial_trace.frozen) return;
    uint32_t count=g_trial_trace.published,seq=(count+1)*2;
    TrialFrame f={0};
    f.tick=s->tick; f.mode=s->mode; f.inhibit=s->inhibit; f.fault=s->fault;
    f.entry_block=g_trial_trace.last_entry_block; f.stable_ms=stable;
    f.inference_us=inference; f.loop_us=loop;
    for (unsigned i=0;i<6;++i) {
        f.motor_tick[i]=s->motor_tick[i]; f.raw[i]=(uint16_t)s->motor_raw[i];
        f.q[i]=s->q[i]; f.qd[i]=s->qd[i]; f.motor_velocity[i]=s->motor_velocity[i];
        f.torque[i]=s->limited_motor_torque[i]; f.feedback[i]=s->feedback[i]; f.action[i]=s->action[i];
    }
    memcpy(f.omega,s->omega,sizeof(f.omega)); memcpy(f.gravity,gravity,sizeof(f.gravity));
    f.confidence=s->odometry_confidence; f.residual=s->wheel_residual;
    volatile TrialFrame *slot=&g_trial_trace.frame[count%TRIAL_TRACE_CAPACITY];
    slot->begin=seq|1U; __asm volatile("" ::: "memory");
    memcpy((void *)&slot->tick,&f.tick,offsetof(TrialFrame,end)-offsetof(TrialFrame,tick));
    slot->end=seq; __asm volatile("" ::: "memory"); slot->begin=seq;
    g_trial_trace.published=count+1;
    if (count>=TRIAL_TRACE_CAPACITY) ++g_trial_trace.overwritten;
}
