#include "board.h"
#include "chassis.h"
#include "fdcan.h"
#include "imu.h"
#include "supported_trial.h"
#include "trial_trace.h"
#include "controller.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
uint32_t SystemCoreClock=1000000;
volatile uint32_t g_boot_reset_flags,g_can_tx_fail,g_can_rx_overflow;
Imu g_imu;RemoteControl g_remote;UART_HandleTypeDef huart5;
static uint32_t now,cycles,enables,nonzero,feeds;
static unsigned snapshots,late_change,submitted;
static bool lose_feedback,tx_fail;
uint32_t HAL_GetTick(void) {return now;}
uint32_t board_cycles(void) {return cycles+=10;}
void board_remote_snapshot(RemoteControl *r) {
    *r=g_remote;if (++snapshots==2 && late_change) {if (late_change==1) r->s1=2;else r->valid=false;}
}
void board_watchdog_feed(void) {++feeds;}
bool can2_healthy(void) {return true;}
void can2_cancel_pending(void) {}
bool can2_send(uint16_t id,const uint8_t d[8]) {
    assert(++submitted<=8); /* Hardware queue capacity; no duplicate stop batch. */
    if (tx_fail) return false;
    if (id<=4 && d[7]==0xfc) ++enables;
    if (id==0x200) for (unsigned i=0;i<8;++i) assert(d[i]==0);
    if (id==0x3fe) {for (unsigned i=0;i<8;++i) if(d[i]) {++nonzero;break;}}
    return true;
}
static void step(uint8_t s1) {
    now+=2;snapshots=submitted=0;g_remote.s1=s1;g_remote.tick=now;g_imu.tick=now;g_imu.samples++;
    const uint16_t raw[6]={3271,2913,3245,2004,4374,7691};
    if (!lose_feedback) for (unsigned i=0;i<6;++i) {
        uint8_t d[8]={raw[i]>>8,raw[i]&255,0,0,0,0,25,1};
        chassis_receive((i==2||i==5 ? 0x200 : 0x300)+motor_id[i],d,now);
    }
    uint32_t old_feeds=feeds;
    chassis_service(now);assert(feeds==old_feeds+1);
}
static void reset(void) {
    now=cycles=enables=nonzero=0;late_change=snapshots=0;lose_feedback=tx_fail=false;
    memset(&g_remote,0,sizeof(g_remote));memset(&g_imu,0,sizeof(g_imu));
    g_remote.valid=true;g_remote.count=1;g_imu.initialized=g_imu.calibrated=true;g_imu.gravity[2]=-1;
    chassis_init();trial_trace_reset();
}
static void arm(void) {
    reset();for(unsigned i=0;i<300;++i) step(2);
    for(unsigned i=0;i<100;++i) step(3);
    assert(!enables && !nonzero && g_chassis.mode==CONTROL_STOP);
    g_supported_session.request=1;
    for(unsigned i=0;i<60;++i) step(3);
    assert(g_supported_session.accepted==1 && g_chassis.mode==CONTROL_ACTIVE && enables>0);
}
int main(void) {
    reset();for(unsigned i=0;i<400;++i) step(3);g_supported_session.request=1;step(3);
    assert(!enables && !nonzero && !g_supported_session.accepted);
    arm();for(unsigned i=0;i<(RETRACT_TOTAL_MS+400)/2;++i) step(3);
    assert(g_chassis.mode==CONTROL_FAULT && g_chassis.fault==6 && g_supported_trial.reason==1);
    assert(nonzero>0 && g_trial_trace.frozen);uint32_t sent=nonzero;
    unsigned active=0;uint32_t first=0,last=0;
    for (unsigned i=0;i<TRIAL_TRACE_CAPACITY;++i) if(g_trial_trace.frame[i].mode==CONTROL_ACTIVE) {
        uint32_t tick=g_trial_trace.frame[i].tick;if(!active || tick<first) first=tick;if(tick>last) last=tick;++active;
    }
    assert(active==RETRACT_TOTAL_MS/RETRACT_RECORD_MS && last-first==RETRACT_TOTAL_MS-RETRACT_RECORD_MS);
    for(unsigned i=0;i<1200;++i) step(3);
    assert(nonzero==sent && g_supported_session.accepted==1);
    g_supported_session.request=2;step(3);assert(g_supported_session.accepted==1);
    for(unsigned change=1;change<=2;++change) {
        arm();sent=nonzero;late_change=change;step(3);
        assert(g_chassis.mode==CONTROL_STOP && nonzero==sent);
    }
    arm();lose_feedback=true;for(unsigned i=0;i<20;++i) step(3);
    assert(g_chassis.mode==CONTROL_FAULT && g_chassis.fault==4);
    arm();now+=5;step(3);assert(g_chassis.fault==2 && !g_supported_session.permit);
    arm();g_imu.gravity[2]=0;step(3);assert(g_chassis.fault==4);
    arm();tx_fail=true;step(3);assert(g_chassis.fault==1);
    puts("PASS: bilateral enable, zero wheels, full-duration trace, one-shot/down-seen, timeout/freeze, no retry, late remote stop/loss, stale feedback, timing, tilt and CAN failure");
    return 0;
}
