/* Fake HAL/CAN integration only. Never links the physical board drivers. */
#include "board.h"
#include "chassis.h"
#include "fdcan.h"
#include "imu.h"
#include "robot_config.h"
#include "telemetry.h"
#include "supported_trial.h"
#include "trial_trace.h"
#include <math.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
uint32_t SystemCoreClock=1000000;
volatile uint32_t g_boot_reset_flags;
volatile uint32_t g_can_tx_fail,g_can_rx_overflow;
Imu g_imu;
RemoteControl g_remote;
UART_HandleTypeDef huart5;
static uint32_t now,cycle_count,nonzero_frames,enable_frames,cancel_count;
static uint16_t raw_counts[6];
static bool tx_fail,enabled;
static unsigned snapshots,late_remote_change;
static bool advance_snapshot_clock;
#if !ROBOT_STAND_ENTRY_ONLY
static uint32_t getup_calls;
static bool fake_getup(RlNetwork *network, const float obs[31], float action[6]) {
    (void)network; (void)obs; ++getup_calls;
    memset(action,0,6*sizeof(float)); return true;
}
#endif
uint32_t HAL_GetTick(void) { return now; }
uint32_t board_cycles(void) { cycle_count+=10; return cycle_count; }
void board_remote_snapshot(RemoteControl *r) {
    *r=g_remote;
    if (advance_snapshot_clock && snapshots==0) { ++now; r->tick=now; }
    if (++snapshots==2) {
        if (late_remote_change==1) r->s1=2;
        if (late_remote_change==2) r->valid=false;
        if (late_remote_change==3) r->channel[1]=660;
    }
}
void board_watchdog_feed(void) {}
bool can2_healthy(void) { return true; }
void can2_cancel_pending(void) { ++cancel_count; }
bool can2_send(uint16_t id,const uint8_t d[8]) {
    if (tx_fail) { ++g_can_tx_fail; return false; }
    if (id==0x3fe || id==0x200) {
        for (unsigned i=0;i<8;++i) if (d[i]) { ++nonzero_frames; break; }
    } else if (id>=1 && id<=4) {
        if (d[7]==0xfc) { ++enable_frames; enabled=true; }
        if (d[7]==0xfd) enabled=false;
    }
    return true;
}
static void reset(void) {
    now=cycle_count=nonzero_frames=enable_frames=cancel_count=0;
    tx_fail=enabled=false; g_can_tx_fail=g_can_rx_overflow=0;
    memset(raw_counts,0,sizeof(raw_counts));
    snapshots=late_remote_change=0;
    advance_snapshot_clock=false;
    memset(&g_remote,0,sizeof(g_remote)); memset(&g_imu,0,sizeof(g_imu));
    g_imu.initialized=g_imu.calibrated=true; g_imu.quaternion[0]=1; g_imu.gravity[2]=-1;
    g_remote.valid=true; g_remote.count=1; g_remote.s2=3;
    chassis_init();
}
static void step(uint32_t t,uint8_t s1,bool motors,bool remote) {
    snapshots=0;
    now=t; g_remote.s1=s1;
    if (remote) g_remote.tick=t;
    g_imu.tick=t; g_imu.samples=t+1;
    if (motors) {
        for (unsigned i=0;i<6;++i) {
            uint8_t d[8]={0,0,0,0,0,0,30,0}; d[7]=enabled ? 1 : 0;
            d[0]=(uint8_t)(raw_counts[i]>>8); d[1]=(uint8_t)raw_counts[i];
            chassis_receive((uint16_t)((i==2 || i==5 ? 0x200 : 0x300)+motor_id[i]),d,t);
        }
    }
    chassis_service(t);
}
static int arm(void) {
    for (uint32_t t=1;t<=20;++t) step(t,2,true,true);
    for (uint32_t t=21;t<=380;++t) step(t,1,true,true);
    CHECK(g_chassis.mode==CONTROL_ACTIVE && enable_frames>0);
    return 0;
}
int test_chassis(void) {
#if ROBOT_SUPPORTED_SOFTWARE_TRIGGER
    reset();
    for (uint32_t t=1;t<=20;++t) step(t,2,true,true);
    for (uint32_t t=21;t<=80;++t) step(t,3,true,true);
    CHECK(!enable_frames && !nonzero_frames && g_chassis.mode==CONTROL_STOP);
    g_supported_session.request=1;
    for (uint32_t t=81;t<=100;++t) step(t,3,true,true);
    CHECK(g_chassis.mode==CONTROL_ACTIVE && g_supported_session.accepted==1);
    advance_snapshot_clock=true; step(101,3,true,true); advance_snapshot_clock=false;
    CHECK(g_chassis.mode==CONTROL_ACTIVE && !(g_chassis.inhibit&2));
    for (uint32_t t=103;t<=600;++t) step(t,3,true,true);
    CHECK(g_chassis.mode==CONTROL_FAULT && g_chassis.fault==6 && g_trial_trace.frozen);
    uint32_t session_sent=nonzero_frames;
    for (uint32_t t=601;t<=3000;++t) step(t,3,true,true);
    CHECK(nonzero_frames==session_sent && g_supported_session.accepted==1);
    g_supported_session.request=2;
    step(3001,3,true,true); step(3002,3,true,true);
    CHECK(g_supported_session.status==1 && nonzero_frames==session_sent); /* frozen record blocks reuse */
    trial_trace_reset(); g_supported_session.request=3;
    for (uint32_t t=3003;t<=3020;++t) step(t,3,true,true);
    CHECK(g_chassis.mode==CONTROL_ACTIVE && g_supported_session.accepted==2);
    session_sent=nonzero_frames; late_remote_change=1;
    step(3021,3,true,true); step(3022,3,true,true);
    CHECK(g_chassis.mode==CONTROL_STOP && nonzero_frames==session_sent);
    return 0;
#endif
#if ROBOT_SUPPORTED_TRIAL_MS
    reset(); int guard_error=arm(); if (guard_error) return guard_error;
    uint32_t start=g_supported_trial.started;
    for (uint32_t t=381;t<start+ROBOT_SUPPORTED_TRIAL_MS;++t) step(t,1,true,true);
    uint32_t sent=nonzero_frames;
    step(start+ROBOT_SUPPORTED_TRIAL_MS,1,true,true);
    step(start+ROBOT_SUPPORTED_TRIAL_MS+1,1,true,true);
    CHECK(g_chassis.mode==CONTROL_FAULT && g_chassis.fault==6 && g_supported_trial.reason==1);
    CHECK(nonzero_frames==sent && g_trial_trace.frozen && g_telemetry.frozen);
    for (uint32_t t=start+ROBOT_SUPPORTED_TRIAL_MS+2;t<1000;++t) step(t,1,true,true);
    CHECK(nonzero_frames==sent && g_chassis.mode==CONTROL_FAULT);
    step(1000,2,true,true); step(1001,2,true,true);
    CHECK(g_chassis.mode==CONTROL_STOP && g_chassis.fault==0);
    reset(); guard_error=arm(); if (guard_error) return guard_error;
    for (uint32_t t=381;t<=422;++t) { raw_counts[0]=(uint16_t)(8192-(t-380)*5); step(t,1,true,true); }
    CHECK(g_chassis.mode==CONTROL_FAULT && g_chassis.fault==6 && g_supported_trial.reason==2);
    sent=nonzero_frames; step(423,1,true,true); step(424,1,true,true);
    CHECK(nonzero_frames==sent && g_trial_trace.frozen);
    return 0;
#endif
    reset();
    for (uint32_t t=1;t<=400;++t) step(t,1,true,true);
    CHECK(nonzero_frames==0 && enable_frames==0 && g_chassis.mode==CONTROL_STOP);
    if (!ROBOT_ENABLE_OUTPUT) {
        CHECK(g_chassis.inhibit&1); return 0;
    }
    /* A stop or lost remote arriving between snapshots must suppress FC as well. */
    for (unsigned change=1;change<=2;++change) {
        reset();
        for (uint32_t t=1;t<=20;++t) step(t,2,true,true);
        late_remote_change=change;
        step(21,1,true,true); step(22,1,true,true);
        CHECK(enable_frames==0 && nonzero_frames==0 && g_chassis.mode==CONTROL_STOP);
    }
#if ROBOT_STAND_ENTRY_ONLY
    reset();
    for (uint32_t t=1;t<=25;++t) step(t,2,true,true);
    g_estimator_quality.confidence=NAN;
    step(26,1,true,true); /* between estimator updates: comparisons must reject NaN */
    CHECK(enable_frames==0 && nonzero_frames==0 && (g_chassis.inhibit&256));
    reset();
    for (uint32_t t=1;t<=20;++t) step(t,2,true,true);
    late_remote_change=3;
    step(21,1,true,true); step(22,1,true,true);
    CHECK(enable_frames==0 && nonzero_frames==0);
    reset();
    for (uint32_t t=1;t<=20;++t) step(t,2,true,true);
    for (uint32_t t=21;t<=28;++t) step(t,1,true,true);
    CHECK(g_chassis.mode==CONTROL_ACTIVE);
    g_remote.channel[1]=660;
    for (uint32_t t=29;t<=180;++t) step(t,1,true,true);
    CHECK(g_rl.command[0]==0 && g_startup.torque_scale<1);
    for (uint32_t t=181;t<=300;++t) step(t,1,true,true);
    CHECK(g_startup.torque_scale==1);
    CHECK(g_rl.command[0]>0 || ROBOT_REMOTE_MAX_FORWARD==0);
    g_remote.channel[1]=-660; g_remote.channel[0]=660;
    for (uint32_t t=301;t<=680;++t) step(t,1,true,true);
    CHECK(g_chassis.mode==CONTROL_ACTIVE);
    CHECK(g_rl.command[1]<0 || ROBOT_REMOTE_MAX_YAW==0);
#if RL_PROFILE_ALLOW_REVERSE
    CHECK(g_rl.command[0]<0);
#endif
    g_remote.channel[0]=g_remote.channel[1]=0;
    for (uint32_t t=681;t<=1000;++t) step(t,1,true,true);
    CHECK(g_chassis.mode==CONTROL_ACTIVE && g_rl.command[0]==0 && g_rl.command[1]==0);
#endif
    reset(); int error=arm(); if (error) return error;
    CHECK(nonzero_frames>0 && g_chassis.policy_runs>0);
    uint32_t before=nonzero_frames;
    step(381,2,true,true); step(382,2,true,true);
    CHECK(nonzero_frames==before && g_chassis.mode==CONTROL_STOP && cancel_count>0);
#if ROBOT_STAND_ENTRY_ONLY
    for (uint32_t t=383;t<=402;++t) step(t,2,true,true);
    for (uint32_t t=403;t<=440;++t) step(t,1,true,true);
#else
    for (uint32_t t=383;t<=440;++t) step(t,1,true,true);
#endif
    CHECK(nonzero_frames>before); /* UP runs again; DOWN is the only stop. */

    reset(); error=arm(); if (error) return error;
    for (uint32_t t=381;t<=482;++t) step(t,1,true,false);
    CHECK(g_chassis.mode==CONTROL_STOP && (g_chassis.inhibit&2));
    before=nonzero_frames;
    for (uint32_t t=483;t<=500;++t) step(t,1,true,true);
    CHECK(nonzero_frames==before && g_chassis.mode==CONTROL_STOP);

    reset(); error=arm(); if (error) return error;
    for (uint32_t t=381;t<=412;++t) step(t,1,false,true);
    CHECK(g_chassis.mode==CONTROL_FAULT && (g_chassis.inhibit&4));
    before=nonzero_frames;
    step(413,1,true,true); step(414,1,true,true); CHECK(nonzero_frames==before);

    reset(); error=arm(); if (error) return error;
    g_imu.gravity[2]=0; before=nonzero_frames;
    step(381,1,true,true); step(382,1,true,true);
    CHECK(g_chassis.mode==CONTROL_FAULT && nonzero_frames==before);

    reset(); error=arm(); if (error) return error;
    tx_fail=true; step(381,1,true,true); step(382,1,true,true);
    CHECK(g_chassis.mode==CONTROL_FAULT && g_chassis.fault==1);

    reset(); error=arm(); if (error) return error;
    before=nonzero_frames; step(390,1,true,true);
    CHECK(g_chassis.mode==CONTROL_FAULT && g_chassis.fault==2 && nonzero_frames==before);
    CHECK(g_telemetry.frozen);

    reset(); /* valid running tilt but outside the configured entry window */
    g_imu.gravity[2]=ROBOT_STAND_ENTRY_ONLY ? -cosf(ROBOT_STAND_ENTRY_TILT_RAD+0.02f) : -.98f;
    for (uint32_t t=1;t<=20;++t) step(t,2,true,true);
    for (uint32_t t=21;t<=380;++t) step(t,1,true,true);
#if ROBOT_STAND_ENTRY_ONLY
    CHECK(g_chassis.mode!=CONTROL_ACTIVE && nonzero_frames==0 && enable_frames==0);
    CHECK(g_chassis.inhibit&256);
    /* The measured supported pose must pass through real encoder/geometry entry. */
    reset();
    float supported_tilt=10.7f*RL_PI/180;
    g_imu.quaternion[0]=cosf(supported_tilt/2); g_imu.quaternion[2]=-sinf(supported_tilt/2);
    g_imu.gravity[0]=-sinf(supported_tilt); g_imu.gravity[2]=-cosf(supported_tilt);
    g_imu.accel[0]=9.81f*sinf(supported_tilt); g_imu.accel[2]=9.81f*cosf(supported_tilt);
    const float supported_q[6]={3.413f,-3.423f,0,-3.419f,2.998f,0};
    const float arm_reference[6]=ROBOT_ARM_REFERENCE_RAD;
    float supported_arm[6]={0};
    for (unsigned h=0;h<=3;h+=3) {
        float sign=h==0 ? 1.0f : -1.0f;
        float beta=sign*supported_q[h+1]+RL_PI/2;
        supported_arm[h]=sign*supported_q[h];
        supported_arm[h+1]=supported_arm[h]+2*atan2f(sinf(beta),RL_LINK_RATIO+cosf(beta));
    }
    for (unsigned i=0;i<6;++i) {
        long count=lroundf((supported_arm[i]-arm_reference[i])*8192/(2*RL_PI));
        raw_counts[i]=(uint16_t)((count%8192+8192)%8192);
    }
    for (uint32_t t=1;t<=20;++t) step(t,2,true,true);
    for (uint32_t t=21;t<=380;++t) step(t,1,true,true);
    CHECK(g_chassis.mode==CONTROL_ACTIVE && g_startup.phase==STARTUP_BALANCE);
    CHECK(g_stand_entry.ready && nonzero_frames>0);
    for (unsigned i=0;i<6;++i) CHECK(fabsf(g_measurement.q[i]-supported_q[i])<0.002f);
    /* A pose outside the configured window still prevents entry. */
    reset();
    raw_counts[0]=(uint16_t)(8192-(unsigned)ceilf((ROBOT_STAND_ENTRY_JOINT_TOL_RAD+0.02f)*8192/(2*RL_PI)));
    for (uint32_t t=1;t<=20;++t) step(t,2,true,true);
    for (uint32_t t=21;t<=380;++t) step(t,1,true,true);
    CHECK(enable_frames==0 && nonzero_frames==0 && (g_chassis.inhibit&256));
    /* A displaced stick prevents enabling, even when the measured pose is valid. */
    reset(); g_remote.channel[1]=660;
    for (uint32_t t=1;t<=20;++t) step(t,2,true,true);
    for (uint32_t t=21;t<=380;++t) step(t,1,true,true);
    CHECK(enable_frames==0 && nonzero_frames==0 && (g_chassis.inhibit&256));
    g_remote.channel[1]=0;
    for (uint32_t t=381;t<=420;++t) step(t,2,true,true);
    for (uint32_t t=421;t<=780;++t) step(t,1,true,true);
    CHECK(g_chassis.mode==CONTROL_ACTIVE && g_stand_entry.ready);
    CHECK(g_startup.phase==STARTUP_BALANCE);
    /* Once balancing, moving the stick must issue a driving request. */
    g_remote.channel[1]=660;
    for (uint32_t t=781;t<=820;++t) step(t,1,true,true);
    CHECK(g_chassis.mode==CONTROL_ACTIVE);
    CHECK(g_rl.command[0]>0 || ROBOT_REMOTE_MAX_FORWARD==0);
    before=nonzero_frames;
    step(821,2,true,true); step(822,2,true,true);
    CHECK(nonzero_frames==before && g_chassis.mode==CONTROL_STOP && !g_stand_entry.ready);
#else
    CHECK(g_chassis.mode==CONTROL_FAULT && g_chassis.fault==5 && nonzero_frames==0);
    CHECK(g_telemetry.frozen);
    reset(); getup_calls=0; g_imu.gravity[2]=-.98f;
    CHECK(chassis_register_getup_actor(fake_getup,1));
    error=arm(); if (error) return error;
    CHECK(g_startup.phase==STARTUP_GETUP && getup_calls>0);
    CHECK(!chassis_register_getup_actor(fake_getup,2));
    g_rl.reference_xy[0]=0.25f; g_rl.reference_yaw=0.2f;
    g_imu.gravity[2]=-1;
    for (uint32_t t=381;t<=1800;++t) step(t,1,true,true);
    CHECK(g_chassis.mode==CONTROL_ACTIVE && g_startup.phase==STARTUP_BALANCE);
    CHECK(g_startup.balance_weight==1 && g_startup.torque_scale==1);
#if RL_PROFILE_HANDOFF_REANCHOR
    CHECK(fabsf(g_rl.reference_xy[0]-g_measurement.xy[0])<1e-5f);
    CHECK(fabsf(g_rl.reference_yaw-g_measurement.yaw)<1e-5f);
#endif
#endif
    return 0;
}
