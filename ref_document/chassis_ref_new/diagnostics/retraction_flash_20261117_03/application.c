/* Isolated suspended-position experiment. Not linked into the normal RL image. */
#include "controller.h"
#include "chassis.h"
#include "board.h"
#include "fdcan.h"
#include "imu.h"
#include "imu_filter.h"
#include "telemetry.h"
#include "trial_trace.h"
#include "supported_trial.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>
ChassisDebug g_chassis;
RlController g_rl; /* ABI-compatible monitoring only; actor is never called. */
RlMeasurement g_measurement;
MotorBank g_motor_snapshot;
StandEntry g_stand_entry;
Retraction g_retraction;
static MotorBank received;
static uint32_t last_loop,last_special,last_record,since,post_stop;
static bool post_pending,stopped_tx;
extern volatile uint32_t g_boot_reset_flags;
void chassis_receive(uint16_t id,const uint8_t data[8],uint32_t now) {
    (void)motor_receive(&received,id,data,8,now);
}
static bool zero(void) {
    const uint8_t d[8]={0};bool a=can2_send(0x3fe,d),b=can2_send(0x200,d);return a&&b;
}
static bool enable_left(bool enable) {
    uint8_t d[8];memset(d,0xff,sizeof(d));bool ok=true;
    for (unsigned id=1;id<=4;++id) {
        d[7]=(enable && id<=2) ? 0xfc : 0xfd;
        if (!can2_send((uint16_t)id,d)) ok=false;
    }
    return ok;
}
static bool moving(void) {return g_chassis.mode==CONTROL_ACTIVE || g_chassis.mode==CONTROL_ENABLING;}
static void stop(uint32_t now,uint32_t why) {
    if (moving()) {post_pending=true;post_stop=now;}
    g_retraction.running=0;
    g_supported_trial.running=0;
    g_supported_session.permit=0;g_supported_session.last_stop=now;g_supported_session.has_stopped=1;
    if (why!=0 && why!=6) g_supported_session.down_seen=0;
    g_chassis.fault=why;g_chassis.mode=why ? CONTROL_FAULT : CONTROL_STOP;
    memset(g_rl.motor_torque,0,sizeof(g_rl.motor_torque));
    /* One stop batch per service call: repeating zero+FD would overflow the
     * eight-slot CAN FIFO when the normal output branch also runs. */
    if (!stopped_tx) {
        stopped_tx=true;last_special=now;
        can2_cancel_pending();(void)zero();(void)enable_left(false);
    }
}
void chassis_init(void) {
    memset(&g_chassis,0,sizeof(g_chassis));memset(&g_rl,0,sizeof(g_rl));
    memset(&g_measurement,0,sizeof(g_measurement));memset(&received,0,sizeof(received));
    memset(&g_motor_snapshot,0,sizeof(g_motor_snapshot));memset(&g_stand_entry,0,sizeof(g_stand_entry));
    memset(&g_retraction,0,sizeof(g_retraction));memset(&g_motor_diagnostics,0,sizeof(g_motor_diagnostics));
    memset(&g_supported_trial,0,sizeof(g_supported_trial));
    memset((void *)&g_supported_session,0,sizeof(g_supported_session));
    trial_trace_boot(g_boot_reset_flags);telemetry_reset();
    last_loop=last_special=last_record=HAL_GetTick();since=post_stop=0;post_pending=stopped_tx=false;
}
void chassis_service(uint32_t caller_now) {
    if (caller_now-last_loop<ROBOT_CONTROL_MS) return;
    uint32_t start=board_cycles();RemoteControl rc;board_remote_snapshot(&rc);
    stopped_tx=false;
    uint32_t mask=__get_PRIMASK();__disable_irq();g_motor_snapshot=received;__set_PRIMASK(mask);
    uint32_t now=HAL_GetTick(),elapsed=now-last_loop;last_loop=now;
    g_chassis.tick=now;++g_chassis.cycles;
    bool remote_ok=remote_fresh(&rc,now),centered=remote_sticks_centered(&rc);
    uint32_t inhibit=ROBOT_ENABLE_OUTPUT ? 0U : 1U;
    if (!remote_ok) inhibit|=2;
    if (!motor_feedback_healthy(&g_motor_snapshot,now)) inhibit|=4;
    if (!g_imu.initialized || !g_imu.calibrated || !g_imu.samples || now-g_imu.tick>ROBOT_IMU_TIMEOUT_MS) inhibit|=8;
    if (!can2_healthy()) inhibit|=16;
    float arm[6],speed[6];
    for (unsigned i=0;i<6;++i) {arm[i]=g_motor_snapshot.motor[i].arm;speed[i]=g_motor_snapshot.motor[i].velocity;}
    bool geometry=!(inhibit&4) && rl_motor_to_joint(arm,speed,g_measurement.q,g_measurement.qd);
    if (!geometry) inhibit|=32;
    if (geometry && (g_measurement.q[1]<rl_physical_knee_limits[0] || g_measurement.q[1]>rl_physical_knee_limits[1] ||
        g_measurement.q[4]<rl_physical_knee_limits[2] || g_measurement.q[4]>rl_physical_knee_limits[3])) inhibit|=32;
    memcpy(g_measurement.gravity,g_imu.gravity,sizeof(g_measurement.gravity));
    memcpy(g_measurement.omega,g_imu.omega,sizeof(g_measurement.omega));g_measurement.yaw=g_imu.yaw;
    if (!isfinite(g_imu.gravity[2]) || g_imu.gravity[2]>-cosf(ROBOT_MAX_TILT_RAD)) inhibit|=64;
    g_imu_fusion.stationary_allowed=remote_ok && rc.s1==2;
    bool still=geometry;
    for (unsigned i=0;i<6;++i) {
        if (!isfinite(g_measurement.qd[i]) || fabsf(g_measurement.qd[i])>.15f) still=false;
        if (fabsf(speed[i])>.02f) g_imu_fusion.stationary_allowed=false;
    }
    bool ready=!inhibit && still && centered && g_measurement.q[4]+RETRACT_DELTA_RAD<=rl_physical_knee_limits[3];
    stand_entry_step(&g_stand_entry,now,moving(),ready);
    g_chassis.inhibit=inhibit;
    if (remote_ok && rc.s1==2) {
        if (moving()) stop(now,0);
        g_chassis.mode=CONTROL_STOP;g_chassis.fault=0;
    }
    if (moving() && (!remote_ok || rc.s1!=3 || !centered)) stop(now,0);
    if (moving() && inhibit) stop(now,4);
    if (moving() && elapsed>ROBOT_CONTROL_MS+1U) {++g_chassis.overruns;stop(now,2);}
    uint32_t accepted=g_supported_session.accepted;
    bool permit=supported_session_step(now,remote_ok,rc.s1,centered,ready && g_stand_entry.ready,
        g_chassis.mode==CONTROL_STOP,g_chassis.mode==CONTROL_FAULT && g_chassis.fault==6,
        g_chassis.mode==CONTROL_FAULT && g_chassis.fault!=6,!g_trial_trace.frozen && !post_pending);
    if (permit && g_supported_session.accepted!=accepted) {
        g_chassis.mode=CONTROL_ENABLING;g_chassis.fault=0;since=now;last_special=now-20U;
        telemetry_reset();
    }
    if (g_chassis.mode==CONTROL_ENABLING && now-since>=100U) {
        g_chassis.mode=CONTROL_ACTIVE;retraction_begin(&g_retraction,now,g_measurement.q);
        g_supported_trial.started=now;g_supported_trial.running=1;g_supported_trial.reason=0;
        memcpy(g_supported_trial.origin,g_measurement.q,sizeof(g_supported_trial.origin));
    }
    if (g_chassis.mode==CONTROL_ACTIVE) {
        if (!retraction_step(&g_retraction,now,g_measurement.q,g_measurement.qd,(float)elapsed*.001f)) {
            g_supported_trial.reason=g_retraction.reason;stop(now,6);
        } else {
            memcpy(g_rl.motor_torque,g_retraction.torque,sizeof(g_rl.motor_torque));
            memcpy(g_rl.target_q,g_retraction.target,sizeof(g_rl.target_q));
        }
    }
    RemoteControl final_rc;board_remote_snapshot(&final_rc);
    if (moving() && (!remote_fresh(&final_rc,HAL_GetTick()) || final_rc.s1!=3 || !remote_sticks_centered(&final_rc))) stop(now,0);
    uint32_t spent=(board_cycles()-start)/(SystemCoreClock/1000000U);
    if (moving() && spent>=2000U) {++g_chassis.overruns;stop(now,2);}
    TelemetryFrame f={0};f.tick=now;f.cycle=g_chassis.cycles;f.control_version=0xD2611171U;
    if (stopped_tx) {
        /* stop() already submitted zero and disabled motors in this cycle. */
    } else if (g_chassis.mode==CONTROL_ACTIVE) {
        if (!motor_encode(g_rl.motor_torque,f.dm,f.dji)) stop(now,4);
        else {
            if (can2_send(0x3fe,f.dm)) f.submitted_mask|=1;
            if (can2_send(0x200,f.dji)) f.submitted_mask|=2;
            if (f.submitted_mask!=3) stop(now,1);
        }
    } else {
        if (!zero()) stop(now,1);
        if (now-last_special>=20U) {last_special=now;if (!enable_left(g_chassis.mode==CONTROL_ENABLING)) stop(now,1);}
    }
    f.mode=g_chassis.mode;f.fault=g_chassis.fault;f.inhibit=inhibit;
    memcpy(f.q,g_measurement.q,sizeof(f.q));memcpy(f.qd,g_measurement.qd,sizeof(f.qd));
    memcpy(f.target_q,g_rl.target_q,sizeof(f.target_q));memcpy(f.limited_motor_torque,g_rl.motor_torque,sizeof(f.limited_motor_torque));
    memcpy(f.omega,g_imu.omega,sizeof(f.omega));memcpy(f.quaternion,g_imu.quaternion,sizeof(f.quaternion));
    f.imu_tick=g_imu.tick;f.remote_tick=rc.tick;
    for (unsigned i=0;i<6;++i) {
        f.motor_tick[i]=g_motor_snapshot.motor[i].tick;f.motor_received[i]=g_motor_snapshot.motor[i].received;
        f.motor_raw[i]=g_motor_snapshot.motor[i].raw;f.motor_velocity[i]=speed[i];f.arm[i]=arm[i];
        f.feedback[i]=g_motor_snapshot.motor[i].current;
    }
    telemetry_publish(&f);
    if (now-last_record>=10U) {last_record=now;trial_trace_publish(&f,g_measurement.gravity,g_stand_entry.stable_ms,0,spent);}
    if (post_pending && now-post_stop>=200U) {g_trial_trace.frozen=1;telemetry_freeze();post_pending=false;}
    trial_trace_transfer(!moving() && !post_pending);
    spent=(board_cycles()-start)/(SystemCoreClock/1000000U);
    if (spent>g_chassis.max_loop_us) g_chassis.max_loop_us=spent;
    board_watchdog_feed();
}
