/* Isolated raw-motor current probe. No actor or joint-space position control. */
#include "controller.h"
#include "measurement.h"
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
static const float encoder_signs[6]=ROBOT_ENCODER_SIGNS;
static uint32_t last_loop,last_special,last_record,since,post_stop;
static bool post_pending,stopped_tx;
static uint32_t record_interval=RETRACT_RECORD_MS;
extern volatile uint32_t g_boot_reset_flags;
void chassis_receive(uint16_t id,const uint8_t data[8],uint32_t now) {
    (void)motor_receive(&received,id,data,8,now);
}
static bool zero(void) {
    const uint8_t d[8]={0};bool a=can2_send(0x3fe,d),b=can2_send(0x200,d);return a&&b;
}
static bool enable_legs(bool enable) {
    uint8_t d[8];memset(d,0xff,sizeof(d));bool ok=true;
    for (unsigned id=1;id<=4;++id) {
        d[7]=enable && id==motor_id[g_retraction.index] ? 0xfc : 0xfd;
        if (!can2_send((uint16_t)id,d)) ok=false;
    }
    return ok;
}
static bool moving(void) {return g_chassis.mode==CONTROL_ACTIVE || g_chassis.mode==CONTROL_ENABLING;}
static void stop(uint32_t now,uint32_t why) {
    if (moving() && !g_stop_event.valid) {
        g_stop_event.tick=now;g_stop_event.fault=why;g_stop_event.reason=g_retraction.reason;g_stop_event.inhibit=g_chassis.inhibit;
        memcpy(g_stop_event.q,g_measurement.q,sizeof(g_stop_event.q));memcpy(g_stop_event.qd,g_measurement.qd,sizeof(g_stop_event.qd));
        memcpy(g_stop_event.torque,g_rl.motor_torque,sizeof(g_stop_event.torque));memcpy(g_stop_event.origin,g_retraction.origin,sizeof(g_stop_event.origin));
        for(unsigned i=0;i<6;++i) g_stop_event.motor_velocity[i]=g_motor_snapshot.motor[i].velocity;
        g_stop_event.valid=1;
    }
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
        can2_cancel_pending();(void)zero();(void)enable_legs(false);
    }
}
void chassis_init(void) {
    memset(&g_chassis,0,sizeof(g_chassis));memset(&g_rl,0,sizeof(g_rl));
    memset(&g_measurement,0,sizeof(g_measurement));memset(&received,0,sizeof(received));
    memset(&g_motor_snapshot,0,sizeof(g_motor_snapshot));memset(&g_stand_entry,0,sizeof(g_stand_entry));
    memset(&g_retraction,0,sizeof(g_retraction));memset(&g_motor_diagnostics,0,sizeof(g_motor_diagnostics));
    memset(&g_static_measurement,0,sizeof(g_static_measurement));memset(&g_stop_event,0,sizeof(g_stop_event));record_interval=RETRACT_RECORD_MS;
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
    /* Trace q is raw-positive output-shaft angle here, NOT generalized joint q.
     * Encoder reference/kinematic sign errors cannot feed back into this probe. */
    bool geometry=!(inhibit&4);
    for(unsigned i=0;i<6;++i) {
        float reduction=(i==2 || i==5) ? ROBOT_WHEEL_GEAR_RATIO : 1;
        g_measurement.q[i]=(float)g_motor_snapshot.motor[i].counts*(2*RL_PI/8192)/reduction;
        g_measurement.qd[i]=speed[i]*encoder_signs[i];
        if(!isfinite(g_measurement.q[i]) || !isfinite(g_measurement.qd[i])) geometry=false;
    }
    if (!geometry) inhibit|=32;
    memcpy(g_measurement.gravity,g_imu.gravity,sizeof(g_measurement.gravity));
    memcpy(g_measurement.omega,g_imu.omega,sizeof(g_measurement.omega));g_measurement.yaw=g_imu.yaw;
    if (!isfinite(g_imu.gravity[2]) || g_imu.gravity[2]>-cosf(ROBOT_MAX_TILT_RAD)) inhibit|=64;
    g_imu_fusion.stationary_allowed=remote_ok && rc.s1==2;
    bool still=geometry;
    for (unsigned i=0;i<6;++i) {
        if (!isfinite(g_measurement.qd[i]) || fabsf(g_measurement.qd[i])>.06f) still=false;
        if (fabsf(speed[i])>.02f) g_imu_fusion.stationary_allowed=false;
    }
    bool ready=g_static_measurement.state!=1 && !inhibit && still && centered && probe_configuration_valid();
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
        memset(&g_stop_event,0,sizeof(g_stop_event));record_interval=RETRACT_RECORD_MS;
        retraction_begin(&g_retraction,now+100U,g_measurement.q);
        g_chassis.mode=CONTROL_ENABLING;g_chassis.fault=0;since=now;last_special=now-20U;
        memcpy(g_supported_trial.origin,g_retraction.origin,sizeof(g_supported_trial.origin));
        telemetry_reset();
    }
    if(g_static_measurement.request!=g_static_measurement.ack) {
        g_static_measurement.ack=g_static_measurement.request;
        if(!moving() && !post_pending && remote_ok && rc.s1==2 && !g_supported_session.permit) {
            static_measurement_begin(now);trial_trace_reset();record_interval=120;last_record=now-120;
            g_static_measurement.tx_fail_start=g_can_tx_fail;g_static_measurement.rx_overflow_start=g_can_rx_overflow;
            g_static_measurement.drdy_start=g_imu.drdy_edges;g_static_measurement.imu_missed_start=g_imu_timing.missed_edges;
        } else g_static_measurement.state=4;
    }
    if(g_static_measurement.state==1) {
        float values[MEASURE_VALUES];memcpy(values,g_measurement.q,6*sizeof(float));memcpy(values+6,g_measurement.qd,6*sizeof(float));
        memcpy(values+12,g_imu.omega,3*sizeof(float));memcpy(values+15,g_imu.gravity,3*sizeof(float));values[18]=g_imu.yaw;
        values[19]=sqrtf(g_imu.accel[0]*g_imu.accel[0]+g_imu.accel[1]*g_imu.accel[1]+g_imu.accel[2]*g_imu.accel[2]);
        if(!geometry) values[0]=NAN;
        StaticMeasurement *s=&g_static_measurement;
        for(unsigned i=0;i<6;++i) {uint32_t age=now-g_motor_snapshot.motor[i].tick;if(age>s->max_motor_age_ms)s->max_motor_age_ms=age;}
        uint32_t age=now-g_imu.tick;if(age>s->max_imu_age_ms)s->max_imu_age_ms=age;
        age=now-rc.tick;if(age>s->max_remote_age_ms)s->max_remote_age_ms=age;
        static_measurement_sample(now,elapsed,remote_ok && rc.s1==2 && !moving(),inhibit,values);
        s->tx_fail_end=g_can_tx_fail;s->rx_overflow_end=g_can_rx_overflow;s->drdy_end=g_imu.drdy_edges;s->imu_missed_end=g_imu_timing.missed_edges;
        if(s->state!=1) g_trial_trace.frozen=1;
    }
    if(g_chassis.mode==CONTROL_ENABLING && !probe_guard(&g_retraction,g_measurement.q,g_measurement.qd)) {
        g_supported_trial.reason=g_retraction.reason;stop(now,6);
    }
    if (g_chassis.mode==CONTROL_ENABLING && now-since>=100U) {
        g_chassis.mode=CONTROL_ACTIVE;
        g_supported_trial.started=now;g_supported_trial.running=1;g_supported_trial.reason=0;
        memcpy(g_supported_trial.origin,g_retraction.origin,sizeof(g_supported_trial.origin));
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
    TelemetryFrame f={0};f.tick=now;f.cycle=g_chassis.cycles;f.control_version=RETRACT_CONTROL_VERSION;
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
        if (now-last_special>=20U) {last_special=now;if (!enable_legs(g_chassis.mode==CONTROL_ENABLING)) stop(now,1);}
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
    if (post_pending && now==post_stop) telemetry_freeze(); /* retain 2 ms samples around the stop, not 200 ms later */
    if (now-last_record>=record_interval) {last_record=now;trial_trace_publish(&f,g_measurement.gravity,g_stand_entry.stable_ms,0,spent);}
    if (post_pending && now-post_stop>=200U) {g_trial_trace.frozen=1;telemetry_freeze();post_pending=false;}
    trial_trace_transfer(!moving() && !post_pending);
    spent=(board_cycles()-start)/(SystemCoreClock/1000000U);
    if (spent>g_chassis.max_loop_us) g_chassis.max_loop_us=spent;
    board_watchdog_feed();
}
