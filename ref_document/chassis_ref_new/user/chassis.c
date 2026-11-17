#include "chassis.h"
#include "board.h"
#include "fdcan.h"
#include "imu.h"
#include "imu_filter.h"
#include "telemetry.h"
#include "trial_trace.h"
#include "supported_trial.h"
#include "state_estimator.h"
#include "algorithm/model/model.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>
ChassisDebug g_chassis;
RlController g_rl;
RlMeasurement g_measurement;
MotorBank g_motor_snapshot;
StartupControl g_startup;
EstimatorQuality g_estimator_quality;
StandEntry g_stand_entry;
uint32_t g_snapshot_clock_races;
static RlController getup_controller;
static RlActorFn getup_actor;
static uint32_t getup_model_version;
static MotorBank received;
static StateEstimator estimator;
static ControlState control;
static uint32_t last_loop,last_sample,last_policy,last_special,last_estimator;
static bool have_measurement,shadow_started;
static uint32_t entry_reason;
extern volatile uint32_t g_boot_reset_flags;
void chassis_receive(uint16_t id, const uint8_t d[8], uint32_t now) {
    (void)motor_receive(&received,id,d,8,now);
}
void chassis_init(void) {
    memset(&g_chassis,0,sizeof(g_chassis)); memset(&g_rl,0,sizeof(g_rl));
    memset(&g_measurement,0,sizeof(g_measurement)); memset(&received,0,sizeof(received));
    memset(&g_motor_snapshot,0,sizeof(g_motor_snapshot)); memset(&control,0,sizeof(control));
    memset(&g_startup,0,sizeof(g_startup)); memset(&g_estimator_quality,0,sizeof(g_estimator_quality));
    memset(&g_stand_entry,0,sizeof(g_stand_entry));
    memset(&g_motor_diagnostics,0,sizeof(g_motor_diagnostics));
    memset(&g_supported_trial,0,sizeof(g_supported_trial));
    memset((void *)&g_supported_session,0,sizeof(g_supported_session));
    g_snapshot_clock_races=0;
    g_supported_session.requested_wheel_cap=g_supported_session.applied_wheel_cap=ROBOT_WHEEL_TORQUE_CAP_NM;
    trial_trace_boot(g_boot_reset_flags); entry_reason=0;
    getup_actor=0; getup_model_version=0; telemetry_reset();
    have_measurement=false; shadow_started=false;
    float output[6];
    bool ok=rl_actor(&g_rl.network,rl_selftest_obs,output);
    for (unsigned i=0;i<6;++i) if (fabsf(output[i]-rl_selftest_output[i])>0.001f) ok=false;
    if (rl_getup_model_available() && !rl_getup_selftest(&g_rl.network)) ok=false;
    /* Latest real-data actors are linked and checked, but remain unregistered:
     * held-out qualification and physical assistance/contact observability are pending. */
    g_chassis.actor_selftest=ok;
    last_loop=last_sample=last_policy=last_special=last_estimator=HAL_GetTick();
    estimator_reset(&estimator);
}
bool chassis_register_getup_actor(RlActorFn actor, uint32_t model_version) {
    if (ROBOT_STAND_ENTRY_ONLY || control.mode!=CONTROL_STOP || !actor || !model_version) return false;
    getup_actor=actor; getup_model_version=model_version; return true;
}
static bool startup_ready(void) {
    float joint_tolerance=ROBOT_STAND_ENTRY_ONLY ? ROBOT_STAND_ENTRY_JOINT_TOL_RAD : 0.15f;
    float tilt_limit=ROBOT_STAND_ENTRY_ONLY ? ROBOT_STAND_ENTRY_TILT_RAD : 8*RL_PI/180;
    float speed_limit=g_startup.phase==STARTUP_GETUP ? RL_PROFILE_ENTRY_SPEED_M_S : 0.08f;
    if (g_startup.phase==STARTUP_HANDOFF) speed_limit=RL_PROFILE_BLEND_SPEED_M_S;
    entry_reason=0;
    if (!isfinite(g_estimator_quality.confidence) || g_estimator_quality.confidence<0.5f) entry_reason|=1;
    if (!isfinite(g_measurement.gravity[2]) || g_measurement.gravity[2]>-cosf(tilt_limit)) entry_reason|=2;
    if (!isfinite(g_measurement.velocity[0]) || fabsf(g_measurement.velocity[0])>speed_limit) entry_reason|=4;
    for (unsigned i=0;i<3;++i)
        if (!isfinite(g_measurement.omega[i]) || fabsf(g_measurement.omega[i])>0.5f) entry_reason|=8;
    for (unsigned i=0;i<6;++i) {
        if (i==2 || i==5) continue;
        float error=g_measurement.q[i]-rl_default_q[i];
        if (i==0 || i==3) error=rl_wrap(error);
        if (!isfinite(error) || fabsf(error)>joint_tolerance) entry_reason|=16;
    }
    /* Contact/clearance proxy from joint geometry and mutually consistent wheel
     * odometry. This proxy still needs physical contact validation. */
    return entry_reason==0;
}
static bool special(uint8_t tail) {
    uint8_t d[8]; memset(d,0xff,8); d[7]=tail;
    bool ok=true;
    for (unsigned id=1;id<=4;++id) if (!can2_send((uint16_t)id,d)) ok=false;
    return ok;
}
static bool zero_output(void) {
    const uint8_t zero[8]={0};
    bool dm=can2_send(0x3fe,zero), dji=can2_send(0x200,zero); return dm && dji;
}
static void fault(uint32_t why) {
    g_chassis.fault=why; control.mode=CONTROL_FAULT; control.down_seen=false;
    memset(&g_stand_entry,0,sizeof(g_stand_entry));
    can2_cancel_pending(); (void)zero_output(); (void)special(0xfd);
}
void chassis_service(uint32_t now) {
    if (now==last_sample) return;
    last_sample=now;
    uint32_t start=board_cycles(),elapsed=now-last_loop;
    RemoteControl rc; board_remote_snapshot(&rc);
    uint32_t mask=__get_PRIMASK(); __disable_irq(); g_motor_snapshot=received; __set_PRIMASK(mask);
    /* IRQ snapshots can be newer than the caller's millisecond tick. Refresh
     * the comparison clock AFTER copying them, not by relaxing freshness. */
    bool future_tick=(int32_t)(rc.tick-now)>0;
    for (unsigned i=0;i<6;++i) future_tick|=(int32_t)(g_motor_snapshot.motor[i].tick-now)>0;
    if (future_tick) ++g_snapshot_clock_races;
    now=HAL_GetTick(); last_sample=now; elapsed=now-last_loop;
    uint32_t inhibit=0;
    bool configured=ROBOT_ENABLE_OUTPUT;
    if (!configured) inhibit|=1;
    bool remote_ok=remote_fresh(&rc,now);
    if (!remote_ok) inhibit|=2;
    if (!motor_feedback_healthy(&g_motor_snapshot,now)) inhibit|=4;
    if (!g_imu.initialized || !g_imu.calibrated || !g_imu.samples || now-g_imu.tick>ROBOT_IMU_TIMEOUT_MS) inhibit|=8;
    if (!can2_healthy()) inhibit|=16;
    if (!g_chassis.actor_selftest) inhibit|=128;
    memcpy(g_measurement.omega,g_imu.omega,sizeof(g_measurement.omega));
    memcpy(g_measurement.gravity,g_imu.gravity,sizeof(g_measurement.gravity)); g_measurement.yaw=g_imu.yaw;
    float arm[6],speed[6]; uint32_t feedback_tick[6];
    g_imu_fusion.stationary_allowed=remote_ok && rc.s1==2;
    for (unsigned i=0;i<6;++i) {
        arm[i]=g_motor_snapshot.motor[i].arm; speed[i]=g_motor_snapshot.motor[i].velocity;
        feedback_tick[i]=g_motor_snapshot.motor[i].tick;
        if (fabsf(speed[i])>0.02f) g_imu_fusion.stationary_allowed=false;
    }
    bool geometry=false;
    if (!(inhibit&(4|8)) && ROBOT_MOTOR_CALIBRATED) {
        if (now-last_estimator>=5) {
            geometry=estimator_update_qualified(&estimator,&g_estimator_quality,&g_measurement,
                g_imu.quaternion,arm,speed,g_imu.omega,g_imu.accel,feedback_tick,now,(float)(now-last_estimator)*0.001f);
            last_estimator=now; have_measurement=geometry;
        } else {
            float unused[6];
            geometry=have_measurement && rl_motor_to_joint(arm,speed,g_measurement.q,unused);
        }
    } else {
        have_measurement=false; estimator.velocity_valid=false; last_estimator=now;
        g_estimator_quality.confidence=0; g_estimator_quality.initialized=false;
        g_estimator_quality.degraded_ms=1;
    }
    if (!geometry) inhibit|=32;
    if (!isfinite(g_imu.gravity[2]) || g_imu.gravity[2]>-cosf(ROBOT_MAX_TILT_RAD)) inhibit|=64;
    if (geometry) {
        if (g_measurement.q[1]<rl_physical_knee_limits[0]-0.02f || g_measurement.q[1]>rl_physical_knee_limits[1]+0.02f ||
            g_measurement.q[4]<rl_physical_knee_limits[2]-0.02f || g_measurement.q[4]>rl_physical_knee_limits[3]+0.02f) inhibit|=32;
        for (unsigned i=0;i<6;++i) if (fabsf(g_measurement.qd[i])>(i==2 || i==5 ? 75 : 35)) inhibit|=32;
    }
    bool measurable=(inhibit&(4|8|16|32|64|128))==0;
    entry_reason=0;
    bool ready=measurable && startup_ready();
    bool stationary=true;
    for (unsigned i=0;i<6;++i) {
        float limit=(i==2 || i==5) ? 0.5f : 0.2f;
        if (!isfinite(g_measurement.qd[i]) || fabsf(g_measurement.qd[i])>limit) stationary=false;
    }
    bool entry_ok=ready && stationary && remote_ok && remote_sticks_centered(&rc);
    if (!stationary) entry_reason|=32;
    if (!remote_ok) entry_reason|=64;
    if (!remote_sticks_centered(&rc)) entry_reason|=128;
    if (!measurable) entry_reason|=256;
    if (control.mode!=CONTROL_ACTIVE && g_stand_entry.tracking && now-g_stand_entry.last_tick>ROBOT_CONTROL_MS+1U) entry_reason|=512;
    trial_trace_entry(entry_reason);
    if (ROBOT_STAND_ENTRY_ONLY) {
        stand_entry_step(&g_stand_entry,now,control.mode==CONTROL_ACTIVE,entry_ok);
        if (control.mode!=CONTROL_ACTIVE && !g_stand_entry.ready) inhibit|=256;
    }
    g_chassis.inhibit=inhibit;
    if (elapsed<ROBOT_CONTROL_MS) return;
    last_loop=now; g_chassis.tick=now; ++g_chassis.cycles;
    ControlMode previous=control.mode;
#if ROBOT_SUPPORTED_SOFTWARE_TRIGGER
    uint32_t accepted=g_supported_session.accepted;
    bool permit=supported_session_step(now,remote_ok,rc.s1,remote_sticks_centered(&rc),inhibit==0,
        control.mode==CONTROL_STOP,control.mode==CONTROL_FAULT && g_chassis.fault==6,
        control.mode==CONTROL_FAULT && g_chassis.fault!=6,!g_trial_trace.frozen);
    if (g_supported_session.accepted!=accepted) {
        control.mode=CONTROL_STOP; control.down_seen=true; g_chassis.fault=0;
    }
    if (permit || (remote_ok && rc.s1==2) || !remote_ok)
        control_state_step(&control,now,remote_ok,rc.s1,rc.s2,remote_sticks_centered(&rc),
                           inhibit==0,motor_all_enabled(&g_motor_snapshot));
    else if (control.mode!=CONTROL_FAULT) control.mode=CONTROL_STOP;
#else
    control_state_step(&control,now,remote_ok,rc.s1,rc.s2,remote_sticks_centered(&rc),
                       inhibit==0,motor_all_enabled(&g_motor_snapshot));
#endif
    if (supported_trial_stop(now,control.mode==CONTROL_ACTIVE,g_measurement.q,speed)) fault(6);
    if (previous!=CONTROL_ACTIVE && control.mode==CONTROL_ACTIVE) telemetry_reset();
    /* Send one enable frame on entry so group-mode motors see it immediately. */
    if (previous!=CONTROL_ENABLING && control.mode==CONTROL_ENABLING) last_special=now-20;
    if (remote_ok && rc.s1==2) g_chassis.fault=0;
    if (elapsed>ROBOT_CONTROL_MS+1 && (previous==CONTROL_ACTIVE || previous==CONTROL_ENABLING)) {
        ++g_chassis.overruns; fault(2);
    }
    if ((previous==CONTROL_ACTIVE || previous==CONTROL_ENABLING) &&
        control.mode!=CONTROL_ACTIVE && control.mode!=CONTROL_ENABLING) can2_cancel_pending();
    if (previous==CONTROL_ACTIVE && control.mode!=CONTROL_ACTIVE)
        memset(&g_stand_entry,0,sizeof(g_stand_entry));
    /* Shadow mode executes the real pipeline only with calibrated, fresh measurements.
     * Config flags still prevent enabling motors. No fabricated IMU/joint data. */
    StartupPhase previous_startup=g_startup.phase;
    bool balance_safe=measurable && (!RL_PROFILE_HAS_GETUP || ready);
    startup_step(&g_startup,now,control.mode==CONTROL_ACTIVE,ready,balance_safe,
                 !ROBOT_STAND_ENTRY_ONLY && getup_actor!=0);
    if (RL_PROFILE_HANDOFF_REANCHOR && previous_startup==STARTUP_GETUP && g_startup.phase==STARTUP_HANDOFF) {
        g_rl.reference_xy[0]=getup_controller.reference_xy[0]=g_measurement.xy[0];
        g_rl.reference_xy[1]=getup_controller.reference_xy[1]=g_measurement.xy[1];
        g_rl.reference_yaw=getup_controller.reference_yaw=g_measurement.yaw;
    }
    if (g_startup.phase==STARTUP_FAILED) fault(5);
    if (measurable && (control.mode==CONTROL_ACTIVE || !ROBOT_ENABLE_OUTPUT)) {
        if (!shadow_started || (control.mode==CONTROL_ACTIVE && previous!=CONTROL_ACTIVE)) {
            rl_reset(&g_rl,&g_measurement); last_policy=now-ROBOT_POLICY_MS; shadow_started=true;
            rl_reset(&getup_controller,&g_measurement);
        }
        if (now-last_policy>=ROBOT_POLICY_MS) {
            float request[3]={0,0,ROBOT_HEIGHT_COMMAND_M};
            bool trusted=g_estimator_quality.confidence>=0.25f;
            if (control.mode==CONTROL_ACTIVE && g_startup.phase==STARTUP_BALANCE &&
                g_startup.torque_scale>=1.0f && trusted) remote_request(&rc,request);
            uint32_t inference_start=board_cycles();
            uint32_t policy_elapsed=now-last_policy;
            if (policy_elapsed>ROBOT_POLICY_MS+1) { ++g_chassis.overruns; fault(2); }
            rl_manage_reference(&g_rl,&g_measurement,trusted,g_estimator_quality.reference_realign);
            rl_manage_reference(&getup_controller,&g_measurement,trusted,g_estimator_quality.reference_realign);
            g_estimator_quality.reference_realign=false;
            bool ok=rl_policy_step(&g_rl,&g_measurement,request,(float)policy_elapsed*0.001f);
            if (control.mode==CONTROL_ACTIVE && (g_startup.phase==STARTUP_GETUP || g_startup.phase==STARTUP_HANDOFF)) {
                float stand[3]={0,0,ROBOT_HEIGHT_COMMAND_M};
                ok=ok && rl_policy_step_actor(&getup_controller,&g_measurement,stand,(float)policy_elapsed*0.001f,getup_actor);
                float blended[6],weight=g_startup.balance_weight;
                for (unsigned i=0;i<6;++i) blended[i]=(1-weight)*getup_controller.action[i]+weight*g_rl.action[i];
                ok=ok && rl_set_action(&g_rl,blended) && rl_set_action(&getup_controller,blended);
            }
            g_chassis.inference_us=(board_cycles()-inference_start)/(SystemCoreClock/1000000U);
            last_policy=now; ++g_chassis.policy_runs;
            if (!ok) fault(3);
        }
        if (!rl_torque_step(&g_rl,&g_measurement)) fault(4);
    } else shadow_started=false;
    TelemetryFrame frame={0};
    frame.tick=now; frame.cycle=g_chassis.cycles;
    frame.control_version=ROBOT_CONTROL_VERSION; frame.model_version=ROBOT_MODEL_VERSION;
    frame.getup_model_version=getup_model_version; frame.phase=g_startup.phase; frame.failure=g_startup.failure;
    frame.inhibit=inhibit; frame.policy_age_ms=now-last_policy;
    memcpy(frame.quaternion,g_imu.quaternion,sizeof(frame.quaternion));
    memcpy(frame.omega,g_imu.omega,sizeof(frame.omega));
    memcpy(frame.acceleration,g_imu.accel,sizeof(frame.acceleration));
    memcpy(frame.bias,g_imu.bias,sizeof(frame.bias));
    memcpy(frame.xy,g_measurement.xy,sizeof(frame.xy));
    memcpy(frame.velocity,g_measurement.velocity,sizeof(frame.velocity));
    frame.imu_missed_edges=g_imu_timing.missed_edges;
    memcpy(frame.q,g_measurement.q,sizeof(frame.q)); memcpy(frame.qd,g_measurement.qd,sizeof(frame.qd));
    memcpy(frame.obs,g_rl.obs,sizeof(frame.obs)); memcpy(frame.action,g_rl.action,sizeof(frame.action));
    memcpy(frame.target_q,g_rl.target_q,sizeof(frame.target_q)); memcpy(frame.target_qd,g_rl.target_qd,sizeof(frame.target_qd));
    float unused_speed[6];
    (void)rl_joint_to_motor(g_measurement.q,g_measurement.qd,g_rl.joint_torque,frame.requested_motor_torque,unused_speed);
    for (unsigned i=0;i<6;++i) {
        frame.motor_tick[i]=feedback_tick[i]; frame.motor_received[i]=g_motor_snapshot.motor[i].received;
        frame.feedback[i]=g_motor_snapshot.motor[i].current;
        frame.motor_raw[i]=g_motor_snapshot.motor[i].raw;
        frame.arm[i]=g_motor_snapshot.motor[i].arm;
        frame.motor_velocity[i]=g_motor_snapshot.motor[i].velocity;
        frame.unwrap_ambiguous[i]=g_motor_diagnostics.ambiguous[i];
#if ROBOT_SUPPORTED_SOFTWARE_TRIGGER
        if (i==2 || i==5) g_rl.motor_torque[i]=rl_clip(g_rl.motor_torque[i],
            -g_supported_session.applied_wheel_cap,g_supported_session.applied_wheel_cap);
#endif
        if (control.mode==CONTROL_ACTIVE) g_rl.motor_torque[i]*=g_startup.torque_scale;
    }
    memcpy(frame.limited_motor_torque,g_rl.motor_torque,sizeof(frame.limited_motor_torque));
    frame.odometry_confidence=g_estimator_quality.confidence; frame.wheel_residual=g_estimator_quality.residual;
    frame.accel_weight=g_imu_fusion.acceleration_weight; frame.imu_dt_s=g_imu_timing.sample_dt_s;
    frame.torque_scale=g_startup.torque_scale; frame.balance_weight=g_startup.balance_weight;
    frame.imu_tick=g_imu.tick; frame.remote_tick=rc.tick;
    /* Check elapsed wall time and remote AGAIN after inference, before any torque TX. */
    uint32_t spent=(board_cycles()-start)/(SystemCoreClock/1000000U);
    if (spent>g_chassis.max_loop_us) g_chassis.max_loop_us=spent;
    if (spent>=ROBOT_CONTROL_MS*1000U) { ++g_chassis.overruns; fault(2); }
    RemoteControl final_rc; board_remote_snapshot(&final_rc);
    if ((control.mode==CONTROL_ACTIVE || control.mode==CONTROL_ENABLING) &&
        (!remote_fresh(&final_rc,HAL_GetTick()) || final_rc.s1==2)) {
        control.mode=CONTROL_STOP; control.down_seen=false; can2_cancel_pending();
        memset(&g_stand_entry,0,sizeof(g_stand_entry));
        g_supported_session.permit=0;
    }
    if (ROBOT_STAND_ENTRY_ONLY && control.mode==CONTROL_ENABLING && !remote_sticks_centered(&final_rc)) {
        control.mode=CONTROL_STOP; control.down_seen=false; can2_cancel_pending();
        memset(&g_stand_entry,0,sizeof(g_stand_entry));
        g_supported_session.permit=0;
    }
#if ROBOT_SUPPORTED_SOFTWARE_TRIGGER
    if ((control.mode==CONTROL_ACTIVE || control.mode==CONTROL_ENABLING) &&
        (final_rc.s1!=3 || !remote_sticks_centered(&final_rc))) {
        control.mode=CONTROL_STOP; control.down_seen=false; can2_cancel_pending();
        memset(&g_stand_entry,0,sizeof(g_stand_entry));
        g_supported_session.permit=0;
    }
#endif
    if (control.mode==CONTROL_ACTIVE) {
        uint8_t dm[8],dji[8];
        if (!motor_encode(g_rl.motor_torque,dm,dji)) fault(1);
        else {
            memcpy(frame.dm,dm,8); memcpy(frame.dji,dji,8);
            if (can2_send(0x3fe,dm)) frame.submitted_mask|=1;
            if (can2_send(0x200,dji)) frame.submitted_mask|=2;
            if (frame.submitted_mask!=3) fault(1);
        }
    } else {
        if (!zero_output()) fault(1);
        if (now-last_special>=20) {
            last_special=now;
            if (!special(control.mode==CONTROL_ENABLING ? 0xfc : 0xfd)) fault(1);
        }
    }
    g_chassis.mode=control.mode;
    frame.mode=control.mode; frame.fault=g_chassis.fault; telemetry_publish(&frame);
    trial_trace_publish(&frame,g_measurement.gravity,g_stand_entry.stable_ms,
                        g_chassis.inference_us,(board_cycles()-start)/(SystemCoreClock/1000000U));
    if ((previous==CONTROL_ACTIVE || previous==CONTROL_ENABLING) &&
        control.mode!=CONTROL_ACTIVE && control.mode!=CONTROL_ENABLING) {
        telemetry_freeze(); g_trial_trace.frozen=1;
    }
    trial_trace_transfer(control.mode==CONTROL_STOP || control.mode==CONTROL_FAULT);
    spent=(board_cycles()-start)/(SystemCoreClock/1000000U);
    if (spent>g_chassis.max_loop_us) g_chassis.max_loop_us=spent;
    if (spent>=ROBOT_CONTROL_MS*1000U) {
        ++g_chassis.overruns; fault(2); g_chassis.mode=control.mode;
        frame.mode=control.mode; frame.fault=g_chassis.fault; telemetry_publish(&frame); telemetry_freeze();
        g_trial_trace.frozen=1;
    }
    board_watchdog_feed();
}
