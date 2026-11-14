#include "chassis.h"
#include "board.h"
#include "fdcan.h"
#include "imu.h"
#include "state_estimator.h"
#include "algorithm/model/model.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>
ChassisDebug g_chassis;
RlController g_rl;
RlMeasurement g_measurement;
MotorBank g_motor_snapshot;
static MotorBank received;
static StateEstimator estimator;
static ControlState control;
static uint32_t last_loop,last_sample,last_policy,last_special,last_estimator;
static bool have_measurement,shadow_started;
void chassis_receive(uint16_t id, const uint8_t d[8], uint32_t now) {
    (void)motor_receive(&received,id,d,8,now);
}
void chassis_init(void) {
    memset(&g_chassis,0,sizeof(g_chassis)); memset(&g_rl,0,sizeof(g_rl));
    memset(&g_measurement,0,sizeof(g_measurement)); memset(&received,0,sizeof(received));
    memset(&g_motor_snapshot,0,sizeof(g_motor_snapshot)); memset(&control,0,sizeof(control));
    have_measurement=false; shadow_started=false;
    float output[6];
    bool ok=rl_actor(&g_rl.network,rl_selftest_obs,output);
    for (unsigned i=0;i<6;++i) if (fabsf(output[i]-rl_selftest_output[i])>0.001f) ok=false;
    g_chassis.actor_selftest=ok;
    last_loop=last_sample=last_policy=last_special=last_estimator=HAL_GetTick();
    estimator_reset(&estimator);
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
    can2_cancel_pending(); (void)zero_output(); (void)special(0xfd);
}
void chassis_service(uint32_t now) {
    if (now==last_sample) return;
    last_sample=now;
    uint32_t start=board_cycles(),elapsed=now-last_loop;
    RemoteControl rc; board_remote_snapshot(&rc);
    uint32_t mask=__get_PRIMASK(); __disable_irq(); g_motor_snapshot=received; __set_PRIMASK(mask);
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
    float arm[6],speed[6];
    for (unsigned i=0;i<6;++i) { arm[i]=g_motor_snapshot.motor[i].arm; speed[i]=g_motor_snapshot.motor[i].velocity; }
    bool geometry=false;
    if (!(inhibit&(4|8)) && ROBOT_MOTOR_CALIBRATED) {
        if (now-last_estimator>=5) {
            geometry=estimator_update(&estimator,&g_measurement,g_imu.quaternion,arm,speed,g_imu.omega,
                                      now,(float)(now-last_estimator)*0.001f);
            last_estimator=now; have_measurement=geometry;
        } else {
            float unused[6];
            geometry=have_measurement && rl_motor_to_joint(arm,speed,g_measurement.q,unused);
        }
    } else { have_measurement=false; estimator.velocity_valid=false; last_estimator=now; }
    if (!geometry) inhibit|=32;
    if (!isfinite(g_imu.gravity[2]) || g_imu.gravity[2]>-cosf(ROBOT_MAX_TILT_RAD)) inhibit|=64;
    if (geometry) {
        if (g_measurement.q[1]<rl_knee_limits[0]-0.02f || g_measurement.q[1]>rl_knee_limits[1]+0.02f ||
            g_measurement.q[4]<rl_knee_limits[2]-0.02f || g_measurement.q[4]>rl_knee_limits[3]+0.02f) inhibit|=32;
        for (unsigned i=0;i<6;++i) if (fabsf(g_measurement.qd[i])>(i==2 || i==5 ? 75 : 35)) inhibit|=32;
    }
    g_chassis.inhibit=inhibit;
    if (elapsed<ROBOT_CONTROL_MS) return;
    last_loop=now; g_chassis.tick=now; ++g_chassis.cycles;
    ControlMode previous=control.mode;
    control_state_step(&control,now,remote_ok,rc.s1,rc.s2,remote_sticks_centered(&rc),
                       inhibit==0,motor_all_enabled(&g_motor_snapshot));
    /* Send one enable frame on entry so group-mode motors see it immediately. */
    if (previous!=CONTROL_ENABLING && control.mode==CONTROL_ENABLING) last_special=now-20;
    if (remote_ok && rc.s1==2 && rc.s2==3) g_chassis.fault=0;
    if (elapsed>ROBOT_CONTROL_MS+1 && (previous==CONTROL_ACTIVE || previous==CONTROL_ENABLING)) {
        ++g_chassis.overruns; fault(2);
    }
    if ((previous==CONTROL_ACTIVE || previous==CONTROL_ENABLING) &&
        control.mode!=CONTROL_ACTIVE && control.mode!=CONTROL_ENABLING) can2_cancel_pending();
    /* Shadow mode executes the real pipeline only with calibrated, fresh measurements.
     * Config flags still prevent enabling motors. No fabricated IMU/joint data. */
    bool measurable=(inhibit&(4|8|16|32|64|128))==0;
    if (measurable && (control.mode==CONTROL_ACTIVE || !ROBOT_ENABLE_OUTPUT)) {
        if (!shadow_started || (control.mode==CONTROL_ACTIVE && previous!=CONTROL_ACTIVE)) {
            rl_reset(&g_rl,&g_measurement); last_policy=now-ROBOT_POLICY_MS; shadow_started=true;
        }
        if (now-last_policy>=ROBOT_POLICY_MS) {
            float request[3]={0,0,ROBOT_HEIGHT_COMMAND_M};
            if (control.mode==CONTROL_ACTIVE) remote_request(&rc,request);
            uint32_t inference_start=board_cycles();
            bool ok=rl_policy_step(&g_rl,&g_measurement,request,0.01f);
            g_chassis.inference_us=(board_cycles()-inference_start)/(SystemCoreClock/1000000U);
            last_policy=now; ++g_chassis.policy_runs;
            if (!ok) fault(3);
        }
        if (!rl_torque_step(&g_rl,&g_measurement)) fault(4);
    } else shadow_started=false;
    /* Check elapsed wall time and remote AGAIN after inference, before any torque TX. */
    uint32_t spent=(board_cycles()-start)/(SystemCoreClock/1000000U);
    if (spent>g_chassis.max_loop_us) g_chassis.max_loop_us=spent;
    if (spent>=ROBOT_CONTROL_MS*1000U) { ++g_chassis.overruns; fault(2); }
    RemoteControl final_rc; board_remote_snapshot(&final_rc);
    if (control.mode==CONTROL_ACTIVE && (!remote_fresh(&final_rc,HAL_GetTick()) || final_rc.s1==2)) {
        control.mode=CONTROL_STOP; control.down_seen=false; can2_cancel_pending();
    }
    if (control.mode==CONTROL_ACTIVE) {
        uint8_t dm[8],dji[8];
        if (!motor_encode(g_rl.motor_torque,dm,dji) || !can2_send(0x3fe,dm) || !can2_send(0x200,dji)) fault(1);
    } else {
        if (!zero_output()) fault(1);
        if (now-last_special>=20) {
            last_special=now;
            if (!special(control.mode==CONTROL_ENABLING ? 0xfc : 0xfd)) fault(1);
        }
    }
    g_chassis.mode=control.mode;
    board_watchdog_feed();
}
