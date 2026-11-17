#include "algorithm/rl.h"
#include "algorithm/model/model.h"
#include "motor_io.h"
#include "robot_config.h"
#include "state_estimator.h"
#include "startup_control.h"
#include "imu_filter.h"
#include "telemetry.h"
#include "supported_trial.h"
#include "trial_trace.h"
#include <math.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
int test_improvements(void) {
    memset((void *)&g_supported_session,0,sizeof(g_supported_session));
    CHECK(!supported_session_step(1,true,2,true,true,true,false,false,true));
    CHECK(!supported_session_step(2,true,3,true,true,true,false,false,true));
    g_supported_session.request=1;
    CHECK(supported_session_step(3,true,3,true,true,true,false,false,true));
    CHECK(!supported_session_step(100,true,3,true,true,false,true,false,false));
    g_supported_session.request=2;
    CHECK(!supported_session_step(200,true,3,true,true,false,true,false,true) && g_supported_session.status==2);
    CHECK(!supported_session_step(2200,true,3,true,true,false,true,false,true));
    g_supported_session.request=3;
    CHECK(!supported_session_step(2201,true,3,true,true,false,true,false,false) && g_supported_session.status==1);
    g_supported_session.request=4;
    CHECK(supported_session_step(2202,true,3,true,true,false,true,false,true));
    CHECK(!supported_session_step(2203,false,3,true,true,false,false,false,true));
    g_supported_session.request=5;
    CHECK(!supported_session_step(5000,true,3,true,true,true,false,false,true));
    CHECK(!g_supported_session.down_seen && g_supported_session.accepted==2);
    supported_session_step(5001,true,2,true,true,true,false,false,true);
    g_supported_session.request=6; g_supported_session.requested_wheel_cap=ROBOT_WHEEL_TORQUE_CAP_NM+0.1f;
    CHECK(!supported_session_step(5002,true,3,true,true,true,false,false,true) && g_supported_session.status==4);
    g_supported_session.request=7; g_supported_session.requested_wheel_cap=0.2f;
    CHECK(supported_session_step(5003,true,3,true,true,true,false,false,true));
    g_supported_session.requested_wheel_cap=ROBOT_WHEEL_TORQUE_CAP_NM;
    CHECK(supported_session_step(5004,true,3,true,true,false,false,false,true));
    CHECK(g_supported_session.applied_wheel_cap==0.2f); /* only new accepted token changes trial parameters */
    if (ROBOT_SUPPORTED_TRIAL_MS) {
        float q0[6]={0},v0[6]={0};
        memset(&g_supported_trial,0,sizeof(g_supported_trial));
        CHECK(!supported_trial_stop(0xfffffff0U,true,q0,v0));
        CHECK(!supported_trial_stop(0x10U,true,q0,v0));
        v0[2]=6.01f; CHECK(supported_trial_stop(0x12U,true,q0,v0));
        CHECK(g_supported_trial.reason==4);
        CHECK(!supported_trial_stop(0x14U,false,q0,v0));
        v0[2]=0; CHECK(!supported_trial_stop(0x16U,true,q0,v0));
        q0[5]=0.71f; CHECK(supported_trial_stop(0x18U,true,q0,v0));
        CHECK(g_supported_trial.reason==3);
        supported_trial_stop(0x20U,false,q0,v0);
        q0[0]=NAN; CHECK(supported_trial_stop(0x22U,true,q0,v0));
        CHECK(g_supported_trial.reason==5);
    }
    trial_trace_reset(); TelemetryFrame source={0}; float trace_gravity[3]={0,0,-1};
    trial_trace_entry(32|512);
    for (unsigned i=0;i<520;++i) { source.tick=i*2; trial_trace_publish(&source,trace_gravity,10,20,30); }
    CHECK(g_trial_trace.published==520 && g_trial_trace.overwritten==8);
    CHECK(g_trial_trace.frame[7].begin==1040 && g_trial_trace.frame[7].end==1040);
    CHECK(g_trial_trace.frame[7].entry_block==(32|512) && g_trial_trace.entry_counts[5]==1);
    g_trial_trace.frozen=1; trial_trace_publish(&source,trace_gravity,0,0,0);
    CHECK(g_trial_trace.published==520);
    trial_trace_boot(0x04000000U); CHECK(g_trial_trace.frozen && g_trial_trace.published==520);
    trial_trace_boot(0x00a00000U); CHECK(!g_trial_trace.frozen && g_trial_trace.published==0);
    g_trace_transfer.command=1; g_trace_transfer.request=1;
    trial_trace_transfer(false); CHECK(g_trace_transfer.status==3 && !g_trial_trace.frozen);
    g_trace_transfer.request=2; trial_trace_transfer(true); CHECK(!g_trace_transfer.status && g_trial_trace.frozen);
    g_trace_transfer.command=2; g_trace_transfer.offset=0; g_trace_transfer.length=68; g_trace_transfer.request=3;
    trial_trace_transfer(true); CHECK(!g_trace_transfer.status && !memcmp((const void *)g_trace_transfer.data,(const void *)&g_trial_trace,68));
    g_trace_transfer.offset=0xffffffffU; g_trace_transfer.request=4;
    trial_trace_transfer(true); CHECK(g_trace_transfer.status==1);
    g_trace_transfer.command=3; g_trace_transfer.request=5; trial_trace_transfer(true);
    CHECK(!g_trace_transfer.status && !g_trial_trace.frozen);
    /* 9000 rotor rpm advances 6144 counts in 5ms, beyond half a turn. */
    MotorBank bank={0};
    uint8_t frame[8]={0,100,0x23,0x28,0,0,25,1};
    CHECK(motor_receive(&bank,0x203,frame,8,10));
    int64_t first=bank.motor[2].counts;
    unsigned raw=(100+6144)%8192; frame[0]=(uint8_t)(raw>>8); frame[1]=(uint8_t)raw;
    CHECK(motor_receive(&bank,0x203,frame,8,15));
    CHECK(bank.motor[2].counts-first==6144 && !bank.motor[2].position_lost);
    CHECK(g_motor_diagnostics.predicted_wraps[2]>0);
    frame[0]=0; frame[1]=100;
    CHECK(motor_receive(&bank,0x203,frame,8,20));
    CHECK(bank.motor[2].position_lost); /* impossible prediction residual */

    ImuFusionState fusion={0}; float up[3]={0,0,1},gravity[3]={0,0,9.81f};
    CHECK(imu_acceleration_weight(&fusion,gravity,up,0.001f)>0.99f);
    float acceleration[3]={1,0,9.81f};
    (void)imu_acceleration_weight(&fusion,acceleration,up,0.001f);
    CHECK(imu_acceleration_weight(&fusion,acceleration,up,0.001f)<0.25f);
    float bias[3]={0},gyro[3]={0,0,0.01f};
    fusion.stationary_allowed=false; fusion.stationary_seconds=3;
    imu_track_bias(&fusion,gyro,bias,9.81f,0.001f); CHECK(bias[2]==0);
    fusion.stationary_allowed=true; fusion.stationary_seconds=3; fusion.jerk=0;
    imu_track_bias(&fusion,gyro,bias,9.81f,0.001f); CHECK(bias[2]>0 && bias[2]<0.01f);

    StateEstimator estimator; estimator_reset(&estimator); EstimatorQuality quality={0};
    RlMeasurement measurement={0}; float arm[6],speed[6]={0},quat[4]={1,0,0,0},omega[3]={0};
    uint32_t ticks[6]={5,5,5,5,5,5};
    for (unsigned h=0;h<=3;h+=3) {
        float sign=h==0 ? 1 : -1,beta=sign*rl_default_q[h+1]+RL_PI/2;
        arm[h]=sign*rl_default_q[h]; arm[h+1]=arm[h]+2*atan2f(sinf(beta),RL_LINK_RATIO+cosf(beta)); arm[h+2]=0;
    }
    CHECK(estimator_update_qualified(&estimator,&quality,&measurement,quat,arm,speed,omega,gravity,ticks,5,.005f));
    CHECK(quality.confidence>.99f);
    speed[2]=40; speed[5]=-40;
    for (unsigned i=0;i<6;++i) ticks[i]=10;
    CHECK(estimator_update_qualified(&estimator,&quality,&measurement,quat,arm,speed,omega,gravity,ticks,10,.005f));
    CHECK(quality.confidence<.25f && quality.degraded_ms>0 && measurement.xy[0]==0);
    speed[2]=speed[5]=0;
    for (unsigned i=0;i<6;++i) ticks[i]=15;
    CHECK(estimator_update_qualified(&estimator,&quality,&measurement,quat,arm,speed,omega,gravity,ticks,15,.005f));
    CHECK(quality.reference_realign && quality.degraded_ms==0);
    quat[0]=2; CHECK(!estimator_update_qualified(&estimator,&quality,&measurement,quat,arm,speed,omega,gravity,ticks,20,.005f));

    StartupControl startup={0};
    startup_step(&startup,100,true,false,true,false); CHECK(startup.failure==STARTUP_MODEL_MISSING && startup.torque_scale==0);
    startup_step(&startup,102,false,false,true,false);
    startup_step(&startup,104,true,false,true,true); CHECK(startup.phase==STARTUP_GETUP && startup.torque_scale==0);
    startup_step(&startup,204,true,true,true,true); CHECK(startup.torque_scale==0.5f);
    startup_step(&startup,1204,true,true,true,true); CHECK(startup.phase==STARTUP_HANDOFF && startup.balance_weight==0);
    startup_step(&startup,1354,true,true,true,true); CHECK(fabsf(startup.balance_weight-.5f)<1e-6f);
    startup_step(&startup,1504,true,true,true,true); CHECK(startup.phase==STARTUP_BALANCE);
    startup_step(&startup,11504,true,true,true,true); CHECK(startup.completed);
    startup_step(&startup,11506,true,true,false,true); CHECK(startup.phase==STARTUP_FAILED && startup.torque_scale==0);
    startup_step(&startup,11508,true,true,true,true); CHECK(startup.phase==STARTUP_FAILED);
    startup_step(&startup,11510,false,true,true,true);
    startup_step(&startup,11512,true,false,true,true);
    startup_step(&startup,11512+RL_PROFILE_GETUP_TIMEOUT_MS,true,false,true,true); CHECK(startup.phase==STARTUP_GETUP);
    startup_step(&startup,11513+RL_PROFILE_GETUP_TIMEOUT_MS,true,false,true,true); CHECK(startup.failure==STARTUP_TIMEOUT);

    RlController controller; rl_reset(&controller,&measurement);
    controller.reference_xy[0]=2; controller.reference_xy[1]=0;
    rl_manage_reference(&controller,&measurement,true,false);
    CHECK(fabsf(controller.reference_xy[0]-.4f)<1e-6f);
    controller.command[0]=.1f; rl_manage_reference(&controller,&measurement,false,false);
    CHECK(controller.reference_xy[0]==measurement.xy[0] && controller.command[0]==0);
    float action[6]={-1,-1,2,1,1,-2}; CHECK(rl_set_action(&controller,action));
    CHECK(controller.target_q[1]==rl_knee_limits[0] && controller.target_q[1]>rl_physical_knee_limits[0]);

    telemetry_reset(); TelemetryFrame entry={0};
    for (unsigned i=0;i<40;++i) { entry.tick=i; telemetry_publish(&entry); }
    CHECK(g_telemetry.published==40 && g_telemetry.overwritten==8);
    CHECK(g_telemetry.frame[7].begin==80 && g_telemetry.frame[7].end==80 && g_telemetry.frame[7].tick==39);
    telemetry_freeze(); telemetry_publish(&entry);
    CHECK(g_telemetry.frozen && g_telemetry.published==40);
    telemetry_reset(); CHECK(!g_telemetry.frozen && g_telemetry.published==0);
    return 0;
}
