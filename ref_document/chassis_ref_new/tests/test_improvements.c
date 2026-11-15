#include "algorithm/rl.h"
#include "algorithm/model/model.h"
#include "motor_io.h"
#include "robot_config.h"
#include "state_estimator.h"
#include "startup_control.h"
#include "imu_filter.h"
#include "telemetry.h"
#include <math.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
int test_improvements(void) {
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
    startup_step(&startup,16514,true,false,true,true); CHECK(startup.failure==STARTUP_TIMEOUT);

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
