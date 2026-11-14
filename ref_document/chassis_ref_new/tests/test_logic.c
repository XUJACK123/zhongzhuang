#include "algorithm/rl.h"
#include "algorithm/model/model.h"
#include "control_state.h"
#include "motor_io.h"
#include "remote_control.h"
#include "state_estimator.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
int test_logic(void) {
    ControlState s={0};
    /* Only the lowest S1 position is zero current; anything else runs after one DOWN. */
    control_state_step(&s,0,true,1,3,true,true,true); CHECK(s.mode==CONTROL_STOP);
    control_state_step(&s,1,true,2,3,true,true,true); CHECK(s.mode==CONTROL_STOP && s.down_seen);
    control_state_step(&s,2,true,1,3,true,true,false); CHECK(s.mode==CONTROL_ENABLING);
    control_state_step(&s,12,true,1,3,true,true,true); CHECK(s.mode==CONTROL_ACTIVE);
    control_state_step(&s,22,true,3,2,true,true,true); CHECK(s.mode==CONTROL_ACTIVE);
    control_state_step(&s,32,true,2,3,true,true,true); CHECK(s.mode==CONTROL_STOP);
    /* Remote loss zeroes output and requires another DOWN before running again. */
    control_state_step(&s,42,true,1,3,true,true,true); CHECK(s.mode==CONTROL_ENABLING);
    control_state_step(&s,52,false,1,3,true,true,true); CHECK(s.mode==CONTROL_STOP && !s.down_seen);
    control_state_step(&s,62,true,1,3,true,true,true); CHECK(s.mode==CONTROL_STOP);
    control_state_step(&s,72,true,2,3,true,true,true); CHECK(s.mode==CONTROL_STOP && s.down_seen);
    /* Hardware fault latches until DOWN is seen again. */
    control_state_step(&s,74,true,1,3,true,true,false); CHECK(s.mode==CONTROL_ENABLING);
    control_state_step(&s,84,true,1,3,true,false,false); CHECK(s.mode==CONTROL_FAULT);
    control_state_step(&s,94,true,1,3,true,true,true); CHECK(s.mode==CONTROL_FAULT);
    control_state_step(&s,104,true,2,3,true,false,false); CHECK(s.mode==CONTROL_STOP);
    /* Enable never completes -> fault. */
    control_state_step(&s,112,true,1,3,true,true,false); CHECK(s.mode==CONTROL_ENABLING);
    control_state_step(&s,112+ROBOT_ENABLE_TIMEOUT_MS+1,true,1,3,true,true,false); CHECK(s.mode==CONTROL_FAULT);

    uint8_t frame[18]={0};
    /* All channels 1024, s1 down=2, s2 middle=3. */
    frame[1]=4; frame[2]=32; frame[4]=1; frame[5]=0xe8; frame[17]=4;
    RemoteControl rc={0};
    CHECK(remote_decode(&rc,frame,18,100)); CHECK(rc.s1==2 && rc.s2==3);
    for (unsigned i=0;i<5;++i) CHECK(rc.channel[i]==0);
    CHECK(remote_fresh(&rc,200)); CHECK(!remote_fresh(&rc,201));
    CHECK(!remote_decode(&rc,frame,17,102)); CHECK(!remote_fresh(&rc,102));
    frame[5]=8; CHECK(!remote_decode(&rc,frame,18,103));
    rc.channel[1]=-660; rc.channel[0]=660;
    float request[3]; remote_request(&rc,request); CHECK(request[0]==0 && request[1]<-2.49f);

    /* Shared physical bus: DM3 must resolve to right KNEE, DJI3 to right WHEEL. */
    static const float cfg_signs[6]=ROBOT_ENCODER_SIGNS;
    MotorBank bank={0}; uint8_t fb[8]={0x1f,0xfe,0,100,0,0,30,1};
    CHECK(motor_receive(&bank,0x303,fb,8,1)); CHECK(bank.motor[1].raw==8190 && bank.motor[2].received==0);
    CHECK(fabsf(bank.motor[1].velocity-RL_PI/30)<1e-6f);
    fb[0]=0; fb[1]=2;
    float before=bank.motor[1].arm;
    CHECK(motor_receive(&bank,0x303,fb,8,2));
    CHECK(fabsf(bank.motor[1].arm-before-cfg_signs[1]*4*2*RL_PI/8192)<1e-6f);
    CHECK(motor_receive(&bank,0x203,fb,8,2)); CHECK(bank.motor[2].received==1);
    CHECK(!motor_receive(&bank,0x203,fb,7,3));
    fb[0]=0x20; fb[1]=0; CHECK(!motor_receive(&bank,0x303,fb,8,3));
    fb[0]=0; fb[1]=5;
    for (unsigned i=0;i<6;++i) {
        unsigned base=i==2 || i==5 ? 0x200 : 0x300;
        CHECK(motor_receive(&bank,(uint16_t)(base+motor_id[i]),fb,8,4));
    }
    CHECK(motor_feedback_healthy(&bank,34)); CHECK(!motor_feedback_healthy(&bank,35));
    fb[7]=8; CHECK(motor_receive(&bank,0x301,fb,8,5)); CHECK(!motor_feedback_healthy(&bank,5));
    fb[7]=1; CHECK(motor_receive(&bank,0x301,fb,8,100)); CHECK(bank.motor[3].position_lost);

    /* Stay inside the configured caps; the bench caps are lower than training. */
    float leg_cap=ROBOT_LEG_TORQUE_CAP_NM*0.9f, wheel_cap=ROBOT_WHEEL_TORQUE_CAP_NM*0.9f;
    float torque[6]={leg_cap*0.3f,-leg_cap*0.6f,wheel_cap*0.8f,leg_cap,-leg_cap,-wheel_cap*0.5f}; uint8_t dm[8],dji[8];
    CHECK(motor_encode(torque,dm,dji));
    for (unsigned i=0;i<6;++i) {
        bool wheel=i==2 || i==5; unsigned slot=2U*(motor_id[i]-1);
        int16_t raw=wheel ? (int16_t)((uint16_t)dji[slot]<<8|dji[slot+1]) :
                            (int16_t)((uint16_t)dm[slot+1]<<8|dm[slot]);
        float unit=wheel ? 20*ROBOT_WHEEL_KT_NM_A/16384 : ROBOT_DM_KT_NM_A*ROBOT_DM_CURRENT_FULL_SCALE_A/16384;
        CHECK(fabsf(raw*unit*cfg_signs[i]-torque[i])<unit+1e-5f);
    }
    CHECK(dji[0]==0 && dji[1]==0 && dji[6]==0 && dji[7]==0);
    torque[1]=NAN; CHECK(!motor_encode(torque,dm,dji));
    for (unsigned i=0;i<8;++i) CHECK(dm[i]==0 && dji[i]==0);

    /* Virtual work: power must be identical in generalized and motor coordinates. */
    float q[6],qd[6]={0.2f,-0.7f,3,-0.4f,0.8f,2},tau[6]={3,5,1,-4,6,-1};
    memcpy(q,rl_default_q,sizeof(q)); float mt[6],ms[6],qback[6],vback[6],arm[6];
    CHECK(rl_joint_to_motor(q,qd,tau,mt,ms));
    for (unsigned h=0;h<=3;h+=3) {
        float sign=h==0 ? 1 : -1, beta=sign*q[h+1]+RL_PI/2;
        arm[h]=sign*q[h]; arm[h+1]=arm[h]+2*atan2f(sinf(beta),RL_LINK_RATIO+cosf(beta));
        arm[h+2]=q[h+2];
    }
    CHECK(rl_motor_to_joint(arm,ms,qback,vback));
    float jp=0,mp=0;
    for (unsigned i=0;i<6;++i) { CHECK(fabsf(q[i]-qback[i])<1e-5f); CHECK(fabsf(qd[i]-vback[i])<1e-5f); jp+=tau[i]*qd[i]; mp+=mt[i]*ms[i]; }
    CHECK(fabsf(jp-mp)<1e-5f);
    /* Straight no-slip rolling in the archived body frame: both wheel axes +Y. */
    StateEstimator estimator; estimator_reset(&estimator);
    float stationary_speed[6]={0,0,3,0,0,3},identity[4]={1,0,0,0},omega[3]={0};
    RlMeasurement rolling={0};
    CHECK(estimator_update(&estimator,&rolling,identity,arm,stationary_speed,omega,5,0.005f));
    CHECK(fabsf(rolling.velocity[0]-0.165f)<1e-5f && fabsf(rolling.velocity[1])<1e-5f);
    CHECK(fabsf(rolling.xy[0]-0.000825f)<1e-6f);
    CHECK(estimator_update(&estimator,&rolling,identity,arm,stationary_speed,omega,10,0.005f));
    CHECK(rolling.qd[0]==0); /* actual position differences override feedback speed */
    arm[1]=arm[0]; CHECK(!rl_motor_to_joint(arm,ms,qback,vback));

    RlMeasurement m={0}; memcpy(m.q,rl_default_q,sizeof(m.q)); m.gravity[2]=-1;
    RlController c; rl_reset(&c,&m);
    float cmd[3]={0.3f,2.5f,0.1f}; CHECK(rl_policy_step(&c,&m,cmd,0.01f));
    CHECK(fabsf(c.command[0]-0.015f)<1e-6f && c.obs[8]==0.5f);
    CHECK(c.obs[28]==0); CHECK(rl_torque_step(&c,&m));
    CHECK(fabsf(c.motor_torque[0])<=12 && fabsf(c.motor_torque[2])<=2);
    c.target_q[1]=NAN; CHECK(!rl_torque_step(&c,&m));
    rl_reset(&c,&m); m.xy[0]=NAN; CHECK(!rl_policy_step(&c,&m,cmd,0.01f));
    return 0;
}
