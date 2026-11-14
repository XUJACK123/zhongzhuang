/* Archived sample_actor31 mathematical contract and actuator_model.py.
 * No history encoder, observation normalizer or stochastic sampling. */
#include "rl.h"
#include "model/model.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>

float rl_clip(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
float rl_wrap(float x) { return atan2f(sinf(x), cosf(x)); }
static bool finite_array(const float *a, unsigned n) {
    for (unsigned i=0; i<n; ++i) if (!isfinite(a[i])) return false;
    return true;
}
static bool finite_measurement(const RlMeasurement *m) {
    return finite_array(m->q,6) && finite_array(m->qd,6) && finite_array(m->omega,3) &&
           finite_array(m->gravity,3) && finite_array(m->xy,2) &&
           isfinite(m->yaw) && finite_array(m->velocity,2);
}
static bool dense(const float *x, unsigned in, float *y, unsigned out,
                  const float *w, const float *b, bool elu) {
    for (unsigned o=0; o<out; ++o) {
        float v=b[o];
        for (unsigned i=0; i<in; ++i) v += w[o*in+i]*x[i];
        if (!isfinite(v)) return false;
        y[o]=(elu && v<0) ? expm1f(v) : v;
    }
    return true;
}
bool rl_actor(RlNetwork *n, const float obs[31], float action[6]) {
    if (!finite_array(obs,31) ||
        !dense(obs,31,n->hidden1,128,rl_w0,rl_b0,true) ||
        !dense(n->hidden1,128,n->hidden2,64,rl_w1,rl_b1,true) ||
        !dense(n->hidden2,64,n->hidden3,32,rl_w2,rl_b2,true) ||
        !dense(n->hidden3,32,action,6,rl_w3,rl_b3,false)) {
        memset(action,0,6*sizeof(float)); return false;
    }
    return true;
}
static float knee_ratio(float alpha) {
    const float r=RL_LINK_RATIO, s=sinf(alpha*0.5f);
    return 0.5f+r*cosf(alpha*0.5f)/(2*sqrtf(1-r*r*s*s));
}
bool rl_motor_to_joint(const float arm[6], const float speed[6], float q[6], float qd[6]) {
    if (!finite_array(arm,6) || !finite_array(speed,6)) return false;
    memcpy(q,arm,6*sizeof(float)); memcpy(qd,speed,6*sizeof(float));
    for (unsigned h=0; h<=3; h+=3) {
        unsigned k=h+1; float sign=h==0 ? 1.0f : -1.0f;
        float alpha=arm[k]-arm[h]; /* branch must be established by absolute calibration */
        if (!(alpha > -2*RL_PI+0.001f && alpha < -0.001f)) return false;
        float beta=alpha*0.5f+asinf(RL_LINK_RATIO*sinf(alpha*0.5f));
        float ratio=knee_ratio(alpha);
        q[h]=sign*arm[h]; q[k]=sign*(beta-RL_PI*0.5f);
        qd[h]=sign*speed[h]; qd[k]=sign*ratio*(speed[k]-speed[h]);
    }
    return true;
}
bool rl_joint_to_motor(const float q[6], const float qd[6], const float tau[6],
                       float motor[6], float speed[6]) {
    if (!finite_array(q,6) || !finite_array(qd,6) || !finite_array(tau,6)) return false;
    memcpy(motor,tau,6*sizeof(float)); memcpy(speed,qd,6*sizeof(float));
    for (unsigned h=0; h<=3; h+=3) {
        unsigned k=h+1; float sign=h==0 ? 1.0f : -1.0f;
        float beta=sign*q[k]+RL_PI*0.5f;
        float alpha=2*atan2f(sinf(beta),RL_LINK_RATIO+cosf(beta));
        if (!(alpha > -2*RL_PI+0.001f && alpha < -0.001f)) return false;
        float r=knee_ratio(alpha);
        if (!(r>0.05f && r<1.0f)) return false;
        motor[h]=sign*(tau[h]-r*tau[k]); motor[k]=sign*r*tau[k];
        speed[h]=sign*qd[h]; speed[k]=sign*(qd[h]+qd[k]/r);
    }
    return finite_array(motor,6) && finite_array(speed,6);
}
void rl_reset(RlController *c, const RlMeasurement *m) {
    memset(c,0,sizeof(*c));
    c->command[2]=ROBOT_HEIGHT_COMMAND_M;
    memcpy(c->reference_xy,m->xy,sizeof(c->reference_xy)); c->reference_yaw=m->yaw;
    memcpy(c->target_q,rl_default_q,sizeof(c->target_q));
}
bool rl_policy_step(RlController *c, const RlMeasurement *m, const float request[3], float dt) {
    static const float rate[3]={1.5f,6.28318530718f,0.12f};
    if (!isfinite(dt) || dt<=0 || dt>0.02f || !finite_array(request,3) ||
        !finite_measurement(m) || !finite_array(c->reference_xy,2) || !isfinite(c->reference_yaw) ||
        !finite_array(c->command,3) || !finite_array(c->action,6)) return false;
    for (unsigned i=0;i<3;++i) c->command[i] += rl_clip(request[i]-c->command[i],-rate[i]*dt,rate[i]*dt);
    float *obs=c->obs;
    for (unsigned i=0;i<3;++i) { obs[i]=m->omega[i]*0.25f; obs[3+i]=m->gravity[i]; }
    obs[6]=c->command[0]*2; obs[7]=c->command[1]*0.25f; obs[8]=c->command[2]*5;
    obs[9]=rl_wrap(m->q[0]-rl_default_q[0]); obs[10]=m->q[1]-rl_default_q[1];
    obs[11]=rl_wrap(m->q[3]-rl_default_q[3]); obs[12]=m->q[4]-rl_default_q[4];
    for (unsigned i=0;i<6;++i) { obs[13+i]=m->qd[i]*0.05f; obs[19+i]=c->action[i]; }
    float co=cosf(m->yaw), si=sinf(m->yaw);
    float dx=c->reference_xy[0]-m->xy[0], dy=c->reference_xy[1]-m->xy[1];
    float err=c->reference_yaw-m->yaw;
    obs[25]=rl_clip(5*(co*dx+si*dy),-2,2); obs[26]=rl_clip(5*(-si*dx+co*dy),-2,2);
    obs[27]=sinf(err); obs[28]=cosf(err)-1; obs[29]=2*m->velocity[0]; obs[30]=2*m->velocity[1];
    /* Reject NaN before clipping (C fmin/fmax would otherwise hide it). */
    if (!finite_array(obs,31)) return false;
    for (unsigned i=0;i<31;++i) obs[i]=rl_clip(obs[i],-100,100);
    if (!rl_actor(&c->network,obs,c->action)) return false;
    for (unsigned i=0;i<6;++i) {
        bool wheel=i==2 || i==5;
        c->action[i]=rl_clip(c->action[i],wheel ? -2 : -1,wheel ? 2 : 1);
        c->target_q[i]=rl_default_q[i]+(wheel ? 0 : 0.5f*c->action[i]);
        c->target_qd[i]=wheel ? 20*c->action[i] : 0;
    }
    c->target_q[1]=rl_clip(c->target_q[1],rl_knee_limits[0],rl_knee_limits[1]);
    c->target_q[4]=rl_clip(c->target_q[4],rl_knee_limits[2],rl_knee_limits[3]);
    /* Advance after observation; reset aligns reference to measured root.
     * Source base legged_robot.py timing is still pending golden-trace parity. */
    float old=c->reference_yaw, half=c->command[1]*dt*0.5f;
    float sinc=fabsf(half)<1e-6f ? 1 : sinf(half)/half;
    float next=rl_wrap(old+2*half), distance=c->command[0]*dt*sinc;
    float dc=cosf(old)-cosf(next), ds=sinf(old)-sinf(next);
    c->reference_xy[0]+=distance*cosf(old+half)+dc*RL_REFERENCE_X-ds*RL_REFERENCE_Y;
    c->reference_xy[1]+=distance*sinf(old+half)+ds*RL_REFERENCE_X+dc*RL_REFERENCE_Y;
    c->reference_yaw=next;
    return true;
}
bool rl_torque_step(RlController *c, const RlMeasurement *m) {
    if (!finite_measurement(m) || !finite_array(c->target_q,6) || !finite_array(c->target_qd,6)) return false;
    for (unsigned i=0;i<6;++i) {
        float error=c->target_q[i]-m->q[i];
        if (i==0 || i==3) error=rl_wrap(error);
        c->joint_torque[i]=rl_clip(rl_kp[i]*error+rl_kd[i]*(c->target_qd[i]-m->qd[i]),
                                  -rl_joint_effort[i],rl_joint_effort[i]);
    }
    float speed[6];
    if (!rl_joint_to_motor(m->q,m->qd,c->joint_torque,c->motor_torque,speed)) return false;
    for (unsigned i=0;i<6;++i) {
        bool wheel=i==2 || i==5;
        float rated=wheel ? RL_WHEEL_RATED_SPEED : RL_LEG_RATED_SPEED;
        float free_speed=wheel ? RL_WHEEL_FREE_SPEED : RL_LEG_FREE_SPEED;
        float cap=wheel ? fminf(RL_WHEEL_RATED_TORQUE,ROBOT_WHEEL_TORQUE_CAP_NM) :
                         fminf(RL_LEG_RATED_TORQUE,ROBOT_LEG_TORQUE_CAP_NM);
        float positive=cap*rl_clip((free_speed-speed[i])/(free_speed-rated),0,1);
        float negative=-cap*rl_clip((free_speed+speed[i])/(free_speed-rated),0,1);
        c->motor_torque[i]=rl_clip(c->motor_torque[i],negative,positive);
    }
    return finite_array(c->motor_torque,6);
}
