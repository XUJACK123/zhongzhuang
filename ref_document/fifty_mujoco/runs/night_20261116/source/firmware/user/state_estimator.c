/* Wheel no-slip odometry. Uses archived URDF wheel-center FK and compensates
 * moving legs/body rotation. Slip, flight and absolute XY/yaw are unobservable;
 * requires independent validation before using reference tracking on hardware. */
#include "state_estimator.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>
static void cross(const float a[3], const float b[3], float c[3]) {
    c[0]=a[1]*b[2]-a[2]*b[1]; c[1]=a[2]*b[0]-a[0]*b[2]; c[2]=a[0]*b[1]-a[1]*b[0];
}
static void rotate(const float r[3][3], const float a[3], float b[3]) {
    for (unsigned i=0;i<3;++i) { b[i]=0; for (unsigned j=0;j<3;++j) b[i]+=r[i][j]*a[j]; }
}
void estimator_reset(StateEstimator *s) { memset(s,0,sizeof(*s)); }
static bool update(StateEstimator *s, EstimatorQuality *quality, RlMeasurement *m, const float quat[4],
                      const float arm[6], const float speed[6], const float omega[3],
                      const float acceleration[3], const uint32_t feedback_tick[6], uint32_t now, float dt) {
    float qd[6];
    if (!rl_motor_to_joint(arm,speed,m->q,qd) || dt<=0 || dt>0.02f) return false;
    /* Training requests position-difference velocity at the 5ms physics rate.
     * Differentiate unwrapped encoder coordinates, hips use shortest increment. */
    if (!s->velocity_valid) {
        memcpy(s->last_q,m->q,sizeof(s->last_q)); memcpy(s->held_qd,qd,sizeof(qd));
        s->velocity_tick=now; s->velocity_valid=true;
    } else if (now-s->velocity_tick>=5) {
        float inv_dt=1000.0f/(float)(now-s->velocity_tick);
        for (unsigned i=0;i<6;++i) {
            float d=m->q[i]-s->last_q[i]; if (i==0 || i==3) d=rl_wrap(d);
            s->held_qd[i]=d*inv_dt; s->last_q[i]=m->q[i];
        }
        s->velocity_tick=now;
    }
    memcpy(m->qd,s->held_qd,sizeof(m->qd));
    float w=quat[0],x=quat[1],y=quat[2],z=quat[3];
    float quaternion_norm=w*w+x*x+y*y+z*z;
    if (!isfinite(quaternion_norm) || fabsf(quaternion_norm-1)>0.02f) return false;
    float r[3][3]={{1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)},
                   {2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)},
                   {2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)}};
    float axle[3]={r[0][1],r[1][1],r[2][1]}, contact[3];
    /* Direction from wheel center to contact, projected perpendicular to axle. */
    float norm=sqrtf(1-axle[2]*axle[2]); if (!(norm>0.5f)) return false;
    contact[0]=ROBOT_WHEEL_RADIUS_M*axle[2]*axle[0]/norm;
    contact[1]=ROBOT_WHEEL_RADIUS_M*axle[2]*axle[1]/norm;
    contact[2]=-ROBOT_WHEEL_RADIUS_M*(1-axle[2]*axle[2])/norm;
    float world_velocity[3]={0};
    float per_wheel[2][3];
    for (unsigned h=0;h<=3;h+=3) {
        unsigned k=h+1; float sign=h==0 ? 1.0f : -1.0f;
        float a=sign*m->q[h], b=sign*(m->q[h]+m->q[k])+RL_PI*0.5f;
        float ad=sign*qd[h],bd=sign*(qd[h]+qd[k]);
        float p[3]={0.21f*cosf(a)+0.25f*cosf(b),h==0 ? -0.260797767f : 0.268202233f,
                    -0.21f*sinf(a)-0.25f*sinf(b)};
        float pd[3]={-0.21f*sinf(a)*ad-0.25f*sinf(b)*bd,0,-0.21f*cosf(a)*ad-0.25f*cosf(b)*bd};
        float body_rotation[3],relative[3],relative_world[3]; cross(omega,p,body_rotation);
        for (unsigned i=0;i<3;++i) relative[i]=body_rotation[i]+pd[i];
        rotate(r,relative,relative_world);
        float spin_body[3]={omega[0],omega[1]+bd+qd[h+2],omega[2]}, spin_world[3],surface[3];
        rotate(r,spin_body,spin_world); cross(spin_world,contact,surface);
        for (unsigned i=0;i<3;++i) {
            per_wheel[h/3][i]=-surface[i]-relative_world[i];
            world_velocity[i]+=0.5f*per_wheel[h/3][i];
        }
    }
    for (unsigned i=0;i<3;++i) if (!isfinite(world_velocity[i]) || fabsf(world_velocity[i])>5) return false;
    bool integrate=true;
    if (quality) {
        float predicted[3],world_acceleration[3]; rotate(r,acceleration,world_acceleration);
        world_acceleration[2]-=9.81f;
        for (unsigned i=0;i<3;++i) {
            if (!isfinite(world_acceleration[i])) return false;
            predicted[i]=quality->predicted_world[i]+rl_clip(world_acceleration[i],-10,10)*dt;
        }
        quality->residual=0;
        for (unsigned i=0;i<3;++i) {
            float difference=per_wheel[0][i]-per_wheel[1][i];
            quality->residual+=difference*difference;
        }
        quality->residual=sqrtf(quality->residual);
        for (unsigned side=0;side<2;++side) {
            uint32_t age=0;
            for (unsigned j=0;j<3;++j) {
                unsigned index=side*3+j; quality->age_ms[index]=now-feedback_tick[index];
                if (quality->age_ms[index]>age) age=quality->age_ms[index];
            }
            float innovation=0;
            for (unsigned i=0;i<3;++i) {
                float difference=per_wheel[side][i]-predicted[i]; innovation+=difference*difference;
                quality->wheel_velocity_world[side][i]=per_wheel[side][i];
            }
            float freshness=rl_clip(1-(float)age/ROBOT_FEEDBACK_TIMEOUT_MS,0,1);
            float scale=ROBOT_ODOMETRY_RESIDUAL_M_S;
            quality->weight[side]=freshness/(1+innovation/(scale*scale));
        }
        bool agree=quality->residual<=ROBOT_ODOMETRY_RESIDUAL_M_S;
        float sum=quality->weight[0]+quality->weight[1];
        quality->confidence=agree ? fminf(1,sum) : (quality->initialized ? 0.5f*fmaxf(quality->weight[0],quality->weight[1]) : 0);
        bool trusted=quality->confidence>=0.25f && sum>0.001f;
        quality->reference_realign|=trusted && quality->degraded_ms>0;
        if (trusted) {
            for (unsigned i=0;i<3;++i) world_velocity[i]=(quality->weight[0]*per_wheel[0][i]+quality->weight[1]*per_wheel[1][i])/sum;
            quality->degraded_ms=0;
        } else {
            quality->degraded_ms+=(uint32_t)(dt*1000+0.5f);
            integrate=false;
            for (unsigned i=0;i<3;++i) world_velocity[i]=quality->initialized && quality->degraded_ms<=100 ? predicted[i] : 0;
        }
        memcpy(quality->predicted_world,world_velocity,sizeof(world_velocity)); quality->initialized=true;
    }
    if (integrate) { s->xy[0]+=world_velocity[0]*dt; s->xy[1]+=world_velocity[1]*dt; }
    memcpy(m->xy,s->xy,sizeof(m->xy));
    for (unsigned i=0;i<2;++i) {
        m->velocity[i]=0; for (unsigned j=0;j<3;++j) m->velocity[i]+=r[j][i]*world_velocity[j];
    }
    return true;
}
bool estimator_update(StateEstimator *s, RlMeasurement *m, const float quat[4],
    const float arm[6], const float speed[6], const float omega[3], uint32_t now, float dt) {
    return update(s,0,m,quat,arm,speed,omega,0,0,now,dt);
}
bool estimator_update_qualified(StateEstimator *s, EstimatorQuality *quality, RlMeasurement *m,
    const float quat[4], const float arm[6], const float speed[6], const float omega[3],
    const float acceleration[3], const uint32_t feedback_tick[6], uint32_t now, float dt) {
    return update(s,quality,m,quat,arm,speed,omega,acceleration,feedback_tick,now,dt);
}
