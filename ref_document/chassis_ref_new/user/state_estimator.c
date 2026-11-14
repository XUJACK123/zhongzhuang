/* Wheel no-slip odometry. Uses archived URDF wheel-center FK and compensates
 * moving legs/body rotation. Slip, flight and absolute XY/yaw are unobservable;
 * requires independent validation before using reference tracking on hardware. */
#include "state_estimator.h"
#include <math.h>
#include <string.h>
static void cross(const float a[3], const float b[3], float c[3]) {
    c[0]=a[1]*b[2]-a[2]*b[1]; c[1]=a[2]*b[0]-a[0]*b[2]; c[2]=a[0]*b[1]-a[1]*b[0];
}
static void rotate(const float r[3][3], const float a[3], float b[3]) {
    for (unsigned i=0;i<3;++i) { b[i]=0; for (unsigned j=0;j<3;++j) b[i]+=r[i][j]*a[j]; }
}
void estimator_reset(StateEstimator *s) { memset(s,0,sizeof(*s)); }
bool estimator_update(StateEstimator *s, RlMeasurement *m, const float quat[4],
                      const float arm[6], const float speed[6], const float omega[3],
                      uint32_t now, float dt) {
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
    float r[3][3]={{1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)},
                   {2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)},
                   {2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)}};
    float axle[3]={r[0][1],r[1][1],r[2][1]}, contact[3];
    /* Direction from wheel center to contact, projected perpendicular to axle. */
    float norm=sqrtf(1-axle[2]*axle[2]); if (!(norm>0.5f)) return false;
    contact[0]=0.055f*axle[2]*axle[0]/norm;
    contact[1]=0.055f*axle[2]*axle[1]/norm;
    contact[2]=-0.055f*(1-axle[2]*axle[2])/norm;
    float world_velocity[3]={0};
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
        for (unsigned i=0;i<3;++i) world_velocity[i]+=0.5f*(-surface[i]-relative_world[i]);
    }
    for (unsigned i=0;i<3;++i) if (!isfinite(world_velocity[i]) || fabsf(world_velocity[i])>5) return false;
    s->xy[0]+=world_velocity[0]*dt; s->xy[1]+=world_velocity[1]*dt;
    memcpy(m->xy,s->xy,sizeof(m->xy));
    for (unsigned i=0;i<2;++i) {
        m->velocity[i]=0; for (unsigned j=0;j<3;++j) m->velocity[i]+=r[j][i]*world_velocity[j];
    }
    return true;
}
