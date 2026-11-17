#include "controller.h"
#include "algorithm/model/model.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
    float q[6]={3.554f,-3.645f,0,-4.067f,2.986f,0},v[6]={0};Retraction s;
    retraction_begin(&s,100,q);
    float last[6]={0};
    for(unsigned t=100;t<3600;t+=2) {
        assert(retraction_step(&s,t,q,v,.002f));
        assert(s.target[4]>=q[4] && s.target[4]<=q[4]+RETRACT_DELTA_RAD+1e-6f);
        assert(s.target[3]==q[3]);
        for(unsigned i=0;i<6;++i) {
            assert(fabsf(s.torque[i])<=20.00001f);
            assert(fabsf(s.torque[i]-last[i])<=(t<2600 ? .03001f : .04001f));last[i]=s.torque[i];
            if(i!=3 && i!=4) assert(s.torque[i]==0);
        }
    }
    assert(fabsf(s.torque[4])<.041f);
    assert(!retraction_step(&s,3600,q,v,.002f) && s.reason==1 && s.torque[4]==0);
    retraction_begin(&s,0xfffffff0U,q);assert(retraction_step(&s,0x10,q,v,.002f));
    v[3]=.81f;assert(!retraction_step(&s,0x12,q,v,.002f) && s.reason==3);v[3]=0;
    retraction_begin(&s,0,q);q[3]+=.18f;assert(!retraction_step(&s,2,q,v,.002f) && s.reason==2);q[3]-=.18f;
    q[4]=rl_physical_knee_limits[3]-.01f;retraction_begin(&s,0,q);
    assert(!retraction_step(&s,2,q,v,.002f) && s.reason==4);
    puts("PASS: smooth fold, hip hold, zero right/wheel torque, torque/slew limits, deadline, wrap and motion/geometry guards");
    return 0;
}
