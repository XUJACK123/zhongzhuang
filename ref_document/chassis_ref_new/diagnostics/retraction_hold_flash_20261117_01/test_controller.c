#include "controller.h"
#include "algorithm/model/model.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
    float q[6]={3.392f,-3.389f,0,-3.649f,3.004f,0},v[6]={0};Retraction s;
    retraction_begin(&s,100,q);
    float last[6]={0};
    for(unsigned t=100;t<100+RETRACT_TOTAL_MS;t+=2) {
        assert(retraction_step(&s,t,q,v,.002f));
        assert(s.target[4]>=q[4] && s.target[4]<=q[4]+RETRACT_DELTA_RAD+1e-6f);
        assert(s.target[1]<=q[1] && s.target[1]>=q[1]-RETRACT_DELTA_RAD-1e-6f);
        assert(s.target[3]==q[3] && s.target[0]==q[0]);
        if (t>100+RETRACT_RAMP_MS && t<100+RETRACT_RAMP_MS+RETRACT_HOLD_MS) {
            assert(s.torque[0]>0 && s.torque[1]<0 && s.torque[3]>0 && s.torque[4]<0);
        }
        for(unsigned i=0;i<6;++i) {
            assert(fabsf(s.torque[i])<=20.00001f);
            assert(fabsf(s.torque[i]-last[i])<=(t<100+RETRACT_RAMP_MS+RETRACT_HOLD_MS ? .03001f : .04001f));last[i]=s.torque[i];
            if(i==2 || i==5) assert(s.torque[i]==0);
        }
        if (t==100+RETRACT_RAMP_MS+RETRACT_HOLD_MS-2) assert(fabsf(s.torque[1])>1 && fabsf(s.torque[4])>1);
    }
    assert(fabsf(s.torque[4])<.041f);
    assert(!retraction_step(&s,100+RETRACT_TOTAL_MS,q,v,.002f) && s.reason==1 && s.torque[4]==0);
    retraction_begin(&s,0xfffffff0U,q);assert(retraction_step(&s,0x10,q,v,.002f));
    v[3]=.81f;assert(!retraction_step(&s,0x12,q,v,.002f) && s.reason==3);v[3]=0;
    retraction_begin(&s,0,q);q[3]+=.18f;assert(!retraction_step(&s,2,q,v,.002f) && s.reason==2);q[3]-=.18f;
    q[4]=rl_physical_knee_limits[3]-.01f;retraction_begin(&s,0,q);
    assert(!retraction_step(&s,2,q,v,.002f) && s.reason==4);
    q[4]=3.004f;q[1]=rl_physical_knee_limits[0]+.01f;retraction_begin(&s,0,q);
    assert(!retraction_step(&s,2,q,v,.002f) && s.reason==4);
    puts("PASS: mirrored bilateral fold, both hips hold, zero wheels, sustained hold, torque/slew limits, deadline, wrap and both target-range guards");
    return 0;
}
