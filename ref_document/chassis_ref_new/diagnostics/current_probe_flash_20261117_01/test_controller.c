#include "controller.h"
#include "motor_io.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
    float q[6]={4,5,0,2,3,0},v[6]={0}; Retraction s;
    unsigned motors[4]={0,1,3,4};
    for(unsigned k=0;k<4;++k) for(int sign=-1;sign<=1;sign+=2) {
        g_probe_config.index=motors[k];g_probe_config.raw_nominal_nm=sign*3;
        assert(probe_configuration_valid());retraction_begin(&s,100,q);
        assert(retraction_step(&s,600,q,v,.002f));
        uint8_t dm[8],dji[8];assert(motor_encode(s.torque,dm,dji));
        for(unsigned slot=0;slot<4;++slot) {
            int16_t word=(int16_t)((unsigned)dm[2*slot]|((unsigned)dm[2*slot+1]<<8));
            if(slot==motor_id[motors[k]]-1U) assert(sign*word>0);
            else assert(word==0);
        }
        for(unsigned i=0;i<8;++i) assert(!dji[i]);
        g_probe_config.index=2;g_probe_config.raw_nominal_nm=999;
        assert(retraction_step(&s,1100,q,v,.002f));assert(s.index==motors[k]);
        assert(fabsf(s.torque[s.index])<=3.00001f);
        assert(retraction_step(&s,2098,q,v,.002f));
        assert(!retraction_step(&s,2100,q,v,.002f) && s.reason==1);
        for(unsigned i=0;i<6;++i) assert(s.torque[i]==0);
    }
    g_probe_config.index=4;
    const float invalid[]={0,.05f,3.01f,-3.01f,NAN,INFINITY};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i) {g_probe_config.raw_nominal_nm=invalid[i];assert(!probe_configuration_valid());}
    g_probe_config.raw_nominal_nm=.5f;
    retraction_begin(&s,0,q);q[0]+=.035f;assert(!retraction_step(&s,2,q,v,.002f) && s.reason==2);q[0]-=.035f;
    retraction_begin(&s,0,q);v[1]=.151f;assert(!retraction_step(&s,2,q,v,.002f) && s.reason==3);v[1]=0;
    retraction_begin(&s,0,q);assert(!retraction_step(&s,2,q,v,.004f) && s.reason==7);
    retraction_begin(&s,0xffffff00U,q);assert(retraction_step(&s,0xffffff00U+1000U,q,v,.002f));
    puts("PASS: selected raw-current polarity, zero other motors/wheels, latched parameters, bounded ramp/hold/release, displacement/speed/timing and wrap");
}
