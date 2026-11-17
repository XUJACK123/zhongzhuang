#include "measurement.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
    float v[MEASURE_VALUES];for(unsigned i=0;i<MEASURE_VALUES;++i)v[i]=(float)i;
    g_static_measurement.request=2;static_measurement_begin(100);
    for(unsigned t=100;t<60100;t+=2)static_measurement_sample(t,2,true,0,v);
    static_measurement_sample(60100,2,true,0,v);
    assert(g_static_measurement.state==2 && g_static_measurement.count==30000 && g_static_measurement.ack==2);
    for(unsigned i=0;i<MEASURE_VALUES;++i)assert(g_static_measurement.mean[i]==v[i] && g_static_measurement.m2[i]==0);
    static_measurement_begin(0xfffffff0U);static_measurement_sample(0x10,4,true,4,v);
    assert(g_static_measurement.invalid_count==1 && g_static_measurement.step_gaps==1);
    static_measurement_sample(0x12,2,false,0,v);assert(g_static_measurement.state==3);
    static_measurement_begin(1);v[0]=NAN;static_measurement_sample(3,2,true,0,v);assert(g_static_measurement.invalid_count==1);
    puts("PASS: 60-second 2ms statistics, invalid samples, interruption and wrap");return 0;
}
