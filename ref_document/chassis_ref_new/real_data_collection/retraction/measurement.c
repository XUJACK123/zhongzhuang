#include "measurement.h"
#include <string.h>
#include <math.h>
#include <float.h>
StaticMeasurement g_static_measurement;
StopEvent g_stop_event;
void static_measurement_begin(uint32_t now) {
    uint32_t token=g_static_measurement.request;
    memset(&g_static_measurement,0,sizeof(g_static_measurement));
    g_static_measurement.request=g_static_measurement.ack=token;
    g_static_measurement.state=1;g_static_measurement.started=now;
    for(unsigned i=0;i<MEASURE_VALUES;++i) {g_static_measurement.min[i]=FLT_MAX;g_static_measurement.max[i]=-FLT_MAX;}
}
void static_measurement_sample(uint32_t now,uint32_t step,bool allowed,uint32_t inhibit,const float v[MEASURE_VALUES]) {
    StaticMeasurement *s=&g_static_measurement;if(s->state!=1) return;
    if(!allowed) {s->state=3;s->ended=now;return;}
    if(now-s->started>=60000U) {s->state=2;s->ended=now;return;}
    if(step>s->max_step_ms) s->max_step_ms=step;
    if(step>3U) ++s->step_gaps;
    s->inhibit_or|=inhibit;
    bool finite=true;for(unsigned i=0;i<MEASURE_VALUES;++i) finite=finite && isfinite(v[i]);
    if(!finite || (inhibit&(2U|4U|8U|16U))) {++s->invalid_count;return;}
    ++s->count;
    for(unsigned i=0;i<MEASURE_VALUES;++i) {
        float delta=v[i]-s->mean[i];s->mean[i]+=delta/s->count;s->m2[i]+=delta*(v[i]-s->mean[i]);
        if(v[i]<s->min[i]) s->min[i]=v[i];
        if(v[i]>s->max[i]) s->max[i]=v[i];
    }
}
