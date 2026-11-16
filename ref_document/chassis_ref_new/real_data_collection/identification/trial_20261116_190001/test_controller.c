#include "limited_current.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
enum {BASE=2200};
static IdentInput healthy(void) {
    IdentInput in={.now=100,.token=1,.s1=3,.remote_valid=true,.healthy=true};return in;
}
static void running(LimitedCurrent *c,IdentInput *in) {
    float a;memset(c,0,sizeof(*c));*in=healthy();ident_step(c,in,&a);assert(c->phase==ARMED && a==0);
    in->now=2099;ident_step(c,in,&a);assert(c->phase==ARMED && a==0);
    in->now=2100;ident_step(c,in,&a);assert(c->phase==ENABLING && a==0);
    in->now=BASE;ident_step(c,in,&a);assert(c->phase==RUNNING && a==0);
}
int main(void) {
    LimitedCurrent c={0};IdentInput in=healthy();float a;
    in.token=0;ident_step(&c,&in,&a);assert(c.phase==IDLE && a==0);
    in.token=1;in.s1=2;ident_step(&c,&in,&a);assert(c.phase==IDLE && c.last_token==1 && a==0);
    in.s1=3;ident_step(&c,&in,&a);assert(c.phase==IDLE && a==0);
    in.token=2;ident_step(&c,&in,&a);assert(c.phase==ARMED && a==0);
    in.s1=2;ident_step(&c,&in,&a);assert(c.phase==DONE && c.reason==REMOTE_STOP && a==0);
    in.s1=3;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    in.token=3;ident_step(&c,&in,&a);assert(c.phase==ARMED && a==0);
    running(&c,&in);in.now=BASE+250;ident_step(&c,&in,&a);assert(fabsf(a+1)<1e-6f);
    in.now=BASE+499;ident_step(&c,&in,&a);assert(a<0 && a>=-2);
    in.now=BASE+500;ident_step(&c,&in,&a);assert(c.phase==COOLDOWN && c.trial_index==1 && a==0);
    in.now=BASE+1500;ident_step(&c,&in,&a);assert(c.phase==ENABLING && a==0);
    in.now=BASE+1600;ident_step(&c,&in,&a);assert(c.phase==RUNNING && a==0);
    in.now=BASE+1850;ident_step(&c,&in,&a);assert(fabsf(a+2)<1e-6f);
    in.now=BASE+2100;ident_step(&c,&in,&a);assert(c.phase==COOLDOWN && a==0);
    in.now=BASE+3100;ident_step(&c,&in,&a);assert(c.phase==ENABLING && a==0);
    in.now=BASE+3200;ident_step(&c,&in,&a);assert(c.phase==RUNNING && a==0);
    in.now=BASE+3450;ident_step(&c,&in,&a);assert(fabsf(a+3)<1e-6f);
    in.now=BASE+3700;ident_step(&c,&in,&a);assert(c.phase==DONE && c.reason==COMPLETE && a==0);
    in.now+=1000;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    in.token=2;ident_step(&c,&in,&a);assert(c.phase==ARMED && a==0);
    running(&c,&in);in.s1=2;ident_step(&c,&in,&a);assert(c.phase==DONE && c.reason==REMOTE_STOP && a==0);
    running(&c,&in);in.remote_age=101;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.remote_valid=false;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.s1=0;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.healthy=false;ident_step(&c,&in,&a);assert(c.phase==DONE && c.reason==FEEDBACK_STOP && a==0);
    running(&c,&in);in.arm[1]=IDENT_MAX_DELTA_RAD+.001f;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.arm[4]=IDENT_MAX_DELTA_RAD+.001f;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.velocity[0]=.501f;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.velocity[1]=.4f;ident_step(&c,&in,&a);assert(c.phase==RUNNING);
    running(&c,&in);in.velocity[2]=.5f;ident_step(&c,&in,&a);assert(c.phase==RUNNING);
    in.velocity[2]=1.001f;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.arm[5]=IDENT_WHEEL_MAX_DELTA_RAD+.001f;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.arm[1]=NAN;ident_step(&c,&in,&a);assert(c.phase==DONE && c.reason==INVALID_INPUT && a==0);
    running(&c,&in);c.since=UINT32_MAX-100;in.now=149;ident_step(&c,&in,&a);assert(fabsf(a+1)<1e-6f);
    in.now=399;ident_step(&c,&in,&a);assert(c.phase==COOLDOWN && a==0);
    in.s1=2;ident_step(&c,&in,&a);assert(c.phase==DONE && c.reason==REMOTE_STOP && a==0);
    puts("Host-trigger, down-stop, cancellation, three-level limits and wrap tests passed");return 0;
}
