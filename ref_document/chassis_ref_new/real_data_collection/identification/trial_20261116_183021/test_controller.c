#include "limited_current.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static IdentInput healthy(void) {
    IdentInput in={.now=100,.token=1,.s1=2,.remote_valid=true,.healthy=true};return in;
}
static void running(LimitedCurrent *c,IdentInput *in) {
    float a;memset(c,0,sizeof(*c));*in=healthy();ident_step(c,in,&a);assert(c->phase==ARMED && a==0);
    in->s1=1;in->now=200;ident_step(c,in,&a);assert(c->phase==ENABLING && a==0);
    in->now=300;ident_step(c,in,&a);assert(c->phase==RUNNING && a==0);
}
int main(void) {
    LimitedCurrent c;IdentInput in;float a;
    memset(&c,0,sizeof(c));in=healthy();in.s1=1;ident_step(&c,&in,&a);assert(c.phase==IDLE && a==0);
    running(&c,&in);in.now=550;ident_step(&c,&in,&a);assert(fabsf(a+.25f)<1e-6f);
    in.now=799;ident_step(&c,&in,&a);assert(a<0 && a>=-.5f);
    in.now=800;ident_step(&c,&in,&a);assert(c.phase==DONE && c.reason==COMPLETE && a==0);
    in.now=1000;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    in.s1=2;ident_step(&c,&in,&a);assert(c.phase==DONE); /* consumed token cannot repeat */
    in.token=2;ident_step(&c,&in,&a);assert(c.phase==ARMED && a==0);
    running(&c,&in);in.s1=2;ident_step(&c,&in,&a);assert(c.phase==DONE && c.reason==REMOTE_STOP && a==0);
    running(&c,&in);in.remote_age=101;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.remote_valid=false;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.s1=0;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.healthy=false;ident_step(&c,&in,&a);assert(c.phase==DONE && c.reason==FEEDBACK_STOP && a==0);
    running(&c,&in);in.arm[1]=IDENT_MAX_DELTA_RAD+.001f;ident_step(&c,&in,&a);assert(c.phase==DONE && c.reason==MOVEMENT_STOP && a==0);
    running(&c,&in);in.arm[4]=IDENT_MAX_DELTA_RAD+.001f;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.velocity[0]=.051f;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    running(&c,&in);in.arm[1]=NAN;ident_step(&c,&in,&a);assert(c.phase==DONE && c.reason==INVALID_INPUT && a==0);
    running(&c,&in);c.since=UINT32_MAX-100;in.now=149;ident_step(&c,&in,&a);assert(fabsf(a+.25f)<1e-6f);
    in.now=399;ident_step(&c,&in,&a);assert(c.phase==DONE && a==0);
    puts("Limited-current stop/interlock tests passed");return 0;
}
