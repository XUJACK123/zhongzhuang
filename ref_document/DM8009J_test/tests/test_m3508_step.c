#include <assert.h>
#include <string.h>
#include "dbus_protocol.h"
#include "m3508_step.h"
static M3508_StepInput input = { .remote_ok=1, .motor_ok=1, .bus_ok=1,
    .right_switch=DBUS_DOWN, .left_switch=DBUS_DOWN, .temperature=25 };
static void Tick(M3508_StepController *c, unsigned duration) {
    for (unsigned n=0; n<duration; n+=2) {
        input.now_ms+=2;
        M3508_StepUpdate(c,&input);
        assert(c->current <= STEP_CURRENT_LIMIT && c->current >= -STEP_CURRENT_LIMIT);
    }
}
static void Arm(M3508_StepController *c) {
    input.right_switch=input.left_switch=DBUS_DOWN;
    Tick(c,120); assert(c->armed && !c->active && c->current==0);
}
static void Up(M3508_StepController *c) {
    input.right_switch=input.left_switch=DBUS_UP;
    Tick(c,50);
}
static void Pack(uint8_t b[18], unsigned right, unsigned left, unsigned ch0) {
    memset(b,0,18);
    uint64_t bits=(uint64_t)ch0 | ((uint64_t)1024<<11) | ((uint64_t)1024<<22) |
                  ((uint64_t)1024<<33) | ((uint64_t)right<<44) | ((uint64_t)left<<46);
    for(unsigned i=0;i<6;++i) b[i]=(uint8_t)(bits>>(i*8));
    b[16]=0; b[17]=4;
}
int main(void) {
    uint8_t frame[18]; DBUS_State remote={0};
    Pack(frame,DBUS_DOWN,DBUS_UP,1684);
    assert(DBUS_Decode(frame,&remote));
    assert(remote.right_switch==DBUS_DOWN && remote.left_switch==DBUS_UP);
    assert(remote.channel[0]==660 && remote.channel[4]==0);
    Pack(frame,0,DBUS_UP,1024); assert(!DBUS_Decode(frame,&remote));
    Pack(frame,DBUS_UP,DBUS_UP,0); assert(!DBUS_Decode(frame,&remote));
    Pack(frame,DBUS_UP,DBUS_UP,1024); frame[12]=2; assert(!DBUS_Decode(frame,&remote));
    assert(M3508_EncoderDelta(8190,3)==5 && M3508_EncoderDelta(3,8190)==-5);

    M3508_StepController c={0};
    input.right_switch=input.left_switch=DBUS_UP;
    Tick(&c,300); assert(!c.armed && c.step_count==0 && c.current==0);
    Arm(&c);
    input.left_switch=DBUS_DOWN; input.right_switch=DBUS_UP;
    Tick(&c,100); assert(c.step_count==0 && c.current==0);
    input.left_switch=DBUS_UP;
    Tick(&c,20); assert(c.step_count==0); /* debounce */
    Tick(&c,40); assert(c.step_count==1 && c.active && c.target_deg==10 && c.current>0);
    Tick(&c,200); assert(c.step_count==1); /* held up never repeats */
    input.position_deg=10; input.rotor_rpm=0;
    Tick(&c,200); assert(c.completed_count==1 && !c.active && c.current==0);
    Tick(&c,300); assert(c.step_count==1 && c.current==0);
    Arm(&c); Up(&c); assert(c.step_count==2 && c.target_deg==20);
    input.right_switch=input.left_switch=DBUS_DOWN;
    Tick(&c,2); assert(!c.active && c.current==0); /* immediate stop */
    Arm(&c); Up(&c); assert(c.active);
    input.remote_ok=0; Tick(&c,2); assert(c.fault==1 && c.current==0 && !c.armed);
    input.remote_ok=1; Tick(&c,100); assert(!c.active && !c.armed); /* reconnect held up */
    Arm(&c); Up(&c); assert(c.active);
    input.motor_ok=0; Tick(&c,2); assert(c.fault==2 && c.current==0);
    input.motor_ok=1; Arm(&c); Up(&c);
    input.bus_ok=0; Tick(&c,2); assert(c.fault==3 && c.current==0);
    input.bus_ok=1; Arm(&c); Up(&c);
    input.temperature=75; Tick(&c,2); assert(c.fault==4 && c.current==0);
    input.temperature=25; Arm(&c); Up(&c);
    input.rotor_rpm=1000; Tick(&c,2); assert(c.fault==5 && c.current==0);
    input.rotor_rpm=0; Arm(&c); Up(&c);
    Tick(&c,STEP_TIMEOUT_MS+100); assert(c.fault==7 && c.current==0 && !c.armed);
    Arm(&c); Up(&c);
    input.left_switch=DBUS_MIDDLE; Tick(&c,2); assert(c.current==0 && !c.active);
    unsigned steps=c.step_count;
    input.left_switch=DBUS_UP; Tick(&c,100); assert(c.step_count==steps); /* short bounce won't repeat */
    input.left_switch=DBUS_MIDDLE; Tick(&c,60);
    input.left_switch=DBUS_UP; Tick(&c,60); assert(c.step_count==steps+1);

    /* Sanity simulation of the complete cascaded loop; not a hardware validation. */
    c=(M3508_StepController){0};
    input=(M3508_StepInput){.remote_ok=1,.motor_ok=1,.bus_ok=1,.temperature=25};
    Arm(&c); Up(&c);
    for(unsigned t=0;t<4900 && !c.completed_count;++t) {
        input.rotor_rpm += ((float)c.current*0.2f-input.rotor_rpm)*(0.002f/0.06f);
        input.position_deg += input.rotor_rpm*6.0f/M3508_GEAR_RATIO*0.002f;
        Tick(&c,2);
    }
    assert(c.completed_count==1 && c.fault==0);
    assert(input.position_deg>9.6f && input.position_deg<10.4f);
    return 0;
}
