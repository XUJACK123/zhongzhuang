#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "dm8009_app.h"
#include "remote_indicator.h"
typedef struct { uint16_t id; uint8_t data[8]; } Frame;
typedef struct {
    DM_App app;
    uint32_t now, count, enable_count, disable_count, cancels, scan_tx;
    Frame sent[32];
    uint16_t ids[2], master;
    uint32_t enabled, mode, suppress_rx, only_first_pass, bad_range, fail_tx, no_enable_ack, ignore_disable;
    float position_deg, speed_deg_s, torque, speed_noise_deg_s;
    DBUS_State remote;
} Rig;
static uint32_t LE32(const uint8_t *b) {
    return (uint32_t)b[0] | (uint32_t)b[1]<<8 | (uint32_t)b[2]<<16 | (uint32_t)b[3]<<24;
}
static int Send(void *ctx, uint16_t id, const uint8_t data[8]) {
    Rig *r = ctx;
    if (r->fail_tx) return 0;
    assert(r->count < 32); r->sent[r->count].id = id;
    memcpy(r->sent[r->count++].data, data, 8);
    if (r->app.stage == DM_SCAN) {
        assert(id == 0x7FF && data[2] == 0x33 && data[3] == 7);
        ++r->scan_tx;
    }
    if (id != 0x7FF && data[0] == 255 && data[1] == 255 && data[7] == 252) ++r->enable_count;
    if (id != 0x7FF && data[0] == 255 && data[1] == 255 && data[7] == 253) ++r->disable_count;
    return 1;
}
static void Cancel(void *ctx) { ++((Rig *)ctx)->cancels; }
static void Status(Rig *r, uint16_t id) {
    float p = (fmodf(r->position_deg+180.0f,360.0f)-180.0f) / 57.295779513f;
    uint32_t raw_p = (uint32_t)((p+12.5f)*65535.0f/25.0f+0.5f);
    uint32_t raw_v = (uint32_t)((r->speed_deg_s/57.295779513f+45.0f)*4095.0f/90.0f+0.5f);
    uint8_t b[8] = {(uint8_t)((r->enabled?0x10U:0U)|(id&15U)),
        (uint8_t)(raw_p>>8), (uint8_t)raw_p, (uint8_t)(raw_v>>4),
        (uint8_t)((raw_v&15U)<<4 | 8U), 0, 25, 25};
    DM_AppReceive(&r->app,r->now,r->master,b);
}
static void Reply(Rig *r) {
    for (uint32_t i=0;i<r->count;++i) {
        Frame *f=&r->sent[i]; uint8_t *d=f->data;
        if (f->id==0x7FF) {
            uint16_t id=(uint16_t)(d[0]|(uint16_t)d[1]<<8);
            if (id!=r->ids[0] && (!r->ids[1] || id!=r->ids[1])) continue;
            if (d[2]==0x33) {
                if (r->only_first_pass && r->app.scan_pass) continue;
                uint8_t b[8]; memcpy(b,d,8); uint32_t value=0;
                switch(d[3]) {
                case 7: value=r->master; break;
                case 8: value=id; break;
                case 10: value=r->mode; break;
                case 13: value=1; break;
                case 14: value=1; break;
                case 9: value=100; break;
                case 20: {float x=9;memcpy(&value,&x,4);break;}
                case 21: {float x=r->bad_range?NAN:12.5f;memcpy(&value,&x,4);break;}
                case 22: {float x=45;memcpy(&value,&x,4);break;}
                case 23: {float x=54;memcpy(&value,&x,4);break;}
                default: assert(0);
                }
                b[4]=(uint8_t)value;b[5]=(uint8_t)(value>>8);b[6]=(uint8_t)(value>>16);b[7]=(uint8_t)(value>>24);
                DM_AppReceive(&r->app,r->now,r->master,b);
            } else if (d[2]==0xCC) Status(r,id);
            else if (d[2]==0x55) {
                assert(d[3]==10 && LE32(d+4)==1);
                r->mode=1;
                /* A write ACK alone must never satisfy readback. */
                DM_AppReceive(&r->app,r->now,r->master,d);
                assert(r->app.mode!=1);
            } else assert(0); /* No Flash saves, zero-position, or unrelated writes. */
        } else if (DM_USE_MULTI && f->id==0x3FEU) {
            unsigned slot=2U*(r->ids[0]-1U);
            for(unsigned j=0;j<8U;++j)if(j!=slot && j!=slot+1U)assert(d[j]==0);
            int16_t raw=(int16_t)((uint16_t)d[slot]|(uint16_t)d[slot+1U]<<8);
            r->torque=(float)raw*(DM_MULTI_KT_NM_A*DM_MULTI_IMAX_A/16384.0f);
            assert(fabsf(r->torque)<=DM_STEP_TORQUE_LIMIT_NM);
        } else if (f->id==r->ids[0]) {
            if (d[0]==255 && d[1]==255 && d[7]==252) {
                if (!r->no_enable_ack) r->enabled=1;
            } else if(d[0]==255 && d[1]==255 && d[7]==253) {
                if(!r->ignore_disable) {r->enabled=0;r->torque=0;}
            } else {
                assert(d[3]==0 && d[4]==0 && d[5]==0 && (d[6]&0xF0)==0); /* Kp=Kd=0 */
                uint32_t t=((uint32_t)d[6]<<8)|d[7];
                r->torque=t*(108.0f/4095.0f)-54;
                assert(fabsf(r->torque)<=DM_STEP_TORQUE_LIMIT_NM);
            }
        } else assert(0);
    }
}
static void Tick(Rig *r) {
    r->count=0; r->remote.last_rx_ms=r->now;
    DM_IO io={.context=r,.send=Send,.cancel_pending=Cancel};
    DM_AppTick(&r->app,r->now,&r->remote,1,&io);
    assert(fabsf(r->app.step.torque_Nm)<=r->app.step.torque_cap_Nm+0.0001f);
    assert(r->app.step.torque_cap_Nm<=DM_STEP_TORQUE_LIMIT_NM+0.0001f);
    if (!r->suppress_rx) {
        Reply(r);
        if(DM_USE_MULTI && r->app.motor_id && !(r->no_enable_ack && r->app.drive==1U)) {
            float angle=fmodf(r->position_deg,360.0f);if(angle<0)angle+=360.0f;
            uint16_t p=(uint16_t)(angle*8192.0f/360.0f+0.5f)%8192U;
            int16_t v=(int16_t)((r->speed_deg_s+r->speed_noise_deg_s)/0.06f);
            uint8_t b[8]={(uint8_t)(p>>8),(uint8_t)p,(uint8_t)((uint16_t)v>>8),(uint8_t)v,0,0,25,1};
            DM_AppReceive(&r->app,r->now,0x300U+r->app.motor_id,b);
        }
    }
    r->now+=2;
}
static void Run(Rig *r,uint32_t ms) { for(uint32_t i=0;i<ms;i+=2) Tick(r); }
static void Init(Rig *r,uint16_t id,uint16_t master) {
    memset(r,0,sizeof(*r));r->ids[0]=id;r->master=master;r->mode=3;
    r->remote.valid=1;r->remote.right_switch=r->remote.left_switch=DBUS_UP;
    DM_AppInit(&r->app,0);
}
static void Ready(Rig *r) {
    Run(r,44000);
    assert(r->app.stage==DM_READY && r->app.error==0);
    assert(r->app.motor_id==r->ids[0] && r->app.master_id==r->master);
    assert(r->app.scan_count==1 && r->app.scan[0].pass_mask==3);
    assert(r->scan_tx==2*(DM_FIXED_CAN_ID?1:(DM_SCAN_LAST_ID-DM_SCAN_FIRST_ID+1)));
    assert(r->mode==1 && !r->enable_count && !r->app.step.armed);
}
static void Arm(Rig *r) {
    r->remote.right_switch=r->remote.left_switch=DBUS_DOWN;Run(r,120);
    assert(r->app.step.armed);
    r->remote.right_switch=r->remote.left_switch=DBUS_UP;Run(r,100);
    assert(r->app.step.active && r->app.step.step_count==1 && r->enable_count==1);
}
static void TestScan(void) {
    Rig r;Init(&r,0x123,0x456);Ready(&r); /* high ID and nonzero Master ID */
    Init(&r,1,0);r.ids[1]=0x321;Run(&r,44000);
    assert(r.app.stage==DM_FAULT && r.app.scan_result==DM_SCAN_MULTIPLE && !r.enable_count);
    Init(&r,1,0);r.suppress_rx=1;Run(&r,44000);
    assert(r.app.scan_result==DM_SCAN_NONE && !r.enable_count);
    Init(&r,1,0);r.only_first_pass=1;Run(&r,44000);
    assert(r.app.scan_result==DM_SCAN_UNCONFIRMED && !r.enable_count);
    Init(&r,1,0);r.bad_range=1;Run(&r,44000);
    assert(r.app.stage==DM_FAULT && r.app.error==DM_ERR_PARAMETER && !r.enable_count);
    /* Forged master, wrong full ID and duplicate replies cannot confirm two passes. */
    Init(&r,1,0);Run(&r,2002);
    uint8_t b[8]={1,0,0x33,7,0,0,0,0};
    DM_AppReceive(&r.app,r.now,99,b);assert(!r.app.scan_count || r.app.scan[0].master_id==0);
}
static void TestMotion(void) {
    Rig r;Init(&r,1,0);Ready(&r);Arm(&r);
    Run(&r,100);assert(r.app.step.step_count==1); /* held up cannot retrigger */
    r.position_deg=DM_STEP_DEGREES;r.speed_deg_s=0;Run(&r,220);
    assert(r.app.step.completed_count==1 && (DM_STEP_HOLD_AT_TARGET?(r.app.step.active && r.enabled):(!r.app.step.active && !r.enabled)));
    Run(&r,200);assert(r.enable_count==1);
    if(DM_STEP_HOLD_AT_TARGET) {Run(&r,DM_STEP_TIMEOUT_MS+100U);assert(r.app.step.holding && !r.app.step.fault && r.app.step.completed_count==1);}
    r.remote.right_switch=r.remote.left_switch=DBUS_MIDDLE;Run(&r,60);
    r.remote.right_switch=r.remote.left_switch=DBUS_UP;Run(&r,100);
    float expected_target=DM_STEP_FIXED_TARGET?DM_STEP_DEGREES:2.0f*DM_STEP_DEGREES;
    assert(r.app.step.step_count==2 && fabsf(r.app.step.target_deg-expected_target)<0.01f);
    uint32_t canceled=r.cancels;
    r.remote.right_switch=DBUS_MIDDLE;Tick(&r);
    assert(!r.enabled && !r.app.step.active && r.cancels>canceled);
    /* Remote loss cannot leave the enabled drive running. */
    r.remote.right_switch=r.remote.left_switch=DBUS_DOWN;Run(&r,120);
    r.remote.right_switch=r.remote.left_switch=DBUS_UP;Run(&r,100);
    r.remote.valid=0;Tick(&r);
    assert(!r.enabled && !r.app.step.armed && r.app.step.fault==1);
    r.remote.valid=1;Run(&r,200);assert(!r.enabled); /* requires down rearm */
}
static void TestStops(void) {
    Rig r;Init(&r,1,0);Ready(&r);Arm(&r);
    r.suppress_rx=1;Run(&r,140);
    assert(r.app.stage==DM_FAULT && r.app.error==DM_ERR_FEEDBACK);
    r.suppress_rx=0;Run(&r,40);assert(!r.enabled); /* disable retries after link returns */
    r.remote.right_switch=r.remote.left_switch=DBUS_DOWN;Run(&r,800);
    assert(r.app.stage==DM_READY && r.app.recovery_count==1 && !r.enabled && !r.app.step.active);
    r.remote.right_switch=r.remote.left_switch=DBUS_UP;Run(&r,100);
    assert(r.app.step.active && r.app.drive==2); /* deliberate new trigger, no auto-motion */
    Init(&r,1,0);Ready(&r);Arm(&r);r.position_deg=DM_STEP_TRAVEL_LIMIT_DEG+1.0f;Run(&r,40);
    assert(r.app.step.fault==8 && !r.enabled);
    Init(&r,1,0);Ready(&r);Arm(&r);r.speed_deg_s=70;Run(&r,40);
    assert(r.app.step.fault==5 && !r.enabled);
    Init(&r,1,0);Ready(&r);Arm(&r);Run(&r,DM_STEP_TIMEOUT_MS+100U);
    assert(r.app.step.fault==7 && !r.enabled);
    Init(&r,1,0);Ready(&r);r.no_enable_ack=1;Arm(&r);Run(&r,600);
    assert(r.app.stage==DM_FAULT && r.app.error==(DM_USE_MULTI?DM_ERR_FEEDBACK:DM_ERR_ENABLE));
    Init(&r,1,0);Ready(&r);Arm(&r);r.fail_tx=1;Tick(&r);
    assert(r.app.stage==DM_FAULT && r.app.error==DM_ERR_TX && !r.app.step.active);
    Init(&r,1,0);Ready(&r);Arm(&r);
    DM_IO io={.context=&r,.send=Send,.cancel_pending=Cancel};
    DM_AppTick(&r.app,r.now,&r.remote,0,&io);
    assert(r.app.stage==DM_FAULT && r.app.error==DM_ERR_BUS && !r.app.step.active);
}
static void TestClosedLoop(void) {
    Rig r;Init(&r,DM_FIXED_CAN_ID?DM_FIXED_CAN_ID:1,0);Ready(&r);Arm(&r);
    for(uint32_t t=0;t<DM_STEP_TIMEOUT_MS/2U && !r.app.step.completed_count;++t) {
        Tick(&r);
        /* Simple free rotor, inertia 0.05 kg*m^2 and viscous damping 0.15. */
        float omega=r.speed_deg_s/57.295779513f;
        float acceleration=(r.enabled?r.torque-0.15f*omega: -0.15f*omega)/0.05f;
        r.speed_deg_s+=acceleration*57.295779513f*0.002f;
        r.position_deg+=r.speed_deg_s*0.002f;
        assert(r.app.stage==DM_READY && !r.app.step.fault);
    }
    assert(r.app.step.completed_count==1 && (DM_STEP_HOLD_AT_TARGET?r.enabled:!r.enabled));
    assert(fabsf(r.position_deg-DM_STEP_DEGREES)<0.5f);
}
static void TestLoadedBreakaway(float noise) {
    if(!DM_USE_MULTI || DM_STEP_TORQUE_LIMIT_NM<2.7f)return;
    Rig r;Init(&r,DM_FIXED_CAN_ID?DM_FIXED_CAN_ID:1,0);Ready(&r);Arm(&r);
    for(uint32_t t=0;t<DM_STEP_TIMEOUT_MS/2U && !r.app.step.completed_count;++t) {
        r.speed_noise_deg_s=noise*((int32_t)(t%4U)-1.5f);
        Tick(&r);
#ifdef DM_TRACE_MODEL
        if(t%5000U==0)fprintf(stderr,"model t=%u pos=%f speed=%f torque=%f integral=%f desired=%f\n",t,r.position_deg,r.speed_deg_s,r.torque,r.app.step.integral,r.app.step.desired_speed_deg_s);
#endif
        float omega=r.speed_deg_s/57.295779513f;
        float applied=r.enabled?r.torque:0;
        if(fabsf(omega)<0.001f && fabsf(applied)<2.6f)omega=0;
        else {
            float direction=fabsf(omega)<0.001f?(applied>0?1:-1):(omega>0?1:-1);
            float next=omega+(applied-direction*2.4f-0.15f*omega)/0.05f*0.002f;
            if(next*omega<0 && fabsf(applied)<2.6f)next=0;
            omega=next;
        }
        r.speed_deg_s=omega*57.295779513f;r.position_deg+=r.speed_deg_s*0.002f;
        if(r.app.step.fault || fabsf(r.speed_deg_s)>=DM_STEP_OVERSPEED_DEG_S)
            fprintf(stderr,"breakaway t=%u speed=%f pos=%f torque=%f fault=%u\n",t,r.speed_deg_s,r.position_deg,r.torque,r.app.step.fault);
        assert(!r.app.step.fault && fabsf(r.speed_deg_s)<DM_STEP_OVERSPEED_DEG_S);
    }
    assert(r.app.step.completed_count==1 && (DM_STEP_HOLD_AT_TARGET?r.enabled:!r.enabled) &&
           fabsf(r.position_deg-DM_STEP_DEGREES)<0.5f);
}
static void TestOutputAngleAndLED(void) {
    Rig r; Init(&r,DM_FIXED_CAN_ID?DM_FIXED_CAN_ID:1,0);r.position_deg=179;Ready(&r);Arm(&r);
    r.position_deg=179.0f+DM_STEP_DEGREES;Run(&r,220);
    assert(r.app.step.completed_count==1 && (DM_STEP_HOLD_AT_TARGET?r.enabled:!r.enabled));
    assert(fabsf(r.app.position_deg-DM_STEP_DEGREES)<0.05f); /* positive motion across +pi -> -pi */
    uint16_t center=32768;
    uint16_t ten_deg=(uint16_t)(center+65535.0f*(10.0f/57.295779513f)/25.0f+0.5f);
    assert(fabsf(DM_OutputPositionDeltaDeg(center,ten_deg,12.5f,DM_OUTPUT_POSITION_PERIOD_RAD)-10.0f)<0.03f);
    assert(fabsf(DM_OutputPositionDeltaDeg(ten_deg,center,12.5f,DM_OUTPUT_POSITION_PERIOD_RAD)+10.0f)<0.03f);
    DBUS_State remote={0};
    assert(DM_RemoteColor(&remote,0)==REMOTE_LED_GREEN);
    remote.valid=1;remote.last_rx_ms=0;remote.right_switch=remote.left_switch=DBUS_DOWN;
    assert(DM_RemoteColor(&remote,0)==REMOTE_LED_BLUE);
    remote.right_switch=remote.left_switch=DBUS_UP;
    assert(DM_RemoteColor(&remote,0)==REMOTE_LED_RED);
    assert(DM_RemoteColor(&remote,101)==REMOTE_LED_GREEN);
    remote.right_switch=DBUS_MIDDLE;
    assert(DM_RemoteColor(&remote,0)==REMOTE_LED_BLUE);
}
static void TestNoDisabledAckRequired(void) {
    Rig r;Init(&r,DM_FIXED_CAN_ID?DM_FIXED_CAN_ID:1,0);
    r.enabled=1;r.ignore_disable=1;Ready(&r);
    assert(r.app.stage==DM_READY && r.app.feedback.state==1 && !r.enable_count);
    Arm(&r);
    assert(r.app.stage==DM_READY && r.app.drive==2 && r.app.step.torque_Nm>0);
    uint32_t canceled=r.cancels,stops=r.disable_count;
    r.remote.right_switch=r.remote.left_switch=DBUS_DOWN;Tick(&r);
    assert(!r.app.step.active && r.app.step.torque_Nm==0 && r.cancels>canceled && r.disable_count>stops);
    /* Remote loss must also stop the command even if FD never changes state. */
    Run(&r,120);r.remote.right_switch=r.remote.left_switch=DBUS_UP;Run(&r,100);
    assert(r.app.step.active);
    r.remote.valid=0;Tick(&r);
    assert(!r.app.step.active && r.app.step.torque_Nm==0 && !r.app.step.armed);
}
static void TestMultiStops(void) {
    if(!DM_USE_MULTI)return;
    Rig r;Init(&r,DM_FIXED_CAN_ID?DM_FIXED_CAN_ID:1,0);Ready(&r);r.ignore_disable=1;Arm(&r);Run(&r,1000);
    assert(r.torque>0);
    r.app.feedback_ms=r.now-DM_RX_TIMEOUT_MS-1U;Tick(&r);
    assert(r.app.stage==DM_READY && r.app.step.active); /* live group data is authoritative */
    r.remote.right_switch=r.remote.left_switch=DBUS_DOWN;Tick(&r);
    assert(r.enabled && r.torque==0 && !r.app.step.active); /* FD ignored: group zero still stops */
    Run(&r,120);r.remote.right_switch=r.remote.left_switch=DBUS_UP;Run(&r,100);
    /* MIT is still fresh, but loss of the actual group feedback must stop. */
    r.count=0;r.now+=DM_RX_TIMEOUT_MS+2;r.remote.last_rx_ms=r.now;
    Status(&r,r.app.motor_id);
    DM_IO io={.context=&r,.send=Send,.cancel_pending=Cancel};
    DM_AppTick(&r.app,r.now,&r.remote,1,&io);Reply(&r);
    assert(r.app.stage==DM_FAULT && r.app.error==DM_ERR_FEEDBACK && r.torque==0);
    Init(&r,DM_FIXED_CAN_ID?DM_FIXED_CAN_ID:1,0);Ready(&r);Arm(&r);r.app.multi_temperature=DM_STEP_MAX_TEMPERATURE;Tick(&r);
    assert(r.app.step.fault==4 && r.torque==0);
    /* Encoder crossing 8191 -> 0 is a small positive delta. */
    DM_App a={.motor_id=2};uint8_t b[8]={0x1F,0xFF,0,0,0,0,25,1};
    DM_AppReceive(&a,0,0x302,b);b[0]=0;b[1]=1;DM_AppReceive(&a,1,0x302,b);
    assert(fabsf(a.multi_position_deg-2.0f*360.0f/8192.0f)<0.001f);
}
int main(void) {
    if(DM_FIXED_CAN_ID) {
        Rig r;Init(&r,DM_FIXED_CAN_ID,0);Ready(&r);Arm(&r);
        r.remote.right_switch=r.remote.left_switch=DBUS_DOWN;Tick(&r);
        assert(!r.enabled && !r.app.step.active);
        TestOutputAngleAndLED();
        Init(&r,DM_FIXED_CAN_ID,0);r.suppress_rx=1;Run(&r,6000);
        assert(r.app.stage==DM_SCAN && !r.enable_count);
        r.suppress_rx=0;Run(&r,1000);
        assert(r.app.stage==DM_READY && !r.enable_count); /* slow motor boot */
    } else {
        TestScan();TestMotion();TestStops();TestClosedLoop();TestOutputAngleAndLED();
    }
    TestNoDisabledAckRequired();TestMultiStops();
    if(DM_USE_MULTI && DM_FIXED_CAN_ID){TestClosedLoop();TestLoadedBreakaway(0);TestLoadedBreakaway(1.0f);}
    puts("DM8009 scan, setup, interlocks, stops and simple closed-loop tests PASS");return 0;
}
