/* DR16/DT7: UART5 RX PD2, 100000 baud, 8 data + even parity + 1 stop.
 * s1 (right switch): DOWN (2) = zero current, any other position runs after
 * one down transition. s2 is ignored. Right vertical ch1 = forward, right
 * horizontal ch0 = yaw; left sticks unused. */
#include "remote_control.h"
#include "robot_config.h"
#include <stdlib.h>

bool remote_decode(RemoteControl *r, const uint8_t *b, unsigned size, uint32_t now) {
    if (size!=18) { r->valid=false; ++r->rejected; return false; }
    int16_t ch[5]={
        (int16_t)((b[0]|b[1]<<8)&0x7ff)-1024,
        (int16_t)(((b[1]>>3)|(b[2]<<5))&0x7ff)-1024,
        (int16_t)(((b[2]>>6)|(b[3]<<2)|(b[4]<<10))&0x7ff)-1024,
        (int16_t)(((b[4]>>1)|(b[5]<<7))&0x7ff)-1024,
        (int16_t)((b[16]|b[17]<<8)&0x7ff)-1024};
    uint8_t s1=(b[5]>>4)&3, s2=(b[5]>>6)&3;
    bool ok=s1>=1 && s1<=3 && s2>=1 && s2<=3;
    for (unsigned i=0;i<5;++i) if (abs(ch[i])>700) ok=false;
    if (!ok) { r->valid=false; ++r->rejected; return false; }
    for (unsigned i=0;i<5;++i) r->channel[i]=ch[i];
    r->s1=s1; r->s2=s2; r->tick=now; ++r->count; r->valid=true;
    return true;
}
bool remote_fresh(const RemoteControl *r, uint32_t now) {
    return r->valid && r->count!=0 && now-r->tick<=ROBOT_REMOTE_TIMEOUT_MS;
}
static float stick(int16_t v) {
    if (abs(v)<=20) return 0;
    float x=(float)v/660;
    return x>1 ? 1 : (x< -1 ? -1 : x);
}
void remote_request(const RemoteControl *r, float c[3]) {
    /* This archived policy was not trained for reverse travel. */
    float forward=stick(r->channel[1]);
    c[0]=(forward>0 ? forward : 0)*ROBOT_REMOTE_MAX_FORWARD;
    c[1]=-stick(r->channel[0])*ROBOT_REMOTE_MAX_YAW;
    c[2]=ROBOT_HEIGHT_COMMAND_M;
}
bool remote_sticks_centered(const RemoteControl *r) {
    for (unsigned i=0;i<4;++i) if (abs(r->channel[i])>30) return false;
    return true;
}
