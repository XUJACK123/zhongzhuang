#include "motor_io.h"
#include "robot_config.h"
#include "algorithm/rl.h"
#include <math.h>
#include <string.h>
const uint8_t motor_id[6]={4,3,3,1,2,2}; /* right hip,knee,wheel; left hip,knee,wheel */
static const float signs[6]=ROBOT_ENCODER_SIGNS;
static const float encoder_reference[6]=ROBOT_ENCODER_REFERENCE_RAD;
static const float arm_reference[6]=ROBOT_ARM_REFERENCE_RAD;
static int16_t be16(const uint8_t *d) { return (int16_t)((uint16_t)d[0]<<8|d[1]); }
bool motor_receive(MotorBank *b, uint16_t id, const uint8_t d[8], unsigned len, uint32_t now) {
    int index=-1;
    for (unsigned i=0;i<6;++i) {
        uint16_t base=(i==2 || i==5) ? 0x200 : 0x300;
        if (id==base+motor_id[i]) { index=(int)i; break; }
    }
    if (index<0) return false;
    if (len!=8) { ++b->rejected; return false; }
    unsigned i=(unsigned)index, raw=(unsigned)(uint16_t)be16(d);
    bool wheel=i==2 || i==5;
    if (raw>=8192) { ++b->rejected; return false; }
    MotorFeedback *m=&b->motor[i];
    if (m->received) {
        if (now-m->tick>ROBOT_FEEDBACK_TIMEOUT_MS) m->position_lost=true;
        int delta=(int)raw-(int)m->raw;
        if (delta>4096) delta-=8192;
        if (delta< -4096) delta+=8192;
        m->counts+=delta;
    } else m->counts=(int64_t)raw;
    float reduction=wheel ? ROBOT_WHEEL_GEAR_RATIO : 1;
    float angle=(float)m->counts*(2*RL_PI/8192)/reduction;
    if (!m->received) {
        /* Stored reference selects the absolute arm branch; power-on is NOT zero. */
        float period=2*RL_PI/reduction;
        angle=encoder_reference[i]+remainderf(angle-encoder_reference[i],period);
        m->counts=(int64_t)llroundf(angle*reduction*(8192/(2*RL_PI)));
    }
    m->arm=arm_reference[i]+signs[i]*(angle-encoder_reference[i]);
    m->velocity=signs[i]*(float)be16(d+2)*(wheel ? 2*RL_PI/(60*reduction) : RL_PI/3000);
    m->current=(float)be16(d+4)*(wheel ? 20.0f/16384 : 0.001f);
    m->raw=(uint16_t)raw; m->temperature=d[6]; m->state=wheel ? 1 : d[7];
    m->tick=now; ++m->received;
    return true;
}
/* DM state byte (DM8009_DIAGNOSTIC.md): 0 disabled, 1 enabled, 8 overvoltage,
 * 9 undervoltage, A overcurrent, B MOS over-temperature, C winding
 * over-temperature, D communication loss, E overload. Group-mode firmware
 * latches D even while feedback keeps arriving, so freshness is the real
 * communication proof and only the power/thermal codes block output. */
static bool dm_fault(uint8_t s) { return (s>=8 && s<=0xC) || s==0xE; }
bool motor_feedback_healthy(const MotorBank *b, uint32_t now) {
    for (unsigned i=0;i<6;++i) {
        const MotorFeedback *m=&b->motor[i];
        if (!m->received || now-m->tick>ROBOT_FEEDBACK_TIMEOUT_MS ||
            m->temperature>=ROBOT_MAX_TEMPERATURE_C || m->position_lost ||
            (i!=2 && i!=5 && dm_fault(m->state))) return false;
    }
    return true;
}
bool motor_all_enabled(const MotorBank *b) {
    for (unsigned i=0;i<6;++i)
        if (i!=2 && i!=5 && (!b->motor[i].received || dm_fault(b->motor[i].state))) return false;
    return true;
}
bool motor_encode(const float t[6], uint8_t dm[8], uint8_t dji[8]) {
    memset(dm,0,8); memset(dji,0,8);
    for (unsigned i=0;i<6;++i) {
        bool wheel=i==2 || i==5;
        float cap=wheel ? ROBOT_WHEEL_TORQUE_CAP_NM : ROBOT_LEG_TORQUE_CAP_NM;
        if (!isfinite(t[i]) || fabsf(t[i])>cap+1e-5f || fabsf(signs[i])!=1) {
            memset(dm,0,8); memset(dji,0,8); return false;
        }
        float scale=wheel ? 16384/(20*ROBOT_WHEEL_KT_NM_A) :
                           16384/(ROBOT_DM_KT_NM_A*ROBOT_DM_CURRENT_FULL_SCALE_A);
        int16_t raw=(int16_t)rl_clip(t[i]*signs[i]*scale,-16384,16384);
        unsigned slot=2U*(motor_id[i]-1U);
        if (wheel) { dji[slot]=(uint8_t)((uint16_t)raw>>8); dji[slot+1]=(uint8_t)raw; }
        else { dm[slot]=(uint8_t)raw; dm[slot+1]=(uint8_t)((uint16_t)raw>>8); }
    }
    return true;
}
