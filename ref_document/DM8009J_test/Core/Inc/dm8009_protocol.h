#ifndef DM8009_PROTOCOL_H
#define DM8009_PROTOCOL_H
#include <stdint.h>

/* Decode radians first: PMAX is the wire mapping range, not the gearbox ratio.
 * Single-turn joint encoders wrap at 2*pi even when PMAX is 12.5 rad. */
static inline float DM_OutputPositionDeltaDeg(uint16_t previous, uint16_t next,
                                              float pmax, float period_rad) {
    float delta = ((int32_t)next - (int32_t)previous) * (2.0f * pmax / 65535.0f);
    if (delta > period_rad * 0.5f) delta -= period_rad;
    if (delta < -period_rad * 0.5f) delta += period_rad;
    return delta * 57.295779513f;
}

/* This project's STM32H7 HAL exposes an unshifted DLC (0..15).
 * The hardware message-RAM DLC field is shifted, but HAL already extracts it. */
static inline uint32_t DM_DlcToLength(uint32_t dlc) {
    static const uint8_t lengths[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    return dlc < 16U ? lengths[dlc] : 0U;
}
static inline int DM_ControlParametersValid(uint32_t current_input, uint32_t mask,
                                            float kt, float tmax) {
    if (!(mask & 2U) || !(tmax > 0.0f && tmax <= 1000.0f)) return 0;
    return !current_input || ((mask & 1U) && kt > 0.0f && kt <= 100.0f);
}

/* MIT position/velocity centered, Kp=Kd=0; only feedforward torque is active.
 * Rounding exposes quantization instead of claiming exact zero/current. */
static inline int DM_MitTorquePayload(float torque, float tmax, uint8_t data[8],
                                      float *quantized_torque) {
    if (!(tmax > 0.0f && tmax <= 1000.0f) ||
        !(torque >= -tmax && torque <= tmax)) return 0;
    uint32_t raw = (uint32_t)((torque + tmax) * (4095.0f / (2.0f * tmax)) + 0.5f);
    if (raw > 4095U) raw = 4095U;
    data[0] = 0x80U; data[1] = 0;
    data[2] = 0x80U; data[3] = 0; /* velocity midpoint and Kp high bits */
    data[4] = 0; data[5] = 0;   /* Kp low bits, Kd high bits */
    data[6] = (uint8_t)(raw >> 8); /* Kd low bits zero */
    data[7] = (uint8_t)raw;
    *quantized_torque = ((float)raw * (2.0f * tmax) / 4095.0f) - tmax;
    return 1;
}

/* HKU CAN_DAMIAO_Multi_CMD: four signed int16 LITTLE-endian slots.
 * Only IDs 1..4 on an isolated bus are supported; all other slots stay zero. */
static inline int DM_MultiTorquePayload(uint16_t motor_id, float torque,
        float kt, float imax, uint8_t data[8], float *actual) {
    if (motor_id < 1U || motor_id > 4U || !(kt > 0 && imax > 0) ||
        !(torque >= -kt*imax && torque <= kt*imax)) return 0;
    int16_t raw = (int16_t)(torque * (16384.0f / (kt*imax)));
    for (unsigned i=0;i<8U;++i) data[i]=0;
    unsigned slot=2U*(motor_id-1U);
    data[slot]=(uint8_t)raw; data[slot+1]=(uint8_t)((uint16_t)raw>>8);
    *actual=(float)raw*(kt*imax/16384.0f);
    return 1;
}

/* Wire format checked against dmBots/motor-sdk, Python u2can/DM_CAN.py.
 * Pure wire encoding helpers; no hardware access or persistent writes. */
static inline void DM_SpecialPayload(uint8_t tail, uint8_t data[8]) {
    for (unsigned i = 0; i < 7; ++i) data[i] = 0xFFU;
    data[7] = tail; /* FC enable, FD disable */
}
static inline void DM_QueryPayload(uint16_t id, uint8_t command,
                                   uint8_t rid, uint8_t data[8]) {
    for (unsigned i = 0; i < 8; ++i) data[i] = 0;
    data[0] = (uint8_t)id;
    data[1] = (uint8_t)(id >> 8);
    data[2] = command; /* CC request status, 33 read parameter */
    data[3] = rid;
}
typedef struct {
    uint8_t motor_id_low4;
    uint8_t state; /* 0 disabled, 1 enabled, 8..E fault (not a boolean) */
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t torque_raw;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
} DM_FeedbackFields;
static inline int DM_DecodeFeedback(uint32_t can_id, uint32_t master_id,
                                    uint16_t motor_id, uint32_t length,
                                    const uint8_t data[8], DM_FeedbackFields *out) {
    if (length != 8U || can_id != master_id ||
        (data[0] & 0xFU) != (motor_id & 0xFU)) return 0;
    out->motor_id_low4 = data[0] & 0xFU;
    out->state = data[0] >> 4;
    out->position_raw = ((uint16_t)data[1] << 8) | data[2];
    out->velocity_raw = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    out->torque_raw = ((uint16_t)(data[4] & 0xFU) << 8) | data[5];
    out->mos_temperature = data[6];
    out->rotor_temperature = data[7];
    return 1;
}
#endif
