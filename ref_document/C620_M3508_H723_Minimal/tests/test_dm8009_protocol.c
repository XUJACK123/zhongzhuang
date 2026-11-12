#include <assert.h>
#include <string.h>
#include "dm8009_protocol.h"

int main(void) {
    /* Hardware regression: KT=0 must block current units but not torque units. */
    assert(DM_ControlParametersValid(0,3,0,54));
    assert(!DM_ControlParametersValid(1,3,0,54));
    assert(DM_ControlParametersValid(1,3,1.261575f,54));
    assert(!DM_ControlParametersValid(0,3,0,0));
    assert(!DM_ControlParametersValid(0,1,1.261575f,54));
    /* Regression: HAL returns 8 for an eight-byte frame, not 8 << 16. */
    assert(DM_DlcToLength(8U) == 8U);
    assert(DM_DlcToLength(0U) == 0U);
    assert(DM_DlcToLength(9U) == 12U);
    assert(DM_DlcToLength(15U) == 64U);
    assert(DM_DlcToLength(0x80000U) == 0U);
    uint8_t data[8];
    float quantized;
    const uint8_t neutral[8] = {0x80,0,0x80,0,0,0,8,0};
    assert(DM_MitTorquePayload(0,54,data,&quantized));
    assert(memcmp(data,neutral,8)==0);
    assert(quantized > 0 && quantized < 0.014f);
    assert(DM_MitTorquePayload(0.05f * 1.261575f,54,data,&quantized));
    assert(data[6]==8 && data[7]==2);
    assert(quantized/1.261575f > 0.05f && quantized/1.261575f < 0.06f);
    assert(DM_MitTorquePayload(0.05f,54,data,&quantized));
    assert(data[6]==8 && data[7]==1);
    assert(quantized > 0.039f && quantized < 0.040f);
    assert(DM_MitTorquePayload(-54,54,data,&quantized) && data[6]==0 && data[7]==0);
    assert(DM_MitTorquePayload(54,54,data,&quantized) && data[6]==15 && data[7]==255);
    assert(!DM_MitTorquePayload(55,54,data,&quantized));
    assert(!DM_MitTorquePayload(0,0,data,&quantized));
    DM_SpecialPayload(0xFC, data);
    const uint8_t enable[8] = {255,255,255,255,255,255,255,252};
    assert(memcmp(data, enable, 8) == 0);
    DM_SpecialPayload(0xFD, data);
    assert(data[0] == 255 && data[7] == 253);
    DM_QueryPayload(0x123, 0xCC, 0, data);
    const uint8_t query[8] = {0x23,0x01,0xCC,0,0,0,0,0};
    assert(memcmp(data, query, 8) == 0);
    DM_QueryPayload(1, 0x33, 7, data);
    assert(data[0] == 1 && data[1] == 0 && data[2] == 0x33 && data[3] == 7);

    /* State and low ID nibble must be separated; velocity and torque share byte4. */
    const uint8_t feedback[8] = {0x11,0x12,0x34,0xAB,0xCD,0xEF,40,41};
    DM_FeedbackFields out;
    assert(DM_DecodeFeedback(0, 0, 1, DM_DlcToLength(8U), feedback, &out));
    assert(out.motor_id_low4 == 1 && out.state == 1);
    assert(out.position_raw == 0x1234);
    assert(out.velocity_raw == 0xABC && out.torque_raw == 0xDEF);
    assert(out.mos_temperature == 40 && out.rotor_temperature == 41);
    assert(!DM_DecodeFeedback(0x201, 0, 1, 8, feedback, &out));
    assert(!DM_DecodeFeedback(0, 0, 2, 8, feedback, &out));
    assert(!DM_DecodeFeedback(0, 0, 1, 7, feedback, &out));
    uint8_t fault[8] = {0xD1,0,0,0,0,0,0,0};
    assert(DM_DecodeFeedback(0x11, 0x11, 1, 8, fault, &out));
    assert(out.state == 0xD); /* communication-loss fault, not enabled */
    fault[0] = 1;
    assert(DM_DecodeFeedback(0x11, 0x11, 1, 8, fault, &out) && out.state == 0);
    return 0;
}
