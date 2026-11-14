#ifndef CHASSIS_H
#define CHASSIS_H
#include <stdint.h>
#include "algorithm/rl.h"
#include "motor_io.h"
#include "control_state.h"
typedef struct {
    uint32_t tick, inhibit, fault, cycles, policy_runs, inference_us, max_loop_us, overruns;
    ControlMode mode;
    uint32_t actor_selftest;
} ChassisDebug;
/* Inhibit: 1 config,2 remote,4 motors,8 IMU,16 bus,32 geometry,64 tilt,
 * 128 policy selftest,256 start pose. Fault: 1 TX,2 timing,3 policy,4 torque. */
extern ChassisDebug g_chassis;
extern RlController g_rl;
extern RlMeasurement g_measurement;
extern MotorBank g_motor_snapshot;
void chassis_init(void);
void chassis_receive(uint16_t id, const uint8_t data[8], uint32_t now);
void chassis_service(uint32_t now);
#endif
