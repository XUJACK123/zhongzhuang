#include "motor_position.h"
#include "bsp_can.h"
#include <math.h>

volatile MotorState motor_state = MOTOR_WAITING;
volatile float motor_angle_deg;
volatile float motor_reference_deg;
static CANInstance *group;
static uint32_t boot_ms, last_ms, move_ms, settled_ms;
static int32_t origin;
static float integral;

static float clamp(float value, float limit)
{
    return value > limit ? limit : (value < -limit ? -limit : value);
}

void MotorPositionInit(void)
{
    group = CANRegister(0x200, 0x201);
    CANServiceInit();
    boot_ms = last_ms = HAL_GetTick();
    motor_state = MOTOR_WAITING;
    motor_angle_deg = motor_reference_deg = integral = 0.0f;
    settled_ms = 0;
}

void MotorPositionUpdate(void)
{
    MotorFeedback fb;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - last_ms;
    if (elapsed < 1U) return;
    last_ms = now;
    CANGetMotorFeedback(&fb);
    float command = 0.0f;
    float dt = elapsed * 0.001f;
    if (HAL_CAN_GetError(&hcan1) != HAL_CAN_ERROR_NONE ||
        (fb.received && fb.temperature >= 70)) motor_state = MOTOR_FAULT;

    if (motor_state == MOTOR_WAITING)
    {
        if (now - boot_ms > 3000U) motor_state = MOTOR_FAULT;
        else if (now - boot_ms >= 500U && fb.received &&
                 now - fb.last_rx_ms <= MOTOR_FEEDBACK_MS &&
                 fb.speed_rpm >= -5 && fb.speed_rpm <= 5)
        {
            origin = fb.total_counts;
            move_ms = now;
            motor_state = MOTOR_MOVING;
        }
    }
    if (motor_state == MOTOR_MOVING || motor_state == MOTOR_HOLDING)
    {
        if (!fb.received || now - fb.last_rx_ms > MOTOR_FEEDBACK_MS || elapsed > 20U)
            motor_state = MOTOR_FAULT;
        else
        {
            motor_angle_deg = (fb.total_counts - origin) *
                             (360.0f / (8192.0f * MOTOR_GEAR_RATIO));
            motor_reference_deg += clamp(MOTOR_MOVE_DEG - motor_reference_deg,
                                         MOTOR_MAX_DEG_S * dt);
            float speed_deg_s = clamp(MOTOR_POSITION_KP *
                (motor_reference_deg - motor_angle_deg), MOTOR_MAX_DEG_S);
            float rpm_error = speed_deg_s * MOTOR_GEAR_RATIO / 6.0f - fb.speed_rpm;
            float candidate = clamp(integral + MOTOR_SPEED_KI * rpm_error * dt,
                                    MOTOR_CURRENT_LIMIT);
            float unsaturated = MOTOR_SPEED_KP * rpm_error + candidate;
            /* Integrate only when unsaturated or when unwinding saturation. */
            if (fabsf(unsaturated) <= MOTOR_CURRENT_LIMIT ||
                (unsaturated > MOTOR_CURRENT_LIMIT && rpm_error < 0.0f) ||
                (unsaturated < -MOTOR_CURRENT_LIMIT && rpm_error > 0.0f))
                integral = candidate;
            command = clamp(MOTOR_SPEED_KP * rpm_error + integral, MOTOR_CURRENT_LIMIT);
            if (fabsf(MOTOR_MOVE_DEG - motor_angle_deg) < 0.2f &&
                fabsf(MOTOR_MOVE_DEG - motor_reference_deg) < 0.001f &&
                fb.speed_rpm >= -3 && fb.speed_rpm <= 3)
            {
                if (settled_ms < 200U) settled_ms += elapsed;
                if (settled_ms >= 200U) motor_state = MOTOR_HOLDING;
            }
            else settled_ms = 0;
            if (motor_state == MOTOR_MOVING && now - move_ms > MOTOR_MOVE_TIMEOUT_MS)
                motor_state = MOTOR_FAULT;
        }
    }
    if (motor_state == MOTOR_FAULT)
    {
        command = integral = 0.0f;
        HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
    CANSetMotorCurrent(group, 1, (int16_t)command);
    if (CANTransmit(group) != HAL_OK)
    {
        motor_state = MOTOR_FAULT;
        HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
}
