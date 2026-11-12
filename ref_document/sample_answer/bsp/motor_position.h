#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H

/* Output shaft units; set ratio to 1.0f for bare rotor angle instead. */
#define MOTOR_GEAR_RATIO       19.0f
#define MOTOR_MOVE_DEG         10.0f
#define MOTOR_MAX_DEG_S        5.0f
#define MOTOR_POSITION_KP      3.0f
#define MOTOR_SPEED_KP         35.0f
#define MOTOR_SPEED_KI         80.0f
#define MOTOR_CURRENT_LIMIT    2000.0f
#define MOTOR_FEEDBACK_MS      50U
#define MOTOR_MOVE_TIMEOUT_MS  10000U

typedef enum { MOTOR_WAITING, MOTOR_MOVING, MOTOR_HOLDING, MOTOR_FAULT } MotorState;
extern volatile MotorState motor_state;
extern volatile float motor_angle_deg;
extern volatile float motor_reference_deg;
void MotorPositionInit(void);
void MotorPositionUpdate(void);
#endif
