/* Isolated diagnostic encoder ceiling. Normal RL still compiles user/motor_io.c
 * directly with its original 20 Nm cap. Mapping, Kt and wire encoding are shared. */
#include "controller.h"
#include "../../user/robot_config.h"
#undef ROBOT_LEG_TORQUE_CAP_NM
#define ROBOT_LEG_TORQUE_CAP_NM RETRACT_CAP_NM
#include "../../user/motor_io.c"
