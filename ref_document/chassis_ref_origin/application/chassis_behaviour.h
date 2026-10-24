/**
 * @file chassis_behaviour.h
 * @author ºÎÇå»ª
 * @brief
 * @version 0.1
 * @date 2022-03-17
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "chassis_task.h"

#define CHASSIS_WIDE 0.6f
#define CHASSIS_ROLL_DEADBAND 0.005f
#define CHASSIS_LEG_MAX 0.50f
#define CHASSIS_LEG_MIN 0.10f

extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move);
extern void chassis_behaviour_control_set(chassis_move_t *chassis_move_rc_to_vector);

#endif
