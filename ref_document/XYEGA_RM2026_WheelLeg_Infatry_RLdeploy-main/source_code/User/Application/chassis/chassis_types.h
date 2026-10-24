//
// Created by Dell on 2026/1/5.
//

#ifndef RM2026_WHEELLEGGED_CHASSIS_CHASSIS_TYPES_H
#define RM2026_WHEELLEGGED_CHASSIS_CHASSIS_TYPES_H

#include "DJImotor.h"
#include "DMmotor.h"
#include "MIT.h"
#include "LQR.h"
#include "PID.h"
#include "data_hub.h"
#include "can_reflector.h"
#include "estimator.h"
#include "leg_solver.h"

namespace ega {
    enum class JointMode
    {
        DISABLE = 0,
        POSITION,
        SPEED,
        FORCE,
    };

    enum class WheelMode
    {
        DISABLE = 0,
        FORCE,
    };

    struct ControlMode final
    {
        JointMode joint_mode = JointMode::DISABLE;
        WheelMode wheel_mode = WheelMode::DISABLE;
    };

    struct Rc final
    {
        float vx = 0.0f;
        float vz = 0.0f;
        float roll = 0.0f;

        float v_leg_len = 0.0f;

        uint8_t mode = 0;
        uint8_t last_mode = 0;
    };

    struct Target final
    {
        ControlMode control_mode_ = {JointMode::DISABLE, WheelMode::DISABLE};

        float s = 0.0f;
        float s_dot = 0.0f;
        float yaw = 0.0f;
        float yaw_dot = 0.0f;

        float leg_len_l = 0.0f;
        float leg_len_r = 0.0f;
        float leg_theta_l = 0.0f;
        float leg_theta_r = 0.0f;

        float leg_theta_dot_l = 0.0f;
        float leg_theta_dot_r = 0.0f;
    };

    struct Feedback final
    {
        uint32_t now_ms = 0;

        float s = 0.0f;
        float s_dot = 0.0f;
        float yaw = 0.0f;
        float yaw_dot = 0.0f;
        float leg_theta_l = 0.0f;
        float leg_theta_dot_l = 0.0f;
        float leg_theta_r = 0.0f;
        float leg_theta_dot_r = 0.0f;
        float pitch = 0.0f;
        float pitch_dot = 0.0f;

        float roll = 0.0f;
        float roll_dot = 0.0f;

        float leg_len_l = 0.0f;
        float leg_len_r = 0.0f;
        float leg_len_l_dot = 0.0f;
        float leg_len_r_dot = 0.0f;
        float leg_len_avg = 0.0f;
        float leg_len_avg_dot = 0.0f;

        float Fn_l = 0.0f;
        float Fn_r = 0.0f;
        bool contact_l = true;
        bool contact_r = true;
    };

    struct FsmContext final
    {
        Feedback fdb;
        Target tgt;

        bool RL_flag = true;
        bool move_flag = false;
    };

    struct Actuation final
    {
        float wheel_tau_l = 0.0f;
        float wheel_tau_r = 0.0f;

        float joint_tau_left_thigh = 0.0f;
        float joint_tau_left_shank = 0.0f;
        float joint_tau_right_thigh = 0.0f;
        float joint_tau_right_shank = 0.0f;
    };

    struct Deps final
    {
        StateEstimator* estimator = nullptr;

        LQR* lqr = nullptr;

        PID* pid_len = nullptr;
        PID* pid_roll = nullptr;

        PID* pid_leg_speed_left = nullptr;
        PID* pid_leg_speed_right = nullptr;
        PID* pid_leg_speed_left_small = nullptr;
        PID* pid_leg_speed_right_small = nullptr;

        MIT* mit_pos_left_thigh = nullptr;
        MIT* mit_pos_left_shank = nullptr;
        MIT* mit_pos_right_thigh = nullptr;
        MIT* mit_pos_right_shank = nullptr;

        LegSolver* leg_left = nullptr;
        LegSolver* leg_right = nullptr;

        DMMotor* joint_left_thigh = nullptr;
        DMMotor* joint_left_shank = nullptr;
        DMMotor* joint_right_thigh = nullptr;
        DMMotor* joint_right_shank = nullptr;

        DJIMotor* wheel_left = nullptr;
        DJIMotor* wheel_right = nullptr;

    };
} // namespace ega

#endif //RM2026_WHEELLEGGED_CHASSIS_CHASSIS_TYPES_H
