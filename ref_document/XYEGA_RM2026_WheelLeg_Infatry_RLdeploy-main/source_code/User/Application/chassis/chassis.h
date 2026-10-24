//
// Created by xiaodaoshi on 2026/01/31.
//

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include "DJIMotor.h"
#include "DMMotor.h"
#include "LQR.h"
#include "MIT.h"
#include "NoopLoop_tof.hpp"
#include "PID.h"
#include "chassis_types.h"
#include "data_hub.h"
#include "estimator.h"
#include "large_recover_fsm.h"
#include "leg_solver.h"
#include "motor.h"
#include "move_fsm.h"
#include "power_limiter.h"
#include "rl_policy.h"
#include "rls.h"
#include "user_configs.h"
#include "user_globals.h"
#include "user_parameters.h"
#include "velocity_planner.h"


namespace ega
{

    using namespace configs;
    using namespace globals;
    using namespace parameters;

    class Chassis final
    {
    public:
        // 定义控制策略枚
        enum class ControlStrategy {
            USE_RL,
            USE_LQR,
        };

        enum class RecoverRotateDirection
        {
            Positive,   // 角度增大方向
            Negative,   // 角度减小方向
        };

        enum class RLActionSource
        {
            OnboardNN,
            Usb,
        };

        struct RecoverTorqueConfig
        {
            float kp;
            float ki;
            float kd;
            float effort_limit;
            float integral_limit;
        };

        ControlStrategy current_strategy_ = ControlStrategy::USE_RL;

        struct Config final
        {
            float mass_body = BODY_MASS;
            float mass_leg = LEG_MASS;
            float mass_wheel = WHEEL_MASS;

            float dist_wheel = WHEEL_DIST;

            float leg_len_max = MAX_LEG_LEN;
            float leg_len_nor = NOR_LEG_LEN;
            float leg_len_min = MIN_LEG_LEN;
        };

        struct RlsParams
        {
            float delta = 1e-5f;
            float lambda = 0.99f;
        };

        struct PowerControlConfig
        {
            PowerLimiter<2>::Config wheel_config;
            RlsParams rls_params;
        };


        static constexpr uint8_t LEFT = 0;
        static constexpr uint8_t RIGHT = 1;

        static constexpr uint8_t LEFT_THIGH = 0;
        static constexpr uint8_t RIGHT_THIGH = 1;
        static constexpr uint8_t RIGHT_SHANK = 2;
        static constexpr uint8_t LEFT_SHANK = 3;
        static constexpr uint8_t RIGHT_WHEEL = 4;
        static constexpr uint8_t LEFT_WHEEL = 5;

        static constexpr uint8_t ACTION_FLOAT_COUNT = 6;
        static constexpr uint8_t OBS_FLOAT_COUNT = 19;
        static constexpr uint8_t OBS_FLAG_COUNT = 1;
        static constexpr uint16_t USB_TRANSMIT_SIZE = static_cast<uint16_t>(
            OBS_FLOAT_COUNT * sizeof(float) + OBS_FLAG_COUNT + 1
        );
        static constexpr uint16_t USB_RECEIVE_SIZE = static_cast<uint16_t>(
            1 + ACTION_FLOAT_COUNT * sizeof(float) + 1
        );

        static constexpr uint16_t max_chassis_power = 600;          // 软件底盘最大功率
        static constexpr uint16_t cap_threshold_up = 128;           // 超电电量线性插值上界
        static constexpr uint16_t cap_threshold_down = 30;          // 超电电量线性插值下界

        // RLS只使用低频、有效且基本稳态的功率样本，避免控制环重复消费同一帧超电数据。
        static constexpr uint32_t kRlsMinUpdateIntervalMs = 10U;
        static constexpr float kRlsMinMeasuredPower = 20.0f;
        static constexpr float kRlsMaxTorqueTrackingError = 1.0f;
        static constexpr float kRlsMaxParamStepRatio = 0.02f;



        static Chassis& getInstance();
        static void init();
        static void controlLoop();
        static void parseData(uint8_t* data, uint16_t len);

        static uint8_t getWheelOnlineMask();
        static bool Leg_OFFLINE_Protect();

        void reset()
        {
            target_ = {};
            fdb_ = {};
            act_ = {};
            last_output_efforts_ = {};
        }

        [[nodiscard]] const Target& target() const { return target_; }
        [[nodiscard]] const Feedback& feedback() const { return fdb_; }
        [[nodiscard]] const Actuation& actuation() const { return act_; }
        [[nodiscard]] float getLeftPhi0() const { return leg_kinematics_[LEFT].geometry.phi0; }
        [[nodiscard]] float getRightPhi0() const { return leg_kinematics_[RIGHT].geometry.phi0; }
        [[nodiscard]] float getLeftL0() const { return leg_kinematics_[LEFT].geometry.l0; }
        [[nodiscard]] float getRightL0() const { return leg_kinematics_[RIGHT].geometry.l0; }
        [[nodiscard]] ChassisPolicy getCurrentPolicy() const { return current_policy_; }
        // UI通信状态：UART回调收到合法TOF帧后置在线，连续1秒未更新后离线。
        // 与跳跃使用的距离/状态有效性判断(left_tof_valid_)相互独立。
        [[nodiscard]] bool isJumpTofOnline() const { return tof_.isOnline(LEFT); }
        [[nodiscard]] float getJumpTofDistance() const { return left_tof_measure_distance; }


        void debug_printf() const;

    private:
        Chassis()
        : rls_(balance_power_control_config.rls_params.delta,
               balance_power_control_config.rls_params.lambda),
          wheel_objs_(balance_power_control_config.wheel_config),
          vy_velocity_planner_(VelocityPlanner::Config{}),
          vz_velocity_planner_(VelocityPlanner::Config{})
        {}

        FsmContext ctx_;

        MoveFSM move_fsm_;

        uint16_t can_use_power_max = 90;

        float target_T_left;
        float target_T_right;
        Config cfg_{};
        Target target_{};
        Feedback fdb_{};
        Actuation act_{};
        Rc rc_{};

        // 估计器
        StateEstimator* estimator_ = nullptr;

        // 控制器
        LQR* lqr_ = nullptr;
        PID* pid_len_ = nullptr;
        PID* pid_roll_ = nullptr;
        PID* pid_leg_speed_left = nullptr;
        PID* pid_leg_speed_right = nullptr;
        PID* pid_leg_speed_left_small_ = nullptr;
        PID* pid_leg_speed_right_small_ = nullptr;

        MIT* mit_pos_left_thigh = nullptr;
        MIT* mit_pos_left_shank = nullptr;
        MIT* mit_pos_right_thigh = nullptr;
        MIT* mit_pos_right_shank = nullptr;

        // 解算器
        LegSolver* leg_left = nullptr;
        LegSolver* leg_right = nullptr;

        // 执行器
        DMMotor* joint_left_thigh = nullptr;
        DMMotor* joint_left_shank = nullptr;
        DMMotor* joint_right_thigh = nullptr;
        DMMotor* joint_right_shank = nullptr;
        DJIMotor* wheel_left = nullptr;
        DJIMotor* wheel_right = nullptr;

        //can转发
        CanReflector* canReflector2Ram_ = nullptr;
        CanReflector* canReflector2Gimbal_ = nullptr;

        static constexpr DMMotor::Config LEFT_SHANK_CONFIG
        {
            .direction = Motor::Direction::NORMAL,                  // 配置电机转向
            .can_handle = &hfdcan2,
            .can_tx_id = 0x01 + LEFT_SHANK,
            .can_rx_id = 0x11 + LEFT_SHANK,
            .p_max_abs = 3.1415928,
            .v_max_abs = 45.0,
            .t_max_abs = 54.0,
            .use_mit = false
        };

        static constexpr DMMotor::Config LEFT_THIGH_CONFIG
        {
            .direction = Motor::Direction::NORMAL,                  // 配置电机转向
            .can_handle = &hfdcan2,
            .can_tx_id = 0x01 + LEFT_THIGH,
            .can_rx_id = 0x11 + LEFT_THIGH,
            .p_max_abs = 3.14159,
            .v_max_abs = 45.0,
            .t_max_abs = 54.0,
            .use_mit = false
        };

        static constexpr DMMotor::Config RIGHT_SHANK_CONFIG
       {
           .direction = Motor::Direction::NORMAL,                  // 配置电机转向
           .can_handle = &hfdcan2,
           .can_tx_id = 0x01 + RIGHT_SHANK,
           .can_rx_id = 0x11 + RIGHT_SHANK,
           .p_max_abs = 3.14159,
           .v_max_abs = 45.0,
           .t_max_abs = 54.0,
           .use_mit = false
       };

        static constexpr DMMotor::Config RIGHT_THIGH_CONFIG
        {
            .direction = Motor::Direction::NORMAL,                  // 配置电机转向
            .can_handle = &hfdcan2,
            .can_tx_id = 0x01 + RIGHT_THIGH,
            .can_rx_id = 0x11 + RIGHT_THIGH,
            .p_max_abs = 3.14159,
            .v_max_abs = 45.0,
            .t_max_abs = 54.0,
            .use_mit = false
        };

        static constexpr DJIMotor::Config LEFT_WHEEL_CONFIG
        {
            .type = DJIMotor::Type::M3508,
            .direction = Motor::Direction::NORMAL,
            .can_handle = &hfdcan2,
            .motor_id = 6 - LEFT_WHEEL,
            .reduction_radio = DJIMotor::REDUCTION_RADIO_BEARPlus,
        };

        static constexpr DJIMotor::Config RIGHT_WHEEL_CONFIG
        {
            .type = DJIMotor::Type::M3508,
            .direction = Motor::Direction::NORMAL,
            .can_handle = &hfdcan2,
            .motor_id = 6 - RIGHT_WHEEL,
            .reduction_radio = DJIMotor::REDUCTION_RADIO_BEARPlus,
        };

        static constexpr PowerControlConfig balance_power_control_config = {
            .wheel_config = {
                .k1 = 0.152f,
                .k2 = 1.25f,
                .k3 = 0.9f,
                .measured_power_correction_ratio = 0.25f,
                .release_hysteresis = 5.0f,
            },
            .rls_params = {
                .delta =  1e-5f,
                .lambda = 0.99f
            },
        };

        static NoopLoopTof::Config makeTofConfig()
        {
            NoopLoopTof::Config config{};
            config.output_mode = ega::NoopLoopTof::OutputMode::Inquire;
            config.ids[0] = 0;
            config.ids[1] = 1;
            config.id_count = 2;
            config.offline_timeout_ms = 1000;
            return config;
        }

        NoopLoopTof tof_{makeTofConfig()};
        float left_tof_measure_distance = 0.0f;
        float right_tof_measure_distance = 0.0f;
        bool left_tof_valid_ = false;
        bool right_tof_valid_ = false;



        float tof_measure_average = 0.0f;
        float tof_measure_last = 0.0f;
        struct LegGeometry
        {
            float phi2 = 0.0f;
            float phi3 = 0.0f;
            float phi0 = 0.0f;
            float l0 = 0.0f;
        };

        struct ForceTorqueMap
        {
            float j11 = 0.0f;
            float j12 = 0.0f;
            float j21 = 0.0f;
            float j22 = 0.0f;
            float det = 0.0f;
            bool valid = false;
        };

        struct LegKinematics
        {
            // 五连杆解算使用的统一坐标系；右腿已在这里完成符号变换。
            float phi1 = 0.0f;
            float phi4 = 0.0f;
            float phi1_dot = 0.0f;
            float phi4_dot = 0.0f;
            LegGeometry geometry{};
            ForceTorqueMap force_torque_map{};
            // relative_phi3 对实体小腿、大腿电机角度的雅可比。
            std::array<float, 2> joint_jacobian{};
            float relative_phi3 = 0.0f;
            float relative_phi3_dot = 0.0f;
            float l0_dot = 0.0f;
            float phi0_dot = 0.0f;
            // LargeRecover 使用实体电机正方向一致的整腿角度坐标。
            float recover_phi = 0.0f;
            float recover_phi_dot = 0.0f;
        };

        std::array<LegKinematics, 2> leg_kinematics_{};

        void calculateRLFinalTorques();

        void sendData() const;

        std::array<float, RLPolicy::OBS_SIZE> buildRLObservation() const;
        void updateRLObservationHistory(const std::array<float, RLPolicy::OBS_SIZE>& obs);
        void updateRLActionsFromOnboardNN();

        void getRLMeasure();

        void updateLegKinematics();

        void updateProjectedGravity();

        void updateRLCMD();

        void updatePolicy();

        void calculateRLTorques();

        void applyPowerControl();
        void executeFinalEfforts();
        void limitFinalTorqueStep(std::array<float, 6>& target_efforts);
        float torqueDeltaToEffortDelta(std::size_t motor_index, float torque_delta) const;
        void applyEffortLowPass(std::array<float, 6>& target_efforts) const;
        void updateLargeRecoverFSM();
        void CalculateLargeRecoverTorque(float left_target_phi_vel,
                                         float right_target_phi_vel);

        ChassisPolicy mapMotionModeToPolicy(const ega::ChassisMotionMode motion_mode);

        static float clipf(float value, float lower, float upper);

        LegGeometry solveLegGeometry(float phi1, float phi4) const;
        std::array<float, 2> solveJaccobian(float phi1,
                                           float phi4,
                                           const LegGeometry& geometry) const;
        ForceTorqueMap buildForceTorqueMap(float phi1,
                                           float phi4,
                                           const LegGeometry& geometry) const;

        std::array<float,2> controlled_wheel_torques_ = {};

        std::array<std::unique_ptr<Motor>, 6> motors_{};
        std::array<float, ACTION_FLOAT_COUNT> actions_{};
        std::array<float, 6> last_actions_{};
        static constexpr std::size_t kSpinActionDelayCycles =1U;
        std::array<std::array<float, 6>, kSpinActionDelayCycles> spin_action_delay_fifo_{};
        std::size_t spin_action_delay_index_ = 0U;
        bool spin_action_delay_active_ = false;
        std::array<float, RLPolicy::OBS_HISTORY_SIZE> obs_history_{};
        bool obs_history_initialized_ = false;
        bool rl_policy_ready_ = false;
        bool rl_policy_fault_ = false;
        RLActionSource rl_action_source_ = RLActionSource::OnboardNN;
        // RLActionSource rl_action_source_ = RLActionSource::Usb;
        std::array<float, 6> q_{};
        std::array<float, 6> qd_{};
        std::array<float, 6> RL_torques_{};
        std::array<float, 6> Last_RL_torques_{};
        std::array<float, 6> LQR_torques_{};
        std::array<float, 6> last_output_efforts_{};
        std::array<float, 6> tau_virtual_{};
        std::array<float, 6> target_efforts{};
        std::array<float, 6> target_torques{};
        RLS<2> rls_;
        PowerLimiter<2> wheel_objs_;
        VelocityPlanner vy_velocity_planner_;
        VelocityPlanner vz_velocity_planner_;
        uint32_t last_rls_update_ms_ = 0U;
        uint32_t last_rls_feedback_sequence_ = 0U;
        bool rls_power_sample_initialized_ = false;

        std::array<float, 2> thigh_angle_{};
        std::array<float, 2> thigh_vel_{};
        std::array<float, 2> shank_angle_{};
        std::array<float, 2> shank_vel_{};
        std::array<float, 2> wheel_pos_{};
        std::array<float, 2> wheels_vel_rad_{};

        std::array<float, 4> quaternions_{};
        std::array<float, 3> projected_g_{};
        std::array<float, 3> gyros_{};
        std::array<float, 3> command_ = {0.0f, 0.0f, 0.20f};
        std::array<float, 3> g_world_ = {0.0f, 0.0f, -1.0f};
        float left_large_recover_phi_vel_integral_ = 0.0f;
        float right_large_recover_phi_vel_integral_ = 0.0f;

        float left_pitch_protect_vel_integral_ = 0.0f;//翻倒时维持电机角度，防止翻过头
        float right_pitch_protect_vel_integral_ = 0.0f;

        ega::LargeRecoverFSM large_recover_fsm_;
        float RecoverStopAngleLEFT = 0.0f;
        float RecoverStopAngleRIGHT = 0.0f;
        float RecoverPhiTargetLEFT = 0.0f;
        float RecoverPhiTargetRIGHT = 0.0f;
        RecoverRotateDirection  Recover_Direction_LEFT = RecoverRotateDirection::Negative;
        RecoverRotateDirection  Recover_Direction_RIGHT = RecoverRotateDirection::Positive;
        bool left_large_recover_arrived_ = false;
        bool right_large_recover_arrived_ = false;
        bool allow_left_leg_revover = false;
        bool allow_right_leg_revover = false;

        static constexpr bool kEnableEffortLowPass = false;
        static constexpr float kEffortLowPassAlpha = 0.9f;

        int loop_count_ = 0;
        uint8_t first_into_LQR = false;

        float jump_policy_phase_ = 0.0f;      // 当前处于跳跃保持时间中的比例，范围 0~1
        float jump_hold_linear_command_ = 0.0f;
        uint32_t jump_start_ms_ = 0U;
        bool post_jump_height_hold_active_ = false;
        uint32_t post_jump_height_hold_start_ms_ = 0U;

        ChassisPolicy current_policy_ = ChassisPolicy::Stop;
        ChassisPolicy last_policy_ = ChassisPolicy::Stop;
        ChassisMotionMode last_motion_mode_ = ChassisMotionMode::Stop;

        float l1_ = 0.175f;
        float l2_ = 0.208f;
    };
} // namespace ega
