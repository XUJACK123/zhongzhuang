// Created by xiaodaoshi on 2026/01/31.
//

#include "chassis.h"

#include <cmath>
#include <cstring>

#include "DJImotor.h"
#include "DMMotor.h"
#include "driver_usb.h"
#include "robot_manager.h"
#include "referee.h"
#include "scap.h"
#include "math_ht.h"
#include "chassis_uitils.h"

namespace ega
{
    using namespace configs;
    using namespace globals;
    using namespace parameters;

    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kDegToRad = kPi / 180.0f;
    constexpr uint32_t kAuxiliaryCanTxPeriodMs = 50U; // Keep the original 200Hz CAN rate

    // 变速小陀螺开关及参数。云台下发的vz保持不变，在底盘Spin模式下仅对
    // command_[1]施加正弦倍率；倍率始终为正，因此不会改变旋转方向。
    constexpr bool kEnableSinusoidalSpin = true;
    constexpr float kSinusoidalSpinFrequencyHz = 0.4f;
    constexpr float kSinusoidalSpinMinScale = 0.6f;
    constexpr float kSinusoidalSpinMaxScale = 1.0f;
    constexpr uint32_t kSinusoidalSpinMaxSpeedHoldMs = 500U;

    constexpr float kTorqueEpsilon = 1.0e-5f;
    constexpr float kMinSin = 1.0e-4f;
    constexpr float kMinDet = 1.0e-6f;
    constexpr float kMinLegLength = 0.01f;
    constexpr float kDefaultGasSpringCompensationLeft = 370.1f;
    constexpr float kDefaultGasSpringCompensationRight = 370.1f;
    constexpr float kSpinGasSpringCompensationLeft = 270.1f;
    constexpr float kSpinGasSpringCompensationRight = 300.1f;
    constexpr float kRealTorqueLimit = 100.0f;

    constexpr float kClipActions = 100.0f;
    constexpr float kPosActionScale = 0.5f;
    constexpr float kVelActionScale = 10.0f;

    constexpr float kLinVelLimit = 2.0f;
    constexpr float kYawVelLimit =11.0f;
    constexpr float kHeightMin = 0.10f;
    constexpr float kHeightMax = 0.30f;
    constexpr float kProtectHeight = 0.12f;

    constexpr float kLargeRecoverThighVelKp = 6.0f;
    constexpr float kLargeRecoverThighVelKi = 6.0f;
    constexpr float kLargeRecoverPhiVelKp = 12.0f;
    constexpr float kLargeRecoverPhiVelKi = 12.0f;

    constexpr float kRecoverPhiPosToVelKp = 100.0f;
    constexpr float kRecoverPhiMaxTargetVel = 4.0f;

    constexpr float kLargeRecoverVelocityIntegralLimit = 30.0f;//积分最大值
    constexpr float kLargeRecoverThighTorqueLimit = 50.0f;//输出最大值

    // LargeRecover 中用虚拟腿长力抵抗五连杆被车身压缩。
    // 左右腿始终跟踪同一个固定腿长目标。
    constexpr float kLargeRecoverLegLengthTarget = 0.342f;
    constexpr float kLargeRecoverLegLengthKp = 2000.0f;
    constexpr float kLargeRecoverLegLengthKd = 60.0f;
    constexpr float kLargeRecoverLegForceLimit = 100.0f;
    constexpr float kLargeRecoverStallVelThreshold = 0.10f;
    constexpr float kLargeRecoverVirtualTorqueLimit = 80.0f;

    inline constexpr float kLargeRecoverIntegralClearAngle = 0.03f;

    constexpr float kLargeRecoverYawReadyDeg = 8.0f;       // yaw_err 小于这个，两条腿都可以动，单位 deg
    constexpr float kLargeRecoverRollTouchDeadband = 0.10f; // roll 判断哪条腿触地，单位 rad

    constexpr float kLargeRecoverFinalPitchAbs = 0.50f;    // pitch 在 [-0.5, 0.5]，认为进入最终可回正区
    constexpr float kLargeRecoverBigPitchAbs = 1.20f;      // 大 pitch 分支阈值

    constexpr float kLargeRecoverPoseTol = 0.25f;          // 判断 thigh 是否已经到某个准备姿态
    constexpr float kLargeRecoverNormalStopAngle = 0.18f;  // 普通恢复动作死区
    constexpr float kLargeRecoverFinalStopAngle = 0.60f;   // 最终站立姿态死区，大一点

    constexpr uint32_t kJumpDurationMs = 480U;
    constexpr float kJumpPhase1_noScale = 0.1f;
    constexpr float kJumpPhase2_putScale = 0.4f;
    constexpr float kJumpFTPScale = 1.0f;
    constexpr bool kEnableJumpTofTrigger = true; // 调试开关：false 时收到 Jump 后立即起跳
    constexpr float kJumpTofTriggerDistanceM = 0.9f;
    constexpr float kJumpTofMinValidDistanceM = 0.05f;
    constexpr float kJumpTofMaxValidDistanceM = 5.0f;
    constexpr uint32_t kTofLogPeriodMs = 100U;
    constexpr float kPostJumpHeightM = 0.25f;
    constexpr uint32_t kPostJumpHeightHoldMs = 1000U;
    bool jump_prepare = false;
    bool jump_active = false;

    constexpr float kFinalTorqueStepLimit = 10.0f;

    constexpr float kSerialTorqueLimit =   1000.0f;
    constexpr float kParallelTorqueLimit = 35.0f;
    constexpr float kWheelTorqueLimit = 5.0f;
    constexpr float kJumpWheelTorqueLimit = 4.0f;

    constexpr int kDofLw = 2;
    constexpr int kDofLf0 = 0;
    constexpr int kDofLf1 = 1;
    constexpr int kDofRf0 = 3;
    constexpr int kDofRf1 = 4;
    constexpr int kDofRw = 5;

    constexpr std::array<float, 6> kDefaultDofPosStable = {-0.23, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f};//先和minirecover一样
    constexpr std::array<float, 4> kDefaultObsDofPosStable = {-0.23f, -0.65f, 0.23f, 0.65f};
    constexpr std::array<float, 6> kPGainsStable = {15.0f, 15.0f, 0.0f, 15.0f, 15.0f, 0.0f};
    constexpr std::array<float, 6> kDGainsStable = {1.0f, 1.0f, 0.1f, 1.0f, 1.0f, 0.1f};
    // constexpr std::array<float, 6> kDefaultDofPosStable =  {0.2f, 0.4f, 0.0f, -0.2f, -0.4f, 0.0f};//区域赛stable
    // constexpr std::array<float, 4> kDefaultObsDofPosStable = {0.2f, 0.4f, -0.2f, -0.4f};
    // constexpr std::array<float, 6> kPGainsStable = {20.0f, 20.0f, 0.0f, 20.0f, 20.0f, 0.0f};
    // constexpr std::array<float, 6> kDGainsStable = {1.0f, 1.0f, 0.2f, 1.0f, 1.0f, 0.2f};


    constexpr std::array<float, 6> kDefaultDofPosMiniRecover = {-0.23, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f};
    // // constexpr std::array<float, 6> kDefaultDofPosMiniRecover = {-0.35, -0.55f, 0.0f, 0.35f, 0.55f, 0.0f};
    constexpr std::array<float, 4> kDefaultObsDofPosMiniRecover = {-0.23f, -0.65f, 0.23f, 0.65f};
    // // constexpr std::array<float, 4> kDefaultObsDofPosMiniRecover = {-0.35f, -0.55f, 0.35f, 0.55f};
    constexpr std::array<float, 6> kPGainsMiniRecover = {15.0f, 15.0f, 0.0f, 15.0f, 15.0f, 0.0f};
    // // constexpr std::array<float, 6> kPGainsMiniRecover = {60.0f, 60.0f, 0.0f, 60.0f, 60.0f, 0.0f};
    constexpr std::array<float, 6> kDGainsMiniRecover = {1.0f, 1.0f, 0.1f, 1.0f, 1.0f, 0.1f};
    // // constexpr std::array<float, 6> kDGainsMiniRecover = {1.0f, 1.0f, 0.2f, 1.0f, 1.0f, 0.2f};
    // constexpr std::array<float, 6> kDefaultDofPosMiniRecover =  {0.2f, 0.4f, 0.0f, -0.2f, -0.4f, 0.0f};//区域赛stable
    // constexpr std::array<float, 4> kDefaultObsDofPosMiniRecover = {0.2f, 0.4f, -0.2f, -0.4f};
    // constexpr std::array<float, 6> kPGainsMiniRecover = {20.0f, 20.0f, 0.0f, 20.0f, 20.0f, 0.0f};
    // constexpr std::array<float, 6> kDGainsMiniRecover = {1.0f, 1.0f, 0.2f, 1.0f, 1.0f, 0.2f};

    constexpr std::array<float, 6> kDefaultDofPosSpin = {-0.23, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f};
    constexpr std::array<float, 4> kDefaultObsDofPosSpin = {-0.23f, -0.65f, 0.23f, 0.65f};
    constexpr std::array<float, 6> kPGainsSpin = {10.0f, 10.0f, 0.0f, 10.0f, 10.0f, 0.0f};
    constexpr std::array<float, 6> kDGainsSpin = {1.0f, 1.0f, 0.1f, 1.0f, 1.0f, 0.1f};


    // constexpr std::array<float, 6> kDefaultDofPosSpin = {0.2f, 0.4f, 0.0f, -0.2f, -0.4f, 0.0f};
    // constexpr std::array<float, 4> kDefaultObsDofPosSpin = {0.2f, 0.4f, -0.2f, -0.4f};
    // constexpr std::array<float, 6> kPGainsSpin = {20.0f, 20.0f, 0.0f, 20.0f, 20.0f, 0.0f};
    // constexpr std::array<float, 6> kDGainsSpin = {1.0f, 1.0f, 0.2f, 1.0f, 1.0f, 0.2f};

    constexpr std::array<float, 6> kDefaultDofPosJump = {0.2f, 0.4f, 0.0f, -0.2f, -0.4f, 0.0f};
    constexpr std::array<float, 4> kDefaultObsDofPosJump = {0.2f, 0.4f, -0.2f, -0.4f};
    // constexpr std::array<float, 6> kPGainsJump = {10.0f, 10.0f, 0.0f, 10.0f, 10.0f, 0.0f};
    // constexpr std::array<float, 6> kPGainsJump = {20.0f, 20.0f, 0.0f, 20.0f, 20.0f, 0.0f};
    constexpr std::array<float, 6> kPGainsJump = {6.0f, 6.0f, 0.0f, 6.0f, 6.0f, 0.0f};
    // constexpr std::array<float, 6> kDGainsJump = {1.0f, 1.0f, 0.25f, 1.0f, 1.0f, 0.25f};
    // constexpr std::array<float, 6> kDGainsJump = {1.0f, 1.0f, 0.2f, 1.0f, 1.0f, 0.2f};
    constexpr std::array<float, 6> kDGainsJump = {0.5f, 0.5f, 0.2f, 0.5f, 0.5f, 0.2f};


    constexpr std::array<float, 3> kGyroScale = {0.25f, 0.25f, 0.25f};
    constexpr std::array<float, 3> kGravityScale = {1.0f, 1.0f, 1.0f};
    // constexpr std::array<float, 3> kCommandScaleStable = {3.0f, 0.25f, 5.0f};
    constexpr std::array<float, 3> kCommandScaleStable = {3.0f, 0.25f, 5.0f};
    constexpr std::array<float, 3> kCommandScaleMiniRecover = {3.0f, 0.25f, 5.0f};
    // constexpr std::array<float, 3> kCommandScaleMiniRecover = {2.0f, 0.25f, 5.0f};
    constexpr std::array<float, 3> kCommandScaleSpin = {2.0f, 0.25f, 5.0f};
    constexpr std::array<float, 3> kCommandScaleJump = {3.0f, 0.25f, 5.0f};

    constexpr std::array<float, 6> kObservedVelScale = {0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f};

    struct RecoverTorqueConfig
    {
        float kp;
        float ki;
        float kd;
        float effort_limit;
        float integral_limit;
    };

    // 非 LargeRecover 的 pitch 保护，以及五连杆映射失效时的单大腿降级控制。
    constexpr RecoverTorqueConfig kLargeRecoverThighVelocityConfig = {
        .kp = kLargeRecoverThighVelKp,
        .ki = kLargeRecoverThighVelKi,
        .kd = 0.0f,
        .effort_limit = kLargeRecoverThighTorqueLimit,
        .integral_limit = kLargeRecoverVelocityIntegralLimit,
    };

    // j22 实测为 0.5，因此 12/12 的虚拟腿角速度 PI 等效于原大腿速度 PI 6/6。
    constexpr RecoverTorqueConfig kLargeRecoverPhiVelocityConfig = {
        .kp = kLargeRecoverPhiVelKp,
        .ki = kLargeRecoverPhiVelKi,
        .kd = 0.0f,
        .effort_limit = kLargeRecoverVirtualTorqueLimit,
        .integral_limit = kLargeRecoverVelocityIntegralLimit,
    };

    float computeDirectedAngleError(const float target_angle,
                                    const float current_angle,
                                    const Chassis::RecoverRotateDirection direction)
    {
        const float raw_error = target_angle - current_angle;

        if (direction == Chassis::RecoverRotateDirection::Positive)
        {
            // 只允许往角度增大的方向走，误差范围 [0, 2pi)
            return wrapTo2Pi(raw_error);
        }
        else
        {
            // 只允许往角度减小的方向走，误差范围 (-2pi, 0]
            return -wrapTo2Pi(-raw_error);
        }
    }

    float computeRecoverVelocityTorque(const float target_vel,
                               const float current_vel,
                               const RecoverTorqueConfig& config,
                               float& integral_state)
    {
        const float error = target_vel - current_vel;

        if (config.ki > 0.0f && config.integral_limit > 0.0f)
        {
            integral_state += error * CONTROL_PERIOD;
            integral_state = clampSymmetric(integral_state, config.integral_limit);
        }
        else
        {
            integral_state = 0.0f;
        }

        const float effort = config.kp * error + config.ki * integral_state;
        return clampSymmetric(effort, config.effort_limit);
    }

    float computeRecoverVelocityTorqueNoStallWindup(
        const float target_vel,
        const float current_vel,
        const RecoverTorqueConfig& config,
        float& integral_state)
    {
        const float error = target_vel - current_vel;
        const bool moving_in_target_direction =
            target_vel * current_vel > 0.0f &&
            fabsf(current_vel) >= kLargeRecoverStallVelThreshold;

        // 堵转或目标速度为 0 时不允许积分蓄能。
        // 腿已经沿目标方向运动后，才用积分补偿稳态速度误差。
        if (config.ki > 0.0f &&
            config.integral_limit > 0.0f &&
            moving_in_target_direction)
        {
            integral_state += error * CONTROL_PERIOD;
            integral_state = clampSymmetric(integral_state, config.integral_limit);
        }
        else
        {
            integral_state = 0.0f;
        }

        const float effort = config.kp * error + config.ki * integral_state;
        return clampSymmetric(effort, config.effort_limit);
    }

    static bool isRecoverArrivedByDirection(float current,
                                        float target,
                                        Chassis::RecoverRotateDirection direction,
                                        float near_tolerance,
                                        float passed_tolerance)
    {
        // 最近角度误差：target - current，范围 [-pi, pi]
        const float nearest_err = wrapToPi(target - current);

        // 原来的 near 逻辑仍然保留：
        // 如果已经在目标附近，不管从哪边接近，都认为 arrived。
        if (fabsf(nearest_err) < near_tolerance)
        {
            return true;
        }

        // target_ahead 表示：
        // 按当前指定方向走，目标还在不在前方。
        //
        // Positive：角度增大方向
        //   target_ahead > 0 ：目标还在前方
        //   target_ahead < 0 ：已经越过目标
        //
        // Negative：角度减小方向
        //   current - target > 0 ：目标还在负方向前方
        //   current - target < 0 ：已经沿负方向越过目标
        const float target_ahead =
            direction == Chassis::RecoverRotateDirection::Positive
                ? wrapToPi(target - current)
                : wrapToPi(current - target);

        const float passed_amount = -target_ahead;

        // 只有“已经越过目标”并且“越过不多”才认为 arrived。
        return passed_amount >= 0.0f && passed_amount < passed_tolerance;
    }

    float computeRecoverPhiTargetVelocityDirectedThenNear(
        const float target_phi,
        const float current_phi,
        const Chassis::RecoverRotateDirection direction,
        const float pos_kp,
        const float max_target_vel,
        const float stop_angle,
        bool& arrived)
    {
        // near_error 只表示“离目标最近还有多远”
        const float near_error = wrapToPi(target_phi - current_phi);

        // 第一次进入目标范围后，锁定 arrived
        if (fabsf(near_error) < stop_angle)
        {
            arrived = true;
        }

        float error = 0.0f;

        if (!arrived)
        {
            // 还没到过目标：强制指定方向
            error = computeDirectedAngleError(target_phi, current_phi, direction);
        }
        else
        {
            // 已经到过目标：改成最近角度保持，防止过线后再转一整圈
            error = near_error;
        }

        // 到目标附近，目标速度直接归零
        if (fabsf(error) < stop_angle)
        {
            return 0.0f;
        }

        const float target_vel = pos_kp * error;
        return clampSymmetric(target_vel, max_target_vel);
    }

    constexpr ega::ChassisPolicy kDefaultPolicy = ega::ChassisPolicy::Stop;

    const std::array<float, 6>& getDefaultDofPos(const ega::ChassisPolicy current_policy)
    {
        if (current_policy == ega::ChassisPolicy::Stable)
        {
            return kDefaultDofPosStable;
        }
        if (current_policy == ega::ChassisPolicy::Spin)
        {
            return kDefaultDofPosSpin;
        }
        if (current_policy == ega::ChassisPolicy::Jump)
        {
            return kDefaultDofPosJump;
        }
        return kDefaultDofPosMiniRecover;
    }

    const std::array<float, 4>& getDefaultObsDofPos(const ega::ChassisPolicy current_policy)
    {
        if (current_policy == ega::ChassisPolicy::Stable)
        {
            return kDefaultObsDofPosStable;
        }
        if (current_policy == ega::ChassisPolicy::Spin)
        {
            return kDefaultObsDofPosSpin;
        }
        if (current_policy == ega::ChassisPolicy::Jump)
        {
            return kDefaultObsDofPosJump;
        }
        return kDefaultObsDofPosMiniRecover;
    }

    const std::array<float, 6>& getPGains(const ega::ChassisPolicy current_policy)
    {
        if (current_policy == ega::ChassisPolicy::Stable)
        {
            return kPGainsStable;
        }
        if (current_policy == ega::ChassisPolicy::Spin)
        {
            return kPGainsSpin;
        }
        if (current_policy == ega::ChassisPolicy::Jump)
        {
            return kPGainsJump;
        }
        return kPGainsMiniRecover;
    }

    const std::array<float, 6>& getDGains(const ega::ChassisPolicy current_policy)
    {
        if (current_policy == ega::ChassisPolicy::Stable)
        {
            return kDGainsStable;
        }
        if (current_policy == ega::ChassisPolicy::Spin)
        {
            return kDGainsSpin;
        }
        if (current_policy == ega::ChassisPolicy::Jump)
        {
            return kDGainsJump;
        }
        return kDGainsMiniRecover;
    }

    const std::array<float, 3>& getCommandScale(const ega::ChassisPolicy current_policy)
    {
        if (current_policy == ega::ChassisPolicy::Stable)
        {
            return kCommandScaleStable;
        }
        if (current_policy == ega::ChassisPolicy::Spin)
        {
            return kCommandScaleSpin;
        }
        if (current_policy == ega::ChassisPolicy::Jump)
        {
            return kCommandScaleJump;
        }
        return kCommandScaleMiniRecover;
    }

    uint8_t encodePolicyForUsb(const ega::ChassisPolicy policy)
    {
        if (policy == ega::ChassisPolicy::Stable)
        {
            return static_cast<uint8_t>(ega::RLMapPolicyToPc::Stable);
        }
        if (policy == ega::ChassisPolicy::Spin)
        {
            return static_cast<uint8_t>(ega::RLMapPolicyToPc::Spin);
        }
        if (policy == ega::ChassisPolicy::Jump)
        {
            return static_cast<uint8_t>(ega::RLMapPolicyToPc::Jump);
        }
        return static_cast<uint8_t>(ega::RLMapPolicyToPc::MiniRecover);
    }

    ega::ChassisPolicy Chassis::mapMotionModeToPolicy(const ega::ChassisMotionMode motion_mode)
    {
        switch (motion_mode)
        {
        case ega::ChassisMotionMode::Stop:
            return ega::ChassisPolicy::Stop;
        case ega::ChassisMotionMode::Stable:
            return ega::ChassisPolicy::Stable;
        case ChassisMotionMode::LargeRecover_Auto:
            return ChassisPolicy::LargeRecover_Auto;
        case ChassisMotionMode::LargeRecover_Crazy:
            return ChassisPolicy::LargeRecover_Crazy;
        case ega::ChassisMotionMode::Jump:
            return ega::ChassisPolicy::Jump;
        case ega::ChassisMotionMode::Spin:
            return ega::ChassisPolicy::Spin;
        case ega::ChassisMotionMode::MiniRecover:
        case ega::ChassisMotionMode::Walk:
        default:
            return ega::ChassisPolicy::MiniRecover;
        }
    }

    void Chassis::controlLoop()
    {
        auto& chassis = getInstance();
        const auto& hub = RobotManager::getDataHubConst();
        chassis.tof_.inquire();

        // chassis.tof_.inquire(1);
        // chassis.tof_.inquire(1);
        // const auto tof_measure = chassis.tof_.getMeasure(1);
        // logger_printf("%.3f\r\n",tof_measure->distance);
        /// 超电部分，后面整理
        const uint16_t referee_power_buffer = Referee::Referee_GetPowerHeat()->chassis_power_buffer;
        const uint16_t referee_power_limit = Referee::Referee_GetRobotState()->chassis_power_limit;
        chassis.can_use_power_max = Scap::getChassisPowerLimit();
        Scap::setRefereePowerLimit(referee_power_limit);
        Scap::setRefereeEnergyBuffer(referee_power_buffer);
        static uint32_t next_cap_tx_ms = HAL_GetTick();
        const uint32_t now_ms = HAL_GetTick();
        if (static_cast<int32_t>(now_ms - next_cap_tx_ms) >= 0)
        {
            Scap::sendCapInfo();
            next_cap_tx_ms += kAuxiliaryCanTxPeriodMs;
            if (static_cast<int32_t>(now_ms - next_cap_tx_ms) >= 0)
            {
                next_cap_tx_ms = now_ms + kAuxiliaryCanTxPeriodMs;
            }
        }
        // logger_printf("%.3f,%.3f,%.3f,%.3f,%3f,%.3f,%.3f\r\n",chassis.motors_[LEFT_SHANK]->getMeasure().angle_rotor,chassis.motors_[RIGHT_SHANK]->getMeasure().angle_rotor,chassis.motors_[LEFT_THIGH]->getMeasure().angle_rotor,chassis.motors_[RIGHT_THIGH]->getMeasure().angle_rotor,chassis.motors_[LEFT_WHEEL]->getMeasure().speed,chassis.motors_[RIGHT_WHEEL]->getMeasure().speed,Scap::getCapEnergy());
        // logger_printf("%.3f,%.3f,%d,%d\r\n",hub.command.vy,hub.command.vz,hub.gimbal.gimbal_online,hub.command.chassis_motion_mode);
        // logger_printf("%.3f,%.3f,%.3f\r\n",referee_power_buffer,referee_power_limit,can_use_power_max);
        chassis.loop_count_++;

        chassis.getRLMeasure();  //read   检查完毕了    极性都没问题
        chassis.updateLegKinematics();
        chassis.updateProjectedGravity();
        chassis.updateRLCMD();
        chassis.updatePolicy();
        if (chassis.rl_action_source_ == RLActionSource::OnboardNN && chassis.rl_policy_fault_)
        {
            RobotManager::getDataHub().work_state = WorkState::Protect;
        }
        // if(chassis.command_[2]>0.15f)
        // {
        //     chassis.command_[2] = 0.225f;
        // }

        if (chassis.loop_count_ % 5U == 0U)
        {
            switch (chassis.rl_action_source_)
            {
            case RLActionSource::OnboardNN:
                chassis.updateRLActionsFromOnboardNN();
                chassis.sendData();
                break;
            case RLActionSource::Usb:
                chassis.sendData();
                break;
            }
        logger_printf("%d,%d,%d,%d,%d,"
                      "%d,%d,%d,%d,%d,"
                      "%d,%d,%d,%d,%d,"
                      "%d,%d,%d,%d,%d,"
                      "%d,%d,%d,%d,%d\r\n",
                      hub.enemy_info.hero.enemy_hp_ratio,
                      hub.enemy_info.hero.allowed_bullet_count,
                      hub.enemy_info.hero.defense_buff,
                      hub.enemy_info.hero.negative_defense_buff,
                      hub.enemy_info.hero.targetable,
                      hub.enemy_info.engineer.enemy_hp_ratio,
                      hub.enemy_info.engineer.allowed_bullet_count,
                      hub.enemy_info.engineer.defense_buff,
                      hub.enemy_info.engineer.negative_defense_buff,
                      hub.enemy_info.engineer.targetable,
                      hub.enemy_info.infantry_3.enemy_hp_ratio,
                     hub.enemy_info.infantry_3.allowed_bullet_count,
                     hub.enemy_info.infantry_3.defense_buff,
                     hub.enemy_info.infantry_3.negative_defense_buff,
                     hub.enemy_info.infantry_3.targetable,
                     hub.enemy_info.infantry_4.enemy_hp_ratio,
                     hub.enemy_info.infantry_4.allowed_bullet_count,
                     hub.enemy_info.infantry_4.defense_buff,
                     hub.enemy_info.infantry_4.negative_defense_buff,
                     hub.enemy_info.infantry_4.targetable,
                     hub.enemy_info.sentry.enemy_hp_ratio,
                     hub.enemy_info.sentry.allowed_bullet_count,
                     hub.enemy_info.sentry.defense_buff,
                     hub.enemy_info.sentry.negative_defense_buff,
                     hub.enemy_info.sentry.targetable);
        }
        chassis.calculateRLTorques();
        chassis.calculateRLFinalTorques();

        // chassis.updateFeedback(chassis.ctx_);
        // chassis.switchRLorLQR();
        // chassis.updateController(chassis.ctx_);
        // chassis.calculateLQRFinalTorques();
        chassis.executeFinalEfforts();
        chassis.debug_printf();
        // logger_printf("%.3f\r\n",chassis.left_tof_measure_distance);
        // logger_printf("%.3f,%d\r\n",tof_measure.distance,tof_measure.status);
        // logger_printf("%.3f,%.3f,%.3f,%.3f\r\n",chassis.thigh_angle_[LEFT],chassis.thigh_angle_[RIGHT],hub.chassis.chassis_pitch,hub.chassis.chassis_roll);
        // logger_printf("%.3f,%.3f,%.3f\r\n",chassis.gyros_[0],chassis.gyros_[1],chassis.gyros_[2]);
    }

    uint8_t Chassis::getWheelOnlineMask()
    {
        auto& chassis = getInstance();
        if (!chassis.motors_[LEFT_WHEEL] || !chassis.motors_[RIGHT_WHEEL])
        {
            return 0;
        }

        return static_cast<uint8_t>(chassis.motors_[LEFT_WHEEL]->isOnline()) |
               static_cast<uint8_t>(static_cast<uint8_t>(chassis.motors_[RIGHT_WHEEL]->isOnline()) << 1);
    }

    bool Chassis::Leg_OFFLINE_Protect() {
        auto& chassis = getInstance();
        if (!chassis.motors_[LEFT_THIGH]->isOnline() && !chassis.motors_[RIGHT_THIGH]->isOnline() && !chassis.motors_[LEFT_SHANK]->isOnline() && !chassis.motors_[RIGHT_SHANK]->isOnline())
        {
            return true;
        }
        return false;
    }

    void Chassis::applyPowerControl()
    {
        // 数据中心同时提供只读命令和可写底盘状态：前者用于选择功率模式，
        // 后者用于记录本周期最终采用的功率上限。
        const auto & data_hub_ = RobotManager::getDataHubConst();
        auto & chassis = RobotManager::getDataHub().chassis;

        // 默认功率仅作为异常枚举值的兜底；Normal/Fast 分支都会重新赋值。
        float total_usable_power = 85.0f;

        // 只有通信数据可信且超电输出已开启时，才使用超电反馈参与功控。
        const bool is_cap_trustable =
            Scap::isCapTrustable() && Scap::isCapOutputEnabled();     //connected = 0 或 error = 1 或 output = 0

        // trust       = 0
        // connected   = 0    因此是connected的问题   在scap.cpp中的rxCallback()中处理connected   10ms检查一次  若500ms没发收到新帧则no connected  Daemon::Config中配置
        // error       = 0
        // output      = 1
        // status      = 0xD4
        // error_level = 0
        // sequence    = 13901
        // 裁判系统缓冲能量(J)、超电剩余能量(0~255)、留出2%裕量后的
        // 裁判功率上限(W)，以及超电报告的底盘最大安全功率(W)。
        const uint16_t referee_energy_buffer = Referee::referee_recv_mesg.powerHeat.chassis_power_buffer;
        const uint8_t cap_energy = Scap::getCapEnergy();
        const float referee_power_max = static_cast<float>(Referee::referee_recv_mesg.robotState.chassis_power_limit) * 0.98f;
        const float chassis_power_limit = Scap::getChassisPowerLimit();
        if (is_cap_trustable)
        {
            // 超电在线时，Normal和Fast使用同一套功率计算逻辑。
            if (cap_energy >= cap_threshold_up)
            {
                total_usable_power =
                    fminf(max_chassis_power, chassis_power_limit);
            }
            else if (cap_energy >= cap_threshold_down)
            {
                total_usable_power = fminf(
                    Map(cap_energy,
                        cap_threshold_down,
                        cap_threshold_up,
                        referee_power_max,
                        max_chassis_power),
                    chassis_power_limit);
            }
            else
            {
                total_usable_power = referee_power_max;
            }
        }
        else
        {
            // 超电反馈不可信时，Fast仅提高buffer充足时的超频倍率。
            const float buffer_overclock_ratio =
                data_hub_.command.chassis_speed_mode == ChassisSpeedMode::Fast
                    ? 1.5f
                    : 1.2f;

            if (referee_energy_buffer >= 50)
            {
                total_usable_power = Map(
                    referee_energy_buffer,
                    50,
                    60,
                    referee_power_max,
                    referee_power_max * buffer_overclock_ratio);
            }
            else
            {
                total_usable_power = referee_power_max;
            }
        }
        // 超电在线时，其报告值是底盘可用功率的硬安全上限。
        if (is_cap_trustable)
            total_usable_power = fminf(total_usable_power, chassis_power_limit);

        // 超电在线时不能用35W下限反向抬高它给出的硬安全上限。
        const float minimum_power = is_cap_trustable ? 0.0f : 45.0f;
        total_usable_power = Clamp(total_usable_power, minimum_power, max_chassis_power);
        // logger_printf("%.3f\r\n", total_usable_power);

        // 裁判系统允许功率过低时，向云台发送35W，使云台直接使用
        // low-power运动参数；否则保持原逻辑，发送底盘最终可用功率。
        chassis.chassis_power_limit =
            referee_power_max < 40.0f ? 35.0f : total_usable_power;
        // logger_printf("%.3f\r\n",referee_power_max);
        // chassis.chassis_power_limit =
        //     35.0f;

        // 功控链路统一使用输出轴力矩(Nm)：输入目标力矩，输出受限后的目标力矩。
        // effort 的换算留到 executeFinalEfforts() 最终下发前统一完成。
        // const auto* left_wheel = static_cast<const DJIMotor*>(motors_[LEFT_WHEEL].get());
        // const auto* right_wheel = static_cast<const DJIMotor*>(motors_[RIGHT_WHEEL].get());
        // const float left_wheel_max_torque = fabsf(
        //     DJIMotor::OUTPUT_MAX_M3508 *
        //     DJIMotor::CURRENT_BIT_2_A_M3508 *
        //     left_wheel->getDJITorqueConstant());
        // const float right_wheel_max_torque = fabsf(
        //     DJIMotor::OUTPUT_MAX_M3508 *
        //     DJIMotor::CURRENT_BIT_2_A_M3508 *
        //     right_wheel->getDJITorqueConstant());
        //
        // // 先裁剪到电机真实可执行的力矩范围，确保功率模型与最终下发一致。
        // target_torques[LEFT_WHEEL] = clipf(
        //     target_torques[LEFT_WHEEL],
        //     -left_wheel_max_torque,
        //     left_wheel_max_torque);
        // target_torques[RIGHT_WHEEL] = clipf(
        //     target_torques[RIGHT_WHEEL],
        //     -right_wheel_max_torque,
        //     right_wheel_max_torque);

        std::array<PowerObject, 2> wheels{};
        wheels[LEFT].connected = motors_[LEFT_WHEEL]->isOnline();
        wheels[LEFT].measure_speed = motors_[LEFT_WHEEL]->getMeasure().speed;
        wheels[LEFT].measure_torque = motors_[LEFT_WHEEL]->getMeasure().torque;
        wheels[LEFT].target_torque = target_torques[LEFT_WHEEL];

        wheels[RIGHT].connected = motors_[RIGHT_WHEEL]->isOnline();
        wheels[RIGHT].measure_speed = motors_[RIGHT_WHEEL]->getMeasure().speed;
        wheels[RIGHT].measure_torque = motors_[RIGHT_WHEEL]->getMeasure().torque;
        wheels[RIGHT].target_torque = target_torques[RIGHT_WHEEL];

        // 超电反馈的是左右轮支路实测功率，与PowerLimiter<2>的建模范围一致。
        // 实测功率用于修正模型预算；帧序号仅用于保证RLS不重复使用同一帧。
        const float measured_bus_power =
            is_cap_trustable ? Scap::getChassisPower() : -1.0f;
        const uint32_t feedback_sequence = Scap::getFeedbackSequence();
        controlled_wheel_torques_ =
            wheel_objs_.control(total_usable_power, measured_bus_power, wheels);
        // controlled_wheel_torques_ =
        //     wheel_objs_.control(1000.0f, measured_bus_power, wheels);     //mark  remenbertoswitch
        target_torques[LEFT_WHEEL] = controlled_wheel_torques_[LEFT];
        target_torques[RIGHT_WHEEL] = controlled_wheel_torques_[RIGHT];
        static uint32_t last_power_log_ms = 0U;
        const uint32_t power_log_now_ms = HAL_GetTick();
        // if (!is_cap_trustable &&
        //     static_cast<uint32_t>(power_log_now_ms - last_power_log_ms) >= 40U)
        // {
        //     last_power_log_ms = power_log_now_ms;
        //     logger_printf(
        //         "%d,%d,%d,%d,0x%02X,%u,%lu\r\n",
        //         static_cast<int>(is_cap_trustable),
        //         static_cast<int>(Scap::isCapConnected()),
        //         static_cast<int>(Scap::isCapError()),
        //         static_cast<int>(Scap::isCapOutputEnabled()),
        //         static_cast<unsigned int>(Scap::getStatusCode()),
        //         static_cast<unsigned int>(Scap::getErrorLevel()),
        //         static_cast<unsigned long>(feedback_sequence));
        // }
        if (static_cast<uint32_t>(power_log_now_ms - last_power_log_ms) >= 40U)
        {
            last_power_log_ms = power_log_now_ms;
            // logger_printf(
            //     "%.3f,%.3f,%.3f\r\n",
            //     total_usable_power,
            //     measured_bus_power,
            //     wheel_objs_.getMeasurePowerSum());
        }

        // RLS更新：
        // 1. 最快10ms一次，且每帧超电反馈最多检查一次；
        // 2. 只在力矩已经基本跟踪、没有明显反馈制动时拟合；
        // 3. 参数限制在初始值的0.5~2倍，单次最多变化2%。
        if (is_cap_trustable &&
            std::isfinite(measured_bus_power) &&
            measured_bus_power > kRlsMinMeasuredPower)
        {
            const uint32_t now_ms = HAL_GetTick();
            const bool interval_elapsed =
                static_cast<uint32_t>(now_ms - last_rls_update_ms_) >= kRlsMinUpdateIntervalMs;
            const bool is_new_power_sample =
                !rls_power_sample_initialized_ ||
                feedback_sequence != last_rls_feedback_sequence_;

            if (interval_elapsed && is_new_power_sample)
            {
                // 无论本帧最终是否满足拟合条件，都不能在后续控制周期重复使用它；
                // 下一帧即使功率数值相同，序号变化后仍会被正常检查。
                last_rls_feedback_sequence_ = feedback_sequence;
                rls_power_sample_initialized_ = true;

                const float limited_torque_left =
                    controlled_wheel_torques_[LEFT];
                const float limited_torque_right =
                    controlled_wheel_torques_[RIGHT];

                const bool torque_tracking_stable =
                    fabsf(limited_torque_left - wheels[LEFT].measure_torque) <=
                        kRlsMaxTorqueTrackingError &&
                    fabsf(limited_torque_right - wheels[RIGHT].measure_torque) <=
                        kRlsMaxTorqueTrackingError;
                const bool non_regenerative =
                    wheels[LEFT].measure_torque * wheels[LEFT].measure_speed >= -1.0f &&
                    wheels[RIGHT].measure_torque * wheels[RIGHT].measure_speed >= -1.0f;

                const auto wheel_params = wheel_objs_.getSampleVector();
                float sample_data[2] = {
                    wheel_params[0],
                    wheel_params[1],
                };
                Matrixf<2, 1> samples(sample_data);
                const float power_loss =
                    measured_bus_power -
                    wheel_objs_.getEffectivePowerSum() -
                    wheel_objs_.getStaticLossPowerSum();

                if (torque_tracking_stable &&
                    non_regenerative &&
                    std::isfinite(power_loss) &&
                    power_loss >= 0.0f &&
                    (wheel_params[0] > 1e-4f || wheel_params[1] > 1e-4f))
                {
                    Matrixf<2, 1> old_params = rls_.getParamsVector();
                    Matrixf<2, 1> candidate_params = rls_.update(samples, power_loss);

                    const float initial_k1 = balance_power_control_config.wheel_config.k1;
                    const float initial_k2 = balance_power_control_config.wheel_config.k2;

                    const auto clamp_rls_param = [](const float candidate,
                                                    const float previous,
                                                    const float initial)
                    {
                        const float absolute_min = initial * RLS<2>::LIMIT_RATIO_MIN;
                        const float absolute_max = initial * RLS<2>::LIMIT_RATIO_MAX;
                        const float safe_previous = fminf(fmaxf(previous, absolute_min), absolute_max);
                        const float step_min = safe_previous * (1.0f - kRlsMaxParamStepRatio);
                        const float step_max = safe_previous * (1.0f + kRlsMaxParamStepRatio);
                        const float lower = fmaxf(absolute_min, step_min);
                        const float upper = fminf(absolute_max, step_max);

                        if (!std::isfinite(candidate))
                            return safe_previous;
                        return fminf(fmaxf(candidate, lower), upper);
                    };


                    float bounded_data[2] = {
                        clamp_rls_param(candidate_params[0][0], old_params[0][0], initial_k1),
                        clamp_rls_param(candidate_params[1][0], old_params[1][0], initial_k2),
                    };
                    const Matrixf<2, 1> bounded_params(bounded_data);
                    rls_.setCurrentParamVector(bounded_params);
                    wheel_objs_.setModelParams(bounded_data[0], bounded_data[1]);

                    // 更新间隔从真正完成辨识的时刻开始计算。
                    last_rls_update_ms_ = now_ms;
                }
            }
        }
        else if (!is_cap_trustable)
        {
            rls_power_sample_initialized_ = false;
        }
    }

    void Chassis::executeFinalEfforts()
    {
        auto* left_wheel = static_cast<DJIMotor*>(motors_[LEFT_WHEEL].get());
        auto* right_wheel = static_cast<DJIMotor*>(motors_[RIGHT_WHEEL].get());
        // 1. 策略仲裁
        if (current_strategy_ == ControlStrategy::USE_RL) {
            target_torques = RL_torques_;
        } else {
            target_torques = LQR_torques_;
        }

        // 最终轮子力矩限幅：Jump策略限制为4Nm，其他策略保持5Nm。
        const float wheel_torque_limit =
            current_policy_ == ChassisPolicy::Jump
                ? kJumpWheelTorqueLimit
                : kWheelTorqueLimit;
        target_torques[LEFT_WHEEL] = clipf(
            target_torques[LEFT_WHEEL],
            -wheel_torque_limit,
            wheel_torque_limit);
        target_torques[RIGHT_WHEEL] = clipf(
            target_torques[RIGHT_WHEEL],
            -wheel_torque_limit,
            wheel_torque_limit);

        // 功控阶段只处理输出轴力矩，单位统一为Nm。
        applyPowerControl();

        // 功控完成后，再把最终目标力矩统一转换为电机框架的effort。
        target_efforts[LEFT_THIGH] = target_torques[LEFT_THIGH] / LEFT_THIGH_CONFIG.t_max_abs * Motor::EFFORT_MAX_ABS;
        target_efforts[RIGHT_THIGH] = target_torques[RIGHT_THIGH] / RIGHT_THIGH_CONFIG.t_max_abs * Motor::EFFORT_MAX_ABS;
        target_efforts[LEFT_SHANK] = target_torques[LEFT_SHANK] / LEFT_SHANK_CONFIG.t_max_abs * Motor::EFFORT_MAX_ABS;
        target_efforts[RIGHT_SHANK] = target_torques[RIGHT_SHANK] / RIGHT_SHANK_CONFIG.t_max_abs * Motor::EFFORT_MAX_ABS;

        target_efforts[LEFT_WHEEL] = target_torques[LEFT_WHEEL]  * Motor::EFFORT_MAX_ABS /
            (DJIMotor::OUTPUT_MAX_M3508  *
            DJIMotor::CURRENT_BIT_2_A_M3508 *
             left_wheel->getDJITorqueConstant());

        target_efforts[RIGHT_WHEEL] = target_torques[RIGHT_WHEEL]  * Motor::EFFORT_MAX_ABS /
            (DJIMotor::OUTPUT_MAX_M3508  *
            DJIMotor::CURRENT_BIT_2_A_M3508 *
             right_wheel->getDJITorqueConstant());

        // 2. 对最终下发的电机力矩做相邻时刻限幅，避免突变
        // limitFinalTorqueStep(target_efforts);

        // applyEffortLowPass(target_efforts);
        last_output_efforts_ = target_efforts;
        if (RobotManager::getDataHub().work_state == WorkState::Protect)
        {
            target_efforts.fill(0.0f);
        }

        // 3. 统一硬件下发
        motors_[LEFT_THIGH]->setEffort(target_efforts[LEFT_THIGH]);
        motors_[LEFT_SHANK]->setEffort(target_efforts[LEFT_SHANK]);
        motors_[RIGHT_THIGH]->setEffort(target_efforts[RIGHT_THIGH]);
        motors_[RIGHT_SHANK]->setEffort(target_efforts[RIGHT_SHANK]);
        // motors_[LEFT_THIGH]->setEffort(0.0f);
        // motors_[LEFT_SHANK]->setEffort(0.0f);
        // motors_[RIGHT_THIGH]->setEffort(0.0f);
        // motors_[RIGHT_SHANK]->setEffort(0.0f);


        // const uint32_t now_ms = HAL_GetTick();

        // const auto& hub = RobotManager::getDataHubConst();

        // hub.command.vy
        // const bool reverse_phase = ((now_ms / 4000U) & 1U) != 0U;


        //
        // const float left_wheel_test_torque = (hub.command.vy >= 0) ? -3.5f : 3.5f;
        // const float x =
        //     RobotManager::getDataHub().work_state == WorkState::Protect
        //         ? 0.0f
        //         : left_wheel_test_torque * Motor::EFFORT_MAX_ABS /
        //             (DJIMotor::OUTPUT_MAX_M3508 *
        //              DJIMotor::CURRENT_BIT_2_A_M3508 *
        //              left_wheel->getDJITorqueConstant());
        // motors_[LEFT_WHEEL]->setEffort(x);
        // motors_[RIGHT_WHEEL]->setEffort(0.0f);
        motors_[LEFT_WHEEL]->setEffort(target_efforts[LEFT_WHEEL]);
        motors_[RIGHT_WHEEL]->setEffort(target_efforts[RIGHT_WHEEL]);
        // motors_[RIGHT_WHEEL]->setEffort(0.0f);
    }

    void Chassis::applyEffortLowPass(std::array<float, 6>& target_efforts) const
    {
        if (!kEnableEffortLowPass)
        {
            return;
        }

        const float alpha = clipf(kEffortLowPassAlpha, 0.0f, 1.0f);
        for (std::size_t i = 0; i < target_efforts.size(); ++i)
        {
            target_efforts[i] = alpha * target_efforts[i] + (1.0f - alpha) * last_output_efforts_[i];
        }
    }

    void Chassis::limitFinalTorqueStep(std::array<float, 6>& target_efforts)
    {
        for (std::size_t i = 0; i < target_efforts.size(); ++i)
        {
            const float effort_delta_limit = torqueDeltaToEffortDelta(i, kFinalTorqueStepLimit);
            target_efforts[i] = clipf(
                target_efforts[i],
                last_output_efforts_[i] - effort_delta_limit,
                last_output_efforts_[i] + effort_delta_limit);
        }
    }

    float Chassis::torqueDeltaToEffortDelta(const std::size_t motor_index, const float torque_delta) const
    {
        switch (motor_index)
        {
        case LEFT_THIGH:
            return torque_delta / LEFT_THIGH_CONFIG.t_max_abs * Motor::EFFORT_MAX_ABS;
        case RIGHT_THIGH:
            return torque_delta / RIGHT_THIGH_CONFIG.t_max_abs * Motor::EFFORT_MAX_ABS;
        case RIGHT_SHANK:
            return torque_delta / RIGHT_SHANK_CONFIG.t_max_abs * Motor::EFFORT_MAX_ABS;
        case LEFT_SHANK:
            return torque_delta / LEFT_SHANK_CONFIG.t_max_abs * Motor::EFFORT_MAX_ABS;
        case RIGHT_WHEEL:
        {
            const auto* right_wheel_motor = static_cast<const DJIMotor*>(motors_[RIGHT_WHEEL].get());
            if (right_wheel_motor == nullptr)
            {
                return Motor::EFFORT_MAX_ABS;
            }

            const float wheel_torque_full_scale =
                DJIMotor::OUTPUT_MAX_M3508 *
                DJIMotor::CURRENT_BIT_2_A_M3508 *
                right_wheel_motor->getDJITorqueConstant();

            return (std::fabs(wheel_torque_full_scale) > kTorqueEpsilon)
                ? torque_delta * Motor::EFFORT_MAX_ABS / wheel_torque_full_scale
                : Motor::EFFORT_MAX_ABS;
        }
        case LEFT_WHEEL:
        {
            const auto* left_wheel_motor = static_cast<const DJIMotor*>(motors_[LEFT_WHEEL].get());
            if (left_wheel_motor == nullptr)
            {
                return Motor::EFFORT_MAX_ABS;
            }

            const float wheel_torque_full_scale =
                DJIMotor::OUTPUT_MAX_M3508 *
                DJIMotor::CURRENT_BIT_2_A_M3508 *
                left_wheel_motor->getDJITorqueConstant();

            return (std::fabs(wheel_torque_full_scale) > kTorqueEpsilon)
                ? torque_delta * Motor::EFFORT_MAX_ABS / wheel_torque_full_scale
                : Motor::EFFORT_MAX_ABS;
        }
        default:
            return Motor::EFFORT_MAX_ABS;
        }
    }

    Chassis& Chassis::getInstance()
    {
        static Chassis instance;
        return instance;
    }

    void Chassis::init()
    {
        auto& chassis = getInstance();
                static StateEstimator estimator{{
            .wheel_radius = WHEEL_RADIUS,
            .wheel_dist = WHEEL_DIST,
            .dt_min = 1e-4f,
            .dt_max = 1e-2f,
            .contact_fn_on = FN_HIGH_THRESHOLD,
            .contact_fn_off = FN_LOW_THRESHOLD,
        }};

        static LQR lqr{{
            .default_mode = LQR::GainMode::FIXED,
            .enable = true,
        }};

        static PID pid_len{{
            .kp = LEG_LEN_KP,
            .ki = 0.0f,
            .kd = LEG_LEN_KD,
            .dead_band = LEG_LEN_DEADBAND,
            .use_ramp = true,
            .ramp_step = LEG_LEN_RAMP,
            .dt = CONTROL_PERIOD * 1000.0f,
        }};

        static PID pid_roll{{
            .kp = LEG_ROLL_KP,
            .ki = 0.0f,
            .kd = LEG_ROLL_KD,
            .dead_band = LEG_ROLL_DEADBAND,
            .use_ramp = true,
            .ramp_step = LEG_ROLL_RAMP,
            .dt = CONTROL_PERIOD * 1000.0f,
        }};

        static PID pid_leg_speed_l{{
            .kp = 10.0f,
            .ki = 0.02f,
            .kd = 1.5f,
            .limit_output = 14.0f,
            .limit_integral = 4.0f,
            .dt = CONTROL_PERIOD * 1000.0f,
        }};
        static PID pid_leg_speed_r{{
            .kp = 10.0f,
            .ki = 0.02f,
            .kd = 1.5f,
            .limit_output = 14.0f,
            .limit_integral = 4.0f,
            .dt = CONTROL_PERIOD * 1000.0f,
        }};


        static PID pid_leg_speed_l_small{{
            .kp = 6.5f,
            .ki = 0.04f,
            .kd = 1.0f,
            .limit_output = 10.0f,
            .limit_integral = 4.0f,
            .dt = CONTROL_PERIOD * 1000.0f,
        }};
        static PID pid_leg_speed_r_small{{
            .kp = 6.5f,
            .ki = 0.04f,
            .kd = 1.0f,
            .limit_output = 10.0f,
            .limit_integral = 4.0f,
            .dt = CONTROL_PERIOD * 1000.0f,
        }};


        static MIT mit_pos_lf{{.kp = DM_MOTOR_KP,
                               .kd = DM_MOTOR_KD,
                               .limit_output = DM_MOTOR_LIMIT_OUTPUT,
                               .limit_error = DM_MOTOR_LIMIT_ERROR,
                               .dead_band = DM_MOTOR_DEADBAND,
                               .use_ramp = true,
                               .ramp_step = DM_MOTOR_RAMP,
                               .dt = CONTROL_PERIOD * 1000.0f}};
        static MIT mit_pos_lb{{.kp = DM_MOTOR_KP,
                               .kd = DM_MOTOR_KD,
                               .limit_output = DM_MOTOR_LIMIT_OUTPUT,
                               .limit_error = DM_MOTOR_LIMIT_ERROR,
                               .dead_band = DM_MOTOR_DEADBAND,
                               .use_ramp = true,
                               .ramp_step = DM_MOTOR_RAMP,
                               .dt = CONTROL_PERIOD * 1000.0f}};
        static MIT mit_pos_rf{{.kp = DM_MOTOR_KP,
                               .kd = DM_MOTOR_KD,
                               .limit_output = DM_MOTOR_LIMIT_OUTPUT,
                               .limit_error = DM_MOTOR_LIMIT_ERROR,
                               .dead_band = DM_MOTOR_DEADBAND,
                               .use_ramp = true,
                               .ramp_step = DM_MOTOR_RAMP,
                               .dt = CONTROL_PERIOD * 1000.0f}};
        static MIT mit_pos_rb{{.kp = DM_MOTOR_KP,
                               .kd = DM_MOTOR_KD,
                               .limit_output = DM_MOTOR_LIMIT_OUTPUT,
                               .limit_error = DM_MOTOR_LIMIT_ERROR,
                               .dead_band = DM_MOTOR_DEADBAND,
                               .use_ramp = true,
                               .ramp_step = DM_MOTOR_RAMP,
                               .dt = CONTROL_PERIOD * 1000.0f}};

        static LegSolver leg_l{
            {.geometry = {.lu = UPPER_LEG_LEN, .lg = LOWER_LEG_LEN}, .offset = {.forward_offset = FORWARD_OFFSET, .backward_offset = BACKWARD_OFFSET}}};
        static LegSolver leg_r{
            {.geometry = {.lu = UPPER_LEG_LEN, .lg = LOWER_LEG_LEN}, .offset = {.forward_offset = FORWARD_OFFSET, .backward_offset = BACKWARD_OFFSET}}};

        chassis.motors_[LEFT_SHANK] = std::make_unique<DMMotor>(LEFT_SHANK_CONFIG);
        chassis.motors_[LEFT_THIGH] = std::make_unique<DMMotor>(LEFT_THIGH_CONFIG);
        chassis.motors_[RIGHT_SHANK] = std::make_unique<DMMotor>(RIGHT_SHANK_CONFIG);
        chassis.motors_[RIGHT_THIGH] = std::make_unique<DMMotor>(RIGHT_THIGH_CONFIG);
        chassis.motors_[LEFT_WHEEL] = std::make_unique<DJIMotor>(LEFT_WHEEL_CONFIG);
        chassis.motors_[RIGHT_WHEEL] = std::make_unique<DJIMotor>(RIGHT_WHEEL_CONFIG);
        Deps deps{};
        deps.estimator = &estimator;
        deps.lqr = &lqr;
        deps.pid_len = &pid_len;
        deps.pid_roll = &pid_roll;
        deps.pid_leg_speed_left = &pid_leg_speed_l;
        deps.pid_leg_speed_right = &pid_leg_speed_r;
        deps.pid_leg_speed_left_small = &pid_leg_speed_l_small;
        deps.pid_leg_speed_right_small = &pid_leg_speed_r_small;

        deps.mit_pos_left_thigh = &mit_pos_lf;
        deps.mit_pos_left_shank = &mit_pos_lb;
        deps.mit_pos_right_thigh = &mit_pos_rf;
        deps.mit_pos_right_shank = &mit_pos_rb;

        deps.leg_left = &leg_l;
        deps.leg_right = &leg_r;

        chassis.joint_left_thigh = dynamic_cast<DMMotor*>(chassis.motors_[RIGHT_THIGH].get());
        chassis.joint_left_shank = dynamic_cast<DMMotor*>(chassis.motors_[RIGHT_SHANK].get());
        chassis.joint_right_thigh = dynamic_cast<DMMotor*>(chassis.motors_[LEFT_THIGH].get());
        chassis.joint_right_shank = dynamic_cast<DMMotor*>(chassis.motors_[LEFT_SHANK].get());

        // chassis.joint_left_thigh->setZeroPosition();
        // chassis.joint_right_thigh->setZeroPosition();
        // chassis.joint_left_shank->setZeroPosition();
        // chassis.joint_right_shank->setZeroPosition();

        deps.wheel_left = dynamic_cast<DJIMotor*>(chassis.motors_[RIGHT_WHEEL].get());
        deps.wheel_right = dynamic_cast<DJIMotor*>(chassis.motors_[LEFT_WHEEL].get());

        // Wire initialized dependencies into the chassis instance.
        chassis.estimator_ = deps.estimator;
        chassis.lqr_ = deps.lqr;
        chassis.pid_len_ = deps.pid_len;
        chassis.pid_roll_ = deps.pid_roll;
        chassis.pid_leg_speed_left = deps.pid_leg_speed_left;
        chassis.pid_leg_speed_right = deps.pid_leg_speed_right;
        chassis.pid_leg_speed_left_small_ = deps.pid_leg_speed_left_small;
        chassis.pid_leg_speed_right_small_ = deps.pid_leg_speed_right_small;

        chassis.mit_pos_left_thigh = deps.mit_pos_left_thigh;
        chassis.mit_pos_left_shank = deps.mit_pos_left_shank;
        chassis.mit_pos_right_thigh = deps.mit_pos_right_thigh;
        chassis.mit_pos_right_shank = deps.mit_pos_right_shank;

        chassis.leg_left = deps.leg_left;
        chassis.leg_right = deps.leg_right;
        chassis.wheel_left = deps.wheel_left;
        chassis.wheel_right = deps.wheel_right;

        static std::array<float, 2> init_params = {
            balance_power_control_config.wheel_config.k1,
            balance_power_control_config.wheel_config.k2,
        };
        const Matrixf<2, 1> initMatrix(init_params.data());
        chassis.rls_.setParamVector(initMatrix);
        chassis.rl_policy_ready_ = RLPolicy::getInstance().init();
        chassis.rl_policy_fault_ = !chassis.rl_policy_ready_;
        HAL_GPIO_WritePin(POWER_PWM_GPIO_Port,POWER_PWM_Pin,GPIO_PIN_SET);
    }

    void Chassis::parseData(uint8_t* data, uint16_t len)
    {
        auto& chassis = getInstance();
        if (len != USB_RECEIVE_SIZE || data[0] != 0xAA || data[len - 1] != 0x0A)
        {
            return;
        }

        if (chassis.rl_action_source_ != RLActionSource::Usb)
        {
            return;
        }

        std::memcpy(chassis.actions_.data(), data + 1, ACTION_FLOAT_COUNT * sizeof(float));
    }

   void Chassis::updateLargeRecoverFSM()
    {
        auto& hub = RobotManager::getDataHub();

        LargeRecoverFSM::Input input{};
        input.pitch = hub.chassis.chassis_pitch;
        input.roll = hub.chassis.chassis_roll;
        input.recover_phi_left = leg_kinematics_[LEFT].recover_phi;
        input.recover_phi_right = leg_kinematics_[RIGHT].recover_phi;
        input.gyro[0] = gyros_[0];
        input.gyro[1] = gyros_[1];
        input.gyro[2] = gyros_[2];
        large_recover_fsm_.update(input, HAL_GetTick());

        const auto& out = large_recover_fsm_.getOutput();

        allow_left_leg_revover = out.allow_left;
        allow_right_leg_revover = out.allow_right;

        RecoverPhiTargetLEFT = out.left_target;
        RecoverPhiTargetRIGHT = out.right_target;

        RecoverStopAngleLEFT = out.left_stop_angle;
        RecoverStopAngleRIGHT = out.right_stop_angle;

        Recover_Direction_LEFT =
            out.left_direction == LargeRecoverFSM::RotateDirection::Positive
                ? RecoverRotateDirection::Positive
                : RecoverRotateDirection::Negative;

        Recover_Direction_RIGHT =
            out.right_direction == LargeRecoverFSM::RotateDirection::Positive
                ? RecoverRotateDirection::Positive
                : RecoverRotateDirection::Negative;

        // ============================================================
        // 目标或方向变化时，清空 arrived 标志
        // 防止 DirectedThenNear 沿用上一阶段的 arrived=true，
        // 导致新状态一上来就走 near 最近方向
        // ============================================================
        static float last_left_target = 0.0f;
        static float last_right_target = 0.0f;
        static RecoverRotateDirection last_left_dir = RecoverRotateDirection::Positive;
        static RecoverRotateDirection last_right_dir = RecoverRotateDirection::Negative;
        static bool last_inited = false;

        if (!last_inited ||
            fabsf(RecoverPhiTargetLEFT - last_left_target) > 0.01f ||
            fabsf(RecoverPhiTargetRIGHT - last_right_target) > 0.01f ||
            Recover_Direction_LEFT != last_left_dir ||
            Recover_Direction_RIGHT != last_right_dir)
        {
            left_large_recover_arrived_ = false;
            right_large_recover_arrived_ = false;

            last_left_target = RecoverPhiTargetLEFT;
            last_right_target = RecoverPhiTargetRIGHT;
            last_left_dir = Recover_Direction_LEFT;
            last_right_dir = Recover_Direction_RIGHT;
            last_inited = true;
        }

        // ============================================================
        // 根据指定方向判断是否已经越过目标
        // 防止当前腿角已经越过目标一点点时，
        // DirectedThenNear 仍然认为 arrived=false，
        // 继续沿指定方向绕优弧回来。
        // ============================================================
        if (isRecoverArrivedByDirection(leg_kinematics_[LEFT].recover_phi,
                                        RecoverPhiTargetLEFT,
                                        Recover_Direction_LEFT,
                                        RecoverStopAngleLEFT,out.left_passed_arrive_angle
                                        ))
        {
            left_large_recover_arrived_ = true;
        }

        if (isRecoverArrivedByDirection(leg_kinematics_[RIGHT].recover_phi,
                                        RecoverPhiTargetRIGHT,
                                        Recover_Direction_RIGHT,
                                        RecoverStopAngleRIGHT,
                                        out.right_passed_arrive_angle))
        {
            right_large_recover_arrived_ = true;
        }
    }

    void Chassis::CalculateLargeRecoverTorque(const float left_target_phi_vel,
                                              const float right_target_phi_vel)
    {
        const LegKinematics& left_leg = leg_kinematics_[LEFT];
        const LegKinematics& right_leg = leg_kinematics_[RIGHT];
        const ForceTorqueMap& left_map = left_leg.force_torque_map;
        const ForceTorqueMap& right_map = right_leg.force_torque_map;

        // 五连杆接近奇异位形时保留原来的单大腿恢复方式，避免使用无效映射。
        if (!left_map.valid || !right_map.valid)
        {
            RL_torques_[LEFT_THIGH] =
                computeRecoverVelocityTorqueNoStallWindup(
                    left_target_phi_vel,
                    thigh_vel_[LEFT],
                    kLargeRecoverThighVelocityConfig,
                    left_large_recover_phi_vel_integral_);
            RL_torques_[LEFT_SHANK] = 0.0f;
            RL_torques_[RIGHT_THIGH] =
                computeRecoverVelocityTorqueNoStallWindup(
                    right_target_phi_vel,
                    thigh_vel_[RIGHT],
                    kLargeRecoverThighVelocityConfig,
                    right_large_recover_phi_vel_integral_);
            RL_torques_[RIGHT_SHANK] = 0.0f;
            return;
        }

        // 实测 recover_phi_dot / thigh_vel = 1、j22 = 0.5。
        // 因此虚拟腿角速度 PI 12/12 与原大腿速度 PI 6/6 等效。
        const float left_virtual_rotate_torque =
            computeRecoverVelocityTorqueNoStallWindup(
                left_target_phi_vel,
                left_leg.recover_phi_dot,
                kLargeRecoverPhiVelocityConfig,
                left_large_recover_phi_vel_integral_);

        const float right_virtual_rotate_torque =
            computeRecoverVelocityTorqueNoStallWindup(
                right_target_phi_vel,
                right_leg.recover_phi_dot,
                kLargeRecoverPhiVelocityConfig,
                right_large_recover_phi_vel_integral_);

        // buildForceTorqueMap() 中的矩阵满足：
        // [tau_shank, tau_thigh]^T = J^T [F_length, T_rotate]^T。
        const float left_leg_force = clipf(
            kLargeRecoverLegLengthKp *
                (kLargeRecoverLegLengthTarget - left_leg.geometry.l0) -
                kLargeRecoverLegLengthKd * left_leg.l0_dot,
            -kLargeRecoverLegForceLimit,
            kLargeRecoverLegForceLimit);

        const float right_leg_force = clipf(
            kLargeRecoverLegLengthKp *
                (kLargeRecoverLegLengthTarget - right_leg.geometry.l0) -
                kLargeRecoverLegLengthKd * right_leg.l0_dot,
            -kLargeRecoverLegForceLimit,
            kLargeRecoverLegForceLimit);


        const float right_virtual_leg_force = -right_leg_force;

        // 运动和到位保持都由 recover_phi 速度 PI 负责；腿长 PD 始终叠加。
        RL_torques_[LEFT_SHANK] =
            left_map.j11 * left_leg_force +
            left_map.j12 * left_virtual_rotate_torque;
        RL_torques_[LEFT_THIGH] =
            left_map.j21 * left_leg_force +
            left_map.j22 * left_virtual_rotate_torque;

        RL_torques_[RIGHT_SHANK] =
            right_map.j11 * right_virtual_leg_force +
            right_map.j12 * right_virtual_rotate_torque;
        RL_torques_[RIGHT_THIGH] =
            right_map.j21 * right_virtual_leg_force +
            right_map.j22 * right_virtual_rotate_torque;
    }

    void Chassis::calculateRLFinalTorques()
    {
        const auto& hub = RobotManager::getDataHubConst();

        const bool is_large_recover =
            current_policy_ == ChassisPolicy::LargeRecover_Auto ||
            current_policy_ == ChassisPolicy::LargeRecover_Crazy;

        const bool pitch_protect =
            fabsf(hub.chassis.chassis_pitch) > 1.4f && !is_large_recover;

        if (!pitch_protect)
        {
            left_pitch_protect_vel_integral_ = 0.0f;
            right_pitch_protect_vel_integral_ = 0.0f;
        }

        if (!is_large_recover)
        {
            left_large_recover_arrived_ = false;
            right_large_recover_arrived_ = false;

            left_large_recover_phi_vel_integral_ = 0.0f;
            right_large_recover_phi_vel_integral_ = 0.0f;
        }

        if (!motors_[LEFT_WHEEL]->isOnline() || !motors_[RIGHT_WHEEL]->isOnline())
        {
            // logger_printf("%.3f,%.3f\r\n",motors_[LEFT_WHEEL]->getMeasure().speed,motors_[RIGHT_WHEEL]->getMeasure().speed);
            RL_torques_.fill(0.0f);
            // logger_printf("%d,%d,%d,%d,%d,%d\r\n",motors_[0]->isOnline(),motors_[1]->isOnline(),motors_[2]->isOnline(),motors_[3]->isOnline(),motors_[4]->isOnline(),motors_[5]->isOnline());
            return;
        }
        // logger_printf("%d,%d,%d,%d,%d,%d\r\n",motors_[0]->isOnline(),motors_[1]->isOnline(),motors_[2]->isOnline(),motors_[3]->isOnline(),motors_[4]->isOnline(),motors_[5]->isOnline());

    // ============================================================
    // pitch 过大保护：
    // 不是 LargeRecover 模式时，不再相信 RL 输出
    // 其它电机力矩清零，只保留左右大腿速度目标为 0 的刹车力矩
    // ============================================================
        if (pitch_protect)
        {
            RL_torques_.fill(0.0f);

            RL_torques_[LEFT_THIGH] =
                computeRecoverVelocityTorque(0.0f,
                                            thigh_vel_[LEFT],
                                            kLargeRecoverThighVelocityConfig,
                                            left_pitch_protect_vel_integral_);

            RL_torques_[RIGHT_THIGH] =
                computeRecoverVelocityTorque(0.0f,
                                            thigh_vel_[RIGHT],
                                            kLargeRecoverThighVelocityConfig,
                                            right_pitch_protect_vel_integral_);

            for (int i = 0; i < 4; i++)
            {
                RL_torques_[i] = clipf(RL_torques_[i], -kParallelTorqueLimit, kParallelTorqueLimit);
            }
            for (int i = 4;i < 6;i++)
            {
                RL_torques_[i] = clipf(RL_torques_[i],-kWheelTorqueLimit, kWheelTorqueLimit);
            }

            Last_RL_torques_ = RL_torques_;
            return;
        }

        switch (current_policy_)
        {
        case ChassisPolicy::Stop:
            RL_torques_.fill(0.0f);
            break;
        case ChassisPolicy::LargeRecover_Auto:
        case ChassisPolicy::LargeRecover_Crazy:
        {
            RL_torques_.fill(0.0f);

            updateLargeRecoverFSM();

            const float left_target_phi_vel = allow_left_leg_revover
                ? computeRecoverPhiTargetVelocityDirectedThenNear(
                    RecoverPhiTargetLEFT,
                    leg_kinematics_[LEFT].recover_phi,
                    Recover_Direction_LEFT,
                    kRecoverPhiPosToVelKp,
                    kRecoverPhiMaxTargetVel,
                    RecoverStopAngleLEFT,
                    left_large_recover_arrived_)
                : 0.0f;

            const float right_target_phi_vel = allow_right_leg_revover
                ? computeRecoverPhiTargetVelocityDirectedThenNear(
                    RecoverPhiTargetRIGHT,
                    leg_kinematics_[RIGHT].recover_phi,
                    Recover_Direction_RIGHT,
                    kRecoverPhiPosToVelKp,
                    kRecoverPhiMaxTargetVel,
                    RecoverStopAngleRIGHT,
                    right_large_recover_arrived_)
                : 0.0f;

            if (fabsf(wrapToPi(RecoverPhiTargetLEFT - leg_kinematics_[LEFT].recover_phi)) < kLargeRecoverIntegralClearAngle ||
                !allow_left_leg_revover)
            {
                left_large_recover_phi_vel_integral_ = 0.0f;
            }

            if (fabsf(wrapToPi(RecoverPhiTargetRIGHT - leg_kinematics_[RIGHT].recover_phi)) < kLargeRecoverIntegralClearAngle ||
                !allow_right_leg_revover)
            {
                right_large_recover_phi_vel_integral_ = 0.0f;
            }

            CalculateLargeRecoverTorque(left_target_phi_vel, right_target_phi_vel);

            break;
        }
        case ChassisPolicy::MiniRecover:
        case ChassisPolicy::Walk:
        case ChassisPolicy::Jump:
        case ChassisPolicy::Down:
        case ChassisPolicy::Spin:
        case ChassisPolicy::Stable:
        default:
            break;
        }

        for (int i = 0; i < 6; i++)
        {
            RL_torques_[i] = clipf(RL_torques_[i], -kParallelTorqueLimit, kParallelTorqueLimit);
        }

        Last_RL_torques_ = RL_torques_;
        // logger_printf("%.3f,%.3f\r\n",RL_torques_[LEFT_WHEEL],RL_torques_[RIGHT_WHEEL]);
        // logger_printf("RL_torques_: [%f, %f, %f, %f, %f, %f]\r\n", left_shank_effort, left_thigh_effort, right_shank_effort, right_thigh_effort, left_wheel_effort, right_wheel_effort);
        return;
    }

    float Chassis::clipf(float value, float lower, float upper)
    {
        if (value < lower)
        {
            return lower;
        }
        if (value > upper)
        {
            return upper;
        }
        return value;
    }

    Chassis::LegGeometry Chassis::solveLegGeometry(float phi1, float phi4) const
    {
        LegGeometry geometry;

        const float x_b = l1_ * cosf(phi1);
        const float y_b = l1_ * sinf(phi1);
        const float x_d = l1_ * cosf(phi4);
        const float y_d = l1_ * sinf(phi4);

        const float dx = x_d - x_b;
        const float dy = y_d - y_b;
        const float a0 = 2.0f * l2_ * dx;
        const float b0 = 2.0f * l2_ * dy;
        const float c0 = dx * dx + dy * dy;

        float disc = a0 * a0 + b0 * b0 - c0 * c0;
        if (disc < 0.0f)
        {
            disc = 0.0f;
        }

        geometry.phi2 = 2.0f * atan2f(b0 + sqrtf(disc), a0 + c0);

        const float x_c = x_b + l2_ * cosf(geometry.phi2);
        const float y_c = y_b + l2_ * sinf(geometry.phi2);

        geometry.phi3 = atan2f(y_c - y_d, x_c - x_d);
        geometry.l0 = sqrtf(x_c * x_c + y_c * y_c);
        geometry.phi0 = atan2f(y_c, x_c);
        return geometry;
    }

    std::array<float, 2> Chassis::solveJaccobian(
        float phi1,
        float phi4,
        const LegGeometry& geometry) const
    {
        const float sin_phi3_phi2 = sinf(geometry.phi3 - geometry.phi2);

        // 五连杆接近奇异位形时解析导数不存在或会急剧增大。
        if (fabsf(sin_phi3_phi2) < kMinSin)
        {
            return {0.0f, 0.0f};
        }

        const float denominator = l2_ * sin_phi3_phi2;
        const float dphi3_dphi1 =
            l1_ * sinf(phi1 - geometry.phi2) / denominator;
        const float dphi3_dphi4 =
            -l1_ * sinf(phi4 - geometry.phi2) / denominator;

        // relative_phi3 = phi3 - phi4 - pi/2。
        // 右腿的内部角度和相对角同时反向，因此换算到实体电机坐标后
        // 仍然使用同一组导数。
        return {dphi3_dphi1, dphi3_dphi4 - 1.0f};
    }

    Chassis::ForceTorqueMap Chassis::buildForceTorqueMap(
        float phi1,
        float phi4,
        const LegGeometry& geometry) const
    {
        ForceTorqueMap map;

        float sin_phi3_2 = sinf(geometry.phi3 - geometry.phi2);
        if (fabsf(sin_phi3_2) < kMinSin)
        {
            sin_phi3_2 = copysignf(kMinSin, sin_phi3_2 == 0.0f ? 1.0f : sin_phi3_2);
        }

        const float safe_l0 = geometry.l0 < kMinLegLength ? kMinLegLength : geometry.l0;
        const float common_phi1 = l1_ * sinf(phi1 - geometry.phi2) / sin_phi3_2;
        const float common_phi4 = l1_ * sinf(geometry.phi3 - phi4) / sin_phi3_2;

        map.j11 = sinf(geometry.phi0 - geometry.phi3) * common_phi1;
        map.j12 = cosf(geometry.phi0 - geometry.phi3) * common_phi1 / safe_l0;
        map.j21 = sinf(geometry.phi0 - geometry.phi2) * common_phi4;
        map.j22 = cosf(geometry.phi0 - geometry.phi2) * common_phi4 / safe_l0;
        map.det = map.j11 * map.j22 - map.j12 * map.j21;
        map.valid = fabsf(map.det) >= kMinDet;
        return map;
    }

    void Chassis::getRLMeasure()
    {
        const auto& chassis_state = RobotManager::getDataHubConst().chassis;
        auto& chassis = RobotManager::getDataHub().chassis;

        // tof_.inquire(LEFT);
        // tof_.inquire(RIGHT);

        const auto* left_tof_measure = tof_.getMeasure(LEFT);
        const auto* right_tof_measure = tof_.getMeasure(RIGHT);

        const auto is_valid_tof = [this](const uint8_t id,
                                         const NoopLoopTof::Measure* measure)
        {
            return measure != nullptr &&
                   tof_.isOnline(id) &&
                   measure->status == 0U &&
                   std::isfinite(measure->distance) &&
                   measure->distance > kJumpTofMinValidDistanceM &&
                   measure->distance < kJumpTofMaxValidDistanceM;
        };

        left_tof_valid_ = is_valid_tof(LEFT, left_tof_measure);
        right_tof_valid_ = is_valid_tof(RIGHT, right_tof_measure);

        // 始终更新UI使用的原始距离；离线回调会把距离清零。
        // 跳跃触发仍严格使用对应的 valid 标志，不能只看这个数值。
        left_tof_measure_distance =
            left_tof_measure != nullptr ? left_tof_measure->distance : 0.0f;
        right_tof_measure_distance =
            right_tof_measure != nullptr ? right_tof_measure->distance : 0.0f;

        static uint32_t last_tof_log_ms = 0U;
        const uint32_t now_ms = HAL_GetTick();
        // if (static_cast<uint32_t>(now_ms - last_tof_log_ms) >= kTofLogPeriodMs)
        // {
        //     last_tof_log_ms = now_ms;
        //     logger_printf("%.3f,%.3f,%d,%d\r\n",
        //                   left_tof_measure_distance,
        //                   right_tof_measure_distance,
        //                   left_tof_valid_,
        //                   right_tof_valid_);
        // }

        // logger_printf("%.3f,%.3f\r\n",left_tof_measure_distance,right_tof_measure_distance);

        quaternions_[0] = chassis_state.imu_quaternion[0];
        quaternions_[1] = chassis_state.imu_quaternion[1];
        quaternions_[2] = chassis_state.imu_quaternion[2];
        quaternions_[3] = chassis_state.imu_quaternion[3];

        gyros_[0] = chassis_state.imu_gyro[0];
        gyros_[1] = chassis_state.imu_gyro[1];
        gyros_[2] = chassis_state.imu_gyro[2];
        // logger_printf("%.3f,%.3f,%.3f\r\n", gyros_[0], gyros_[1], gyros_[2]);

        const Motor::Measure left_thigh_measure = motors_[LEFT_THIGH]->getMeasure();
        const Motor::Measure right_thigh_measure = motors_[RIGHT_THIGH]->getMeasure();
        const Motor::Measure left_shank_measure = motors_[LEFT_SHANK]->getMeasure();
        const Motor::Measure right_shank_measure = motors_[RIGHT_SHANK]->getMeasure();
        const Motor::Measure left_wheel_measure = motors_[LEFT_WHEEL]->getMeasure();
        const Motor::Measure right_wheel_measure = motors_[RIGHT_WHEEL]->getMeasure();
        chassis.is_wheel_online = motors_[LEFT_WHEEL]->isOnline() && motors_[RIGHT_WHEEL]->isOnline();
        // logger_printf("%.3f,%.3f,%.3f,%.3f\r\n",left_thigh_measure.angle_rotor,right_thigh_measure.angle_rotor,left_shank_measure.angle_rotor,right_shank_measure.angle_rotor);

        thigh_angle_[LEFT] = wrapToPi(left_thigh_measure.angle_rotor + (-0.2569f));
        thigh_vel_[LEFT] = left_thigh_measure.speed ;
        shank_angle_[LEFT] = wrapToPi(left_shank_measure.angle_rotor + (0.767f));
        shank_vel_[LEFT] = left_shank_measure.speed;

        thigh_angle_[RIGHT] = wrapToPi(right_thigh_measure.angle_rotor + 0.2569f);
        thigh_vel_[RIGHT] = right_thigh_measure.speed;
        shank_angle_[RIGHT] = wrapToPi(right_shank_measure.angle_rotor + (-0.767f));
        shank_vel_[RIGHT] = right_shank_measure.speed;

        // logger_printf("%.3f,%.3f\r\n", thigh_vel_[LEFT], thigh_vel_[RIGHT]);

        // logger_printf("%.3f,%.3f,%.3f,%.3f\r\n",thigh_angle_[LEFT],thigh_angle_[RIGHT],shank_angle_[LEFT],shank_angle_[RIGHT]);
        wheels_vel_rad_[LEFT] = -left_wheel_measure.speed;
        wheels_vel_rad_[RIGHT] = -right_wheel_measure.speed;   //负号
        // logger_printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",thigh_angle_[LEFT],shank_angle_[LEFT],thigh_angle_[RIGHT],shank_angle_[RIGHT], wheels_vel_rad_[LEFT], wheels_vel_rad_[RIGHT]);
    }

    void Chassis::updateLegKinematics()
    {
        for (uint8_t side = LEFT; side <= RIGHT; ++side)
        {
            LegKinematics& leg = leg_kinematics_[side];
            const float coordinate_sign = side == LEFT ? 1.0f : -1.0f;

            leg.phi1 = coordinate_sign * shank_angle_[side];
            leg.phi4 = coordinate_sign * thigh_angle_[side];
            leg.phi1_dot = coordinate_sign * shank_vel_[side];
            leg.phi4_dot = coordinate_sign * thigh_vel_[side];

            leg.geometry = solveLegGeometry(leg.phi1, leg.phi4);
            leg.joint_jacobian = solveJaccobian(leg.phi1,
                                                leg.phi4,
                                                leg.geometry);
            leg.force_torque_map = buildForceTorqueMap(leg.phi1,
                                                       leg.phi4,
                                                       leg.geometry);

            leg.relative_phi3 = side == LEFT
                ? leg.geometry.phi3 - leg.phi4 - kPi * 0.5f
                : -leg.geometry.phi3 + leg.phi4 + kPi * 0.5f;
            leg.relative_phi3_dot =
                leg.joint_jacobian[0] * shank_vel_[side] +
                leg.joint_jacobian[1] * thigh_vel_[side];
            leg.l0_dot =
                leg.force_torque_map.j11 * leg.phi1_dot +
                leg.force_torque_map.j21 * leg.phi4_dot;
            leg.phi0_dot =
                leg.force_torque_map.j12 * leg.phi1_dot +
                leg.force_torque_map.j22 * leg.phi4_dot;

            // 右腿取反，使左右 recover_phi 的正方向都与实体大腿电机正方向一致。
            leg.recover_phi = coordinate_sign * leg.geometry.phi0;
            leg.recover_phi_dot = coordinate_sign * leg.phi0_dot;
        }

        q_[kDofLf0] = wrapToPi(thigh_angle_[LEFT]);
        q_[kDofRf0] = wrapToPi(thigh_angle_[RIGHT]);
        q_[kDofLf1] = wrapToPi(leg_kinematics_[LEFT].relative_phi3);
        q_[kDofRf1] = wrapToPi(leg_kinematics_[RIGHT].relative_phi3);

        qd_[kDofLf0] = thigh_vel_[LEFT];
        qd_[kDofRf0] = thigh_vel_[RIGHT];
        qd_[kDofLw] = wheels_vel_rad_[LEFT];
        qd_[kDofRw] = wheels_vel_rad_[RIGHT];
        qd_[kDofLf1] = leg_kinematics_[LEFT].relative_phi3_dot;
        qd_[kDofRf1] = leg_kinematics_[RIGHT].relative_phi3_dot;
    }



    void Chassis::updateProjectedGravity()
    {
        const float qx = -quaternions_[1];
        const float qy = -quaternions_[2];
        const float qz = -quaternions_[3];
        const float qw = quaternions_[0];

        const float vx = g_world_[0];
        const float vy = g_world_[1];
        const float vz = g_world_[2];

        const float tx = 2.0f * (qy * vz - qz * vy);
        const float ty = 2.0f * (qz * vx - qx * vz);
        const float tz = 2.0f * (qx * vy - qy * vx);

        const float cx = qy * tz - qz * ty;
        const float cy = qz * tx - qx * tz;
        const float cz = qx * ty - qy * tx;

        projected_g_[0] = vx + qw * tx + cx;

        // projected_g_[0] +=  0.02;
        projected_g_[1] = vy + qw * ty + cy;
        projected_g_[2] = vz + qw * tz + cz;
        // logger_printf("%.3f,%.3f,%.3f\r\n", projected_g_[0], projected_g_[1], projected_g_[2]);
    }

     void Chassis::updateRLCMD()
     {
        const auto& hub = RobotManager::getDataHubConst();
        // logger_printf("%.3f,%.3f\r\n",hub.gimbal.yaw_ecd,hub.gimbal.origin_chassis_yaw_target);
        if (hub.work_state != WorkState::Work)
        {
             vy_velocity_planner_.reset();
             vz_velocity_planner_.reset();
             command_[0] = 0.0f;
             command_[1] = 0.0f;
             command_[2] = clipf(kProtectHeight, kHeightMin, kHeightMax);
             return;
        }

        vy_velocity_planner_.update({hub.command.vy});
        vy_velocity_planner_.calculate();
        vz_velocity_planner_.update({hub.command.vz});
        vz_velocity_planner_.calculate();
        // logger_printf("%.3f\r\n",vy_velocity_planner_.output().v);

        if (fabs(hub.command.vy) < fabs(vy_velocity_planner_.output().v) && ((hub.command.vy * vy_velocity_planner_.output().v) >= 0.0f))
        {
            command_[0] = hub.command.vy;
        }else if (hub.command.vy * vy_velocity_planner_.output().v <= 0.0f)
        {
            command_[0] = hub.command.vy;
        }else
        {
            command_[0] = vy_velocity_planner_.output().v;
        }

        // 默认沿用云台下发的线速度；MiniRecover/Stable下只要存在非零方向
        // 指令，就固定使用2m/s，并保留原指令方向。零输入必须保持停车。
        // command_[0] = hub.command.vy;
        // const bool use_fixed_linear_speed =
        //     current_policy_ == ChassisPolicy::MiniRecover ||
        //     current_policy_ == ChassisPolicy::Stable;
        // if (use_fixed_linear_speed && hub.command.vy != 0.0f)
        // {
        //     command_[0] = copysignf(1.9f, hub.command.vy);
        // }
        // command_[1] = vz_velocity_planner_.output().v;
        // command_[0] = hub.command.vy;
        float yaw_velocity_command = hub.command.vz;
        const bool sinusoidal_spin_active =
            kEnableSinusoidalSpin &&
            hub.command.chassis_motion_mode == ChassisMotionMode::Spin;

        // 只在Spin模式修改底盘内部角速度指令，不改写DataHub中的云台原始值。
        // 进入Spin后先保持一小段最大转速；保持结束后从cos(0)=1连续进入变速波形。
        static bool sinusoidal_spin_was_active = false;
        static uint32_t sinusoidal_spin_start_ms = 0U;
        if (sinusoidal_spin_active)
        {
            const uint32_t now_ms = HAL_GetTick();
            if (!sinusoidal_spin_was_active)
            {
                sinusoidal_spin_start_ms = now_ms;
                sinusoidal_spin_was_active = true;
            }

            const uint32_t elapsed_ms =
                static_cast<uint32_t>(now_ms - sinusoidal_spin_start_ms);
            const float scale_mid =
                0.5f * (kSinusoidalSpinMinScale + kSinusoidalSpinMaxScale);
            const float scale_amplitude =
                0.5f * (kSinusoidalSpinMaxScale - kSinusoidalSpinMinScale);
            float scale = kSinusoidalSpinMaxScale;
            if (elapsed_ms >= kSinusoidalSpinMaxSpeedHoldMs)
            {
                const float waveform_elapsed_s =
                    static_cast<float>(elapsed_ms - kSinusoidalSpinMaxSpeedHoldMs) *
                    0.001f;
                scale = scale_mid + scale_amplitude *
                    cosf(2.0f * kPi * kSinusoidalSpinFrequencyHz *
                         waveform_elapsed_s);
            }

            yaw_velocity_command *= scale;
        }
        else
        {
            sinusoidal_spin_was_active = false;
        }

        command_[1] = clipf(yaw_velocity_command, -kYawVelLimit, kYawVelLimit);
        command_[2] = hub.command.height;
     }

    void Chassis::updatePolicy()
    {
        auto& hub = RobotManager::getDataHub();
        // logger_printf("%.3f\r\n",left_tof_measure_distance);
        if (hub.work_state != WorkState::Work)
        {
            current_policy_ = kDefaultPolicy;
            hub.command.chassis_policy = kDefaultPolicy;
            if (current_policy_ != last_policy_)
            {
                obs_history_initialized_ = false;
                last_actions_.fill(0.0f);
                last_policy_ = current_policy_;
            }

            jump_prepare = false;
            jump_active = false;
            jump_start_ms_ = 0U;
            post_jump_height_hold_active_ = false;
            post_jump_height_hold_start_ms_ = 0U;
            jump_policy_phase_ = 0.0f;
            jump_hold_linear_command_ = 0.0f;
            last_motion_mode_ = ChassisMotionMode::Stop;
            return;
        }

        const ChassisMotionMode motion_mode = hub.command.chassis_motion_mode;
        const uint32_t now_ms = HAL_GetTick();
        const bool jump_request_started =
            motion_mode == ChassisMotionMode::Jump &&
            last_motion_mode_ != ChassisMotionMode::Jump;

        if (jump_request_started && !jump_prepare && !jump_active)
        {
            post_jump_height_hold_active_ = false;
            post_jump_height_hold_start_ms_ = 0U;
            jump_prepare = true;
            jump_active = false;

        }
        else if (motion_mode != ChassisMotionMode::Jump && jump_prepare && !jump_active)
        {
            // 长按松开且TOF尚未触发：退出跳跃准备；真正起跳后不允许取消。
            jump_prepare = false;
            jump_hold_linear_command_ = 0.0f;
        }

        if (jump_prepare && !jump_active)
        {
            const bool jump_tof_online = tof_.isOnline(LEFT);
            const bool tof_triggered =
                left_tof_valid_ &&
                left_tof_measure_distance < kJumpTofTriggerDistanceM;

            // TOF在线时必须等可信距离进入阈值；传感器真正离线时旁路TOF，
            // 按下跳跃键后直接执行。在线但数据异常不会被误判成“离线直跳”。
            if (!kEnableJumpTofTrigger || !jump_tof_online || tof_triggered)
            {
                jump_active = true;
                jump_start_ms_ = now_ms;
                jump_hold_linear_command_ = command_[0];
            }
        }

        if (jump_prepare && jump_active &&
            static_cast<uint32_t>(now_ms - jump_start_ms_) < kJumpDurationMs)
        {
            const uint32_t elapsed_ms = static_cast<uint32_t>(now_ms - jump_start_ms_);
            jump_policy_phase_ = static_cast<float>(elapsed_ms) /
                                 static_cast<float>(kJumpDurationMs);
            // command_[0] = -2.0f;
            command_[0] = jump_hold_linear_command_ - 0.1f;
            command_[1] = 0.0f;
            command_[2] = 0.10f;
            current_policy_ = ChassisPolicy::Jump;

        }
        else
        {
            if (jump_prepare && jump_active)
            {
                jump_prepare = false;
                jump_active = false;
                post_jump_height_hold_active_ = true;
                post_jump_height_hold_start_ms_ = now_ms;
            }
            jump_policy_phase_ = 0.0f;
            jump_hold_linear_command_ = 0.0f;
            // Jump策略只能由TOF触发后的jump_active进入。
            // 跳跃结束后即使按键仍按住，也保持MiniRecover，直到松开后再次按下。
            current_policy_ =
                (jump_prepare || motion_mode == ChassisMotionMode::Jump)
                    ? ChassisPolicy::MiniRecover
                    : mapMotionModeToPolicy(motion_mode);
        }

        // 准备和跳跃执行阶段固定0.1m；跳跃结束后固定0.25m并保持1s，
        // 到时自动恢复云台每周期下发的高度目标。
        if (jump_prepare)
        {
            command_[2] = 0.1f;
        }
        else if (post_jump_height_hold_active_)
        {
            current_policy_ = ChassisPolicy::MiniRecover;
            if (static_cast<uint32_t>(now_ms - post_jump_height_hold_start_ms_) <
                kPostJumpHeightHoldMs)
            {
                command_[2] = kPostJumpHeightM;
            }
            else
            {
                post_jump_height_hold_active_ = false;
                post_jump_height_hold_start_ms_ = 0U;
            }
        }

        last_motion_mode_ = motion_mode;
        hub.command.chassis_policy = current_policy_;
        if (current_policy_ != last_policy_)
        {
            obs_history_initialized_ = false;
            last_actions_.fill(0.0f);
            last_policy_ = current_policy_;
        }

    }

     void Chassis::sendData() const
        {
            static uint8_t tx_buffer[USB_TRANSMIT_SIZE] = {0};
            constexpr size_t kObsByteCount = OBS_FLOAT_COUNT * sizeof(float);
            const auto& default_obs_dof_pos = getDefaultObsDofPos(current_policy_);
            const auto& command_scale = getCommandScale(current_policy_);
            const uint8_t policy_flag = encodePolicyForUsb(current_policy_);

            std::array<float, OBS_FLOAT_COUNT> obs{};
            size_t obs_index = 0;

            auto push_obs = [&](float value) {
                if (obs_index < obs.size())
                {
                    obs[obs_index++] = value;
                }
            };

            for (size_t i = 0; i < 3; ++i)
            {
                push_obs(gyros_[i] * kGyroScale[i]);
            }
            for (size_t i = 0; i < 3; ++i)
            {
                push_obs(projected_g_[i] * kGravityScale[i]);
            }
            for (size_t i = 0; i < 3; ++i)
            {
                push_obs(command_[i] * command_scale[i]);
            }

            push_obs(q_[kDofLf0] - default_obs_dof_pos[0]);
            push_obs(q_[kDofLf1] - default_obs_dof_pos[1]);
            push_obs(q_[kDofRf0] - default_obs_dof_pos[2]);
            push_obs(q_[kDofRf1] - default_obs_dof_pos[3]);

            for (size_t i = 0; i < qd_.size(); ++i)
            {
                push_obs(qd_[i] * kObservedVelScale[i]);
            }
            // logger_printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",qd_[0],qd_[1],qd_[2],qd_[3],qd_[4],qd_[5]);

            std::memcpy(tx_buffer, obs.data(), kObsByteCount);
            tx_buffer[kObsByteCount] = policy_flag;
            tx_buffer[USB_TRANSMIT_SIZE - 1] = '\n';
            USB::send(tx_buffer, USB_TRANSMIT_SIZE);
        }

        std::array<float, RLPolicy::OBS_SIZE> Chassis::buildRLObservation() const
        {
            const auto& default_obs_dof_pos = getDefaultObsDofPos(current_policy_);
            const auto& command_scale = getCommandScale(current_policy_);
            std::array<float, RLPolicy::OBS_SIZE> obs{};
            size_t obs_index = 0;

            auto push_obs = [&](const float value) {
                if (obs_index < obs.size())
                {
                    obs[obs_index++] = value;
                }
            };

            for (size_t i = 0; i < 3; ++i)
            {
                push_obs(gyros_[i] * kGyroScale[i]);
            }
            for (size_t i = 0; i < 3; ++i)
            {
                push_obs(projected_g_[i] * kGravityScale[i]);
            }

            for (size_t i = 0; i < 3; ++i)
            {
                push_obs(command_[i] * command_scale[i]);
            }

            push_obs(q_[kDofLf0] - default_obs_dof_pos[0]);
            push_obs(q_[kDofLf1] - default_obs_dof_pos[1]);
            push_obs(q_[kDofRf0] - default_obs_dof_pos[2]);
            push_obs(q_[kDofRf1] - default_obs_dof_pos[3]);

            for (size_t i = 0; i < qd_.size(); ++i)
            {
                push_obs(qd_[i] * kObservedVelScale[i]);
            }

            for (const float last_action : last_actions_)
            {
                push_obs(last_action);
            }

            return obs;
        }

        void Chassis::updateRLObservationHistory(const std::array<float, RLPolicy::OBS_SIZE>& obs)
        {
            constexpr size_t kObsSize = RLPolicy::OBS_SIZE;
            constexpr size_t kHistoryFrames = RLPolicy::OBS_HISTORY_SIZE / RLPolicy::OBS_SIZE;

            if (!obs_history_initialized_)
            {
                for (size_t frame = 0; frame < kHistoryFrames; ++frame)
                {
                    for (size_t i = 0; i < kObsSize; ++i)
                    {
                        obs_history_[frame * kObsSize + i] = obs[i];
                    }
                }
                obs_history_initialized_ = true;
                return;
            }

            for (size_t frame = 0; frame + 1 < kHistoryFrames; ++frame)
            {
                for (size_t i = 0; i < kObsSize; ++i)
                {
                    obs_history_[frame * kObsSize + i] = obs_history_[(frame + 1) * kObsSize + i];
                }
            }

            for (size_t i = 0; i < kObsSize; ++i)
            {
                obs_history_[(kHistoryFrames - 1) * kObsSize + i] = obs[i];
            }
        }

        void Chassis::updateRLActionsFromOnboardNN()
        {
            auto obs = buildRLObservation();
            updateRLObservationHistory(obs);

            // Zero-input NN test:
            // Uncomment this block to force obs[25] and obs_history[125] to all zeros.
            // obs.fill(0.0f);
            // obs_history_.fill(0.0f);

            RLPolicy::Model model = RLPolicy::Model::Stable;
            if (current_policy_ == ChassisPolicy::MiniRecover)
            {
                model = RLPolicy::Model::Upstairs;
            }
            else if (current_policy_ == ChassisPolicy::Spin)
            {
                model = RLPolicy::Model::Pin;
            }
            else if (current_policy_ == ChassisPolicy::Jump)
            {
                model = RLPolicy::Model::Jump;
            }

            // logger_printf("%d\r\n",model);
            rl_policy_ready_ = RLPolicy::getInstance().run(model, obs, obs_history_, actions_);
            if (!rl_policy_ready_)
            {
                actions_.fill(0.0f);
                rl_policy_fault_ = true;
                RobotManager::getDataHub().work_state = WorkState::Protect;
            }
        }

        void Chassis::calculateRLTorques()
        {
            const auto& default_dof_pos = getDefaultDofPos(current_policy_);
            const auto& p_gains = getPGains(current_policy_);
            const auto& d_gains = getDGains(current_policy_);
            const bool is_spin = current_policy_ == ChassisPolicy::Spin;
            const float gas_spring_compensation_left =
                is_spin
                    ? kSpinGasSpringCompensationLeft
                    : kDefaultGasSpringCompensationLeft;
            const float gas_spring_compensation_right =
                is_spin
                    ? kSpinGasSpringCompensationRight
                    : kDefaultGasSpringCompensationRight;
            std::array<float, 6> policy_act{};
            std::array<float, 6> act{};
            std::array<float, 6> pos_ref{};
            std::array<float, 6> vel_ref{};

            for (size_t i = 0; i < policy_act.size(); ++i)
            {
                policy_act[i] = clipf(actions_[i], -kClipActions, kClipActions);
            }

            if (current_policy_ == ChassisPolicy::Spin)
            {
                if (!spin_action_delay_active_)
                {
                    for (auto& delayed_action : spin_action_delay_fifo_)
                    {
                        delayed_action.fill(0.0f);
                    }
                    spin_action_delay_index_ = 0U;
                    spin_action_delay_active_ = true;
                }

                // 只延迟实际执行的action；策略观测中的last_actions_仍保存本周期模型输出，
                // 与训练端self.actions和action_fifo的语义保持一致。
                act = spin_action_delay_fifo_[spin_action_delay_index_];
                spin_action_delay_fifo_[spin_action_delay_index_] = policy_act;
                spin_action_delay_index_ =
                    (spin_action_delay_index_ + 1U) % kSpinActionDelayCycles;
            }
            else
            {
                if (spin_action_delay_active_)
                {
                    for (auto& delayed_action : spin_action_delay_fifo_)
                    {
                        delayed_action.fill(0.0f);
                    }
                    spin_action_delay_index_ = 0U;
                    spin_action_delay_active_ = false;
                }
                act = policy_act;
            }

            // logger_printf("act: %.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", actions_[0], actions_[1], actions_[2], actions_[3], actions_[4], actions_[5]);
            // logger_printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", act[0], act[1], act[2], act[3], act[4], act[5]);
            pos_ref[kDofLf0] = act[kDofLf0] * kPosActionScale;
            pos_ref[kDofLf1] = act[kDofLf1] * kPosActionScale;
            pos_ref[kDofRf0] = act[kDofRf0] * kPosActionScale;
            pos_ref[kDofRf1] = act[kDofRf1] * kPosActionScale;

            vel_ref[kDofLw] = act[kDofLw] * kVelActionScale;
            vel_ref[kDofRw] = act[kDofRw] * kVelActionScale;
            // logger_printf("%.3f,%.3f,%.3f,%.3f\r\n", pos_ref[kDofLf0], pos_ref[kDofLf1], pos_ref[kDofRf0], pos_ref[kDofRf1]);
            // logger_printf("%.3f,%.3f\r\n", vel_ref[kDofLw], vel_ref[kDofRw]);
            for (size_t i = 0; i < tau_virtual_.size(); ++i)
            {
                tau_virtual_[i] =
                    p_gains[i] * (pos_ref[i] + default_dof_pos[i] - q_[i]) +
                    d_gains[i] * (vel_ref[i] - qd_[i]);
                tau_virtual_[i] = clipf(tau_virtual_[i],-kSerialTorqueLimit,+kSerialTorqueLimit);
            }
            // logger_printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", tau_virtual_[kDofLf0], tau_virtual_[kDofLf1], tau_virtual_[kDofLw], tau_virtual_[kDofRf0], tau_virtual_[kDofRf1], tau_virtual_[kDofRw]);

            const LegKinematics& left_leg = leg_kinematics_[LEFT];
            const LegKinematics& right_leg = leg_kinematics_[RIGHT];
            const std::array<float, 2>& left_joint_jacobian =
                left_leg.joint_jacobian;
            const std::array<float, 2>& right_joint_jacobian =
                right_leg.joint_jacobian;

            // joint_jacobian[0] 对应小腿电机，joint_jacobian[1] 对应大腿电机。
            float tau_lf0_act = tau_virtual_[kDofLf0] +
                tau_virtual_[kDofLf1] * left_joint_jacobian[1];
            float tau_lf20_act =
                tau_virtual_[kDofLf1] * left_joint_jacobian[0];
            float tau_rf0_act = tau_virtual_[kDofRf0] +
                tau_virtual_[kDofRf1] * right_joint_jacobian[1];
            float tau_rf20_act =
                tau_virtual_[kDofRf1] * right_joint_jacobian[0];

            const ForceTorqueMap& left_map = left_leg.force_torque_map;
            if (left_map.valid)
            {
                float ftp_force = (left_map.j22 * tau_lf20_act - left_map.j12 * tau_lf0_act) / left_map.det;
                const float ftp_torque = (-left_map.j21 * tau_lf20_act + left_map.j11 * tau_lf0_act) / left_map.det;

                if (current_policy_ == ChassisPolicy::Jump)
                {
                    if (kJumpPhase1_noScale < jump_policy_phase_ && jump_policy_phase_ < kJumpPhase2_putScale)
                    {
                        ftp_force *= kJumpFTPScale;
                    }else
                    {
                        ftp_force *= 1.0f;
                    }
                }

                ftp_force -= gas_spring_compensation_left * left_leg.geometry.l0;

                tau_lf20_act = left_map.j11 * ftp_force + left_map.j12 * ftp_torque;
                tau_lf0_act = left_map.j21 * ftp_force + left_map.j22 * ftp_torque;
            }
            const ForceTorqueMap& right_map = right_leg.force_torque_map;
            if (right_map.valid)
            {
                float ftp_force = (right_map.j22 * tau_rf20_act - right_map.j12 * tau_rf0_act) / right_map.det;
                const float ftp_torque = (-right_map.j21 * tau_rf20_act + right_map.j11 * tau_rf0_act) / right_map.det;

                if (current_policy_ == ChassisPolicy::Jump)
                {
                    if (kJumpPhase1_noScale < jump_policy_phase_ && jump_policy_phase_ < kJumpPhase2_putScale)
                    {
                        ftp_force *= kJumpFTPScale;
                    }else
                    {
                        ftp_force *= 1.0f;
                    }
                }

                ftp_force += gas_spring_compensation_right * right_leg.geometry.l0;

                tau_rf20_act = right_map.j11 * ftp_force + right_map.j12 * ftp_torque;
                tau_rf0_act = right_map.j21 * ftp_force + right_map.j22 * ftp_torque;
            }

        // logger_printf("%.3f,%.3f,%.3f\r\n",command_[2],left_leg.geometry.l0,right_leg.geometry.l0);
            RL_torques_[LEFT_THIGH] =  tau_lf0_act;
            RL_torques_[LEFT_SHANK] =  tau_lf20_act;
            RL_torques_[RIGHT_THIGH] = tau_rf0_act;
            RL_torques_[RIGHT_SHANK] = tau_rf20_act;
            RL_torques_[LEFT_WHEEL] =  -tau_virtual_[kDofLw];
            RL_torques_[RIGHT_WHEEL] = -tau_virtual_[kDofRw];

            for (float& torque : RL_torques_)
            {
                torque = clipf(torque, -kRealTorqueLimit, kRealTorqueLimit);
            }

            // logger_printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", RL_torques_[LEFT_THIGH], RL_torques_[LEFT_SHANK], RL_torques_[LEFT_WHEEL], RL_torques_[RIGHT_THIGH], RL_torques_[RIGHT_SHANK], RL_torques_[RIGHT_WHEEL]);

            for (size_t i = 0; i < last_actions_.size(); ++i)
            {
                last_actions_[i] = policy_act[i];
            }
        }

    void Chassis::debug_printf() const
    {
        const auto& hub = RobotManager::getDataHubConst();
    }
} // namespace ega
