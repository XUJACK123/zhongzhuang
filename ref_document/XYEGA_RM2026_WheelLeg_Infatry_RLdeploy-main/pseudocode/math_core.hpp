// ============================================================
// 文件: chassis_math_core.hpp
// 说明: 轮腿机器人底盘 RL 控制核心数学库
//       剥离了 CAN/电机/TOF 硬件依赖，仅保留运动学、观测构建、力矩映射。
//       RL 主链路的关键常数、雅可比、动作历史和气弹簧补偿与实机代码一致。
// ============================================================
#pragma once

#include <array>
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace ega_math {

// ---------- RL 主链路常量 ----------
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kMinSin = 1.0e-4f;
constexpr float kMinDet = 1.0e-6f;
constexpr float kMinLegLength = 0.01f;
constexpr float kRealTorqueLimit = 100.0f;
constexpr float kClipActions = 100.0f;
constexpr float kPosActionScale = 0.5f;
constexpr float kVelActionScale = 10.0f;
constexpr float kSerialTorqueLimit = 1000.0f;
constexpr float kParallelTorqueLimit = 35.0f;
constexpr float kWheelTorqueLimit = 5.0f;
constexpr float kJumpWheelTorqueLimit = 4.0f;
constexpr float kJumpPhase1_noScale = 0.1f;
constexpr float kJumpPhase2_putScale = 0.4f;
constexpr float kJumpFTPScale = 1.0f;

// 气弹簧补偿常数 (关键物理参数)
constexpr float kDefaultGasSpringCompensationLeft  = 370.1f;
constexpr float kDefaultGasSpringCompensationRight = 370.1f;
constexpr float kSpinGasSpringCompensationLeft     = 270.1f;
constexpr float kSpinGasSpringCompensationRight    = 300.1f;

// 观测与动作维度 (与 RL 策略严格对齐)
static constexpr uint8_t OBS_SIZE = 25;
static constexpr uint8_t OBS_HISTORY_SIZE = 125;   // 5帧 × 25维
static constexpr uint8_t OBS_HISTORY_FRAMES = 5;   // 历史帧数
static constexpr uint8_t ACTION_SIZE = 6;
static constexpr uint8_t DOF = 6;

// DOF 索引 (交错布局: [左大腿, 左虚拟小腿, 左轮, 右大腿, 右虚拟小腿, 右轮])
// 与 RL 模型训练时的观测/动作布局严格一致，不可更改
constexpr int kDofLf0 = 0;  // 左大腿
constexpr int kDofLf1 = 1;  // 左虚拟小腿
constexpr int kDofLw  = 2;  // 左轮
constexpr int kDofRf0 = 3;  // 右大腿
constexpr int kDofRf1 = 4;  // 右虚拟小腿
constexpr int kDofRw  = 5;  // 右轮

// ---------- 物理电机通道索引 (与推理索引不一致!) ----------
// 这是实际电机在 CAN 总线上的通道顺序，左右腿交替排列。
// 仅用于 output_torques → 电机下发 的最终映射，推理链路内部一律用 kDofXxx。
// 两套索引的转换点在 ChassisMathCore::calculateRLTorques 的输出赋值处。
static constexpr uint8_t LEFT_THIGH  = 0;
static constexpr uint8_t RIGHT_THIGH = 1;
static constexpr uint8_t RIGHT_SHANK = 2;
static constexpr uint8_t LEFT_SHANK  = 3;
static constexpr uint8_t RIGHT_WHEEL = 4;
static constexpr uint8_t LEFT_WHEEL  = 5;

// ---------- 电机偏置 (实机零点 → 策略零点) ----------
// 实机电机编码器零点与 RL 策略训练时的关节零点不一致，需要统一。
// 左大腿: 策略角 = 实机角 - thigh_offset
// 右大腿: 策略角 = 实机角 + thigh_offset
// 左小腿: 策略角 = 实机角 + shank_offset
// 右小腿: 策略角 = 实机角 - shank_offset
constexpr float kThighOffset = 0.2569f;  // 大腿电机偏置
constexpr float kShankOffset = 0.767f;    // 小腿电机偏置

// ---------- 核心数据结构（保留 RL 数学链路需要的字段） ----------
struct LegGeometry {
    float phi2 = 0.0f;  // 膝关节角度
    float phi3 = 0.0f;  // 虚拟腿方向角
    float phi0 = 0.0f;  // 虚拟腿摆角
    float l0   = 0.0f;  // 虚拟腿长度
};

struct ForceTorqueMap {
    float j11 = 0.0f, j12 = 0.0f;
    float j21 = 0.0f, j22 = 0.0f;
    float det = 0.0f;
    bool  valid = false;
};

struct LegKinematics {
    float phi1 = 0.0f, phi4 = 0.0f;          // 关节角度
    float phi1_dot = 0.0f, phi4_dot = 0.0f;  // 关节角速度
    LegGeometry geometry;
    ForceTorqueMap force_torque_map;
    std::array<float, 2> joint_jacobian;     // [dphi3/dphi1, dphi3/dphi4]
    float relative_phi3 = 0.0f;              // 相对虚拟角
    float relative_phi3_dot = 0.0f;
    float l0_dot = 0.0f;
    float phi0_dot = 0.0f;
};

// ---------- 核心控制类 (纯数学，无硬件指针) ----------
class MathCore {
public:
    MathCore();

    // 1. 运动学求解器 (完全保留原始数学公式)
    LegGeometry solveLegGeometry(float phi1, float phi4) const;
    std::array<float, 2> solveJaccobian(float phi1, float phi4, const LegGeometry& geom) const;
    ForceTorqueMap buildForceTorqueMap(float phi1, float phi4, const LegGeometry& geom) const;

    // 2. 更新虚拟腿状态 (输入: 关节角度/角速度/轮速，输出: leg_kinematics_, q_, qd_)
    void updateLegKinematics(
        float left_thigh_angle, float left_shank_angle,
        float right_thigh_angle, float right_shank_angle,
        float left_thigh_vel, float left_shank_vel,
        float right_thigh_vel, float right_shank_vel,
        float left_wheel_vel, float right_wheel_vel
    );

    // 3. 构建 RL 观测向量 (核心: 拼接陀螺仪、重力投影、指令、关节偏差、历史动作)
    //    注意: command 应为已缩放后的指令 (缩放只在此处之外做一次)
    void buildRLObservation(
        const std::array<float, 3>& gyro,           // 陀螺仪 (rad/s)
        const std::array<float, 4>& quaternion,     // IMU 四元数 [w, x, y, z]
        const std::array<float, 3>& command,        // [vx, yaw_rate, height] (已缩放)
        const std::array<float, 4>& default_obs_dof_pos, // 策略相关的观测中位 [左大腿,左虚拟小腿,右大腿,右虚拟小腿]
        std::array<float, OBS_SIZE>& obs            // 输出观测
    );

    // 4. 更新观测历史堆叠 (循环队列，保留最近5帧)
    void updateObservationHistory(const std::array<float, OBS_SIZE>& current_obs);

    // 5. 获取完整历史观测 (125维，用于 RL 推理)
    const std::array<float, OBS_HISTORY_SIZE>& getObservationHistory() const {
        return obs_history_;
    }

    // 6. 重置历史堆叠 (策略切换时调用)
    void resetObservationHistory();

    // 策略切换时与观测历史一起清零，与实车 Chassis::updatePolicy() 一致。
    void resetLastActions();

    // 7. 计算 RL 力矩 (核心: 位置环 + 雅可比映射 + 气弹簧补偿)
    //    PD 参数由调用方按当前策略传入，不硬编码
    void calculateRLTorques(
        const std::array<float, ACTION_SIZE>& actions,  // RL 网络输出
        const std::array<float, 6>& default_dof_pos,    // 策略相关的关节中位
        const std::array<float, 6>& p_gains,            // 策略相关的 Kp
        const std::array<float, 6>& d_gains,            // 策略相关的 Kd
        bool is_spin_mode,                              // 是否 Spin 模式 (影响气弹簧)
        bool is_jump_mode,                              // 是否 Jump 模式 (影响最终轮子限幅)
        float jump_phase,                               // 跳跃相位 (0~1)
        std::array<float, DOF>& output_torques          // 输出关节力矩 (物理电机通道索引: LEFT_THIGH/RIGHT_THIGH/RIGHT_SHANK/LEFT_SHANK/RIGHT_WHEEL/LEFT_WHEEL)
    );

    // 获取内部运动学状态 (用于调试)
    const std::array<LegKinematics, 2>& getLegKinematics() const { return leg_kinematics_; }
    const std::array<float, DOF>& getJointAngles() const { return q_; }
    const std::array<float, DOF>& getJointVelocities() const { return qd_; }
    bool isHistoryInitialized() const { return history_initialized_; }
    const std::array<float, ACTION_SIZE>& getLastActions() const { return last_actions_; }

    // 辅助函数
    static float wrapToPi(float angle);
    static float clipf(float value, float lower, float upper);
    static float safe_asin(float x);

private:
    // 内部成员变量（对应原始 chassis 的 RL 状态）
    std::array<LegKinematics, 2> leg_kinematics_{};
    std::array<float, DOF> q_{};      // 关节位置 (交错布局: [左大腿,左虚拟小腿,左轮,右大腿,右虚拟小腿,右轮])
    std::array<float, DOF> qd_{};     // 关节速度 (交错布局，同上)
    std::array<float, DOF> tau_virtual_{};
    std::array<float, DOF> last_actions_ = {};

    // 实车 Spin 模式对执行动作延迟 1 个 500Hz 控制周期；观测中的
    // last_actions_ 仍保存本周期模型输出，而不是延迟后的执行动作。
    static constexpr std::size_t kSpinActionDelayCycles = 1U;
    std::array<std::array<float, ACTION_SIZE>, kSpinActionDelayCycles> spin_action_delay_fifo_{};
    std::size_t spin_action_delay_index_ = 0U;
    bool spin_action_delay_active_ = false;

    // 观测历史堆叠 (5帧 × 25维 = 125维)
    std::array<float, OBS_HISTORY_SIZE> obs_history_{};
    bool history_initialized_ = false;

    float l1_ = 0.175f;  // 大腿长度
    float l2_ = 0.208f;  // 小腿长度
};

} // namespace ega_math
