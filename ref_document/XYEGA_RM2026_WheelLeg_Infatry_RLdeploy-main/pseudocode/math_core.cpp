// ============================================================
// 文件: chassis_math_core.cpp
// 说明: 实现所有核心数学逻辑，包括雅可比、虚拟腿更新、RL力矩映射。
//       气弹簧补偿逻辑完整保留。新增观测历史堆叠（updateObservationHistory）。
// ============================================================
#include "math_core.hpp"
#include <cmath>

namespace ega_math {

// ---------- 工具函数 ----------
float MathCore::wrapToPi(float angle) {
    angle = fmodf(angle, 2.0f * kPi);
    if (angle > kPi) angle -= 2.0f * kPi;
    if (angle < -kPi) angle += 2.0f * kPi;
    return angle;
}

float MathCore::clipf(float value, float lower, float upper) {
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

float MathCore::safe_asin(float x) {
    return asinf(std::max(-1.0f, std::min(1.0f, x)));
}

// ---------- 构造函数 ----------
MathCore::MathCore() {
    leg_kinematics_.fill(LegKinematics{});
    q_.fill(0.0f);
    qd_.fill(0.0f);
    last_actions_.fill(0.0f);
    obs_history_.fill(0.0f);
    history_initialized_ = false;
}

// ---------- 运动学求解 (公式完全复现原始代码) ----------
LegGeometry MathCore::solveLegGeometry(float phi1, float phi4) const {
    LegGeometry g;
    // 基于平行四边形/五连杆机构的虚拟腿解算
    float x_b = l1_ * cosf(phi1), y_b = l1_ * sinf(phi1);
    float x_d = l1_ * cosf(phi4), y_d = l1_ * sinf(phi4);
    float dx = x_d - x_b, dy = y_d - y_b;
    float a0 = 2.0f * l2_ * dx, b0 = 2.0f * l2_ * dy;
    float c0 = dx * dx + dy * dy;
    float disc = a0 * a0 + b0 * b0 - c0 * c0;
    if (disc < 0.0f) disc = 0.0f;
    g.phi2 = 2.0f * atan2f(b0 + sqrtf(disc), a0 + c0);
    float x_c = x_b + l2_ * cosf(g.phi2), y_c = y_b + l2_ * sinf(g.phi2);
    g.phi3 = atan2f(y_c - y_d, x_c - x_d);
    g.l0 = sqrtf(x_c * x_c + y_c * y_c);
    g.phi0 = atan2f(y_c, x_c);
    return g;
}

std::array<float, 2> MathCore::solveJaccobian(float phi1, float phi4, const LegGeometry& g) const {
    float sin_phi3_phi2 = sinf(g.phi3 - g.phi2);
    if (fabsf(sin_phi3_phi2) < kMinSin) return {0.0f, 0.0f};
    float denom = l2_ * sin_phi3_phi2;
    float dphi3_dphi1 = l1_ * sinf(phi1 - g.phi2) / denom;
    float dphi3_dphi4 = -l1_ * sinf(phi4 - g.phi2) / denom;
    return {dphi3_dphi1, dphi3_dphi4 - 1.0f};
}

ForceTorqueMap MathCore::buildForceTorqueMap(float phi1, float phi4, const LegGeometry& g) const {
    ForceTorqueMap map;
    float sin_phi3_2 = sinf(g.phi3 - g.phi2);
    if (fabsf(sin_phi3_2) < kMinSin)
        sin_phi3_2 = copysignf(kMinSin, sin_phi3_2 == 0.0f ? 1.0f : sin_phi3_2);
    float safe_l0 = g.l0 < kMinLegLength ? kMinLegLength : g.l0;
    float common_phi1 = l1_ * sinf(phi1 - g.phi2) / sin_phi3_2;
    float common_phi4 = l1_ * sinf(g.phi3 - phi4) / sin_phi3_2;
    map.j11 = sinf(g.phi0 - g.phi3) * common_phi1;
    map.j12 = cosf(g.phi0 - g.phi3) * common_phi1 / safe_l0;
    map.j21 = sinf(g.phi0 - g.phi2) * common_phi4;
    map.j22 = cosf(g.phi0 - g.phi2) * common_phi4 / safe_l0;
    map.det = map.j11 * map.j22 - map.j12 * map.j21;
    map.valid = fabsf(map.det) >= kMinDet;
    return map;
}

// ---------- 更新虚拟腿 (完全保留计算流程) ----------
void MathCore::updateLegKinematics(
    float lt_angle, float ls_angle, float rt_angle, float rs_angle,
    float lt_vel, float ls_vel, float rt_vel, float rs_vel,
    float lw_vel, float rw_vel)
{
    // 左侧 (符号因子: +1)
    LegKinematics& left = leg_kinematics_[0];
    left.phi1 = ls_angle;
    left.phi4 = lt_angle;
    left.phi1_dot = ls_vel;
    left.phi4_dot = lt_vel;
    left.geometry = solveLegGeometry(left.phi1, left.phi4);
    left.joint_jacobian = solveJaccobian(left.phi1, left.phi4, left.geometry);
    left.force_torque_map = buildForceTorqueMap(left.phi1, left.phi4, left.geometry);
    left.relative_phi3 = left.geometry.phi3 - left.phi4 - kPi * 0.5f;
    left.relative_phi3_dot = left.joint_jacobian[0] * ls_vel + left.joint_jacobian[1] * lt_vel;
    left.l0_dot = left.force_torque_map.j11 * left.phi1_dot + left.force_torque_map.j21 * left.phi4_dot;
    left.phi0_dot = left.force_torque_map.j12 * left.phi1_dot + left.force_torque_map.j22 * left.phi4_dot;

    // 右侧 (符号因子: -1)
    LegKinematics& right = leg_kinematics_[1];
    right.phi1 = -rs_angle;
    right.phi4 = -rt_angle;
    right.phi1_dot = -rs_vel;
    right.phi4_dot = -rt_vel;
    right.geometry = solveLegGeometry(right.phi1, right.phi4);
    right.joint_jacobian = solveJaccobian(right.phi1, right.phi4, right.geometry);
    right.force_torque_map = buildForceTorqueMap(right.phi1, right.phi4, right.geometry);
    right.relative_phi3 = -right.geometry.phi3 + right.phi4 + kPi * 0.5f;
    // relative_phi3 对实体右小腿/右大腿角度求导后，两个坐标反向相互抵消，
    // 因此这里必须使用实体电机的原始速度符号，与实车源码一致。
    right.relative_phi3_dot = right.joint_jacobian[0] * rs_vel + right.joint_jacobian[1] * rt_vel;
    right.l0_dot = right.force_torque_map.j11 * right.phi1_dot + right.force_torque_map.j21 * right.phi4_dot;
    right.phi0_dot = right.force_torque_map.j12 * right.phi1_dot + right.force_torque_map.j22 * right.phi4_dot;

    // 填充全局关节向量 q_ 和 qd_ (交错布局: [左大腿,左虚拟小腿,左轮,右大腿,右虚拟小腿,右轮])
    q_[kDofLf0] = wrapToPi(lt_angle);
    q_[kDofLf1] = wrapToPi(left.relative_phi3);
    q_[kDofRf0] = wrapToPi(rt_angle);
    q_[kDofRf1] = wrapToPi(right.relative_phi3);
    // q_[kDofLw], q_[kDofRw] 为轮子角度，不使用 (仅速度相关)
    qd_[kDofLf0] = lt_vel;
    qd_[kDofLf1] = left.relative_phi3_dot;
    qd_[kDofLw]  = lw_vel;
    qd_[kDofRf0] = rt_vel;
    qd_[kDofRf1] = right.relative_phi3_dot;
    qd_[kDofRw]  = rw_vel;
}

// ---------- 构建 RL 观测 (19维 + 6维历史 = 25维) ----------
// 注意: command 应为已缩放后的指令 (缩放在调用方用 params.cmd_scale 做一次)
void MathCore::buildRLObservation(
    const std::array<float, 3>& gyro,
    const std::array<float, 4>& quat,
    const std::array<float, 3>& command,
    const std::array<float, 4>& default_obs_dof_pos,
    std::array<float, OBS_SIZE>& obs)
{
    // --- 计算投影重力 (将世界重力投影到机体坐标系) ---
    // 公式: g_body = R^T * [0, 0, -1]^T
    // 四元数: quat = [w, x, y, z]
    float qx = -quat[1], qy = -quat[2], qz = -quat[3], qw = quat[0];
    float gx = 0.0f, gy = 0.0f, gz = -1.0f; // 世界重力
    float tx = 2.0f * (qy * gz - qz * gy);
    float ty = 2.0f * (qz * gx - qx * gz);
    float tz = 2.0f * (qx * gy - qy * gx);
    float cx = qy * tz - qz * ty;
    float cy = qz * tx - qx * tz;
    float cz = qx * ty - qy * tx;
    std::array<float, 3> proj_g = {
        gx + qw * tx + cx,
        gy + qw * ty + cy,
        gz + qw * tz + cz
    };

    // --- 定义缩放因子 (与原始一致) ---
    const std::array<float, 3> kGyroScale = {0.25f, 0.25f, 0.25f};
    const std::array<float, 3> kGravityScale = {1.0f, 1.0f, 1.0f};
    // 注意: 指令缩放已在调用方用 params.cmd_scale 完成，此处不再缩放 (避免双重缩放)
    const std::array<float, 6> kObservedVelScale = {0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f};

    // default_obs_dof_pos 由调用方按策略传入 (Jump 策略中位不同)

    // --- 拼接观测向量 ---
    size_t idx = 0;
    auto push = [&](float v) { if (idx < obs.size()) obs[idx++] = v; };

    // 1. 陀螺仪 (3)
    for (int i=0; i<3; ++i) push(gyro[i] * kGyroScale[i]);
    // 2. 投影重力 (3)
    for (int i=0; i<3; ++i) push(proj_g[i] * kGravityScale[i]);
    // 3. 指令 (3) — 已缩放，直接使用
    for (int i=0; i<3; ++i) push(command[i]);
    // 4. 关节角度偏差 (4) [左大腿, 左虚拟小腿, 右大腿, 右虚拟小腿] — 使用交错布局索引
    push(q_[kDofLf0] - default_obs_dof_pos[0]);
    push(q_[kDofLf1] - default_obs_dof_pos[1]);
    push(q_[kDofRf0] - default_obs_dof_pos[2]);
    push(q_[kDofRf1] - default_obs_dof_pos[3]);
    // 5. 关节速度 (6) 
    for (int i=0; i<6; ++i) push(qd_[i] * kObservedVelScale[i]);
    // 6. 上一步动作 (6) -> 构成 25维
    for (float a : last_actions_) push(a);

    // 补全剩余 (理论正好25)
    while (idx < obs.size()) push(0.0f);
}

// ---------- 更新观测历史堆叠 (循环队列) ----------
void MathCore::updateObservationHistory(const std::array<float, OBS_SIZE>& current_obs)
{
    if (!history_initialized_) {
        // 首次调用：用当前观测填充所有历史帧
        for (size_t i = 0; i < OBS_HISTORY_SIZE; ++i) {
            obs_history_[i] = current_obs[i % OBS_SIZE];
        }
        history_initialized_ = true;
        return;
    }

    // 整体左移 25 个位置 (丢弃最老的一帧)
    for (size_t i = 0; i < OBS_HISTORY_SIZE - OBS_SIZE; ++i) {
        obs_history_[i] = obs_history_[i + OBS_SIZE];
    }

    // 将当前观测写入最后 25 个位置 (最新一帧)
    for (size_t i = 0; i < OBS_SIZE; ++i) {
        obs_history_[OBS_HISTORY_SIZE - OBS_SIZE + i] = current_obs[i];
    }
}

// ---------- 重置历史堆叠 ----------
void MathCore::resetObservationHistory()
{
    obs_history_.fill(0.0f);
    history_initialized_ = false;
}

void MathCore::resetLastActions()
{
    last_actions_.fill(0.0f);
}

// ---------- 计算 RL 力矩 (含气弹簧补偿) ----------
// PD 参数由调用方按当前策略传入 (Stable/Spin/Jump/MiniRecover 各不同)
void MathCore::calculateRLTorques(
    const std::array<float, ACTION_SIZE>& actions,
    const std::array<float, 6>& default_dof_pos,
    const std::array<float, 6>& p_gains,
    const std::array<float, 6>& d_gains,
    bool is_spin_mode,
    bool is_jump_mode,
    float jump_phase,
    std::array<float, DOF>& output_torques)
{
    // --- 1. 动作限幅与缩放 ---
    std::array<float, 6> policy_act{};
    std::array<float, 6> act{};
    for (size_t i = 0; i < 6; ++i) {
        policy_act[i] = clipf(actions[i], -kClipActions, kClipActions);
    }

    // 与实车一致：Spin 只延迟实际执行动作一个 500Hz 周期；用于下一帧
    // 观测的 last_actions_ 仍保存当前模型输出 policy_act。
    if (is_spin_mode) {
        if (!spin_action_delay_active_) {
            for (auto& delayed_action : spin_action_delay_fifo_) {
                delayed_action.fill(0.0f);
            }
            spin_action_delay_index_ = 0U;
            spin_action_delay_active_ = true;
        }
        act = spin_action_delay_fifo_[spin_action_delay_index_];
        spin_action_delay_fifo_[spin_action_delay_index_] = policy_act;
        spin_action_delay_index_ =
            (spin_action_delay_index_ + 1U) % kSpinActionDelayCycles;
    } else {
        if (spin_action_delay_active_) {
            for (auto& delayed_action : spin_action_delay_fifo_) {
                delayed_action.fill(0.0f);
            }
            spin_action_delay_index_ = 0U;
            spin_action_delay_active_ = false;
        }
        act = policy_act;
    }

    // --- 2. 计算虚拟力矩 (PD 控制器) ---
    // default_dof_pos / p_gains / d_gains 由调用方按当前策略传入

    std::array<float, 6> pos_ref{}, vel_ref{};
    pos_ref[kDofLf0] = act[kDofLf0] * kPosActionScale;   // 左大腿
    pos_ref[kDofLf1] = act[kDofLf1] * kPosActionScale;   // 左虚拟小腿
    pos_ref[kDofRf0] = act[kDofRf0] * kPosActionScale;   // 右大腿
    pos_ref[kDofRf1] = act[kDofRf1] * kPosActionScale;   // 右虚拟小腿
    vel_ref[kDofLw]  = act[kDofLw]  * kVelActionScale;   // 左轮速度
    vel_ref[kDofRw]  = act[kDofRw]  * kVelActionScale;   // 右轮速度

    // 计算虚拟力矩 tau_virtual_
    for (size_t i = 0; i < 6; ++i) {
        tau_virtual_[i] = p_gains[i] * (pos_ref[i] + default_dof_pos[i] - q_[i])
                        + d_gains[i] * (vel_ref[i] - qd_[i]);
        tau_virtual_[i] = clipf(tau_virtual_[i], -kSerialTorqueLimit, kSerialTorqueLimit);
    }

    // --- 3. 雅可比映射 (虚拟力矩 -> 关节力矩) ---
    const LegKinematics& left = leg_kinematics_[0];
    const LegKinematics& right = leg_kinematics_[1];
    const auto& left_jac = left.joint_jacobian;
    const auto& right_jac = right.joint_jacobian;

    // 映射到实际大腿和小腿力矩 (使用交错布局索引)
    float tau_lf0_act = tau_virtual_[kDofLf0] + tau_virtual_[kDofLf1] * left_jac[1];
    float tau_lf20_act = tau_virtual_[kDofLf1] * left_jac[0];
    float tau_rf0_act = tau_virtual_[kDofRf0] + tau_virtual_[kDofRf1] * right_jac[1];
    float tau_rf20_act = tau_virtual_[kDofRf1] * right_jac[0];

    // --- 4. 力/力矩域转换 + 气弹簧补偿 (关键算法) ---
    const ForceTorqueMap& left_map = left.force_torque_map;
    if (left_map.valid) {
        // 足端力/力矩
        float ftp_force = (left_map.j22 * tau_lf20_act - left_map.j12 * tau_lf0_act) / left_map.det;
        float ftp_torque = (-left_map.j21 * tau_lf20_act + left_map.j11 * tau_lf0_act) / left_map.det;

        // 保留实车跳跃相位缩放接口；当前上场参数 kJumpFTPScale 为 1.0。
        if (is_jump_mode &&
            kJumpPhase1_noScale < jump_phase && jump_phase < kJumpPhase2_putScale) {
            ftp_force *= kJumpFTPScale;
        }

        // ***** 气弹簧补偿 (核心物理补偿) *****
        float gas_spring_left = is_spin_mode ? kSpinGasSpringCompensationLeft : kDefaultGasSpringCompensationLeft;
        ftp_force -= gas_spring_left * left.geometry.l0;

        // 映射回关节力矩
        tau_lf20_act = left_map.j11 * ftp_force + left_map.j12 * ftp_torque;
        tau_lf0_act = left_map.j21 * ftp_force + left_map.j22 * ftp_torque;
    }

    const ForceTorqueMap& right_map = right.force_torque_map;
    if (right_map.valid) {
        float ftp_force = (right_map.j22 * tau_rf20_act - right_map.j12 * tau_rf0_act) / right_map.det;
        float ftp_torque = (-right_map.j21 * tau_rf20_act + right_map.j11 * tau_rf0_act) / right_map.det;

        if (is_jump_mode &&
            kJumpPhase1_noScale < jump_phase && jump_phase < kJumpPhase2_putScale) {
            ftp_force *= kJumpFTPScale;
        }

        // ***** 气弹簧补偿 (右腿符号相反) *****
        float gas_spring_right = is_spin_mode ? kSpinGasSpringCompensationRight : kDefaultGasSpringCompensationRight;
        ftp_force += gas_spring_right * right.geometry.l0;  // 注意这里是 + 号

        tau_rf20_act = right_map.j11 * ftp_force + right_map.j12 * ftp_torque;
        tau_rf0_act = right_map.j21 * ftp_force + right_map.j22 * ftp_torque;
    }

    // --- 5. 输出力矩限幅与赋值 ---
    // 注意: output_torques 为物理电机通道索引 (左右交替排列，与推理索引不一致!)
    //       tau_virtual_ 仍为 kDof 推理交错布局，此处是两套索引的转换点
    output_torques[LEFT_THIGH]  = clipf(tau_lf0_act, -kParallelTorqueLimit, kParallelTorqueLimit);    // 左大腿
    output_torques[LEFT_SHANK]  = clipf(tau_lf20_act, -kParallelTorqueLimit, kParallelTorqueLimit);   // 左小腿
    output_torques[RIGHT_THIGH] = clipf(tau_rf0_act, -kParallelTorqueLimit, kParallelTorqueLimit);    // 右大腿
    output_torques[RIGHT_SHANK] = clipf(tau_rf20_act, -kParallelTorqueLimit, kParallelTorqueLimit);   // 右小腿
    const float wheel_torque_limit = is_jump_mode
        ? kJumpWheelTorqueLimit
        : kWheelTorqueLimit;
    output_torques[LEFT_WHEEL]  = clipf(-tau_virtual_[kDofLw], -wheel_torque_limit, wheel_torque_limit);
    output_torques[RIGHT_WHEEL] = clipf(-tau_virtual_[kDofRw], -wheel_torque_limit, wheel_torque_limit);

    // 保存上一步动作
    last_actions_ = policy_act;
}

} // namespace ega_math
