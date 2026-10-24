//
// Created by Dell on 2026/1/3.
//

#include "leg_solver.h"


#include <arm_math.h>
#include <cstring>

#include "logger.h"
#include "utils_basic.h"

namespace ega {
    using namespace configs;

    LegSolver::LegSolver(const Config& config) : config_(config), control_mode_(ControlMode::DISABLE) {}

    /**
     * @brief 设置虚拟腿位控制目标
     *
     * 将控制模式切换为 POSITION，并保存虚拟腿期望的长度与角度等目标量。
     * 设置成功后标记目标已就绪，供 calculate() 周期内使用。
     */
    void LegSolver::setPositionTarget(const Target& target)
    {
        control_mode_ = ControlMode::POSITION;
        target_ = target;
        is_target_set_ = true;
    }

    /**
     * @brief 设置虚拟腿力控制目标
     *
     * 将控制模式切换为 FORCE，并保存虚拟腿期望的径向力与切向力矩等目标量。
     * 设置成功后标记目标已就绪，供 calculate() 周期内使用。
     */
    void LegSolver::setForceTarget(const Target& target)
    {
        control_mode_ = ControlMode::FORCE;
        target_ = target;
        is_target_set_ = true;
    }

    void LegSolver::setTarget(const Target& target)
    {
        target_ = target;
        is_target_set_ = true;
    }

    void LegSolver::clear()
    {
        target_ = Target{};
        clearOutput();
        cache_.clear();

        control_mode_ = ControlMode::DISABLE;
        is_target_set_ = false;
    }

    /**
     * @brief 求解主流程：更新运动学/雅各比，并生成控制输出
     *
     * 每个周期执行一次：清空输出与缓存，读取并预处理输入，
     * 计算正/逆运动学、雅各比矩阵、目标虚拟腿力到关节力矩映射，
     * 以及由关节力矩反馈反解虚拟腿受力。
     *
     * 根据当前控制模式生成输出命令：
     * - DISABLE 或未设置目标：输出保持/零力矩；
     * - POSITION：输出关节角目标（力矩置零）；
     * - FORCE：输出关节力矩目标（角度命令置为当前反馈）。
     */
    void LegSolver::calculate()
    {
        clearOutput();
        cache_.clear();

        preprocessInput();
        computeForwardKinematics();
        computeJacobian();
        computeVirtualLegVelocity();
        computeVirtualLegMeasuredForce();
    }

    void LegSolver::computeOutput()
    {
        computeInverseKinematics();
        computeVirtualLegTargetForce();

        if (control_mode_ == ControlMode::DISABLE || !is_target_set_)
        {
            output_.agl1_cmd = 0.0f;
            output_.agl2_cmd = 0.0f;
            output_.tau1_cmd = 0.0f;
            output_.tau2_cmd = 0.0f;
            return;
        }

        if (control_mode_ == ControlMode::POSITION)
        {
            output_.tau1_cmd = 0.0f;
            output_.tau2_cmd = 0.0f;
        } else if (control_mode_ == ControlMode::FORCE)
        {
            output_.agl1_cmd = 0.0f;
            output_.agl2_cmd = 0.0f;
        }
    }

    /**
     * @brief 预处理电机输入的关节角度与角速度
     *
     * 将电机原始反馈的关节角度叠加安装/机械偏置，
     * 得到用于运动学与动力学计算的绝对关节角；
     * 同时拷贝关节角速度到内部缓存。
     *
     * 若输入数据无效（NaN / Inf），则清空缓存状态。
     */
    void LegSolver::preprocessInput()
    {
        if (!input_.isFinite())
        {
            cache_.q1 = 0.0f;
            cache_.q2 = 0.0f;
            cache_.dq1 = 0.0f;
            cache_.dq2 = 0.0f;
            return;
        }

        cache_.q1 = input_.q1 + config_.offset.forward_offset;
        cache_.q2 = input_.q2 + config_.offset.backward_offset;
        cache_.dq1 = input_.dq1;
        cache_.dq2 = input_.dq2;
    }

    /**
     * @brief 清空本周期输出
     *
     * 将输出结构体各字段重置为默认值，避免未更新字段残留上周期数据。
     * 通常在每个控制周期开始时调用。
     */
    void LegSolver::clearOutput()
    {
        output_.l0 = 0.0f;
        output_.l0_dot = 0.0f;
        output_.phi0 = 0.0f;
        output_.phi0_dot = 0.0f;

        output_.F_meas = 0.0f;
        output_.T_meas = 0.0f;
        output_.force_valid = false;

        output_.agl1_cmd = 0.0f;
        output_.agl2_cmd = 0.0f;
        output_.tau1_cmd = 0.0f;
        output_.tau2_cmd = 0.0f;

        output_.pos_valid = false;
        output_.tor_valid = false;
    }

    /**
     * @brief 正运动学计算（关节角 -> 虚拟腿状态）
     *
     * 根据两侧驱动关节的绝对角度，计算并联机构的几何闭式解，
     * 得到足端位置以及对应的虚拟腿长度 l0 和虚拟腿角 phi0。
     *
     * 其中虚拟腿角 phi0 以 +Y 方向为零位，逆时针为正。
     */
    void LegSolver::computeForwardKinematics()
    {
        const float lu = config_.geometry.lu;
        const float lg = config_.geometry.lg;

        const float q1 = cache_.q1;
        const float q2 = cache_.q2;
        float s1, c1, s2, c2;
        arm_sin_cos_f32(q1 * RAD2DEG, &s1, &c1);
        arm_sin_cos_f32(q2 * RAD2DEG, &s2, &c2);

        cache_.coord[x_A] = lu * c1;
        cache_.coord[y_A] = lu * s1;
        cache_.coord[x_B] = lu * c2;
        cache_.coord[y_B] = lu * s2;

        const float dx = (cache_.coord[x_B] - cache_.coord[x_A]);
        const float dy = (cache_.coord[y_B] - cache_.coord[y_A]);

        const float A = 2.0f * lg * dx;
        const float B = 2.0f * lg * dy;
        const float C = dx * dx + dy * dy;

        float tmp = A * A + B * B - C * C;
        tmp = tmp < 0.0f ? 0.0f : tmp;

        float root;
        arm_sqrt_f32(tmp, &root);

        cache_.phiA = 2.0f * atan2f(B + root, A + C);

        float sA, cA;
        arm_sin_cos_f32(cache_.phiA * RAD2DEG, &sA, &cA);

        cache_.coord[x_P] = cache_.coord[x_A] + lg * cA;
        cache_.coord[y_P] = cache_.coord[y_A] + lg * sA;

        cache_.phiB = atan2f(cache_.coord[y_P] - cache_.coord[y_B], cache_.coord[x_P] - cache_.coord[x_B]);
        output_.beta = cache_.phiB - output_.phi0;

        const float xP = cache_.coord[x_P];
        const float yP = cache_.coord[y_P];

        output_.l0 = sqrtf(xP * xP + yP * yP);
        output_.phi0 = utils::wrapPi_(atan2f(yP, xP) - 0.5f * PI);
    }

    /**
     * @brief 逆运动学计算（虚拟腿目标 -> 关节角目标）
     *
     * 输入目标虚拟腿长度 l 和虚拟腿角 phi（以 +Y 为零位，逆时针为正），
     * 通过几何法（余弦定理）求解两侧驱动关节的绝对角度。
     *
     * 该并联机构存在两组对称解，本函数将候选解提升到与当前反馈角度最近的等价圈数，
     * 并选择变化量更小的一组，以保证输出关节目标的连续性。
     *
     * 若目标不可达（越过几何可达域），则标记 pos_valid 为 false。
     */
    void LegSolver::computeInverseKinematics()
    {
        const float lu = config_.geometry.lu;
        const float lg = config_.geometry.lg;

        const float l = target_.leg_len;
        const float q = utils::wrapPi_(target_.leg_phi);

        const float q1 = cache_.q1;
        const float q2 = cache_.q2;

        if (l < fabsf(lu - lg) || l > (lu + lg) || l < 1e-6f)
        {
            output_.pos_valid = false;
            return;
        }

        float cosb = (lu * lu + l * l - lg * lg) / (2.0f * lu * l);
        cosb = utils::limit(cosb, -1.0f, 1.0f);

        const float beta = acosf(cosb);
        const float base = q + 0.5f * PI;

        const float q1_1 = utils::liftToNearest(base - beta, q1);
        const float q2_1 = utils::liftToNearest(base + beta, q2);
        const float q1_2 = utils::liftToNearest(base + beta, q1);
        const float q2_2 = utils::liftToNearest(base - beta, q2);

        const float cost_1 = (q1_1 - q1) * (q1_1 - q1) + (q2_1 - q2) * (q2_1 - q2);
        const float cost_2 = (q1_2 - q1) * (q1_2 - q1) + (q2_2 - q2) * (q2_2 - q2);

        const float off1 = config_.offset.forward_offset;
        const float off2 = config_.offset.backward_offset;

        if (cost_1 <= cost_2)
        {
            output_.agl1_cmd = q1_1 - off1;
            output_.agl2_cmd = q2_1 - off2;
        } else
        {
            output_.agl1_cmd = q1_2 - off1;
            output_.agl2_cmd = q2_2 - off2;
        }
        output_.pos_valid = true;
    }

    /**
     * @brief 计算并联腿的雅各比与极坐标速度/力映射矩阵
     *
     * 计算足端直角坐标速度雅各比 J（dq -> [dx, dy]），并进一步计算
     * 极坐标速度雅各比 S（dq -> [dL0, dphi0]）。
     *
     * 同时构造虚拟腿广义力（径向力 Fr、切向力矩 Mt）到关节力矩的映射 T，
     * 满足 tau = T * [Fr; Mt]，并计算 T 的行列式与逆矩阵用于测力反解。
     */
    void LegSolver::computeJacobian()
    {
        const float lu = config_.geometry.lu;
        const float lg = config_.geometry.lg;

        const float q1 = cache_.q1;
        const float q2 = cache_.q2;

        const float qA = cache_.phiA;
        const float qB = cache_.phiB;

        const float q1A = q1 - qA;
        const float q2B = q2 - qB;
        const float qAB = qA - qB;

        float sA, cA, sB, cB;
        float s1A, c1A, s2B, c2B, sAB, cAB;
        arm_sin_cos_f32(qA * RAD2DEG, &sA, &cA);
        arm_sin_cos_f32(qB * RAD2DEG, &sB, &cB);
        arm_sin_cos_f32(q1A * RAD2DEG, &s1A, &c1A);
        arm_sin_cos_f32(q2B * RAD2DEG, &s2B, &c2B);
        arm_sin_cos_f32(qAB * RAD2DEG, &sAB, &cAB);

        cache_.J11 = lu * s1A * sB / sAB;
        cache_.J12 = -lu * s2B * sA / sAB;
        cache_.J21 = -lu * s1A * cB / sAB;
        cache_.J22 = lu * s2B * cA / sAB;

        const float l0 = output_.l0;
        const float q0 = output_.phi0 + 0.5f * PI;

        const float q0A = q0 - qA;
        const float q0B = q0 - qB;

        float s0A, c0A, s0B, c0B;
        arm_sin_cos_f32(q0A * RAD2DEG, &s0A, &c0A);
        arm_sin_cos_f32(q0B * RAD2DEG, &s0B, &c0B);

        cache_.S11 = -lu * s0B * s1A / sAB;
        cache_.S12 = lu * s0A * s2B / sAB;
        cache_.S21 = -lu * c0B * s1A / (l0 * sAB);
        cache_.S22 = lu * c0A * s2B / (l0 * sAB);

        cache_.T11 = -lu * s0B * s1A / sAB;
        cache_.T12 = -lu * c0B * s1A / (l0 * sAB);
        cache_.T21 = lu * s0A * s2B / sAB;
        cache_.T22 = lu * c0A * s2B / (l0 * sAB);

        cache_.det_T = cache_.T11 * cache_.T22 - cache_.T12 * cache_.T21;

        cache_.inv_T11 = cache_.T22 / cache_.det_T;
        cache_.inv_T12 = -cache_.T12 / cache_.det_T;
        cache_.inv_T21 = -cache_.T21 / cache_.det_T;
        cache_.inv_T22 = cache_.T11 / cache_.det_T;
    }

    /**
     * @brief 计算虚拟腿的极坐标速度（l0_dot / phi0_dot）
     *
     * 根据关节角速度 (dq1, dq2) 和极坐标速度雅各比矩阵 S，
     * 计算虚拟腿长度变化率 l0_dot 与虚拟腿角速度 phi0_dot。
     */
    void LegSolver::computeVirtualLegVelocity()
    {
        const float dq1 = cache_.dq1;
        const float dq2 = cache_.dq2;

        output_.l0_dot = cache_.S11 * dq1 + cache_.S12 * dq2;
        output_.phi0_dot = cache_.S21 * dq1 + cache_.S22 * dq2;
    }

    /**
     * @brief 根据目标虚拟腿广义力计算关节力矩命令
     *
     * 输入虚拟腿的径向力 Fr 和切向力矩 Mt，
     * 通过极坐标力雅各比矩阵 T（S 的转置）
     * 映射为对应的关节驱动力矩命令。
     */
    void LegSolver::computeVirtualLegTargetForce()
    {
        const float target_F = target_.leg_F;
        const float target_T = target_.leg_T;

        output_.tau1_cmd = cache_.T11 * target_F + cache_.T12 * target_T;
        output_.tau2_cmd = cache_.T21 * target_F + cache_.T22 * target_T;
        output_.tor_valid = true;
    }

    /**
     * @brief 由测得关节力矩反解虚拟腿受力
     *
     * 通过 Jacobian 逆转置将关节力矩反解为足端力，
     * 并投影得到虚拟腿的径向支持力和切向转矩。
     */
    void LegSolver::computeVirtualLegMeasuredForce()
    {
        const float tau1 = input_.tau1_meas;
        const float tau2 = input_.tau2_meas;

        output_.F_meas = cache_.inv_T11 * tau1 + cache_.inv_T12 * tau2;
        output_.T_meas = cache_.inv_T21 * tau1 + cache_.inv_T22 * tau2;
        output_.force_valid = true;
    }
} // namespace ega
