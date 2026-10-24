//
// Created by Dell on 2026/1/3.
//

#ifndef RM2026_WHEELLEGGED_CHASSIS_LEG_SOLVER_H
#define RM2026_WHEELLEGGED_CHASSIS_LEG_SOLVER_H

#include <cmath>
#include <memory>

#include "user_configs.h"
#include "user_globals.h"

namespace ega {
    using namespace configs;
    using namespace globals;

    /** 右侧视图
      * x___                O
      *    |          lu   /|\  lu
      *    |y            /  |  \
      *                B \  |  / A
      *                   \ | /
      *                lg  \|/  lg
      *                     P
      *`
      */

    class LegSolver
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        static constexpr float K_NOR_EPS = 1e-6f; // 常规数学防护
        static constexpr float K_LEG_EPS = 1e-3f; // 腿长防护
        static constexpr float K_DET_EPS = 1e-6f; // 奇异性防护

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        enum class ControlMode
        {
            DISABLE = 0,
            POSITION,
            FORCE,
        };

        struct Input
        {
            float q1 = 0.0f;  // 髋关节角
            float q2 = 0.0f;  // 膝关节角
            float dq1 = 0.0f; // 髋角速度
            float dq2 = 0.0f; // 膝角速度

            float tau1_meas = 0.0f; // 测得髋关节力矩
            float tau2_meas = 0.0f; // 测得膝关节力矩

            [[nodiscard]] bool isFinite() const
            {
                return std::isfinite(q1) && std::isfinite(q2) && std::isfinite(dq1) && std::isfinite(dq2) && std::isfinite(tau1_meas) &&
                       std::isfinite(tau2_meas);
            }
        };

        struct Target
        {
            float leg_len = 0.0f;
            float leg_phi = 0.0f;

            float leg_F = 0.0f;
            float leg_T = 0.0f;
        };

        struct Output
        {
            // 虚拟腿状态
            float l0 = 0.0f;       // 【腿长，m】
            float l0_dot = 0.0f;   // 【腿长变化速度，m/s】
            float phi0 = 0.0f;     // 【腿相对机体角度，rad】
            float phi0_dot = 0.0f; // 【腿相对机体角度变化速度，rad/s】

            float F_meas = 0.0f; // 【测量解算得到的径向支持力】
            float T_meas = 0.0f; // 【测量解算得到的切向转矩】

            bool force_valid = false; // 反解的足端力

            float agl1_cmd = 0.0f; // 【前侧电机输出角度，rad】
            float agl2_cmd = 0.0f; // 【后侧电机输出角度，rad】
            float tau1_cmd = 0.0f; // 【前侧电机输出转矩，N·m】
            float tau2_cmd = 0.0f; // 【后侧电机输出转矩，N·m】

            bool pos_valid = false;
            bool tor_valid = false;

            float beta = 0.0f;

            [[nodiscard]] bool isFinite() const
            {
                return std::isfinite(l0) && std::isfinite(l0_dot) && std::isfinite(phi0) && std::isfinite(phi0_dot) && std::isfinite(agl1_cmd) &&
                       std::isfinite(agl2_cmd) && std::isfinite(tau1_cmd) && std::isfinite(tau2_cmd) && std::isfinite(F_meas) && std::isfinite(T_meas);
            }
        };

        struct Cache
        {
            // 考虑偏置后的电机角度
            float q1 = 0.0f, q2 = 0.0f;
            float dq1 = 0.0f, dq2 = 0.0f;

            // 腿坐标系
            float coord[6] = {0.0f};
            float phiA = 0.0f, phiB = 0.0f;

            // J: [dxP; dyP] = J * [dq1; dq2]
            float J11, J12, J21, J22;
            // S: [dL0; dphi0] = S * [dq1; dq2]
            float S11, S12, S21, S22;
            // T: [tau1; tau2] = T * [Fr; Mt]
            float T11, T12, T21, T22;
            // Inverse T
            float inv_T11, inv_T12, inv_T21, inv_T22;
            float det_T;

            void clear() { *this = Cache{}; }
        };

        typedef enum COORD
        {
            x_A,
            y_A,
            x_B,
            y_B,
            x_P,
            y_P,
        } Coord;

        struct Config
        {
            struct Geometry
            {
                float lu = UPPER_LEG_LEN; // 大腿长度
                float lg = LOWER_LEG_LEN; // 小腿长度
            } geometry;

            struct Offset
            {
                float forward_offset = FORWARD_OFFSET;
                float backward_offset = BACKWARD_OFFSET;
            } offset;
        };

        /* ====================== 3. 静态接口 ====================== */

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit LegSolver(const Config& config);
        ~LegSolver() = default;

        /* ====================== 5. 公共接口 ====================== */
    public:
        void update(const Input& in) { input_ = in; } // 更新 Leg Solver 输入
        void calculate();                             // 计算运动学
        void computeOutput();                         // 根据控制模式求解输出

        [[nodiscard]] const Input& input() const { return input_; }    // 获取 Input 数据
        [[nodiscard]] const Target& target() const { return target_; } // 获取 Target 数据
        [[nodiscard]] const Output& output() const { return output_; } // 获取 Output 数据
        [[nodiscard]] const Cache& cache() const { return cache_; }    // 获取 Cache 数据

        void clear(); // 清空 Leg Solver

        void setControlMode(const ControlMode mode) { control_mode_ = mode; }   // 设置当前控制模式
        [[nodiscard]] ControlMode controlMode() const { return control_mode_; } // 获取当前控制模式

        void setPositionTarget(const Target& target); // 位控模式下，设置目标数据
        void setForceTarget(const Target& target);    // 力控模式下，设置目标数据
        void setTarget(const Target& target);         // 设置目标数据

        void resetTarget()
        { // 清空目标数据
            target_ = Target{};
            is_target_set_ = false;
        }

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    protected:
        Input input_;
        Target target_;
        Output output_;
        Cache cache_;

        Config config_;
        ControlMode control_mode_;

        bool is_target_set_ = false;

    private:
        void preprocessInput();
        void clearOutput();

        void computeForwardKinematics();
        void computeInverseKinematics();

        void computeJacobian();

        void computeVirtualLegVelocity();
        void computeVirtualLegTargetForce();
        void computeVirtualLegMeasuredForce();
    };

} // namespace ega

#endif //RM2026_WHEELLEGGED_CHASSIS_LEG_SOLVER_H
