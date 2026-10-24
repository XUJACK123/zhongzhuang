// ============================================================
// 文件: policy_params_design.hpp
// 说明: 不同策略的 PD 增益、站立中位、观测缩放——纯查找表
//       MiniRecover 为默认策略，与 Stable 参数相同但语义不同。
// ============================================================
#pragma once

#include <array>
#include <cstdint>

namespace ega_policy {

enum class ChassisPolicy : uint8_t {
    Stop,
    Jump,
    Walk,
    Spin,
    MiniRecover,
    LargeRecover_Auto,
    LargeRecover_Crazy,
    Stable,
    Down,
};

struct PolicyParams {
    std::array<float, 6> dof_pos;    // 关节目标中位 (6维, 用于力矩PD)
    std::array<float, 4> obs_dof_pos; // 观测用关节中位 (4维, [左大腿,左虚拟小腿,右大腿,右虚拟小腿])
    std::array<float, 6> p_gains;    // 位置环 Kp
    std::array<float, 6> d_gains;    // 速度环 Kd
    std::array<float, 3> cmd_scale;  // [vx, yaw_rate, height] 观测缩放
};

class PolicyParamsTable {
public:
    static PolicyParams get(ChassisPolicy policy) {
        switch (policy) {
            case ChassisPolicy::Stable:      return stableParams();
            case ChassisPolicy::Spin:        return spinParams();
            case ChassisPolicy::Jump:        return jumpParams();
            case ChassisPolicy::MiniRecover: return miniRecoverParams();
            case ChassisPolicy::Walk:
            case ChassisPolicy::Down:
            case ChassisPolicy::LargeRecover_Auto:
            case ChassisPolicy::LargeRecover_Crazy:
            case ChassisPolicy::Stop:
            default:                         return miniRecoverParams();
        }
    }

private:
    static PolicyParams stableParams() {
        return {
            {-0.23f, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f},
            {-0.23f, -0.65f, 0.23f, 0.65f},
            {15.0f, 15.0f, 0.0f, 15.0f, 15.0f, 0.0f},
            {1.0f, 1.0f, 0.1f, 1.0f, 1.0f, 0.1f},
            {3.0f, 0.25f, 5.0f},
        };
    }

    static PolicyParams miniRecoverParams() {
        // MiniRecover 与 Stable 参数一致，但语义不同：
        // MiniRecover 是在 Walk/MiniRecover 模式下的默认策略
        return {
            {-0.23f, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f},
            {-0.23f, -0.65f, 0.23f, 0.65f},
            {15.0f, 15.0f, 0.0f, 15.0f, 15.0f, 0.0f},
            {1.0f, 1.0f, 0.1f, 1.0f, 1.0f, 0.1f},
            {3.0f, 0.25f, 5.0f},
        };
    }

    static PolicyParams spinParams() {
        return {
            {-0.23f, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f},
            {-0.23f, -0.65f, 0.23f, 0.65f},
            {10.0f, 10.0f, 0.0f, 10.0f, 10.0f, 0.0f},
            {1.0f, 1.0f, 0.1f, 1.0f, 1.0f, 0.1f},
            {2.0f, 0.25f, 5.0f},
        };
    }

    static PolicyParams jumpParams() {
        return {
            {0.2f, 0.4f, 0.0f, -0.2f, -0.4f, 0.0f},
            {0.2f, 0.4f, -0.2f, -0.4f},
            {6.0f, 6.0f, 0.0f, 6.0f, 6.0f, 0.0f},
            {0.5f, 0.5f, 0.2f, 0.5f, 0.5f, 0.2f},
            {3.0f, 0.25f, 5.0f},
        };
    }
};

} // namespace ega_policy
