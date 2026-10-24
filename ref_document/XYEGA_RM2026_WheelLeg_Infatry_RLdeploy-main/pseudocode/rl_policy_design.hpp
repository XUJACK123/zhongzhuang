// ============================================================
// 文件: rl_policy_design.hpp
// 说明: RL 策略推理调度器——基于源码 rl_policy.h 剥离硬件依赖
//       去掉了 DWT 计时相关的实际实现，保留接口框架
// ============================================================
#pragma once

#include <array>
#include <cstdint>

namespace ega_rl {

class RLPolicy {
public:
    enum class Model : uint8_t {
        Stable,
        Upstairs,
        Pin,
        Jump,
    };

    static constexpr uint8_t OBS_SIZE = 25;
    static constexpr uint8_t OBS_HISTORY_SIZE = 125;
    static constexpr uint8_t ACTION_SIZE = 6;

    static RLPolicy& getInstance();

    bool init();

    bool run(Model model,
             const std::array<float, OBS_SIZE>& obs,
             const std::array<float, OBS_HISTORY_SIZE>& obs_history,
             std::array<float, ACTION_SIZE>& actions);

    [[nodiscard]] bool isReady() const { return ready_; }

    // 源码中用 DWT 测量单次推理耗时，剥离后恒为 0
    // 需要时可恢复：const uint64_t t0 = DWTInstance::getTimeline_us();
    //                last_inference_us_ = DWTInstance::getTimeline_us() - t0;
    [[nodiscard]] uint32_t lastInferenceUs() const { return last_inference_us_; }

private:
    RLPolicy() = default;
    bool ready_ = false;
    uint32_t last_inference_us_ = 0;
};

} // namespace ega_rl
