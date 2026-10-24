#pragma once

#include <array>
#include <cstdint>

namespace ega
{
    class RLPolicy
    {
    public:
        enum class Model : uint8_t
        {
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
        [[nodiscard]] uint32_t lastInferenceUs() const { return last_inference_us_; }

    private:
        RLPolicy() = default;

        bool ready_ = false;
        uint32_t last_inference_us_ = 0;
    };
}
