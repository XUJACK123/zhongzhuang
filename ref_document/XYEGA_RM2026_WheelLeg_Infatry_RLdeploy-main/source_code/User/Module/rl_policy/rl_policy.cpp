#include "rl_policy.h"

#include <cstring>

#include "driver_dwt.h"
#include "main.h"

extern "C"
{
#include "jump.h"
#include "jump_data.h"
#include "pin.h"
#include "pin_data.h"
#include "stable.h"
#include "stable_data.h"
#include "upstairs.h"
#include "upstairs_data.h"
}

namespace ega
{
    namespace
    {
        struct NetworkContext
        {
            ai_handle network = AI_HANDLE_NULL;
            ai_buffer* inputs = nullptr;
            ai_buffer* outputs = nullptr;
            bool ready = false;
        };

        NetworkContext stable_ctx;
        NetworkContext upstairs_ctx;
        NetworkContext pin_ctx;
        NetworkContext jump_ctx;

        // stable 网络运行时的中间缓存，用来存各层临时结果。CubeAI 要求 4 字节对齐。
        AI_ALIGNED(4)
        ai_u8 stable_activations[AI_STABLE_DATA_ACTIVATION_1_SIZE];

        // upstairs 网络运行时的中间缓存。每个网络各用各的缓存，避免运行时互相覆盖。
        AI_ALIGNED(4)
        ai_u8 upstairs_activations[AI_UPSTAIRS_DATA_ACTIVATION_1_SIZE];

        // pin 网络运行时的中间缓存。
        AI_ALIGNED(4)
        ai_u8 pin_activations[AI_PIN_DATA_ACTIVATION_1_SIZE];

        // jump 网络运行时的中间缓存。
        AI_ALIGNED(4)
        ai_u8 jump_activations[AI_JUMP_DATA_ACTIVATION_1_SIZE];

        bool initStable()
        {
            if (stable_ctx.ready)
            {
                return true;
            }

            // activation_buffers 是给 CubeAI 的中间缓存地址表；这里只有 1 块缓存。
            // activation_buffers[0] 存的是 stable_activations 这块内存的起始地址。
            const ai_handle activation_buffers[] = {
                AI_HANDLE_PTR(stable_activations),
            };

            const ai_error create_error = ai_stable_create_and_init(
                &stable_ctx.network,
                activation_buffers,
                nullptr);
            if (create_error.type != AI_ERROR_NONE)
            {
                stable_ctx.ready = false;
                return false;
            }

            ai_u16 input_count = 0;
            ai_u16 output_count = 0;
            stable_ctx.inputs = ai_stable_inputs_get(stable_ctx.network, &input_count);
            stable_ctx.outputs = ai_stable_outputs_get(stable_ctx.network, &output_count);
            stable_ctx.ready = stable_ctx.inputs != nullptr
                            && stable_ctx.outputs != nullptr
                            && input_count == AI_STABLE_IN_NUM
                            && output_count == AI_STABLE_OUT_NUM;
            return stable_ctx.ready;
        }

        bool initUpstairs()
        {
            if (upstairs_ctx.ready)
            {
                return true;
            }

            // activation_buffers 是给 CubeAI 的中间缓存地址表；这里只有 1 块缓存。
            // activation_buffers[0] 存的是 upstairs_activations 这块内存的起始地址。
            const ai_handle activation_buffers[] = {
                AI_HANDLE_PTR(upstairs_activations),
            };

            const ai_error create_error = ai_upstairs_create_and_init(
                &upstairs_ctx.network,
                activation_buffers,
                nullptr);
            if (create_error.type != AI_ERROR_NONE)
            {
                upstairs_ctx.ready = false;
                return false;
            }

            ai_u16 input_count = 0;
            ai_u16 output_count = 0;
            upstairs_ctx.inputs = ai_upstairs_inputs_get(upstairs_ctx.network, &input_count);
            upstairs_ctx.outputs = ai_upstairs_outputs_get(upstairs_ctx.network, &output_count);
            upstairs_ctx.ready = upstairs_ctx.inputs != nullptr
                                && upstairs_ctx.outputs != nullptr
                                && input_count == AI_UPSTAIRS_IN_NUM
                                && output_count == AI_UPSTAIRS_OUT_NUM;
            return upstairs_ctx.ready;
        }

        bool initPin()
        {
            if (pin_ctx.ready)
            {
                return true;
            }

            const ai_handle activation_buffers[] = {
                AI_HANDLE_PTR(pin_activations),
            };

            const ai_error create_error = ai_pin_create_and_init(
                &pin_ctx.network,
                activation_buffers,
                nullptr);
            if (create_error.type != AI_ERROR_NONE)
            {
                pin_ctx.ready = false;
                return false;
            }

            ai_u16 input_count = 0;
            ai_u16 output_count = 0;
            pin_ctx.inputs = ai_pin_inputs_get(pin_ctx.network, &input_count);
            pin_ctx.outputs = ai_pin_outputs_get(pin_ctx.network, &output_count);
            pin_ctx.ready = pin_ctx.inputs != nullptr
                          && pin_ctx.outputs != nullptr
                          && input_count == AI_PIN_IN_NUM
                          && output_count == AI_PIN_OUT_NUM;
            return pin_ctx.ready;
        }

        bool initJump()
        {
            if (jump_ctx.ready)
            {
                return true;
            }

            const ai_handle activation_buffers[] = {
                AI_HANDLE_PTR(jump_activations),
            };

            const ai_error create_error = ai_jump_create_and_init(
                &jump_ctx.network,
                activation_buffers,
                nullptr);
            if (create_error.type != AI_ERROR_NONE)
            {
                jump_ctx.ready = false;
                return false;
            }

            ai_u16 input_count = 0;
            ai_u16 output_count = 0;
            jump_ctx.inputs = ai_jump_inputs_get(jump_ctx.network, &input_count);
            jump_ctx.outputs = ai_jump_outputs_get(jump_ctx.network, &output_count);
            jump_ctx.ready = jump_ctx.inputs != nullptr
                          && jump_ctx.outputs != nullptr
                          && input_count == AI_JUMP_IN_NUM
                          && output_count == AI_JUMP_OUT_NUM;
            return jump_ctx.ready;
        }

        NetworkContext& contextForModel(const RLPolicy::Model model)
        {
            switch (model)
            {
            case RLPolicy::Model::Upstairs:
                return upstairs_ctx;
            case RLPolicy::Model::Pin:
                return pin_ctx;
            case RLPolicy::Model::Jump:
                return jump_ctx;
            case RLPolicy::Model::Stable:
            default:
                return stable_ctx;
            }
        }
    }

    RLPolicy& RLPolicy::getInstance()
    {
        static RLPolicy instance;
        return instance;
    }

    bool RLPolicy::init()
    {
        if (ready_)
        {
            return true;
        }

        __HAL_RCC_CRC_CLK_ENABLE();

        ready_ = initStable() && initUpstairs() && initPin() && initJump();
        return ready_;
    }

    bool RLPolicy::run(const Model model,
                       const std::array<float, OBS_SIZE>& obs,
                       const std::array<float, OBS_HISTORY_SIZE>& obs_history,
                       std::array<float, ACTION_SIZE>& actions)
    {
        if (!ready_ && !init())
        {
            actions.fill(0.0f);
            return false;
        }

        auto& ctx = contextForModel(model);
        if (!ctx.ready)
        {
            actions.fill(0.0f);
            return false;
        }

        auto* obs_input = AI_BUFFER_DATA(&ctx.inputs[0], ai_float);
        auto* obs_history_input = AI_BUFFER_DATA(&ctx.inputs[1], ai_float);
        auto* actions_output = AI_BUFFER_DATA(&ctx.outputs[0], ai_float);
        if (obs_input == nullptr || obs_history_input == nullptr || actions_output == nullptr)
        {
            actions.fill(0.0f);
            return false;
        }

        const uint64_t inference_start_us = DWTInstance::getTimeline_us();
        ai_i32 processed_batches = 0;
        if (model == Model::Upstairs)
        {
            std::memcpy(obs_input, obs.data(), AI_UPSTAIRS_IN_1_SIZE_BYTES);
            std::memcpy(obs_history_input, obs_history.data(), AI_UPSTAIRS_IN_2_SIZE_BYTES);
            processed_batches = ai_upstairs_run(ctx.network, ctx.inputs, ctx.outputs);
        }
        else if (model == Model::Pin)
        {
            std::memcpy(obs_input, obs.data(), AI_PIN_IN_1_SIZE_BYTES);
            std::memcpy(obs_history_input, obs_history.data(), AI_PIN_IN_2_SIZE_BYTES);
            processed_batches = ai_pin_run(ctx.network, ctx.inputs, ctx.outputs);
        }
        else if (model == Model::Jump)
        {
            std::memcpy(obs_input, obs.data(), AI_JUMP_IN_1_SIZE_BYTES);
            std::memcpy(obs_history_input, obs_history.data(), AI_JUMP_IN_2_SIZE_BYTES);
            processed_batches = ai_jump_run(ctx.network, ctx.inputs, ctx.outputs);
        }
        else
        {
            std::memcpy(obs_input, obs.data(), AI_STABLE_IN_1_SIZE_BYTES);
            std::memcpy(obs_history_input, obs_history.data(), AI_STABLE_IN_2_SIZE_BYTES);
            processed_batches = ai_stable_run(ctx.network, ctx.inputs, ctx.outputs);
        }
        last_inference_us_ = static_cast<uint32_t>(DWTInstance::getTimeline_us() - inference_start_us);

        if (processed_batches != 1)
        {
            if (model == Model::Upstairs)
            {
                (void)ai_upstairs_get_error(ctx.network);
            }
            else if (model == Model::Pin)
            {
                (void)ai_pin_get_error(ctx.network);
            }
            else if (model == Model::Jump)
            {
                (void)ai_jump_get_error(ctx.network);
            }
            else
            {
                (void)ai_stable_get_error(ctx.network);
            }
            actions.fill(0.0f);
            return false;
        }

        if (model == Model::Upstairs)
        {
            std::memcpy(actions.data(), actions_output, AI_UPSTAIRS_OUT_1_SIZE_BYTES);
        }
        else if (model == Model::Pin)
        {
            std::memcpy(actions.data(), actions_output, AI_PIN_OUT_1_SIZE_BYTES);
        }
        else if (model == Model::Jump)
        {
            std::memcpy(actions.data(), actions_output, AI_JUMP_OUT_1_SIZE_BYTES);
        }
        else
        {
            std::memcpy(actions.data(), actions_output, AI_STABLE_OUT_1_SIZE_BYTES);
        }
        return true;
    }
}
