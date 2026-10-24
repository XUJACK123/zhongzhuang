// ============================================================
// 文件: chassis_main_loop_design.cpp
// 说明: 底盘控制主循环——伪代码
//       对齐原始 chassis.cpp 的 RL 部署主链路；硬件、恢复和功率控制保留接口
// ============================================================

#include "math_core.hpp"
#include "rl_policy_design.hpp"
#include "policy_params_design.hpp"

using namespace ega_math;
using namespace ega_rl;
using namespace ega_policy;

// ============================================================
// 数据结构
// ============================================================
struct SensorData {
    float gyro[3]{};
    float quaternion[4]{1.0f, 0.0f, 0.0f, 0.0f};
    float left_thigh_angle = 0.0f, left_thigh_vel = 0.0f;
    float left_shank_angle = 0.0f, left_shank_vel = 0.0f;
    float right_thigh_angle = 0.0f, right_thigh_vel = 0.0f;
    float right_shank_angle = 0.0f, right_shank_vel = 0.0f;
    float left_wheel_vel = 0.0f, right_wheel_vel = 0.0f;
};

struct Command {
    // 这里保存实车 updateRLCMD()/updatePolicy() 处理后的最终指令与策略；
    // 速度规划、变速 Spin、TOF 跳跃触发和跳后高度保持属于上层状态逻辑。
    float vx = 0.0f;
    float vz = 0.0f;
    float height = 0.20f;
    float jump_phase = 0.0f;
    ChassisPolicy policy = ChassisPolicy::Stop;
};

// ============================================================
// 主控制器
// ============================================================
class Controller {
public:
    void init() {
        rl_policy_fault_ = !RLPolicy::getInstance().init();
        math_core_ = MathCore();
    }

    void controlLoop(uint32_t now_ms) {
        // ============================================================
        // 控制频率说明:
        //   本函数由定时器以 500Hz (2ms) 调用
        //   - 执行层 (运动学/力矩计算/下发): 500Hz, 每周期都跑
        //   - RL 推理层 (观测构建/历史堆叠/NN推理): 100Hz, 每5个周期跑一次
        // ============================================================
        loop_count_++;

        // 1. 读取传感器
        readSensors();

        // 2. 更新运动学
        math_core_.updateLegKinematics(
            sensor_.left_thigh_angle, sensor_.left_shank_angle,
            sensor_.right_thigh_angle, sensor_.right_shank_angle,
            sensor_.left_thigh_vel, sensor_.left_shank_vel,
            sensor_.right_thigh_vel, sensor_.right_shank_vel,
            sensor_.left_wheel_vel, sensor_.right_wheel_vel
        );

        // 3. 获取策略参数
        PolicyParams params = PolicyParamsTable::get(cmd_.policy);

        // 4. 策略变化时清空历史观测和上一动作，与实车 updatePolicy() 一致
        handlePolicyTransition();

        // ===== 100Hz RL 推理分支 (每 5 个控制周期执行一次) =====
        if (loop_count_ % 5U == 0U) {
        // 5. 构建 RL 观测（使用 ega_math 的 OBS_SIZE）
        // 注意: 指令在此处用 params.cmd_scale 缩放一次，buildRLObservation 内部不再缩放
        std::array<float, ega_math::OBS_SIZE> obs;
        math_core_.buildRLObservation(
            {sensor_.gyro[0], sensor_.gyro[1], sensor_.gyro[2]},
            {sensor_.quaternion[0], sensor_.quaternion[1],
             sensor_.quaternion[2], sensor_.quaternion[3]},
            {cmd_.vx * params.cmd_scale[0],
             cmd_.vz * params.cmd_scale[1],
             cmd_.height * params.cmd_scale[2]},
            params.obs_dof_pos,
            obs
        );

        // 6. 更新观测历史堆叠
        math_core_.updateObservationHistory(obs);
        const auto& obs_history = math_core_.getObservationHistory();

        // 7. RL 推理（使用 RLPolicy::ACTION_SIZE）
        RLPolicy::Model model = policyToModel(cmd_.policy);
        if (!RLPolicy::getInstance().run(model, obs, obs_history, actions_)) {
            actions_.fill(0.0f);
            rl_policy_fault_ = true;
        }
        } // ===== 100Hz 推理分支结束 =====

        // ===== 500Hz 执行层 (每个控制周期都执行，复用最近一次的 actions_) =====
        // 8. 计算 RL 力矩（使用 ega_math 的 DOF）
        // PD 参数按当前策略动态传入 (Stable/Spin/Jump/MiniRecover 各不同)
        bool is_spin = (cmd_.policy == ChassisPolicy::Spin);
        bool is_jump = (cmd_.policy == ChassisPolicy::Jump);
        std::array<float, ega_math::DOF> rl_torques;
        math_core_.calculateRLTorques(
            actions_,
            params.dof_pos,
            params.p_gains,
            params.d_gains,
            is_spin,
            is_jump,
            cmd_.jump_phase,
            rl_torques
        );

        // 9. 500Hz 最终策略/安全仲裁。Stop 必须在每个周期清零，不能等到
        //    下一次 100Hz 推理；LargeRecover 在这里覆盖 RL 力矩。
        applyFinalPolicyAndSafety(now_ms, rl_torques);

        // 10. 功率控制（限制轮子力矩）
        applyPowerControl(rl_torques);

        // 11. 下发力矩到电机
        sendTorquesToMotors(rl_torques);
    }

private:
    SensorData sensor_{};
    Command cmd_{};
    MathCore math_core_;
    uint32_t loop_count_ = 0;   // 控制周期计数器 (用于推理降频)
    std::array<float, RLPolicy::ACTION_SIZE> actions_ = {};      // 最近一次 RL 推理输出 (跨周期保持)
    ChassisPolicy last_policy_ = ChassisPolicy::Stop;
    bool rl_policy_fault_ = false;

    void readSensors() {
        // 读 IMU: gyro, quaternion, pitch → sensor_
        // 读裁判系统: 剩余能量 → sensor_
        // 读 TOF 测距 (左) → sensor_

        // 读电机编码器原始角度 (实机零点)，硬件读取后填充
        float raw_lt = 0.0f, raw_ls = 0.0f, raw_rt = 0.0f, raw_rs = 0.0f;
        float raw_lt_vel = 0.0f, raw_ls_vel = 0.0f;
        float raw_rt_vel = 0.0f, raw_rs_vel = 0.0f;
        float raw_lw_vel = 0.0f, raw_rw_vel = 0.0f;

        // --- 电机偏置转换: 实机零点 → 策略零点 ---
        // 实机编码器零点与 RL 训练时关节零点不一致，必须统一后才能送入推理链路
        // 左大腿: -thigh_offset, 右大腿: +thigh_offset
        // 左小腿: +shank_offset, 右小腿: -shank_offset
        // (轮子速度取反在运动学更新中处理)
        sensor_.left_thigh_angle  = MathCore::wrapToPi(raw_lt - kThighOffset);
        sensor_.left_shank_angle  = MathCore::wrapToPi(raw_ls + kShankOffset);
        sensor_.right_thigh_angle = MathCore::wrapToPi(raw_rt + kThighOffset);
        sensor_.right_shank_angle = MathCore::wrapToPi(raw_rs - kShankOffset);
        sensor_.left_thigh_vel = raw_lt_vel;
        sensor_.left_shank_vel = raw_ls_vel;
        sensor_.right_thigh_vel = raw_rt_vel;
        sensor_.right_shank_vel = raw_rs_vel;
        // 与实车 getRLMeasure() 一致，两个轮速进入策略前均取反。
        sensor_.left_wheel_vel  = -raw_lw_vel;
        sensor_.right_wheel_vel = -raw_rw_vel;
    }

    void handlePolicyTransition() {
        if (cmd_.policy == last_policy_) return;
        math_core_.resetObservationHistory();
        math_core_.resetLastActions();
        last_policy_ = cmd_.policy;
    }

    // ----- 策略 → 模型 映射表 -----
    // ChassisPolicy    →  Model      说明
    // ──────────────────────────────────────────────────
    // MiniRecover      →  Upstairs   默认策略
    // Stable           →  Stable     稳定行走
    // Spin             →  Pin        原地旋转
    // Jump             →  Jump       跳跃
    // 其他策略          →  Stable     与实车默认分支一致；最终力矩随后被仲裁
    RLPolicy::Model policyToModel(ChassisPolicy policy) {
        switch (policy) {
            case ChassisPolicy::Stable:      return RLPolicy::Model::Stable;
            case ChassisPolicy::Spin:        return RLPolicy::Model::Pin;
            case ChassisPolicy::Jump:        return RLPolicy::Model::Jump;
            case ChassisPolicy::MiniRecover: return RLPolicy::Model::Upstairs;
            case ChassisPolicy::Walk:
            case ChassisPolicy::Down:
            case ChassisPolicy::LargeRecover_Auto:
            case ChassisPolicy::LargeRecover_Crazy:
            case ChassisPolicy::Stop:
            default:                         return RLPolicy::Model::Stable;
        }
    }

    static bool isLargeRecoverPolicy(ChassisPolicy policy) {
        return policy == ChassisPolicy::LargeRecover_Auto ||
               policy == ChassisPolicy::LargeRecover_Crazy;
    }

    // ----- 翻倒自救（思路） -----
    void updateRecoverFSM(uint32_t now_ms) {
        (void)now_ms;
        // 仅在 LargeRecover_Auto / LargeRecover_Crazy 模式下生效
        // 状态机：FinalStand ↔ PositiveSwing ↔ PositiveSecondSwing
        //               ↔ NegativeSwing ↔ NegativeSecondSwing
        // 输入：pitch, roll, gyro, left_recover_phi, right_recover_phi
        // 决策逻辑：
        //   - pitch 回正 → FinalStand
        //   - pitch > 阈值 → Positive 方向，单腿依次摆动到支撑位
        //   - 支撑位稳定后 → SecondSwing 双腿二次摆回站立位
        //   - 反向翻倒时立即切换方向
        // 单腿/双腿控制：根据 roll 决定先动腿，到达后释放另一条腿
        // 输出：allow_left/allow_right, left_target, right_target,
        //       left_dir, right_dir, stop_angle
        //                        pitch > 1.2
        // FinalStand -------------------> PositiveSwing
        //     |                               |
        //     | pitch < -1.2                  | 两腿到达支撑位 + gyro稳定
        //     |                               ↓
        //     |                       PositiveSecondSwing
        //     |                               |
        //     |                     pitch回正(<0.5) 或 兜底退出
        //     |                               |
        //     |<------------------------------+
        //     |
        //     |                               pitch < -1.2
        //     +--------------------------> NegativeSwing
        //                                     |
        //                                     | 两腿到达支撑位 + gyro稳定
        //                                     ↓
        //                             NegativeSecondSwing
        //                                     |
        //                                   pitch回正(<0.5) 或 兜底退出
        //                                     |
        //                                     +----------> FinalStand
    }

    void applyRecoverTorques(std::array<float, DOF>& torques) {
        (void)torques;
        // 若为 LargeRecover_Auto / LargeRecover_Crazy，用恢复力矩覆盖四个腿关节
        // 物理电机索引: LEFT_THIGH(0), RIGHT_THIGH(1), RIGHT_SHANK(2), LEFT_SHANK(3)
        // 恢复力矩 = 速度环 PI（目标角速度 - 当前角速度）
        //           + 虚拟腿长控制（目标腿长 - 当前腿长）
        // 根据 allow_left/allow_right 决定是否驱动该腿
    }

    void applyFinalPolicyAndSafety(uint32_t now_ms, std::array<float, DOF>& torques) {
        // 实车还会在这里处理轮电机离线和 |pitch| > 1.4rad 的刹车保护；
        // 这些依赖硬件状态，移植时必须接入，不能直接省略。
        if (rl_policy_fault_) {
            torques.fill(0.0f);
            return;
        }

        if (cmd_.policy == ChassisPolicy::Stop) {
            torques.fill(0.0f);
            return;
        }

        if (isLargeRecoverPolicy(cmd_.policy)) {
            updateRecoverFSM(now_ms);
            torques.fill(0.0f);
            applyRecoverTorques(torques);
        }

        // 对应实车 calculateRLFinalTorques() 的腿部最终限幅。
        for (size_t i = LEFT_THIGH; i <= LEFT_SHANK; ++i) {
            torques[i] = MathCore::clipf(
                torques[i], -kParallelTorqueLimit, kParallelTorqueLimit);
        }
    }

    // ----- 功率控制（思路） -----
    void applyPowerControl(std::array<float, DOF>& torques) {
        (void)torques;
        // 读取裁判系统剩余能量 + 超级电容电量
        // 计算可用总功率
        // RLS 在线辨识轮子损耗参数（k1, k2）
        // 根据可用功率动态限制轮子力矩 (物理电机索引: RIGHT_WHEEL=4, LEFT_WHEEL=5)
        // 用的是港科开源的功率控制算法，具体参考https://bbs.robomaster.com/article/54121?source=4
        // 由于是强化学习控制的，不好区分用于维持姿态的力和用于运动的力，所以没有采用balancePower中的功率控制
        // 得益于强化学习强大的鲁棒性，只需直接clip力矩就行，具体就是把港科的powercontrol.h里面的函数改写成不分配，直接clip的形式，很暴力；
        // 当然为了在各个功率下都能正常运动，给出的cmd要随着功率变化而变化，我们采取的是低功率时线性插值的方法；
    }

    void sendTorquesToMotors(const std::array<float, DOF>& torques) {
        (void)torques;
        // torques 为物理电机通道索引 (左右交替排列):
        //   [0]LEFT_THIGH, [1]RIGHT_THIGH, [2]RIGHT_SHANK, [3]LEFT_SHANK, [4]RIGHT_WHEEL, [5]LEFT_WHEEL
        // 力矩 → effort 映射 → 步进限幅 → 写 CAN
    }
};

// 全局入口
static Controller g_chassis;
void chassis_init() { g_chassis.init(); }
void chassis_loop(uint32_t now_ms) { g_chassis.controlLoop(now_ms); }
