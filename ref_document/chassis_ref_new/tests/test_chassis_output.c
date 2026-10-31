/* 执行真实 chassis_task 循环；仅替换硬件快照、CAN 和调度器。
 * 跳过 INS 算法依赖，提供本测试所需的快照类型。
 */
#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define INS_TASK_H
typedef struct {
    float quaternion[4];
    float euler_rad[3];
    float gyro_rad_s[3];
    float acceleration_m_s2[3];
    uint32_t update_time_ms;
    bool valid;
} ins_snapshot_t;
void INS_get_snapshot(ins_snapshot_t *snapshot);
uint32_t HAL_GetTick(void);
#include "../Task/chassis_task.c"

typedef struct {
    bool valid;
    uint32_t age;
    uint8_t right;
    uint8_t left;
    bool submit;
    bool expect_motion;
} cycle_t;
/* 即便生产者一直提交非零命令，也不能越过遥控许可。 */
#if TEST_MOTOR_ENABLE
static const cycle_t cycles[] = {
    {false, 0,   1, 1, true,  false},
    {true,  0,   1, 2, true,  true},  /* 上电即上档；左拨杆下档不阻止 */
    {true,  0,   1, 3, true,  true},
    {true,  100, 1, 1, true,  true},
    {true,  101, 1, 1, true,  false}, /* 超时必须归零 */
    {true,  0,   1, 1, false, false}, /* 不能复用断连前旧命令 */
    {true,  0,   1, 1, true,  true},  /* 新命令恢复，无需下档解锁 */
    {false, 0,   1, 1, true,  false},
    {true,  0,   1, 1, true,  true},
    {true,  0,   2, 1, true,  false}, /* 下档不使能，只发零 */
    {true,  0,   1, 1, false, false},
    {true,  0,   1, 1, true,  true},
    {true,  0,   3, 1, true,  false}, /* 中档停止 */
    {false, 0,   2, 1, true,  false},
};
#else
static const cycle_t cycles[] = {
    {false, 0,   1, 1, true,  false}, /* 未收到遥控 */
    {true,  0,   1, 1, true,  false}, /* 上电上档不能解锁 */
    {true,  0,   2, 1, true,  false}, /* 下档恢复 */
    {true,  0,   1, 1, true,  false}, /* 上档等待新命令 */
    {true,  0,   1, 1, true,  true},
    {true,  100, 1, 1, true,  true},  /* 超时边界 */
    {true,  101, 1, 1, true,  false}, /* valid 未清也必须停止 */
    {true,  0,   1, 1, true,  false}, /* 重连上档不能自动恢复 */
    {true,  0,   2, 1, true,  false},
    {true,  0,   1, 1, true,  false},
    {true,  0,   1, 1, true,  true},
    {false, 0,   1, 1, true,  false}, /* 运行中解码/UART 失效 */
    {true,  0,   1, 1, true,  false},
    {true,  0,   2, 1, true,  false},
    {true,  0,   1, 1, true,  false},
    {true,  0,   1, 1, true,  true},
    {true,  0,   1, 3, true,  true},  /* 正式底座仅要求右拨杆上档 */
    {true,  0,   1, 2, true,  true},
    {true,  0,   1, 1, true,  true},
    {true,  0,   3, 1, true,  false}, /* 右拨杆中档 */
    {true,  0,   2, 1, true,  false}, /* 右拨杆下档 */
    {false, 0,   2, 1, true,  false}, /* 无遥控不能发送关节使能 */
};
#endif

static size_t cycle;
static uint32_t now_ms;
static unsigned motion_calls, zero_calls, init_calls;
static jmp_buf finished;

uint32_t HAL_GetTick(void) { return now_ms; }
TickType_t xTaskGetTickCount(void) { return now_ms; }
bool motor_can_init(void) { return true; }
void motor_can_joint_init_reset(void) {}
bool motor_can_joint_init_step(uint32_t now) {
    (void)now;
    ++init_calls;
    return true;
}
void motor_can_get_snapshot(motor_can_snapshot_t *snapshot) {
    memset(snapshot, 0, sizeof(*snapshot));
    for (size_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i) {
        snapshot->motor[i].valid = true;
        snapshot->motor[i].status = 1U;
        snapshot->motor[i].update_time_ms = now_ms;
    }
}
void INS_get_snapshot(ins_snapshot_t *snapshot) {
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->valid = !TEST_MOTOR_ENABLE; /* 台架故意模拟 IMU 未就绪 */
    snapshot->update_time_ms = now_ms;
}
void remote_control_get_snapshot(remote_control_snapshot_t *snapshot) {
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->valid = cycles[cycle].valid;
    snapshot->update_time_ms = now_ms - cycles[cycle].age;
    snapshot->frame.switch_right = cycles[cycle].right;
    snapshot->frame.switch_left = cycles[cycle].left;
}
bool motor_can_send_torques(const float requested[CHASSIS_MOTOR_COUNT],
                            float applied[CHASSIS_MOTOR_COUNT]) {
    assert(cycles[cycle].expect_motion);
    assert(requested[0] == 0.1f);
    memcpy(applied, requested, sizeof(float) * CHASSIS_MOTOR_COUNT);
    ++motion_calls;
    return true;
}
bool motor_can_send_zero(void) {
    assert(!cycles[cycle].expect_motion);
    ++zero_calls;
    return true;
}
void vTaskDelayUntil(TickType_t *wake_time, TickType_t period) {
    const cycle_t *current = &cycles[cycle];
    assert(motion_calls == (current->expect_motion ? 1U : 0U));
    assert(zero_calls == (current->expect_motion ? 0U : 1U));
    assert(init_calls == (current->valid && current->age <= 100U && current->right == (TEST_MOTOR_ENABLE ? 1U : 2U) ? 1U : 0U));
    chassis_feedback_t feedback;
    assert(chassis_get_feedback(&feedback));
    if (!current->expect_motion)
        for (size_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i)
            assert(feedback.applied_torque_nm[i] == 0.0f);
    if (!current->valid || current->age > 100U) {
        assert(feedback.state == CHASSIS_STATE_LOCKED);
        assert(feedback.fault_flags & CHASSIS_FAULT_DBUS_TIMEOUT);
    }
    if (++cycle == sizeof(cycles) / sizeof(cycles[0])) longjmp(finished, 1);
    motion_calls = zero_calls = init_calls = 0;
    now_ms += period;
    *wake_time = now_ms;
    if (cycles[cycle].submit) {
        const float requested[CHASSIS_MOTOR_COUNT] = {0.1f};
        assert(chassis_submit_torque(requested));
    }
}
int main(void) {
    /* 第二轮跨越 uint32 毫秒回绕，验证时间差判断。 */
    const uint32_t starts[] = {1000U, UINT32_MAX - 10U};
    for (size_t run = 0; run < 2U; ++run) {
        cycle = 0;
        now_ms = starts[run];
        motion_calls = zero_calls = init_calls = 0;
        if (setjmp(finished) == 0) chassis_task(NULL);
    }
    puts("chassis output guard tests passed");
    return 0;
}
