#include "CAN_receive.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "fdcan.h"
#include "task.h"

#define DM_J8009_POSITION_MIN (-3.14f)
#define DM_J8009_POSITION_MAX 3.14f
#define DM_J8009_SPEED_MIN (-45.0f)
#define DM_J8009_SPEED_MAX 45.0f
#define DM_J8009_TORQUE_MIN (-54.0f)
#define DM_J8009_TORQUE_MAX 54.0f

#define JOINT_INIT_RETRY_LIMIT 3U
#define JOINT_INIT_PHASE_DELAY_MS 20U

typedef struct
{
    bool initialized;
    uint16_t encoder;
    uint16_t last_encoder;
    int32_t turns;
    int16_t speed_rpm;
    int16_t current;
} wheel_raw_t;

static volatile motor_can_snapshot_t motor_snapshot;
static wheel_raw_t wheel_raw[2];
static uint8_t joint_init_phase;
static uint8_t joint_init_attempts;
static uint32_t joint_init_next_ms;

static float uint_to_float(uint32_t value, float minimum, float maximum, uint8_t bits)
{
    return (float)value * (maximum - minimum) / (float)((1UL << bits) - 1UL) + minimum;
}

static uint16_t float_to_uint(float value, float minimum, float maximum, uint8_t bits)
{
    if (value > maximum)
    {
        value = maximum;
    }
    else if (value < minimum)
    {
        value = minimum;
    }
    return (uint16_t)((value - minimum) * (float)((1UL << bits) - 1UL) / (maximum - minimum));
}

static bool configure_bus(FDCAN_HandleTypeDef *bus)
{
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;

    return HAL_FDCAN_ConfigFilter(bus, &filter) == HAL_OK &&
           HAL_FDCAN_ConfigGlobalFilter(bus, FDCAN_REJECT, FDCAN_REJECT,
                                        FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) == HAL_OK &&
           HAL_FDCAN_ActivateNotification(bus, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) == HAL_OK &&
           HAL_FDCAN_Start(bus) == HAL_OK;
}

bool motor_can_init(void)
{
    memset((void *)&motor_snapshot, 0, sizeof(motor_snapshot));
    memset(wheel_raw, 0, sizeof(wheel_raw));
    motor_can_joint_init_reset();
    return configure_bus(&hfdcan1) && configure_bus(&hfdcan2);
}

void motor_can_get_snapshot(motor_can_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }
    taskENTER_CRITICAL();
    *snapshot = motor_snapshot;
    taskEXIT_CRITICAL();
}

static void update_wheel(size_t motor_index, size_t raw_index, const uint8_t data[8])
{
    wheel_raw_t *raw = &wheel_raw[raw_index];
    const uint16_t encoder = (uint16_t)((data[0] << 8) | data[1]);

    raw->last_encoder = raw->encoder;
    raw->encoder = encoder;
    raw->speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    raw->current = (int16_t)((data[4] << 8) | data[5]);

    if (raw->initialized)
    {
        raw->turns = chassis_update_encoder_turns(raw->encoder, raw->last_encoder, raw->turns);
    }
    else
    {
        /* 第一帧只能建立零点，不能把上电编码值误判为跨圈。 */
        raw->initialized = true;
    }

    motor_snapshot.motor[motor_index].position_rad =
        ((float)(raw->turns * 8192L + raw->encoder) / 8192.0f) *
        (2.0f * 3.14159265358979323846f) / CHASSIS_WHEEL_GEAR_RATIO;
    motor_snapshot.motor[motor_index].speed_rad_s = chassis_wheel_rpm_to_rad_s(raw->speed_rpm);
    motor_snapshot.motor[motor_index].torque_nm = chassis_wheel_current_to_torque(raw->current);
    motor_snapshot.motor[motor_index].status = 1U;
    motor_snapshot.motor[motor_index].update_time_ms = HAL_GetTick();
    motor_snapshot.motor[motor_index].valid = true;
}

static void update_joint(size_t motor_index, const uint8_t data[8])
{
    const uint16_t position = (uint16_t)((data[1] << 8) | data[2]);
    const uint16_t speed = (uint16_t)((data[3] << 4) | (data[4] >> 4));
    const uint16_t torque = (uint16_t)(((data[4] & 0x0fU) << 8) | data[5]);

    motor_snapshot.motor[motor_index].status = (uint8_t)((data[0] >> 4) & 0x0fU);
    motor_snapshot.motor[motor_index].position_rad =
        uint_to_float(position, DM_J8009_POSITION_MIN, DM_J8009_POSITION_MAX, 16U);
    motor_snapshot.motor[motor_index].speed_rad_s =
        uint_to_float(speed, DM_J8009_SPEED_MIN, DM_J8009_SPEED_MAX, 12U);
    motor_snapshot.motor[motor_index].torque_nm =
        uint_to_float(torque, DM_J8009_TORQUE_MIN, DM_J8009_TORQUE_MAX, 12U);
    motor_snapshot.motor[motor_index].update_time_ms = HAL_GetTick();
    motor_snapshot.motor[motor_index].valid = true;
}

static void process_rx(FDCAN_HandleTypeDef *bus, const FDCAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    size_t motor_index;
    bool is_wheel;
    uint8_t bus_number;

    bus_number = bus == &hfdcan1 ? 1U : (bus == &hfdcan2 ? 2U : 0U);
    if (!chassis_validate_can_feedback(bus_number,
                                       header->Identifier,
                                       header->IdType == FDCAN_STANDARD_ID,
                                       header->RxFrameType == FDCAN_DATA_FRAME,
                                       header->DataLength == FDCAN_DLC_BYTES_8 ? 8U : 0U,
                                       &motor_index,
                                       &is_wheel))
    {
        return;
    }
    if (is_wheel)
    {
        update_wheel(motor_index, bus_number == 1U ? 0U : 1U, data);
    }
    else
    {
        update_joint(motor_index, data);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *bus, uint32_t interrupt_flags)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if ((interrupt_flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(bus, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(bus, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }
        process_rx(bus, &header, data);
    }
}

static bool send_frame(FDCAN_HandleTypeDef *bus, uint32_t identifier, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = identifier;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(bus, &header, data) == HAL_OK;
}

static bool send_joint_torque(FDCAN_HandleTypeDef *bus, uint32_t identifier, float torque_nm)
{
    uint8_t data[8] = {0};
    const uint16_t encoded = float_to_uint(torque_nm,
                                           DM_J8009_TORQUE_MIN,
                                           DM_J8009_TORQUE_MAX,
                                           12U);
    data[6] = (uint8_t)(encoded >> 8);
    data[7] = (uint8_t)encoded;
    return send_frame(bus, identifier, data);
}

static bool send_wheel_torque(FDCAN_HandleTypeDef *bus, bool right, float torque_nm)
{
    uint8_t data[8] = {0};
    const int16_t current = chassis_wheel_torque_to_current(torque_nm);
    const size_t offset = right ? 2U : 0U;
    data[offset] = (uint8_t)((uint16_t)current >> 8);
    data[offset + 1U] = (uint8_t)current;
    return send_frame(bus, 0x200U, data);
}

bool motor_can_send_torques(const float requested_nm[CHASSIS_MOTOR_COUNT],
                            float applied_nm[CHASSIS_MOTOR_COUNT])
{
    bool ok = true;

    if (!chassis_torque_vector_valid(requested_nm) || applied_nm == NULL)
    {
        return false;
    }

    for (size_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i)
    {
        applied_nm[i] = chassis_limit_torque(i, requested_nm[i]);
    }

    ok &= send_joint_torque(&hfdcan1, 0x003U, applied_nm[CHASSIS_MOTOR_LEFT_FRONT]);
    ok &= send_joint_torque(&hfdcan1, 0x004U, applied_nm[CHASSIS_MOTOR_LEFT_REAR]);
    ok &= send_joint_torque(&hfdcan2, 0x002U, applied_nm[CHASSIS_MOTOR_RIGHT_FRONT]);
    ok &= send_joint_torque(&hfdcan2, 0x001U, applied_nm[CHASSIS_MOTOR_RIGHT_REAR]);
    ok &= send_wheel_torque(&hfdcan1, false, applied_nm[CHASSIS_MOTOR_LEFT_WHEEL]);
    ok &= send_wheel_torque(&hfdcan2, true, applied_nm[CHASSIS_MOTOR_RIGHT_WHEEL]);
    return ok;
}

bool motor_can_send_zero(void)
{
    float zero[CHASSIS_MOTOR_COUNT] = {0};
    float applied[CHASSIS_MOTOR_COUNT];
    return motor_can_send_torques(zero, applied);
}

static bool send_joint_special(FDCAN_HandleTypeDef *bus, uint32_t identifier, uint8_t command)
{
    uint8_t data[8] = {0xffU, 0xffU, 0xffU, 0xffU, 0xffU, 0xffU, 0xffU, command};
    return send_frame(bus, identifier, data);
}

static bool send_all_joint_special(uint8_t command)
{
    bool ok = true;
    ok &= send_joint_special(&hfdcan1, 0x003U, command);
    ok &= send_joint_special(&hfdcan1, 0x004U, command);
    ok &= send_joint_special(&hfdcan2, 0x002U, command);
    ok &= send_joint_special(&hfdcan2, 0x001U, command);
    return ok;
}

void motor_can_joint_init_reset(void)
{
    joint_init_phase = 0U;
    joint_init_attempts = 0U;
    joint_init_next_ms = 0U;
}

bool motor_can_all_joints_enabled(void)
{
    motor_can_snapshot_t snapshot;
    motor_can_get_snapshot(&snapshot);
    for (size_t i = 0; i < CHASSIS_MOTOR_LEFT_WHEEL; ++i)
    {
        if (!snapshot.motor[i].valid || snapshot.motor[i].status != 1U)
        {
            return false;
        }
    }
    return true;
}

bool motor_can_joint_init_step(uint32_t now_ms)
{
    if (motor_can_all_joints_enabled())
    {
        return true;
    }
    if (joint_init_attempts >= JOINT_INIT_RETRY_LIMIT ||
        (joint_init_next_ms != 0U && (int32_t)(now_ms - joint_init_next_ms) < 0))
    {
        return false;
    }

    if (joint_init_phase == 0U)
    {
        const bool sent = send_all_joint_special(0xfbU); /* 清除故障。 */
        if (sent)
        {
            joint_init_phase = 1U;
        }
        else
        {
            ++joint_init_attempts;
        }
    }
    else
    {
        /* 无论本次入队是否成功，都消耗一次额度，避免故障总线上无限重试。 */
        (void)send_all_joint_special(0xfcU); /* 使能并进入 MIT 力矩模式。 */
        joint_init_phase = 0U;
        ++joint_init_attempts;
    }
    joint_init_next_ms = now_ms + JOINT_INIT_PHASE_DELAY_MS;
    return false;
}
