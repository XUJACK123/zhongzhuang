/* STM32H723 / DM-MC02: discover one DM8009, then DBUS both-up -> +20 degrees. */
#include "main.h"
#include "dbus_remote.h"
#include "dm8009_app.h"
#include "board_led.h"
#if DM_DIRECT_TEST
#include "dm8009_direct.h"
volatile uint32_t g_direct_start_request, g_direct_abort_request;
volatile DM_DirectTest g_direct;
#endif
_Static_assert(FDCAN_DLC_BYTES_8 == 8U, "Review HAL DLC convention");
typedef struct {
    uint32_t sequence, tick_ms, id, id_type, frame_type, dlc;
    uint8_t data[8];
} CAN_RawFrame;
/* Read-only snapshots. Live Watch edits do not bypass the remote interlock. */
volatile DM_App g_dm;
volatile uint32_t g_init_stage, g_error;
volatile uint32_t g_can_tx_queued, g_can_tx_failed, g_can_rx_total;
volatile uint32_t g_can_rx_lost, g_can_rx_failed;
volatile uint32_t g_can_bus_off, g_can_error_passive, g_can_last_error;
volatile uint32_t g_can_tx_error_count, g_can_rx_error_count, g_can_status_failed;
volatile uint32_t g_can_last_hal_error, g_can_tx_free;
volatile CAN_RawFrame g_can_last_rx, g_can_rx_ring[32];
/* Preserve motor replies even if another ID uploads at high rate. */
volatile CAN_RawFrame g_can_motor_rx_ring[32];
volatile uint32_t g_can_motor_rx_count, g_can_disable_tx_count, g_can_enable_tx_count;
volatile uint32_t g_can_last_tx_id;
volatile uint8_t g_can_last_tx_data[8];
static uint32_t rx_index;
static int Send(void *context, uint16_t id, const uint8_t data[8]) {
    (void)context;
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier = id; tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME; tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE; tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.BitRateSwitch = FDCAN_BRS_OFF; tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) &&
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx, (uint8_t *)data) == HAL_OK) {
        ++g_can_tx_queued;
        g_can_last_tx_id=id;
        for(uint32_t i=0;i<8U;++i)g_can_last_tx_data[i]=data[i];
        if(data[0]==0xFFU && data[1]==0xFFU && data[2]==0xFFU) {
            if(data[7]==0xFDU)++g_can_disable_tx_count;
            if(data[7]==0xFCU)++g_can_enable_tx_count;
        }
        return 1;
    }
    ++g_can_tx_failed; return 0;
}
static void Cancel(void *context) {
    (void)context;
    (void)HAL_FDCAN_AbortTxRequest(&hfdcan2, 0xFFFFU);
}
static int BusOK(void) {
    FDCAN_ProtocolStatusTypeDef status = {0};
    FDCAN_ErrorCountersTypeDef errors = {0};
    int ok = HAL_FDCAN_GetProtocolStatus(&hfdcan2, &status) == HAL_OK;
    if (ok) {
        g_can_bus_off = status.BusOff; g_can_error_passive = status.ErrorPassive;
        g_can_last_error = status.LastErrorCode;
    } else ++g_can_status_failed;
    if (HAL_FDCAN_GetErrorCounters(&hfdcan2, &errors) == HAL_OK) {
        g_can_tx_error_count = errors.TxErrorCnt; g_can_rx_error_count = errors.RxErrorCnt;
    } else { ++g_can_status_failed; ok = 0; }
    g_can_tx_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
    g_can_last_hal_error = HAL_FDCAN_GetError(&hfdcan2);
    return ok && !status.BusOff;
}
static void Receive(DM_App *a) {
    if (__HAL_FDCAN_GET_FLAG(&hfdcan2, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST)) {
        ++g_can_rx_lost;
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan2, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
    }
    uint32_t pending = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0);
    while (pending--) {
        FDCAN_RxHeaderTypeDef rx = {0}; uint8_t data[64] = {0};
        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
            ++g_can_rx_failed; break;
        }
        CAN_RawFrame raw = { .sequence = ++g_can_rx_total, .tick_ms = HAL_GetTick(),
            .id = rx.Identifier, .id_type = rx.IdType, .frame_type = rx.RxFrameType,
            .dlc = rx.DataLength };
        uint32_t length = DM_DlcToLength(rx.DataLength);
        for (uint32_t i = 0; i < 8U && i < length; ++i) raw.data[i] = data[i];
        g_can_last_rx = raw; g_can_rx_ring[rx_index] = raw; rx_index = (rx_index+1U)%32U;
        if (rx.IdType == FDCAN_STANDARD_ID && rx.RxFrameType == FDCAN_DATA_FRAME &&
            rx.FDFormat == FDCAN_CLASSIC_CAN && length == 8U) {
            if(a->motor_id && (rx.Identifier==a->master_id ||
                (DM_USE_MULTI && rx.Identifier==0x300U+a->motor_id))) {
#if DM_DIRECT_TEST
                /* Freeze the last motor frames at the first terminal result. */
                if(g_direct.phase!=3U)
#endif
                {
                    g_can_motor_rx_ring[g_can_motor_rx_count%32U]=raw;
                    ++g_can_motor_rx_count;
                }
            }
            DM_AppReceive(a, raw.tick_ms, (uint16_t)rx.Identifier, data);
        }
    }
}
int main(void) {
    SCB_EnableICache();
    HAL_Init(); g_init_stage = 1;
    SystemClock_Config(); g_init_stage = 2;
    FDCAN2_Clock_Config(); g_init_stage = 3;
    MX_FDCAN2_Init(); g_init_stage = 4;
    if (!DBUS_Init()) Error_Handler();
    if (!BoardLED_Init()) Error_Handler();
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID; filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK; filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0; filter.FilterID2 = 0; /* all standard IDs, including MST_ID=0 */
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) Error_Handler();
    g_init_stage = 5;
    DM_App app; DM_AppInit(&app, HAL_GetTick());
#if DM_DIRECT_TEST
    DM_DirectTest direct={0};
    uint32_t last_direct_stop_ms=0;
#endif
    const DM_IO io = { .send = Send, .cancel_pending = Cancel };
    g_init_stage = 6;
    while (1) {
        Receive(&app);
        DBUS_State remote = DBUS_Snapshot();
        DBUS_State control_remote=remote;
#if DM_DIRECT_TEST
        DM_DirectTick(&direct,&app,HAL_GetTick(),g_direct_start_request,g_direct_abort_request,&control_remote);
        g_direct_start_request=0;
#endif
        DM_AppTick(&app, HAL_GetTick(), &control_remote, BusOK(), &io);
#if DM_DIRECT_TEST
        /* Also overwrite the last target with zero current/neutral after the one-shot,
         * then request disable. Never leave a nonzero target latched. */
        if(direct.phase==3 && app.motor_id && HAL_GetTick()-last_direct_stop_ms>=20U) {
            uint8_t data[8];float quantized;
            last_direct_stop_ms=HAL_GetTick();
            if(DM_USE_MULTI) {
                for(uint32_t i=0;i<8U;++i)data[i]=0;
                (void)Send(0,0x3FEU,data);
            } else if(DM_MitTorquePayload(0,app.tmax_Nm,data,&quantized))
                (void)Send(0,app.motor_id,data);
            DM_SpecialPayload(0xFD,data);(void)Send(0,app.motor_id,data);
        }
        g_direct=direct;
#endif
        g_dm = app;
        BoardLED_Update(&remote, HAL_GetTick());
        HAL_Delay(DM_TX_PERIOD_MS);
    }
}
void SysTick_Handler(void) { HAL_IncTick(); }
void Error_Handler(void) {
    g_error = 1; __disable_irq(); while (1) { }
}
