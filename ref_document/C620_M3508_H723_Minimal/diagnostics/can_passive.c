/** CAN2 receive-only diagnostic. No motor command frames are queued. */
#include "main.h"
#include <string.h>

typedef struct {
    uint32_t tick_ms, id, id_type, dlc, frame_type;
    uint8_t data[8];
} CanRawFrame;

typedef struct {
    uint32_t signature, stage, loop_count, rx_count, rx_fail;
    uint32_t rx_pin_high, rx_pin_low;
    uint32_t c620_seen_mask, c620_count[8];
    uint32_t last_id, last_id_type, last_dlc;
    uint32_t psr, ecr, ir, cccr, nbtp, rxf0s;
    uint32_t last_lec, bus_off, clock_hz, last_rx_ms;
    uint32_t seen_std_ids[64];
    uint32_t ring_next;
    CanRawFrame ring[32];
} CanDiagnostic;

volatile CanDiagnostic g_can_diag = {.signature = 0x723C6201U};
volatile uint32_t g_error = 0;

volatile uint32_t g_can_loopback_result = 0;
volatile uint32_t g_can_loopback_rx_id = 0;

static HAL_StatusTypeDef Start_All_Id_Reception(void) {
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_FDCAN_Start(&hfdcan2);
}

/* Internal loopback has MON+TEST+LBCK set. No test frame appears on CANH/CANL. */
static void Internal_Loopback_Test(void) {
    hfdcan2.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) { g_can_loopback_result = 0xE001; return; }
    if (Start_All_Id_Reception() != HAL_OK) { g_can_loopback_result = 0xE002; return; }
    if ((hfdcan2.Instance->CCCR & (FDCAN_CCCR_MON | FDCAN_CCCR_TEST)) !=
        (FDCAN_CCCR_MON | FDCAN_CCCR_TEST) ||
        (hfdcan2.Instance->TEST & FDCAN_TEST_LBCK) == 0U) {
        g_can_loopback_result = 0xE008;
        return;
    }
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier = 0x123;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    uint8_t sent[8] = {0x12,0x34,0x56,0x78,0x9A,0xBC,0xDE,0xF0};
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx, sent) != HAL_OK) {
        g_can_loopback_result = 0xE003; return;
    }
    uint32_t start = HAL_GetTick();
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) == 0U) {
        if (HAL_GetTick() - start > 20U) { g_can_loopback_result = 0xE004; return; }
    }
    FDCAN_RxHeaderTypeDef rx = {0};
    uint8_t received[64] = {0};
    if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx, received) != HAL_OK ||
        rx.Identifier != 0x123 || rx.DataLength != FDCAN_DLC_BYTES_8 ||
        memcmp(sent, received, 8) != 0) {
        g_can_loopback_result = 0xE005; return;
    }
    g_can_loopback_rx_id = rx.Identifier;
    g_can_loopback_result = 1;
}

int main(void) {
    g_can_diag.stage = 1;
    SCB_EnableICache();
    HAL_Init();
    g_can_diag.stage = 2;
    SystemClock_Config();
    g_can_diag.stage = 3;
    FDCAN2_Clock_Config();
    g_can_diag.clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    g_can_diag.stage = 4;
    MX_FDCAN2_Init();

    g_can_diag.stage = 40;
    Internal_Loopback_Test();
    (void)HAL_FDCAN_Stop(&hfdcan2);
    hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK || Start_All_Id_Reception() != HAL_OK) {
        Error_Handler();
    }
    g_can_diag.stage = 5;

    for (;;) {
        g_can_diag.loop_count++;
        if ((GPIOB->IDR & GPIO_PIN_5) != 0U) g_can_diag.rx_pin_high++;
        else g_can_diag.rx_pin_low++;

        if ((g_can_diag.loop_count & 0x3FFU) == 0U) {
            uint32_t psr = hfdcan2.Instance->PSR;
            g_can_diag.psr = psr;
            g_can_diag.ecr = hfdcan2.Instance->ECR;
            g_can_diag.ir = hfdcan2.Instance->IR;
            g_can_diag.cccr = hfdcan2.Instance->CCCR;
            g_can_diag.nbtp = hfdcan2.Instance->NBTP;
            g_can_diag.rxf0s = hfdcan2.Instance->RXF0S;
            g_can_diag.bus_off = (psr >> 7) & 1U;
            uint32_t lec = psr & 7U;
            if (lec != 7U && lec != 0U) g_can_diag.last_lec = lec;
        }
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) == 0U) continue;
        FDCAN_RxHeaderTypeDef rx = {0};
        uint8_t data[64] = {0};
        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
            g_can_diag.rx_fail++;
            continue;
        }
        uint32_t now = HAL_GetTick();
        g_can_diag.rx_count++;
        g_can_diag.last_id = rx.Identifier;
        g_can_diag.last_id_type = rx.IdType;
        g_can_diag.last_dlc = rx.DataLength;
        g_can_diag.last_rx_ms = now;
        if (rx.IdType == FDCAN_STANDARD_ID && rx.Identifier < 2048U) {
            g_can_diag.seen_std_ids[rx.Identifier / 32U] |= 1U << (rx.Identifier % 32U);
            if (rx.Identifier >= 0x201U && rx.Identifier <= 0x208U &&
                rx.RxFrameType == FDCAN_DATA_FRAME && rx.DataLength == FDCAN_DLC_BYTES_8) {
                uint32_t i = rx.Identifier - 0x201U;
                g_can_diag.c620_count[i]++;
                g_can_diag.c620_seen_mask |= 1U << i;
            }
        }
        uint32_t slot = g_can_diag.ring_next;
        g_can_diag.ring[slot].tick_ms = now;
        g_can_diag.ring[slot].id = rx.Identifier;
        g_can_diag.ring[slot].id_type = rx.IdType;
        g_can_diag.ring[slot].dlc = rx.DataLength;
        g_can_diag.ring[slot].frame_type = rx.RxFrameType;
        for (uint32_t i = 0; i < 8U; ++i) g_can_diag.ring[slot].data[i] = data[i];
        g_can_diag.ring_next = (slot + 1U) % 32U;
    }
}

void SysTick_Handler(void) { HAL_IncTick(); }
void Error_Handler(void) {
    g_error = 1;
    __disable_irq();
    while (1) { }
}
