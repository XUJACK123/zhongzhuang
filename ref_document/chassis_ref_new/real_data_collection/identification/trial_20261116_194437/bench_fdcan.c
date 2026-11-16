/* Isolated diagnostic fork of Core/Src/fdcan.c. Normal firmware is unchanged. */
#include "fdcan.h"
#include "chassis.h"
#include "limited_current.h"
#include "diag_can.h"
#include "can_tx_age.h"
#include <string.h>
extern volatile LimitedCurrent g_ident;
volatile DiagCanMonitor g_diag_can;
static void diag_poll(void);
static CanTxAge tx_age;
FDCAN_HandleTypeDef hfdcan2;
volatile uint32_t g_can_tx_fail, g_can_rx_overflow;
volatile CanMonitor g_can_monitor={.node={{.id=0x304},{.id=0x303},{.id=0x203},
                                        {.id=0x301},{.id=0x302},{.id=0x202}}};
bool can2_init(void) {
    hfdcan2.Instance=FDCAN2;
    hfdcan2.Init.FrameFormat=FDCAN_FRAME_CLASSIC;
    hfdcan2.Init.Mode=FDCAN_MODE_NORMAL;
    /* Retain arbitration losers; explicitly expire pending nonzero requests. */
    hfdcan2.Init.AutoRetransmission=ENABLE;
    hfdcan2.Init.TransmitPause=DISABLE;
    hfdcan2.Init.ProtocolException=DISABLE;
    hfdcan2.Init.NominalPrescaler=3; /* 96MHz / 3 / (1+23+8) = 1Mbps */
    hfdcan2.Init.NominalSyncJumpWidth=1;
    hfdcan2.Init.NominalTimeSeg1=23; hfdcan2.Init.NominalTimeSeg2=8;
    hfdcan2.Init.DataPrescaler=3; hfdcan2.Init.DataSyncJumpWidth=1;
    hfdcan2.Init.DataTimeSeg1=23; hfdcan2.Init.DataTimeSeg2=8;
    hfdcan2.Init.MessageRAMOffset=0; /* only CAN2 is initialized */
    hfdcan2.Init.StdFiltersNbr=1; hfdcan2.Init.ExtFiltersNbr=0;
    hfdcan2.Init.RxFifo0ElmtsNbr=24; hfdcan2.Init.RxFifo0ElmtSize=FDCAN_DATA_BYTES_8;
    hfdcan2.Init.RxFifo1ElmtsNbr=0; hfdcan2.Init.RxFifo1ElmtSize=FDCAN_DATA_BYTES_8;
    hfdcan2.Init.RxBuffersNbr=0; hfdcan2.Init.RxBufferSize=FDCAN_DATA_BYTES_8;
    hfdcan2.Init.TxEventsNbr=16; hfdcan2.Init.TxBuffersNbr=0;
    hfdcan2.Init.TxFifoQueueElmtsNbr=8; hfdcan2.Init.TxFifoQueueMode=FDCAN_TX_FIFO_OPERATION;
    hfdcan2.Init.TxElmtSize=FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(&hfdcan2)!=HAL_OK) return false;
    if(HAL_FDCAN_ConfigTimestampCounter(&hfdcan2,FDCAN_TIMESTAMP_PRESC_16)!=HAL_OK ||
       HAL_FDCAN_EnableTimestampCounter(&hfdcan2,FDCAN_TIMESTAMP_INTERNAL)!=HAL_OK)return false;
    FDCAN_FilterTypeDef f={0};
    f.IdType=FDCAN_STANDARD_ID; f.FilterType=FDCAN_FILTER_RANGE;
    f.FilterConfig=FDCAN_FILTER_TO_RXFIFO0; f.FilterID1=0x200; f.FilterID2=0x3fe;
    return HAL_FDCAN_ConfigFilter(&hfdcan2,&f)==HAL_OK &&
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE)==HAL_OK &&
        HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO0_MESSAGE_LOST,0)==HAL_OK &&
        HAL_FDCAN_Start(&hfdcan2)==HAL_OK;
}
bool can2_send(uint16_t id, const uint8_t d[8]) {
    FDCAN_TxHeaderTypeDef h={0};
    h.Identifier=id; h.IdType=FDCAN_STANDARD_ID; h.TxFrameType=FDCAN_DATA_FRAME;
    h.DataLength=FDCAN_DLC_BYTES_8; h.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    h.BitRateSwitch=FDCAN_BRS_OFF; h.FDFormat=FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl=(id==0x3fe || (id>=1 && id<=4)) ? FDCAN_STORE_TX_EVENTS : FDCAN_NO_TX_EVENTS;
    h.MessageMarker=(id>=1 && id<=4) ? d[7] : 0;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&h,d)!=HAL_OK) { ++g_can_tx_fail; return false; }
    bool nonzero=false;
    if(id==0x3fe || id==0x200)for(unsigned i=0;i<8;++i)nonzero=nonzero || d[i]!=0;
    can_tx_age_record(&tx_age,HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan2),HAL_GetTick(),nonzero);
    ++g_can_monitor.tx_queued;
    if (id==0x3fe || id==0x200) {
        for (unsigned i=0;i<8;++i) if (d[i]) { ++g_can_monitor.nonzero_torque_frames; break; }
    } else if (id>=1 && id<=4 && d[0]==0xff && d[7]==0xfc) ++g_can_monitor.enable_commands;
    return true;
}
void can2_cancel_pending(void) { (void)HAL_FDCAN_AbortTxRequest(&hfdcan2,0xffffffffU);can_tx_age_clear(&tx_age); }
bool can2_healthy(void) {
    uint32_t expired=can_tx_age_expired(&tx_age,hfdcan2.Instance->TXBRP,HAL_GetTick());
    if(expired) {
        (void)HAL_FDCAN_AbortTxRequest(&hfdcan2,expired);
        for(unsigned i=0;i<32;++i)if(expired&(1U<<i))++g_diag_can.expired_torque_abort_requests;
        tx_age.nonzero_mask&=~expired;
    }
    diag_poll();
    FDCAN_ProtocolStatusTypeDef s={0}; FDCAN_ErrorCountersTypeDef c={0};
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan2,&s)!=HAL_OK ||
        HAL_FDCAN_GetErrorCounters(&hfdcan2,&c)!=HAL_OK) return false;
    g_can_monitor.tx_error=c.TxErrorCnt; g_can_monitor.rx_error=c.RxErrorCnt;
    g_can_monitor.bus_off=s.BusOff; g_can_monitor.last_error=s.LastErrorCode;
    ++g_can_monitor.status_samples;
    return !s.BusOff && !s.ErrorPassive && c.TxErrorCnt<32 && c.RxErrorCnt<32;
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h, uint32_t flags) {
    if (flags&FDCAN_IT_RX_FIFO0_MESSAGE_LOST) ++g_can_rx_overflow;
    if (h->Instance!=FDCAN2) return;
    while (HAL_FDCAN_GetRxFifoFillLevel(h,FDCAN_RX_FIFO0)>0) {
        FDCAN_RxHeaderTypeDef rx; uint8_t data[64];
        if (HAL_FDCAN_GetRxMessage(h,FDCAN_RX_FIFO0,&rx,data)!=HAL_OK) break;
        if (rx.IdType==FDCAN_STANDARD_ID && rx.RxFrameType==FDCAN_DATA_FRAME &&
            rx.FDFormat==FDCAN_CLASSIC_CAN && rx.DataLength==FDCAN_DLC_BYTES_8) {
            ++g_can_monitor.received;
            bool found=false;
            for (unsigned i=0;i<6;++i) if (rx.Identifier==g_can_monitor.node[i].id) {
                ++g_can_monitor.node[i].received; g_can_monitor.node[i].last_ms=HAL_GetTick();
                for (unsigned j=0;j<8;++j) g_can_monitor.node[i].data[j]=data[j];
                found=true; break;
            }
            if (!found) ++g_can_monitor.other;
            if(rx.Identifier==0x3fe) {++g_diag_can.foreign_group_rx;for(unsigned j=0;j<8;++j)g_diag_can.last_foreign_group[j]=data[j];}
            if(rx.Identifier==0x200) {++g_diag_can.foreign_dji_rx;for(unsigned j=0;j<8;++j)g_diag_can.last_foreign_dji[j]=data[j];}
            chassis_receive((uint16_t)rx.Identifier,data,HAL_GetTick());
        }
    }
}
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *h) {
    if (h->Instance!=FDCAN2) return;
    __HAL_RCC_FDCAN_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g={0}; g.Pin=GPIO_PIN_5|GPIO_PIN_6; g.Mode=GPIO_MODE_AF_PP;
    g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB,&g);
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn,3,0); HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
}

/* Hardware Tx events capture the frame start timestamp on successful transmission.
 * 1Mbps nominal bit time * timestamp prescaler16 = 16us; this is not a motor acknowledgement. */
static void diag_poll(void) {
    static uint32_t previous_phase=IDLE;
    static uint16_t last_stamp;
    static bool have_stamp;
    static uint16_t all_last_stamp;
    static bool all_have_stamp;
    uint32_t phase=g_ident.phase;
    if(phase==ARMED && previous_phase!=ARMED) {
        g_diag_can.active_groups_tx=0;g_diag_can.active_fc_tx=0;g_diag_can.active_gap_max_us=0;have_stamp=false;
    }
    bool active=phase==ENABLING || phase==RUNNING || phase==COOLDOWN;
    if(!active)have_stamp=false;
    if(__HAL_FDCAN_GET_FLAG(&hfdcan2,FDCAN_FLAG_TX_EVT_FIFO_ELT_LOST)) {
        ++g_diag_can.event_loss_flags_seen;
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan2,FDCAN_FLAG_TX_EVT_FIFO_ELT_LOST);
    }
    while((hfdcan2.Instance->TXEFS & FDCAN_TXEFS_EFFL)!=0) {
        FDCAN_TxEventFifoTypeDef event;
        if(HAL_FDCAN_GetTxEvent(&hfdcan2,&event)!=HAL_OK)break;
        if(event.Identifier==0x3fe) {
            ++g_diag_can.groups_tx;
            uint16_t all_stamp=(uint16_t)event.TxTimestamp;
            if(all_have_stamp && HAL_GetTick()>2000U) {
                uint32_t all_gap=(uint32_t)(uint16_t)(all_stamp-all_last_stamp)*16U;
                if(all_gap>g_diag_can.all_gap_max_us)g_diag_can.all_gap_max_us=all_gap;
            }
            all_last_stamp=all_stamp;all_have_stamp=true;
            if(active) {
                ++g_diag_can.active_groups_tx;
                uint16_t stamp=(uint16_t)event.TxTimestamp;
                if(have_stamp) {uint32_t gap=(uint32_t)(uint16_t)(stamp-last_stamp)*16U;if(gap>g_diag_can.active_gap_max_us)g_diag_can.active_gap_max_us=gap;}
                last_stamp=stamp;have_stamp=true;
            }
        } else if(event.MessageMarker==0xfc) {++g_diag_can.fc_tx;if(active)++g_diag_can.active_fc_tx;}
        else if(event.MessageMarker==0xfd)++g_diag_can.fd_tx;
    }
    previous_phase=phase;
}
