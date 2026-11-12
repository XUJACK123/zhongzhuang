#include "can_monitor.h"
#include "can.h"
#include "main.h"

volatile CanObservedBus can_observed[2];

static void init_bus(CAN_HandleTypeDef *can, uint32_t bank)
{
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    /* ID=0 and mask=0 accept all standard/extended, data/remote frames. */
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(can, &filter) != HAL_OK || HAL_CAN_Start(can) != HAL_OK)
        Error_Handler();
}

void CanMonitorInit(void)
{
    init_bus(&hcan1, 0);
    init_bus(&hcan2, 14);
}

static void store_frame(volatile CanObservedFrame *out, const CAN_RxHeaderTypeDef *hdr,
                        const uint8_t *data, uint32_t count)
{
    out->tick_ms = HAL_GetTick();
    out->id = hdr->IDE == CAN_ID_EXT ? hdr->ExtId : hdr->StdId;
    out->extended = hdr->IDE == CAN_ID_EXT;
    out->remote = hdr->RTR == CAN_RTR_REMOTE;
    out->dlc = hdr->DLC;
    for (uint32_t i = 0; i < 8; ++i) out->data[i] = data[i];
    out->count = count;
}

static void poll_bus(CAN_HandleTypeDef *can, volatile CanObservedBus *out)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    /* Bound each pass so a busy bus cannot starve the other controller. */
    for (unsigned n = 0; n < 8 && HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0); ++n)
    {
        for (unsigned i = 0; i < 8; ++i) data[i] = 0;
        if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
        uint32_t count = ++out->total;
        store_frame(&out->latest, &header, data, count);
        if (header.IDE == CAN_ID_STD)
        {
            uint32_t id = header.StdId;
            out->standard_id_seen[id / 32U] |= 1UL << (id % 32U);
            if (header.RTR == CAN_RTR_DATA && header.DLC == 8 && id >= 0x201 && id <= 0x208)
            {
                uint32_t index = id - 0x201;
                uint32_t c = out->c620[index].count + 1;
                store_frame(&out->c620[index], &header, data, c);
                out->c620_seen_mask |= 1UL << index;
            }
        }
    }
    if (__HAL_CAN_GET_FLAG(can, CAN_FLAG_FOV0))
    {
        ++out->fifo_overruns;
        __HAL_CAN_CLEAR_FLAG(can, CAN_FLAG_FOV0);
    }
    out->last_error = can->Instance->ESR;
}

void CanMonitorPoll(void)
{
    poll_bus(&hcan1, &can_observed[0]);
    poll_bus(&hcan2, &can_observed[1]);
}
