#include "fdcan.h"

FDCAN_HandleTypeDef hfdcan1;

static const can_pin_option_t *active_option;

static void fdcan_apply_defaults(FDCAN_HandleTypeDef *fdcan, uint32_t mode, uint32_t prescaler)
{
    fdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    fdcan->Init.Mode = mode;
    fdcan->Init.AutoRetransmission = (mode == FDCAN_MODE_NORMAL) ? DISABLE : ENABLE;
    fdcan->Init.TransmitPause = DISABLE;
    fdcan->Init.ProtocolException = DISABLE;
    fdcan->Init.NominalPrescaler = prescaler;
    fdcan->Init.NominalSyncJumpWidth = 1;
    fdcan->Init.NominalTimeSeg1 = 23;
    fdcan->Init.NominalTimeSeg2 = 8;
    fdcan->Init.DataPrescaler = prescaler;
    fdcan->Init.DataSyncJumpWidth = 10;
    fdcan->Init.DataTimeSeg1 = 23;
    fdcan->Init.DataTimeSeg2 = 8;
    fdcan->Init.MessageRAMOffset = 0;
    fdcan->Init.StdFiltersNbr = 1;
    fdcan->Init.ExtFiltersNbr = 0;
    fdcan->Init.RxFifo0ElmtsNbr = 16;
    fdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    fdcan->Init.RxFifo1ElmtsNbr = 0;
    fdcan->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    fdcan->Init.RxBuffersNbr = 0;
    fdcan->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    fdcan->Init.TxEventsNbr = 0;
    fdcan->Init.TxBuffersNbr = 0;
    fdcan->Init.TxFifoQueueElmtsNbr = 4;
    fdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    fdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_8;
}

bool can_probe_open(const can_pin_option_t *option, uint32_t mode)
{
    can_probe_close();
    if (option == NULL)
    {
        return false;
    }

    active_option = option;
    hfdcan1.Instance = option->instance;
    fdcan_apply_defaults(&hfdcan1, mode, option->prescaler);

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        can_probe_close();
        return false;
    }

    /* 掩码全零 = 接受所有标准帧，便于扫描未知 ID。 */
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK ||
        HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        can_probe_close();
        return false;
    }

    return true;
}

void can_probe_close(void)
{
    if (hfdcan1.Instance != NULL)
    {
        (void)HAL_FDCAN_Stop(&hfdcan1);
        (void)HAL_FDCAN_DeInit(&hfdcan1);
    }
    active_option = NULL;
}

static IRQn_Type can_irq(FDCAN_GlobalTypeDef *instance)
{
    if (instance == FDCAN1)
    {
        return FDCAN1_IT0_IRQn;
    }
    if (instance == FDCAN2)
    {
        return FDCAN2_IT0_IRQn;
    }
    return FDCAN3_IT0_IRQn;
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcan)
{
    GPIO_InitTypeDef gpio = {0};

    if (active_option == NULL || fdcan->Instance != active_option->instance)
    {
        return;
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();

    if (active_option->port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (active_option->port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (active_option->port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (active_option->port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (active_option->port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (active_option->port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (active_option->port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
    else if (active_option->port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();

    gpio.Pin = active_option->tx_pin | active_option->rx_pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = active_option->alternate;
    HAL_GPIO_Init(active_option->port, &gpio);

    HAL_NVIC_SetPriority(can_irq(active_option->instance), 5U, 0U);
    HAL_NVIC_EnableIRQ(can_irq(active_option->instance));
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcan)
{
    if (active_option == NULL || fdcan->Instance != active_option->instance)
    {
        return;
    }

    HAL_GPIO_DeInit(active_option->port, active_option->tx_pin | active_option->rx_pin);
    HAL_NVIC_DisableIRQ(can_irq(active_option->instance));
    __HAL_RCC_FDCAN_CLK_DISABLE();
}
