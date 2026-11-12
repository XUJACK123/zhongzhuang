#include "main.h"
#include "fdcan.h"

/*
 * C620/M3508 自动枚举固件。
 *
 * 对每个候选引脚组合：先用回环模式验证 FDCAN 内核与时钟可用（回环由芯片内部完成，
 * 不能证明外部收发器），再进入正常模式发送全零电流（电调不动），监听 0x201..0x208。
 * 只有正常模式下收到对端 ACK 或反馈帧，才说明总线上真的电调通信。
 * 结果全部写入 RAM，可用 Keil Watch 或 OpenOCD/GDB 读取。
 */

#define C620_GROUP_ID_LOW 0x200U
#define C620_GROUP_ID_HIGH 0x1FFU
#define C620_FEEDBACK_MIN 0x201U
#define C620_FEEDBACK_MAX 0x208U

#define LOOPBACK_ID 0x123U
#define LOOPBACK_MS 120U
#define NORMAL_MS 250U
#define TX_PERIOD_MS 10U
#define ROUND_PAUSE_MS 500U

/* 识别到电调后：每 CONFIRM_PERIOD_MS 发一小段电流让电机轻抖，便于无调试器时确认。 */
#define CONFIRM_PULSE_ENABLE 1
#define CONFIRM_CURRENT 300
#define CONFIRM_WIDTH_MS 150U
#define CONFIRM_PERIOD_MS 2000U

/*
 * 逐槽位扫描模式：按 0x200 槽0..3、0x1FF 槽0..3 顺序各通电 SWEEP_ON_MS，
 * 用肉眼观察电机转动即可反推电调 ID。用外部回环模式发送，避免无 ACK 时
 * 反复 bus-off 导致发不出帧。
 */
#define SWEEP_MODE 0
#define SWEEP_BUS_INDEX 3U     /* probe_options[3] = CAN2 PB6/PB5 @1Mbps */
#define SWEEP_CURRENT 1000
#define SWEEP_ON_MS 1500U
#define SWEEP_OFF_MS 1200U

/* 板载晶振频率。若板子实际是 25 MHz（H723 核心板常见），CAN 会因 4% 误差完全通信不了。 */
#define HSE_MHZ 24U

#if HSE_MHZ == 24
/* 24 MHz / 3 = 8 MHz，VCO 8 x 68.75 = 550 MHz，FDCAN 12 x 16 / 2 = 96 MHz */
#define PLL1_M 3U
#define PLL1_N 68U
#define PLL1_FRACN 6144U
#define PLL1_VCI RCC_PLL1VCIRANGE_3
#define PLL2_M 2U
#define PLL2_N 16U
#define PLL2_VCI RCC_PLL2VCIRANGE_3
#elif HSE_MHZ == 25
/* 25 MHz / 25 = 1 MHz，VCO 1 x 550 = 550 MHz，FDCAN 1 x 192 / 2 = 96 MHz */
#define PLL1_M 25U
#define PLL1_N 550U
#define PLL1_FRACN 0U
#define PLL1_VCI RCC_PLL1VCIRANGE_0
#define PLL2_M 25U
#define PLL2_N 192U
#define PLL2_VCI RCC_PLL2VCIRANGE_0
#else
#error "HSE_MHZ must be 24 or 25"
#endif

/* FDCAN3 在 PD12/PD13 上也是 AF9，HAL 未提供对应宏。 */
#define FDCAN3_PIN_AF 0x09U

/* 板级电源使能：原 CubeMX 工程上电即拉高，去掉后 CAN 收发器可能没有供电。 */
#define POWER_OUT1_EN_PIN GPIO_PIN_14 /* PC14 */
#define POWER_5V_PIN GPIO_PIN_15      /* PC15 */
#define JOINT_SWITCH_PIN GPIO_PIN_0   /* PA0 */

typedef struct
{
    volatile uint32_t loop_echo; /* 外部回环收到的帧数，>0 表示收发器通路正常 */
    volatile uint32_t tx_ok;
    volatile uint32_t tx_fail;
    volatile uint32_t rx_count;  /* 正常模式下收到的帧数 */
    volatile uint32_t bus_off;
    volatile uint32_t tec;
    volatile uint32_t rec;
    volatile uint32_t esc_id;    /* 0 = 未识别 */
    volatile uint32_t esc_mask;
    volatile uint32_t last_id;
    volatile uint32_t command_id;
    volatile uint32_t open_fail;
    volatile uint32_t rx_idle;   /* RX 引脚加下拉后的电平：1 = 有器件把它拉高（多半有收发器） */
    volatile uint32_t echo_high; /* TX=1 时 RX 读到的电平 */
    volatile uint32_t echo_low;  /* TX=0 时 RX 读到的电平：0 = RX 跟随 TX，说明收发器在工作 */
} probe_result_t;

typedef struct
{
    volatile uint32_t found; /* 0=无反应 1=仅回环通过 2=已发现电调 */
    volatile uint32_t probe_index;
    volatile uint32_t instance; /* 1/2/3 */
    volatile uint32_t port;     /* ASCII 'A'..'H' */
    volatile uint32_t tx_pin;
    volatile uint32_t rx_pin;
    volatile uint32_t esc_id;
    volatile uint32_t command_id;
    volatile uint32_t slot;
    volatile uint32_t rate_kbps; /* 该次命中的比特率 */
    volatile uint32_t rounds;
} scan_summary_t;

static const can_pin_option_t probe_options[] = {
    {FDCAN1, GPIOD, GPIO_PIN_1, GPIO_PIN_0, GPIO_AF9_FDCAN1, 3U},
    {FDCAN1, GPIOD, GPIO_PIN_1, GPIO_PIN_0, GPIO_AF9_FDCAN1, 6U},
    {FDCAN1, GPIOD, GPIO_PIN_1, GPIO_PIN_0, GPIO_AF9_FDCAN1, 12U},
    {FDCAN2, GPIOB, GPIO_PIN_6, GPIO_PIN_5, GPIO_AF9_FDCAN2, 3U},
    {FDCAN2, GPIOB, GPIO_PIN_6, GPIO_PIN_5, GPIO_AF9_FDCAN2, 6U},
    {FDCAN2, GPIOB, GPIO_PIN_6, GPIO_PIN_5, GPIO_AF9_FDCAN2, 12U},
    {FDCAN3, GPIOD, GPIO_PIN_13, GPIO_PIN_12, FDCAN3_PIN_AF, 3U},
    {FDCAN3, GPIOD, GPIO_PIN_13, GPIO_PIN_12, FDCAN3_PIN_AF, 6U},
    {FDCAN3, GPIOD, GPIO_PIN_13, GPIO_PIN_12, FDCAN3_PIN_AF, 12U},
};

#define PROBE_COUNT (sizeof(probe_options) / sizeof(probe_options[0]))

volatile probe_result_t g_probe[PROBE_COUNT];
volatile scan_summary_t g_scan;
volatile uint32_t g_probe_index;
volatile uint32_t g_loopback_phase;
volatile uint32_t g_sweep_step;  /* 当前扫描的槽位 0..7 */
volatile uint32_t g_sweep_cycle; /* 已完成的扫描轮数 */

static void SystemClock_Config(void);
static void PeriphCommonClock_Config(void);
static void PowerRails_Init(void);
static void run_probe(uint32_t index);
static void confirm_service(void);
static void sweep_service(void);
static void send_frame(uint32_t id, const uint8_t data[8]);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    PeriphCommonClock_Config();
    PowerRails_Init();
    HAL_Delay(100U);

#if SWEEP_MODE
    sweep_service();
#endif

    while (1)
    {
        if (g_scan.found == 2U)
        {
            confirm_service();
        }

        for (uint32_t i = 0U; i < PROBE_COUNT; ++i)
        {
            run_probe(i);
        }
        ++g_scan.rounds;
        HAL_Delay(ROUND_PAUSE_MS);
    }
}

/* 逐槽位通电，用肉眼确认电调 ID：每 1.5 s 换一个槽位，停 1.2 s 再下一个。 */
static void sweep_service(void)
{
    const can_pin_option_t *option = &probe_options[SWEEP_BUS_INDEX];
    uint8_t data[8] = {0};
    uint32_t step = 0U;

    while (!can_probe_open(option, FDCAN_MODE_EXTERNAL_LOOPBACK))
    {
        HAL_Delay(ROUND_PAUSE_MS);
    }

    g_probe_index = 0U;
    g_probe[0].esc_id = 0U;
    g_probe[0].esc_mask = 0U;

    while (1)
    {
        const uint32_t group = (step < 4U) ? C620_GROUP_ID_LOW : C620_GROUP_ID_HIGH;
        const uint32_t slot = step % 4U;

        g_sweep_step = step;
        g_sweep_cycle = step / 8U;

        for (uint32_t elapsed = 0U; elapsed < (SWEEP_ON_MS + SWEEP_OFF_MS); elapsed += TX_PERIOD_MS)
        {
            const uint32_t offset = 2U * slot;
            data[offset] = 0U;
            data[offset + 1U] = 0U;

            if (elapsed < SWEEP_ON_MS)
            {
                data[offset] = (uint8_t)((uint16_t)SWEEP_CURRENT >> 8);
                data[offset + 1U] = (uint8_t)SWEEP_CURRENT;
            }

            send_frame(group, data);
            HAL_Delay(TX_PERIOD_MS);
        }

        step = (step + 1U) % 8U;
    }
}

/* 已识别电调：常驻该总线，持续发零电流，并按周期给一小段电流便于肉眼确认。 */
static void confirm_service(void)
{
    const can_pin_option_t *option = &probe_options[g_scan.probe_index];
    uint8_t data[8] = {0};
    uint32_t last_tx = HAL_GetTick() - TX_PERIOD_MS;

    if (g_scan.command_id == 0U || !can_probe_open(option, FDCAN_MODE_NORMAL))
    {
        HAL_Delay(ROUND_PAUSE_MS);
        return;
    }

    g_probe_index = g_scan.probe_index;
    while (1)
    {
        const uint32_t now = HAL_GetTick();
        if ((now - last_tx) < TX_PERIOD_MS)
        {
            continue;
        }
        last_tx = now;

        const uint32_t phase = now % CONFIRM_PERIOD_MS;
        int16_t current = 0;
#if CONFIRM_PULSE_ENABLE
        if (phase < CONFIRM_WIDTH_MS)
        {
            current = CONFIRM_CURRENT;
        }
#endif
        data[2U * g_scan.slot] = (uint8_t)((uint16_t)current >> 8);
        data[2U * g_scan.slot + 1U] = (uint8_t)current;
        send_frame(g_scan.command_id, data);
    }
}

static void PowerRails_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, POWER_OUT1_EN_PIN | POWER_5V_PIN, GPIO_PIN_SET);
    gpio.Pin = POWER_OUT1_EN_PIN | POWER_5V_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    HAL_GPIO_WritePin(GPIOA, JOINT_SWITCH_PIN, GPIO_PIN_SET);
    gpio.Pin = JOINT_SWITCH_PIN;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void send_frame(uint32_t id, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) != HAL_OK)
    {
        ++g_probe[g_probe_index].tx_fail;
        return;
    }
    ++g_probe[g_probe_index].tx_ok;
}

static void read_counters(uint32_t index)
{
    FDCAN_ProtocolStatusTypeDef status;
    FDCAN_ErrorCountersTypeDef counters;
    static uint32_t bus_off_seen;

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) == HAL_OK)
    {
        if (status.BusOff != 0U && bus_off_seen == 0U)
        {
            ++g_probe[index].bus_off;
        }
        bus_off_seen = status.BusOff;
    }
    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &counters) == HAL_OK)
    {
        g_probe[index].tec = counters.TxErrorCnt;
        g_probe[index].rec = counters.RxErrorCnt;
    }
}

static void run_phase(uint32_t duration_ms, uint32_t loopback)
{
    uint8_t data[8] = {0};
    const uint32_t start = HAL_GetTick();
    uint32_t last_tx = start - TX_PERIOD_MS;

    g_loopback_phase = loopback;
    while ((HAL_GetTick() - start) < duration_ms)
    {
        const uint32_t now = HAL_GetTick();
        if ((now - last_tx) >= TX_PERIOD_MS)
        {
            last_tx = now;
            if (loopback != 0U)
            {
                send_frame(LOOPBACK_ID, data);
            }
            else
            {
                send_frame(C620_GROUP_ID_LOW, data);
                send_frame(C620_GROUP_ID_HIGH, data);
            }
        }
        read_counters(g_probe_index);
    }
    g_loopback_phase = 0U;
}

static void run_probe(uint32_t index)
{
    const can_pin_option_t *option = &probe_options[index];

    g_probe_index = index;
    g_probe[index].loop_echo = 0U;
    g_probe[index].tx_ok = 0U;
    g_probe[index].tx_fail = 0U;
    g_probe[index].rx_count = 0U;
    g_probe[index].bus_off = 0U;
    g_probe[index].tec = 0U;
    g_probe[index].rec = 0U;
    g_probe[index].esc_id = 0U;
    g_probe[index].esc_mask = 0U;
    g_probe[index].last_id = 0U;
    g_probe[index].command_id = 0U;
    g_probe[index].open_fail = 0U;
    g_probe[index].rx_idle = 0U;
    g_probe[index].echo_high = 0U;
    g_probe[index].echo_low = 0U;

    /* RX 引脚下拉后仍为高：该引脚被外部器件（收发器的 R 输出）拉高。 */
    GPIO_InitTypeDef gpio = {0};
    if (option->port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (option->port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (option->port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (option->port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (option->port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (option->port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (option->port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
    else if (option->port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();

    gpio.Pin = option->rx_pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(option->port, &gpio);

    /* TX 用开漏+上拉驱动：只会拉低，不会对外部强驱动产生电压冲突。 */
    gpio.Pin = option->tx_pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(option->port, &gpio);

    HAL_GPIO_WritePin(option->port, option->tx_pin, GPIO_PIN_SET);
    for (volatile uint32_t spin = 0U; spin < 20000U; ++spin)
    {
    }
    g_probe[index].rx_idle = (HAL_GPIO_ReadPin(option->port, option->rx_pin) == GPIO_PIN_SET) ? 1U : 0U;
    g_probe[index].echo_high = g_probe[index].rx_idle;

    HAL_GPIO_WritePin(option->port, option->tx_pin, GPIO_PIN_RESET);
    for (volatile uint32_t spin = 0U; spin < 20000U; ++spin)
    {
    }
    g_probe[index].echo_low = (HAL_GPIO_ReadPin(option->port, option->rx_pin) == GPIO_PIN_SET) ? 1U : 0U;

    HAL_GPIO_WritePin(option->port, option->tx_pin, GPIO_PIN_SET);
    HAL_GPIO_DeInit(option->port, option->tx_pin);
    HAL_GPIO_DeInit(option->port, option->rx_pin);

    if (!can_probe_open(option, FDCAN_MODE_EXTERNAL_LOOPBACK))
    {
        g_probe[index].open_fail = 1U;
        return;
    }
    run_phase(LOOPBACK_MS, 1U);
    can_probe_close();

    if (!can_probe_open(option, FDCAN_MODE_NORMAL))
    {
        g_probe[index].open_fail = 2U;
        return;
    }
    run_phase(NORMAL_MS, 0U);
    can_probe_close();

    if (g_probe[index].esc_id != 0U)
    {
        g_scan.found = 2U;
    }
    else if (g_probe[index].rx_count != 0U && g_scan.found == 0U)
    {
        g_scan.found = 1U;
    }

    if (g_scan.found == 2U && g_probe[index].esc_id == 0U)
    {
        return;
    }

    g_scan.probe_index = index;
    g_scan.instance = (option->instance == FDCAN1) ? 1U : (option->instance == FDCAN2 ? 2U : 3U);
    g_scan.port = (option->port == GPIOA) ? 'A' : (option->port == GPIOB) ? 'B'
                 : (option->port == GPIOC) ? 'C' : (option->port == GPIOD) ? 'D'
                 : (option->port == GPIOE) ? 'E' : (option->port == GPIOF) ? 'F'
                 : (option->port == GPIOG) ? 'G' : 'H';
    g_scan.tx_pin = option->tx_pin;
    g_scan.rx_pin = option->rx_pin;
    g_scan.esc_id = g_probe[index].esc_id;
    g_scan.command_id = g_probe[index].command_id;
    g_scan.slot = (g_probe[index].esc_id == 0U) ? 0U : (g_probe[index].esc_id - 1U) % 4U;
    g_scan.rate_kbps = 96000U / (option->prescaler * 32U);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
    const uint32_t index = g_probe_index;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U || index >= PROBE_COUNT)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }

        if (g_loopback_phase != 0U)
        {
            ++g_probe[index].loop_echo;
            continue;
        }

        ++g_probe[index].rx_count;
        g_probe[index].last_id = header.Identifier;

        if (header.Identifier >= C620_FEEDBACK_MIN && header.Identifier <= C620_FEEDBACK_MAX)
        {
            const uint32_t number = header.Identifier - 0x200U;
            g_probe[index].esc_mask |= 1UL << (number - 1U);
            g_probe[index].esc_id = number;
            g_probe[index].command_id = (number <= 4U) ? C620_GROUP_ID_LOW : C620_GROUP_ID_HIGH;
        }
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = PLL1_M;
    oscillator.PLL.PLLN = PLL1_N;
    oscillator.PLL.PLLP = 1;
    oscillator.PLL.PLLQ = 2;
    oscillator.PLL.PLLR = 2;
    oscillator.PLL.PLLRGE = PLL1_VCI;
    oscillator.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    oscillator.PLL.PLLFRACN = PLL1_FRACN;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
    {
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                       RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clocks.AHBCLKDivider = RCC_HCLK_DIV2;
    clocks.APB3CLKDivider = RCC_APB3_DIV2;
    clocks.APB1CLKDivider = RCC_APB1_DIV2;
    clocks.APB2CLKDivider = RCC_APB2_DIV2;
    clocks.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

static void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef peripheral_clock = {0};

    peripheral_clock.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    peripheral_clock.PLL2.PLL2M = PLL2_M;
    peripheral_clock.PLL2.PLL2N = PLL2_N;
    peripheral_clock.PLL2.PLL2P = 2;
    peripheral_clock.PLL2.PLL2Q = 2;
    peripheral_clock.PLL2.PLL2R = 2;
    peripheral_clock.PLL2.PLL2RGE = PLL2_VCI;
    peripheral_clock.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    peripheral_clock.PLL2.PLL2FRACN = 0;
    peripheral_clock.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&peripheral_clock) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
