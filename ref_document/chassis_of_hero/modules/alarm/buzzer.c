#include "bsp_pwm.h"
#include "buzzer.h"
#include "bsp_dwt.h"
#include "string.h"

static PWMInstance *buzzer_pwm;
static BuzzerTask_t task_queue[BUZZER_TASK_NUM];
static uint8_t head, tail, count;
static uint32_t task_timeout;
static uint8_t key_last = 1;

/**
 * @brief 蜂鸣器初始化
 *
 */
void BuzzerInit()
{
    PWM_Init_Config_s cfg = {
        .htim = &htim4,
        .channel = TIM_CHANNEL_3,
        .dutyratio = 0,
        .period = 0.001,
    };
    buzzer_pwm = PWMRegister(&cfg);
}

static void PushTask(uint32_t freq, uint8_t volume, uint16_t duration)
{
    if (count >= BUZZER_TASK_NUM)
        return;
    task_queue[tail].freq = freq;
    task_queue[tail].volume = volume;
    task_queue[tail].duration = duration;
    tail = (tail + 1) % BUZZER_TASK_NUM;
    count++;
}

void BuzzerBeepOnce(void)
{
    if (count > BUZZER_TASK_NUM - 2) return;
    PushTask(6000, 50, 100);
    PushTask(6000, 0, 100);
}

void BuzzerBeepTriple(void)                               
{                                                         
    PushTask(6000, 50, 100);                              
    PushTask(6000, 0,  100);                              
    PushTask(6000, 50, 100);                              
    PushTask(6000, 0,  100);                              
    PushTask(6000, 50, 100);                              
    PushTask(6000, 0,  100);                              
}

void BuzzerHandle(void)
{
    uint32_t now = DWT_GetTimeline_ms();
    if (count == 0)
    {
        PWMSetDutyRatio(buzzer_pwm, 0);
        return;
    }
    if (now >= task_timeout)
    {
        BuzzerTask_t *t = &task_queue[head];
        head = (head + 1) % BUZZER_TASK_NUM;
        count--;
        task_timeout = now + t->duration;
        if (t->volume > 0)
        {
            PWMSetPeriod(buzzer_pwm, 1.0f / t->freq);
            PWMSetDutyRatio(buzzer_pwm, t->volume * 0.005f);
        }
        else
        {
            PWMSetDutyRatio(buzzer_pwm, 0);
        }
    }
}

void BuzzerTask()
{
    uint8_t key_now = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    if (key_last == 1 && key_now == GPIO_PIN_RESET)
        BuzzerBeepOnce();
    key_last = key_now;
}