#ifndef BUZZER_H
#define BUZZER_H
#include "bsp_pwm.h"
#define BUZZER_DEVICE_CNT 5

#define  DoFreq  523
#define  ReFreq  587
#define  MiFreq  659
#define  FaFreq  698
#define  SoFreq  784
#define  LaFreq  880
#define  SiFreq  988

typedef enum
{
    OCTAVE_1 = 0,
    OCTAVE_2,
    OCTAVE_3,
    OCTAVE_4,
    OCTAVE_5,
    OCTAVE_6,
    OCTAVE_7,
    OCTAVE_8,
}octave_e;

typedef enum
{
    ALARM_LEVEL_HIGH = 0,
    ALARM_LEVEL_ABOVE_MEDIUM,
    ALARM_LEVEL_MEDIUM,
    ALARM_LEVEL_BELOW_MEDIUM,
    ALARM_LEVEL_LOW,
}AlarmLevel_e;

typedef enum
{
    ALARM_OFF = 0,
    ALARM_ON,
}AlarmState_e;
typedef struct
{
    AlarmLevel_e alarm_level;
    octave_e octave;
    float loudness;
}Buzzer_config_s;

typedef struct
{
    float loudness;
    octave_e octave;
    AlarmLevel_e alarm_level;
    AlarmState_e alarm_state;
}BuzzzerInstance;

#define BUZZER_TASK_NUM 20                                                                              
                                                                                                        
typedef struct {                                                                                        
    uint32_t freq;     // 频率 Hz                                                                       
    uint16_t duration; // 时长 ms                                                                       
    uint8_t volume;    // 音量 0-100                                                                    
} BuzzerTask_t;                                                                                         
                                                                                                        
// BuzzerTask() 用这个代替遍历 buzzer_list                                                              


void BuzzerInit();
void BuzzerTask();
void BuzzerHandle(void);
void BuzzerBeepTriple(void);  // 哔三声  
void BuzzerBeepOnce(void);
#endif // !BUZZER_H
