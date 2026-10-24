#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct
{
    /* 仅保留 IMU 温控所需的位置式 PID。 */
    float Kp;
    float Ki;
    float Kd;
	
	float I_Band;
	
    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;

	float error;
	float pre_error;
	float pre_fdb;
	float dt;//控制周期，直接看任务vTaskDelay，比如vTaskDelay(2)，则为0.002

} pid_type_def;
/**
  * @brief          pid结构数据初始化
  * 
  * @param[out]     pid: PID结构数据指针
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
 */
extern void PID_init(pid_type_def *pid, float Kp, float Ki, float Kd, float I_Band, float dt, float max_out, float max_iout);

/**
 * @brief          pid计算
 * 
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @return         pid输出
 */
extern float PID_calc(pid_type_def *pid, float ref, float set);
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
