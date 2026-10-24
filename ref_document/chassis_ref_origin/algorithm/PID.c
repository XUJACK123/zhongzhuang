#include "pid.h"
#include "math.h"
#include "user_lib.h"

#define LimitMax(input, max)   \
	{                          \
		if (input > max)       \
		{                      \
			input = max;       \
		}                      \
		else if (input < -max) \
		{                      \
			input = -max;      \
		}                      \
	}

void abs_limit(float *num, float Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}
	
/**
 * @brief          pid结构数据初始化
 * @param[out]     pid: PID结构数据指针
 * @param[in]      kd
 * @param[in]      ki
 * @param[in]      kd
 * @param[in]      kf 注意：这个前馈其实只是前馈的一种，简单以set值前馈应该不适应大部分情况
 * @param[in]      max_out: pid最大输出
 * @param[in]      max_iout: pid最大积分输出
 */

void PID_init(pid_type_def *pid, float Kp, float Ki, float Kd, float I_Band, float dt, float max_out, float max_iout)
{
	if (pid == NULL)
	{
		return;
	}
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->I_Band=I_Band;
	pid->dt=dt;
	pid->max_out = max_out;
	pid->max_iout = max_iout;
	pid->error = pid->pre_error = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief          pid计算
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @return         pid输出
 */
float PID_calc(pid_type_def *pid, float ref, float set)
{
	if (pid == NULL)
	{
		return 0.0f;
	}
	//数据更新
	pid->pre_error = pid->error;
	pid->set = set;
	pid->fdb = ref;
	pid->error = set - ref;
	//比例
	pid->Pout = pid->Kp * pid->error;
	//积分
	if(fabs(pid->error)<pid->I_Band)//积分分离，避免过早引入积分
	{
		pid->Iout += pid -> Ki * (pid->error+pid->pre_error)/2 *pid->dt;//梯形积分比直接积分更精确更顺滑
	}
	else
	{
		pid->Iout=0;
	}
	LimitMax(pid->Iout, pid->max_iout); //积分限幅
	//微分
	pid->Dout = -pid->Kd * ((pid->fdb-pid->pre_fdb)/pid->dt);//应该加个低通滤波，勉强先用了
	//输出
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	LimitMax(pid->out, pid->max_out);
	//数据更新
	pid->pre_error=pid->error;
	pid->pre_fdb=pid->fdb;
	
	return pid->out;
}

/**
 * @brief          pid 输出清除
 * @param[out]     pid: PID结构数据指针
 * @retval         none
 */
void PID_clear(pid_type_def *pid)
{
	if (pid == NULL)
	{
		return;
	}
	pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
	pid->fdb = pid->set = 0.0f;
}

/**
 * @brief 云台PID初始化函数
 *
 * @param pid PID数据结构指针
 * @param kp
 * @param ki
 * @param kd
 * @param kf
 * @param maxout 最大输出
 * @param max_iout 最大积分输出
 */
void gimbal_PID_init(gimbal_PID_t *pid, float Kp, float Ki, float Kd, float Kf, float I_Band, float dt, float max_out, float max_iout)
{
	if (pid == NULL)
	{
		return;
	}
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->I_Band=I_Band;
	pid->dt=dt;
	pid->max_out = max_out;
	pid->max_iout = max_iout;
	pid->error = pid->pre_error = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief 云台PID计算函数
 *
 * @param pid
 * @param get
 * @param set
 * @param error_delta 误差微分项，直接从传感器读取，不计算
 * @return fp32
 */
float gimbal_PID_calc(gimbal_PID_t *pid, float get, float set)
{
	if (pid == NULL)
	{
		return 0.0f;
	}
	//数据更新
	pid->pre_error = pid->error;
	pid->set = set;
	pid->fdb = get;
	pid->error = rad_format(set - get);
	//比例
	pid->Pout = pid->Kp * pid->error;
	//积分
	if(fabs(pid->error)<pid->I_Band)//积分分离，避免过早引入积分
	{
		pid->Iout += pid -> Ki * (pid->error+pid->pre_error)/2 *pid->dt;//梯形积分比直接积分更精确更顺滑
	}
	else
	{
		pid->Iout=0;
	}
	LimitMax(pid->Iout, pid->max_iout); //积分限幅
	//微分
	pid->Dout = -pid->Kd * ((pid->fdb-pid->pre_fdb)/pid->dt);
	//前馈
	pid->Fout = pid->Kf*(pid->set-pid->last_set);
	//输出
	pid->out = pid->Pout + pid->Iout + pid->Dout+pid->Fout;
	LimitMax(pid->out, pid->max_out);
	//数据更新
	pid->last_set=pid->set;
	pid->pre_error=pid->error;
	pid->pre_fdb=pid->fdb;
	
	return pid->out;
}
