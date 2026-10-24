#include "user_lib.h"
#include "tim.h"

/**
 * @brief          一阶低通滤波初始化
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      滤波参数
 * @retval         返回空
 */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
 * @brief          一阶低通滤波计算
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @retval         返回空
 */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//卡尔曼滤波
/*        
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好        
*/
double KalmanFilter(const double ResrcData, Kf*kf)
{
    double R = kf->MeasureNoise_R;
    double Q = kf->ProcessNiose_Q;
    double x_mid = kf->x_last;
    double x_now;


    double p_mid;
    double p_now;
    double kg;

    x_mid = kf->x_last;                           //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid = kf->p_last + Q;                       //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
    kg = p_mid / (p_mid + R);                 //kg为kalman filter，R为噪声
    x_now = x_mid + kg * (ResrcData - x_mid); //估计出的最优值

    p_now = (1 - kg) * p_mid; //最优值对应的covariance

    kf->p_last = p_now; //更新covariance值
    kf->x_last = x_now; //更新系统状态值

    return x_now;
}

/**
 * @brief  对称限幅函数
 * @param  value   需要被限幅的值
 * @param  max_limit 正向限幅值
 * @return         限幅后的值
 */
float limit_symmetric(float value, float max_limit)
{
    if (value > max_limit)
	{
        return max_limit;
    } else if (value < -max_limit)
	{
        return -max_limit;
    }
    return value;
}

void My_delay_us(uint16_t us)
{
	//*TIM1挂载的时钟为275M，除以设置的预分频（275-1），得1M是频率，即1us计数一次*/
	 uint16_t  count = 0xffff-us-15535;//0xffff是定时器1的向上计算最大值65535，为了好判断是否到延迟时间，加了个偏移15535
	__HAL_TIM_SetCounter(&htim1,count);//设置40000-us为TIM1当前值
	HAL_TIM_Base_Start(&htim1);//开始向上计数
	while( count<0xffff-15535)//当值大于等于40000值时结束循环，即delay_us的结束
    {
        count = __HAL_TIM_GetCounter(&htim1);//获取当前值判断是否能否结束循环
    };
	HAL_TIM_Base_Stop(&htim1);//停止计数，用于下次循环
}

//循环限幅函数
float loop_fp32_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}
//快速开平方
float fast_sqrt(float x)
{
    float half_x = 0.5f * x;
    int i = *(int*)&x;          // 将浮点数解释为整数
    i = 0x5f3759df - (i >> 1);  // 魔法常数
    x = *(float*)&i;           // 再解释回浮点数
    x = x * (1.5f - half_x * x * x);  // 牛顿迭代
    return 1.0f / x;
}
