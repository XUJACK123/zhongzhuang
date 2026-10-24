#ifndef USER_LIB_H
#define USER_LIB_H

#include "main.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

typedef __packed struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

typedef struct filter
{
    double ProcessNiose_Q;
    double MeasureNoise_R;
    double x_last;
    double p_last;
} Kf;

/**
 * @brief 卡尔曼滤波
 *
 * @param[in] ResrcData
 * @param[in] ProcessNiose_Q
 * @param[in] MeasureNoise_R
 * @return double
 */
double KalmanFilter(const double ResrcData, Kf *kf);

extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
extern float limit_symmetric(float value, float max_limit);
extern void My_delay_us(uint16_t us);
extern float loop_fp32_constrain(float Input, float minValue, float maxValue);
extern float fast_sqrt(float x);
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
