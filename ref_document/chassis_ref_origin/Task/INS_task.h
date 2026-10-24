#ifndef INS_TASK_H
#define INS_TASK_H

#include "main.h"
#include "kalman_filter.h"

#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间

#define TEMPERATURE_PID_KP 100.0f 			//温度控制PID的kp
#define TEMPERATURE_PID_KI 25.0f    		//温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f    			//温度控制PID的kd
#define TEMPERATURE_PID_KF 0.0f   			//温度控制PID的kf
#define TEMPERATURE_PID_I_BAND 5.0f    		//温度控制PID的ki的“死区”，小于这个值Iout为0
#define TEMPERATURE_PID_DT 0.001f   		//温度控制PID的控制时间
#define TEMPERATURE_PID_MAX_OUT 1500.0f  	//温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 500.0f 	//温度控制PID的max_iout


typedef struct
{
  float angle_yaw;
  float angle_pitch;
  float angle_roll;

  float wx;
  float wy;
  float wz;

  float ax;
  float ay;
  float az;
	
} INS_data_t;

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

#define X 0
#define Y 1
#define Z 2

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // 四元数估计值
    float GyroBias[3]; // 陀螺仪零偏估计值

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    float dt; // 姿态更新周期
    mat ChiSquare;
    float ChiSquare_Data[1];      // 卡方检验检测函数
    float ChiSquareTestThreshold; // 卡方检验阈值
    float lambda;                 // 渐消因子

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

extern INS_data_t INS_data;
extern void INS_task(void const *pvParameters);

#endif
