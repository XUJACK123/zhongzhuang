//这页代码好像是移植玺佬＋参考草凡佬改的，受限本人知识，最后就算套个数据直接用了

#include "INS_task.h"
#include "Freertos.h"
#include "task.h"
#include "BMI088driver.h"
#include "PID.h"
#include "kalman_filter.h"

extern TIM_HandleTypeDef htim3;

bmi088_real_data_t bmi088_real_data;
pid_type_def imu_temp_pid;
INS_data_t INS_data;
QEKF_INS_t QEKF_INS;
IMU_Param_t IMU_Param;
INS_t INS;
static volatile uint32_t ins_update_time_ms;
static volatile bool ins_valid;

float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0,
                                 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, 0,
                                 0, 0, 0, 0, 0, 1};
float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 10000, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 10000};
float IMU_QuaternionEKF_Q[36] = {0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 0.01, 0, 0, 0,
                                 0, 0, 0, 0.01, 0, 0,
                                 0, 0, 0, 0, 0.000001, 0,
                                 0, 0, 0, 0, 0, 0.000001};
float IMU_QuaternionEKF_R[9] = {1000000, 0, 0,
                                0, 1000000, 0,
                                0, 0, 1000000};
float IMU_QuaternionEKF_K[18];
float IMU_QuaternionEKF_H[18];
//机体坐标系 (Body Frame, b系)，板子安装时的坐标系，受板子安装影响(这里没用到可以不管)
//xb: 机头向前，yb: 向右，zb: 向下
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};
//导航坐标系 (Navigation Frame, n系)，世界坐标系（这里因为速度融合用到世界加速度，所以板子安装位置水平时要对应修改）
const float gravity[3] = {0, 0, 9.81f};

static void IMU_TempControl(void);
static void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);
static void INS_Init(void);
static void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
								
void INS_task(void const *pvParameters)
{
        (void)pvParameters;
	vTaskDelay(INS_TASK_INIT_TIME);
	//陀螺仪初始化
    while(BMI088_init())
    {
    vTaskDelay(100);
    }
	INS_Init();

	while(1)
	{
		//陀螺仪温度控制
		IMU_TempControl();
		
		BMI088_read(bmi088_real_data.gyro,bmi088_real_data.accel,&bmi088_real_data.temp);
		INS.Gyro[0]=bmi088_real_data.gyro[0];
		INS.Gyro[1]=bmi088_real_data.gyro[1];
		INS.Gyro[2]=bmi088_real_data.gyro[2];
		INS.Accel[0]=bmi088_real_data.accel[0];
		INS.Accel[1]=bmi088_real_data.accel[1];
		INS.Accel[2]=bmi088_real_data.accel[2];
		
        IMU_QuaternionEKF_Update(INS.Gyro[0], INS.Gyro[1], INS.Gyro[2], INS.Accel[0], INS.Accel[1], INS.Accel[2], 0.001);
		memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体坐标系到世界坐标系（就是两个坐标系转化，乘以一个旋转矩阵而已），
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);
		
        // 世界坐标系到机体坐标系,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * 0.001f / (INS.AccelLPF + 0.001f) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + 0.001f);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 乘一个旋转矩阵得到世界系的三轴加速度

        // 获取最终数据
        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
		
		//因为是后移植的代码，考虑的原来的兼容性，最后对原来的INS_data进行了赋值操作，最好应该是修改并删去INS_data部分
		/*注意：我这里xyz好像交叉给负号什么的，是相当于根据当前陀螺仪安装方式，乘以一个旋转矩阵
				保证朝前为正x轴，左是正y轴，上是正z轴
				以右手法则，大拇指指向轴的正方向，四指的旋转方向为角速度和角度的减小的方向
				在c板代码里的旋转矩阵作用最后也和我这个一样，可以考虑写成那样
					修改时不会旋转矩阵，可以看当前xyz轴和希望的比较，然后直接改对应的xyz轴
					比如，希望正x轴是负y轴，正y轴是正x轴，可以x与y交换，然后给y一个负号就像我这样*/
		INS_data.angle_yaw=-QEKF_INS.Yaw;
		INS_data.angle_pitch=-QEKF_INS.Pitch;
		INS_data.angle_roll=QEKF_INS.Roll;
		INS_data.ax=-INS.MotionAccel_n[1];//QEKF_INS.Accel[0];
		INS_data.ay=-INS.MotionAccel_n[0];//QEKF_INS.Accel[1];
		INS_data.az=INS.MotionAccel_n[2];//QEKF_INS.Accel[2];
		INS_data.wx=QEKF_INS.Gyro[1];
		INS_data.wy=-QEKF_INS.Gyro[0];
		INS_data.wz=-QEKF_INS.Gyro[2];
		ins_update_time_ms = HAL_GetTick();
		ins_valid = true;
		
		vTaskDelay(1);
	}
}

void INS_get_snapshot(ins_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    memcpy(snapshot->quaternion, INS.q, sizeof(snapshot->quaternion));
    snapshot->euler_rad[0] = INS_data.angle_roll;
    snapshot->euler_rad[1] = INS_data.angle_pitch;
    snapshot->euler_rad[2] = INS_data.angle_yaw;
    snapshot->gyro_rad_s[0] = INS_data.wx;
    snapshot->gyro_rad_s[1] = INS_data.wy;
    snapshot->gyro_rad_s[2] = INS_data.wz;
    snapshot->acceleration_m_s2[0] = INS_data.ax;
    snapshot->acceleration_m_s2[1] = INS_data.ay;
    snapshot->acceleration_m_s2[2] = INS_data.az;
    snapshot->update_time_ms = ins_update_time_ms;
    snapshot->valid = ins_valid;
    taskEXIT_CRITICAL();
}

static void IMU_TempControl(void)
{
	//PID计算
	PID_calc(&imu_temp_pid,bmi088_real_data.temp,45.0f);
	
	//温控限幅
	if (imu_temp_pid.out > imu_temp_pid.max_out) imu_temp_pid.out = imu_temp_pid.max_out;
	if (imu_temp_pid.out < 0) imu_temp_pid.out = 0.0f;
	
	//最高温度限制保护
	if(bmi088_real_data.temp>=48.0f) imu_temp_pid.out=0;
	
	//out值
	htim3.Instance->CCR4 = (uint16_t)imu_temp_pid.out;
}

//快速开倒数平方
float invSqrt(float x) {
	float halfx = 0.5f * x;
	union { float value; uint32_t bits; } convert = { .value = x };
	convert.bits = 0x5f3759dfUL - (convert.bits >> 1);
	float y = convert.value;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void INS_Init(void)
{
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

	//f_c = 1 / (2π × τ) 根据截止频率选定τ
	//τ 越大 → 截止频率越低 → 滤波越强（更平滑，但滞后更大）
	//τ 越小 → 截止频率越高 → 滤波越弱（响应更快，但噪声多）
	//f_c可以用频谱分析法找界限，这里直接用王工的参数τ后面感兴趣可以改一改（这里一定用问题，只能说效果不是最好的）
	//另：根据对BMI088的配置并查手册，可以找到截止频率z轴200Hz，x、y轴为230Hz，所以可以得τ=1/（200*2π）=0.000795（待参试，最好是频谱分析法）
    INS.AccelLPF = 0.0085;//王工的0.0085参数jcope看波形与官方例程相比太陡峭了
	
	//陀螺仪温控初始化
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	PID_init(&imu_temp_pid, TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD ,TEMPERATURE_PID_I_BAND,TEMPERATURE_PID_DT,TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
	
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s2
 * @param[in]       update period in s
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
	//相比普通的卡尔曼，F和H是会变，是非线性的，在速度估计中是线性的，所以没有外挂函数
	
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    static float halfgxdt, halfgydt, halfgzdt;
    static float accelInvNorm;
    if (!QEKF_INS.Initialized)
    {
        IMU_QuaternionEKF_Init(10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);//没用卡方检测
    }

    /*
	F, number with * represent vals to be set
	
     0      1*     2*     3*  |   4     5
     6*     7      8*     9*  |  10    11
    12*    13*    14     15*  |  16    17
    18*    19*    20*    21   |  22    23
	---------------------------------------
    24     25     26     27   |  28    29
    30     31     32     33   |  34    35
    */
    QEKF_INS.dt = dt;

	// 测量值-零漂值
    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    // F的左上角部分 4x4子矩阵所需的三个值
    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * dt;

    // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
    if (QEKF_INS.UpdateCount == 0) // 如果是第一次进入,需要初始化低通滤波
    {
        QEKF_INS.Accel[0] = ax;
        QEKF_INS.Accel[1] = ay;
        QEKF_INS.Accel[2] = az;
    }
	
	//一阶低通滤波传递函数 H(s) = 1 / (τs + 1) -> 离散化 y[k] = y[k-1] * (τ / (τ + Δt)) + x[k] *Δt / (τ + Δt) 
    QEKF_INS.Accel[0] = QEKF_INS.Accel[0] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ax * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[1] = QEKF_INS.Accel[1] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ay * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[2] = QEKF_INS.Accel[2] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + az * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);

    // set z,单位化重力加速度向量
    accelInvNorm = invSqrt(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] + QEKF_INS.Accel[1] * QEKF_INS.Accel[1] + QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);
    for (uint8_t i = 0; i < 3; i++)
    {
        QEKF_INS.IMU_QuaternionEKF.MeasuredVector[i] = QEKF_INS.Accel[i] * accelInvNorm; // 用加速度向量更新量测值
    }

    // 单位化后判断机体是否处于运动稳定
    QEKF_INS.gyro_norm = 1.0f / invSqrt(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] + QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] + QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2]);
    QEKF_INS.accl_norm = 1.0f / accelInvNorm;

    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    // 稍后在最后的姿态更新部分会利用StableFlag来确定
    if (QEKF_INS.gyro_norm < 0.3f && QEKF_INS.accl_norm > 9.8f - 0.5f && QEKF_INS.accl_norm < 9.8f + 0.5f)
    {
        QEKF_INS.StableFlag = 1;
    }
    else
    {
        QEKF_INS.StableFlag = 0;
    }

    // set Q R,过程噪声和观测噪声矩阵
    QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.R_data[0] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[4] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[8] = QEKF_INS.R;

    // 调用kalman_filter.c封装好的函数,注意几个User_Funcx_f的调用
    Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

    // 获取融合后的数据,包括四元数和xy零飘值（在Kalman_Filter_Update里更新的FilteredValue）
    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];
    QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移

    // 利用四元数反解欧拉角
    QEKF_INS.Yaw = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]), 2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) - 1.0f);// * 57.295779513f;
    QEKF_INS.Pitch = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]), 2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) - 1.0f);// * 57.295779513f;
    QEKF_INS.Roll = asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2]));// * 57.295779513f;

    // get Yaw total, yaw数据可能会超过360/57.295779513f,处理一下方便其他功能使用(如小陀螺)
    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f/57.295779513f)
    {
        QEKF_INS.YawRoundCount--;
    }
    else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f/57.295779513f)
    {
        QEKF_INS.YawRoundCount++;
    }
    QEKF_INS.YawTotalAngle = QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
	QEKF_INS.UpdateCount++; // 初始化低通滤波自锁，也看数据更新了几次
}

void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf)
{
	//自锁，只初始化一次
    QEKF_INS.Initialized = 1;
	
	//设置过程噪声和测量噪声
	//Q1是q0,q1,q2,q3四元数的过程噪声，Q2是w_bias_x，w_bias_y轴的过程噪声（对零偏不完全信任），R是三维加速度的测量噪声
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;
	
	//卡方检测相关变量，但是移植自王工开源代码，应该没有完全发挥作用
	//应该是做了简化，可能是加上卡方检测，代码运算量太大
    QEKF_INS.ChiSquareTestThreshold = 1e-8;
    QEKF_INS.ConvergeFlag = 0;
    QEKF_INS.ErrorCount = 0;
	//测量更新计次和初始化滤波自锁
	
    QEKF_INS.UpdateCount = 0;//测量更新计次和初始化滤波自锁
    if (lambda > 1)
    {
        lambda = 1;
    }
    QEKF_INS.lambda = lambda;
    QEKF_INS.accLPFcoef = lpf;

    // 初始化矩阵维度信息
    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);
    Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, (float *)QEKF_INS.ChiSquare_Data);

    // 姿态初始化
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0;

    // 自定义函数初始化,用于扩展或增加kf的基础功能
    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;

    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

//获取量测信息
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));//当前协方差矩阵，反映观测数据修正后的新不确定性。
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));//当前的增益系数（即可信权重）
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));//当前的量测函数的雅各比矩阵，用于之后的预测
}
//先验估计
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;
    static float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // quaternion normalize
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    kf->F_data[4] = q1 * QEKF_INS.dt / 2;
    kf->F_data[5] = q2 * QEKF_INS.dt / 2;

    kf->F_data[10] = -q0 * QEKF_INS.dt / 2;
    kf->F_data[11] = q3 * QEKF_INS.dt / 2;

    kf->F_data[16] = -q3 * QEKF_INS.dt / 2;
    kf->F_data[17] = -q0 * QEKF_INS.dt / 2;

    kf->F_data[22] = q2 * QEKF_INS.dt / 2;
    kf->F_data[23] = -q1 * QEKF_INS.dt / 2;

    // fading filter,防止零飘参数过度收敛
    kf->P_data[28] /= QEKF_INS.lambda;
    kf->P_data[35] /= QEKF_INS.lambda;

    // 限幅,防止发散
    if (kf->P_data[28] > 10000)
    {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000)
    {
        kf->P_data[35] = 10000;
    }
}
//预测更新
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    static float doubleq0, doubleq1, doubleq2, doubleq3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}
//量测更新
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;

    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    // 计算预测得到的重力加速度方向(通过姿态获取的)
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 计算预测值和各个轴的方向余弦
    for (uint8_t i = 0; i < 3; i++)
    {
        QEKF_INS.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    // 利用加速度计数据修正
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test,卡方检验
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare);
    // rk is small,filter converged/converging
    if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold)
    {
        QEKF_INS.ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged
    if (QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
    {
        if (QEKF_INS.StableFlag)
        {
            QEKF_INS.ErrorCount++; // 载体静止时仍无法通过卡方检验
        }
        else
        {
            QEKF_INS.ErrorCount = 0;
        }

        if (QEKF_INS.ErrorCount > 50)
        {
            // 滤波器发散
            QEKF_INS.ConvergeFlag = 0;
            kf->SkipEq5 = FALSE; // step-5 is cov mat P updating
        }
        else
        {
            //  残差未通过卡方检验 仅预测
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE; // part5 is P updating
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk越小则增益越大,否则更相信预测值
        if (QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
        {
            QEKF_INS.AdaptiveGainScale = (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0]) / (0.9f * QEKF_INS.ChiSquareTestThreshold);
        }
        else
        {
            QEKF_INS.AdaptiveGainScale = 1;
        }
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = FALSE;
    }

    // cal kf-gain K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // implement adaptive
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; i++)
    {
        kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf->K_data[i * 3 + j] *= QEKF_INS.OrientationCosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))

    // 零漂修正限幅,一般不会有过大的漂移
    if (QEKF_INS.ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * QEKF_INS.dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * QEKF_INS.dt;
            }
        }
    }

    // 不修正yaw轴数据
    kf->temp_vector.pData[3] = 0;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

/**
 * @brief          机体坐标系下的向量转换到地球坐标系
 * @param[1]       机体系向量
 * @param[2]       地球系向量
 * @param[3]       四元数，表示机体坐标系相对于地球坐标系的旋转姿态
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          地球坐标系到机体坐标系
 * @param[1]       地球系向量
 * @param[2]       机体系向量
 * @param[3]       四元数，表示地球坐标系相对于机体坐标系的旋转姿态
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

