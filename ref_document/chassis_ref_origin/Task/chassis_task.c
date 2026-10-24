#include "chassis_task.h"
#include "remote_control.h"
#include "user_lib.h"
#include "detect_task.h"
#include "Freertos.h"
#include "task.h"
#include "CAN_receive.h"
#include "chassis_behaviour.h"
#include "leg_position.h"
#include "math.h"
#include "leg_speed.h"
#include "LQR_K.h"
#include "leg_force.h"
#include "BMI088driver.h"
#include "INS_task.h"
#include "arm_math.h"
#include "kalman_filter.h"

#include "KNN.h"
#include "ClimbKNN.h"

chassis_move_t chassis_move;//底盘运动数据
extern TwoBoardControlGimbal_t TwoBoardControlGimbal;

static void chassis_init(void);
static void chassis_feedback_update(void);
static void chassis_set_mode(void);
static void chassis_set_contorl(void);
static void chassis_control_loop(void);
static void chassis_Fn_calc(void);

KalmanFilter_t BodyMotionEstimation;
//状态向量x=[v a],量测向量z=[v a]
float MotionEstimation_P[4]={	1.0f,		0.0f,			//[1 0
								0.0f,		1.0f,		};	//	0 1]
float MotionEstimation_F[4]={	1.0f,		0.001f,			//[1 t
								0.0f,		1.0f,		};	//	0 1]
float MotionEstimation_H[4]={	1.0f,		0.0f,
								0.0f,		1.0f,		};	//不变
float MotionEstimation_Q[4]={	0.1f,		0.0f,			//[速度过程噪声       0	
								0.0f,		0.1f,		};	//       0       加速度过程噪声]Q_a要大，相信加速度从而实现滤掉打滑
float MotionEstimation_R[4]={	100.0f,		0.00f,			 //[速度测量噪声       0	
								0.0f,		1000000000000.0f,	};   //       0       加速度测量噪声]

//腿长平均值算LQR
float Leg_L;
//设置控制量
float x_set;
float dx_set;
								
//roll轴补偿力								
float stab_roll_F=0;
float Forward_F_Right;
float Forward_F_Left;								
static void chassis_speed_calc(void);
static void chassis_stay_STANDING(void);
static void chassis_stay_PID_CONTROL_LEG_AND_WHEEL(void);
static void chassis_stay_FAILING(void);
static void chassis_stay_ZERO_FORCE(void);
static void chassis_jump_set(void);
static void chassis_get_F(void);
static void chassis_stay_CLIMB_THE_STAIRS(void);
static void Chassis_Power_Control(void);
//旋转离心力补偿
float FnForFa=0;
float LeftFnForFa=0;
float RightFnForFa=0;
//跳跃补偿力							
float JumpThetaToMid=0;
//气弹簧力
float LeftSpringForceForV=0;
float LeftSpringForceForTp=0;
float RightSpringForceForV=0;
float RightSpringForceForTp=0;

#define GasSpringTempAngle 0.2443f
float LeftGasSpringLegToAngle=0;
float LeftGasSpringL=0; 
float LeftGasSpringForceForV=0; 

float RightGasSpringLegToAngle=0;
float RightGasSpringL=0; 
float RightGasSpringForceForV=0; 

float last_rightTp=0;
float last_leftTp=0;

//一点补偿让身体平衡
float pitchToBadyBalance=0.10;//0.12后退则补偿为正，前进补偿给负

float liftoff_detected=0;
float confidence=0;

float OutPowerWheelDx;
float OutPowerJointDx;
void chassis_task(void const *pvParameters)
{
  //空闲一段时间
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
  //底盘初始化
  chassis_init();
  //判断底盘电机是否都在线
   while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(CHASSIS_MOTOR5_TOE) || toe_is_error(CHASSIS_MOTOR6_TOE))
   {
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
   }
   
  while (1)
  {
    //设置底盘控制模式
    chassis_set_mode();
    //底盘数据更新
    chassis_feedback_update();
    //底盘控制量设置
    chassis_set_contorl();
    //底盘控制
		chassis_control_loop();
			//当遥控器掉线的时候，发送给底盘电机零电流.
			if (toe_is_error(DBUS_TOE))
			{
			CAN_cmd_wheel_left(0);
			CAN_cmd_wheel_right(0);
			}
			else
			{
				if(chassis_move.chassis_mode==CHASSIS_ZERO_FORCE||chassis_move.chassis_mode==CHASSIS_SELF_SAVING)
				{
					CAN_cmd_wheel_left(0);
					CAN_cmd_wheel_right(0);
				}
				else
				{
					CAN_cmd_wheel_left(chassis_move.Wheel_Left.set_current);
					CAN_cmd_wheel_right(chassis_move.Wheel_Right.set_current);
				}

			}
	//关节电机掉线时轮毂电机电流置零
	if(toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(CHASSIS_MOTOR5_TOE) || toe_is_error(CHASSIS_MOTOR6_TOE))
	{
		CAN_cmd_wheel_left(0);
		CAN_cmd_wheel_right(0);
	}
	
	//当有一个关节电机不正常则重新初始化
	if(chassis_move.Joint_Left_Ahead.chassis_motor_measure->Status!=1||
		 chassis_move.Joint_Left_Back.chassis_motor_measure->Status!=1||
		 chassis_move.Joint_Right_Ahead.chassis_motor_measure->Status!=1||
		 chassis_move.Joint_Right_Back.chassis_motor_measure->Status!=1)
   {
	//关节电机初始化
	Joint_motor_Init();
   }
	
    //系统延时
   vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  }
}

static void chassis_init(void)
{
	//速度估计卡尔曼
	Kalman_Filter_Init(&BodyMotionEstimation,2,0,2);
	memcpy(BodyMotionEstimation.P_data,MotionEstimation_P,sizeof(MotionEstimation_P));
	memcpy(BodyMotionEstimation.F_data,MotionEstimation_F,sizeof(MotionEstimation_F));
	memcpy(BodyMotionEstimation.H_data,MotionEstimation_H,sizeof(MotionEstimation_H));
	memcpy(BodyMotionEstimation.Q_data,MotionEstimation_Q,sizeof(MotionEstimation_Q));
	memcpy(BodyMotionEstimation.R_data,MotionEstimation_R,sizeof(MotionEstimation_R));
	//通过指针获取各个电机数据
	chassis_move.Wheel_Left.chassis_motor_measure=get_wheel_left_measure_point();
	chassis_move.Wheel_Right.chassis_motor_measure=get_wheel_right_measure_point();
	chassis_move.Joint_Left_Ahead.chassis_motor_measure=get_joint_left_ahead_measure_point();
	chassis_move.Joint_Left_Back.chassis_motor_measure=get_joint_left_back_measure_point();
	chassis_move.Joint_Right_Ahead.chassis_motor_measure=get_joint_right_ahead_measure_point();
	chassis_move.Joint_Right_Back.chassis_motor_measure=get_joint_right_back_measure_point();
	//腿长PID
	PID_init(&chassis_move.Right_Leg.L_PID,JOINT_CURRENT_KP,JOINT_CURRENT_KI,JOINT_CURRENT_KD,JOINT_CURRENT_I_BAND,JOINT_CURRENT_DT,JOINT_CURRENT_MAX_OUT,JOINT_CURRENT_MAX_IOUT);
	PID_init(&chassis_move.Left_Leg.L_PID,JOINT_CURRENT_KP,JOINT_CURRENT_KI,JOINT_CURRENT_KD,JOINT_CURRENT_I_BAND,JOINT_CURRENT_DT,JOINT_CURRENT_MAX_OUT,JOINT_CURRENT_MAX_IOUT);
	gimbal_PID_init(&chassis_move.Joint_Right_Ahead.JointAngle,JOINT_Angle_KP,JOINT_Angle_KI,JOINT_Angle_KD,JOINT_Angle_KF,JOINT_Angle_I_BAND,JOINT_Angle_DT,JOINT_Angle_MAX_OUT,JOINT_Angle_MAX_IOUT);//5,0,0,0,0,0.001,5,
	PID_init(&chassis_move.Joint_Right_Ahead.JointSpeed,JOINT_Speed_KP,JOINT_Speed_KI,JOINT_Speed_KD,JOINT_Speed_I_BAND,JOINT_Speed_DT,JOINT_Speed_MAX_OUT,JOINT_Speed_MAX_IOUT);//10,0,0,0,0.001,30,0
	gimbal_PID_init(&chassis_move.Joint_Right_Back.JointAngle,JOINT_Angle_KP,JOINT_Angle_KI,JOINT_Angle_KD,JOINT_Angle_KF,JOINT_Angle_I_BAND,JOINT_Angle_DT,JOINT_Angle_MAX_OUT,JOINT_Angle_MAX_IOUT);
	PID_init(&chassis_move.Joint_Right_Back.JointSpeed,JOINT_Speed_KP,JOINT_Speed_KI,JOINT_Speed_KD,JOINT_Speed_I_BAND,JOINT_Speed_DT,JOINT_Speed_MAX_OUT,JOINT_Speed_MAX_IOUT);
	gimbal_PID_init(&chassis_move.Joint_Left_Ahead.JointAngle,JOINT_Angle_KP,JOINT_Angle_KI,JOINT_Angle_KD,JOINT_Angle_KF,JOINT_Angle_I_BAND,JOINT_Angle_DT,JOINT_Angle_MAX_OUT,JOINT_Angle_MAX_IOUT);
	PID_init(&chassis_move.Joint_Left_Ahead.JointSpeed,JOINT_Speed_KP,JOINT_Speed_KI,JOINT_Speed_KD,JOINT_Speed_I_BAND,JOINT_Speed_DT,JOINT_Speed_MAX_OUT,JOINT_Speed_MAX_IOUT);
	gimbal_PID_init(&chassis_move.Joint_Left_Back.JointAngle,JOINT_Angle_KP,JOINT_Angle_KI,JOINT_Angle_KD,JOINT_Angle_KF,JOINT_Angle_I_BAND,JOINT_Angle_DT,JOINT_Angle_MAX_OUT,JOINT_Angle_MAX_IOUT);
	PID_init(&chassis_move.Joint_Left_Back.JointSpeed,JOINT_Speed_KP,JOINT_Speed_KI,JOINT_Speed_KD,JOINT_Speed_I_BAND,JOINT_Speed_DT,JOINT_Speed_MAX_OUT,JOINT_Speed_MAX_IOUT);
	//防劈叉PD
	chassis_move.Coordinate_Tp_P=COORDINATE_TP_P;
	chassis_move.Coordinate_Tp_D=COORDINATE_TP_D;
	//转向力矩的P
	chassis_move.Yaw_turn_P=YAW_TURN_P;
	//腿长初始设置
	chassis_move.Left_Leg.leg_L_set=CHASSIS_LEG_MIN;
	chassis_move.Right_Leg.leg_L_set=CHASSIS_LEG_MIN;
	chassis_move.L_set=CHASSIS_LEG_MIN;
	chassis_move.Left_Leg.leg_phi0_set=1.57;
	chassis_move.Right_Leg.leg_phi0_set=1.57;
	//初始化为无力模式
	chassis_move.chassis_mode = CHASSIS_ZERO_FORCE;
	//底盘数据获取
	chassis_feedback_update();
	//跳跃相关初始化
	chassis_move.chassis_jump_stage=SQUAT_DOWN;
	chassis_move.ClimbTheStairsFlag=0;
	//爬台阶相关初始化
	chassis_move.chassis_climb_stage=FALL_DOWN;
	chassis_move.ChassisClimbFallDownTime=0;
	//变速小陀螺相关初始化
	chassis_move.TopSwitchTime=0;
	
	chassis_move.ClimbFlag=0;
	impact_detector_init();
	chassis_move.AutoSave=0;
   //强行保证复位后时电机使能
   while (chassis_move.Joint_Left_Ahead.chassis_motor_measure->Status!=1||
		  chassis_move.Joint_Left_Back.chassis_motor_measure->Status!=1||
		  chassis_move.Joint_Right_Ahead.chassis_motor_measure->Status!=1||
		  chassis_move.Joint_Right_Back.chassis_motor_measure->Status!=1)
   {
	//关节电机初始化
	Joint_motor_Init();
	vTaskDelay(CHASSIS_CONTROL_TIME_MS);
   }
}

static void chassis_feedback_update(void)
{	
	/*左右腿坐标系统一朝y轴，四指的方向是减小方向*/
	//底盘腿长状态
	//后面的系数是调零点是电机角符合模型，可以先调左腿，右腿角度和左腿一致即可
	chassis_move.Left_Leg.leg_phi1=rad_format(chassis_move.Joint_Left_Back.chassis_motor_measure->Position-0.54f);
	chassis_move.Left_Leg.leg_phi4=rad_format(chassis_move.Joint_Left_Ahead.chassis_motor_measure->Position-3.78f+1.28f);
	leg_position(chassis_move.Left_Leg.leg_phi1,chassis_move.Left_Leg.leg_phi4,&chassis_move.Left_Leg.leg_L,&chassis_move.Left_Leg.leg_phi0);
	chassis_move.Right_Leg.leg_phi4=-rad_format(chassis_move.Joint_Right_Ahead.chassis_motor_measure->Position+4.25f);
	chassis_move.Right_Leg.leg_phi1=-rad_format(chassis_move.Joint_Right_Back.chassis_motor_measure->Position-2.4f);
	leg_position(chassis_move.Right_Leg.leg_phi1,chassis_move.Right_Leg.leg_phi4,&chassis_move.Right_Leg.leg_L,&chassis_move.Right_Leg.leg_phi0);
	//腿长速度状态
	chassis_move.Left_Leg.leg_d_phi1=chassis_move.Joint_Left_Back.chassis_motor_measure->Speed;
	chassis_move.Left_Leg.leg_d_phi4=chassis_move.Joint_Left_Ahead.chassis_motor_measure->Speed;
	leg_speed(chassis_move.Left_Leg.leg_d_phi1,chassis_move.Left_Leg.leg_d_phi4,chassis_move.Left_Leg.leg_phi1,chassis_move.Left_Leg.leg_phi4,&chassis_move.Left_Leg.leg_d_L,&chassis_move.Left_Leg.leg_d_phi0);
	chassis_move.Right_Leg.leg_d_phi1=-chassis_move.Joint_Right_Back.chassis_motor_measure->Speed;
	chassis_move.Right_Leg.leg_d_phi4=-chassis_move.Joint_Right_Ahead.chassis_motor_measure->Speed;
	leg_speed(chassis_move.Right_Leg.leg_d_phi1,chassis_move.Right_Leg.leg_d_phi4,chassis_move.Right_Leg.leg_phi1,chassis_move.Right_Leg.leg_phi4,&chassis_move.Right_Leg.leg_d_L,&chassis_move.Right_Leg.leg_d_phi0);
		/*更新状态空间*/
	//更新模型二阶倒立摆的一阶倾角（腿）
	chassis_move.Leg_State.left_theta=rad_format((chassis_move.Left_Leg.leg_phi0-1.57f)-INS_data.angle_pitch);
	chassis_move.Leg_State.left_d_theta=chassis_move.Left_Leg.leg_d_phi0-INS_data.wy;
	chassis_move.Leg_State.right_theta=rad_format(((chassis_move.Right_Leg.leg_phi0-1.57f))-INS_data.angle_pitch);
	chassis_move.Leg_State.right_d_theta=((chassis_move.Right_Leg.leg_d_phi0))-INS_data.wy;
	chassis_move.Leg_State.theta=rad_format(-INS_data.angle_pitch+((chassis_move.Left_Leg.leg_phi0-1.57f)+(chassis_move.Right_Leg.leg_phi0-1.57f))/2);//使电机旋转角度符合模型
	chassis_move.Leg_State.d_theta=-INS_data.wy+(chassis_move.Left_Leg.leg_d_phi0+(chassis_move.Right_Leg.leg_d_phi0))/2;//INS_data.wy;
	//更新模型二阶倒立摆的二阶倾角（机体）
	chassis_move.Leg_State.phi=INS_data.angle_pitch;
	chassis_move.Leg_State.d_phi=INS_data.wy;
	//更新速度融合原始值
	chassis_speed_calc();
	//速度融合
	if(chassis_move.chassis_mode==CHASSIS_ZERO_FORCE)
	{
		chassis_move.Leg_State.kf_x=0;
		chassis_move.Leg_State.kf_dx=0;
		chassis_move.Leg_State.d_x_raw=0;
		BodyMotionEstimation.MeasuredVector[0]=0;
		BodyMotionEstimation.MeasuredVector[1]=0;
		Kalman_Filter_Update(&BodyMotionEstimation);
	}
	else
	{
		BodyMotionEstimation.MeasuredVector[0]=chassis_move.Leg_State.d_x_raw;
		BodyMotionEstimation.MeasuredVector[1]=chassis_move.Leg_State.dd_x;
		Kalman_Filter_Update(&BodyMotionEstimation);
		chassis_move.Leg_State.kf_dx=BodyMotionEstimation.FilteredValue[0];
	}
	/*控速度控位移QR参数是差不多的，但是控速度手感更好，控位移起初调试的时候好用*/
	
	//控位移，相当于PD控制
//	chassis_move.Leg_State.kf_x+=chassis_move.Leg_State.kf_dx*0.001f;
//	x_set=x_set+chassis_move.vx_set*0.001f;
//	dx_set=0;
	//控速度，相当于PI控制
	if(fabs(chassis_move.Leg_State.kf_dx)<=0.1f)
	{	
	chassis_move.Leg_State.kf_x+=chassis_move.Leg_State.kf_dx*0.001f;
	}
	else
	{
	chassis_move.Leg_State.kf_x=0;
	}
/////////////////////////最后来不及细弄功率控制的下限方案//////////////////////
	//这个是我为了防止超功率，对速度项的最大限制
	if(chassis_move.ChassisPowerOutFlag)
	{
		OutPowerWheelDx=0.4;
		OutPowerJointDx=0.6;
	}
	else
	{
		OutPowerWheelDx=0.8;
		OutPowerJointDx=1.2;
	}
	//这个是我为了防止超功率，对速度set的最大限制
	if(chassis_move.ChassisPowerOutFlag)
	{
		dx_set=limit_symmetric(chassis_move.vx_set,1.3);
		if(SuperCap.capEnergy>100&&chassis_move.vx_set==0)
		{
		chassis_move.ChassisPowerOutFlag=0;
		}
	}
	else
	{
		dx_set=chassis_move.vx_set;
	}
///////////////////////////////////////////////////////////////////////////////
	//支持力计算
	chassis_Fn_calc();
}

static void chassis_set_mode(void)
{
	chassis_behaviour_mode_set(&chassis_move);
}

static void chassis_set_contorl(void)
{
  chassis_behaviour_control_set(&chassis_move);
}

static void chassis_control_loop(void)
{	
	//计算出支持力F
	chassis_get_F();
	//通过腿长求增益矩阵K
	LQR_K(Leg_L,chassis_move.Left_Leg.K);
	LQR_K(Leg_L,chassis_move.Right_Leg.K);
	//设置转向力矩
	//看操作手意思是否需要底盘跟随云台更快
	chassis_move.Yaw_turn=limit_symmetric(4*(chassis_move.Yaw_turn_P*(0-TwoBoardControlGimbal.relative_angle_yaw)-INS_data.wz),4.0f);//Kp控制旋转（需要闭环，闭环实现即转即停）
//	chassis_move.Yaw_turn=limit_symmetric(4*(TwoBoardControlGimbal.vy_set*2-INS_data.wz),2.0f);//Kp控制旋转（需要闭环，闭环实现即转即停），单闭环，无双板时可用
	if(chassis_move.chassis_mode==CHASSIS_TOP)
	{
		chassis_move.Yaw_turn=limit_symmetric(2*(chassis_move.wz_set-INS_data.wz),4.0f);//后面扭矩会除以2分配给轮毂，所以max可以是2倍的轮毂最大扭矩，但是out越大越消耗功率，看自己决定，当时我一级时功率不够用，急的改小了
	}  
	
			/*根据状态控制电机*/
	if(chassis_move.chassis_mode==CHASSIS_FOLLOW_GIMBAL||chassis_move.chassis_mode==CHASSIS_TOP)
	{
		
				if(toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(CHASSIS_MOTOR5_TOE) || toe_is_error(CHASSIS_MOTOR6_TOE))
				{
					chassis_move.chassis_state=ZERO_FORCE;
				}		
		//这个注释用于强行进入某个模式
		//chassis_move.chassis_state=PID_CONTROL_LEG_AND_WHEEL;
		switch(chassis_move.chassis_state)
		{
			case STANDING:
			{
				chassis_stay_STANDING();
				
				//正常站立后突然倒地，单箭头状态转换为无力，防止机体发散一直冲的保护
				if(fabs(INS_data.angle_pitch)>1.6f||chassis_move.Left_Leg.leg_phi0<0.4f||chassis_move.Left_Leg.leg_phi0>2.8f||chassis_move.Right_Leg.leg_phi0<0.4f||chassis_move.Right_Leg.leg_phi0>2.8f)
				{
					chassis_move.chassis_state=ZERO_FORCE;
				}
				//掉线保护
				if(toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(CHASSIS_MOTOR5_TOE) || toe_is_error(CHASSIS_MOTOR6_TOE))
				{
					chassis_move.chassis_state=ZERO_FORCE;
				}		
				
				impact_detector_update(chassis_move.Leg_State.kf_dx,chassis_move.Leg_State.wheel_dx,(chassis_move.Leg_State.kf_dx-chassis_move.Leg_State.wheel_dx),chassis_move.Leg_State.wheel_dx_Gap);
				chassis_move.ClimbFlag=impact_get_confidence();

///////////////////KNN检测上台阶///////////////////////////////////////////////////////
//自己调的参数不好，轮毂的R给太大，轮腿退化成小板凳（theta和pitch的K大小近似）				
//导致自己不适用腿倾角过大的方案
//但是速度差检测方案面对小弹丸打滑又由点无力，而且150mm和200mm的阈值不能共用
//所以自己用上了KNN检测，但是实际效果也并非特别好，也容易误判，自己连夜打补丁勉强实现效果（真是绕了好大一圈）
//我这个KNN判断适用于进距离伸腿进入该模式然后上台阶，远一点就容易误判（也是受限于时间，也许多采集远距离上台阶的数据就好）
//但是我近距离是考虑到远距离可能中间有小弹丸，但是新规则小弹丸少了，而且官方场地会自己清理小弹丸了，也许直接速度差就够用了QAQ
				if(Leg_L>=0.29f)
				{
				chassis_move.ClimbTime++;
					if(chassis_move.ClimbTime>=400)
					{
					chassis_move.ClimbTime=400;
					}
				}
				else
				{
				chassis_move.ClimbTime=0;
				}
					
				//蹭台阶时转换阈值
				if(chassis_move.ClimbTheStairsFlag==1&&impact_is_detected()&&chassis_move.ClimbTime==400&&chassis_move.vx_set>=0.2f)//&&(chassis_move.Leg_State.kf_dx-chassis_move.Leg_State.wheel_dx)<=-1.5f&&(chassis_move.Leg_State.kf_dx)>1.3f&&TwoBoardControlGimbal.legL_mode==3)//&&(chassis_move.Left_Leg.leg_phi0>2.0f&&chassis_move.Right_Leg.leg_phi0>2.0))
				{
					//chassis_move.ClimbFlag=1;
					chassis_move.chassis_state=CLIMB_THE_STAIRS;
				}
/////////////////////////////////////////////////////////////////////////////////////
				break;
			}
			case ZERO_FORCE:
			{
				chassis_stay_ZERO_FORCE();
				
				//这个if使得我死后复活无法直接进入不了下一个状态，除非换模式，使得遥控器换模式可以复活，或者按z键复活
				if(chassis_move.last_chassis_mode==CHASSIS_ZERO_FORCE&&(chassis_move.chassis_mode==CHASSIS_FOLLOW_GIMBAL||chassis_move.chassis_mode==CHASSIS_TOP))
				{
						//强行保证复位后时电机使能
					while(
					chassis_move.Joint_Left_Ahead.chassis_motor_measure->Status!=1||
					chassis_move.Joint_Left_Back.chassis_motor_measure->Status!=1||
					chassis_move.Joint_Right_Ahead.chassis_motor_measure->Status!=1||
					chassis_move.Joint_Right_Back.chassis_motor_measure->Status!=1)
					{
						//关节电机初始化
						Joint_motor_Init();
						vTaskDelay(CHASSIS_CONTROL_TIME_MS);
					}
					chassis_move.chassis_state=FALLING_DOWN;
					//身体朝上腿正常范围可直接准备起立
						if(fabs(INS_data.angle_pitch)<0.8f&&chassis_move.Right_Leg.leg_phi0<3.0f&&chassis_move.Right_Leg.leg_phi0>0.7f&&chassis_move.Left_Leg.leg_phi0<3.0f&&chassis_move.Left_Leg.leg_phi0>0.7f)
						{
							chassis_move.chassis_state=FALLING_TO_STANDING;
						}
				}
				break;
			}
			case FALLING_DOWN:
			{
				//这里原本想起立前头摆正前或正后，但是有点设计问题，旋转头和腿时会有概率卡死，所以变成了头无力旋转腿
				if(1)//(fabs(TwoBoardControlGimbal.relative_angle_yaw)<0.2f||fabs(TwoBoardControlGimbal.relative_angle_yaw)>2.9f)
				{
						//自救时腿伸长
						chassis_move.Right_Leg.leg_L_set=0.4;
						chassis_move.Left_Leg.leg_L_set=0.4;
						//通过设置phi角实现旋转腿，每次set越大转的越快
						chassis_move.Left_Leg.leg_phi0_set=chassis_move.Left_Leg.leg_phi0-0.3f;
						chassis_move.Right_Leg.leg_phi0_set=chassis_move.Right_Leg.leg_phi0-0.3f;
							//某个特殊的倒地自救腿需要换个方向
							if(INS_data.angle_pitch<-1.0f)
							{
								chassis_move.Left_Leg.leg_phi0_set=chassis_move.Left_Leg.leg_phi0+0.3f;
								chassis_move.Right_Leg.leg_phi0_set=chassis_move.Right_Leg.leg_phi0+0.3f;
							}
						//腿旋转到可起立状态就不转了
						if((chassis_move.Leg_State.left_theta<1.4f&&0.5f<chassis_move.Leg_State.left_theta)&&fabs(INS_data.angle_pitch)<0.8f)//((fabs(1.4f-chassis_move.Leg_State.left_theta)<0.1f&&fabs(INS_data.angle_pitch)<0.8f))
						{
							chassis_move.Left_Leg.leg_phi0_set=chassis_move.Left_Leg.leg_phi0;
						}
						if((chassis_move.Leg_State.right_theta<1.4f&&0.5f<chassis_move.Leg_State.right_theta)&&fabs(INS_data.angle_pitch)<0.8f)						//if(fabs(1.4f-chassis_move.Leg_State.right_theta)<0.1f&&fabs(INS_data.angle_pitch)<0.8f)
						{
							chassis_move.Right_Leg.leg_phi0_set=chassis_move.Right_Leg.leg_phi0;
						}
						//当两个腿差太大时，可能会导致起不来，一直游泳，所以需要变个差速
						if((fabs(chassis_move.Leg_State.left_theta-chassis_move.Leg_State.right_theta)>0.8)&&fabs(INS_data.angle_pitch)>0.3f)
						{
							if(INS_data.angle_pitch<-1.0f)
							{
							chassis_move.Left_Leg.leg_phi0_set=chassis_move.Left_Leg.leg_phi0+0.3f;
							chassis_move.Right_Leg.leg_phi0_set=chassis_move.Right_Leg.leg_phi0+0.6f;
							}
							else
							{
							chassis_move.Left_Leg.leg_phi0_set=chassis_move.Left_Leg.leg_phi0-0.3f;
							chassis_move.Right_Leg.leg_phi0_set=chassis_move.Right_Leg.leg_phi0-0.6f;
							}
						}
						
						chassis_stay_FAILING();
				}
					//当腿进入目标可起立状态则切换状态
						if((chassis_move.Leg_State.left_theta<1.4f&&0.5f<chassis_move.Leg_State.left_theta)&&(chassis_move.Leg_State.right_theta<1.4f&&0.5f<chassis_move.Leg_State.right_theta)&&fabs(INS_data.angle_pitch)<0.3f)					//if((fabs(1.4f-chassis_move.Leg_State.left_theta)<0.1f)&&(fabs(1.4f-chassis_move.Leg_State.right_theta)<0.1f)&&fabs(INS_data.angle_pitch)<0.3)
					{
						chassis_move.chassis_state=FALLING_TO_STANDING;
					}
				break;
			}
			case FALLING_TO_STANDING:
			{		
				chassis_move.Left_Leg.leg_L_set=0.12;
				chassis_move.Right_Leg.leg_L_set=0.12;
				chassis_move.L_set=0.12;
				chassis_move.Left_Leg.leg_phi0_set=1.57;
				chassis_move.Right_Leg.leg_phi0_set=1.57;
				chassis_stay_FAILING();
				
				//先变成小板凳再进入LQR
				if((fabs(chassis_move.Left_Leg.leg_phi0_set-chassis_move.Left_Leg.leg_phi0)<0.1f)&&(fabs(chassis_move.Right_Leg.leg_phi0_set-chassis_move.Right_Leg.leg_phi0)<0.1f))
				{
					chassis_move.chassis_state=STANDING;
					chassis_move.ClimbTheStairsFlag=0;
					chassis_move.skip_ClimbTheStairsFlag=0;
				}
				break;
			}
			case PID_CONTROL_LEG_AND_WHEEL://小板凳模式，调好后其实不怎么用的到，验证算法、参数、debug'时可能会再用到
			{		
				chassis_stay_PID_CONTROL_LEG_AND_WHEEL();
				break;
			}
			case CLIMB_THE_STAIRS://上台阶的状态
			{
				chassis_stay_CLIMB_THE_STAIRS();
				break;
			}
			default:
				chassis_stay_ZERO_FORCE();
				break;
		}
	}
	else
	{
		chassis_stay_FAILING();
	}
}



static void chassis_speed_calc(void)
{
	static float left_wheel_w,right_wheel_w;
	static float left_body_dx,right_body_dx;
	//最后要的dx是平动的速度，不能直接用编码器值，具体为何解算看论文
		left_wheel_w=chassis_move.Wheel_Left.chassis_motor_measure->speed_rads+INS_data.wy-chassis_move.Left_Leg.leg_d_phi0;//沿y轴正方向，四指方向为减小的方向
		right_wheel_w=-chassis_move.Wheel_Right.chassis_motor_measure->speed_rads+INS_data.wy-chassis_move.Right_Leg.leg_d_phi0;//沿y轴正方向，四指方向为减小的方向
		chassis_move.Leg_State.wheel_dx=(-left_wheel_w-right_wheel_w)*WheelRadius;
		chassis_move.Leg_State.wheel_dx_Gap=(left_wheel_w-right_wheel_w)*WheelRadius;
		left_body_dx=-left_wheel_w*WheelRadius+chassis_move.Left_Leg.leg_L*chassis_move.Leg_State.d_theta*arm_cos_f32(chassis_move.Leg_State.left_theta)+chassis_move.Left_Leg.leg_d_L*arm_sin_f32(chassis_move.Leg_State.left_theta);
		right_body_dx=-right_wheel_w*WheelRadius+chassis_move.Right_Leg.leg_L*chassis_move.Leg_State.d_theta*arm_cos_f32(chassis_move.Leg_State.right_theta)+chassis_move.Right_Leg.leg_d_L*arm_sin_f32(chassis_move.Leg_State.right_theta);
		chassis_move.Leg_State.d_x_raw=(right_body_dx+left_body_dx)/2;
	chassis_move.Leg_State.dd_x=INS_data.ax;
}

static void chassis_Fn_calc(void)
{
	//离地检测的支持力计算，具体看玺佬论文
	chassis_move.Right_Leg.leg_dd_L=0.19f*((chassis_move.Right_Leg.leg_d_L-chassis_move.Right_Leg.leg_d_L_last)/0.001f)+0.81f*chassis_move.Right_Leg.leg_dd_L;
	chassis_move.Left_Leg.leg_dd_L=0.19f*((chassis_move.Left_Leg.leg_d_L-chassis_move.Left_Leg.leg_d_L_last)/0.001f)+0.81f*chassis_move.Left_Leg.leg_dd_L;
	chassis_move.Leg_State.right_dd_theta=0.19f*((chassis_move.Leg_State.d_theta - chassis_move.Leg_State.right_d_theta_last)/0.001f)+0.81f*chassis_move.Leg_State.right_dd_theta;
	chassis_move.Leg_State.left_dd_theta=0.19f*((chassis_move.Leg_State.d_theta - chassis_move.Leg_State.left_d_theta_last)/0.001f)+0.81f*chassis_move.Leg_State.left_dd_theta;

	float P_R,P_L;
	P_R=-chassis_move.Right_Leg.F*arm_cos_f32(chassis_move.Leg_State.right_theta)+chassis_move.Right_Leg.Tp*arm_sin_f32(chassis_move.Leg_State.right_theta)/chassis_move.Right_Leg.leg_L;
	chassis_move.Right_Leg.Fn=P_R+1.54f*9.76f+1.54f*
	(
	INS_data.az-
	chassis_move.Right_Leg.leg_dd_L*arm_cos_f32(chassis_move.Leg_State.right_theta)+
	2*chassis_move.Right_Leg.leg_d_L*chassis_move.Leg_State.right_d_theta*arm_sin_f32(chassis_move.Leg_State.right_theta)+
	chassis_move.Right_Leg.leg_L*chassis_move.Leg_State.right_dd_theta*arm_sin_f32(chassis_move.Leg_State.right_theta)+
	chassis_move.Right_Leg.leg_L*chassis_move.Leg_State.right_d_theta*chassis_move.Leg_State.right_d_theta*arm_cos_f32(chassis_move.Leg_State.right_theta)
	);//注意：这里的Fn是地面对轮子的支持力，所以不应该加上气弹簧的竖值分量。但是可能是我忽略气弹簧带来的Tp，所以Fn算的较小（30、0），所以FnMax变成了-20
	
	P_L=-chassis_move.Left_Leg.F*arm_cos_f32(chassis_move.Leg_State.left_theta)+chassis_move.Left_Leg.Tp*arm_sin_f32(chassis_move.Leg_State.left_theta)/chassis_move.Left_Leg.leg_L;
	chassis_move.Left_Leg.Fn=P_L+1.54f*9.76f+1.54f*
	(
	INS_data.az-
	chassis_move.Left_Leg.leg_dd_L*arm_cos_f32(chassis_move.Leg_State.left_theta)+
	2*chassis_move.Left_Leg.leg_d_L*chassis_move.Leg_State.left_d_theta*arm_sin_f32(chassis_move.Leg_State.left_theta)+
	chassis_move.Left_Leg.leg_L*chassis_move.Leg_State.left_dd_theta*arm_sin_f32(chassis_move.Leg_State.left_theta)+
	chassis_move.Left_Leg.leg_L*chassis_move.Leg_State.left_d_theta*chassis_move.Leg_State.left_d_theta*arm_cos_f32(chassis_move.Leg_State.left_theta)
	);
	
	chassis_move.Right_Leg.leg_d_L_last=chassis_move.Right_Leg.leg_d_L;
	chassis_move.Leg_State.right_d_theta_last=chassis_move.Leg_State.d_theta;
	chassis_move.Left_Leg.leg_d_L_last=chassis_move.Left_Leg.leg_d_L;
	chassis_move.Leg_State.left_d_theta_last=chassis_move.Leg_State.d_theta;
}

static void chassis_stay_STANDING(void)
{
	 //不同的腿长情况给不同的补充，实现一个比较好的原地站立效果
		if(TwoBoardControlGimbal.legL_mode==3)
		{
		pitchToBadyBalance=0.13;
		}
		else if(TwoBoardControlGimbal.legL_mode==1)
		{
		pitchToBadyBalance=0.17;
		}
		else
		{
		pitchToBadyBalance=0.20;
		}
		
		chassis_move.Wheel_Left.Tp=	
		(0
		+chassis_move.Left_Leg.K[0]*(chassis_move.Leg_State.theta-pitchToBadyBalance)		
		+chassis_move.Left_Leg.K[2]*chassis_move.Leg_State.d_theta		
		-chassis_move.Left_Leg.K[4]*(x_set-chassis_move.Leg_State.kf_x)				
		-chassis_move.Left_Leg.K[6]*limit_symmetric((dx_set-chassis_move.Leg_State.kf_dx),OutPowerWheelDx)				
		+chassis_move.Left_Leg.K[8]*(chassis_move.Leg_State.phi)		
		+chassis_move.Left_Leg.K[10]*(chassis_move.Leg_State.d_phi)
		);
		chassis_move.Wheel_Right.Tp=	
		-(0
		+chassis_move.Right_Leg.K[0]*(chassis_move.Leg_State.theta-pitchToBadyBalance)			
		+chassis_move.Right_Leg.K[2]*chassis_move.Leg_State.d_theta	
		-chassis_move.Right_Leg.K[4]*(x_set-chassis_move.Leg_State.kf_x)				
		-chassis_move.Right_Leg.K[6]*limit_symmetric((dx_set-chassis_move.Leg_State.kf_dx),OutPowerWheelDx)					
		+chassis_move.Right_Leg.K[8]*(chassis_move.Leg_State.phi)		
		+chassis_move.Right_Leg.K[10]*(chassis_move.Leg_State.d_phi)			
		);
		chassis_move.Left_Leg.Tp=
		-(0
		+chassis_move.Left_Leg.K[1]*(chassis_move.Leg_State.theta-pitchToBadyBalance)	
		+chassis_move.Left_Leg.K[3]*chassis_move.Leg_State.d_theta
		-chassis_move.Left_Leg.K[5]*(x_set-chassis_move.Leg_State.kf_x)	
		-chassis_move.Left_Leg.K[7]*limit_symmetric((dx_set-chassis_move.Leg_State.kf_dx),OutPowerJointDx)				
		+chassis_move.Left_Leg.K[9]*(chassis_move.Leg_State.phi)	
		+chassis_move.Left_Leg.K[11]*chassis_move.Leg_State.d_phi		
		);
		chassis_move.Right_Leg.Tp=
		-(0
		+chassis_move.Right_Leg.K[1]*(chassis_move.Leg_State.theta-pitchToBadyBalance)		
		+chassis_move.Right_Leg.K[3]*chassis_move.Leg_State.d_theta	
		-chassis_move.Right_Leg.K[5]*(x_set-chassis_move.Leg_State.kf_x)	
		-chassis_move.Right_Leg.K[7]*limit_symmetric((dx_set-chassis_move.Leg_State.kf_dx),OutPowerJointDx)				
		+chassis_move.Right_Leg.K[9]*(chassis_move.Leg_State.phi)	
		+chassis_move.Right_Leg.K[11]*chassis_move.Leg_State.d_phi			
		);
		
		//对于刚加速时转弯，可能会倾斜并且触地，本质是转弯的扭矩影响平衡了，所以自己这里有个限幅
		if(fabs((chassis_move.Right_Leg.leg_phi0+chassis_move.Left_Leg.leg_phi0)/2-1.57f)>=0.45f)
		{
			chassis_move.Yaw_turn=limit_symmetric(chassis_move.Yaw_turn,fabs(chassis_move.Right_Leg.Tp)/2);
		}
		
					if(chassis_move.chassis_jump_stage==CONTRACTING_LEGS||chassis_move.chassis_jump_stage==EXTENDING_LEGS)
					{
					chassis_move.Wheel_Left.Tp=	
					(0
					+chassis_move.Left_Leg.K[0]*(chassis_move.Leg_State.theta-0.25)//在跳跃离地前给个神秘的偏置有利于身体在空中平衡，本质应该是离地前轮子旋转打滑了		
					+chassis_move.Left_Leg.K[2]*chassis_move.Leg_State.d_theta		
					+chassis_move.Left_Leg.K[4]*(x_set-chassis_move.Leg_State.kf_x)				
					+chassis_move.Left_Leg.K[6]*(dx_set-chassis_move.Leg_State.kf_dx)		
					+chassis_move.Left_Leg.K[8]*(chassis_move.Leg_State.phi)		
					+chassis_move.Left_Leg.K[10]*(chassis_move.Leg_State.d_phi)	
					);
					chassis_move.Wheel_Right.Tp=	
					-(0
					+chassis_move.Right_Leg.K[0]*(chassis_move.Leg_State.theta-0.25)			
					+chassis_move.Right_Leg.K[2]*chassis_move.Leg_State.d_theta		
					+chassis_move.Right_Leg.K[4]*(x_set-chassis_move.Leg_State.kf_x)				
					+chassis_move.Right_Leg.K[6]*(dx_set-chassis_move.Leg_State.kf_dx)		
					+chassis_move.Right_Leg.K[8]*(chassis_move.Leg_State.phi)		
					+chassis_move.Right_Leg.K[10]*(chassis_move.Leg_State.d_phi)	
					);
					}
		chassis_move.Wheel_Left.Tp=limit_symmetric(chassis_move.Wheel_Left.Tp,Maxout_3508);
		chassis_move.Wheel_Right.Tp=limit_symmetric(chassis_move.Wheel_Right.Tp,Maxout_3508);
		
		chassis_move.Wheel_Left.set_current=limit_symmetric((chassis_move.Wheel_Left.Tp-chassis_move.Yaw_turn/2),Maxout_3508)*TorqueToCurrent_3508*(268/17)/13.94f;//20*16384/0.3f/0.820941232f;
		chassis_move.Wheel_Right.set_current=limit_symmetric((chassis_move.Wheel_Right.Tp-chassis_move.Yaw_turn/2),Maxout_3508)*TorqueToCurrent_3508*(268/17)/13.94f;//20*16384/0.3f/0.820941232f;//((268/17)/(3591/187));

		//在这里加功率限制的函数，超级丑陋的功率控制，参考了港科轮腿功率控制和狼牙23功率控制////////////////////////////////////////////////////////////////////////////			
				Chassis_Power_Control();
				
		chassis_move.Left_Leg.Fn_flag=0;
		chassis_move.Right_Leg.Fn_flag=0;
		//防劈叉PID计算
		chassis_move.Coordinate_Tp=chassis_move.Coordinate_Tp_P*(chassis_move.Left_Leg.leg_phi0-chassis_move.Right_Leg.leg_phi0)+chassis_move.Coordinate_Tp_D*(chassis_move.Left_Leg.leg_d_phi0-chassis_move.Right_Leg.leg_d_phi0);
		
		//参考alan哥的话的KNN离地检测，但是自己采集的数据好像不太好，只有竖值抱着的时候腿和轮子才是松的，但是仍然可以下台阶和飞坡
		//原本采用离心力监测方案，可是在带气弹簧后容易误判导致抖动。（也是自己笨，气弹簧的补充本来不应该算入支持力里的）
	float raw_state[10] = {
			chassis_move.Leg_State.theta,    
			chassis_move.Leg_State.d_theta,   
			chassis_move.Leg_State.kf_x,      
			chassis_move.Leg_State.kf_dx,    
			chassis_move.Leg_State.phi,      
			chassis_move.Leg_State.d_phi,     
			chassis_move.Left_Leg.F,         
			chassis_move.Right_Leg.F,         
			chassis_move.Right_Leg.Tp,        
			chassis_move.Left_Leg.Tp          
	};

    confidence = knn_predict(raw_state);

    if (confidence >= THRESHOLD) {
        // 判断为离地
        liftoff_detected = 1;
    } else {
        liftoff_detected = 0;
    }
					
			//跳跃时同时进入离地检测
			if(liftoff_detected||(chassis_move.JumpFlyFlag==1&&chassis_move.Right_Leg.Fn_flag==1))
			{
				chassis_move.Left_Leg.Fn_flag=1;
				chassis_move.Left_Leg.Tp=
				-(0
				+chassis_move.Left_Leg.K[1]*(chassis_move.Leg_State.left_theta)
				+chassis_move.Left_Leg.K[3]*chassis_move.Leg_State.left_d_theta			
				);
				chassis_move.Wheel_Left.set_current=0;
				chassis_move.Coordinate_Tp=0;
				
						if(chassis_move.JumpFlyFlag==1)
						{
							chassis_move.Left_Leg.Tp=
							-(0
							+chassis_move.Left_Leg.K[1]*(chassis_move.Leg_State.theta)	
							+chassis_move.Left_Leg.K[3]*chassis_move.Leg_State.d_theta	
							);
						}
				
			}
			if(liftoff_detected||(chassis_move.JumpFlyFlag==1&&chassis_move.Left_Leg.Fn_flag==1))
			{
				chassis_move.Right_Leg.Fn_flag=1;
				chassis_move.Right_Leg.Tp=
				-(0
				+chassis_move.Right_Leg.K[1]*(chassis_move.Leg_State.right_theta)		
				+chassis_move.Right_Leg.K[3]*chassis_move.Leg_State.right_d_theta
				);
				chassis_move.Wheel_Right.set_current=0;
				chassis_move.Coordinate_Tp=0;
				
						if(chassis_move.JumpFlyFlag==1)
						{
							chassis_move.Right_Leg.Tp=
							-(0
							+chassis_move.Right_Leg.K[1]*(chassis_move.Leg_State.right_theta)		
							+chassis_move.Right_Leg.K[3]*chassis_move.Leg_State.right_d_theta
							);
						}
			}

		//计算平衡腿部力矩-
		chassis_move.Left_Leg.Tp=chassis_move.Left_Leg.Tp-chassis_move.Coordinate_Tp;//-LeftSpringForceForTp;
		chassis_move.Right_Leg.Tp=chassis_move.Right_Leg.Tp+chassis_move.Coordinate_Tp;//-RightSpringForceForTp;
			
		/*正方向看关节电机，逆时针为负*/
		//关节所需力矩解算到各个关节电机
		leg_force(chassis_move.Left_Leg.F,chassis_move.Left_Leg.Tp,chassis_move.Left_Leg.leg_phi1,chassis_move.Left_Leg.leg_phi4,chassis_move.Left_Leg.T);//chassis_move.Left_Leg.Tp
		chassis_move.Joint_Left_Ahead.set_Tp=chassis_move.Left_Leg.T[0];//+
		chassis_move.Joint_Left_Back.set_Tp=chassis_move.Left_Leg.T[1];//-
			
		leg_force(chassis_move.Right_Leg.F,chassis_move.Right_Leg.Tp,chassis_move.Right_Leg.leg_phi1,chassis_move.Right_Leg.leg_phi4,chassis_move.Right_Leg.T);//chassis_move.Right_Leg.Tp
		chassis_move.Joint_Right_Ahead.set_Tp=-chassis_move.Right_Leg.T[0];//-
		chassis_move.Joint_Right_Back.set_Tp=-chassis_move.Right_Leg.T[1];//+
}

static void chassis_stay_PID_CONTROL_LEG_AND_WHEEL(void)
{
		//底盘phi范围太小，倒地启动时加入x、dx会发散，也为之后加入腿部启动做准备
		chassis_move.Wheel_Left.Tp=	
		(0
//		+chassis_move.Left_Leg.K[0]*(chassis_move.Leg_State.theta)			
//		+chassis_move.Left_Leg.K[2]*(chassis_move.Leg_State.d_theta)
		-chassis_move.Right_Leg.K[4]*(x_set-chassis_move.Leg_State.kf_x)		
		-chassis_move.Right_Leg.K[6]*(dx_set-chassis_move.Leg_State.kf_dx)//limit_symmetric((dx_set-chassis_move.Leg_State.kf_dx),0.2)			
		+chassis_move.Left_Leg.K[8]*(chassis_move.Leg_State.phi)		
		+chassis_move.Left_Leg.K[10]*(chassis_move.Leg_State.d_phi)
		)
		;
		//扭矩赋值
		chassis_move.Wheel_Left.Tp=limit_symmetric(chassis_move.Wheel_Left.Tp,Maxout_3508);
		chassis_move.Wheel_Right.Tp=-chassis_move.Wheel_Left.Tp;
		chassis_move.Wheel_Left.set_current=(chassis_move.Wheel_Left.Tp-chassis_move.Yaw_turn/2)/20*16384/0.3f/0.820941232f;//((268/17)/(3591/187));//-chassis_move.Yaw_turn/2
		chassis_move.Wheel_Right.set_current=(chassis_move.Wheel_Right.Tp-chassis_move.Yaw_turn/2)/20*16384/0.3f/0.820941232f;
		
			chassis_move.Left_Leg.leg_L_set=CHASSIS_LEG_MIN;
			chassis_move.Right_Leg.leg_L_set=CHASSIS_LEG_MIN;
			chassis_move.L_set=CHASSIS_LEG_MIN;
			chassis_move.Left_Leg.leg_phi0_set=1.57;
			chassis_move.Right_Leg.leg_phi0_set=1.57;
			//逆解腿部，用于起立
			LegToJoint(chassis_move.Right_Leg.leg_L_set,chassis_move.Right_Leg.leg_phi0_set,&chassis_move.Right_Leg.leg_phi1_set,&chassis_move.Right_Leg.leg_phi4_set);
			gimbal_PID_calc(&chassis_move.Joint_Right_Ahead.JointAngle,chassis_move.Right_Leg.leg_phi4,chassis_move.Right_Leg.leg_phi4_set);
			PID_calc(&chassis_move.Joint_Right_Ahead.JointSpeed,chassis_move.Right_Leg.leg_d_phi4,chassis_move.Joint_Right_Ahead.JointAngle.out);
			chassis_move.Joint_Right_Ahead.set_Tp=-chassis_move.Joint_Right_Ahead.JointSpeed.out;
			
			gimbal_PID_calc(&chassis_move.Joint_Right_Back.JointAngle,chassis_move.Right_Leg.leg_phi1,chassis_move.Right_Leg.leg_phi1_set);
			PID_calc(&chassis_move.Joint_Right_Back.JointSpeed,chassis_move.Right_Leg.leg_d_phi1,chassis_move.Joint_Right_Back.JointAngle.out);
			chassis_move.Joint_Right_Back.set_Tp=-chassis_move.Joint_Right_Back.JointSpeed.out;

			LegToJoint(chassis_move.Left_Leg.leg_L_set,chassis_move.Left_Leg.leg_phi0_set,&chassis_move.Left_Leg.leg_phi1_set,&chassis_move.Left_Leg.leg_phi4_set);
			gimbal_PID_calc(&chassis_move.Joint_Left_Ahead.JointAngle,chassis_move.Left_Leg.leg_phi4,chassis_move.Left_Leg.leg_phi4_set);
			PID_calc(&chassis_move.Joint_Left_Ahead.JointSpeed,chassis_move.Joint_Left_Ahead.chassis_motor_measure->Speed,chassis_move.Joint_Left_Ahead.JointAngle.out);
			chassis_move.Joint_Left_Ahead.set_Tp=chassis_move.Joint_Left_Ahead.JointSpeed.out;
			
			gimbal_PID_calc(&chassis_move.Joint_Left_Back.JointAngle,chassis_move.Left_Leg.leg_phi1,chassis_move.Left_Leg.leg_phi1_set);
			PID_calc(&chassis_move.Joint_Left_Back.JointSpeed,chassis_move.Joint_Left_Back.chassis_motor_measure->Speed,chassis_move.Joint_Left_Back.JointAngle.out);
			chassis_move.Joint_Left_Back.set_Tp=chassis_move.Joint_Left_Back.JointSpeed.out;

}

static void chassis_stay_FAILING(void)
{
		chassis_move.Wheel_Left.set_current=0;
		chassis_move.Wheel_Right.set_current=0;
		//逆解腿部，用于起立
		LegToJoint(chassis_move.Right_Leg.leg_L_set,chassis_move.Right_Leg.leg_phi0_set,&chassis_move.Right_Leg.leg_phi1_set,&chassis_move.Right_Leg.leg_phi4_set);
	
		gimbal_PID_calc(&chassis_move.Joint_Right_Ahead.JointAngle,chassis_move.Right_Leg.leg_phi4,chassis_move.Right_Leg.leg_phi4_set);
		PID_calc(&chassis_move.Joint_Right_Ahead.JointSpeed,chassis_move.Right_Leg.leg_d_phi4,chassis_move.Joint_Right_Ahead.JointAngle.out);
		chassis_move.Joint_Right_Ahead.set_Tp=-chassis_move.Joint_Right_Ahead.JointSpeed.out;
		gimbal_PID_calc(&chassis_move.Joint_Right_Back.JointAngle,chassis_move.Right_Leg.leg_phi1,chassis_move.Right_Leg.leg_phi1_set);
		PID_calc(&chassis_move.Joint_Right_Back.JointSpeed,chassis_move.Right_Leg.leg_d_phi1,chassis_move.Joint_Right_Back.JointAngle.out);
		chassis_move.Joint_Right_Back.set_Tp=-chassis_move.Joint_Right_Back.JointSpeed.out;

		LegToJoint(chassis_move.Left_Leg.leg_L_set,chassis_move.Left_Leg.leg_phi0_set,&chassis_move.Left_Leg.leg_phi1_set,&chassis_move.Left_Leg.leg_phi4_set);
	
		gimbal_PID_calc(&chassis_move.Joint_Left_Ahead.JointAngle,chassis_move.Left_Leg.leg_phi4,chassis_move.Left_Leg.leg_phi4_set);
		PID_calc(&chassis_move.Joint_Left_Ahead.JointSpeed,chassis_move.Joint_Left_Ahead.chassis_motor_measure->Speed,chassis_move.Joint_Left_Ahead.JointAngle.out);
		chassis_move.Joint_Left_Ahead.set_Tp=chassis_move.Joint_Left_Ahead.JointSpeed.out;
		gimbal_PID_calc(&chassis_move.Joint_Left_Back.JointAngle,chassis_move.Left_Leg.leg_phi1,chassis_move.Left_Leg.leg_phi1_set);
		PID_calc(&chassis_move.Joint_Left_Back.JointSpeed,chassis_move.Joint_Left_Back.chassis_motor_measure->Speed,chassis_move.Joint_Left_Back.JointAngle.out);
		chassis_move.Joint_Left_Back.set_Tp=chassis_move.Joint_Left_Back.JointSpeed.out;
}

static void chassis_stay_ZERO_FORCE(void)
{
		chassis_move.Wheel_Left.set_current=0;
		chassis_move.Wheel_Right.set_current=0;
		chassis_move.Joint_Right_Ahead.set_Tp=0;
		chassis_move.Joint_Right_Back.set_Tp=0;
		chassis_move.Joint_Left_Ahead.set_Tp=0;
		chassis_move.Joint_Left_Back.set_Tp=0;
}

static void chassis_jump_set(void)
{
	//用elseif的状态机
	if(chassis_move.chassis_jump_stage==SQUAT_DOWN)//收腿
	{
		chassis_move.JumpFlyFlag=0;
		PID_init(&chassis_move.Right_Leg.L_PID,3000,0,200,JOINT_CURRENT_I_BAND,JOINT_CURRENT_DT,JOINT_CURRENT_MAX_OUT,JOINT_CURRENT_MAX_IOUT);
		PID_init(&chassis_move.Left_Leg.L_PID,3000,0,200,JOINT_CURRENT_I_BAND,JOINT_CURRENT_DT,JOINT_CURRENT_MAX_OUT,JOINT_CURRENT_MAX_IOUT);
		chassis_move.L_set=0.15f;
		chassis_move.Left_Leg.leg_L_set=0.15f;
		chassis_move.Right_Leg.leg_L_set=0.15f;
		chassis_move.JumpF=-200.0f;
		chassis_move.chassis_jump_stage=EXTENDING_LEGS;
	}
	//伸腿
	else if(chassis_move.chassis_jump_stage==EXTENDING_LEGS&&(chassis_move.Left_Leg.leg_L<=0.16f)&&(chassis_move.Right_Leg.leg_L<=0.16f))//&&(fabs(chassis_move.Leg_State.theta)<0.1f))
	{
		chassis_move.JumpFlyFlag=1;
		chassis_move.L_set=0.4f;
		chassis_move.Left_Leg.leg_L_set=0.4f;
		chassis_move.Right_Leg.leg_L_set=0.4f;
		chassis_move.JumpF=0.0f;//200.0f;
		chassis_move.chassis_jump_stage=CONTRACTING_LEGS;
	}
	//空中收腿
	else if(chassis_move.chassis_jump_stage==CONTRACTING_LEGS&&(chassis_move.Left_Leg.leg_L>=0.30f)&&(chassis_move.Right_Leg.leg_L>=0.30f))//&&(fabs(chassis_move.Leg_State.theta)<0.1f))
	{
		chassis_move.JumpFlyFlag=1;
		chassis_move.L_set=0.15f;
		chassis_move.Left_Leg.leg_L_set=0.1f;
		chassis_move.Right_Leg.leg_L_set=0.1f;
		chassis_move.JumpF=-200.0f;//0
		chassis_move.chassis_jump_stage=SOFT_LANDING;
	}
	//为落地准备伸腿
	else if(chassis_move.chassis_jump_stage==SOFT_LANDING&&(chassis_move.Left_Leg.leg_L<=0.18f)&&(chassis_move.Right_Leg.leg_L<=0.18f))
	{
		chassis_move.JumpFlyTime++;
		chassis_move.JumpFlyFlag=1;
		chassis_move.L_set=0.24f;
		chassis_move.Left_Leg.leg_L_set=0.18f;
		chassis_move.Right_Leg.leg_L_set=0.18f;
		chassis_move.JumpF=-200.0f;//0
		if(chassis_move.JumpFlyTime>=220)
		{
		chassis_move.chassis_jump_stage=RETURN_TO_READY;
		}
	}
	//为落地准备收腿
	else if(chassis_move.chassis_jump_stage==RETURN_TO_READY)//&&(chassis_move.Left_Leg.leg_L>=0.24f)&&(chassis_move.Right_Leg.leg_L>=0.24f))//&&(chassis_move.Left_Leg.Fn>FnMax)&&(chassis_move.Right_Leg.Fn>FnMax)
	{
		chassis_move.JumpFlyTime=0;
		chassis_move.JumpFlyFlag=1;
		chassis_move.L_set=0.24f;
		chassis_move.Left_Leg.leg_L_set=0.18f;
		chassis_move.Right_Leg.leg_L_set=0.18f;
		chassis_move.JumpF=60.0f;
		chassis_move.chassis_jump_stage=READY_TO_JUMP;
	}
	//为下次跳跃做准备，防止继续联调
	else if(chassis_move.chassis_jump_stage==READY_TO_JUMP)
	{
		chassis_move.JumpFlyFlag=0;
		chassis_move.L_set=0.24f;
		chassis_move.Left_Leg.leg_L_set=0.18f;
		chassis_move.Right_Leg.leg_L_set=0.18f;
		chassis_move.JumpF=60.0f;
	}
	
}

static void chassis_get_F(void)
{
	//跳的时候有新的F和腿长PID不一样，所以有个选择
	if(chassis_move.JumpFlag==1)
	{
		chassis_jump_set();
	}
	else
	{
		chassis_move.JumpFlyFlag=0;
		chassis_move.JumpF=0;
		chassis_move.chassis_jump_stage=SQUAT_DOWN;
		if(chassis_move.Right_Leg.L_PID.Ki!=JOINT_CURRENT_KI)
		{
		PID_init(&chassis_move.Right_Leg.L_PID,JOINT_CURRENT_KP,JOINT_CURRENT_KI,JOINT_CURRENT_KD,JOINT_CURRENT_I_BAND,JOINT_CURRENT_DT,JOINT_CURRENT_MAX_OUT,JOINT_CURRENT_MAX_IOUT);
		PID_init(&chassis_move.Left_Leg.L_PID,JOINT_CURRENT_KP,JOINT_CURRENT_KI,JOINT_CURRENT_KD,JOINT_CURRENT_I_BAND,JOINT_CURRENT_DT,JOINT_CURRENT_MAX_OUT,JOINT_CURRENT_MAX_IOUT);
		}
	}
	/*设定腿的F，包括跳跃力，roll轴补偿力，前馈力，设定L的PID力*/
	
				//roll轴补偿力计算，简单直接套了一个PD	
				stab_roll_F=(chassis_move.roll_set-INS_data.angle_roll)*StabRollForceKP-INS_data.wx*StabRollForceKD;
				//roll轴补偿力限幅
				if(stab_roll_F>StabRollForceMAX) 
				{
				stab_roll_F=StabRollForceMAX;
				}
				else if(stab_roll_F<-StabRollForceMAX) 
				{
				stab_roll_F=-StabRollForceMAX;
				}
				
		Forward_F_Right=ForwardForceRight;
		Forward_F_Left=ForwardForceRight;
	
							//无力时清零防止影响下次控制
							if(chassis_move.chassis_mode==CHASSIS_ZERO_FORCE)
							{
							stab_roll_F=0;
							Forward_F_Right=0;
							Forward_F_Left=0;
							}

	Leg_L=(chassis_move.Left_Leg.leg_L+chassis_move.Right_Leg.leg_L)/2;
						
			//南方科技大学24轮腿开源报告的离心力补偿方案，用于解决高速运动转向时离心力太大导致的翻车。
			FnForFa=1.5*(24*(2*Leg_L+0.5f)/8/0.5f/0.5f*fabs(pow(chassis_move.Wheel_Right.chassis_motor_measure->speed_rads*0.065f,2)-pow(chassis_move.Wheel_Left.chassis_motor_measure->speed_rads*0.065f,2)));
			if(fabs(chassis_move.Wheel_Right.chassis_motor_measure->speed_rads)>fabs(chassis_move.Wheel_Left.chassis_motor_measure->speed_rads))
			{
				RightFnForFa=FnForFa;
				LeftFnForFa=-FnForFa;
			}
			else if(fabs(chassis_move.Wheel_Right.chassis_motor_measure->speed_rads)<fabs(chassis_move.Wheel_Left.chassis_motor_measure->speed_rads))
			{
				LeftFnForFa=FnForFa;
				RightFnForFa=-FnForFa;
			}

		//气弹簧补偿部分
		LeftGasSpringLegToAngle=acos((0.21*0.21+0.25*0.25-chassis_move.Left_Leg.leg_L*chassis_move.Left_Leg.leg_L)/(2.0*0.21*0.25));
		LeftGasSpringL=sqrt(0.049*0.049+0.2*0.2-2*0.049*0.2*arm_cos_f32(LeftGasSpringLegToAngle-GasSpringTempAngle));
		LeftGasSpringForceForV=-2*300.0f*((0.049*0.2*arm_sin_f32(LeftGasSpringLegToAngle-GasSpringTempAngle))/(0.2*0.25*arm_sin_f32(LeftGasSpringLegToAngle)))*(chassis_move.Left_Leg.leg_L/LeftGasSpringL);
	
		RightGasSpringLegToAngle=acos((0.21*0.21+0.25*0.25-chassis_move.Right_Leg.leg_L*chassis_move.Right_Leg.leg_L)/(2.0*0.21*0.25));
		RightGasSpringL=sqrt(0.049*0.049+0.2*0.2-2*0.049*0.2*arm_cos_f32(RightGasSpringLegToAngle-GasSpringTempAngle));
		RightGasSpringForceForV=-2*300.0f*((0.049*0.2*arm_sin_f32(RightGasSpringLegToAngle-GasSpringTempAngle))/(0.2*0.25*arm_sin_f32(RightGasSpringLegToAngle)))*(chassis_move.Right_Leg.leg_L/RightGasSpringL);

		if(chassis_move.JumpFlyFlag==1)//跳跃时应该放开气弹簧的补充，不如就会像我一样0N和450N跳跃高度一样QAQ
		{
		LeftGasSpringForceForV=0;
		RightGasSpringForceForV=0;
		}
			
		//暂时通过设定腿长然后KP设定F,-是上提，正是下拉
	chassis_move.Right_Leg.F=-LeftGasSpringForceForV-Forward_F_Right-PID_calc(&chassis_move.Right_Leg.L_PID,chassis_move.Right_Leg.leg_L,chassis_move.Right_Leg.leg_L_set)-chassis_move.JumpF-RightFnForFa-stab_roll_F;
	chassis_move.Left_Leg.F=-RightGasSpringForceForV-Forward_F_Left-PID_calc(&chassis_move.Left_Leg.L_PID,chassis_move.Left_Leg.leg_L,chassis_move.Left_Leg.leg_L_set)-chassis_move.JumpF-LeftFnForFa+stab_roll_F;
			
}

static void chassis_stay_CLIMB_THE_STAIRS(void)
{
				//蹭台阶状态机
				switch(chassis_move.chassis_climb_stage)
				{
					case FALL_DOWN://身体刚倒下，腿定死
					{
						//这个状态应该是进入之后立马切换，是之前腿长时可直接通过倾角上台阶的方式
						chassis_move.ChassisClimbFallDownTime++;
						//腿身长摆正的控制量
						chassis_move.Left_Leg.leg_phi0_set=1.57;
						chassis_move.Right_Leg.leg_phi0_set=1.57;
						chassis_move.Right_Leg.leg_L_set=0.4;
						chassis_move.Left_Leg.leg_L_set=0.4;
										if(TwoBoardControlGimbal.legL_mode==3)//只要是伸着腿就可以进下一阶段，因为这个状态的逻辑判断已经放在STANDING
										{
											chassis_move.chassis_climb_stage=LEGS_TURN;
										}
									if(((chassis_move.ChassisClimbFallDownTime)>=20))//键鼠控制时的一个小bug的补充
									{
										chassis_move.ChassisClimbFallDownTime=0;
										chassis_move.chassis_state=STANDING;
									}
							break;
					}
					case LEGS_TURN://旋转腿
					{
						chassis_move.ChassisClimbFallDownTime++;
						chassis_move.Right_Leg.leg_L_set=0.4;
						chassis_move.Left_Leg.leg_L_set=0.4;
						chassis_move.Left_Leg.leg_phi0_set=chassis_move.Left_Leg.leg_phi0+0.5f;//改大小控制旋转速度
						chassis_move.Right_Leg.leg_phi0_set=chassis_move.Right_Leg.leg_phi0+0.5f;
						if((fabs(1.4f-chassis_move.Leg_State.left_theta)<0.1f))//旋转到大概的角度就停下
						{
							chassis_move.Left_Leg.leg_phi0_set=chassis_move.Left_Leg.leg_phi0;
						}
						if(fabs(1.4f-chassis_move.Leg_State.right_theta)<0.1f)
						{
							chassis_move.Right_Leg.leg_phi0_set=chassis_move.Right_Leg.leg_phi0;
						}
										//两条腿都转到目标位置然后准备收腿
										if((fabs(1.4f-chassis_move.Leg_State.left_theta)<0.1f)&&(fabs(1.4f-chassis_move.Leg_State.right_theta)<0.1f))
										{
											chassis_move.chassis_climb_stage=LEGS_SHORT;
										}
							break;
					}
					case LEGS_SHORT://收腿
					{
						//腿长变短，腿倾角不变
						chassis_move.Right_Leg.leg_L_set=0.1;
						chassis_move.Left_Leg.leg_L_set=0.1;
						chassis_move.Left_Leg.leg_phi0_set=chassis_move.Left_Leg.leg_phi0;
						chassis_move.Right_Leg.leg_phi0_set=chassis_move.Right_Leg.leg_phi0;
										if(chassis_move.Right_Leg.leg_L<0.2&&chassis_move.Left_Leg.leg_L<0.2)//收完腿就可以起立了
										{
											chassis_move.chassis_state=FALLING_TO_STANDING;
										}
							break;
					}
			default:
				chassis_move.chassis_state=ZERO_FORCE;
				break;
				}

			chassis_move.Wheel_Left.set_current=0;
			chassis_move.Wheel_Right.set_current=0;
			//逆解腿部，用于起立
			LegToJoint(chassis_move.Right_Leg.leg_L_set,chassis_move.Right_Leg.leg_phi0_set,&chassis_move.Right_Leg.leg_phi1_set,&chassis_move.Right_Leg.leg_phi4_set);
			gimbal_PID_calc(&chassis_move.Joint_Right_Ahead.JointAngle,chassis_move.Right_Leg.leg_phi4,chassis_move.Right_Leg.leg_phi4_set);
			PID_calc(&chassis_move.Joint_Right_Ahead.JointSpeed,chassis_move.Right_Leg.leg_d_phi4,chassis_move.Joint_Right_Ahead.JointAngle.out);
			chassis_move.Joint_Right_Ahead.set_Tp=-chassis_move.Joint_Right_Ahead.JointSpeed.out;
			
			gimbal_PID_calc(&chassis_move.Joint_Right_Back.JointAngle,chassis_move.Right_Leg.leg_phi1,chassis_move.Right_Leg.leg_phi1_set);
			PID_calc(&chassis_move.Joint_Right_Back.JointSpeed,chassis_move.Right_Leg.leg_d_phi1,chassis_move.Joint_Right_Back.JointAngle.out);
			chassis_move.Joint_Right_Back.set_Tp=-chassis_move.Joint_Right_Back.JointSpeed.out;

			LegToJoint(chassis_move.Left_Leg.leg_L_set,chassis_move.Left_Leg.leg_phi0_set,&chassis_move.Left_Leg.leg_phi1_set,&chassis_move.Left_Leg.leg_phi4_set);
			gimbal_PID_calc(&chassis_move.Joint_Left_Ahead.JointAngle,chassis_move.Left_Leg.leg_phi4,chassis_move.Left_Leg.leg_phi4_set);
			PID_calc(&chassis_move.Joint_Left_Ahead.JointSpeed,chassis_move.Joint_Left_Ahead.chassis_motor_measure->Speed,chassis_move.Joint_Left_Ahead.JointAngle.out);
			chassis_move.Joint_Left_Ahead.set_Tp=chassis_move.Joint_Left_Ahead.JointSpeed.out;
			
			gimbal_PID_calc(&chassis_move.Joint_Left_Back.JointAngle,chassis_move.Left_Leg.leg_phi1,chassis_move.Left_Leg.leg_phi1_set);
			PID_calc(&chassis_move.Joint_Left_Back.JointSpeed,chassis_move.Joint_Left_Back.chassis_motor_measure->Speed,chassis_move.Joint_Left_Back.JointAngle.out);
			chassis_move.Joint_Left_Back.set_Tp=chassis_move.Joint_Left_Back.JointSpeed.out;					
}

#define   MOTOR_3508_R 0.0989f//电阻常数
#define   MOTOR_3508_K 0.001709f
#define   MOTOR_3508_P 8.186f//热损耗和静息损耗常数
#define   RM3508_CURRENT_RATIO 20.0f/16384.0f//电流映射
		float Mright,Mleft,UleftSpeed,UrightSpeed,Uyaw,BlancePower,RemainPower,MovePower,TurnPower;
		float MleftToCurrent,MrightToCurrent,UyawToCurrent,UleftSpeedToCurrent,UrightSpeedToCurrent;
static void Chassis_Power_Control(void)
{
	//功率预测
	chassis_move.chassisPowerControl.ChassisPowerPredict=(0
	+MOTOR_3508_K*abs(chassis_move.Wheel_Left.set_current)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Left.chassis_motor_measure->speed_rpm)
	+MOTOR_3508_K*abs(chassis_move.Wheel_Right.set_current)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Right.chassis_motor_measure->speed_rpm)
	+MOTOR_3508_R*abs(chassis_move.Wheel_Left.set_current)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Left.set_current)*RM3508_CURRENT_RATIO
	+MOTOR_3508_R*abs(chassis_move.Wheel_Right.set_current)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Right.set_current)*RM3508_CURRENT_RATIO
	+MOTOR_3508_P
	+MOTOR_3508_P
	);
	
	//当预测的功率超出当前功率上限时
	if(((chassis_move.chassisPowerControl.ChassisPowerPredict>=TwoBoardControlGimbal.chassis_power_limit)&&(SuperCap.capEnergy<100)&&chassis_move.vx_set==0)||SuperCap.capEnergy<50)//TwoBoardControlGimbal.ChassisPower
	{
		chassis_move.ChassisPowerOutFlag=1;
		Mleft=// 左腿保持平衡扭矩
		(0
		+chassis_move.Left_Leg.K[0]*(chassis_move.Leg_State.theta-pitchToBadyBalance)//JumpThetaToMid是为了保持原地平衡的补充，JumpThetaToMid是跳跃时保证姿态		
		+chassis_move.Left_Leg.K[2]*chassis_move.Leg_State.d_theta		
		+chassis_move.Left_Leg.K[8]*(chassis_move.Leg_State.phi)		
		+chassis_move.Left_Leg.K[10]*(chassis_move.Leg_State.d_phi)	
		);
		Mright=//右腿保持平衡扭矩	
		-(0
		+chassis_move.Right_Leg.K[0]*(chassis_move.Leg_State.theta-pitchToBadyBalance)			
		+chassis_move.Right_Leg.K[2]*chassis_move.Leg_State.d_theta		
		+chassis_move.Right_Leg.K[8]*(chassis_move.Leg_State.phi)		
		+chassis_move.Right_Leg.K[10]*(chassis_move.Leg_State.d_phi)	
		);
		Uyaw=chassis_move.Yaw_turn;//旋转扭矩
		UleftSpeed=//左腿前进扭矩
		(-
		+chassis_move.Left_Leg.K[4]*(x_set-chassis_move.Leg_State.kf_x)				
		+chassis_move.Left_Leg.K[6]*(dx_set-chassis_move.Leg_State.kf_dx)	
		);
		UrightSpeed=//右腿前进扭矩
		(-
		+chassis_move.Right_Leg.K[4]*(x_set-chassis_move.Leg_State.kf_x)				
		+chassis_move.Right_Leg.K[6]*(dx_set-chassis_move.Leg_State.kf_dx)	
		);
		
		/*底盘保持平衡所需功率*/
		MleftToCurrent=Mleft*TorqueToCurrent_3508*(268/17)/13.94f;
		MrightToCurrent=Mright*TorqueToCurrent_3508*(268/17)/13.94f;
		BlancePower=
		(0
			+MOTOR_3508_K*fabs(MleftToCurrent)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Left.chassis_motor_measure->speed_rpm)
			+MOTOR_3508_K*fabs(MrightToCurrent)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Right.chassis_motor_measure->speed_rpm)
			+MOTOR_3508_R*fabs(MleftToCurrent)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Left.set_current)*RM3508_CURRENT_RATIO
			+MOTOR_3508_R*fabs(MrightToCurrent)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Right.set_current)*RM3508_CURRENT_RATIO
			+MOTOR_3508_P
			+MOTOR_3508_P
		);

		/*底盘移动需要的功率*/
		UleftSpeedToCurrent=UleftSpeed*TorqueToCurrent_3508*(268/17)/13.94f;
		UrightSpeedToCurrent=UrightSpeed*TorqueToCurrent_3508*(268/17)/13.94f;
		MovePower=
		(0
			+MOTOR_3508_K*fabs(UleftSpeedToCurrent)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Left.chassis_motor_measure->speed_rpm)
			+MOTOR_3508_K*fabs(UrightSpeedToCurrent)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Right.chassis_motor_measure->speed_rpm)
			+MOTOR_3508_R*fabs(UleftSpeedToCurrent)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Left.set_current)*RM3508_CURRENT_RATIO
			+MOTOR_3508_R*fabs(UrightSpeedToCurrent)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Right.set_current)*RM3508_CURRENT_RATIO
		);
		/*底盘旋转需要的功率*/
		UyawToCurrent=Uyaw*TorqueToCurrent_3508*(268/17)/13.94f;
		TurnPower=
		(0
			+MOTOR_3508_K*fabs(UyawToCurrent/2)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Left.chassis_motor_measure->speed_rpm)
			+MOTOR_3508_K*fabs(UyawToCurrent/2)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Right.chassis_motor_measure->speed_rpm)
			+MOTOR_3508_R*fabs(UyawToCurrent/2)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Left.set_current)*RM3508_CURRENT_RATIO
			+MOTOR_3508_R*fabs(UyawToCurrent/2)*RM3508_CURRENT_RATIO*abs(chassis_move.Wheel_Right.set_current)*RM3508_CURRENT_RATIO
		);
		
		//除保持平衡以外的剩余功率
		RemainPower=chassis_move.chassisPowerControl.ChassisPowerPredict-BlancePower;
		if(RemainPower<0)
		{
			RemainPower=0;
		}
		
		//衰减功率
		chassis_move.Wheel_Left.Tp=Mleft+UleftSpeed/(MovePower+TurnPower)*RemainPower*0.5f-Uyaw/2/(MovePower+TurnPower)*RemainPower*0.5f;//这里是固定衰减，最好肯定是可变衰减
		chassis_move.Right_Leg.Tp=Mright+UrightSpeed/(MovePower+TurnPower)*RemainPower*0.5f-Uyaw/2/(MovePower+TurnPower)*RemainPower*0.5f;
		if(chassis_move.vx_set==0)//不移动超功率的情况
		{
		chassis_move.Wheel_Left.Tp=Mleft+UleftSpeed/(MovePower+TurnPower)*RemainPower*0-Uyaw/2/(MovePower+TurnPower)*RemainPower*1;
		chassis_move.Right_Leg.Tp=Mright+UrightSpeed/(MovePower+TurnPower)*RemainPower*0-Uyaw/2/(MovePower+TurnPower)*RemainPower*1;
		}
		
		//重新设置电流
		chassis_move.Wheel_Left.Tp=limit_symmetric(chassis_move.Wheel_Left.Tp,Maxout_3508);
		chassis_move.Wheel_Right.Tp=limit_symmetric(chassis_move.Wheel_Right.Tp,Maxout_3508);
		chassis_move.Wheel_Left.set_current=(chassis_move.Wheel_Left.Tp)*TorqueToCurrent_3508*(268/17)/13.94f;//20*16384/0.3f/0.820941232f;
		chassis_move.Wheel_Right.set_current=(chassis_move.Wheel_Right.Tp)*TorqueToCurrent_3508*(268/17)/13.94f;//20*16384/0.3f/0.820941232f;//((268/17)/(3591/187));

	}
}
