/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"

#include "CAN_receive.h"
#include "pid.h"

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357*3
//底盘任务控制间隔 1ms
#define CHASSIS_CONTROL_TIME_MS 1

//腿长PID
#define JOINT_CURRENT_KP 1000.0f//600
#define JOINT_CURRENT_KI 100.0f//400如果给零注意是否影响跳跃后的腿长PID
#define JOINT_CURRENT_KD 120.0f//80
#define JOINT_CURRENT_KF 0.0f//100000
#define JOINT_CURRENT_I_BAND 0.04f
#define JOINT_CURRENT_DT 0.001f
#define JOINT_CURRENT_MAX_OUT 250.0f
#define JOINT_CURRENT_MAX_IOUT 50.0f

#define COORDINATE_TP_P 40.0f//40.0f
#define COORDINATE_TP_D 3.0f

#define YAW_TURN_P -10.0f

#define Maxout_3508 3.3		//用峰值扭矩算再稍微给大一点点应该也没问题（看减速箱的扭矩转速图），用最大电流算转速快时会急速下降，大概最大值5
#define WheelRadius 0.065f
#define ForwardForceLeft 		100
#define ForwardForceRight 	100
#define StabRollForceMAX	60 
#define StabRollForceKP		100
#define StabRollForceKD		40	
#define DEADBAND_3508 		300					//650
#define TorqueToCurrent_3508 3608.020604302f 	//20*16384/0.3/(268/17)/(3591/187)，但是本车后面轮毂减速比又改了，所以会有*(268/17)/13.94f，不过其他268/17可以直接用这个
#define FnMax				-20//-20//50

#define JOINT_Angle_KP 10.0f
#define JOINT_Angle_KI 0.0f
#define JOINT_Angle_KD 0.0f
#define JOINT_Angle_KF 0.0f
#define JOINT_Angle_I_BAND 0.0f
#define JOINT_Angle_DT 0.001f
#define JOINT_Angle_MAX_OUT 10.0f
#define JOINT_Angle_MAX_IOUT 0.0f

#define JOINT_Speed_KP 8.0f
#define JOINT_Speed_KI 0.0f
#define JOINT_Speed_KD 0.0f
#define JOINT_Speed_I_BAND 0.0f
#define JOINT_Speed_DT 0.001f
#define JOINT_Speed_MAX_OUT 30.0f
#define JOINT_Speed_MAX_IOUT 0.0f

typedef enum
{
  CHASSIS_ZERO_FORCE = 0,   	//底盘无力, 就是直接不控制底盘
  CHASSIS_FOLLOW_GIMBAL,   		//底盘跟随云台旋转
  CHASSIS_TOP,   				//底盘小陀螺

  CHASSIS_SELF_SAVING,  		//底盘自救
	
  CHASSIS_VMC_TEXT,				//底盘测试VMC
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float speed;          		//当前速度
  float speed_rpm_set;     		//设定速度
  int16_t set_current; 		//发送电流
  float Tp;
	
//  pid_type_def current_pid;
} wheel_motor_t;

typedef struct
{
  const Joint_Motor_t *chassis_motor_measure;
  float set_Tp; 		//发送电流
  gimbal_PID_t JointAngle;
  pid_type_def JointSpeed;
} joint_motor_t;

typedef struct
{
  double leg_phi1;
  double leg_phi4;
  double leg_L;
  double leg_phi0;
	
  float leg_phi1_set;
  float leg_phi4_set;
  float leg_L_set;
  float leg_phi0_set;
	
  double leg_d_phi1;
  double leg_d_phi4;
  double leg_d_L;
  double leg_d_phi0;
	
  double K[12];
  double Tp;
	
  double F,T[2];
  double leg_dd_L;
  double leg_d_L_last;
  double Fn;
  uint8_t Fn_flag;
  pid_type_def L_PID;
} Leg_state_t;

typedef struct
{
  double left_theta;
  double left_d_theta;
  double right_theta;
  double right_d_theta;
	
  double theta;	
  double d_theta;
  double d_x_raw;
	double wheel_dx;
  double kf_x;
  double kf_dx;
  double dd_x;
  double phi;
  double d_phi;
	
  double right_dd_theta;
  double left_dd_theta;
  double right_d_theta_last;
  double left_d_theta_last;
	
	double wheel_dx_Gap;
} state_t;

typedef struct
{
	float ChassisPowerPredict;
} chassisPowerControl_t;

typedef enum
{
  FALLING_DOWN = 0,   		//底盘处于倒地状态
  FALLING_TO_STANDING,		//底盘处于可起立状态
  STANDING,								//底盘处于正常站立状态
  ZERO_FORCE,							//底盘无力死亡

  PID_CONTROL_LEG_AND_WHEEL,//底盘腿部PID控制，轮毂LQR控制，用于前期调试用
	CLIMB_THE_STAIRS,					//底盘蹭台阶时转腿
} chassis_state_e;

typedef enum
{
  READY_TO_JUMP = 0, 
	SQUAT_DOWN,
	EXTENDING_LEGS,
	CONTRACTING_LEGS,
	SOFT_LANDING,
	RETURN_TO_READY,

} chassis_jump_stage_e;

typedef enum
{
  FALL_DOWN = 0, 
	LEGS_TURN,
	LEGS_SHORT,
} chassis_climb_stage_e;

typedef struct
{
  //电机信息
  wheel_motor_t Wheel_Left;
  wheel_motor_t Wheel_Right;
  joint_motor_t Joint_Left_Ahead;
  joint_motor_t Joint_Left_Back;
  joint_motor_t Joint_Right_Ahead;
  joint_motor_t Joint_Right_Back;
  //腿部状态
  Leg_state_t Left_Leg;
  Leg_state_t Right_Leg;
  //腿部状态变量
  state_t Leg_State;
  //底盘模式
  chassis_mode_e chassis_mode;
  chassis_mode_e last_chassis_mode;
  //底盘当前状态
  chassis_state_e chassis_state;
	
  float Coordinate_Tp;
  float Coordinate_Tp_P;
  float Coordinate_Tp_D;
	
  float Yaw_turn;
  float Yaw_turn_P;
  float Yaw_turn_D;
	
  float vx_set;
  float wz_set;
  
  float delat_L;
  float L_set;
  float roll_set;
	
	float JumpF;
	uint8_t JumpFlag;
	uint8_t JumpFlyFlag;
	chassis_jump_stage_e chassis_jump_stage;
	uint16_t JumpFlyTime;
	
	uint8_t ClimbTheStairsFlag;
	uint8_t skip_ClimbTheStairsFlag;
	chassis_climb_stage_e chassis_climb_stage;
	uint16_t ChassisClimbFallDownTime;
	
	uint16_t TopSwitchTime;
	chassisPowerControl_t chassisPowerControl;
	uint8_t UpFlag;
	float ClimbFlag;
	uint16_t ClimbTime;
	
	uint16_t AutoSave;
	uint8_t ChassisPowerOutFlag;
}chassis_move_t;

/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
extern void chassis_task(void const *pvParameters);

#endif
