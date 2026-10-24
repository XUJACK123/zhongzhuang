#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#define ANGLE_T 8191

#define DM_J8009_PMAX				3.14f//DM_J8009电机位置最大值12.5rad
#define DM_J8009_PMIN				-3.14f//DM_J8009电机位置最小值-12.5rad
#define DM_J8009_VMAX				45.0f//DM_J8009电机速度最大值45rad/s
#define DM_J8009_VMIN				-45.0f//DM_J8009电机速度最小值-45rad/s
#define DM_J8009_TMAX				54.0f//DM_J8009电机转矩最大值54N·m
#define DM_J8009_TMIN				-54.0f//DM_J8009电机转矩最小值-54N·m

typedef struct
{
  uint8_t Status;//状态
	
  float Position;//位置(rad)
  float Speed;//速度(rad/s)
  float Torque;//转矩(N·m)
}Joint_Motor_t;

typedef struct
{
	//上发下
	float vx_set;										//1
	float vy_set;										//1
	uint8_t mode;										//1，跳跃flag
	float roll_set;									//1
	uint8_t legL_mode;							//1
	float relative_angle_yaw;				//3
	
	uint16_t buffer_energy_chassis;	//2,底盘缓冲能量
	uint16_t chassis_power_limit;		//2,底盘功率上限

	float LeftPhi0Set;							//1
	float LeftLengthSet;						//1
	float RightPhi0Set;							//1
	float RightLengthSet;						//1
	
	//下发上
	float LeftPhi0;									//1
	float LeftLength;								//1
	float RightPhi0;								//1
	float RightLength;							//1
	
}TwoBoardControlGimbal_t;

typedef struct
{
	uint8_t errorCode;          //错误码
	float chassisPower;         //底盘当前功率
	uint16_t chassisPowerLimit; //返回的功率上限
	uint8_t capEnergy;          //电容能量
}SuperCap_t;


typedef struct
{
  /*3508协议*/
  int16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
	/*减速比后的扭矩与转速*/
	float Torque;
	float speed_rads;
	/*记录旋转角度（减速比后）*/
	float angle; //int32型数据，存放累加ecd值用
	int32_t round;
	float last_angle;
} motor_measure_t;

extern void CAN_cmd_joint1(float Tor);
extern void CAN_cmd_joint2(float Tor);
extern void CAN_cmd_joint3(float Tor);
extern void CAN_cmd_joint4(float Tor);
extern void Joint_motor_Init(void);
extern void CAN_cmd_wheel_right(int16_t current);
extern void CAN_cmd_wheel_left(int16_t current);
extern void chassisToGimbal(void);
extern SuperCap_t SuperCap;
const motor_measure_t *get_wheel_left_measure_point(void);
const motor_measure_t *get_wheel_right_measure_point(void);
const Joint_Motor_t *get_joint_right_back_measure_point(void);
const Joint_Motor_t *get_joint_right_ahead_measure_point(void);
const Joint_Motor_t *get_joint_left_ahead_measure_point(void);
const Joint_Motor_t *get_joint_left_back_measure_point(void);
#endif
