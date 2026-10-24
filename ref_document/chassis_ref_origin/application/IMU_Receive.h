#ifndef IMU_RECEIVE_H
#define IMU_RECEIVE_H

#include "main.h"

#define WIT_DATA_LENGTH 44
#define WIT_BUF_LENGTH  2*WIT_DATA_LENGTH

#define wit_acc_head 	0x51
#define wit_gyro_head 	0x52
#define wit_ang_head 	0x53
#define wit_meg_head 	0x54

#define acc_u16_to_float 	0.00478515625f
#define gyro_u16_to_float 	0.06103515625f
#define ang_u16_to_float 	0.0054931640625f

#define degree_to_rad 0.01745329f

typedef struct
{
	int16_t iAcc[3];
	int16_t iGyro[3];
	int16_t iAngle[3];
	int16_t iMeg[3];	//单位毫高斯
	
	float fAcc[3];		//加速度，m/s2
	float fGyro[3];		//角速度，°/s
	float fGyro_rad[3];	//角速度，rad/s
	float fAngle[3];	//角度，°
	float fAngle_rad[3];//角度，rad
	float fMeg[3];		//单位是微特，10毫高斯=1μT
	
	uint8_t temperature;
	uint8_t version;
	
}WIT_data_t;

void uart_to_INS(uint8_t buf[]);
extern void WIT_imu_init(void);
extern WIT_data_t IMU_Data;
#endif
