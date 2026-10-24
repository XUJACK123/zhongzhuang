//#ifndef __RS485_RECEIVE_H
//#define __RS485_RECEIVE_H

//#include "main.h"

//#define UNITREE_RX_BUF_NUM 78
//#define UNITREE_SEND_FRAME_LENGTH 34

//typedef struct __attribute__((packed))
//{
//	uint8_t start_1;
//	uint8_t start_2;
//	uint8_t Motor_ID;
//	uint8_t Reserved_1;//¿ÉºöÂÔ
//	
//	uint8_t mode;
//	uint8_t ModifyBit;//¿ÉºöÂÔ
//	uint8_t ReadBit;//¿ÉºöÂÔ
//	uint8_t Reserved_2;//¿ÉºöÂÔ
//	
//	uint32_t Modify;//¿ÉºöÂÔ
//	int16_t Motor_T;//float*256
//	int16_t Motor_W;//float*128
//	int32_t Motor_P;
//	int16_t Motor_P_K;
//	int16_t Motor_W_K;
//	int8_t LowHzMotorCmdIndex;//¿ÉºöÂÔ
//	int8_t LowHzMotorCmdByte;
//	int32_t Reserved_3;//¿ÉºöÂÔ
//	
//	int32_t CRC_Data;
//}A1_Send_t;

//typedef struct 
//{
//	float Current_Torque;
//	float Current_Speed;
//	float Current_Position;
//	uint8_t Current_Temp;
//	uint8_t Current_Error;
//}Joint_motor_t;

//extern void RS485_Receive_Init(void);
//extern void USART2_cmd_current(int16_t Motor_T,uint8_t ID);
//extern void USART3_cmd_current(int16_t Motor_T,uint8_t ID);
//extern uint8_t unitree_usart_buf[2][UNITREE_RX_BUF_NUM];

//const Joint_motor_t *get_joint_left_ahead_measure_point(void);
//const Joint_motor_t *get_joint_left_back_measure_point(void);
//const Joint_motor_t *get_joint_right_ahead_measure_point(void);
//const Joint_motor_t *get_joint_right_back_measure_point(void);

//#endif
