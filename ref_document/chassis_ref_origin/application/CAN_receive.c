#include "CAN_receive.h"
#include "main.h"
#include "detect_task.h"
#include <string.h>

#include "chassis_task.h"
extern chassis_move_t chassis_move;//底盘运动数据

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
FDCAN_TxHeaderTypeDef Header;
uint8_t chassis_can_send_data[8];
uint8_t ChaToGim_can_send_data[8];

motor_measure_t motor_chassis[2];
Joint_Motor_t Joint_motor[4];
TwoBoardControlGimbal_t TwoBoardControlGimbal;
SuperCap_t SuperCap;
static void Calc_Motor_Angle(motor_measure_t *motor);
static float DM_J8009_uint_to_float(int x,float x_min,float x_max,int bits);

// hal库CAN回调函数,接收电机数据
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);	
	switch (rx_header.Identifier)
	{
		case 0x201: //左轮
		{
			motor_chassis[0].last_ecd=motor_chassis[0].ecd;
			motor_chassis[0].temperate=rx_data[6];
			motor_chassis[0].ecd=(rx_data[0]<<8|rx_data[1]);
			motor_chassis[0].speed_rpm=(rx_data[2]<<8|rx_data[3]);
			motor_chassis[0].given_current=(rx_data[4]<<8|rx_data[5]);
			motor_chassis[0].Torque=(float)(motor_chassis[0].given_current)/16384*20*0.3f/3591*187*268/17;
			motor_chassis[0].speed_rads=motor_chassis[0].speed_rpm*3.14159/30/268*17;
			Calc_Motor_Angle(&motor_chassis[0]);
			motor_chassis[0].last_angle=motor_chassis[0].angle;
			detect_hook(CHASSIS_MOTOR1_TOE);
			break;
		}
		case 0x202: //右轮
		{
			motor_chassis[1].last_ecd=motor_chassis[1].ecd;
			motor_chassis[1].temperate=rx_data[6];
			motor_chassis[1].ecd=(rx_data[0]<<8|rx_data[1]);
			motor_chassis[1].speed_rpm=(rx_data[2]<<8|rx_data[3]);
			motor_chassis[1].given_current=(rx_data[4]<<8|rx_data[5]);
			motor_chassis[1].Torque=(float)(motor_chassis[1].given_current)/16384*20*0.3f/3591*187*268/17;
			motor_chassis[1].speed_rads=motor_chassis[1].speed_rpm*3.14159/30/268*17;
			Calc_Motor_Angle(&motor_chassis[1]);
			motor_chassis[1].last_angle=motor_chassis[1].angle;
			detect_hook(CHASSIS_MOTOR2_TOE);
			break;
		}
		case 0x001: //右后关节
		{
			uint16_t Position_int=(int16_t)((uint16_t)rx_data[1]<<8|rx_data[2]);
			uint16_t Speed_int=(int16_t)((uint16_t)rx_data[3]<<4|rx_data[4]>>4);
			uint16_t Torque_int=(int16_t)((uint16_t)(rx_data[4] & 0x0F)<<8|rx_data[5]);
			
			Joint_motor[0].Status=(rx_data[0]>>4) & 0x0F;
			Joint_motor[0].Position=DM_J8009_uint_to_float(Position_int,DM_J8009_PMIN,DM_J8009_PMAX,16);
			Joint_motor[0].Speed=DM_J8009_uint_to_float(Speed_int,DM_J8009_VMIN,DM_J8009_VMAX,12);
			Joint_motor[0].Torque=DM_J8009_uint_to_float(Torque_int,DM_J8009_TMIN,DM_J8009_TMAX,12);
			detect_hook(CHASSIS_MOTOR3_TOE);
			break;
		}
		case 0x002: //右前关节
		{
			uint16_t Position_int=(int16_t)((uint16_t)rx_data[1]<<8|rx_data[2]);
			uint16_t Speed_int=(int16_t)((uint16_t)rx_data[3]<<4|rx_data[4]>>4);
			uint16_t Torque_int=(int16_t)((uint16_t)(rx_data[4] & 0x0F)<<8|rx_data[5]);
			
			Joint_motor[1].Status=(rx_data[0]>>4) & 0x0F;
			Joint_motor[1].Position=DM_J8009_uint_to_float(Position_int,DM_J8009_PMIN,DM_J8009_PMAX,16);
			Joint_motor[1].Speed=DM_J8009_uint_to_float(Speed_int,DM_J8009_VMIN,DM_J8009_VMAX,12);
			Joint_motor[1].Torque=DM_J8009_uint_to_float(Torque_int,DM_J8009_TMIN,DM_J8009_TMAX,12);
			detect_hook(CHASSIS_MOTOR4_TOE);
			break;
		}
		case 0x003: //左前关节
		{
			uint16_t Position_int=(int16_t)((uint16_t)rx_data[1]<<8|rx_data[2]);
			uint16_t Speed_int=(int16_t)((uint16_t)rx_data[3]<<4|rx_data[4]>>4);
			uint16_t Torque_int=(int16_t)((uint16_t)(rx_data[4] & 0x0F)<<8|rx_data[5]);
			
			Joint_motor[2].Status=(rx_data[0]>>4) & 0x0F;
			Joint_motor[2].Position=DM_J8009_uint_to_float(Position_int,DM_J8009_PMIN,DM_J8009_PMAX,16);
			Joint_motor[2].Speed=DM_J8009_uint_to_float(Speed_int,DM_J8009_VMIN,DM_J8009_VMAX,12);
			Joint_motor[2].Torque=DM_J8009_uint_to_float(Torque_int,DM_J8009_TMIN,DM_J8009_TMAX,12);
			detect_hook(CHASSIS_MOTOR5_TOE);
			break;
		}
		case 0x004: //左后关节
		{
			uint16_t Position_int=(int16_t)((uint16_t)rx_data[1]<<8|rx_data[2]);
			uint16_t Speed_int=(int16_t)((uint16_t)rx_data[3]<<4|rx_data[4]>>4);
			uint16_t Torque_int=(int16_t)((uint16_t)(rx_data[4] & 0x0F)<<8|rx_data[5]);
			
			Joint_motor[3].Status=(rx_data[0]>>4) & 0x0F;
			Joint_motor[3].Position=DM_J8009_uint_to_float(Position_int,DM_J8009_PMIN,DM_J8009_PMAX,16);
			Joint_motor[3].Speed=DM_J8009_uint_to_float(Speed_int,DM_J8009_VMIN,DM_J8009_VMAX,12);
			Joint_motor[3].Torque=DM_J8009_uint_to_float(Torque_int,DM_J8009_TMIN,DM_J8009_TMAX,12);
			detect_hook(CHASSIS_MOTOR6_TOE);
			break;
		}
		case 0x081: //来自云台板的消息控制底盘
		{
			TwoBoardControlGimbal.vx_set=(((int8_t)rx_data[0])*0.1f);//8位范围是-128~127，所以将（实际值*10）作为协议，实现正负值和保留一位精度
			TwoBoardControlGimbal.vy_set=(((int8_t)rx_data[1])*0.1f);
			TwoBoardControlGimbal.roll_set=((int8_t)(rx_data[2])*0.01f);
			TwoBoardControlGimbal.legL_mode=rx_data[3];
			TwoBoardControlGimbal.mode=rx_data[4];
			TwoBoardControlGimbal.relative_angle_yaw=(int32_t)((rx_data[5]<<24)|(rx_data[6]<<16)|(rx_data[7]<<8))*0.000001f;//将（实际值*1000000）作为协议，实现正负值和保留6位精度
			detect_hook(DBUS_TOE);
			break;
		}
		case 0x082: //来自云台板的消息控制底盘
		{
			TwoBoardControlGimbal.chassis_power_limit=rx_data[0]>>8|rx_data[1];
			TwoBoardControlGimbal.buffer_energy_chassis=rx_data[2]>>8|rx_data[3];
			TwoBoardControlGimbal.LeftLengthSet=(((int8_t)rx_data[4])*0.01f);
			TwoBoardControlGimbal.LeftPhi0Set=(((int8_t)rx_data[5])*0.1f);
			TwoBoardControlGimbal.RightLengthSet=(((int8_t)rx_data[6])*0.01f);
			TwoBoardControlGimbal.RightPhi0Set=(((int8_t)rx_data[7])*0.1f);
			detect_hook(DBUS_TOE);
			break;
		}
		case 0x051: //超级电容控制板消息
		{
			SuperCap.errorCode=rx_data[0];
			uint8_t power_bytes[4] = {rx_data[1], rx_data[2], rx_data[3], rx_data[4]};
      memcpy(&SuperCap.chassisPower, power_bytes, 4);//需要调用<string.h>
			SuperCap.chassisPowerLimit = (uint16_t)rx_data[5] | ((uint16_t)rx_data[6] << 8);
			SuperCap.capEnergy=rx_data[7];
			break;
		}
		
		default:
		{
			break;
		}
	}
}

//底盘给云台发消息
void chassisToGimbal(void)
{
  Header.Identifier=0x031;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  ChaToGim_can_send_data[0] = (int8_t)(chassis_move.Left_Leg.leg_phi0*10);
  ChaToGim_can_send_data[1] = (int8_t)(chassis_move.Left_Leg.leg_L*100);
  ChaToGim_can_send_data[2] = (int8_t)(chassis_move.Right_Leg.leg_phi0*10);
  ChaToGim_can_send_data[3] = (int8_t)(chassis_move.Right_Leg.leg_L*100);
  ChaToGim_can_send_data[4] = chassis_move.chassis_state;
  ChaToGim_can_send_data[5] = 0x00;
  ChaToGim_can_send_data[6] = 0x00;
  ChaToGim_can_send_data[7] = 0x00;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3,&Header,ChaToGim_can_send_data);
}

//数据限幅
float Data_Clipping(float Data,float Data_Min,float Data_Max)
{
	if(Data>Data_Max)return Data_Max;
	else if(Data<Data_Min)return Data_Min;
	return Data;
}

//根据DM_J8009通信协议，将整型uint转换成浮点数float
static float DM_J8009_uint_to_float(int x,float x_min,float x_max,int bits)
{
	float span=x_max-x_min;
	float offset=x_min;
	return (float)(((float)x)*span/((float)(1<<bits)-1)+offset);
}

//根据DM_J8009通信协议，将浮点数float转换成整型uint
uint16_t DM_J8009_float_to_uint(float x,float x_min,float x_max,int bits)
{
	float span=x_max-x_min;
	float offset=x_min;
	return (uint16_t)((x-offset)/span*((float)((1<<bits)-1)));
}

//发送关节电机扭矩
extern void CAN_cmd_joint1(float Tor) 
{
  Header.Identifier=0x01;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  
  Tor=Data_Clipping(Tor,DM_J8009_TMIN,DM_J8009_TMAX);
  uint16_t Torque=DM_J8009_float_to_uint(Tor,DM_J8009_TMIN,DM_J8009_TMAX,12);
	
  chassis_can_send_data[0] = 0x00;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = 0x00;
  chassis_can_send_data[5] = 0x00;
  chassis_can_send_data[6] = Torque >> 8;
  chassis_can_send_data[7] = Torque & 0x00FF;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&Header,chassis_can_send_data);
}

//发送关节电机扭矩
extern void CAN_cmd_joint2(float Tor) 
{
  Header.Identifier=0x02;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  
  Tor=Data_Clipping(Tor,DM_J8009_TMIN,DM_J8009_TMAX);
  uint16_t Torque=DM_J8009_float_to_uint(Tor,DM_J8009_TMIN,DM_J8009_TMAX,12);
	
  chassis_can_send_data[0] = 0x00;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = 0x00;
  chassis_can_send_data[5] = 0x00;
  chassis_can_send_data[6] = Torque >> 8;
  chassis_can_send_data[7] = Torque;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&Header,chassis_can_send_data);
}

//发送关节电机扭矩
extern void CAN_cmd_joint3(float Tor) 
{
  Header.Identifier=0x03;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
	
  Tor=Data_Clipping(Tor,DM_J8009_TMIN,DM_J8009_TMAX);
  uint16_t Torque=DM_J8009_float_to_uint(Tor,DM_J8009_TMIN,DM_J8009_TMAX,12);

  chassis_can_send_data[0] = 0x00;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = 0x00;
  chassis_can_send_data[5] = 0x00;
  chassis_can_send_data[6] = Torque >> 8;
  chassis_can_send_data[7] = Torque;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&Header,chassis_can_send_data);
}

//发送关节电机扭矩
extern void CAN_cmd_joint4(float Tor) 
{
  Header.Identifier=0x04;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
	
  Tor=Data_Clipping(Tor,DM_J8009_TMIN,DM_J8009_TMAX);
  uint16_t Torque=DM_J8009_float_to_uint(Tor,DM_J8009_TMIN,DM_J8009_TMAX,12);
	
  chassis_can_send_data[0] = 0x00;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = 0x00;
  chassis_can_send_data[5] = 0x00;
  chassis_can_send_data[6] = Torque >> 8;
  chassis_can_send_data[7] = Torque;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&Header,chassis_can_send_data);
}

extern void CAN_cmd_wheel_right(int16_t current)
{
  Header.Identifier=0x200;//2号
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  chassis_can_send_data[0] = 0x00;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = current >> 8;
  chassis_can_send_data[3] = current;
  chassis_can_send_data[4] = 0x00;
  chassis_can_send_data[5] = 0x00;
  chassis_can_send_data[6] = 0x00;
  chassis_can_send_data[7] = 0x00;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&Header,chassis_can_send_data);
}

extern void CAN_cmd_wheel_left(int16_t current)
{
  Header.Identifier=0x200;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  chassis_can_send_data[0] = current >> 8;
  chassis_can_send_data[1] = current;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = 0x00;
  chassis_can_send_data[5] = 0x00;
  chassis_can_send_data[6] = 0x00;
  chassis_can_send_data[7] = 0x00;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&Header,chassis_can_send_data);
}

//获取轮毂电机指针
const motor_measure_t *get_wheel_left_measure_point(void)
{
  return &motor_chassis[0];
}

//获取轮毂电机指针
const motor_measure_t *get_wheel_right_measure_point(void)
{
  return &motor_chassis[1];
}
//获取关节电机指针
const Joint_Motor_t *get_joint_right_back_measure_point(void)
{
  return &Joint_motor[0];
}
//获取关节电机指针
const Joint_Motor_t *get_joint_right_ahead_measure_point(void)
{
  return &Joint_motor[1];
}
//获取关节电机指针
const Joint_Motor_t *get_joint_left_ahead_measure_point(void)
{
  return &Joint_motor[2];
}
//获取关节电机指针
const Joint_Motor_t *get_joint_left_back_measure_point(void)
{
  return &Joint_motor[3];
}

//计算轮毂电机旋转的角度
void Calc_Motor_Angle(motor_measure_t *motor)
{

	if (motor->ecd - motor->last_ecd > 4095.5)
	{
		motor->round--;
	}
	else if (motor->ecd - motor->last_ecd < -4095.5)
	{
		motor->round++;
	}	

	motor->angle = (float)(motor->round * ANGLE_T + motor->ecd)/ANGLE_T*3.1415926f*2/268*17;
}

//关节电机1初始化
static void CAN_cmd_joint1_Init(void) 
{
  Header.Identifier=0x01;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  
  chassis_can_send_data[0] = 0xFF;
  chassis_can_send_data[1] = 0xFF;
  chassis_can_send_data[2] = 0xFF;
  chassis_can_send_data[3] = 0xFF;
  chassis_can_send_data[4] = 0xFF;
  chassis_can_send_data[5] = 0xFF;
  chassis_can_send_data[6] = 0xFF;
  chassis_can_send_data[7] = 0xFC;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&Header,chassis_can_send_data);
}

static void CAN_cmd_joint1_enable(void) 
{
  Header.Identifier=0x01;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  
  chassis_can_send_data[0] = 0xFF;
  chassis_can_send_data[1] = 0xFF;
  chassis_can_send_data[2] = 0xFF;
  chassis_can_send_data[3] = 0xFF;
  chassis_can_send_data[4] = 0xFF;
  chassis_can_send_data[5] = 0xFF;
  chassis_can_send_data[6] = 0xFF;
  chassis_can_send_data[7] = 0xFB;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&Header,chassis_can_send_data);
}

//关节电机2初始化
static void CAN_cmd_joint2_Init(void) 
{
  Header.Identifier=0x02;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  
  chassis_can_send_data[0] = 0xFF;
  chassis_can_send_data[1] = 0xFF;
  chassis_can_send_data[2] = 0xFF;
  chassis_can_send_data[3] = 0xFF;
  chassis_can_send_data[4] = 0xFF;
  chassis_can_send_data[5] = 0xFF;
  chassis_can_send_data[6] = 0xFF;
  chassis_can_send_data[7] = 0xFC;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&Header,chassis_can_send_data);
}

static void CAN_cmd_joint2_enable(void) 
{
  Header.Identifier=0x02;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  
  chassis_can_send_data[0] = 0xFF;
  chassis_can_send_data[1] = 0xFF;
  chassis_can_send_data[2] = 0xFF;
  chassis_can_send_data[3] = 0xFF;
  chassis_can_send_data[4] = 0xFF;
  chassis_can_send_data[5] = 0xFF;
  chassis_can_send_data[6] = 0xFF;
  chassis_can_send_data[7] = 0xFB;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&Header,chassis_can_send_data);
}

//关节电机3初始化
static void CAN_cmd_joint3_Init(void) 
{
  Header.Identifier=0x03;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  
  chassis_can_send_data[0] = 0xFF;
  chassis_can_send_data[1] = 0xFF;
  chassis_can_send_data[2] = 0xFF;
  chassis_can_send_data[3] = 0xFF;
  chassis_can_send_data[4] = 0xFF;
  chassis_can_send_data[5] = 0xFF;
  chassis_can_send_data[6] = 0xFF;
  chassis_can_send_data[7] = 0xFC;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&Header,chassis_can_send_data);
}

static void CAN_cmd_joint3_enable(void) 
{
  Header.Identifier=0x03;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  
  chassis_can_send_data[0] = 0xFF;
  chassis_can_send_data[1] = 0xFF;
  chassis_can_send_data[2] = 0xFF;
  chassis_can_send_data[3] = 0xFF;
  chassis_can_send_data[4] = 0xFF;
  chassis_can_send_data[5] = 0xFF;
  chassis_can_send_data[6] = 0xFF;
  chassis_can_send_data[7] = 0xFB;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&Header,chassis_can_send_data);
}

//关节电机4初始化
static void CAN_cmd_joint4_Init(void) 
{
  Header.Identifier=0x04;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  
  chassis_can_send_data[0] = 0xFF;
  chassis_can_send_data[1] = 0xFF;
  chassis_can_send_data[2] = 0xFF;
  chassis_can_send_data[3] = 0xFF;
  chassis_can_send_data[4] = 0xFF;
  chassis_can_send_data[5] = 0xFF;
  chassis_can_send_data[6] = 0xFF;
  chassis_can_send_data[7] = 0xFC;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&Header,chassis_can_send_data);
}

static void CAN_cmd_joint4_enable(void) 
{
  Header.Identifier=0x04;
  Header.IdType = FDCAN_STANDARD_ID;
  Header.TxFrameType = FDCAN_DATA_FRAME;
  Header.DataLength = 8;
  Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  Header.BitRateSwitch = FDCAN_BRS_OFF;
  Header.FDFormat =  FDCAN_CLASSIC_CAN;           
  Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  Header.MessageMarker = 0;
  
  chassis_can_send_data[0] = 0xFF;
  chassis_can_send_data[1] = 0xFF;
  chassis_can_send_data[2] = 0xFF;
  chassis_can_send_data[3] = 0xFF;
  chassis_can_send_data[4] = 0xFF;
  chassis_can_send_data[5] = 0xFF;
  chassis_can_send_data[6] = 0xFF;
  chassis_can_send_data[7] = 0xFB;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&Header,chassis_can_send_data);
}

//关节电机初始化
void Joint_motor_Init(void)
{
	//电机使能启动
	if(Joint_motor[0].Status==0x00||Joint_motor[1].Status==0x00||Joint_motor[2].Status==0x00||Joint_motor[3].Status==0x00)
	{
	CAN_cmd_joint1_Init();
	CAN_cmd_joint3_Init();
	HAL_Delay(20);//电机是回环式的，一发一收，所以需要一点点延时
	CAN_cmd_joint2_Init();
	CAN_cmd_joint4_Init();
	}
	//电机CANTimeout时使能启动
	else if(Joint_motor[0].Status==0x0D||Joint_motor[1].Status==0x0D||Joint_motor[2].Status==0x0D||Joint_motor[3].Status==0x0D)
	{
	CAN_cmd_joint1_enable();
	CAN_cmd_joint3_enable();
	HAL_Delay(20);
	CAN_cmd_joint2_enable();
	CAN_cmd_joint4_enable();
	}
	else
	{
	CAN_cmd_joint1(0);
	CAN_cmd_joint3(0);
	HAL_Delay(20);
	CAN_cmd_joint2(0);
	CAN_cmd_joint4(0);
	}
}
