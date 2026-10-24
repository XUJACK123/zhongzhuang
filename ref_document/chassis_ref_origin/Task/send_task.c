//主要是给关节四个电机发消息，我一开始旋转的是MIT模式，所以变成了这样，也许用一拖四模式会好一点

#include "send_task.h"
#include "detect_task.h"
#include "Freertos.h"
#include "task.h"
#include "CAN_receive.h"
#include "chassis_task.h"
#include "USART7_Receive.h"
#include "user_lib.h"
#include "Image_Trans_Receive.h"

extern chassis_move_t chassis_move;//底盘运动数据
static uint8_t SendFrequencyflag=0;
void send_task(void const *pvParameters)
{
  //空闲一段时间
    vTaskDelay(610);
	
	while(1)
	{			
			user_send_data();//无线串口发送数据，用于debug
			SendFrequencyflag++;
			if(SendFrequencyflag>=2)
			{
			SendFrequencyflag=0;
			chassisToGimbal();//给上云台发消息
			}
	//关节电机掉线时电机电流置零
	if(toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(CHASSIS_MOTOR5_TOE) || toe_is_error(CHASSIS_MOTOR6_TOE))
	{
		CAN_cmd_joint1(0);
		CAN_cmd_joint3(0);
		My_delay_us(200);
		CAN_cmd_joint2(0);
		CAN_cmd_joint4(0);
		vTaskDelay(1);
	}
		//当遥控器掉线的时候，发送给底盘电机零电流.
		if (toe_is_error(DBUS_TOE))
		{
			CAN_cmd_joint1(0);
			CAN_cmd_joint3(0);
			My_delay_us(200);//看别人帖子提到来回时间是126us，因为实际freertos的1ms也没有跑满，所以这里改为硬等200us，后续优化从这里下手
			CAN_cmd_joint2(0);
			CAN_cmd_joint4(0);
			vTaskDelay(1);
		}
		else
		{
			if(chassis_move.chassis_mode==CHASSIS_ZERO_FORCE)
			{
				
			CAN_cmd_joint1(0);
			CAN_cmd_joint3(0);
			My_delay_us(200);
			CAN_cmd_joint2(0);
			CAN_cmd_joint4(0);
			vTaskDelay(1);
			}
			else
			{				
//			CAN_cmd_joint1(0);
//			CAN_cmd_joint3(0);
//			My_delay_us(200);
//			CAN_cmd_joint2(0);
//			CAN_cmd_joint4(0);
//			vTaskDelay(1);
//				
			CAN_cmd_joint1(chassis_move.Joint_Right_Back.set_Tp);//chassis_move.Joint_Right_Back.set_Tp
			CAN_cmd_joint3(chassis_move.Joint_Left_Ahead.set_Tp);//chassis_move.Joint_Left_Ahead.set_Tp
			My_delay_us(200);
			CAN_cmd_joint2(chassis_move.Joint_Right_Ahead.set_Tp);//chassis_move.Joint_Right_Ahead.set_Tp
			CAN_cmd_joint4(chassis_move.Joint_Left_Back.set_Tp);//chassis_move.Joint_Left_Back.set_Tp
			vTaskDelay(1);
			}
			
		}
		
	}
	
}
