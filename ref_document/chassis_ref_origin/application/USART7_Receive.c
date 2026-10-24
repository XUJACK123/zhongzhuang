//这个文件是我通过无线串口给电脑发消息用作调试，也可以随便改改当个串口用

#include "USART7_Receive.h"
#include "bsp_usart7.h"
#include <string.h>
#include "chassis_task.h"
#include "INS_task.h"
#include "remote_control.h"

#define SET_BUF_LENGTH sizeof(user_receive_t)*2
#define RECEIVE_LENGTH sizeof(user_receive_t)

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_rx;

__attribute__((section (".AXI_SRAM"))) uint8_t Receive_data[2][SET_BUF_LENGTH];//见COD王工开源，原理就是DMA2不能访问地址0x20000000
user_send_t user_send;
user_receive_t user_receive;
void receive_data_solve(volatile const uint8_t *buf, user_receive_t *user_send);

extern chassis_move_t chassis_move;
extern INS_data_t INS_data;
extern RC_ctrl_t rc_ctrl;
extern TwoBoardControlGimbal_t TwoBoardControlGimbal;
extern float Leg_L;
extern float LeftSpringForceForV;
extern float LeftSpringForceForTp;
extern float RightSpringForceForV;
extern float RightSpringForceForTp;
extern SuperCap_t SuperCap;

extern float LeftGasSpringForceForV; 
extern float RightGasSpringForceForV; 

void USART7_receive_init(void)
{
	UART7_DMA_init( Receive_data[0], Receive_data[1], SET_BUF_LENGTH);
}

void My_UART7_IRQHandler(void)
{
       static uint16_t this_time_rx_len = 0;
        if ((((DMA_Stream_TypeDef *)huart7.hdmarx->Instance)->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_uart7_rx);

            // get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SET_BUF_LENGTH - ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->NDTR;

            // reset set_data_lenght
            //重新设定数据长度
            ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->NDTR = SET_BUF_LENGTH;

            // set memory buffer 1
            //设定缓冲区1
            ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->CR |= DMA_SxCR_CT;

            // enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_uart7_rx);

            if (this_time_rx_len == RECEIVE_LENGTH)
            {
				receive_data_solve(Receive_data[0],&user_receive);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_uart7_rx);

            // get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SET_BUF_LENGTH - ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->NDTR;

            // reset set_data_lenght
            //重新设定数据长度
            ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->NDTR = SET_BUF_LENGTH;

            // set memory buffer 0
            //设定缓冲区0
            ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_uart7_rx);

            if (this_time_rx_len == RECEIVE_LENGTH)
            {
				receive_data_solve(Receive_data[1],&user_receive);
            }
        }
}

void receive_data_solve(volatile const uint8_t *buf, user_receive_t *user_receive)
{
	user_receive->data1=(buf[0]<<8|buf[1]);

}
float data[11];
uint8_t Datacount=0;
void user_send_data(void)
{
	__attribute__((section (".AXI_SRAM"))) static uint8_t tx_buf[sizeof(user_send_t)];

	data[0]=chassis_move.Joint_Left_Ahead.chassis_motor_measure->Torque;
	data[1]=chassis_move.Joint_Left_Back.chassis_motor_measure->Torque;
	data[2]=chassis_move.Joint_Right_Ahead.chassis_motor_measure->Torque;
	data[3]=chassis_move.Joint_Right_Back.chassis_motor_measure->Torque;
	data[4]=chassis_move.Right_Leg.Tp;
	data[5]=chassis_move.Left_Leg.Tp;
	data[6]=chassis_move.Right_Leg.leg_d_phi0;
	data[7]=chassis_move.Left_Leg.leg_d_phi0;
	data[8]=chassis_move.Right_Leg.leg_phi0;
	data[9]=chassis_move.chassis_climb_stage;
	data[10]=chassis_move.ClimbFlag;
	 
	memcpy(tx_buf ,(uint8_t *)&data, sizeof(data));
	tx_buf[44]=0x00;
	tx_buf[45]=0x00;
	tx_buf[46]=0x80;
	tx_buf[47]=0x7F;
//	data[1]=sin(2*t);
//	data[2]=sin(3*t);f
//	data[3]=sin(4*t);
//	HAL_UART_Transmit_DMA(&huart7,(uint8_t *)data,sizeof(float)*4);
//	HAL_UART_Transmit(&huart7,(uint8_t *)data,sizeof(float)*4,100);
//	HAL_UART_Transmit_DMA(&huart7,(uint8_t *)tail,sizeof(float)*4);

//	tx_buf[0]=
	HAL_UART_Transmit_DMA(&huart7,tx_buf,sizeof(user_send_t));
}
