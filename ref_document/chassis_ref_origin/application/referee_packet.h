#ifndef REFEREE_PACKET_H
#define REFEREE_PACKET_H

#include "main.h"

#pragma pack(push, 1)//强调不需要编译器自动补齐

typedef struct {
    uint8_t sof;            // 起始字节 0xA5
    uint16_t data_length;   // data长度
    uint8_t seq;            // 包序号
    uint8_t crc8;           // 帧头CRC8校验
} FrameHeader_t;				// 帧头结构

typedef struct {
    FrameHeader_t header;     // 5字节帧头
    uint16_t cmd_id;      	  // 命令码ID
    uint8_t *data;        	  // 数据段（动态分配）
    uint16_t crc16;       	  // 整包CRC16校验
} RefereeSystemFrame_t;			// 完整帧结构

typedef struct{
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_1_barrel_heat; 
}ReceiveReferee_t;
#pragma pack(pop)//恢复编译器正常补齐状态

extern unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
extern unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
extern void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
extern uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
extern uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
extern void append_CRC16_check_sum(uint8_t * pchMessage,uint32_t dwLength);
#endif
