/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "crc16.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;

// void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
// {
//     send_data.enemy_color = enemy_color;
//     send_data.work_mode = work_mode;
//     send_data.bullet_speed = bullet_speed;
// }

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    if (recv_len < 14) return;                                                                                       
                                                                                                                      
    uint16_t expected_crc = crc_16(vis_recv_buff, 12);                                                               
    uint16_t received_crc = vis_recv_buff[12] | (vis_recv_buff[13] << 8);                                            
                                                                                                                    
    if (expected_crc != received_crc) {                                                                              
        // 数据坏了，直接丢弃这一帧                                                                                  
        return;                                                                                                      
    } 

    if(recv_len >= 12)
    {
        memcpy(&recv_data.yaw,   vis_recv_buff,      4);                                                             
        memcpy(&recv_data.pitch, vis_recv_buff + 4,  4);                                                             
        memcpy(&recv_data.roll,  vis_recv_buff + 8,  4);
    }
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    memcpy(send_buff,      &send_data.yaw,   4);                                                                     
    memcpy(send_buff + 4,  &send_data.pitch, 4);                                                                     
    memcpy(send_buff + 8,  &send_data.roll,  4); 
    uint16_t crc = crc_16(send_buff, 12);                                                                            
    send_buff[12] = crc & 0xff;                                                                                      
    send_buff[13] = (crc >> 8) & 0xff;                                                                               
    USBTransmit(send_buff, 14); 
    USBTransmit(send_buff, VISION_SEND_SIZE);
}

#endif // VISION_USE_VCP
