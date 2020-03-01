/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       tx2_receive.c/h
  * @brief      视觉反馈数据处理，通过USART6串口接收视觉反馈数据，利用DMA传输方式节
	*							约CPU资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，
	*							串口的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            	Author          Modification
  *  V1.0.0     June-30-2019     Ziqi Yang          1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef TX2_RECEIVE_H
#define TX2_RECEIVE_H
#include "main.h"
#include "crc_lib.h"
#include "user_lib.h"
#include "stdlib.h"
#include "string.h"

//DMA缓冲区数组大小
#define VISION_RX_BUF_NUM 18u

//视觉未发现目标返回值
#define VISION_NO_TARGET_VALUE 9999

//视觉数据包常数
#define VISION_PROTOCOL_HEADER                 0xA5
#define VISION_PROTOCOL_HEADER_SIZE            1
#define VISION_PROTOCOL_CRC16_SIZE             2
#define VISION_PROTOCOL_DATA_SIZE							 15
#define VISION_PROTOCOL_FRAME_MAX_SIZE            (VISION_PROTOCOL_HEADER_SIZE + VISION_PROTOCOL_CRC16_SIZE + VISION_PROTOCOL_DATA_SIZE) 

/* ----------------------- Data Struct ------------------------------------- */
//视觉数据包校验步骤
typedef enum
{
		VISION_STEP_HEADER_SOF  = 0,
		VISION_STEP_DATA_CRC16  = 1,
} vision_unpack_step_e;

//视觉数据2字节16进制转10进制数据共用体
union vision_2_byte_t
{
		int16_t d;
		unsigned char buf[2];
};

//视觉数据包结构体
typedef __packed struct
{
		uint8_t        protocol_packet[VISION_PROTOCOL_FRAME_MAX_SIZE];
		vision_unpack_step_e  unpack_step;
		uint16_t       index;
} vision_unpack_data_t;

//视觉反馈角度结构体
typedef __packed struct
{
		fp32 yaw;
		fp32 pitch;
} Gimbal_Angle_t;

//视觉数据结构体
typedef __packed struct
{
		uint8_t 				unpack_data[VISION_PROTOCOL_FRAME_MAX_SIZE];
		Gimbal_Angle_t 	difference_angle;
} vision_receive_t;

/* ----------------------- Internal Data ----------------------------------- */

extern void Vision_Init(void);
extern const vision_receive_t *get_vision_receive_point(void);

extern void vision_unable(void);

#endif
