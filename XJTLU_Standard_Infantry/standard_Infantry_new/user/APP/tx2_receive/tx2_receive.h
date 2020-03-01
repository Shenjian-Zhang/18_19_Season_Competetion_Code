/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       tx2_receive.c/h
  * @brief      �Ӿ��������ݴ���ͨ��USART6���ڽ����Ӿ��������ݣ�����DMA���䷽ʽ��
	*							ԼCPU��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA��
	*							���ڵķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            	Author          Modification
  *  V1.0.0     June-30-2019     Ziqi Yang          1. ���
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

//DMA�����������С
#define VISION_RX_BUF_NUM 18u

//�Ӿ�δ����Ŀ�귵��ֵ
#define VISION_NO_TARGET_VALUE 9999

//�Ӿ����ݰ�����
#define VISION_PROTOCOL_HEADER                 0xA5
#define VISION_PROTOCOL_HEADER_SIZE            1
#define VISION_PROTOCOL_CRC16_SIZE             2
#define VISION_PROTOCOL_DATA_SIZE							 15
#define VISION_PROTOCOL_FRAME_MAX_SIZE            (VISION_PROTOCOL_HEADER_SIZE + VISION_PROTOCOL_CRC16_SIZE + VISION_PROTOCOL_DATA_SIZE) 

/* ----------------------- Data Struct ------------------------------------- */
//�Ӿ����ݰ�У�鲽��
typedef enum
{
		VISION_STEP_HEADER_SOF  = 0,
		VISION_STEP_DATA_CRC16  = 1,
} vision_unpack_step_e;

//�Ӿ�����2�ֽ�16����ת10�������ݹ�����
union vision_2_byte_t
{
		int16_t d;
		unsigned char buf[2];
};

//�Ӿ����ݰ��ṹ��
typedef __packed struct
{
		uint8_t        protocol_packet[VISION_PROTOCOL_FRAME_MAX_SIZE];
		vision_unpack_step_e  unpack_step;
		uint16_t       index;
} vision_unpack_data_t;

//�Ӿ������ǶȽṹ��
typedef __packed struct
{
		fp32 yaw;
		fp32 pitch;
} Gimbal_Angle_t;

//�Ӿ����ݽṹ��
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
