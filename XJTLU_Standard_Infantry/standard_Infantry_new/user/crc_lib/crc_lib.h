/**
  ****************************(C) COPYRIGHT FREE********************************
  * @file       refree_receive.c/h
  * @brief      裁判系统数据处理，通过USART3串口收发裁判系统数据，利用DMA传输方式节
	*							约CPU资源，利用串口空闲中断来拉起处理函数。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            	Author          Modification
  *  V1.0.0     July-10-2019     Ziqi Yang          1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT FREE********************************
  */
#ifndef CRC_LIB_H
#define CRC_LIB_H
#include "main.h"

//CRC校验、获取
extern uint8_t ref_get_crc8(uint8_t *p_msg, unsigned int len, uint8_t crc8);
extern uint32_t ref_verify_crc8(uint8_t *p_msg, unsigned int len);
extern uint16_t ref_get_crc16(uint8_t *p_msg, uint16_t len, uint16_t crc16);
extern uint32_t ref_verify_crc16(uint8_t *p_msg, uint16_t len);

extern const uint8_t ref_crc8_init;
extern const uint8_t ref_crc8_tab[256];
extern const uint16_t ref_crc16_init;
extern const uint16_t ref_crc16_tab[256];

#endif
