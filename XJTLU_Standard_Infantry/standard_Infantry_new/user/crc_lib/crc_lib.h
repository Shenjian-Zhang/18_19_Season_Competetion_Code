/**
  ****************************(C) COPYRIGHT FREE********************************
  * @file       refree_receive.c/h
  * @brief      ����ϵͳ���ݴ���ͨ��USART3�����շ�����ϵͳ���ݣ�����DMA���䷽ʽ��
	*							ԼCPU��Դ�����ô��ڿ����ж�������������
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            	Author          Modification
  *  V1.0.0     July-10-2019     Ziqi Yang          1. ���
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

//CRCУ�顢��ȡ
extern uint8_t ref_get_crc8(uint8_t *p_msg, unsigned int len, uint8_t crc8);
extern uint32_t ref_verify_crc8(uint8_t *p_msg, unsigned int len);
extern uint16_t ref_get_crc16(uint8_t *p_msg, uint16_t len, uint16_t crc16);
extern uint32_t ref_verify_crc16(uint8_t *p_msg, uint16_t len);

extern const uint8_t ref_crc8_init;
extern const uint8_t ref_crc8_tab[256];
extern const uint16_t ref_crc16_init;
extern const uint16_t ref_crc16_tab[256];

#endif
