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
#ifndef REFREE_SYSTEM_H
#define REFREE_SYSTEM_H
#include "main.h"
#include "crc_lib.h"
#include "string.h"
#include "delay.h"

//DMA�����������С
#define REFREE_RX_BUF_NUM 						256u
#define REFREE_TX_BUF_NUM 						28u

//����ϵͳ���ݰ�
#define REF_PROTOCOL_HEADER                 0xA5
#define REF_PROTOCOL_HEADER_SIZE            5
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_PROTOCOL_FRAME_MAX_SIZE					128

//����ϵͳcmd_id
#define ROBOT_LIVE_CMDID					0x0003
#define ROBOT_STATE_CMDID					0x0201
#define ROBOT_POWER_HEAT_CMDID		0x0202
#define ROBOT_SHOOT_CMDID					0x0207

//�ͻ����Զ������ݻ�����λ��
#define CLIENT_CUSTOM_VALUE_1_BUF_NUM		13
#define CLIENT_CUSTOM_VALUE_2_BUF_NUM		17
#define CLIENT_CUSTOM_VALUE_3_BUF_NUM		21

//�ͻ����Զ�����������Ԫ��
typedef enum
{
		CLIENT_CUSTOM_VALUE_1 = 0,
		CLIENT_CUSTOM_VALUE_2,
		CLIENT_CUSTOM_VALUE_3
} client_custom_value_t;

//����ϵͳ���ݰ�У�鲽��
typedef enum
{
		STEP_HEADER_SOF  = 0,
		STEP_LENGTH_LOW  = 1,
		STEP_LENGTH_HIGH = 2,
		STEP_FRAME_SEQ   = 3,
		STEP_HEADER_CRC8 = 4,
		STEP_DATA_CRC16  = 5,
} unpack_step_e;

//����ϵͳ4�ֽ�16����ת10���Ƹ������ݹ�����
union refree_4_byte_t
{
		float f;
		unsigned char buf[4];
};

//����ϵͳ�ͻ���ָʾ�ƹ�����
typedef struct
{
		uint8_t buf;
		unsigned char marks[6];
} client_custom_marks_t;

//����ϵͳ���ݰ��ṹ��
typedef __packed struct
{
		uint16_t       data_len;
		uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
		unpack_step_e  unpack_step;
		uint16_t       index;
} unpack_data_t;

//�����˴��״̬����
typedef __packed struct
{
		uint16_t robot_legion;
} ext_game_robot_survivors_t;

//������״̬����
typedef __packed struct
{
		uint8_t robot_id;
		uint8_t robot_level;
		uint16_t remain_HP;
		uint16_t max_HP;
		uint16_t shooter_heat0_cooling_rate;
		uint16_t shooter_heat0_cooling_limit;
		uint16_t shooter_heat1_cooling_rate;
		uint16_t shooter_heat1_cooling_limit;
		uint8_t mains_power_gimbal_output : 1; 
		uint8_t mains_power_chassis_output : 1;
		uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

//������ʵʱ������������
typedef __packed struct
{
		float chassis_volt; 
		float chassis_current; 
		float chassis_power; 
		uint16_t chassis_power_buffer; 
		uint16_t shooter_heat0; 
		uint16_t shooter_heat1; 
} ext_power_heat_data_t;

//������ʵʱ�������
typedef __packed struct
{ 
		uint8_t bullet_type; 
		uint8_t bullet_freq; 
		float bullet_speed; 
} ext_shoot_data_t;

//����ϵͳ�����ṹ��
typedef __packed struct
{
		uint16_t cmd_id;
		ext_game_robot_survivors_t robot_survive;
		ext_game_robot_state_t robot_state;
		ext_power_heat_data_t robot_power_heat;
		ext_shoot_data_t robot_shoot;
} refree_feedback_t;

//�ͻ����Զ������ݽṹ��
typedef struct
{
		union refree_4_byte_t custom_value[3];
		client_custom_marks_t custom_mark;
} client_custom_data_t;

//����ϵͳ���ݽṹ��
typedef struct
{
		refree_feedback_t 				feedback;
		client_custom_data_t			client;
} refree_system_t;

extern void Refree_Init(void);
extern refree_system_t *get_refree_system_data_point(void);
extern void set_client_custom_value(refree_system_t *set_refree_client_custom_value, client_custom_value_t custom_value_pos, float custom_data);
extern void set_client_custom_mark(refree_system_t *set_refree_client_custom_mark, uint16_t mark_pos, uint16_t mark_state);

#endif
