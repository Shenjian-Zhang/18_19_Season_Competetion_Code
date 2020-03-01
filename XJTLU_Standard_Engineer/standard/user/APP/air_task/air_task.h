/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       Calibrate_task.c/h
  * @brief      ���У׼����豸�����ݣ�������̨�������ǣ����ٶȼƣ�������
  *             ��̨У׼��Ҫ����ֵ�������С��ԽǶȣ���������ҪУ׼��Ư
  *             ���ٶȼƺʹ�����ֻ��д�ýӿں��������ٶȼ�Ŀǰû�б�ҪУ׼��
  *             ��������δʹ���ڽ����㷨�С�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef AIR_TASK_H
#define AIR_TASK_H
#include "main.h"
#include "stm32f4xx.h"

#define AIR_CONTROL_TIME 1 //У׼���������е����� Ϊ1ms

#define get_remote_ctrl_point_air() get_remote_control_point() //��ȡң�����Ľṹ��ָ��

//���ÿ�������
extern void air_task(void *pvParameters);
void air_configuration(void);
#endif
