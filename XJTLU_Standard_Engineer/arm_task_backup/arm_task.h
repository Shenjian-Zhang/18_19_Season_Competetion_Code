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
#ifndef ARM_TASK_H
#define ARM_TASK_H
#include "main.h"
#include "stm32f4xx.h"
#include "CAN_Receive.h"
#include "Remote_Control.h"
#include "pid.h"
#include "led.h"

#define ARM_CONTROL_TIME 2 //У׼���������е����� Ϊ1ms

#define get_remote_ctrl_point_arm() get_remote_control_point() //��ȡң�����Ľṹ��ָ��

#define ARM_ECD_PID_KP 0.3f
#define ARM_ECD_PID_KI 0.0f
#define ARM_ECD_PID_KD 0.0f
#define ARM_ECD_PID_MAX_OUT 5000.0f
#define ARM_ECD_PID_MAX_IOUT 2000.0f

#define ARM_SPEED_PID_KP 6.0f
#define ARM_SPEED_PID_KI 0.0f
#define ARM_SPEED_PID_KD 0.0f
#define ARM_SPEED_PID_MAX_OUT 6000.0f
#define ARM_SPEED_PID_MAX_IOUT 2000.0f

#define ARM_ECD_TARGET 8192*19/2
#define ARM_ECD_LEN 25000

typedef struct
{
  const motor_measure_t *arm_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} Arm_Motor_t;

typedef struct
{
  //Arm_Motor_t motor_arm;          //���̵������
	PidTypeDef arm_ecd_pid;             //���̵���ٶ�pid
  PidTypeDef arm_speed_pid;             //���̵���ٶ�pid
} arm_move_t;

//���ÿ�������
extern void arm_task(void *pvParameters);
void arm_init(void);

extern motor_measure_t motor_arm;
#endif
