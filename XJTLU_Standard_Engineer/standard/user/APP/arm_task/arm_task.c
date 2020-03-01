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

//����У׼�豸�����֣�У׼��ʶ����У׼����flash��С��У׼�����Ӧ��У׼���ݵ�ַ
#include "arm_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t arm_task_stack;
#endif

arm_move_t arm_move;

void arm_task(void *pvParameters)
{
    //����һ��ʱ��
    vTaskDelay(ARM_TASK_INIT_TIME);	
		//��е�۳�ʼ��
		arm_init(&arm_move);

		while (1)
		{
				lock_state_update(&arm_move);
				if(arm_move.arm_lock == ARM_LOCKED)
				{
						if(arm_move.arm_RC->key.v & KEY_PRESSED_OFFSET_F)
						{
								if(arm_move.motor_arm.arm_motor_measure->total_ecd <= (int32_t)(ARM_ECD_PER_ANGLE * 180))
								{
										arm_move.motor_arm.speed_set = PID_Calc(&arm_move.arm_speed_pid, arm_move.motor_arm.arm_motor_measure->total_ecd, (int32_t)(ARM_ECD_PER_ANGLE * 180));
										arm_move.motor_arm.give_current = PID_Calc(&arm_move.arm_current_pid, arm_move.motor_arm.arm_motor_measure->speed_rpm, arm_move.motor_arm.speed_set);
										CAN_CMD_ARM(arm_move.motor_arm.give_current,0,0,0);
								}
								else if(arm_move.motor_arm.arm_motor_measure->total_ecd >= (int32_t)(ARM_ECD_PER_ANGLE * 183))
								{
										arm_move.motor_arm.speed_set = PID_Calc(&arm_move.arm_speed_pid, arm_move.motor_arm.arm_motor_measure->total_ecd, (int32_t)(ARM_ECD_PER_ANGLE * 180));
										arm_move.motor_arm.give_current = PID_Calc(&arm_move.arm_current_pid, arm_move.motor_arm.arm_motor_measure->speed_rpm, arm_move.motor_arm.speed_set);
										CAN_CMD_ARM(arm_move.motor_arm.give_current,0,0,0);
								}
								else
								{
										CAN_CMD_ARM(0,0,0,0);
								}
						}
						else if(arm_move.arm_RC->key.v & KEY_PRESSED_OFFSET_G)
						{
								if(arm_move.motor_arm.arm_motor_measure->total_ecd >= (int32_t)0)
								{
										arm_move.motor_arm.speed_set = PID_Calc(&arm_move.arm_speed_pid, arm_move.motor_arm.arm_motor_measure->total_ecd, (int32_t)0);
										arm_move.motor_arm.give_current = PID_Calc(&arm_move.arm_current_pid, arm_move.motor_arm.arm_motor_measure->speed_rpm, arm_move.motor_arm.speed_set);
										CAN_CMD_ARM(arm_move.motor_arm.give_current,0,0,0);
								}
								else if(arm_move.motor_arm.arm_motor_measure->total_ecd <= (int32_t)(-ARM_ECD_PER_ANGLE * 3))
								{
										arm_move.motor_arm.speed_set = PID_Calc(&arm_move.arm_speed_pid, arm_move.motor_arm.arm_motor_measure->total_ecd, (int32_t)0);
										arm_move.motor_arm.give_current = PID_Calc(&arm_move.arm_current_pid, arm_move.motor_arm.arm_motor_measure->speed_rpm, arm_move.motor_arm.speed_set);
										CAN_CMD_ARM(arm_move.motor_arm.give_current,0,0,0);
								}
								else
								{
										CAN_CMD_ARM(0,0,0,0);
								}
						}
						else
						{
								CAN_CMD_ARM(0,0,0,0);
						}
				}
	
				vTaskDelay(ARM_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
				arm_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

static void arm_init(arm_move_t *arm_move_init)
{
		if (arm_move_init == NULL)
		{
				return;
		}
		
		const static fp32 motor_speed_pid[3] = {ARM_SPEED_PID_KP, ARM_SPEED_PID_KI, ARM_SPEED_PID_KD};
		const static fp32 motor_current_pid[3] = {ARM_CURRENT_PID_KP, ARM_CURRENT_PID_KI, ARM_CURRENT_PID_KD};
	
		//��ʼ����е�۵�����ݵ�ַ
		arm_move_init->motor_arm.arm_motor_measure = get_Arm_Motor_Measure_Point();
		//��ʼ��ת������״̬
		arm_move_init->arm_lock = ARM_LOCKED;
		//��ʼ��ת��λ��
		arm_move_init->arm_angle = ARM_ANGLE_0;
		//��ʼ��ת���ٶ�
		//arm_move_init->motor_arm.speed_set = ARM_MOVE_SPEED_RPM;
		//��ȡң����ָ��
		arm_move_init->arm_RC = get_remote_control_point();
		//��ʼ��PID�˶�
		PID_Init(&arm_move_init->arm_speed_pid, PID_POSITION, motor_speed_pid, ARM_SPEED_PID_MAX_OUT, ARM_SPEED_PID_MAX_IOUT);
		PID_Init(&arm_move_init->arm_current_pid, PID_POSITION, motor_current_pid, ARM_CURRENT_PID_MAX_OUT, ARM_CURRENT_PID_MAX_IOUT);
}

static void lock_state_update(arm_move_t *arm_move_update)
{
		if(arm_move_update->arm_RC->key.v & KEY_PRESSED_OFFSET_CTRL)
		{
				arm_move_update->arm_lock = ARM_LOCKED;
		}
		else if(arm_move_update->arm_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)
		{
				arm_move_update->arm_lock = ARM_UNLOCKED;
		}
}
