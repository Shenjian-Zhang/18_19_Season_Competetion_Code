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

#include "pid_s.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t arm_task_stack;
#endif

static const RC_ctrl_t *arm_RC; //ң�����ṹ��ָ��
arm_move_t *arm_move;

int32_t arm_total_ecd_target = ARM_ECD_TARGET;
int16_t arm_motor_speed_target = 0;
int16_t arm_motor_current_target = 0;
int32_t len = 0, state = 0;
		
void arm_task(void *pvParameters)
{		
		//��е�۳�ʼ��
		arm_init();

		//��ȡң��������
		arm_RC = get_remote_ctrl_point_arm();
		while (1)
		{
				//arm_motor_speed_target = PID_Calc(&arm_move->arm_ecd_pid, motor_arm.total_ecd, arm_total_ecd_target);
				//arm_motor_current_target = PID_Calc(&arm_move->arm_speed_pid, motor_arm.speed_rpm, arm_motor_speed_target);
				if((arm_RC->key.v & KEY_PRESSED_OFFSET_V) && state == 0)
				{
						motor_arm.total_ecd = 0;
						for(len = 0; len < 5000; len++)
						{
								arm_motor_speed_target = pid_calc(&pid_3508, motor_arm.total_ecd, -arm_total_ecd_target);
								arm_motor_current_target = pid_calc(&pid_3508_speed, motor_arm.speed_rpm, arm_motor_speed_target);			
								CAN_CMD_ARM(arm_motor_current_target,0,0,0);
						}
						if(state == 0) state = 1;
						CAN_CMD_ARM(0,0,0,0);
				}
				else
				{
						CAN_CMD_ARM(0,0,0,0);
				}

				if((arm_RC->key.v & KEY_PRESSED_OFFSET_X) && state == 1)
				{
						motor_arm.total_ecd = 0;
						for(len = 0; len < 35000; len++)
						{
								arm_motor_speed_target = pid_calc(&pid_3508, motor_arm.total_ecd, arm_total_ecd_target);
								arm_motor_current_target = pid_calc(&pid_3508_speed, motor_arm.speed_rpm, arm_motor_speed_target);			
								CAN_CMD_ARM(arm_motor_current_target,0,0,0);
						}
						if(state == 1) state = 0;
						CAN_CMD_ARM(0,0,0,0);
				}
				else
				{
						CAN_CMD_ARM(0,0,0,0);
				}
	
				vTaskDelay(ARM_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
				arm_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void arm_init(void)
{
//	if (arm_move == NULL)
//  {
//		return;
//  }

  //��е��λ�û�pid
  //const fp32 arm_ecd_pid[3] = {0.3, 0, 0};
	//��е���ٶȻ�pid
	//const fp32 arm_speed_pid[3] = {6, 0, 0};

  //��ʼ��PID �˶�
  //PID_Init(&arm_move->arm_ecd_pid, PID_POSITION, arm_ecd_pid, ARM_ECD_PID_MAX_OUT, ARM_ECD_PID_MAX_IOUT);
	//PID_Init(&arm_move->arm_speed_pid, PID_POSITION, arm_speed_pid, ARM_SPEED_PID_MAX_OUT, ARM_SPEED_PID_MAX_IOUT);
	
	PID_struct_init(&pid_3508,			 POSITION_PID, 12000, 5000, 10.0f, 0.0f,  0);
	PID_struct_init(&pid_3508_speed, POSITION_PID, 12000, 5000, 15.0f, 	0.0f,  	0);
}
