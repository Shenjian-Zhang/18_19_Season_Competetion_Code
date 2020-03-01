/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       Calibrate_task.c/h
  * @brief      完成校准相关设备的数据，包括云台，陀螺仪，加速度计，磁力计
  *             云台校准主要是中值，最大最小相对角度，陀螺仪主要校准零漂
  *             加速度计和磁力计只是写好接口函数，加速度计目前没有必要校准，
  *             磁力计尚未使用在解算算法中。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

//包含校准设备的名字，校准标识符，校准数据flash大小，校准命令，对应的校准数据地址
#include "arm_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "pid_s.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t arm_task_stack;
#endif

static const RC_ctrl_t *arm_RC; //遥控器结构体指针
arm_move_t *arm_move;

int32_t arm_total_ecd_target = ARM_ECD_TARGET;
int16_t arm_motor_speed_target = 0;
int16_t arm_motor_current_target = 0;
int32_t len = 0, state = 0;
		
void arm_task(void *pvParameters)
{		
		//机械臂初始化
		arm_init();

		//获取遥控器数据
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

  //机械臂位置环pid
  //const fp32 arm_ecd_pid[3] = {0.3, 0, 0};
	//机械臂速度环pid
	//const fp32 arm_speed_pid[3] = {6, 0, 0};

  //初始化PID 运动
  //PID_Init(&arm_move->arm_ecd_pid, PID_POSITION, arm_ecd_pid, ARM_ECD_PID_MAX_OUT, ARM_ECD_PID_MAX_IOUT);
	//PID_Init(&arm_move->arm_speed_pid, PID_POSITION, arm_speed_pid, ARM_SPEED_PID_MAX_OUT, ARM_SPEED_PID_MAX_IOUT);
	
	PID_struct_init(&pid_3508,			 POSITION_PID, 12000, 5000, 10.0f, 0.0f,  0);
	PID_struct_init(&pid_3508_speed, POSITION_PID, 12000, 5000, 15.0f, 	0.0f,  	0);
}
