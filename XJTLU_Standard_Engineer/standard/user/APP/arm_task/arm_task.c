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

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t arm_task_stack;
#endif

arm_move_t arm_move;

void arm_task(void *pvParameters)
{
    //空闲一段时间
    vTaskDelay(ARM_TASK_INIT_TIME);	
		//机械臂初始化
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
	
		//初始化机械臂电机数据地址
		arm_move_init->motor_arm.arm_motor_measure = get_Arm_Motor_Measure_Point();
		//初始化转动锁定状态
		arm_move_init->arm_lock = ARM_LOCKED;
		//初始化转动位置
		arm_move_init->arm_angle = ARM_ANGLE_0;
		//初始化转动速度
		//arm_move_init->motor_arm.speed_set = ARM_MOVE_SPEED_RPM;
		//获取遥控器指针
		arm_move_init->arm_RC = get_remote_control_point();
		//初始化PID运动
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
