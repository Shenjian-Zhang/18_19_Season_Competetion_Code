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
#ifndef ARM_TASK_H
#define ARM_TASK_H
#include "main.h"
#include "stm32f4xx.h"
#include "CAN_Receive.h"
#include "Remote_Control.h"
#include "pid.h"
#include "led.h"

#define ARM_CONTROL_TIME 2 //校准任务函数运行的周期 为1ms

//#define ARM_SPEED_PID_KP 50.0f
//#define ARM_SPEED_PID_KI 0.0f
//#define ARM_SPEED_PID_KD 0.0f
//#define ARM_SPEED_PID_MAX_OUT 16000.0f
//#define ARM_SPEED_PID_MAX_IOUT 2000.0f
#define ARM_SPEED_PID_KP 0.3f
#define ARM_SPEED_PID_KI 0.0f
#define ARM_SPEED_PID_KD 0.0f
#define ARM_SPEED_PID_MAX_OUT 5000.0f
#define ARM_SPEED_PID_MAX_IOUT 2000.0f

#define ARM_CURRENT_PID_KP 6.0f
#define ARM_CURRENT_PID_KI 0.0f
#define ARM_CURRENT_PID_KD 0.0f
#define ARM_CURRENT_PID_MAX_OUT 6000.0f
#define ARM_CURRENT_PID_MAX_IOUT 2000.0f

#define ARM_ECD_PER_ANGLE 433.2f
#define ARM_TASK_INIT_TIME 357
#define ARM_MOVE_SPEED_RPM 500

typedef enum
{
		ARM_LOCKED = 0,
		ARM_UNLOCKED
} Arm_Lock_t;

typedef enum
{
		ARM_ANGLE_0 = 0,
		ARM_ANGLE_90,
		ARM_ANGLE_180
} Arm_Angle_t;

typedef struct
{
  motor_measure_t *arm_motor_measure;
  int16_t speed_set;
  int16_t give_current;
} Arm_Motor_t;

typedef struct
{
	const RC_ctrl_t *arm_RC;
  Arm_Motor_t motor_arm;          //底盘电机数据
	Arm_Lock_t arm_lock;
	Arm_Angle_t arm_angle;
  PidTypeDef arm_speed_pid;             //底盘电机速度pid
	PidTypeDef arm_current_pid;
} arm_move_t;

//气泵控制任务
extern void arm_task(void *pvParameters);
static void arm_init(arm_move_t *arm_move_init);
static void lock_state_update(arm_move_t *arm_move_update);
#endif
