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

#define get_remote_ctrl_point_arm() get_remote_control_point() //获取遥控器的结构体指针

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
  //Arm_Motor_t motor_arm;          //底盘电机数据
	PidTypeDef arm_ecd_pid;             //底盘电机速度pid
  PidTypeDef arm_speed_pid;             //底盘电机速度pid
} arm_move_t;

//气泵控制任务
extern void arm_task(void *pvParameters);
void arm_init(void);

extern motor_measure_t motor_arm;
#endif
