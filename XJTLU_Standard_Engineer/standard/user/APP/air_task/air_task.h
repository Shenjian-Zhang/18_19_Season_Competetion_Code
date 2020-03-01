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
#ifndef AIR_TASK_H
#define AIR_TASK_H
#include "main.h"
#include "stm32f4xx.h"

#define AIR_CONTROL_TIME 1 //校准任务函数运行的周期 为1ms

#define get_remote_ctrl_point_air() get_remote_control_point() //获取遥控器的结构体指针

//气泵控制任务
extern void air_task(void *pvParameters);
void air_configuration(void);
#endif
