/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       start_task.c/h
  * @brief      �������񣬽�һ������������������Դ�������������ȼ�,
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

#include "Start_Task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Detect_Task.h"
#include "Calibrate_Task.h"
#include "User_Task.h"
#include "INS_Task.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "air_task.h"
#include "arm_task.h"

#define INS_TASK_PRIO 20
#define INS_TASK_SIZE 512
static TaskHandle_t INSTask_Handler;

#define GIMBAL_TASK_PRIO 19
#define GIMBAL_STK_SIZE 512
TaskHandle_t GIMBALTask_Handler;

#define Chassis_TASK_PRIO 18
#define Chassis_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;

#define User_TASK_PRIO 4
#define User_STK_SIZE 256
static TaskHandle_t UserTask_Handler;

#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

#define CALIBRATE_TASK_PRIO 5
#define CALIBRATE_STK_SIZE 512
static TaskHandle_t CalibrateTask_Handler;

#define Detect_TASK_PRIO 10
#define Detect_STK_SIZE 512
static TaskHandle_t DetectTask_Handler;

#define Air_TASK_PRIO 10
#define Air_STK_SIZE 512
static TaskHandle_t AirTask_Handler;

#define Arm_TASK_PRIO 10
#define Arm_STK_SIZE 512
static TaskHandle_t ArmTask_Handler;

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

//    xTaskCreate((TaskFunction_t)INSTask,
//                (const char *)"INSTask",
//                (uint16_t)INS_TASK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)INS_TASK_PRIO,
//                (TaskHandle_t *)&INSTask_Handler);

//    xTaskCreate((TaskFunction_t)GIMBAL_task,
//                (const char *)"GIMBAL_task",
//                (uint16_t)GIMBAL_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)GIMBAL_TASK_PRIO,
//                (TaskHandle_t *)&GIMBALTask_Handler);

//    xTaskCreate((TaskFunction_t)chassis_task,
//                (const char *)"ChassisTask",
//                (uint16_t)Chassis_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)Chassis_TASK_PRIO,
//                (TaskHandle_t *)&ChassisTask_Handler);

//    xTaskCreate((TaskFunction_t)UserTask,
//                (const char *)"UserTask",
//                (uint16_t)User_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)User_TASK_PRIO,
//                (TaskHandle_t *)&UserTask_Handler);

//    xTaskCreate((TaskFunction_t)calibrate_task,
//                (const char *)"CaliTask",
//                (uint16_t)CALIBRATE_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)CALIBRATE_TASK_PRIO,
//                (TaskHandle_t *)&CalibrateTask_Handler);

//    xTaskCreate((TaskFunction_t)DetectTask,
//                (const char *)"DetectTask",
//                (uint16_t)Detect_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)Detect_TASK_PRIO,
//                (TaskHandle_t *)&DetectTask_Handler);

//    xTaskCreate((TaskFunction_t)air_task,
//                (const char *)"AirTask",
//                (uint16_t)Air_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)Air_TASK_PRIO,
//                (TaskHandle_t *)&AirTask_Handler);

    xTaskCreate((TaskFunction_t)arm_task,
                (const char *)"ArmTask",
                (uint16_t)Arm_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Arm_TASK_PRIO,
                (TaskHandle_t *)&ArmTask_Handler);
								
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //������
                (const char *)"start_task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
}