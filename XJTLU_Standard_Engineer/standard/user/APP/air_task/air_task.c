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
#include "air_task.h"
#include "Remote_Control.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t air_task_stack;
#endif

static const RC_ctrl_t *air_RC; //ң�����ṹ��ָ��

void air_task(void *pvParameters)
{
		//��ȡң��������
		air_RC = get_remote_ctrl_point_air();

		while (1)
		{
				if(air_RC->key.v & KEY_PRESSED_OFFSET_B)
				{
						TIM_SetCompare1(TIM2, 3000);		
				}
				else
				{
						TIM_SetCompare1(TIM2, 0);
				}
		
				if(air_RC->key.v & KEY_PRESSED_OFFSET_C)
				{
						TIM_SetCompare2(TIM2, 3000);
				}
				else
				{
						TIM_SetCompare2(TIM2, 0);
				}
				
				if(air_RC->key.v & KEY_PRESSED_OFFSET_A)
				{
						TIM_SetCompare3(TIM2, 3000);		
				}
				else
				{
						TIM_SetCompare3(TIM2, 0);
				}
		
				if(air_RC->key.v & KEY_PRESSED_OFFSET_D)
				{
						TIM_SetCompare4(TIM2, 3000);
				}
				else
				{
						TIM_SetCompare4(TIM2, 0);
				}
			
    vTaskDelay(AIR_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
				air_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void air_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, DISABLE);

    TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 180 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_Pulse = 3000;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
		
		TIM_SetCompare1(TIM2, 0);
		TIM_SetCompare2(TIM2, 0);
		TIM_SetCompare3(TIM2, 0);
		TIM_SetCompare4(TIM2, 0);
}
