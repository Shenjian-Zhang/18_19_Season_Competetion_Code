#include "fric.h"

#include "stm32f4xx.h"

void fric_PWM_configuration(void) //
{

    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, DISABLE);

    TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 180 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_Pulse = 1000;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
		TIM_OC2Init(TIM1, &TIM_OCInitStructure);
		TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);

    fric_off();
		fric_big_off();

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    GPIO_SetBits(GPIOF, GPIO_Pin_10);
}

void fric_off(void)
{
    TIM_SetCompare1(TIM1, Fric_OFF);
    TIM_SetCompare4(TIM1, Fric_OFF);
}
void fric1_on(uint16_t cmd)
{
    TIM_SetCompare1(TIM1, cmd);
}
void fric2_on(uint16_t cmd)
{
    TIM_SetCompare4(TIM1, cmd);
}

void fric_Big_PWM_configuration(void) //
{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//    TIM_OCInitTypeDef TIM_OCInitStructure;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
//		TIM_DeInit(TIM8);

//    GPIO_PinAFConfig(GPIOI, GPIO_PinSource5, GPIO_AF_TIM8);
//		GPIO_PinAFConfig(GPIOI, GPIO_PinSource6, GPIO_AF_TIM8);

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOI, &GPIO_InitStructure);

//    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM8, ENABLE);
//    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM8, DISABLE);

//    TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1;
//    TIM_TimeBaseInitStructure.TIM_Prescaler = 180 - 1;
//    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

//    TIM_TimeBaseInitStructure.TIM_ClockDivision = 0; //TIM_CKD_DIV1;

//    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);

//    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
//    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
//    TIM_OCInitStructure.TIM_Pulse = 1000;

//    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
//		TIM_OC2Init(TIM8, &TIM_OCInitStructure);

//    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
//		TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

//    TIM_ARRPreloadConfig(TIM8, ENABLE);

//    TIM_CtrlPWMOutputs(TIM8, ENABLE);

//    TIM_Cmd(TIM8, ENABLE);
//		
////		fric_big_off();
		;
}

void fric_big_off(void)
{
    TIM_SetCompare2(TIM1, Fric_Big_OFF);
		TIM_SetCompare3(TIM1, Fric_Big_OFF);
}
void fric1_big_on(uint16_t cmd)
{
    TIM_SetCompare2(TIM1, cmd);
}
void fric2_big_on(uint16_t cmd)
{
    TIM_SetCompare3(TIM1, cmd);
}
