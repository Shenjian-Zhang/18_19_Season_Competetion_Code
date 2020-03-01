#include "super_cap.h"
#include "stm32f4xx.h"

void super_cap_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    super_cap_off();
}

void super_cap_on(void)
{
    GPIO_SetBits(GPIOF, GPIO_Pin_0);
}

void super_cap_off(void)
{
    GPIO_ResetBits(GPIOF, GPIO_Pin_0);
}
