#include "super_cap.h"
#include "stm32f4xx.h"

super_cap_t super_cap;

void super_cap_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

		super_cap_stop_charging();
    super_cap_off();
	
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);	

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);
    
    ADC_Cmd(ADC1, ENABLE);
}

void super_cap_start_charging(void)
{
		super_cap_off();
		GPIO_SetBits(GPIOF, GPIO_Pin_1);
		super_cap.state = SUPER_CAP_OFF_CHARGING;
}

void super_cap_stop_charging(void)
{
		GPIO_ResetBits(GPIOF, GPIO_Pin_1);
		super_cap.state = SUPER_CAP_OFF_NO_CHARGING;
}	

void super_cap_on(void)
{
		super_cap_stop_charging();
    GPIO_SetBits(GPIOF, GPIO_Pin_0);
		super_cap.state = SUPER_CAP_ON;
}

void super_cap_off(void)
{
    GPIO_ResetBits(GPIOF, GPIO_Pin_0);
		super_cap.state = SUPER_CAP_OFF_NO_CHARGING;
}

float get_super_cap_voltage(void)
{
		float voltage = 0.0f;
	
    ADC_ClearFlag(ADC1,ADC_FLAG_STRT|ADC_FLAG_OVR|ADC_FLAG_EOC);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_480Cycles);

    ADC_SoftwareStartConv(ADC1);

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    {
        ;
    }
		
		voltage = ADC_GetConversionValue(ADC1) * 23.3 / 4096;
    return voltage;
}

super_cap_t *get_super_cap_data(void)
{
		return &super_cap;
}
