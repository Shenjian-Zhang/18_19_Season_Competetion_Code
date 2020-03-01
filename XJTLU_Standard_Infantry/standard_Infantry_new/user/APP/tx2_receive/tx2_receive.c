/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       tx2_receive.c/h
  * @brief      �Ӿ��������ݴ���ͨ��USART6���ڽ����Ӿ��������ݣ�����DMA���䷽ʽ��
	*							ԼCPU��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA��
	*							���ڵķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            	Author          Modification
  *  V1.0.0     June-30-2019     Ziqi Yang          1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "tx2_receive.h"
#include "stm32f4xx.h"

//�Ӿ����ڳ�ʼ��
static void vision_receive_init(uint32_t baudrate);

//�Ӿ����ݽ���
static void vision_data_unpack_handle(vision_receive_t *vision_data_unpack);
static void vision_data_update(vision_receive_t *vision_data_update);

//DMA����������
static char vision_rx_buf[VISION_RX_BUF_NUM];

//�Ӿ�����ԭʼ���� 
uint8_t vision_receive_data_raw[VISION_RX_BUF_NUM] = {0};

//�Ӿ����ݽṹ��
vision_receive_t vision_receive;

//�Ӿ�����2�ֽ�16����ת10�������ݹ�����
static union vision_2_byte_t vision_pitch_temp, vision_yaw_temp;

void Vision_Init(void)
{
		vision_receive_init(115200);
		vision_receive.difference_angle.pitch = VISION_NO_TARGET_VALUE;
		vision_receive.difference_angle.yaw = VISION_NO_TARGET_VALUE;
}

void vision_receive_init(uint32_t baudrate)
{		
		/* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

		RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, DISABLE);

    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6); //PB7  usart1 rx
    /* -------------- Configure GPIO ---------------------------------------*/
    {
				GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOG, &GPIO_InitStructure);

        USART_DeInit(USART6);

        USART_InitStructure.USART_BaudRate = baudrate;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_Mode = USART_Mode_Rx;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART6, &USART_InitStructure);

        USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

        USART_ClearFlag(USART6, USART_FLAG_IDLE);
        USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);

        USART_Cmd(USART6, ENABLE);
		}

    /* -------------- Configure NVIC ---------------------------------------*/
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = VISION_NVIC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    //DMA2 stream1 ch5  or (DMA2 stream2 ch5)    !!!!!!! P206 of the datasheet
    /* -------------- Configure DMA -----------------------------------------*/
    {
				DMA_InitTypeDef DMA_InitStructure;
        DMA_DeInit(DMA2_Stream1);

        DMA_InitStructure.DMA_Channel = DMA_Channel_5;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)vision_rx_buf;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_BufferSize = VISION_RX_BUF_NUM;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA2_Stream1, &DMA_InitStructure);
        DMA_Cmd(DMA2_Stream1, DISABLE); //Add a disable
        DMA_Cmd(DMA2_Stream1, ENABLE);
		}
}

//�����Ӿ����ݱ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
const vision_receive_t *get_vision_receive_point(void)
{
    return &vision_receive;
}

//USART�����жϷ�����
void USART6_IRQHandler(void)
{
    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART6);
    }
    else if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
		{
				static uint16_t this_time_rx_len = 0;
				
        USART_ReceiveData(USART6);
				DMA_Cmd(DMA2_Stream1, DISABLE);
				this_time_rx_len = VISION_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream1);
				DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
				USART_ClearITPendingBit(USART6, USART_IT_IDLE);
				DMA_Cmd(DMA2_Stream1, ENABLE);
				for(int i = 0; i < VISION_RX_BUF_NUM; i++)
				{
						vision_receive_data_raw[i] = vision_rx_buf[i];
						vision_rx_buf[i] = 0;
				}
				vision_data_unpack_handle(&vision_receive);
				vision_data_update(&vision_receive);
    }
}

//�Ӿ����ݰ���ȡ
static void vision_data_unpack_handle(vision_receive_t *vision_data_unpack)
{
		vision_unpack_data_t p_obj;
		uint8_t byte = 0;
		uint16_t vision_data_legal = 0;
	
		p_obj.index = 0;
		p_obj.unpack_step = VISION_STEP_HEADER_SOF;
		//while(!vision_data_legal)
		for(int i = 0; i < VISION_PROTOCOL_FRAME_MAX_SIZE; i++)
		{
				byte = vision_receive_data_raw[p_obj.index];
				switch(p_obj.unpack_step)
				{
						case VISION_STEP_HEADER_SOF:
						{
								if(byte == VISION_PROTOCOL_HEADER)
								{
										p_obj.protocol_packet[p_obj.index++] = byte;
										p_obj.unpack_step = VISION_STEP_DATA_CRC16;
								}
								else
								{
										p_obj.index = 0;
								}
						}	break;
								
						case VISION_STEP_DATA_CRC16:
						{
								if (p_obj.index < (VISION_PROTOCOL_FRAME_MAX_SIZE))
								{
										p_obj.protocol_packet[p_obj.index++] = byte;  
								}
								else
								{
										if ( ref_verify_crc16(p_obj.protocol_packet, VISION_PROTOCOL_FRAME_MAX_SIZE))
										{
												vision_data_legal = 1;
										}
										else
										{
												vision_data_legal = 0;
												p_obj.unpack_step = VISION_STEP_HEADER_SOF;
												p_obj.index = 0;
										}
								}
						}	break;
								
						default:
						{
								break;
						}
				}
		}
		for(p_obj.index = 0; p_obj.index < VISION_PROTOCOL_FRAME_MAX_SIZE; p_obj.index++)
		{
				vision_data_unpack->unpack_data[p_obj.index] = p_obj.protocol_packet[p_obj.index];
		}
}

//�Ӿ����ݸ���
static void vision_data_update(vision_receive_t *vision_data_update)
{
		float vision_temp[2] = {0};
		
		vision_pitch_temp.buf[0] = vision_data_update->unpack_data[2];
		vision_pitch_temp.buf[1] = vision_data_update->unpack_data[3];
		
		vision_yaw_temp.buf[0] = vision_data_update->unpack_data[4];
		vision_yaw_temp.buf[1] = vision_data_update->unpack_data[5];
	
		vision_temp[0] = (float)vision_pitch_temp.d;
		vision_temp[1] = (float)vision_yaw_temp.d;

		if(vision_temp[0] != VISION_NO_TARGET_VALUE)
		{
				vision_temp[0] = theta_format(vision_temp[0]);
				vision_temp[0] = rad_format(vision_temp[0]);
		}
		if(vision_temp[1] != VISION_NO_TARGET_VALUE)
		{
				vision_temp[1] = theta_format(vision_temp[1]);
				vision_temp[1] = rad_format(vision_temp[1]);
		}
	
		vision_data_update->difference_angle.pitch = vision_temp[0];
		vision_data_update->difference_angle.yaw = vision_temp[1];
}

void vision_unable(void)
{
		USART_Cmd(USART6, DISABLE);
}
