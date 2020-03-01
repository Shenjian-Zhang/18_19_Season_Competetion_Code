/**
  ****************************(C) COPYRIGHT FREE********************************
  * @file       refree_receive.c/h
  * @brief      裁判系统数据处理，通过USART3串口收发裁判系统数据，利用DMA传输方式节
	*							约CPU资源，利用串口空闲中断来拉起处理函数。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            	Author          Modification
  *  V1.0.0     July-10-2019     Ziqi Yang          1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT FREE********************************
  */

#include "refree_system.h"
#include "stm32f4xx.h"

//裁判系统串口初始化
static void refree_receive_init(uint32_t baudrate);
	
//裁判系统数据解析
static void get_refree_system_real_data(refree_system_t *refree_system_real_data);
static void refree_data_calc(unpack_data_t *p_obj, refree_system_t *refree_system_data_calc);

//裁判系统数据打包
static void refree_system_data_pack_handle(refree_system_t *refree_system_data_pack);
	
//开启DMA发送
static void REFREE_DMA_TX_Start(char *data, uint16_t size);
	
//DMA缓冲区数组
static char refree_rx_buf[REFREE_RX_BUF_NUM] = {0};
static char refree_tx_buf[REFREE_TX_BUF_NUM] = {0};

//裁判系统接收原始数据
static uint8_t refree_receive_data_raw[REFREE_RX_BUF_NUM] = {0};

//裁判系统结构体
static refree_system_t refree_system = {0};

//4字节16进制数据共用体
static union refree_4_byte_t refree_chassis_power_temp, refree_shoot_info_temp;

static uint8_t ref_seq_num = 0;

//裁判系统串口初始化（外部）
void Refree_Init(void)
{
		refree_receive_init(115200);
}

//裁判系统串口初始化
static void refree_receive_init(uint32_t baudrate)
{		
		/* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

		RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3); //PD9  usart3 rx
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3); //PD8  usart3 tx
    /* -------------- Configure GPIO ---------------------------------------*/
    {
				GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOD, &GPIO_InitStructure);

        USART_DeInit(USART3);

        USART_InitStructure.USART_BaudRate = baudrate;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART3, &USART_InitStructure);

        USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
				USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

        USART_ClearFlag(USART3, USART_FLAG_IDLE);
        USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
				USART_ITConfig(USART3, USART_IT_TC,   DISABLE);
				USART_ITConfig(USART3, USART_IT_TXE,  DISABLE); 

        USART_Cmd(USART3, ENABLE);
		}

    /* -------------- Configure NVIC ---------------------------------------*/
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = REFREE_NVIC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
			
				NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Stream3_IRQn;           
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = REFREE_NVIC;          
				NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; 
				NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
    }

    //DMA1 stream1 ch4    !!!!!!! P206 of the datasheet
    /* -------------- Configure DMA -----------------------------------------*/
    {
				DMA_InitTypeDef DMA_InitStructure;
        DMA_DeInit(DMA1_Stream1);

        DMA_InitStructure.DMA_Channel = DMA_Channel_4;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)refree_rx_buf;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_BufferSize = REFREE_RX_BUF_NUM;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream1, &DMA_InitStructure);
        DMA_Cmd(DMA1_Stream1, DISABLE); //Add a disable
        DMA_Cmd(DMA1_Stream1, ENABLE);
				
        DMA_DeInit(DMA1_Stream3);

        DMA_InitStructure.DMA_Channel = DMA_Channel_4;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)refree_tx_buf;
        DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure.DMA_BufferSize = REFREE_TX_BUF_NUM;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream3, &DMA_InitStructure);
				
				DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
        DMA_Cmd(DMA1_Stream3, DISABLE);
		}
}

//USART串口中断服务函数
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART3);
    }
    else if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
		{
				static uint16_t this_time_rx_len = 0;
				
        USART_ReceiveData(USART3);
				DMA_Cmd(DMA1_Stream1, DISABLE);
				this_time_rx_len = REFREE_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream1);
				DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
				USART_ClearITPendingBit(USART3, USART_IT_IDLE);
				USART3->SR;
				USART3->DR;				
				DMA_Cmd(DMA1_Stream1, ENABLE);
				for(int i = 0; i < REFREE_RX_BUF_NUM; i++)
				{
						refree_receive_data_raw[i] = refree_rx_buf[i];
						refree_rx_buf[i] = 0;
				}
				get_refree_system_real_data(&refree_system);
    }
}

//串口DMA发送中断服务函数
void DMA1_Stream3_IRQHandler(void)
{ 
    if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET)   
    {  
        //清除标志位
        DMA_ClearFlag(DMA1_Stream3, DMA_IT_TCIF3);  
        //关闭DMA
        DMA_Cmd(DMA1_Stream3, DISABLE);
    }
}

//DMA发送函数
static void REFREE_DMA_TX_Start(char *data, uint16_t size)
{
    /* 复制数据 */
    memcpy(refree_tx_buf, data, size);
    /* 设置传输数据长度 */  
    DMA_SetCurrDataCounter(DMA1_Stream3, size);  
    /* 打开DMA,开始发送 */  
    DMA_Cmd(DMA1_Stream3, ENABLE);  
}

//返回裁判系统结构体指针
refree_system_t *get_refree_system_data_point(void)
{
		return &refree_system;
}

//裁判系统数据解析
static void get_refree_system_real_data(refree_system_t *refree_system_real_data)
{
		unpack_data_t p_obj;
		uint16_t i = 0;
		uint8_t byte = 0;
		uint16_t refree_data_legal = 0;
	
		p_obj.data_len = 0;
		p_obj.index = 0;
		p_obj.unpack_step = STEP_HEADER_SOF;
		for(int i = 0; i < REFREE_RX_BUF_NUM; i++)
		{
				byte = refree_receive_data_raw[i];
				switch(p_obj.unpack_step)
				{
						case STEP_HEADER_SOF:
						{
								if(byte == REF_PROTOCOL_HEADER)
								{
										p_obj.protocol_packet[p_obj.index++] = byte;
										p_obj.unpack_step = STEP_LENGTH_LOW;
								}
								else
								{
										p_obj.index = 0;
								}
						}	break;
						
						case STEP_LENGTH_LOW:
						{
								p_obj.data_len = byte;
								p_obj.protocol_packet[p_obj.index++] = byte;
								p_obj.unpack_step = STEP_LENGTH_HIGH;
						} break;
						
						case STEP_LENGTH_HIGH:
						{
								p_obj.data_len |= (byte << 8);
								p_obj.protocol_packet[p_obj.index++] = byte;
								if(p_obj.data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
								{
										p_obj.unpack_step = STEP_FRAME_SEQ;
								}
								else
								{
										p_obj.unpack_step = STEP_HEADER_SOF;
										p_obj.index = 0;
								}
						} break;
						
						case STEP_FRAME_SEQ:
						{
								p_obj.protocol_packet[p_obj.index++] = byte;
								p_obj.unpack_step = STEP_HEADER_CRC8;
						}	break;

						case STEP_HEADER_CRC8:
						{
								p_obj.protocol_packet[p_obj.index++] = byte;
								if (p_obj.index == REF_PROTOCOL_HEADER_SIZE)
								{
										if ( ref_verify_crc8(p_obj.protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
										{
												p_obj.unpack_step = STEP_DATA_CRC16;
										}
										else
										{
												p_obj.unpack_step = STEP_HEADER_SOF;
												p_obj.index = 0;
										}
								}
						}	break;
								
						case STEP_DATA_CRC16:
						{
								if (p_obj.index < (REF_HEADER_CRC_CMDID_LEN + p_obj.data_len))
								{
										p_obj.protocol_packet[p_obj.index++] = byte;  
								}
								if (p_obj.index >= (REF_HEADER_CRC_CMDID_LEN + p_obj.data_len))
								{
										p_obj.unpack_step = STEP_HEADER_SOF;
										p_obj.index = 0;

										if ( ref_verify_crc16(p_obj.protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj.data_len) )
										{
												p_obj.unpack_step = STEP_HEADER_SOF;
												p_obj.index = 0;
										}
								}
						}	break;
								
						default:
						{
								break;
						}
				}
				refree_data_calc(&p_obj, refree_system_real_data);
		}
}

//获取crc8校验数据
static uint8_t ref_get_crc8(uint8_t *p_msg, unsigned int len, uint8_t crc8)
{
    uint8_t uc_index;

    while (len--)
    {
        uc_index = crc8^(*p_msg++);
        crc8  = ref_crc8_tab[uc_index];
    }

    return(crc8);
}

//校验crc8数据
static uint32_t ref_verify_crc8(uint8_t *p_msg, unsigned int len)
{
    uint8_t uc_expected = 0;

    if (p_msg == 0 || (len <= 2)) return 0;

    uc_expected = ref_get_crc8 (p_msg, len-1, ref_crc8_init);

    return ( uc_expected == p_msg[len-1] );
}

//获取crc16校验数据
static uint16_t ref_get_crc16(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;
    
    if (p_msg == NULL)
    {
        return 0xFFFF;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8)  ^ ref_crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

//校验crc16数据
static uint32_t ref_verify_crc16(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if ((p_msg == NULL) || (len <= 2))
    {
        return 1;
    }
    w_expected = ref_get_crc16 ( p_msg, len - 2, ref_crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

//裁判系统数据解析及回传结构体
static void refree_data_calc(unpack_data_t *p_obj, refree_system_t *refree_system_data_calc)
{
		refree_system_data_calc->feedback.cmd_id = ((p_obj->protocol_packet[5]) | (p_obj->protocol_packet[6] << 8));
		if(refree_system_data_calc->feedback.cmd_id == ROBOT_LIVE_CMDID)
		{
				refree_system_data_calc->feedback.robot_survive.robot_legion = ((p_obj->protocol_packet[7]) | (p_obj->protocol_packet[8] << 8));
		}
		else if(refree_system_data_calc->feedback.cmd_id == ROBOT_STATE_CMDID) 
		{
				refree_system_data_calc->feedback.robot_state.robot_id = p_obj->protocol_packet[7];
				refree_system_data_calc->feedback.robot_state.robot_level = p_obj->protocol_packet[8];
				refree_system_data_calc->feedback.robot_state.remain_HP = ((p_obj->protocol_packet[9]) | (p_obj->protocol_packet[10] << 8));
				refree_system_data_calc->feedback.robot_state.max_HP = ((p_obj->protocol_packet[11]) | (p_obj->protocol_packet[12] << 8));
				refree_system_data_calc->feedback.robot_state.shooter_heat0_cooling_rate = ((p_obj->protocol_packet[13]) | (p_obj->protocol_packet[14] << 8));
				refree_system_data_calc->feedback.robot_state.shooter_heat0_cooling_limit = ((p_obj->protocol_packet[15]) | (p_obj->protocol_packet[16] << 8));
				refree_system_data_calc->feedback.robot_state.shooter_heat1_cooling_rate = ((p_obj->protocol_packet[17]) | (p_obj->protocol_packet[18] << 8));
				refree_system_data_calc->feedback.robot_state.shooter_heat1_cooling_limit = ((p_obj->protocol_packet[19]) | (p_obj->protocol_packet[20] << 8));
				refree_system_data_calc->feedback.robot_state.mains_power_gimbal_output = (p_obj->protocol_packet[21]);
				refree_system_data_calc->feedback.robot_state.mains_power_chassis_output = (p_obj->protocol_packet[22]);
				refree_system_data_calc->feedback.robot_state.mains_power_shooter_output = (p_obj->protocol_packet[23]);
		}
		else if(refree_system_data_calc->feedback.cmd_id == ROBOT_POWER_HEAT_CMDID)
		{
				refree_system_data_calc->feedback.robot_power_heat.chassis_volt = ((p_obj->protocol_packet[7]) | (p_obj->protocol_packet[8] << 8)) / 1000.0f;
				refree_system_data_calc->feedback.robot_power_heat.chassis_current = ((p_obj->protocol_packet[9]) | (p_obj->protocol_packet[10] << 8)) / 1000.0f;
				refree_chassis_power_temp.buf[0] = p_obj->protocol_packet[11];
				refree_chassis_power_temp.buf[1] = p_obj->protocol_packet[12];
				refree_chassis_power_temp.buf[2] = p_obj->protocol_packet[13];
				refree_chassis_power_temp.buf[3] = p_obj->protocol_packet[14];
				refree_system_data_calc->feedback.robot_power_heat.chassis_power = refree_chassis_power_temp.f;
				refree_system_data_calc->feedback.robot_power_heat.chassis_power_buffer = ((p_obj->protocol_packet[15]) | (p_obj->protocol_packet[16] << 8));
				refree_system_data_calc->feedback.robot_power_heat.shooter_heat0 = ((p_obj->protocol_packet[17]) | (p_obj->protocol_packet[18] << 8));
				refree_system_data_calc->feedback.robot_power_heat.shooter_heat1 = ((p_obj->protocol_packet[19]) | (p_obj->protocol_packet[20] << 8));
		}
		else if(refree_system_data_calc->feedback.cmd_id == ROBOT_SHOOT_CMDID)
		{
				refree_system_data_calc->feedback.robot_shoot.bullet_type = p_obj->protocol_packet[7];
				refree_system_data_calc->feedback.robot_shoot.bullet_freq = p_obj->protocol_packet[8];
				refree_shoot_info_temp.buf[0] = p_obj->protocol_packet[9];
				refree_shoot_info_temp.buf[1] = p_obj->protocol_packet[10];
				refree_shoot_info_temp.buf[2] = p_obj->protocol_packet[11];
				refree_shoot_info_temp.buf[3] = p_obj->protocol_packet[12];
				refree_system_data_calc->feedback.robot_shoot.bullet_speed = refree_shoot_info_temp.f;		
		}
}

//裁判系统发送数据打包
static void refree_system_data_pack_handle(refree_system_t *refree_system_data_pack)
{
		uint16_t crc16 = 0;
		uint16_t i = 0;
	
		//帧头数据
		refree_tx_buf[0]  = REF_PROTOCOL_HEADER;
		refree_tx_buf[1]  = 0x13;
		refree_tx_buf[2]  = 0x00;
		refree_tx_buf[3]  = ref_seq_num++;
		refree_tx_buf[4]  = ref_get_crc8 ( (uint8_t *)refree_tx_buf, REF_PROTOCOL_HEADER_SIZE-1, ref_crc8_init);
		
		//客户端CMDID 0x0301
		refree_tx_buf[5]  = 0x01;
		refree_tx_buf[6]  = 0x03;
		
		//客户端自定义数据内容ID 0xD180
		refree_tx_buf[7]  = 0x80;
		refree_tx_buf[8]  = 0xD1;
		
		//发送者ID 步兵
		refree_tx_buf[9]  = 0x03;
		refree_tx_buf[10] = 0x00;

		//客户端ID 步兵
		refree_tx_buf[11] = 0x03;
		refree_tx_buf[12] = 0x01;
		
		//自定义浮点数据
		for (i = 0 ; i < 4; i++)
		{
				refree_tx_buf[CLIENT_CUSTOM_VALUE_1_BUF_NUM + i] = refree_system_data_pack->client.custom_value[CLIENT_CUSTOM_VALUE_1].buf[i];
		}

		for (i = 0 ; i < 4; i++)
		{
				refree_tx_buf[CLIENT_CUSTOM_VALUE_2_BUF_NUM + i] = refree_system_data_pack->client.custom_value[CLIENT_CUSTOM_VALUE_2].buf[i];
		}
		
		for (i = 0 ; i < 4; i++)
		{
				refree_tx_buf[CLIENT_CUSTOM_VALUE_3_BUF_NUM + i] = refree_system_data_pack->client.custom_value[CLIENT_CUSTOM_VALUE_3].buf[i];
		}
		
		//6个指示灯		
		refree_system_data_pack->client.custom_mark.buf = ((refree_system_data_pack->client.custom_mark.marks[0]) | 
																											 (refree_system_data_pack->client.custom_mark.marks[1] << 1) |
																											 (refree_system_data_pack->client.custom_mark.marks[2] << 2) |
																											 (refree_system_data_pack->client.custom_mark.marks[3] << 3) |
																											 (refree_system_data_pack->client.custom_mark.marks[4] << 4) |
																											 (refree_system_data_pack->client.custom_mark.marks[5] << 5));
		refree_tx_buf[25] = refree_system_data_pack->client.custom_mark.buf;
		
		//帧尾CRC16校验
		crc16 = ref_get_crc16 ( (uint8_t *)refree_tx_buf, 28-2, ref_crc16_init );
		refree_tx_buf[26] = (uint8_t)(crc16 & 0x00ff);
		refree_tx_buf[27] = (uint8_t)((crc16 >> 8)& 0x00ff);
}

//设定客户端自定义浮点数
void set_client_custom_value(refree_system_t *set_refree_client_custom_value, client_custom_value_t custom_value_pos, float custom_value)
{
		if((set_refree_client_custom_value == NULL) || (custom_value_pos != CLIENT_CUSTOM_VALUE_1 && 
																										custom_value_pos != CLIENT_CUSTOM_VALUE_2 && 
																										custom_value_pos != CLIENT_CUSTOM_VALUE_3))
		{
				return;
		}
		
		set_refree_client_custom_value->client.custom_value[custom_value_pos].f = custom_value;
		refree_system_data_pack_handle(set_refree_client_custom_value);
		REFREE_DMA_TX_Start(refree_tx_buf, REFREE_TX_BUF_NUM);
}

//设定客户端6个指示灯状态
void set_client_custom_mark(refree_system_t *set_refree_client_custom_mark, uint16_t mark_pos, uint16_t mark_state)
{
		if((set_refree_client_custom_mark == NULL) || mark_pos > 5 || (mark_state != 0 && mark_state != 1))
		{
				return;
		}

		set_refree_client_custom_mark->client.custom_mark.marks[mark_pos] = mark_state;
		refree_system_data_pack_handle(set_refree_client_custom_mark);
		REFREE_DMA_TX_Start(refree_tx_buf, REFREE_TX_BUF_NUM);
}
