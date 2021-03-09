/*
 *Copyright (c) 2018-2020, qianliyan Robot
 *Change log:
 *	2020-10-14
 *
 *Date: 2020-xx-xx
 *Author: Feng
 *Remarks: http://www.qlybot.com/
 *Mail: 921656494@qq.com
 *version 1.0
 *��Ȩ���� ����ؾ�
*/
#include "sys.h"
#include "includes.h"
/* ������� -S */
u8 USART2_RX_BUFFER_SIZE=64;
u16 Check_485=0;
u8 Usart2_temp,n;
u8 rs485buf_0[7]={0XDD,0XA5,0X03,0X00,0XFF,0XFD,0X77};    /* ��ȡָ������Ϣ */
/* ������� -E */

/* ����ʹ�ܱ�־λ */
#if EN_USART2_RX
/* ����2�жϷ����� */
void USART2_IRQHandler(void)
{
	#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();
	#endif
	if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET)
	{
		/* ����������IDLE��־λ */
		USART2->SR;
		USART2->DR;
		/* �ر�DMA��׼���������� */
		DMA_Cmd(DMA1_Stream5, DISABLE);
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
		/* ���������ɱ�־ */
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TEIF5);
		/* �����������־ */
	}
	/* ������һ��DMA���� */
	DMA_Enable(DMA1_Stream5,USART2_RX_BUFFER_SIZE);
	#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();
	#endif
}
#endif
/*
RS485+DMA���ճ�ʼ������
bound:������
���ô���2
*/ 
void RS485_Init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	/* �����ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
  /* ����2���Ÿ���ӳ�� */
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	/* ����2 IO���� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	/* 485�����������ã�485ģʽ���� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  /* USART2 ��ʼ������ */
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
	/* ʹ�ܴ���2 */
  USART_Cmd(USART2, ENABLE);
#if EN_USART2_RX	
	/* Usart2 NVIC���� */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//���������ж�
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);//����DMA����
	USART_Cmd(USART2, ENABLE);
	/* DMA��ʼ�� */
  DMA_DeInit(DMA1_Stream5);
	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE){}//�ȴ�DMA������ 
  /* ���� DMA Stream */
  DMA_InitStructure.DMA_Channel 						= DMA_Channel_4;  							//ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&(USART2->DR);			//DMA�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)RS485_RX_BUF;				//DMA �洢��0��ַ
  DMA_InitStructure.DMA_DIR 					    	= DMA_DIR_PeripheralToMemory;		//direction of transmit.
  DMA_InitStructure.DMA_BufferSize 				  = USART2_RX_BUFFER_SIZE;				//���ݴ����� 
  DMA_InitStructure.DMA_PeripheralInc				= DMA_PeripheralInc_Disable;		//���������ģʽ
  DMA_InitStructure.DMA_MemoryInc 					= DMA_MemoryInc_Enable;					//�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;	//�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;			//�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_Mode 								= DMA_Mode_Normal;							// ʹ����ͨģʽ 
  DMA_InitStructure.DMA_Priority 						= DMA_Priority_High;						//�е����ȼ�
  DMA_InitStructure.DMA_FIFOMode 						= DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;				//�洢��ͻ�����δ���
  DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;		//����ͻ�����δ���
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	/* ʹ��DMA1������Ĭ��Ϊ����ģʽ */
  DMA_Cmd(DMA1_Stream5,ENABLE);
	RS485_TX_EN=0;
}
/* �����Ϣ��ѵ��ѯ���� */
void Rs485_Flash(u16 time)
{
	static int rs485_temp;
//	static u8 ch,last_ch;
	if(++rs485_temp==time)
	{
		/* ���͵����Ϣ��ȡָ�� */
		RS485_Send_Data(rs485buf_0,sizeof(rs485buf_0));
		/* ��ӡ�����Ϣ */
//		RS485_Receive_Data();
		/* ����35������ С��20����� ����35������ */
		if(RS485_RX_BUF[23]>35)
			R_LED=1,G_LED=0,B_LED=1; 
		else if(RS485_RX_BUF[23]<20)
			R_LED=1,G_LED=1,B_LED=0;
		else if(RS485_RX_BUF[23]<35)
			R_LED=1,G_LED=0,B_LED=0;
		/* OLED ��Ļ����ʾ */
//			OLED_ShowString(00,10,(u8*)"Elec:");
//			OLED_ShowNumber(50,10,RS485_RX_BUF[23],3,12);
//			OLED_ShowString(65,10,(u8*)"%");
//			OLED_Refresh_Gram();
		rs485_temp=0;
	}
}
/* ����һ��DMA���� */
void DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
	DMA_Cmd(DMA_Streamx, DISABLE);                      //�ȹر�DMA,����������
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//�ȴ��������
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          	//���ô������ݳ��� 
	DMA_Cmd(DMA_Streamx, ENABLE);                      	//����DMA
}
/* 
RS485ָ��ͺ���
buf:�������׵�ַ
len:���͵��ֽ���(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�鲻Ҫ����64���ֽ�)
*/
void RS485_Send_Data(u8 *buf,u8 len)
{
	/* ����Ϊ����ģʽ */
	RS485_TX_EN=1;
	/* ѭ���������� */
	for(n=0;n<len;n++)
	{
		/* �ȴ����ͽ��� */
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
		/* �������� */
		USART_SendData(USART2,buf[n]);
	}
	/* �ȴ����ͽ��� ����Ϊ����ģʽ */
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
	RS485_TX_EN=0;
}
u8 high_bit,low_bit;
/* ��ӡ�����Ϣ */
void RS485_Receive_Data(void)
{
	/* �ۼ�ȡ����1 �õ�У���� */
	for(n=2;n<31;n++)
		Check_485 += RS485_RX_BUF[n];
	Check_485 = (~Check_485)+1;
	/* У��ͨ������ӡ��Ϣ */
	if(Check_485 == (RS485_RX_BUF[31]*256+RS485_RX_BUF[32]))
	{
		switch(RS485_RX_BUF[1])
		{
			case 0x03:
			{
				low_bit = RS485_RX_BUF[7];
				high_bit = RS485_RX_BUF[6];
				printf("********************************\r\n");
				printf("�ܵ�ѹ---------->> %d mV\r\n", 	(RS485_RX_BUF[4]*256+RS485_RX_BUF[5])*10);
				
				if(((high_bit&0x80)>>7))
				{
					printf("�ŵ���\r\n");
					high_bit = ((~high_bit)+1);
					low_bit = ((~low_bit)+1);
				}
				else if(!((high_bit&0x80)>>7))
				{
					if(low_bit)
						printf("�����\r\n");
					else
						printf("�ŵ���\r\n");
				}
				
//				printf("%d\r\n", 	(RS485_RX_BUF[6]&0x80)>>7); && low_bit  && !((low_bit&0x80)>>7)   && ((low_bit&0x80)>>7)
				printf("��  ��---------->> %d mA\r\n", 	(high_bit*256+low_bit)*10);
//				printf("ʣ������-------->> %d mAh\r\n",	(RS485_RX_BUF[8]*256+RS485_RX_BUF[9])*10);
//				printf("�������-------->> %d mAh\r\n",	(RS485_RX_BUF[10]*256+RS485_RX_BUF[11])*10);
//				printf("ѭ������-------->> %d ��/Cycle\r\n",(RS485_RX_BUF[12]*256+RS485_RX_BUF[13])*10);
//				printf("��������-------->> %d��%d��%d��\r\n",2000+((RS485_RX_BUF[14]*256+RS485_RX_BUF[15])>>9),\
//				((RS485_RX_BUF[14]*256+RS485_RX_BUF[15])>>5)&0x0f,(RS485_RX_BUF[14]*256+RS485_RX_BUF[15])&0x1f);
//				printf("����״̬_�ʹ�--->> %d \r\n",	RS485_RX_BUF[16]*256+RS485_RX_BUF[17]);
//				printf("����״̬_�ߴ�--->> %d \r\n",	RS485_RX_BUF[18]*256+RS485_RX_BUF[19]);
//				printf("����״̬-------->> %d \r\n",	RS485_RX_BUF[20]*256+RS485_RX_BUF[21]);
//				printf("����汾-------->> %d \r\n",	RS485_RX_BUF[22]);
//				printf("ʣ������-------->> %d %% \r\n",RS485_RX_BUF[23]);
//				printf("FET����״̬----->> %d \r\n",RS485_RX_BUF[24]);
//				printf("��ش���-------->> %d \r\n",RS485_RX_BUF[25]);
//				printf("NTC����--------->> %d \r\n",RS485_RX_BUF[26]);
//				printf("NTC_1----------->> %d \r\n",((RS485_RX_BUF[27]*256+RS485_RX_BUF[28])-2731)/10);
//				printf("NTC_2----------->> %d \r\n",((RS485_RX_BUF[29]*256+RS485_RX_BUF[30])-2731)/10);
				break;
			}
			case 0x04:
			{
				printf("********************************\r\n");
				printf("��ش�_1-------->> %d mV\r\n",RS485_RX_BUF[4]*256+RS485_RX_BUF[5]);
				printf("��ش�_2-------->> %d mV\r\n",RS485_RX_BUF[6]*256+RS485_RX_BUF[7]);
				printf("��ش�_3-------->> %d mV\r\n",RS485_RX_BUF[8]*256+RS485_RX_BUF[9]);
				printf("��ش�_4-------->> %d mV\r\n",RS485_RX_BUF[10]*256+RS485_RX_BUF[11]);
				printf("��ش�_5-------->> %d mV\r\n",RS485_RX_BUF[12]*256+RS485_RX_BUF[13]);
				printf("��ش�_6-------->> %d mV\r\n",RS485_RX_BUF[14]*256+RS485_RX_BUF[15]);
				printf("��ش�_7-------->> %d mV\r\n",RS485_RX_BUF[16]*256+RS485_RX_BUF[17]);
				printf("��ش�_8-------->> %d mV\r\n",RS485_RX_BUF[18]*256+RS485_RX_BUF[19]);
				break;
			}
			case 0x05:
			{
				printf("********************************\r\n");
				printf("��ذ汾��Ϣ-------->> %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\r\n",RS485_RX_BUF[4],\
				RS485_RX_BUF[4],RS485_RX_BUF[5],RS485_RX_BUF[6],RS485_RX_BUF[7],RS485_RX_BUF[8],RS485_RX_BUF[9],\
				RS485_RX_BUF[10],RS485_RX_BUF[11],RS485_RX_BUF[12],RS485_RX_BUF[13],RS485_RX_BUF[14],RS485_RX_BUF[15],\
				RS485_RX_BUF[16],RS485_RX_BUF[17],RS485_RX_BUF[18],RS485_RX_BUF[19]);
				break;
			}
		}
	}
	Check_485=0;
}
