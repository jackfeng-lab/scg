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
 *版权所有 盗版必究
*/
#include "sys.h"
#include "includes.h"
/* 定义变量 -S */
u8 USART2_RX_BUFFER_SIZE=64;
u16 Check_485=0;
u8 Usart2_temp,n;
u8 rs485buf_0[7]={0XDD,0XA5,0X03,0X00,0XFF,0XFD,0X77};    /* 读取指令电池信息 */
/* 定义变量 -E */

/* 接收使能标志位 */
#if EN_USART2_RX
/* 串口2中断服务函数 */
void USART2_IRQHandler(void)
{
	#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();
	#endif
	if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET)
	{
		/* 软件序列清除IDLE标志位 */
		USART2->SR;
		USART2->DR;
		/* 关闭DMA，准备重新配置 */
		DMA_Cmd(DMA1_Stream5, DISABLE);
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
		/* 清除传输完成标志 */
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TEIF5);
		/* 清除传输错误标志 */
	}
	/* 开启下一次DMA接收 */
	DMA_Enable(DMA1_Stream5,USART2_RX_BUFFER_SIZE);
	#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();
	#endif
}
#endif
/*
RS485+DMA接收初始化函数
bound:波特率
复用串口2
*/ 
void RS485_Init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	/* 打开相关时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
  /* 串口2引脚复用映射 */
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	/* 串口2 IO配置 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	/* 485控制引脚配置，485模式控制 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  /* USART2 初始化设置 */
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
	/* 使能串口2 */
  USART_Cmd(USART2, ENABLE);
#if EN_USART2_RX	
	/* Usart2 NVIC配置 */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//开启空闲中断
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);//开启DMA接收
	USART_Cmd(USART2, ENABLE);
	/* DMA初始化 */
  DMA_DeInit(DMA1_Stream5);
	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE){}//等待DMA可配置 
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel 						= DMA_Channel_4;  							//通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&(USART2->DR);			//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)RS485_RX_BUF;				//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR 					    	= DMA_DIR_PeripheralToMemory;		//direction of transmit.
  DMA_InitStructure.DMA_BufferSize 				  = USART2_RX_BUFFER_SIZE;				//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc				= DMA_PeripheralInc_Disable;		//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc 					= DMA_MemoryInc_Enable;					//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode 								= DMA_Mode_Normal;							// 使用普通模式 
  DMA_InitStructure.DMA_Priority 						= DMA_Priority_High;						//中等优先级
  DMA_InitStructure.DMA_FIFOMode 						= DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;				//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;		//外设突发单次传输
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	/* 使能DMA1，并且默认为接收模式 */
  DMA_Cmd(DMA1_Stream5,ENABLE);
	RS485_TX_EN=0;
}
/* 电池信息轮训查询函数 */
void Rs485_Flash(u16 time)
{
	static int rs485_temp;
//	static u8 ch,last_ch;
	if(++rs485_temp==time)
	{
		/* 发送电池信息获取指令 */
		RS485_Send_Data(rs485buf_0,sizeof(rs485buf_0));
		/* 打印电池信息 */
//		RS485_Receive_Data();
		/* 大于35亮绿蓝 小于20亮红灯 低于35亮红蓝 */
		if(RS485_RX_BUF[23]>35)
			R_LED=1,G_LED=0,B_LED=1; 
		else if(RS485_RX_BUF[23]<20)
			R_LED=1,G_LED=1,B_LED=0;
		else if(RS485_RX_BUF[23]<35)
			R_LED=1,G_LED=0,B_LED=0;
		/* OLED 屏幕的显示 */
//			OLED_ShowString(00,10,(u8*)"Elec:");
//			OLED_ShowNumber(50,10,RS485_RX_BUF[23],3,12);
//			OLED_ShowString(65,10,(u8*)"%");
//			OLED_Refresh_Gram();
		rs485_temp=0;
	}
}
/* 开启一次DMA传输 */
void DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
	DMA_Cmd(DMA_Streamx, DISABLE);                      //先关闭DMA,才能设置它
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//等待传输结束
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          	//设置传输数据长度 
	DMA_Cmd(DMA_Streamx, ENABLE);                      	//开启DMA
}
/* 
RS485指令发送函数
buf:发送区首地址
len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
*/
void RS485_Send_Data(u8 *buf,u8 len)
{
	/* 设置为发送模式 */
	RS485_TX_EN=1;
	/* 循环发送数据 */
	for(n=0;n<len;n++)
	{
		/* 等待发送结束 */
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
		/* 发送数据 */
		USART_SendData(USART2,buf[n]);
	}
	/* 等待发送结束 设置为接收模式 */
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
	RS485_TX_EN=0;
}
u8 high_bit,low_bit;
/* 打印电池信息 */
void RS485_Receive_Data(void)
{
	/* 累加取反加1 得到校验码 */
	for(n=2;n<31;n++)
		Check_485 += RS485_RX_BUF[n];
	Check_485 = (~Check_485)+1;
	/* 校验通过，打印信息 */
	if(Check_485 == (RS485_RX_BUF[31]*256+RS485_RX_BUF[32]))
	{
		switch(RS485_RX_BUF[1])
		{
			case 0x03:
			{
				low_bit = RS485_RX_BUF[7];
				high_bit = RS485_RX_BUF[6];
				printf("********************************\r\n");
				printf("总电压---------->> %d mV\r\n", 	(RS485_RX_BUF[4]*256+RS485_RX_BUF[5])*10);
				
				if(((high_bit&0x80)>>7))
				{
					printf("放电中\r\n");
					high_bit = ((~high_bit)+1);
					low_bit = ((~low_bit)+1);
				}
				else if(!((high_bit&0x80)>>7))
				{
					if(low_bit)
						printf("充电中\r\n");
					else
						printf("放电中\r\n");
				}
				
//				printf("%d\r\n", 	(RS485_RX_BUF[6]&0x80)>>7); && low_bit  && !((low_bit&0x80)>>7)   && ((low_bit&0x80)>>7)
				printf("电  流---------->> %d mA\r\n", 	(high_bit*256+low_bit)*10);
//				printf("剩余容量-------->> %d mAh\r\n",	(RS485_RX_BUF[8]*256+RS485_RX_BUF[9])*10);
//				printf("标称容量-------->> %d mAh\r\n",	(RS485_RX_BUF[10]*256+RS485_RX_BUF[11])*10);
//				printf("循环次数-------->> %d 次/Cycle\r\n",(RS485_RX_BUF[12]*256+RS485_RX_BUF[13])*10);
//				printf("生产日期-------->> %d年%d月%d日\r\n",2000+((RS485_RX_BUF[14]*256+RS485_RX_BUF[15])>>9),\
//				((RS485_RX_BUF[14]*256+RS485_RX_BUF[15])>>5)&0x0f,(RS485_RX_BUF[14]*256+RS485_RX_BUF[15])&0x1f);
//				printf("均衡状态_低串--->> %d \r\n",	RS485_RX_BUF[16]*256+RS485_RX_BUF[17]);
//				printf("均衡状态_高串--->> %d \r\n",	RS485_RX_BUF[18]*256+RS485_RX_BUF[19]);
//				printf("保护状态-------->> %d \r\n",	RS485_RX_BUF[20]*256+RS485_RX_BUF[21]);
//				printf("软件版本-------->> %d \r\n",	RS485_RX_BUF[22]);
//				printf("剩余容量-------->> %d %% \r\n",RS485_RX_BUF[23]);
//				printf("FET控制状态----->> %d \r\n",RS485_RX_BUF[24]);
//				printf("电池串数-------->> %d \r\n",RS485_RX_BUF[25]);
//				printf("NTC个数--------->> %d \r\n",RS485_RX_BUF[26]);
//				printf("NTC_1----------->> %d \r\n",((RS485_RX_BUF[27]*256+RS485_RX_BUF[28])-2731)/10);
//				printf("NTC_2----------->> %d \r\n",((RS485_RX_BUF[29]*256+RS485_RX_BUF[30])-2731)/10);
				break;
			}
			case 0x04:
			{
				printf("********************************\r\n");
				printf("电池串_1-------->> %d mV\r\n",RS485_RX_BUF[4]*256+RS485_RX_BUF[5]);
				printf("电池串_2-------->> %d mV\r\n",RS485_RX_BUF[6]*256+RS485_RX_BUF[7]);
				printf("电池串_3-------->> %d mV\r\n",RS485_RX_BUF[8]*256+RS485_RX_BUF[9]);
				printf("电池串_4-------->> %d mV\r\n",RS485_RX_BUF[10]*256+RS485_RX_BUF[11]);
				printf("电池串_5-------->> %d mV\r\n",RS485_RX_BUF[12]*256+RS485_RX_BUF[13]);
				printf("电池串_6-------->> %d mV\r\n",RS485_RX_BUF[14]*256+RS485_RX_BUF[15]);
				printf("电池串_7-------->> %d mV\r\n",RS485_RX_BUF[16]*256+RS485_RX_BUF[17]);
				printf("电池串_8-------->> %d mV\r\n",RS485_RX_BUF[18]*256+RS485_RX_BUF[19]);
				break;
			}
			case 0x05:
			{
				printf("********************************\r\n");
				printf("电池版本信息-------->> %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\r\n",RS485_RX_BUF[4],\
				RS485_RX_BUF[4],RS485_RX_BUF[5],RS485_RX_BUF[6],RS485_RX_BUF[7],RS485_RX_BUF[8],RS485_RX_BUF[9],\
				RS485_RX_BUF[10],RS485_RX_BUF[11],RS485_RX_BUF[12],RS485_RX_BUF[13],RS485_RX_BUF[14],RS485_RX_BUF[15],\
				RS485_RX_BUF[16],RS485_RX_BUF[17],RS485_RX_BUF[18],RS485_RX_BUF[19]);
				break;
			}
		}
	}
	Check_485=0;
}
