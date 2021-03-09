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
#include "ir.h"
#include "string.h"
#include "includes.h"					//ucos ʹ��	

//���� ���ճ�ʼ��
void TIM3_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	//GPIOB6  ���ù���,����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);
	//��ʱ��TIM12����
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_Period=20000;
	TIM_TimeBaseInitStruct.TIM_Prescaler=84-1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);
	//���벶��
	TIM_ICInitStruct.TIM_Channel=TIM_Channel_1;	//ͨ��1
	TIM_ICInitStruct.TIM_ICFilter=0x03;					//��ʹ���˲���
	TIM_ICInitStruct.TIM_ICPolarity=TIM_ICPolarity_Rising;//�½��ز���
	TIM_ICInitStruct.TIM_ICPrescaler=TIM_ICPSC_DIV1;//����Ƶ
	TIM_ICInitStruct.TIM_ICSelection=TIM_ICSelection_DirectTI;//IC1
	TIM_ICInit(TIM3,&TIM_ICInitStruct);
	
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIM3, &TIM_ICInitStruct);

	TIM_ITConfig(TIM3,TIM_IT_CC1 | TIM_IT_Update,ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStruct);

	TIM_Cmd(TIM3,ENABLE);
}

/////////////////
uint8_t HOME_IR_R1_Data = 0;
uint8_t HOME_IR_R1_KeyVal = 0;
uint8_t HOME_IR_R1_Flag = 0;
uint32_t TIMOC1_val=0;
uint32_t TIMOC1_Data = 0;
uint8_t TIMOC1_sta = 0;
uint8_t TIMUpdateFlag = 0;
////////////////
uint8_t HOME_IR_R2_Data = 0;
uint8_t HOME_IR_R2_KeyVal = 0;
uint8_t HOME_IR_R2_Flag = 0;
uint32_t TIMOC2_val = 0;
uint32_t TIMOC2_Data = 0;
uint8_t TIMOC2_sta = 0;
///////////////
void TIM3_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();
#endif
	//�����ж�
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		//���ж�
		TIM_ClearFlag(TIM3,TIM_FLAG_Update);
		TIMUpdateFlag = 1;
	}
	//�����ж�
	if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		TIM_ClearFlag(TIM3,TIM_FLAG_CC1); //���ж�
		if(1 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6))//�����ز���
    {
      TIMUpdateFlag = 0;
      TIMOC1_val = TIM_GetCapture1(TIM3);
      TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Falling);//CC1P=1 ����Ϊ�½��ز���	
    }
		else
		{
			if(TIMUpdateFlag == 1)
			{
				TIMUpdateFlag=0;
				TIMOC1_Data = TIM_GetCapture1(TIM3) + 20000 - TIMOC1_val;//��ȡCCR1Ҳ������CC1IF��־λ
			}
			else
			{
				TIMOC1_Data = TIM_GetCapture1(TIM3) - TIMOC1_val;
			}
			TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Rising); //CC4P=0	����Ϊ�����ز���
			
			if(TIMOC1_Data >4000 && TIMOC1_Data<5000)
			{
				if(TIMOC1_sta == 9)
        {
          HOME_IR_R1_KeyVal = HOME_IR_R1_Data;
//					printf("HOME_IR_R1 = 0x0%x \r\n", HOME_IR_R1_KeyVal);
          HOME_IR_R1_Flag = 1;
        }
        TIMOC1_sta = 1;
        HOME_IR_R1_Data = 0;
				return ;
			}

			if(TIMOC1_sta != 0)
			{
				if(TIMOC1_Data > 460 && TIMOC1_Data < 660)			//560Ϊ��׼ֵ,560us
        {
          TIMOC1_sta++;
          HOME_IR_R1_Data <<= 1;	//����һλ.
          HOME_IR_R1_Data |= 0;	//���յ�0   
        }
        else if(TIMOC1_Data > 1580 && TIMOC1_Data < 1780)	//1680Ϊ��׼ֵ,1680us
        {
          TIMOC1_sta++;
          HOME_IR_R1_Data <<= 1;	//����һλ.
          HOME_IR_R1_Data |= 1;	//���յ�1
        }
			}
		}
	}
	/**********************************************************************************/
//  if(TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
//  {
//    TIM_ClearFlag(TIM3, TIM_IT_CC2);
//    
//    if(1 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7))//�����ز���
//    {
//      TIMUpdateFlag = 0;
//      TIMOC2_val = TIM_GetCapture2(TIM3);

//      TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���	
//    }
//    else //�½��ز���
//    {
//      if(1 == TIMUpdateFlag)
//      {
//        TIMUpdateFlag = 0;
//        TIMOC2_Data = TIM_GetCapture2(TIM3) + 20000 - TIMOC2_val;//��ȡCCR1Ҳ������CC1IF��־λ
//      }
//      else/**/
//      {
//        TIMOC2_Data = TIM_GetCapture2(TIM3) - TIMOC2_val;
//      }
//      TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Rising); //CC4P=0	����Ϊ�����ز���
//      
//      
//      if(TIMOC2_Data >4000 && TIMOC2_Data<5000)		//4500Ϊ��׼ֵ4.5ms  �ɹ����յ���������
//      {
//        if(TIMOC2_sta == 9)
//        {
//            HOME_IR_R2_Flag = 1;
//            HOME_IR_R2_KeyVal = HOME_IR_R2_Data;
//						printf("HOME_IR_R2 = 0x0%x \r\n", HOME_IR_R2_KeyVal);
//        }
//        TIMOC2_sta = 1;
//        HOME_IR_R2_Data = 0;
//        return;
//        /**/
//      }
//      
//      if(TIMOC2_sta != 0)
//      {
//        
//        if(TIMOC2_Data > 460 && TIMOC2_Data < 660)			//560Ϊ��׼ֵ,560us
//        {
//          TIMOC2_sta++;
//          HOME_IR_R2_Data <<= 1;	//����һλ.
//          HOME_IR_R2_Data |= 0;	//���յ�0	   
//        }
//        else if(TIMOC2_Data > 1580 && TIMOC2_Data < 1780)	//1680Ϊ��׼ֵ,1680us
//        {
//          TIMOC2_sta++;
//          HOME_IR_R2_Data <<= 1;	//����һλ.
//          HOME_IR_R2_Data |= 1;	//���յ�1
//        }
//        else
//        {
//					
//        }
//      
//			}
//		}
//	}
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();
#endif
}

