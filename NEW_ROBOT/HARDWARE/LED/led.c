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
#include "led.h" 
/* LED IO��ʼ�� */
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* GPIOA8��ʼ������ */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* GPIOA8���� */
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
}
