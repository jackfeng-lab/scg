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
#include "bump.h"
/*
�������ܣ���ײ��IO��ʼ��
��ڲ�������
����  ֵ���� 
*/
void BUMP_Init(void)
{
 	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*
�������ܣ�����ɨ��
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������ 
*/
u8 click(void)
{
	//һֱ����һֱ����
	/*
//	static u8 flag_key=1;//�������ɿ���־
	if(KEY==0)
	{
//		flag_key=0;flag_key=1
		return 1;	// ��������
	}
//	else if(KEY==1)	;
	return 0;//�ް�������
	*/
	/* һֱ����һ������ */
	static u8 flag_key=1;
	if(flag_key&&BUMP_KEY==0)
	{
		flag_key=0;
		/* �������� */
		return 1;
	}
	else if(1==BUMP_KEY)	flag_key=1;
	/* �ް������� */
	return 0;
}
