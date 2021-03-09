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
#include "bump.h"
/*
函数功能：防撞条IO初始化
入口参数：无
返回  值：无 
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
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击 
*/
u8 click(void)
{
	//一直按下一直命令
	/*
//	static u8 flag_key=1;//按键按松开标志
	if(KEY==0)
	{
//		flag_key=0;flag_key=1
		return 1;	// 按键按下
	}
//	else if(KEY==1)	;
	return 0;//无按键按下
	*/
	/* 一直按下一次命令 */
	static u8 flag_key=1;
	if(flag_key&&BUMP_KEY==0)
	{
		flag_key=0;
		/* 按键按下 */
		return 1;
	}
	else if(1==BUMP_KEY)	flag_key=1;
	/* 无按键按下 */
	return 0;
}
