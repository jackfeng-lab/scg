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
#ifndef __CTRL_H
#define __CTRL_H

#include "includes.h"	 

void Kinematic_Analysis(float Vy,float Vz);
void Usart_Control(void);
void Data_Process(void);
void Data_Process2(void);
void USART_TX(void);

#endif
