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
#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"
#include "includes.h"   

/* CAN1����RX0�ж�ʹ�� 0,��ʹ��;1,ʹ�� */
#define CAN1_RX0_INT_ENABLE	1

/* �������� - S*/
u8 CAN1_Mode_Init(void);
void CANopen_PV_SET(u32 Acc,u32 Dec,s32 TargetVelocity_Lift,s32 TargetVelocity_Right);
void CANopen_PV_Init(void);
void Motor_Disenable(void);
void Motor_Enable(void);

void Motor_PV_Zero(void);
void CAN1_Read(void);
u8 CAN1_Send(u16 Id,u8 CAN1_DLC);
u16 Encoder_ReadA(void);
u16 Encoder_ReadB(void);
/* �������� - E*/			

#endif
