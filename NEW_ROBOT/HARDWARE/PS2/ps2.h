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
#ifndef __PS2_H_
#define __PS2_H_

#include "sys.h"
#include "includes.h"
      
/* �ֱ�IO�� */
#define DI   PBin(12)         //PC2  ����
#define DO_H PBout(13)=1      //����λ��
#define DO_L PBout(13)=0      //����λ��
#define CS_H PCout(8)=1       //CS����
#define CS_L PCout(8)=0       //CS����
#define CLK_H PCout(9)=1      //ʱ������
#define CLK_L PCout(9)=0      //ʱ������

/* �ֱ������� */
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

//#define WHAMMY_BAR		8

/* These are stick values */
#define PSS_RX 5
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;

void PS2_Init(void);
u8 PS2_RedLight(void);   	//�ж��Ƿ�Ϊ���ģʽ
void PS2_ReadData(void); 	//���ֱ�����
void PS2_Cmd(u8 CMD);		  //���ֱ���������
u8 PS2_DataKey(void);		  //����ֵ��ȡ
u8 PS2_AnologData(u8 button); //�õ�һ��ҡ�˵�ģ����
void PS2_ClearData(void);	  //������ݻ�����
void PS2_Vibration(u8 motor1, u8 motor2);//������motor1  0xFF���������أ�motor2  0x40~0xFF

void PS2_EnterConfing(void);	 		//��������
void PS2_TurnOnAnalogMode(void); 	//����ģ����
void PS2_VibrationMode(void);    	//������
void PS2_ExitConfing(void);	     	//�������
void PS2_SetInit(void);		     		//���ó�ʼ��
void PS2_Receive (void);
#endif