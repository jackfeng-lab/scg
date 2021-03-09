#ifndef __DXIIC_H_
#define __DXIIC_H_

#include "sys.h"
#include "includes.h"
  	   		   
/* IO�������� */
#define DXSDA_IN()  {GPIOC->MODER&=~(3<<(5*2));GPIOC->MODER|=0<<5*2;}	//PB9����ģʽ
#define DXSDA_OUT() {GPIOC->MODER&=~(3<<(5*2));GPIOC->MODER|=1<<5*2;} //PB9���ģʽ
/* IO�궨�� */
#define DXIIC_SCL    PCout(4) //SCL
#define DXIIC_SDA    PCout(5) //SDA	 
#define DXREAD_SDA   PCin(5)  //����SDA 

/* �������� */
void DXIIC_Init(void);                //��ʼ��IIC��IO��				 
int DXIIC_Start(void);				//����IIC��ʼ�ź�
void DXIIC_Stop(void);	  			//����IICֹͣ�ź�
void DXIIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 DXIIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
int DXIIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void DXIIC_Ack(void);					//IIC����ACK�ź�
void DXIIC_NAck(void);				//IIC������ACK�ź�

u8 KS103_ReadOneByte(u8 address, u8 reg);
void KS103_WriteOneByte(u8 address,u8 reg,u8 command);

#endif
