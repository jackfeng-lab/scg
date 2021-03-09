#ifndef __DXIIC_H_
#define __DXIIC_H_

#include "sys.h"
#include "includes.h"
  	   		   
/* IO方向设置 */
#define DXSDA_IN()  {GPIOC->MODER&=~(3<<(5*2));GPIOC->MODER|=0<<5*2;}	//PB9输入模式
#define DXSDA_OUT() {GPIOC->MODER&=~(3<<(5*2));GPIOC->MODER|=1<<5*2;} //PB9输出模式
/* IO宏定义 */
#define DXIIC_SCL    PCout(4) //SCL
#define DXIIC_SDA    PCout(5) //SDA	 
#define DXREAD_SDA   PCin(5)  //输入SDA 

/* 函数声明 */
void DXIIC_Init(void);                //初始化IIC的IO口				 
int DXIIC_Start(void);				//发送IIC开始信号
void DXIIC_Stop(void);	  			//发送IIC停止信号
void DXIIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 DXIIC_Read_Byte(unsigned char ack);//IIC读取一个字节
int DXIIC_Wait_Ack(void); 				//IIC等待ACK信号
void DXIIC_Ack(void);					//IIC发送ACK信号
void DXIIC_NAck(void);				//IIC不发送ACK信号

u8 KS103_ReadOneByte(u8 address, u8 reg);
void KS103_WriteOneByte(u8 address,u8 reg,u8 command);

#endif
