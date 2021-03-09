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
#include "dxiic.h"
/*
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*/
void DXIIC_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOB时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
	DXIIC_SCL=1;
	DXIIC_SDA=1;
}

u8 KS103_ReadOneByte(u8 address, u8 reg)
{
	u8 temp=0;
	DXIIC_Start();
	DXIIC_Send_Byte(address); //发送低地址
	DXIIC_Wait_Ack();
	DXIIC_Send_Byte(reg); //发送低地址
	DXIIC_Wait_Ack();
	DXIIC_Start();
	DXIIC_Send_Byte(address+1); //进入接收模式
	DXIIC_Wait_Ack();
	delay_us(50); //增加此代码通信成功！！！
	temp=DXIIC_Read_Byte(0); //读寄存器 3
	DXIIC_Stop();//产生一个停止条件
	return temp;
}

void KS103_WriteOneByte(u8 address,u8 reg,u8 command)
{
	DXIIC_Start();
	DXIIC_Send_Byte(address); //发送写命令
	DXIIC_Wait_Ack();
	DXIIC_Send_Byte(reg);//发送高地址
	DXIIC_Wait_Ack();
	DXIIC_Send_Byte(command); //发送低地址
	DXIIC_Wait_Ack();
	DXIIC_Stop();//产生一个停止条件
}

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
int DXIIC_Start(void)
{
	DXSDA_OUT();     //sda线输出
	DXIIC_SDA=1;
	DXIIC_SCL=1;
	delay_us(10);
 	DXIIC_SDA=0;
	delay_us(10);
	DXIIC_SCL=0;//钳住I2C总线，准备发送或接收数据
	return 1;
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
void DXIIC_Stop(void)
{
	DXSDA_OUT();//sda线输出
	DXIIC_SCL=0;
	DXIIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(10);
	DXIIC_SCL=1;
	DXIIC_SDA=1;//发送I2C总线结束信号
	delay_us(10);						   	
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
int DXIIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	DXSDA_IN();      //SDA设置为输入  
	DXIIC_SDA=1;delay_us(6);	   
	DXIIC_SCL=1;delay_us(6);	 
	while(DXREAD_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			DXIIC_Stop();
			return 0;
		}
	}
	DXIIC_SCL=0;//时钟输出0 	   
	return 1;  
} 

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
void DXIIC_Ack(void)
{
	DXIIC_SCL=0;
	DXSDA_OUT();
	DXIIC_SDA=0;
	delay_us(10);
	DXIIC_SCL=1;
	delay_us(10);
	DXIIC_SCL=0;
}
	
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/	    
void DXIIC_NAck(void)
{
	DXIIC_SCL=0;
	DXSDA_OUT();
	DXIIC_SDA=1;
	delay_us(10);
	DXIIC_SCL=1;
	delay_us(10);
	DXIIC_SCL=0;
}
/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
void DXIIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		DXSDA_OUT(); 	    
    DXIIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
      DXIIC_SDA=(txd&0x80)>>7;
      txd<<=1; 	  
			delay_us(10);   
			DXIIC_SCL=1;
			delay_us(10); 
			DXIIC_SCL=0;	
			delay_us(10);
    }	 
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
u8 DXIIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	DXSDA_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
	{
		DXIIC_SCL=0; 
		delay_us(10);
		DXIIC_SCL=1;
		receive<<=1;
		if(DXREAD_SDA)receive++;   
		delay_us(5); 
   }					 
    if (ack)
        DXIIC_Ack(); //发送ACK 
    else
        DXIIC_NAck();//发送nACK  
    return receive;
}

//------------------End of File----------------------------
