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
#include "dxiic.h"
/*
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*/
void DXIIC_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOBʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
	DXIIC_SCL=1;
	DXIIC_SDA=1;
}

u8 KS103_ReadOneByte(u8 address, u8 reg)
{
	u8 temp=0;
	DXIIC_Start();
	DXIIC_Send_Byte(address); //���͵͵�ַ
	DXIIC_Wait_Ack();
	DXIIC_Send_Byte(reg); //���͵͵�ַ
	DXIIC_Wait_Ack();
	DXIIC_Start();
	DXIIC_Send_Byte(address+1); //�������ģʽ
	DXIIC_Wait_Ack();
	delay_us(50); //���Ӵ˴���ͨ�ųɹ�������
	temp=DXIIC_Read_Byte(0); //���Ĵ��� 3
	DXIIC_Stop();//����һ��ֹͣ����
	return temp;
}

void KS103_WriteOneByte(u8 address,u8 reg,u8 command)
{
	DXIIC_Start();
	DXIIC_Send_Byte(address); //����д����
	DXIIC_Wait_Ack();
	DXIIC_Send_Byte(reg);//���͸ߵ�ַ
	DXIIC_Wait_Ack();
	DXIIC_Send_Byte(command); //���͵͵�ַ
	DXIIC_Wait_Ack();
	DXIIC_Stop();//����һ��ֹͣ����
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Start(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/
int DXIIC_Start(void)
{
	DXSDA_OUT();     //sda�����
	DXIIC_SDA=1;
	DXIIC_SCL=1;
	delay_us(10);
 	DXIIC_SDA=0;
	delay_us(10);
	DXIIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ��������
	return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
void DXIIC_Stop(void)
{
	DXSDA_OUT();//sda�����
	DXIIC_SCL=0;
	DXIIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(10);
	DXIIC_SCL=1;
	DXIIC_SDA=1;//����I2C���߽����ź�
	delay_us(10);						   	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/
int DXIIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	DXSDA_IN();      //SDA����Ϊ����  
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
	DXIIC_SCL=0;//ʱ�����0 	   
	return 1;  
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
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
	
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(u8 txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void DXIIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		DXSDA_OUT(); 	    
    DXIIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
u8 DXIIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	DXSDA_IN();//SDA����Ϊ����
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
        DXIIC_Ack(); //����ACK 
    else
        DXIIC_NAck();//����nACK  
    return receive;
}

//------------------End of File----------------------------
