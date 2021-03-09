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
#include "includes.h"

/* 定义变量 */
u8 CAN1Sedbuf[8];
u8 CAN1Redbuf[8];
u8 CAN1_DATAsize; 
u16 CAN1_ID;
u8 CAN1_REDBUFF = 0;
/* 速度模式PV： */
u32 PV_spd;
u32 PP_spd;
u32 PT_spd;

CanTxMsg TxMes1;			/* CAN1定义消息发送结构体 */
CanRxMsg RxMes1;			/* CAN1定义消息接收结构体 */

u16 Encoder_num;			/* 读取编码器数据 */

/* SDO CMD */
#define  SDO_W1   0x2F
#define  SDO_W2   0x2B
#define  SDO_W4   0x23
#define  SDO_RD   0x40
/*  速度模式 */
#define PP_Mode  1
#define PV_Mode  3 
#define PT_Mode  4 
/* 左右轮ID */
#define  Left_Wheel_ID              0x0001
#define  Right_Wheel_ID             0x0003

u32 Last_CAN1_ID;

/* CAN1初始化 */
u8 CAN1_Mode_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	CAN_InitTypeDef        CAN_InitStructure;

	/* 使能相关时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                  											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	/* 初始化GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* 引脚复用映射配置 */
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
	/* CAN单元设置 */
	//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
	//则波特率为:42M/((6+7+1)*3)=1Mbps
	CAN_InitStructure.CAN_TTCM = DISABLE;	        //禁止时间触发通信模式（CAN硬件计数器激活用于发送/接邮箱生产的时间戳）
	CAN_InitStructure.CAN_ABOM = DISABLE;	        //软件自动离线管理（CAN硬件在总线关闭时的状态）	  
	CAN_InitStructure.CAN_AWUM = DISABLE;         //0：睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)；1：睡眠模式硬件唤醒。
	CAN_InitStructure.CAN_NART = ENABLE;	        //0：CAN硬件将自动重发，直到成功；1：禁止报文自动传送（只发1次）
	CAN_InitStructure.CAN_RFLM = DISABLE;	        //0：FIFO报文溢出后不锁定,新的覆盖旧的；1：FIFO报文溢出后锁定，下1条将丢失。
	CAN_InitStructure.CAN_TXFP = DISABLE;	        //0：优先级由报文标识符决定；1：优先级由请求顺序决定。
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //正常模式
	CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;	    //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 1tq ~ 4tq（位变化在此阶段发生）
	CAN_InitStructure.CAN_BS1  = CAN_BS1_7tq;     //Tbs1范围 1tq ~ 16tq（定义采样点的位置）
	CAN_InitStructure.CAN_BS2  = CAN_BS2_6tq;     //Tbs2范围 1tq ~ 8tq６（定义发送点的位置）
	CAN_InitStructure.CAN_Prescaler = 3;        	//分频系数(Fdiv)为brp+1（波特预率分频1-1024）	bps=1000
	CAN_Init(CAN1, &CAN_InitStructure);           // 初始化CAN1
	/* 配置过滤器 */
	/* 只接收0x581和0x583，因为返回的ID为0x580+ID，此为运动信息返回 */
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)0x0581<<21)&0xffff0000)>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=(((u32)0x0581<<21)&0xffff0000)>>16;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	/* 开启接收0x701和0x703，此为上电信息返回 */
	CAN_FilterInitStructure.CAN_FilterNumber=1;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)0x0701<<21)&0xffff0000)>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=(((u32)0x0701<<21)&0xffff0000)>>16;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	/* 开启接收0x081，此为掉电信息返回 */
	CAN_FilterInitStructure.CAN_FilterNumber=2;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)0x0081<<21)&0xffff0000)>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=(((u32)0x0081<<21)&0xffff0000)>>16;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	/* PV配置 使能电机 */
	CANopen_PV_Init();
	return 0;
}
/* CAN1的发送函数 */
u8 CAN1_Send(u16 Id,u8 CAN1_DLC)
{
	u8 mail;
	u16 i=0;
	TxMes1.StdId = Id;	         	//标准标识符ID
	TxMes1.ExtId = Id;	         	//扩展标示符ID（29位）
	TxMes1.IDE = CAN_Id_Standard; //使用标准贞
	TxMes1.RTR = CAN_RTR_Data;		//消息类型为数据帧；/遥控帧
	TxMes1.DLC = CAN1_DLC;		    // 	1·报文长度	
	for(i=0;i<CAN1_DLC;i++)
	TxMes1.Data[i]=CAN1Sedbuf[i];	// 第一帧信息          
	mail = CAN_Transmit(CAN1, &TxMes1);   
	i=0;
	/* 等待发送结束 */
	while((CAN_TransmitStatus(CAN1, mail)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;
	if(i>=0XFFF)return 1;
	return 0;
}
//CAN1的接受函数
void CAN1_Read(void)
{
	u32 i;
	/* 没有接收到数据,直接退出  */
  if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)	return;
	CAN_Receive(CAN1, CAN_FIFO0, &RxMes1);							/* 读取数据	*/
	for(i=0;i<RxMes1.DLC;i++)
  CAN1Redbuf[i] = RxMes1.Data[i];
	CAN1_DATAsize = RxMes1.DLC;
	CAN1_ID       = RxMes1.StdId;
	CAN1_REDBUFF  = 1;
	/* 打印报文信息，用以调试 */
//	printf("CAN1_ID = %x | CAN1_DATAsize = %d | CAN1Redbuf = ",CAN1_ID,CAN1_DATAsize);
//	for(i=0;i<RxMes1.DLC;i++)
//		printf("%02x ",CAN1Redbuf[i]);
//	printf("\r\n");
	/* 对报文信息进行处理 */
	if(CAN1_ID == 0x0081)
	{
		/* 检测到从站断电Last_CAN1_ID==701&& */
		CAN_Message_Flag=1;
	}
	if(Last_CAN1_ID==0x0701&&CAN1_ID== 0X0703)
	{
		/* 检测到从站上电 MCU复位 */
		NVIC_SystemReset();
	}
	Last_CAN1_ID=CAN1_ID;
}
/* 模式设置 */
u8 Contol_Mode_SET(u8 CANopen_ID,u8 CANopen_mode)
{
	 CAN1Sedbuf[0]=SDO_W1;
	 CAN1Sedbuf[1]=0x60;
   CAN1Sedbuf[2]=0x60;
	 CAN1Sedbuf[3]=0x00;
	 CAN1Sedbuf[4]=CANopen_mode;
	 CAN1Sedbuf[5]=0x00;
	 CAN1Sedbuf[6]=0x00;
	 CAN1Sedbuf[7]=0x00;	
	 CAN1_Send(0x600 + CANopen_ID,8);
	 delay_ms(5);
	 return (1);
} 
/* 激活节点 */
u8 CANopen_Activate(u8 CANopen_ID){
	 
	 CAN1Sedbuf[0]=0x01;
	 CAN1Sedbuf[1]=CANopen_ID;	
	 CAN1_Send(0x000,2);
	 delay_ms(5);
	 return(1);
} 
/* 写数据 */
u8 SDO_Write_OD(u8 CANopen_ID,u8 CMD, u16 Index, u8 SubIndex, u32 DATA)
{
   CAN1Sedbuf[0]=CMD;
	 CAN1Sedbuf[1]=(u8)(Index    & 0xFF);
   CAN1Sedbuf[2]=(u8)(Index>>8 & 0xFF);
	 CAN1Sedbuf[3]=SubIndex;
	 CAN1Sedbuf[4]=(u8)(DATA     & 0xFF);
	 CAN1Sedbuf[5]=(u8)(DATA>>8  & 0xFF);
	 CAN1Sedbuf[6]=(u8)(DATA>>16 & 0xFF);
	 CAN1Sedbuf[7]=(u8)(DATA>>24 & 0xFF);	
	 CAN1_Send(0x600 + CANopen_ID,8);
	 delay_ms(3);
   return (1);
}
/* 协议速度配置 */
void CANopen_PV_Init(void)
{
	/* STEP1:激活节点1、节点2 */
	CANopen_Activate( Left_Wheel_ID  );
	CANopen_Activate( Right_Wheel_ID );

	/* STEP2:设置速度模式	6060H写为3 */
	Contol_Mode_SET( Left_Wheel_ID,  PV_Mode );
	Contol_Mode_SET( Right_Wheel_ID, PV_Mode );

	/* STEP3:设置加减速	写6083H和6084H */
	SDO_Write_OD( Left_Wheel_ID, SDO_W4, 0x6083,0x00,0x00003A98);
	SDO_Write_OD( Left_Wheel_ID, SDO_W4, 0x6084,0x00,0x00003A98);

	SDO_Write_OD( Right_Wheel_ID,SDO_W4, 0x6083,0x00,0x00003A98);
	SDO_Write_OD( Right_Wheel_ID,SDO_W4, 0x6084,0x00,0x00003A98);

	/* STEP4:设置目标转速为0	写60FFH */
	SDO_Write_OD( Left_Wheel_ID, SDO_W4, 0x60FF,0x00,0x00000000);
	SDO_Write_OD( Right_Wheel_ID,SDO_W4, 0x60FF,0x00,0x00000000);

	/* 电机使能 */
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000006);
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000007);
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x0000000F);
	
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000006);
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000007);
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x0000000F);
}
/* 速度的设置 */
void CANopen_PV_SET(u32 Acc,u32 Dec,s32 Tageviy_L,s32 Tageviy_R)
{
	/* STEP1:设置加减速时间	写6083H和6084H */	 
//	 SDO_Write_OD( Left_Wheel_ID, SDO_W4,0x6083,0x00,0x000003e8);
//   SDO_Write_OD( Left_Wheel_ID, SDO_W4,0x6084,0x00,0x000003e8);
//	 
//	 SDO_Write_OD( Right_Wheel_ID,SDO_W4,0x6083,0x00,0x000003e8);
//	 SDO_Write_OD( Right_Wheel_ID,SDO_W4,0x6084,0x00,0x000003e8);
	 
	/* STEP2:设置目标转速 	写60FFH */
	if(Tageviy_L>1500)
		Tageviy_L=1500;
	else if(Tageviy_L<-1500)
		Tageviy_L=-1500;
	if(Tageviy_R>1500)
		Tageviy_R=1500;
	else if(Tageviy_R<-1500)
		Tageviy_R=-1500;
	
	SDO_Write_OD( Left_Wheel_ID, SDO_W4,0x60FF,0x00,Tageviy_L);
	SDO_Write_OD( Right_Wheel_ID,SDO_W4,0x60FF,0x00,Tageviy_R);
}
/* 电机失能 */
void Motor_Disenable(void)
{
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000007);
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000007);
}
/* 电机使能 */
void Motor_Enable(void)
{
	/* 电机使能 */
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000006);
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000007);
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x0000000F);
	
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000006);
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000007);
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x0000000F);
}
/* 读取实际A电机转速 */
u16 Encoder_ReadA(void)
{
	SDO_Write_OD( Left_Wheel_ID, SDO_RD,0x6063,0x00,0x00000000);
	CAN1_Read();
	Encoder_num = CAN1Redbuf[4] | CAN1Redbuf[5]<<8 | CAN1Redbuf[6]<<16 | CAN1Redbuf[7]<<24;
	return Encoder_num;
}
/* 读取实际B电机转速 */
u16 Encoder_ReadB(void)
{
	SDO_Write_OD( Right_Wheel_ID, SDO_RD, 0x6063,0x00,0x00000000);
	CAN1_Read();
	Encoder_num = CAN1Redbuf[4] | CAN1Redbuf[5]<<8 | CAN1Redbuf[6]<<16 | CAN1Redbuf[7]<<24;
	return Encoder_num;
}
/* 电机速度给零 */
void Motor_PV_Zero(void)
{
   SDO_Write_OD( Left_Wheel_ID, SDO_W4,0x60FF,0x00,0x00000000);
	 SDO_Write_OD( Right_Wheel_ID,SDO_W4,0x60FF,0x00,0x00000000);
}
/***************************************************************/
/*
void Motor_PV_Go(void)
{
//写0x60FF速度值
   SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x60FF,0x00,PV_spd);
	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x60FF,0x00,PV_spd);	
}
void Motor_PV_Left(void)
{ 
//写0x60FF速度值
	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x60FF,0x00,~PV_spd+1);
	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x60FF,0x00,PV_spd);	
}
void Motor_PV_Right(void)
{
//写0x60FF速度值
	SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x60FF,0x00,PV_spd);
	SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x60FF,0x00,~PV_spd+1);
}
void Motor_PV_Back(void)
{
//写0x60FF速度值
	SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x60FF,0x00,~PV_spd+1);
	SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x60FF,0x00,~PV_spd+1);
}*/
