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
#include <string.h>
/* 定义变量 -S */
u8 Flag_Target;        //相关标志位
u16 num_A,num_B,last_num_A,last_num_B;	//读取到的编码器信息
int last_UA_Encoder,last_UB_Encoder;	//保存上一次编码器信息,方便滤波
/* 定义变量 -E */
/*
函数功能：40ms软件定时器1的回调函数
中断触发严格保证采样和数据处理的时间同步
*/
void ctrl_tim_callback(OS_TMR *ptmr,void *p_arg)
{
	/* 接到串口遥控解锁指令之后，使能串口控制输入 */
	if(Usart_ON_Flag==1||PS2_ON_Flag==1)	Usart_Control();
	/* 红外测距的防悬崖跌落 */
	if(Flag_Stop==1&&Tageviy_L>0&&Tageviy_R<0)
		Tageviy_L=Tageviy_R=0;
	/* 写入速度 */
	CANopen_PV_SET(0,0,Tageviy_L,Tageviy_R);
	/* 40ms控制一次，为了保证M法测速的时间基准 */
	Flag_Target=!Flag_Target;
	if(Flag_Target)
	{
		last_num_A = Encoder_ReadA();
		last_num_B = Encoder_ReadB();
		Data_Process2();
		return ;
	}
	else if(!Flag_Target)
	{
		/* 时隔40ms，再次读取编码器的值 */
		num_A = Encoder_ReadA();
		num_B = Encoder_ReadB();
		/* 数据处理 */
		Data_Process();
		return ;
	}
}
/*
 *函数功能：编码器数据处理，处理脉冲数
 *入口参数：无
 *返回  值：无
 *后续可以改为 UA_Encoder=abs(num_A-last_num_A)
*/
void Data_Process(void)
{
	UA_Encoder = num_A-last_num_A;
	UB_Encoder = num_B-last_num_B;
	/* 过滤峰值 */
	if(abs(UA_Encoder)>1000)		UA_Encoder=last_UA_Encoder;
	if(abs(UB_Encoder)>1000)		UB_Encoder=last_UB_Encoder;
	/* 终端打印 */
//	printf("UA_Encoder = %d UB_Encoder = %d\r\n",UA_Encoder,UB_Encoder);
	USART_TX();
	/* 保存上一次的数据 */
	last_UA_Encoder=UA_Encoder,last_UB_Encoder=UB_Encoder;
}
void Data_Process2(void)
{
	UA_Encoder=last_num_A-num_A;
	UB_Encoder=last_num_B-num_B;
	/* 过滤峰值 */
	if(abs(UA_Encoder)>1000)		UA_Encoder=last_UA_Encoder;
	if(abs(UB_Encoder)>1000)		UB_Encoder=last_UB_Encoder;
	/* 终端打印 */
//	printf("UA_Encoder = %d UB_Encoder = %d\r\n",UA_Encoder,UB_Encoder);
	USART_TX();
	/* 保存上一次的数据 */
	last_UA_Encoder=UA_Encoder,last_UB_Encoder=UB_Encoder;
}
/*
 *函数功能：小车运动数学模型
 *入口参数：X Y三轴速度或者位置
 *返回  值：无
*/
#define a_PARAMETER          (0.275f)
void Kinematic_Analysis(float Vy,float Vz)
{
	Tageviy_L = +Vy+Vz*(a_PARAMETER);
	Tageviy_R	= -Vy+Vz*(a_PARAMETER);
}
/*
 *函数功能：接收串口和PS2手柄控制指令进行处理
 *入口参数：无
 *返回  值：无
*/
int flag_1,flag_2,RX,LY,Yuzhi=40;
u8 Check_Sum;
void Usart_Control(void)
{
	Check_Sum=(0X80+0X90+Urxbuf[0]+Urxbuf[1]+Urxbuf[2]+Urxbuf[3]+Urxbuf[4]+Urxbuf[5]\
	+Urxbuf[6]+Urxbuf[7]);
	/* 串口接收到的指令解码 */
	if((Usart_ON_Flag==1) && (Check_Sum==Urxbuf[8]))
	{
		if(!bump_flag)
		{
			if(Urxbuf[0]>0)			flag_1=	1;
			else 								flag_1=-1;
			if(Urxbuf[4]>0)			flag_2=-1;
			else 								flag_2=	1;

			Tageviy_L	= (Urxbuf[3] | Urxbuf[2]<<8 | Urxbuf[1]<<16)*flag_1*150/56;	
			Tageviy_R= (Urxbuf[7] | Urxbuf[6]<<8 | Urxbuf[5]<<16)*flag_2*150/56;
		}
		else
		{
			bump++;
			Tageviy_L= -100,Tageviy_R=100;
		}
	}
	/* 手柄指令解码 */
	else if (PS2_ON_Flag==1)
	{
		RX=PS2_RX-127;
		LY=PS2_LY-127;
		/* 过滤掉20以内的模拟值 */
		if(RX>-Yuzhi && RX<Yuzhi)		RX=0;
		if(LY>-Yuzhi && LY<Yuzhi)		LY=0;
		
		Move_Z=  RX*RC_Velocity	/15;
		Move_Y= -LY*RC_Velocity	/10;
		/* 保证同步 */
		if(Move_Y<0)	Move_Z=-Move_Z;
		/* 得到控制目标值，进行运动学分析 */		
		Kinematic_Analysis(Move_Y*0.7882f,Move_Z*1.3757f);
	}
//	memset(Urxbuf,0,sizeof(u8)*8);
//	Urxbuf[8]=0x10;
}
/* 通过串口把自身的运动数据发送给上位机 */
void USART_TX(void)
{
	u8 Direction_A,Direction_B,CheckSum;
	if(UA_Encoder>=0) 			Direction_A=0X01;
	else if(UA_Encoder<0) 	Direction_A=0X00;
	if(UB_Encoder>=0) 			Direction_B=0X00;
	else if(UB_Encoder<0) 	Direction_B=0X01;
	/* 累加的校验和 */
	CheckSum=( 0x80+0x90+Direction_A+(abs(UA_Encoder)>>16)+(abs(UA_Encoder)>>8) \
	+abs(UA_Encoder)+Direction_B+(abs(UB_Encoder)>>16)+(abs(UB_Encoder)>>8)+abs(UB_Encoder) );
	/* 发送到串口1 */
	usart1_send(0x80);
	usart1_send(0x90);
	usart1_send(Direction_A);
	usart1_send(abs(UA_Encoder)>>16);
	usart1_send(abs(UA_Encoder)>>8);
	usart1_send(abs(UA_Encoder));
	usart1_send(Direction_B);
	usart1_send(abs(UB_Encoder)>>16);
	usart1_send(abs(UB_Encoder)>>8);
	usart1_send(abs(UB_Encoder));
	usart1_send(CheckSum);
}

