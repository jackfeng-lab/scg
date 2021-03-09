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
 *fromelf.exe --bin -o ./bin/NEW_ROBOT.bin ./../OBJ/NEW_ROBOT.axf ---固件升级 
*/
#include "sys.h"
#include "includes.h"
/* 激光测距通讯地址 */
#define ADDR_1 	0x10
#define ADDR_2	0x20

/* 定义变量 -S */
static u8 Num_Stop;
u8 Flag_Stop,Flag_Show,bump,bump_flag=0;					//停止标志位和 显示标志位 默认停止 显示打开
u8 Usart_Flag;  				//蓝牙遥控相关变量和运行状态标志位//延时相关变量
u8 Urxbuf[9],Usart_ON_Flag=1,PS2_ON_Flag=0;	//串口控制相关变量
float Move_Y,Move_Z;  						//三轴角度和XYZ轴目标速度
int RC_Velocity=3,UA_Encoder=0,UB_Encoder=0;   	//设置遥控的初始速度
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
u16 Dist_Sound,Dist_Light_L,Dist_Light_R;	//激光+超声波测距
u8 Flag_Stop,Flag_Run;					//停止和运动FLAG
u8 RS485_RX_BUF[64],RS485_Flag,LED_Flag;
s32 Tageviy_L,Tageviy_R;//左右轮子的速度
u8 Receive_Flag1,Receive_Flag2,CAN_Message_Flag;
u8 Power_buf[5]={0x5A,0x05,0x00,0x01,0x60};//获取数据
u8 Power_receive[9];
/* 定义变量 -E */
//Tageviy_LTageviy_R
/* 函数声明 -S */
void Laser_Distance(void);
/* 函数声明 -E */

/////////////////////////UCOSII任务设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			12 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  					64
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);

//LED任务
//设置LED任务优先级
#define LED_TASK_PRIO       			10
//设置LED任务堆栈大小
#define LED_STK_SIZE  		    		64
//LED任务堆栈	
OS_STK LED_TASK_STK[LED_STK_SIZE];
//LED任务函数
void led_task(void *pdata);

//PS2任务
//设置PS2任务优先级
#define PS2_TASK_PRIO       			8
//设置PS2任务堆栈大小
#define PS2_STK_SIZE  		    		64
//PS2任务堆栈	
OS_STK PS2_TASK_STK[PS2_STK_SIZE];
//PS2任务函数
void PS2_task(void *pdata);

//超声波任务
//设置超声波任务优先级
#define DXIIC_TASK_PRIO       			7
//设置超声波任务堆栈大小
#define DXIIC_STK_SIZE  		    		128
//超声波任务堆栈
OS_STK DXIIC_TASK_STK[DXIIC_STK_SIZE];
//超声波任务函数
void DXIIC_task(void *pdata);

//激光测距任务
//设置激光测距任务优先级
#define IIC_TASK_PRIO       			11
//设置激光测距任务堆栈大小
#define IIC_STK_SIZE  		    		128
//激光测距任务堆栈
OS_STK IIC_TASK_STK[IIC_STK_SIZE];
//激光测距任务函数
void IIC_task(void *pdata);

OS_TMR   * ctrl_tim;			//控制--软件定时器
void ctrl_tim_callback(OS_TMR *ptmr,void *p_arg);
/*
*主函数
*各模块的初始化
*创建并启动开始任务
*/
int main(void)
{
	delay_init(168);		  //初始化延时函数
	LED_Init();		        //初始化LED端口
	RGB_Init();						//初始化RGB灯条
	BUMP_Init();					//初始化防撞条
	PS2_Init();						//初始化PS2手柄
	uart_init(115200);		//初始化串口1
	RS485_Init(9600);     //初始化串口2(485)
//	IIC_Init();         	//初始化IIC1_北醒激光
//	DXIIC_Init();					//初始化IIC2_超声波
	delay_ms(2000);       //延时等待系统稳定
	CAN1_Mode_Init();			//初始化CAN总线
//	TIM3_Config();
	OSInit();
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();
}
//开始任务
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	u8 err;
	pdata = pdata;
	/* 进入临界区(无法被中断打断) */
	OS_ENTER_CRITICAL();
	/* 工作灯任务 */
	OSTaskCreate(led_task,(void *)0,(OS_STK*)&LED_TASK_STK[LED_STK_SIZE-1],LED_TASK_PRIO);
	/* PS2任务 */
	OSTaskCreate(PS2_task,(void *)0,(OS_STK*)&PS2_TASK_STK[PS2_STK_SIZE-1],PS2_TASK_PRIO);
	/* 激光测距任务 */
//	OSTaskCreate(IIC_task,(void *)0,(OS_STK*)&IIC_TASK_STK[IIC_STK_SIZE-1],IIC_TASK_PRIO);
	/* 超声波任务 */
//	OSTaskCreate(DXIIC_task,(void *)0,(OS_STK*)&DXIIC_TASK_STK[DXIIC_STK_SIZE-1],DXIIC_TASK_PRIO);
	/* 创建软件定时器，40ms执行一次 */
	ctrl_tim=OSTmrCreate(10,4,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)ctrl_tim_callback,0,(u8*)"ctrl_tim",&err);
	OSTmrStart(ctrl_tim,&err);			//启动软件定时器1
	OSTaskSuspend(START_TASK_PRIO);	//挂起开始任务
	/* 退出临界区(可以被中断打断) */
	OS_EXIT_CRITICAL();
}
//LED任务
void led_task(void *pdata)
{
	while(1)
	{
		/* 电量查询 */
		Rs485_Flash(2);
		/* 工作灯闪烁 */
		GPIO_ToggleBits(GPIOA,GPIO_Pin_8);
		delay_ms(500);
	}
}
//PS2任务
void PS2_task(void *pdata)
{
	while(1)
	{
		/* PS2手柄控制 */
		PS2_Receive();
		/* 防撞条碰撞反应 ,PS2_ON_Flag=0,Usart_ON_Flag=1*/
		if(click())
		{
			bump_flag=1;
			memset(Urxbuf,0,8);
			Urxbuf[8]=0x10;
		}
		if(bump>50)
			bump=0,bump_flag=0;
		
		delay_ms(40);
		/* 开启手柄，关闭串口 */
		if(PS2_KEY==4)
		{
			PS2_ON_Flag=1,Usart_ON_Flag=0;
			RC_Velocity=3;
		}
		/* 开启串口，关闭手柄 */
		if(PS2_KEY==1)
		{
			PS2_ON_Flag=0,Usart_ON_Flag=1;
			B_LED=1;
		}
//		if(PS2_KEY==7)
//		{
//			Motor_Disenable();
//		}
//		if(PS2_KEY==5)
//		{
//			Motor_Enable();
//		}
	}
}
//激光测距防跌落
void IIC_task(void *pdata)
{
	while(1)
	{
		/* 获取距离 */
		Laser_Distance();
		/* 悬崖判断 */
		if(Dist_Light_R>74)
		{
			Num_Stop++;
			if(Num_Stop>5)
			{
				Flag_Stop=1;
				Num_Stop=5;
			}
		}
		else if(Dist_Light_R<74)
		{
			Num_Stop--;
			if(Num_Stop<1)
			{
				Flag_Stop=0;
				Num_Stop=1;
			}
		}
		delay_ms(100);
	}
}
//超声波任务
void DXIIC_task(void *pdata)
{
	while(1)
	{
		KS103_WriteOneByte(0XE8,0X02,0XB4);
		delay_ms(100);
		Dist_Sound = KS103_ReadOneByte(0xe8, 0x02);
		Dist_Sound <<= 8;
		Dist_Sound += KS103_ReadOneByte(0xe8, 0x03);
		printf(" Dist_Sound = %d | \r\n",Dist_Sound);
	}
}
/*************************/
/* 激光测距 */
void Laser_Distance(void)
{
//	i2cWrite(ADDR_1,5,Power_buf);
//	i2cRead(ADDR_1,9,Power_receive);
//	Dist_Light_L=Power_receive[2]+Power_receive[3]*256;
//		for(u8 i=0;i<9;i++)
//			printf("0x%x ",receivebuf[i]);
//	printf("	Dist_Light_1 = %4d |	",Dist_Light_L+2);
//	delay_us(10);
	i2cWrite(ADDR_2,5,Power_buf);
	i2cRead(ADDR_2,9,Power_receive);
	Dist_Light_R = Power_receive[2]+Power_receive[3]*256;
//		for(i=0;i<9;i++)
//			printf("0x%x ",receivebuf[i]);
//	printf("	Dist_Light_2 = %4d \r\n",Dist_Light_R+3);
}
