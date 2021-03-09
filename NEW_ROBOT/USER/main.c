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
 *fromelf.exe --bin -o ./bin/NEW_ROBOT.bin ./../OBJ/NEW_ROBOT.axf ---�̼����� 
*/
#include "sys.h"
#include "includes.h"
/* ������ͨѶ��ַ */
#define ADDR_1 	0x10
#define ADDR_2	0x20

/* ������� -S */
static u8 Num_Stop;
u8 Flag_Stop,Flag_Show,bump,bump_flag=0;					//ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
u8 Usart_Flag;  				//����ң����ر���������״̬��־λ//��ʱ��ر���
u8 Urxbuf[9],Usart_ON_Flag=1,PS2_ON_Flag=0;	//���ڿ�����ر���
float Move_Y,Move_Z;  						//����ǶȺ�XYZ��Ŀ���ٶ�
int RC_Velocity=3,UA_Encoder=0,UB_Encoder=0;   	//����ң�صĳ�ʼ�ٶ�
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
u16 Dist_Sound,Dist_Light_L,Dist_Light_R;	//����+���������
u8 Flag_Stop,Flag_Run;					//ֹͣ���˶�FLAG
u8 RS485_RX_BUF[64],RS485_Flag,LED_Flag;
s32 Tageviy_L,Tageviy_R;//�������ӵ��ٶ�
u8 Receive_Flag1,Receive_Flag2,CAN_Message_Flag;
u8 Power_buf[5]={0x5A,0x05,0x00,0x01,0x60};//��ȡ����
u8 Power_receive[9];
/* ������� -E */
//Tageviy_LTageviy_R
/* �������� -S */
void Laser_Distance(void);
/* �������� -E */

/////////////////////////UCOSII��������///////////////////////////////////
//START ����
//�����������ȼ�
#define START_TASK_PRIO      			12 //��ʼ��������ȼ�����Ϊ���
//���������ջ��С
#define START_STK_SIZE  					64
//�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);

//LED����
//����LED�������ȼ�
#define LED_TASK_PRIO       			10
//����LED�����ջ��С
#define LED_STK_SIZE  		    		64
//LED�����ջ	
OS_STK LED_TASK_STK[LED_STK_SIZE];
//LED������
void led_task(void *pdata);

//PS2����
//����PS2�������ȼ�
#define PS2_TASK_PRIO       			8
//����PS2�����ջ��С
#define PS2_STK_SIZE  		    		64
//PS2�����ջ	
OS_STK PS2_TASK_STK[PS2_STK_SIZE];
//PS2������
void PS2_task(void *pdata);

//����������
//���ó������������ȼ�
#define DXIIC_TASK_PRIO       			7
//���ó����������ջ��С
#define DXIIC_STK_SIZE  		    		128
//�����������ջ
OS_STK DXIIC_TASK_STK[DXIIC_STK_SIZE];
//������������
void DXIIC_task(void *pdata);

//����������
//���ü������������ȼ�
#define IIC_TASK_PRIO       			11
//���ü����������ջ��С
#define IIC_STK_SIZE  		    		128
//�����������ջ
OS_STK IIC_TASK_STK[IIC_STK_SIZE];
//������������
void IIC_task(void *pdata);

OS_TMR   * ctrl_tim;			//����--�����ʱ��
void ctrl_tim_callback(OS_TMR *ptmr,void *p_arg);
/*
*������
*��ģ��ĳ�ʼ��
*������������ʼ����
*/
int main(void)
{
	delay_init(168);		  //��ʼ����ʱ����
	LED_Init();		        //��ʼ��LED�˿�
	RGB_Init();						//��ʼ��RGB����
	BUMP_Init();					//��ʼ����ײ��
	PS2_Init();						//��ʼ��PS2�ֱ�
	uart_init(115200);		//��ʼ������1
	RS485_Init(9600);     //��ʼ������2(485)
//	IIC_Init();         	//��ʼ��IIC1_���Ѽ���
//	DXIIC_Init();					//��ʼ��IIC2_������
	delay_ms(2000);       //��ʱ�ȴ�ϵͳ�ȶ�
	CAN1_Mode_Init();			//��ʼ��CAN����
//	TIM3_Config();
	OSInit();
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	OSStart();
}
//��ʼ����
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	u8 err;
	pdata = pdata;
	/* �����ٽ���(�޷����жϴ��) */
	OS_ENTER_CRITICAL();
	/* ���������� */
	OSTaskCreate(led_task,(void *)0,(OS_STK*)&LED_TASK_STK[LED_STK_SIZE-1],LED_TASK_PRIO);
	/* PS2���� */
	OSTaskCreate(PS2_task,(void *)0,(OS_STK*)&PS2_TASK_STK[PS2_STK_SIZE-1],PS2_TASK_PRIO);
	/* ���������� */
//	OSTaskCreate(IIC_task,(void *)0,(OS_STK*)&IIC_TASK_STK[IIC_STK_SIZE-1],IIC_TASK_PRIO);
	/* ���������� */
//	OSTaskCreate(DXIIC_task,(void *)0,(OS_STK*)&DXIIC_TASK_STK[DXIIC_STK_SIZE-1],DXIIC_TASK_PRIO);
	/* ���������ʱ����40msִ��һ�� */
	ctrl_tim=OSTmrCreate(10,4,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)ctrl_tim_callback,0,(u8*)"ctrl_tim",&err);
	OSTmrStart(ctrl_tim,&err);			//���������ʱ��1
	OSTaskSuspend(START_TASK_PRIO);	//����ʼ����
	/* �˳��ٽ���(���Ա��жϴ��) */
	OS_EXIT_CRITICAL();
}
//LED����
void led_task(void *pdata)
{
	while(1)
	{
		/* ������ѯ */
		Rs485_Flash(2);
		/* ��������˸ */
		GPIO_ToggleBits(GPIOA,GPIO_Pin_8);
		delay_ms(500);
	}
}
//PS2����
void PS2_task(void *pdata)
{
	while(1)
	{
		/* PS2�ֱ����� */
		PS2_Receive();
		/* ��ײ����ײ��Ӧ ,PS2_ON_Flag=0,Usart_ON_Flag=1*/
		if(click())
		{
			bump_flag=1;
			memset(Urxbuf,0,8);
			Urxbuf[8]=0x10;
		}
		if(bump>50)
			bump=0,bump_flag=0;
		
		delay_ms(40);
		/* �����ֱ����رմ��� */
		if(PS2_KEY==4)
		{
			PS2_ON_Flag=1,Usart_ON_Flag=0;
			RC_Velocity=3;
		}
		/* �������ڣ��ر��ֱ� */
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
//�����������
void IIC_task(void *pdata)
{
	while(1)
	{
		/* ��ȡ���� */
		Laser_Distance();
		/* �����ж� */
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
//����������
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
/* ������ */
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
