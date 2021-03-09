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
#include "includes.h"

/* ������� */
u8 CAN1Sedbuf[8];
u8 CAN1Redbuf[8];
u8 CAN1_DATAsize; 
u16 CAN1_ID;
u8 CAN1_REDBUFF = 0;
/* �ٶ�ģʽPV�� */
u32 PV_spd;
u32 PP_spd;
u32 PT_spd;

CanTxMsg TxMes1;			/* CAN1������Ϣ���ͽṹ�� */
CanRxMsg RxMes1;			/* CAN1������Ϣ���սṹ�� */

u16 Encoder_num;			/* ��ȡ���������� */

/* SDO CMD */
#define  SDO_W1   0x2F
#define  SDO_W2   0x2B
#define  SDO_W4   0x23
#define  SDO_RD   0x40
/*  �ٶ�ģʽ */
#define PP_Mode  1
#define PV_Mode  3 
#define PT_Mode  4 
/* ������ID */
#define  Left_Wheel_ID              0x0001
#define  Right_Wheel_ID             0x0003

u32 Last_CAN1_ID;

/* CAN1��ʼ�� */
u8 CAN1_Mode_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	CAN_InitTypeDef        CAN_InitStructure;

	/* ʹ�����ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                  											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	/* ��ʼ��GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* ���Ÿ���ӳ������ */
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
	/* CAN��Ԫ���� */
	//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
	//������Ϊ:42M/((6+7+1)*3)=1Mbps
	CAN_InitStructure.CAN_TTCM = DISABLE;	        //��ֹʱ�䴥��ͨ��ģʽ��CANӲ���������������ڷ���/������������ʱ�����
	CAN_InitStructure.CAN_ABOM = DISABLE;	        //����Զ����߹���CANӲ�������߹ر�ʱ��״̬��	  
	CAN_InitStructure.CAN_AWUM = DISABLE;         //0��˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)��1��˯��ģʽӲ�����ѡ�
	CAN_InitStructure.CAN_NART = ENABLE;	        //0��CANӲ�����Զ��ط���ֱ���ɹ���1����ֹ�����Զ����ͣ�ֻ��1�Σ�
	CAN_InitStructure.CAN_RFLM = DISABLE;	        //0��FIFO�������������,�µĸ��Ǿɵģ�1��FIFO�����������������1������ʧ��
	CAN_InitStructure.CAN_TXFP = DISABLE;	        //0�����ȼ��ɱ��ı�ʶ��������1�����ȼ�������˳�������
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //����ģʽ
	CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;	    //����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ 1tq ~ 4tq��λ�仯�ڴ˽׶η�����
	CAN_InitStructure.CAN_BS1  = CAN_BS1_7tq;     //Tbs1��Χ 1tq ~ 16tq������������λ�ã�
	CAN_InitStructure.CAN_BS2  = CAN_BS2_6tq;     //Tbs2��Χ 1tq ~ 8tq�������巢�͵��λ�ã�
	CAN_InitStructure.CAN_Prescaler = 3;        	//��Ƶϵ��(Fdiv)Ϊbrp+1������Ԥ�ʷ�Ƶ1-1024��	bps=1000
	CAN_Init(CAN1, &CAN_InitStructure);           // ��ʼ��CAN1
	/* ���ù����� */
	/* ֻ����0x581��0x583����Ϊ���ص�IDΪ0x580+ID����Ϊ�˶���Ϣ���� */
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
	/* ��������0x701��0x703����Ϊ�ϵ���Ϣ���� */
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
	/* ��������0x081����Ϊ������Ϣ���� */
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

	/* PV���� ʹ�ܵ�� */
	CANopen_PV_Init();
	return 0;
}
/* CAN1�ķ��ͺ��� */
u8 CAN1_Send(u16 Id,u8 CAN1_DLC)
{
	u8 mail;
	u16 i=0;
	TxMes1.StdId = Id;	         	//��׼��ʶ��ID
	TxMes1.ExtId = Id;	         	//��չ��ʾ��ID��29λ��
	TxMes1.IDE = CAN_Id_Standard; //ʹ�ñ�׼��
	TxMes1.RTR = CAN_RTR_Data;		//��Ϣ����Ϊ����֡��/ң��֡
	TxMes1.DLC = CAN1_DLC;		    // 	1�����ĳ���	
	for(i=0;i<CAN1_DLC;i++)
	TxMes1.Data[i]=CAN1Sedbuf[i];	// ��һ֡��Ϣ          
	mail = CAN_Transmit(CAN1, &TxMes1);   
	i=0;
	/* �ȴ����ͽ��� */
	while((CAN_TransmitStatus(CAN1, mail)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;
	if(i>=0XFFF)return 1;
	return 0;
}
//CAN1�Ľ��ܺ���
void CAN1_Read(void)
{
	u32 i;
	/* û�н��յ�����,ֱ���˳�  */
  if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)	return;
	CAN_Receive(CAN1, CAN_FIFO0, &RxMes1);							/* ��ȡ����	*/
	for(i=0;i<RxMes1.DLC;i++)
  CAN1Redbuf[i] = RxMes1.Data[i];
	CAN1_DATAsize = RxMes1.DLC;
	CAN1_ID       = RxMes1.StdId;
	CAN1_REDBUFF  = 1;
	/* ��ӡ������Ϣ�����Ե��� */
//	printf("CAN1_ID = %x | CAN1_DATAsize = %d | CAN1Redbuf = ",CAN1_ID,CAN1_DATAsize);
//	for(i=0;i<RxMes1.DLC;i++)
//		printf("%02x ",CAN1Redbuf[i]);
//	printf("\r\n");
	/* �Ա�����Ϣ���д��� */
	if(CAN1_ID == 0x0081)
	{
		/* ��⵽��վ�ϵ�Last_CAN1_ID==701&& */
		CAN_Message_Flag=1;
	}
	if(Last_CAN1_ID==0x0701&&CAN1_ID== 0X0703)
	{
		/* ��⵽��վ�ϵ� MCU��λ */
		NVIC_SystemReset();
	}
	Last_CAN1_ID=CAN1_ID;
}
/* ģʽ���� */
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
/* ����ڵ� */
u8 CANopen_Activate(u8 CANopen_ID){
	 
	 CAN1Sedbuf[0]=0x01;
	 CAN1Sedbuf[1]=CANopen_ID;	
	 CAN1_Send(0x000,2);
	 delay_ms(5);
	 return(1);
} 
/* д���� */
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
/* Э���ٶ����� */
void CANopen_PV_Init(void)
{
	/* STEP1:����ڵ�1���ڵ�2 */
	CANopen_Activate( Left_Wheel_ID  );
	CANopen_Activate( Right_Wheel_ID );

	/* STEP2:�����ٶ�ģʽ	6060HдΪ3 */
	Contol_Mode_SET( Left_Wheel_ID,  PV_Mode );
	Contol_Mode_SET( Right_Wheel_ID, PV_Mode );

	/* STEP3:���üӼ���	д6083H��6084H */
	SDO_Write_OD( Left_Wheel_ID, SDO_W4, 0x6083,0x00,0x00003A98);
	SDO_Write_OD( Left_Wheel_ID, SDO_W4, 0x6084,0x00,0x00003A98);

	SDO_Write_OD( Right_Wheel_ID,SDO_W4, 0x6083,0x00,0x00003A98);
	SDO_Write_OD( Right_Wheel_ID,SDO_W4, 0x6084,0x00,0x00003A98);

	/* STEP4:����Ŀ��ת��Ϊ0	д60FFH */
	SDO_Write_OD( Left_Wheel_ID, SDO_W4, 0x60FF,0x00,0x00000000);
	SDO_Write_OD( Right_Wheel_ID,SDO_W4, 0x60FF,0x00,0x00000000);

	/* ���ʹ�� */
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000006);
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000007);
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x0000000F);
	
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000006);
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000007);
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x0000000F);
}
/* �ٶȵ����� */
void CANopen_PV_SET(u32 Acc,u32 Dec,s32 Tageviy_L,s32 Tageviy_R)
{
	/* STEP1:���üӼ���ʱ��	д6083H��6084H */	 
//	 SDO_Write_OD( Left_Wheel_ID, SDO_W4,0x6083,0x00,0x000003e8);
//   SDO_Write_OD( Left_Wheel_ID, SDO_W4,0x6084,0x00,0x000003e8);
//	 
//	 SDO_Write_OD( Right_Wheel_ID,SDO_W4,0x6083,0x00,0x000003e8);
//	 SDO_Write_OD( Right_Wheel_ID,SDO_W4,0x6084,0x00,0x000003e8);
	 
	/* STEP2:����Ŀ��ת�� 	д60FFH */
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
/* ���ʧ�� */
void Motor_Disenable(void)
{
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000007);
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000007);
}
/* ���ʹ�� */
void Motor_Enable(void)
{
	/* ���ʹ�� */
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000006);
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000007);
	SDO_Write_OD( Left_Wheel_ID, SDO_W2, 0x6040,0x00,0x0000000F);
	
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000006);
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x00000007);
	SDO_Write_OD( Right_Wheel_ID, SDO_W2, 0x6040,0x00,0x0000000F);
}
/* ��ȡʵ��A���ת�� */
u16 Encoder_ReadA(void)
{
	SDO_Write_OD( Left_Wheel_ID, SDO_RD,0x6063,0x00,0x00000000);
	CAN1_Read();
	Encoder_num = CAN1Redbuf[4] | CAN1Redbuf[5]<<8 | CAN1Redbuf[6]<<16 | CAN1Redbuf[7]<<24;
	return Encoder_num;
}
/* ��ȡʵ��B���ת�� */
u16 Encoder_ReadB(void)
{
	SDO_Write_OD( Right_Wheel_ID, SDO_RD, 0x6063,0x00,0x00000000);
	CAN1_Read();
	Encoder_num = CAN1Redbuf[4] | CAN1Redbuf[5]<<8 | CAN1Redbuf[6]<<16 | CAN1Redbuf[7]<<24;
	return Encoder_num;
}
/* ����ٶȸ��� */
void Motor_PV_Zero(void)
{
   SDO_Write_OD( Left_Wheel_ID, SDO_W4,0x60FF,0x00,0x00000000);
	 SDO_Write_OD( Right_Wheel_ID,SDO_W4,0x60FF,0x00,0x00000000);
}
/***************************************************************/
/*
void Motor_PV_Go(void)
{
//д0x60FF�ٶ�ֵ
   SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x60FF,0x00,PV_spd);
	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x60FF,0x00,PV_spd);	
}
void Motor_PV_Left(void)
{ 
//д0x60FF�ٶ�ֵ
	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x60FF,0x00,~PV_spd+1);
	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x60FF,0x00,PV_spd);	
}
void Motor_PV_Right(void)
{
//д0x60FF�ٶ�ֵ
	SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x60FF,0x00,PV_spd);
	SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x60FF,0x00,~PV_spd+1);
}
void Motor_PV_Back(void)
{
//д0x60FF�ٶ�ֵ
	SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x60FF,0x00,~PV_spd+1);
	SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x60FF,0x00,~PV_spd+1);
}*/
