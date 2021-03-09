#ifndef __RS485_H
#define __RS485_H
#include "sys.h"
#include "includes.h"
/* ģʽ����485ģʽ����.0,����;1,���� */
#define RS485_TX_EN		PAout(6)
/* ����봮���жϽ��գ�����EN_USART2_RXΪ1����������Ϊ00,������;1,���� */
#define EN_USART2_RX 	1

/* ��������-S */
void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);
void RS485_Receive_Data(void);
void Rs485_Flash(u16 time);
void DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);

/* ��������-E */

#endif

