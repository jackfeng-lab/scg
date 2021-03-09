#ifndef __RS485_H
#define __RS485_H
#include "sys.h"
#include "includes.h"
/* 模式控制485模式控制.0,接收;1,发送 */
#define RS485_TX_EN		PAout(6)
/* 如果想串口中断接收，设置EN_USART2_RX为1，否则设置为00,不接收;1,接收 */
#define EN_USART2_RX 	1

/* 函数声明-S */
void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);
void RS485_Receive_Data(void);
void Rs485_Flash(u16 time);
void DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);

/* 函数声明-E */

#endif

