#ifndef __OLED_H_
#define __OLED_H_
#include "sys.h"
#include "includes.h"

/* OLED�˿ڶ��� */
#define OLED_RST_Clr() PCout(6)=0   //RST
#define OLED_RST_Set() PCout(6)=1   //RST

#define OLED_RS_Clr() PCout(7)=0    //DC
#define OLED_RS_Set() PCout(7)=1    //DC

#define OLED_SCLK_Clr()  PBout(15)=0  //SCL
#define OLED_SCLK_Set()  PBout(15)=1   //SCL

#define OLED_SDIN_Clr()  PBout(14)=0   //SDA
#define OLED_SDIN_Set()  PBout(14)=1   //SDA

#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

/* OLED�����ú��� */
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);
void oled_show(void);

#endif  

