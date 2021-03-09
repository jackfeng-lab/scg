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
#ifndef __RGBLIGHT_H_
#define __RGBLIGHT_H_

#include "sys.h"
#include "includes.h"

#define R_LED PCout(10)		/* ��� */
#define G_LED PAout(15)		/* �̵� */
#define B_LED PCout(11)		/* ���� */

void RGB_Init(void);

#endif
