#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "sys.h"
//LED �˿ڶ���
#define LED PBout(13) 
void BSP_LED_Init(void);  //��ʼ��
void LedFlash(u16 time);

#endif
