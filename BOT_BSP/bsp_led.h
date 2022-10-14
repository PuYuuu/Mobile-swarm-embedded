#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "sys.h"
//LED 端口定义
#define LED PBout(13) 
void BSP_LED_Init(void);  //初始化
void LedFlash(u16 time);

#endif
