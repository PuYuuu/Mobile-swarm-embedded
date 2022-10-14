#ifndef __BSP_KEY_H
#define __BSP_KEY_H	 

#include "sys.h"
#define KEY PBin(14)
// #define AVOIDAMCE_MODE PBin(12)	// 这里的MODE是将原来选择速度模式还是位置模式的拨码开关，
								// 更改为是否开启避障控制，另外现默认为速度模式控制

void BSP_KEY_Init(void);      	// 按键初始化
u8 click_N_Double (u8 time);  	// 单击按键扫描和双击按键扫描
u8 click(void);               	// 单击按键扫描
u8 Long_Press(void);

#endif 
