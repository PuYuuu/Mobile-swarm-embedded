#ifndef __BSP_ADC_H
#define __BSP_ADC_H	

#include "sys.h"
#define Battery_Ch 5

void BSP_ADC_Init(void);
u16 Get_Adc(uint8_t ch);
int Get_battery_volt(void);   

#endif 















