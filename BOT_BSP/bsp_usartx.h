#ifndef __BSP_USRATX_H
#define __BSP_USRATX_H 

#include <stdio.h>
#include "sys.h"

void usart1_send(u8 data);
void uart1_init(u32 pclk2,u32 bound);

void usart2_send(u8 data);
void uart2_init(u32 pclk2,u32 bound);

void usart3_send(u8 data);
void uart3_init(u32 pclk2,u32 bound);


#endif

