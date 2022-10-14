#ifndef __APP_COMMUNICATE_H
#define __APP_COMMUNICATE_H

#include <math.h>
#include "bsp_usartx.h"

#define USART_REC_LEN 40  	//定义最大接收字节数 40
#define XBEE_REC_LEN  100

#define ENABLE_ALL_MOTORS		0
#define DISENABLE_ALL_MOTORS	1

#define SWITCH_FORMATION_0      2
#define SWITCH_FORMATION_1      3
#define SWITCH_FORMATION_2      4

void sendPosition(int x, int y, uint8_t tar);
void sendOrder(uint8_t order, uint8_t tar);
void xbeeSend(uint8_t send_data[], uint8_t tar_add, uint8_t send_size);

#endif

