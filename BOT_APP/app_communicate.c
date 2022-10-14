#include "app_communicate.h"

uint8_t USART_RX_BUF[USART_REC_LEN];    			// 接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART_RX_STA=0;       						// 接收状态标记
void USART1_IRQHandler(void)                		// 串口1中断服务程序
{
	uint8_t _res;
	
	if(USART1->SR & (1<<5))  						// 接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		_res = USART1->DR;							// 读取接收到的数据

		if((USART_RX_STA & 0x8000) == 0) {			// 接收未完成
			if(USART_RX_STA & 0x4000) {				// 接收到了0x0d
				if(_res != 0x0a)	{
					USART_RX_STA = 0;				// 接收错误,重新开始
				} else {
					USART_RX_STA |= 0x8000;			// 接收完成了，解析数据帧
					
					uint8_t _data_i = 0;
					for (_data_i = 0; _data_i < 3; ++_data_i) {
						uint8_t _iMultipySix = 6 * _data_i;
						groundTruth[_data_i].x = USART_RX_BUF[1 + _iMultipySix] * 256 + USART_RX_BUF[2 + _iMultipySix];
						groundTruth[_data_i].y = USART_RX_BUF[4 + _iMultipySix] * 256 + USART_RX_BUF[5 + _iMultipySix];
						
						if (USART_RX_BUF[0 + _iMultipySix] == 0x01) {
							groundTruth[_data_i].x = -groundTruth[_data_i].x;
						}
						if (USART_RX_BUF[3 + _iMultipySix] == 0x01) {
							groundTruth[_data_i].y = -groundTruth[_data_i].y;
						}
					}
					obs_x = USART_RX_BUF[19] * 256 + USART_RX_BUF[20];
					obs_y = USART_RX_BUF[22] * 256 + USART_RX_BUF[23];
					if (USART_RX_BUF[18] == 0x01) {
						obs_x = -obs_x;
					}
					if (USART_RX_BUF[21] == 0x01) {
						obs_y = -obs_y;
					}

					USART_RX_STA = 0;
				}
			} else {								// 还没收到0X0D
				if(_res == 0x0d) {
					USART_RX_STA |= 0x4000;
				} else {
					USART_RX_BUF[USART_RX_STA & 0X3FFF] = _res ;
					USART_RX_STA++;
					if(USART_RX_STA > (USART_REC_LEN - 1)) {
						USART_RX_STA = 0;			//接收数据错误,重新开始接收
					}						
				}		 
			}
		} 
	} 
} 

uint8_t agentAddress[5][2] = { {0xA2, 0xA0},	// 终端协调器
							{0xA2, 0xA1},		// agent 1
							{0xA2, 0xA2},		// agent 2
							{0xA2, 0xA3},		// agent 3
							{0xA2, 0xA4},		// agent 4
							};

							
// 0 失能全部电机，1 使能全部电机
void sendOrder(uint8_t order, uint8_t tar)
{
	uint8_t sendDataBuffer[2];
	
	if (order == ENABLE_ALL_MOTORS || order == DISENABLE_ALL_MOTORS) {
		sendDataBuffer[0] = 0x01;
		sendDataBuffer[1] = order == ENABLE_ALL_MOTORS ? 0x01 : 0x00;
	} else if (order == SWITCH_FORMATION_0 || order == SWITCH_FORMATION_1 || order == SWITCH_FORMATION_2) {
		sendDataBuffer[0] = 0x02;
		sendDataBuffer[1] = order - 2;
	} 
	

	xbeeSend(sendDataBuffer, tar, 2);
}
							
							
void sendPosition(int x, int y, uint8_t tar)
{
	uint8_t sendDataBuffer[6];
	
	if (x < 0)	sendDataBuffer[0] = 0x01;
	else 		sendDataBuffer[0] = 0x00;
	if (y < 0)	sendDataBuffer[3] = 0x01;
	else 		sendDataBuffer[3] = 0x00;

	sendDataBuffer[1] = abs(x) / 256;
	sendDataBuffer[2] = abs(x) % 256;
	sendDataBuffer[4] = abs(y) / 256;
	sendDataBuffer[5] = abs(y) % 256;

	xbeeSend(sendDataBuffer, tar, 6);
}

void xbeeSend(uint8_t send_data[], uint8_t tar_add, uint8_t send_size)
{
	uint8_t  sendDataBuffer[30];
	uint8_t  xbeeDataLenth = 0;
	uint16_t xbeeCheckSum = 0;
	uint8_t  xbee_i = 0;
	
	sendDataBuffer[0] = 0x7E;						// 帧头
	sendDataBuffer[1] = (0x05 + send_size) / 256;	
	sendDataBuffer[2] = (0x05 + send_size) % 256;	// 帧长
	sendDataBuffer[3] = 0x01;						// 帧类型 0x01 16位地址传送帧
	sendDataBuffer[4] = 0x01;						// 帧ID （不重要）
	for (xbee_i = 0; xbee_i < 2; ++xbee_i) {		// 传输目标地址
		sendDataBuffer[5 + xbee_i] = agentAddress[tar_add][xbee_i];
	}
	sendDataBuffer[7] = 0x01;						// Options	Disable ACK
	for (xbee_i = 0; xbee_i < send_size; ++xbee_i) {
		sendDataBuffer[8 + xbee_i] =  send_data[xbee_i];
	}
	xbeeDataLenth = sendDataBuffer[2] + 3;
	for (xbee_i = 3; xbee_i < xbeeDataLenth; ++xbee_i) {
		xbeeCheckSum += sendDataBuffer[xbee_i];
	}
	xbeeCheckSum = 0xFF - (xbeeCheckSum % 256);
	sendDataBuffer[xbeeDataLenth] = xbeeCheckSum;
	
	for(xbee_i = 0; xbee_i < xbeeDataLenth + 1; ++xbee_i)
	{
		usart3_send(sendDataBuffer[xbee_i]);
	}
}

uint8_t XBEE_RX_BUF[XBEE_REC_LEN];
uint8_t XBEE_RX_LEN = 0;
uint8_t XBEE_RX_FLAG = 0;
uint8_t XBEE_dataSize = 0;

void USART3_IRQHandler(void)
{	
	if(USART3->SR & (1<<5))			//接收到数据
	{	      
		uint8_t temp;
		temp = USART3->DR;
		
		if (XBEE_RX_FLAG == 1) {										// 开始接收数据
			XBEE_RX_BUF[XBEE_RX_LEN] = temp;
			XBEE_RX_LEN++;
			if (XBEE_RX_LEN == 3) {
				XBEE_dataSize = XBEE_RX_BUF[2];
			} else if (XBEE_RX_LEN > XBEE_dataSize + 3) {				// 根据帧长判断是否接收完毕
				XBEE_RX_FLAG = 2;
			}
		} else if (!XBEE_RX_LEN && !XBEE_RX_FLAG && temp == 0x7E) {		// 接收到帧头
			XBEE_RX_FLAG = 1;
			XBEE_RX_BUF[0] = 0x7E;
			XBEE_RX_LEN = 1;
			XBEE_dataSize = 0;
		} 
		
		if (XBEE_RX_FLAG == 2) {										// 接收完成后 解析帧数据
			if (XBEE_RX_BUF[3] == 0x89) {								// 如果是发送状态回复帧则不做处理
				
			} else if (XBEE_RX_BUF[3] == 0x81) {						// 数据帧
				if (XBEE_RX_BUF[8] == 0x01 || XBEE_RX_BUF[8] == 0x00) {
					if (XBEE_RX_BUF[5] == 0xA0) {						// 如果是上位机发送的数据帧，则设定目标位置
//						Move_X = XBEE_RX_BUF[9] * 256 + XBEE_RX_BUF[10];
//						Move_Y = XBEE_RX_BUF[12] * 256 + XBEE_RX_BUF[13];
//						if (XBEE_RX_BUF[8] == 0x01)		Move_X = -Move_X;
//						if (XBEE_RX_BUF[11] == 0x01)	Move_Y = -Move_Y;
					}
				} else if (XBEE_RX_BUF[8] == 0x02){
					startFlag = 1;
				} else if (XBEE_RX_BUF[8] == 0x03) {
					startFlag = 0;
				}
			} 
			XBEE_RX_LEN = 0;
			XBEE_RX_FLAG = 0;
		}
	}	
}

/**************************************************************************
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART2_IRQHandler(void)
{	
	if(USART2->SR&(1<<5) )//接收到数据
	{	      
		uint8_t Usart_Receive = USART2->DR;

		if (Usart_Receive == 's') {
			startFlag = !startFlag;
			if (startFlag == 0) {
				sendOrder(DISENABLE_ALL_MOTORS, 2);
				sendOrder(DISENABLE_ALL_MOTORS, 3);
			} else {
				sendOrder(ENABLE_ALL_MOTORS, 2);
				sendOrder(ENABLE_ALL_MOTORS, 3);
			}
		} else if (Usart_Receive == 'a') {
			formationType = 0;
			sendOrder(SWITCH_FORMATION_0, 2);
			sendOrder(SWITCH_FORMATION_0, 3);
		} else if (Usart_Receive == 'b') {
			formationType = 1;
			sendOrder(SWITCH_FORMATION_1, 2);
			sendOrder(SWITCH_FORMATION_1, 3);
		} else if (Usart_Receive == 'c') {
			formationType = 2;
			sendOrder(SWITCH_FORMATION_2, 2);
			sendOrder(SWITCH_FORMATION_2, 3);
		}
	}
	return 0;	
}

