#ifndef __BSP_KEY_H
#define __BSP_KEY_H	 

#include "sys.h"
#define KEY PBin(14)
// #define AVOIDAMCE_MODE PBin(12)	// �����MODE�ǽ�ԭ��ѡ���ٶ�ģʽ����λ��ģʽ�Ĳ��뿪�أ�
								// ����Ϊ�Ƿ������Ͽ��ƣ�������Ĭ��Ϊ�ٶ�ģʽ����

void BSP_KEY_Init(void);      	// ������ʼ��
u8 click_N_Double (u8 time);  	// ��������ɨ���˫������ɨ��
u8 click(void);               	// ��������ɨ��
u8 Long_Press(void);

#endif 
