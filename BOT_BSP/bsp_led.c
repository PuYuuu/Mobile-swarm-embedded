#include "bsp_led.h"

/**************************************************************************
函数功能：LED接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void BSP_LED_Init(void)
{
	RCC->APB2ENR|=1<<3; //使能 PORT 时钟  
	GPIOB->CRH&=0XFF0FFFFF;
	GPIOB->CRH|=0X00300000;//推挽输出
	GPIOB->ODR|=1<<13; // 输出高
}

/**************************************************************************
函数功能：LED闪烁
入口参数：闪烁频率 
返回  值：无
**************************************************************************/
void LedFlash(u16 time)
{
	static int temp;
	if(0==time) 			LED=0;
	else if(++temp==time)	LED=~LED,temp=0;
}
