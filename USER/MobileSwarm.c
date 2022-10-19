#include "sys.h"
#include "app_show.h"
#include "app_communicate.h"

float targetVx = 0, targetVy = 0, targetVz = 0;
int targetPoint_X = 0, targetPoint_Y = 0, targetPoint_Z = 0; 

int Encoder_A,Encoder_B,Encoder_C,Encoder_D;	
long int Position_A,Position_B,Position_C,Position_D;                     		                 
long int Motor_A,Motor_B,Motor_C,Motor_D;       
long int Target_A,Target_B,Target_C,Target_D;		                       
uint8_t delay_50,delay_flag;                         								
float Pitch,Roll,Yaw;   						// 三轴角度
float Velocity_KP = 5, Velocity_KI = 10;		// 速度控制PID参数  10  10  
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY,CONTROL_MODE;	// 遥控器相关变量
/* * * * * * * * * * * * * 编队相关变量定义 * * * * * * * * * * * * * * * */ 
uint8_t startFlag = 0, advoidMode = 0;
uint8_t formationType = 0, traceType = 0;
uint8_t formatChange_FLAG = 0, traceChange_FLAG = 0;
struct GROUND groundTruth[4] = {{0, 0},{0, 0}, {0, 0}, {0, 0}};
struct GROUND groundForma[][4] = {{{0, 0},{-700, 0},{-1400, 0},{0, 0}},
								{{0, 0},{0, -700},{0, -1400},{0, 0}},
								{{0, 0},{700, -700},{-700, -700},{0, 0}}};
int obs_x = 0, obs_y = 0;
/* * * * * * * * * * * * * 变量定义结束* * * * * * * * * * * * * * * * * */ 

int main(void)
{ 
	Stm32_Clock_Init(9);            // 系统时钟设置
	delay_init(72);                 // 延时初始化
	JTAG_Set(JTAG_SWD_DISABLE);     // 关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           // 打开SWD接口 可以利用主板的SWD接口调试
									   
	BSP_LED_Init();                 // 初始化与 LED 连接的硬件接口
	BSP_KEY_Init();                 // 按键初始化
									   
	uart1_init(72, 115200);         // 串口1初始化
	// uart2_init(36, 9600);          	// 串口2初始化
	// uart3_init(36, 115200);      // 串口3初始化	XBEE
	BSP_Encoder_Init();				// 编码器初始化
	BSP_OLED_Init();                // OLED初始化
									   
	BSP_IIC_Init();                 // IIC初始化
	delay_ms(1000);                    
	BSP_MPU6050_Init();           	// MPU6050初始化	
	BSP_DMP_Init();                 // 初始化DMP     MPU6050姿态解算
	BSP_PWM_Init(7199,0);   		// 初始化PWM 10KHZ，用于驱动电机
	BSP_PS2_Init();					// PS2驱动端口初始化
	BSP_PS2_SetInit();				// PS2配置初始化,配置“红绿灯模式”，并选择是否可以修改
	
	delay_ms(1000);
	BSP_EXTI_Init();                // MPU6050 5ms定时中断初始化
	
	while(1)
	{	
		APP_OLED_Show();
		
		PS2_KEY = PS2_DataKey();
		PS2_LX = PS2_AnologData(PSS_LX);      
		PS2_LY = PS2_AnologData(PSS_LY);
		PS2_RX = PS2_AnologData(PSS_RX);
		PS2_RY = PS2_AnologData(PSS_RY);
		if (PS2_KEY == PSB_PINK) {
			startFlag = 1;
		} else if (PS2_KEY == PSB_RED) {
			startFlag = 0;
		} else if (PS2_KEY == PSB_GREEN) {
			advoidMode = 1;
		} else if (PS2_KEY == PSB_BLUE) {
			advoidMode = 0;
		} else if (PS2_KEY == PSB_R1) {
			CONTROL_MODE = 1;
		} else if (PS2_KEY == PSB_R2) {
			CONTROL_MODE = 0;
		}

		delay_flag = 1;	
		delay_50=0;
		while(delay_flag);	   
	} 
}
