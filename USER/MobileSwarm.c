#include "sys.h"
#include "app_show.h"
#include "app_communicate.h"

float targetVx = 0, targetVy = 0, targetVz = 0;
int targetPoint_X = 0, targetPoint_Y = 2700, targetPoint_Z = 0; 

int Encoder_A,Encoder_B,Encoder_C,Encoder_D;	
long int Position_A,Position_B,Position_C,Position_D;                     		                 
long int Motor_A,Motor_B,Motor_C,Motor_D;       
long int Target_A,Target_B,Target_C,Target_D;		                       
uint8_t delay_50,delay_flag;                         								
float Pitch,Roll,Yaw;   						// ����Ƕ�
float Velocity_KP = 5, Velocity_KI = 10;		// �ٶȿ���PID����  10  10  
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY,CONTROL_MODE;	// ң������ر���
/* * * * * * * * * * * * * �����ر������� * * * * * * * * * * * * * * * */ 
uint8_t startFlag = 0, advoidMode = 0;
uint8_t formationType = 0, traceType = 0;
uint8_t formatChange_FLAG = 0, traceChange_FLAG = 0;
struct GROUND groundTruth[4] = {{0, 0},{0, 0}, {0, 0}, {0, 0}};
struct GROUND groundForma[][4] = {{{0, 0},{-700, 0},{-1400, 0},{0, 0}},
								{{0, 0},{0, -700},{0, -1400},{0, 0}},
								{{0, 0},{700, -700},{-700, -700},{0, 0}}};
int obs_x = 0, obs_y = 0;
/* * * * * * * * * * * * * �����������* * * * * * * * * * * * * * * * * */ 

int main(void)
{ 
	Stm32_Clock_Init(9);            // ϵͳʱ������
	delay_init(72);                 // ��ʱ��ʼ��
	JTAG_Set(JTAG_SWD_DISABLE);     // �ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           // ��SWD�ӿ� �������������SWD�ӿڵ���
									   
	BSP_LED_Init();                 // ��ʼ���� LED ���ӵ�Ӳ���ӿ�
	BSP_KEY_Init();                 // ������ʼ��
									   
	uart1_init(72, 115200);         // ����1��ʼ��
	// uart2_init(36, 9600);          	// ����2��ʼ��
	// uart3_init(36, 115200);      // ����3��ʼ��	XBEE
	BSP_Encoder_Init();				// ��������ʼ��
	BSP_OLED_Init();                // OLED��ʼ��
									   
	BSP_IIC_Init();                 // IIC��ʼ��
	delay_ms(1000);                    
	BSP_MPU6050_Init();           	// MPU6050��ʼ��	
	BSP_DMP_Init();                 // ��ʼ��DMP     MPU6050��̬����
	BSP_PWM_Init(7199,0);   		// ��ʼ��PWM 10KHZ�������������
	BSP_PS2_Init();					// PS2�����˿ڳ�ʼ��
	BSP_PS2_SetInit();				// PS2���ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
	
	delay_ms(1000);
	BSP_EXTI_Init();                // MPU6050 5ms��ʱ�жϳ�ʼ��
	
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
