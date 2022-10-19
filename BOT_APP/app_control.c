#include "app_control.h"
#include "app_communicate.h"

uint8_t period_5ms_count = 0;	  	// 

int EXTI15_10_IRQHandler(void)
{
	if (INT == 0) {
		EXTI->PR = 1 << 15;					 	// ���LINE5�ϵ��жϱ�־λ
		
		if (delay_flag == 1) {
			if (++delay_50 == 10)
				delay_50 = 0, delay_flag = 0; 	//���������ṩ50ms�ľ�׼��ʱ
		}								   
		period_5ms_count++;
		Read_DMP();                    			// ������̬		
		
		if (period_5ms_count % 2 == 0) {
			/* 10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ���������� */
			Encoder_A = Read_Encoder(2);	   // ��ȡ��������ֵ
			Position_A += Encoder_A;		   // ���ֵõ��ٶ�
			Encoder_B = Read_Encoder(3);	   // ��ȡ��������ֵ
			Position_B += Encoder_B;		   // ���ֵõ��ٶ�
			Encoder_C = -Read_Encoder(4);	   // ��ȡ��������ֵ
			Position_C += Encoder_C;		   // ���ֵõ��ٶ�
			Encoder_D = -Read_Encoder(5);	   // ��ȡ��������ֵ
			Position_D += Encoder_D;		   // ���ֵõ��ٶ�
			Read_DMP();						   // ������̬
			LedFlash(100);					   // LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬
			
			Motor_A = Incremental_PI_A(Encoder_A, Target_A); 	// �ٶȱջ����Ƽ�����A����PWM
			Motor_B = Incremental_PI_B(Encoder_B, Target_B); 	// �ٶȱջ����Ƽ�����B����PWM
			Motor_C = Incremental_PI_C(Encoder_C, Target_C); 	// �ٶȱջ����Ƽ�����C����PWM
			Motor_D = Incremental_PI_D(Encoder_D, Target_D); 	// �ٶȱջ����Ƽ�����C����PWM
			
			Xianfu_Pwm(6900);							 		// PWM�޷� ���6900
			Set_Pwm(Motor_A, Motor_B, Motor_C, Motor_D); 		// ��ֵ��PWM�Ĵ���
		}
		if (period_5ms_count >= 10) {
			/* 50msִ��һ�� */
			if (startFlag == 1) {
				if (CONTROL_MODE) {
					main_Controller();
					Kinematic_Analysis(targetVx, targetVy, 0);
				} else {
					targetVz = ((128.0 - PS2_LX) / 128.0) * 30.0;
					targetVy = ((128.0 - PS2_LY) / 128.0) * 20.0;
					targetVx = ((PS2_RX - 128.0) / 128.0) * 20.0;
					Kinematic_Analysis(targetVx, targetVy, targetVz);
				}
			} else {
				Kinematic_Analysis(0, 0, 0);
			}
			
			period_5ms_count = 0;
		}
	}
	return 0;
}

/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ�����X Y Z �����ٶȻ���λ��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float Vx, float Vy, float Vz)
{
	Target_A = -Vx + Vy - Vz * (a_PARAMETER + b_PARAMETER);
	Target_B = +Vx + Vy - Vz * (a_PARAMETER + b_PARAMETER);
	Target_C = -Vx + Vy + Vz * (a_PARAMETER + b_PARAMETER);
	Target_D = +Vx + Vy + Vz * (a_PARAMETER + b_PARAMETER);
}

void main_Controller(void)
{
	int dist = 0;
	int VGoal_X = 0, VGoal_Y = 0;
	int robot_obs_x = 0, robot_obs_y = 0, VRobo_X = 0, VRobo_Y = 0;
	int VObst_X = 0, VObst_Y = 0;
	int loop_i = 0;
	
	// move-to-goal
	dist = (targetPoint_X - groundTruth[0].x) * (targetPoint_X - groundTruth[0].x) + 
		(targetPoint_Y - groundTruth[0].y) * (targetPoint_Y - groundTruth[0].y);
	if (dist > 225) {
		VGoal_X = (targetPoint_X - groundTruth[0].x) / 12;
		VGoal_Y = (targetPoint_Y - groundTruth[0].y) / 12;
		Numerical_Limit(&VGoal_X, &VGoal_Y, 25);
	}
	
	if (advoidMode == 1) {
		// avoid-static-obstacle
		dist = (int)sqrt(obs_x * obs_x + obs_y * obs_y);
		// С����ֵʱ��������ٶȶ���ϰ���
		if (dist <= 150) {
			VObst_X = obs_x > 0 ? -150 : 150;
			VObst_Y = obs_y > 0 ? -150 : 150;
		} else if (dist <= 450) {
			int magnitude = 25 * ((float)(450 - dist) / (450.0 - 150));
			double theta = atan((double)abs(obs_y) / abs(obs_x));
			VObst_X = -1 * mysign(obs_x) * magnitude * cos(theta);
			VObst_Y = -1 * mysign(obs_y) * magnitude * sin(theta);
			Numerical_Limit(&VObst_X, &VObst_Y, 25);
		}
		
		// avoid-robot
		dist = INT_MAX;
		for (loop_i = 0; loop_i < 3; ++loop_i) {
			if (loop_i != AGENT_NUM) {
				int tmp_dist = (groundTruth[loop_i].x - groundTruth[AGENT_NUM].x) * (groundTruth[loop_i].x - groundTruth[AGENT_NUM].x) + 
					(groundTruth[loop_i].y - groundTruth[AGENT_NUM].y) * (groundTruth[loop_i].y - groundTruth[AGENT_NUM].y);
				if (tmp_dist < dist) {
					dist = tmp_dist;
					robot_obs_x = groundTruth[loop_i].x - groundTruth[AGENT_NUM].x;
					robot_obs_y = groundTruth[loop_i].y - groundTruth[AGENT_NUM].y;
				}
			}
		}
		if (dist <= 150) {
			VRobo_X = robot_obs_x > 0 ? -150 : 150;
			VRobo_Y = robot_obs_y > 0 ? -150 : 150;
		} else if (dist <= 350) {
			int magnitude = 25 * ((float)(350 - dist) / (350.0 - 150));
			double theta = atan((double)abs(robot_obs_y) / abs(robot_obs_x));
			VRobo_X = -1 * mysign(robot_obs_x) * magnitude * cos(theta);
			VRobo_Y = -1 * mysign(robot_obs_x) * magnitude * sin(theta);
			Numerical_Limit(&VRobo_X, &VRobo_Y, 25);
		}
	}
	
	targetVx = 0.6 * VGoal_X  + 0.285 * VRobo_X + 0.430 * VObst_X;
	targetVy = 0.6 * VGoal_Y  + 0.285 * VRobo_Y + 0.430 * VObst_Y;
}

void Numerical_Limit(int* x, int* y, int amplitude)
{
	int Maximum;
	
	Maximum = abs(*x) > abs(*y) ? abs(*x) : abs(*y);
	if (Maximum > amplitude) {
		*x = *x * amplitude / Maximum;
		*y = *y * amplitude / Maximum;
	}
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d)
{
	int siqu = 0;
	
	if (motor_a > 0) {
		PWMA = motor_a + siqu, INA = 0;
	} else {
		PWMA = 7200 + motor_a - siqu, INA = 1;
	}

	if (motor_b > 0) {
		PWMB = 7200 - motor_b - siqu, INB = 1;
	} else {
		PWMB = -motor_b + siqu, INB = 0;
	}

	if (motor_c > 0) {
		PWMC = 7200 - motor_c - siqu, INC = 1;
	} else {
		PWMC = -motor_c + siqu, INC = 0;
	}

	if (motor_d > 0) {
		PWMD = motor_d + siqu, IND = 0;
	} else {
		PWMD = 7200 + motor_d - siqu, IND = 1;
	}
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{
	if (Motor_A < -amplitude)	Motor_A = -amplitude;
	if (Motor_A > amplitude)	Motor_A = amplitude;
	if (Motor_B < -amplitude)	Motor_B = -amplitude;
	if (Motor_B > amplitude)	Motor_B = amplitude;
	if (Motor_C < -amplitude)	Motor_C = -amplitude;
	if (Motor_C > amplitude)	Motor_C = amplitude;
	if (Motor_D < -amplitude)	Motor_D = -amplitude;
	if (Motor_D > amplitude)	Motor_D = amplitude;
}

int8_t mysign(int num)
{
	if (num > 0) {
		return 1;
	} else if (num < 0) {
		return -1;
	}
	
	return 0;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A(int Encoder, int Target)
{
	static int Bias, Pwm, Last_bias;
	Bias = Encoder - Target;									  //����ƫ��
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; //����ʽPI������
	if (Pwm > 7200)
		Pwm = 7200;
	if (Pwm < -7200)
		Pwm = -7200;
	Last_bias = Bias; //������һ��ƫ��
	return Pwm;		  //�������
}

int Incremental_PI_B(int Encoder, int Target)
{
	static int Bias, Pwm, Last_bias;
	Bias = Encoder - Target;									  //����ƫ��
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; //����ʽPI������
	if (Pwm > 7200)
		Pwm = 7200;
	if (Pwm < -7200)
		Pwm = -7200;
	Last_bias = Bias; //������һ��ƫ��
	return Pwm;		  //�������
}

int Incremental_PI_C(int Encoder, int Target)
{
	static int Bias, Pwm, Last_bias;
	Bias = Encoder - Target;									  //����ƫ��
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; //����ʽPI������
	if (Pwm > 7200)
		Pwm = 7200;
	if (Pwm < -7200)
		Pwm = -7200;
	Last_bias = Bias; //������һ��ƫ��
	return Pwm;		  //�������
}

int Incremental_PI_D(int Encoder, int Target)
{
	static int Bias, Pwm, Last_bias;
	Bias = Encoder - Target;									  //����ƫ��
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; //����ʽPI������
	if (Pwm > 7200)
		Pwm = 7200;
	if (Pwm < -7200)
		Pwm = -7200;
	Last_bias = Bias; //������һ��ƫ��
	return Pwm;		  //�������
}


