#include "app_control.h"
#include "app_communicate.h"

uint8_t period_5ms_count = 0;	  	// 

int EXTI15_10_IRQHandler(void)
{
	if (INT == 0) {
		EXTI->PR = 1 << 15;					 	// 清除LINE5上的中断标志位
		
		if (delay_flag == 1) {
			if (++delay_50 == 10)
				delay_50 = 0, delay_flag = 0; 	//给主函数提供50ms的精准延时
		}								   
		period_5ms_count++;
		Read_DMP();                    			// 更新姿态		
		
		if (period_5ms_count % 2 == 0) {
			/* 10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据 */
			Encoder_A = Read_Encoder(2);	   // 读取编码器的值
			Position_A += Encoder_A;		   // 积分得到速度
			Encoder_B = Read_Encoder(3);	   // 读取编码器的值
			Position_B += Encoder_B;		   // 积分得到速度
			Encoder_C = -Read_Encoder(4);	   // 读取编码器的值
			Position_C += Encoder_C;		   // 积分得到速度
			Encoder_D = -Read_Encoder(5);	   // 读取编码器的值
			Position_D += Encoder_D;		   // 积分得到速度
			Read_DMP();						   // 更新姿态
			LedFlash(100);					   // LED闪烁;常规模式 1s改变一次指示灯的状态
			
			Motor_A = Incremental_PI_A(Encoder_A, Target_A); 	// 速度闭环控制计算电机A最终PWM
			Motor_B = Incremental_PI_B(Encoder_B, Target_B); 	// 速度闭环控制计算电机B最终PWM
			Motor_C = Incremental_PI_C(Encoder_C, Target_C); 	// 速度闭环控制计算电机C最终PWM
			Motor_D = Incremental_PI_D(Encoder_D, Target_D); 	// 速度闭环控制计算电机C最终PWM
			
			Xianfu_Pwm(6900);							 		// PWM限幅 最大6900
			Set_Pwm(Motor_A, Motor_B, Motor_C, Motor_D); 		// 赋值给PWM寄存器
		}
		if (period_5ms_count >= 10) {
			/* 50ms执行一次 */
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
函数功能：小车运动数学模型
入口参数：X Y Z 三轴速度或者位置
返回  值：无
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
		// 小于阈值时，以最大速度躲避障碍物
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
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
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
函数功能：限制PWM赋值 
入口参数：幅值
返回  值：无
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
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A(int Encoder, int Target)
{
	static int Bias, Pwm, Last_bias;
	Bias = Encoder - Target;									  //计算偏差
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; //增量式PI控制器
	if (Pwm > 7200)
		Pwm = 7200;
	if (Pwm < -7200)
		Pwm = -7200;
	Last_bias = Bias; //保存上一次偏差
	return Pwm;		  //增量输出
}

int Incremental_PI_B(int Encoder, int Target)
{
	static int Bias, Pwm, Last_bias;
	Bias = Encoder - Target;									  //计算偏差
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; //增量式PI控制器
	if (Pwm > 7200)
		Pwm = 7200;
	if (Pwm < -7200)
		Pwm = -7200;
	Last_bias = Bias; //保存上一次偏差
	return Pwm;		  //增量输出
}

int Incremental_PI_C(int Encoder, int Target)
{
	static int Bias, Pwm, Last_bias;
	Bias = Encoder - Target;									  //计算偏差
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; //增量式PI控制器
	if (Pwm > 7200)
		Pwm = 7200;
	if (Pwm < -7200)
		Pwm = -7200;
	Last_bias = Bias; //保存上一次偏差
	return Pwm;		  //增量输出
}

int Incremental_PI_D(int Encoder, int Target)
{
	static int Bias, Pwm, Last_bias;
	Bias = Encoder - Target;									  //计算偏差
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; //增量式PI控制器
	if (Pwm > 7200)
		Pwm = 7200;
	if (Pwm < -7200)
		Pwm = -7200;
	Last_bias = Bias; //保存上一次偏差
	return Pwm;		  //增量输出
}


