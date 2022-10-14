#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "sys.h"

#define PI 3.14159265
#define a_PARAMETER (0.095f)
#define b_PARAMETER (0.086f)

int EXTI15_10_IRQHandler(void);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d);
void Kinematic_Analysis(float Vx,float Vy,float Vz);
void Key(void);
void Xianfu_Pwm(int amplitude);
int8_t mysign(int num);

void main_Controller(void);
void Numerical_Limit(int* x, int* y, int amplitude);
int Incremental_PI_A (int Encoder,int Target);
int Incremental_PI_B (int Encoder,int Target);
int Incremental_PI_C (int Encoder,int Target);
int Incremental_PI_D (int Encoder,int Target);

#endif
