#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f1xx_hal.h"

void Motor_Set(int motor1,int motor2);
void Motor_Limit(int *motorA,int *motorB);
void Motor_Stop(float *Med_Jiaodu,float *Jiaodu);

#endif
