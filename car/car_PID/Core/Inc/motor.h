#ifndef _MOTOR_H
#define _MOTOR_H

#include "stm32f1xx_hal.h"

void Load(int motor1,int motor2);
void Moter_Limit(int *motorA,int *motorB);
void Stop(float *Med_Jiaodu,float *Jiaodu);
#endif
