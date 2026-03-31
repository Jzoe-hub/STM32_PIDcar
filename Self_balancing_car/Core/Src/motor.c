#include "motor.h"


extern TIM_HandleTypeDef htim1;
extern uint8_t Stop;
#define PWM_MAX 7200
#define PWM_MIN -7200

//取绝对值
int abs(int p)
{
	if(p>0)
	return p;
	else 
	return -p;
}

//电机速度方向大小
void Motor_Set(int motor1,int motor2)
{
	//电机1（A）
	if(motor1>0)//正转动
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,abs(motor1));
	//电机2(B)
	if(motor2>0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,abs(motor2));
}
//限制电机转速
void Motor_Limit(int *motorA,int *motorB)
{
	if(*motorA>PWM_MAX)*motorA=PWM_MAX;
	if(*motorA<PWM_MIN)*motorA=PWM_MIN;
	if(*motorB>PWM_MAX)*motorB=PWM_MAX;
	if(*motorB<PWM_MIN)*motorB=PWM_MIN;
}
//倾倒保护
void Motor_Stop(float *Med_Jiaodu,float *Jiaodu)
{
	if(abs((int)(*Jiaodu-*Med_Jiaodu))>60)
	{
		Motor_Set(0,0);
		Stop = 1;
	}
}
