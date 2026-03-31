#include "encoder.h"

int Read_Speed(TIM_HandleTypeDef *htim)
{
	int temp;
	/*计数器0~65536，如果是负数：值就是65536-n；
	short类型是-32768~32768就能正常显示负数了*/
	temp = (short)__HAL_TIM_GetCounter(htim);
	__HAL_TIM_SetCounter(htim,0);
	return temp;
}
