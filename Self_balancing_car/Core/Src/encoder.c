#include "encoder.h"

//编码器测速

/*计数器0~65536，如果是负数：值就是65536-n；
	short类型是-32768~32768就能正常显示负数了；例如short中的-1->65535*/

int Encoder_ReadSpeed(TIM_HandleTypeDef *htim)
{
	int temp;
	temp = (short)__HAL_TIM_GetCounter(htim);
	__HAL_TIM_SetCounter(htim,0);//读完要清除cnt,为下次测量做准备
	return temp;
}
