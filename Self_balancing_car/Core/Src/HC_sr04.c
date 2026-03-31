#include "hc_sr04.h"
#include "pid.h"

uint16_t cnt;
float distance;
extern TIM_HandleTypeDef htim3;


void RCCdelay_us(uint32_t udelay)//微秒级延时函数
{
  __IO uint32_t Delay = udelay * 72 / 8;//(SystemCoreClock / 8U / 1000000U)
    //见stm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
  do
  {
    __NOP();
  }
  while (Delay --);
}

//超声波发送Trig
void Send_Trig(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	RCCdelay_us(12);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
}

//中断回调函数
//获取echo高电平时长,并计算出距离
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_2)
	{
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)==GPIO_PIN_SET)
		{
			__HAL_TIM_SetCounter(&htim3,0);
			HAL_TIM_Base_Start(&htim3);
		}else 
		{
			HAL_TIM_Base_Stop(&htim3);
			cnt = __HAL_TIM_GetCounter(&htim3);
			distance = cnt*0.017;//distance = cnt*1e-6f*340/2*100cm;
		}
	}
	if(GPIO_Pin == GPIO_PIN_5)
	{
		Control();
	}
}
