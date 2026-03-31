#ifndef __HC_SR04_H__
#define __HC_SR04_H__

#include "stm32f1xx_hal.h"

void Send_Trig(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


#endif
