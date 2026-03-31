#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f1xx_hal.h"

int Encoder_ReadSpeed(TIM_HandleTypeDef *htim);

#endif
