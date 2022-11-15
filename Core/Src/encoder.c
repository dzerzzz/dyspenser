/*
 * encoder.c
 *
 *  Created on: Nov 9, 2022
 *      Author: Jaroslaw
 */

#include "encoder.h"

void SetMaxValue(int value)
{
	__HAL_TIM_SET_COUNTER(&htim3, value * 4); //TODO: Remove this after test;
}

void ResetPosition(void)
{
	__HAL_TIM_SET_COUNTER(&htim3, 0);
}
