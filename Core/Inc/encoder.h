/*
 * encoder.h
 *
 *  Created on: Nov 9, 2022
 *      Author: Jaroslaw
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

extern TIM_HandleTypeDef htim3;

void SetMaxValue(int value);
void ResetPosition(void);
#endif /* INC_ENCODER_H_ */
