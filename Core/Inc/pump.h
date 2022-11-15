/*
 * pump.h
 *
 *  Created on: Nov 15, 2022
 *      Author: Jaroslaw
 */

#ifndef INC_PUMP_H_
#define INC_PUMP_H_

#include "main.h"

extern TIM_HandleTypeDef htim10;

void StopPump(void);
void StartPump(void);
void ChangePumpPourTime(int ms);

#endif /* INC_PUMP_H_ */
