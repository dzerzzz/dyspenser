/*
 * stepper.h
 *
 *  Created on: 10 Nov 2022
 *      Author: Jaroslaw
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "main.h"

extern TIM_HandleTypeDef htim2;

#define stepsperrev 4096

void delay(uint16_t us);
void stepper_set_rpm(int rpm);
void stepper_half_drive(int step);
void stepper_step_angle(float angle, int rpm);
void StepperCallback(void);
void StepperStart(void);
#endif /* INC_STEPPER_H_ */
