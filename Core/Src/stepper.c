/*
 * stepper.c
 *
 *  Created on: 10 Nov 2022
 *      Author: Jaroslaw
 */
#include "stepper.h"

void delay(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void stepper_set_rpm(int rpm) // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	delay(60000000 / stepsperrev / rpm);
}

void stepper_half_drive(int step) {
	switch (step) {
	case 0:
		HAL_GPIO_WritePin(STEPIN1_GPIO_Port, STEPIN1_Pin, GPIO_PIN_SET);   // IN1
		HAL_GPIO_WritePin(STEPIN2_GPIO_Port, STEPIN2_Pin, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(STEPIN3_GPIO_Port, STEPIN3_Pin, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(STEPIN4_GPIO_Port, STEPIN4_Pin, GPIO_PIN_RESET);   // IN4
		break;

	case 1:
		HAL_GPIO_WritePin(STEPIN1_GPIO_Port, STEPIN1_Pin, GPIO_PIN_SET);   // IN1
		HAL_GPIO_WritePin(STEPIN2_GPIO_Port, STEPIN2_Pin, GPIO_PIN_SET);   // IN2
		HAL_GPIO_WritePin(STEPIN3_GPIO_Port, STEPIN3_Pin, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(STEPIN4_GPIO_Port, STEPIN4_Pin, GPIO_PIN_RESET);   // IN4
		break;

	case 2:
		HAL_GPIO_WritePin(STEPIN1_GPIO_Port, STEPIN1_Pin, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(STEPIN2_GPIO_Port, STEPIN2_Pin, GPIO_PIN_SET);   // IN2
		HAL_GPIO_WritePin(STEPIN3_GPIO_Port, STEPIN3_Pin, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(STEPIN4_GPIO_Port, STEPIN4_Pin, GPIO_PIN_RESET);   // IN4
		break;

	case 3:
		HAL_GPIO_WritePin(STEPIN1_GPIO_Port, STEPIN1_Pin, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(STEPIN2_GPIO_Port, STEPIN2_Pin, GPIO_PIN_SET);   // IN2
		HAL_GPIO_WritePin(STEPIN3_GPIO_Port, STEPIN3_Pin, GPIO_PIN_SET);   // IN3
		HAL_GPIO_WritePin(STEPIN4_GPIO_Port, STEPIN4_Pin, GPIO_PIN_RESET);   // IN4
		break;

	case 4:
		HAL_GPIO_WritePin(STEPIN1_GPIO_Port, STEPIN1_Pin, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(STEPIN2_GPIO_Port, STEPIN2_Pin, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(STEPIN3_GPIO_Port, STEPIN3_Pin, GPIO_PIN_SET);   // IN3
		HAL_GPIO_WritePin(STEPIN4_GPIO_Port, STEPIN4_Pin, GPIO_PIN_RESET);   // IN4
		break;

	case 5:
		HAL_GPIO_WritePin(STEPIN1_GPIO_Port, STEPIN1_Pin, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(STEPIN2_GPIO_Port, STEPIN2_Pin, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(STEPIN3_GPIO_Port, STEPIN3_Pin, GPIO_PIN_SET);   // IN3
		HAL_GPIO_WritePin(STEPIN4_GPIO_Port, STEPIN4_Pin, GPIO_PIN_SET);   // IN4
		break;

	case 6:
		HAL_GPIO_WritePin(STEPIN1_GPIO_Port, STEPIN1_Pin, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(STEPIN2_GPIO_Port, STEPIN2_Pin, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(STEPIN3_GPIO_Port, STEPIN3_Pin, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(STEPIN4_GPIO_Port, STEPIN4_Pin, GPIO_PIN_SET);   // IN4
		break;

	case 7:
		HAL_GPIO_WritePin(STEPIN1_GPIO_Port, STEPIN1_Pin, GPIO_PIN_SET);   // IN1
		HAL_GPIO_WritePin(STEPIN2_GPIO_Port, STEPIN2_Pin, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(STEPIN3_GPIO_Port, STEPIN3_Pin, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(STEPIN4_GPIO_Port, STEPIN4_Pin, GPIO_PIN_SET);   // IN4
		break;
	}
}

void stepper_step_angle(float angle, int rpm) {
	float anglepersequence = 0.703125;  // 360 = 512 sequences
	int numberofsequences = (int) (angle / anglepersequence);

	for (int seq = 0; seq < numberofsequences; seq++) {
		for (int step = 7; step >= 0; step--) {
			stepper_half_drive(step);
			stepper_set_rpm(rpm);
		}
	}
}
