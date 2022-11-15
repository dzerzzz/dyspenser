/*
 * pump.c
 *
 *  Created on: Nov 15, 2022
 *      Author: Jaroslaw
 */

#include "pump.h"

void StartPump(void) {
	//status = Pouring; //TODO: Add status
	HAL_TIM_Base_Stop_IT(&htim10); //TODO: Check if necessary
	HAL_GPIO_WritePin(PUMPIN1_GPIO_Port, PUMPIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); //TODO: Remove this after tests
	HAL_TIM_Base_Start_IT(&htim10);
}

void StopPump(void) {
	//status = Idle; //TODO: Add status
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); //TODO: Remove this after tests
	HAL_GPIO_WritePin(PUMPIN1_GPIO_Port, PUMPIN1_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Stop_IT(&htim10);
}

void ChangePumpPourTime(int ms) {
	switch (ms) {
	case 1000:
		TIM10->PSC = 19999; //Do calculation for proper value
		TIM10->ARR = 3599; ///As above
		break;
	case 2000: //TODO: This is wrong
		TIM10->PSC = 19999; //Do calculation for proper value
		TIM10->ARR = 1799; ///As above
		break;
	case 3000: //TODO: This is wrong
		TIM10->PSC = 19999; //Do calculation for proper value
		TIM10->ARR = 12000; ///As above
		break;
	case 4000:
		TIM10->PSC = 19999;
		TIM10->ARR = 14399;
		break;
	default:
		TIM10->PSC = 19999; //Do calculation for proper value
		TIM10->ARR = 6544; ///As above
		break;
	}
}
