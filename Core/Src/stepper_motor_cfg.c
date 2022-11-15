/*
 * stepper_motor_cfg.c
 *
 *  Created on: Nov 15, 2022
 *      Author: Jaroslaw
 */

/*
 * File: STEPPER_cfg.c
 * Driver Name: [[ STEPPER Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "stepper_motor.h"
#include "main.h"

const STEPPER_CfgType STEPPER_CfgParam[STEPPER_UNITS] =
{
	// Stepper Motor 1 Configurations
    {
	    {STEPIN1_GPIO_Port, STEPIN2_GPIO_Port, STEPIN3_GPIO_Port, STEPIN4_GPIO_Port},
		{STEPIN1_Pin, STEPIN2_Pin, STEPIN3_Pin, STEPIN4_Pin},
		4096,
		STEPPER_UNIPOLAR,
		HALF_STEP_DRIVE
	}
};

