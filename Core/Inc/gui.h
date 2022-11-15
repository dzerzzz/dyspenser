/*
 * gui.h
 *
 *  Created on: Nov 9, 2022
 *      Author: Jaroslaw
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

#include "bitmaps.h"
#include "main.h"
#include "ssd1306.h"

typedef enum{
	StartLayer,
	MenuLayer,
	PutShot,
	SettingsLayer,
	Searching
} GuiState;

extern TIM_HandleTypeDef htim11;

extern int flagDirectionPut;
extern int animation;
extern int flagDirectionSearch;
extern int fluid;

void Print(GuiState state);
void DrawShotFill(void);
void DrawShotSearching(void);
void ClearScreen(void);
void SetAnimationTime(void);

#endif /* INC_GUI_H_ */
