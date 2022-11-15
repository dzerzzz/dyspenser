/*
 * gui.c
 *
 *  Created on: Nov 9, 2022
 *      Author: Jaroslaw
 */
#include "gui.h"
int flagDirectionPut = 0;
int animation = 0;
int flagDirectionSearch = 0;
int fluid = 40;
void SetAnimationTime(void){
	HAL_TIM_Base_Stop_IT(&htim11);

	if (animation == 0)
	{
		if (flagDirectionPut == 0)
		{
			flagDirectionPut = 1;
			animation = 20;
		}
		else
		{
			flagDirectionPut = 0;
			animation = 10;
		}

		if (flagDirectionSearch == 0)
		{
			flagDirectionSearch = 1;
			animation = 20;
		}
		else
		{
			flagDirectionSearch = 0;
			animation = 20;
		}
	}

	if (fluid == 0)
	{
		fluid = 40;
	}
	fluid--;

	animation--;
	HAL_TIM_Base_Start_IT(&htim11);
}

void DrawShotSearching(void) {
	if (flagDirectionSearch == 1)
	{
		SSD1306_DrawFilledRectangle(10, 0, 100, 60, 0);
		SSD1306_DrawBitmap(20+animation, 0, lupa, 60, 58, 1);
		SSD1306_UpdateScreen();
	}
	else
	{
		SSD1306_DrawFilledRectangle(10, 0, 100, 60, 0);
		SSD1306_DrawBitmap(40-animation, 0, lupa, 60, 58, 1);
		SSD1306_UpdateScreen();
	}
}

void DrawShotFill(void) {
		SSD1306_DrawBitmap(40, 0, kielon, 40, 60, 1);
		SSD1306_UpdateScreen();
		//ciecz
		int16_t byteWidth = (30 + 7) / 8;
		uint8_t byte = 0;
		int16_t x = 45;
		int16_t y = 44;
		for (int16_t j = 40; j > fluid; j--, y--) {
			for (int16_t i = 0; i < 30; i++) {
				if (i & 7) {
					byte <<= 1;
				} else {
					byte = (*(const unsigned char*) (&ciecz[j * byteWidth
							+ i / 8]));
				}
				if (byte & 0x80)
					SSD1306_DrawPixel(x + i, y, 1);
			}
	}
}

void Print(GuiState state) {
	switch (state) {
	case StartLayer:
//		if(animation>10)
//		{
			SSD1306_GotoXY(0, 25);
			SSD1306_Puts("Press key to start", &Font_7x10, 1);
			SSD1306_UpdateScreen();
//		}
//		else
//		{
//			ClearScreen();
//		}
		break;
	default:
		break;
	}
}

void ClearScreen(void)
{
	SSD1306_Clear();
}
