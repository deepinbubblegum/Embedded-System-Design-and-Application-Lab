/**
  ******************************************************************************
  * File Name          : koibitoex_lcd2.h
  * Description        : Character-LCD device driver for Koibito_Ex board
  ******************************************************************************
  * COPYRIGHT(c) 2019 Dr.Panan Potipantong
  ******************************************************************************
  */

/* Include guard, Macro guard */
/* https://en.wikipedia.org/wiki/Include_guard */
#ifndef KOIBITOEX_LCD2_H
#define KOIBITOEX_LCD2_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Public define -------------------------------------------------------------*/

#define LCD_LINE_1					0
#define LCD_LINE_2					1

#define LCD_CURSOR_OFF			0
#define LCD_CURSOR_ON				1

#define LCD_BACKLIGHT_OFF		0
#define LCD_BACKLIGHT_ON		1

#define LCD_CLS_ALL					0
#define LCD_CLS_LINE_1			1
#define LCD_CLS_LINE_2			2

/* Public function prototypes ------------------------------------------------*/

void LcdInit(void);
void LcdCls(uint8_t status);
void LcdSetCursor(uint8_t status);
void LcdSetBackLight(uint8_t status);
void LcdSetPosition(uint8_t line, uint8_t pos);
void LcdPutCh(uint8_t c);
void LcdPutS(uint8_t *s);

#endif /* KOIBITOEX_LCD2_H */

/*********************************END OF FILE**********************************/
