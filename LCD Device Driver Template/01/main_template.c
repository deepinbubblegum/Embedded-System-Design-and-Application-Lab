/* USER CODE BEGIN Includes */
#include "koibitoex_lcd2.h"
/* USER CODE END Includes */

/*----------------------------------------------------------------------------*/

  /* USER CODE BEGIN 2 */
	uint8_t Str1[] = "Cursor On";
	uint8_t Str2[20];
	uint16_t round = 1;
	
	LcdInit();
  /* USER CODE END 2 */

/*----------------------------------------------------------------------------*/
  
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    LcdCls(LCD_CLS_ALL);
    LcdSetCursor(LCD_CURSOR_OFF);
    LcdSetPosition(LCD_LINE_1, 0);
    LcdPutS((uint8_t *)"Hello Embedded!");
    HAL_Delay(2000);

    LcdCls(LCD_CLS_LINE_1);
    LcdSetCursor(LCD_CURSOR_ON);
    LcdSetPosition(LCD_LINE_2, 5);
    LcdPutS(Str1);
    HAL_Delay(2000);

    LcdCls(LCD_CLS_LINE_2);
    LcdSetCursor(LCD_CURSOR_OFF);
    LcdSetPosition(LCD_LINE_1, 1);
    sprintf((char *)Str2, "Round : %d", round);
    LcdPutS(Str2);
    round++;
    HAL_Delay(2000);

    LcdCls(LCD_CLS_LINE_1);
    LcdSetPosition(LCD_LINE_1, 2);
    LcdPutCh('B');
    LcdPutS((uint8_t *)"acklight Test");
    LcdSetCursor(LCD_CURSOR_OFF);
    LcdSetBackLight(LCD_BACKLIGHT_OFF);
    HAL_Delay(1000);
    LcdSetBackLight(LCD_BACKLIGHT_ON);
    HAL_Delay(1000);
    /* USER CODE END WHILE */
	
/*----------------------------------------------------------------------------*/