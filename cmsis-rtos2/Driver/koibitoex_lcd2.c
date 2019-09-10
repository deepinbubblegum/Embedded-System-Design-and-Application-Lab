/**
  ******************************************************************************
  * File Name          : koibitoex_lcd2.c
  * Description        : Character-LCD device driver for Koibito_Ex board
  ******************************************************************************
  * COPYRIGHT(c) 2019 Dr.Panan Potipantong
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "koibitoex_lcd2.h"

/* Private define ------------------------------------------------------------*/

#define LCD_CONTROL_PORT GPIOB
#define RS  GPIO_PIN_0
#define RW  GPIO_PIN_1
#define E   GPIO_PIN_2

#define LCD_DATA_PORT GPIOC
#define DB4 GPIO_PIN_0 
#define DB5 GPIO_PIN_1
#define DB6 GPIO_PIN_2
#define DB7 GPIO_PIN_3

#define LCD_BLK_PORT GPIOE
#define BLK GPIO_PIN_7

/* Private function prototypes -----------------------------------------------*/
static void LcdWriteNibble(uint8_t data);
static void LcdWriteCmd(uint8_t lcdCmd);
static void LcdWriteData(uint8_t lcdData);

/* API functions -------------------------------------------------------------*/

void LcdInit(void)
{
  /* RS = 0 */
  HAL_GPIO_WritePin(LCD_CONTROL_PORT, RS, GPIO_PIN_RESET);
  /* RW = 0 */
  HAL_GPIO_WritePin(LCD_CONTROL_PORT, RW, GPIO_PIN_RESET);
  /* E = 0 */
  HAL_GPIO_WritePin(LCD_CONTROL_PORT, E, GPIO_PIN_RESET);
  /* BLK = 1 */
  HAL_GPIO_WritePin(LCD_BLK_PORT, BLK, GPIO_PIN_SET);
  /* Start-Up Delay */
  HAL_Delay(15);
  
  LcdWriteNibble(0x30);
  HAL_Delay(5);
  LcdWriteNibble(0x30);
  HAL_Delay(1);
  LcdWriteNibble(0x30);
  HAL_Delay(1);
  LcdWriteNibble(0x20);
  HAL_Delay(1);
  
  /* Function set */
  LcdWriteCmd(0x28);
  HAL_Delay(1);

  /* Display on/off control */
  LcdWriteCmd(0x0F);
  HAL_Delay(1);

  /* Clear display */
  LcdWriteCmd(0x01);
  HAL_Delay(2);

  /* Entry mode set */
  LcdWriteCmd(0x06);
  HAL_Delay(1);
  
  /* Set Position */
  LcdWriteCmd(0x07 | 0x80);
  HAL_Delay(1);
}

void LcdCls(uint8_t status)
{
  if(status == LCD_CLS_ALL)
  {
    LcdWriteCmd(0x01);
  }
  else if(status == LCD_CLS_LINE_1)
  {
		LcdSetPosition(LCD_LINE_1, 0);
		LcdPutS((uint8_t *)"                ");
  }
  else if(status == LCD_CLS_LINE_2)
  {
		LcdSetPosition(LCD_LINE_2, 0);
		LcdPutS((uint8_t *)"                ");
  }
  HAL_Delay(1);
  
}

void LcdSetCursor(uint8_t status)
{
	if(status == LCD_CURSOR_ON)
  {
		LcdWriteCmd(0x0F);
  }
	else if(status == LCD_CURSOR_OFF)
  {
		LcdWriteCmd(0x0C);
  }
	HAL_Delay(1);
}

void LcdSetBackLight(uint8_t status)
{
  if(status == LCD_BACKLIGHT_ON)
  {
    HAL_GPIO_WritePin(LCD_BLK_PORT, BLK, GPIO_PIN_SET);
  }
  else if(status == LCD_BACKLIGHT_OFF)
  {
    HAL_GPIO_WritePin(LCD_BLK_PORT, BLK, GPIO_PIN_RESET);
  }
  HAL_Delay(1);
}

void LcdSetPosition(uint8_t line, uint8_t pos)
{
	if(line == LCD_LINE_1)
  {
		LcdWriteCmd(pos | 0x80);
  }
	if(line == LCD_LINE_2)
  {
		LcdWriteCmd((pos + 0x40) | 0x80);
  }
	HAL_Delay(1);		
}

void LcdPutCh(uint8_t c)
{
	LcdWriteData(c);
	HAL_Delay(1);		
}

void LcdPutS(uint8_t *s)
{
	unsigned char i = 0;
	while(*(s+i))
	{
		LcdWriteData(*(s+i));
		HAL_Delay(1);	
		i++;
	}
}

static void LcdWriteNibble(uint8_t data)
{
  HAL_GPIO_WritePin(LCD_DATA_PORT, DB7, data & 0x80);
  HAL_GPIO_WritePin(LCD_DATA_PORT, DB6, data & 0x40);
  HAL_GPIO_WritePin(LCD_DATA_PORT, DB5, data & 0x20);
  HAL_GPIO_WritePin(LCD_DATA_PORT, DB4, data & 0x10);
  
  HAL_GPIO_WritePin(LCD_CONTROL_PORT, E, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CONTROL_PORT, E, GPIO_PIN_RESET);
}

static void LcdWriteCmd(uint8_t lcdCmd)
{
	HAL_GPIO_WritePin(LCD_CONTROL_PORT, RS, GPIO_PIN_RESET);
	LcdWriteNibble(lcdCmd & 0xf0);
	LcdWriteNibble(lcdCmd << 4);
}

static void LcdWriteData(uint8_t lcdData)
{
	HAL_GPIO_WritePin(LCD_CONTROL_PORT, RS, GPIO_PIN_SET);
	LcdWriteNibble(lcdData & 0xf0);
	LcdWriteNibble(lcdData << 4);
}

/*********************************END OF FILE**********************************/
