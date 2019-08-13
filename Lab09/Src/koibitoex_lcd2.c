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

#define LCD2_DB_GPIO_PORT		GPIOC
#define LCD2_DB4_GPIO_PIN		GPIO_PIN_0
#define LCD2_DB5_GPIO_PIN		GPIO_PIN_1
#define LCD2_DB6_GPIO_PIN		GPIO_PIN_2
#define LCD2_DB7_GPIO_PIN		GPIO_PIN_3

#define LCD2_CTR_GPIO_PORT	GPIOB
#define LCD2_RS_GPIO_PIN		GPIO_PIN_0
#define LCD2_RW_GPIO_PIN		GPIO_PIN_1
#define LCD2_E_GPIO_PIN			GPIO_PIN_2

#define LCD2_BL_GPIO_PORT		GPIOE
#define LCD2_BL_GPIO_PIN		GPIO_PIN_7

#define RS(x)		HAL_GPIO_WritePin(LCD2_CTR_GPIO_PORT, LCD2_RS_GPIO_PIN, (GPIO_PinState)x);
#define RW(x)		HAL_GPIO_WritePin(LCD2_CTR_GPIO_PORT, LCD2_RW_GPIO_PIN, (GPIO_PinState)x);	
#define E(x)		HAL_GPIO_WritePin(LCD2_CTR_GPIO_PORT, LCD2_E_GPIO_PIN, (GPIO_PinState)x);	
#define BL(x)		HAL_GPIO_WritePin(LCD2_BL_GPIO_PORT, LCD2_BL_GPIO_PIN, (GPIO_PinState)x);

/* Private function prototypes -----------------------------------------------*/

static void LcdWriteNibble(uint8_t lcdData);
static void LcdWriteCmd(uint8_t lcdCmd);
static void LcdWriteData(uint8_t lcdData);

/* API functions -------------------------------------------------------------*/

void LcdInit(void)
{
		RS(0);
		RW(0);
		E(0);
		BL(1);
	
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
}

void LcdCls(uint8_t status)
{
	if(status == LCD_CLS_LINE_1)
	{
		LcdSetPosition(LCD_LINE_1, 0);
		LcdPutS((uint8_t *)"                ");
	}
	else if(status == LCD_CLS_LINE_2)
	{
		LcdSetPosition(LCD_LINE_2, 0);
		LcdPutS((uint8_t *)"                ");
	}
	else if(status == LCD_CLS_ALL)
	{
		LcdWriteCmd(0x01);
		HAL_Delay(2);
	}
}

void LcdSetCursor(uint8_t status)
{
	if(status == LCD_CURSOR_ON)
		LcdWriteCmd(0x0F);
	else if(status == LCD_CURSOR_OFF)
		LcdWriteCmd(0x0C);
	HAL_Delay(1);
}

void LcdSetBackLight(uint8_t status)
{
	BL(status);
}

void LcdSetPosition(uint8_t line, uint8_t pos)
{
	if(line == LCD_LINE_1)
			LcdWriteCmd(pos | 0x80);
	if(line == LCD_LINE_2)
			LcdWriteCmd((pos + 0x40) | 0x80);
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

static void LcdWriteNibble(uint8_t lcdData)
{
	HAL_GPIO_WritePin(LCD2_DB_GPIO_PORT, LCD2_DB7_GPIO_PIN, (GPIO_PinState)(lcdData & 0x80));
	HAL_GPIO_WritePin(LCD2_DB_GPIO_PORT, LCD2_DB6_GPIO_PIN, (GPIO_PinState)(lcdData & 0x40));
	HAL_GPIO_WritePin(LCD2_DB_GPIO_PORT, LCD2_DB5_GPIO_PIN, (GPIO_PinState)(lcdData & 0x20));
	HAL_GPIO_WritePin(LCD2_DB_GPIO_PORT, LCD2_DB4_GPIO_PIN, (GPIO_PinState)(lcdData & 0x10));

	E(1);
	E(0);
}

static void LcdWriteCmd(uint8_t lcdCmd)
{
	RS(0);
	LcdWriteNibble(lcdCmd & 0xf0);
	LcdWriteNibble(lcdCmd << 4);
}

static void LcdWriteData(uint8_t lcdData)
{
	RS(1);
	LcdWriteNibble(lcdData & 0xf0);
	LcdWriteNibble(lcdData << 4);
}

/*********************************END OF FILE**********************************/
