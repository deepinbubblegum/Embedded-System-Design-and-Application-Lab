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


/* API functions -------------------------------------------------------------*/

void LcdInit(void)
{

}

void LcdCls(uint8_t status)
{

}

void LcdSetCursor(uint8_t status)
{

}

void LcdSetBackLight(uint8_t status)
{

}

void LcdSetPosition(uint8_t line, uint8_t pos)
{
	
}

void LcdPutCh(uint8_t c)
{
	
}

void LcdPutS(uint8_t *s)
{

}

static void LcdWriteNibble(uint8_t lcdData)
{

}

static void LcdWriteCmd(uint8_t lcdCmd)
{

}

static void LcdWriteData(uint8_t lcdData)
{

}

/*********************************END OF FILE**********************************/
