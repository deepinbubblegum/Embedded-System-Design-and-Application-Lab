#include "led_circle.h"
#define LED_PORT GPIOE
#define LED_0 GPIO_PIN_8
#define LED_1 GPIO_PIN_9
#define LED_2 GPIO_PIN_10
#define LED_3 GPIO_PIN_11
#define LED_4 GPIO_PIN_12
#define LED_5 GPIO_PIN_13
#define LED_6 GPIO_PIN_14
#define LED_7 GPIO_PIN_15

void LedWriteCircle(uint8_t pos){
	HAL_GPIO_WritePin(LED_PORT, LED_0, (pos == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT, LED_1, (pos == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT, LED_2, (pos == 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT, LED_3, (pos == 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT, LED_4, (pos == 4) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT, LED_5, (pos == 5) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT, LED_6, (pos == 6) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT, LED_7, (pos == 7) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
