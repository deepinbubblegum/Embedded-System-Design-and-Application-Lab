#include "stm32f3xx_hal.h"

void Systick_Handler_Link(void)
{
  HAL_IncTick();
}
