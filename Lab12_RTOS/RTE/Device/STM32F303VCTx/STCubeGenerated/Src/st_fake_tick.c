#include "st_fake_tick.h"

osThreadId_t tid_StFakeTick;
static void StFakeTick(void * argument);

const osThreadAttr_t StFakeTick_attr =
{ .stack_size = 256,
  .priority = osPriorityHigh
};

void StFakeTickInit(void)
{
  tid_StFakeTick = osThreadNew(StFakeTick ,NULL, &StFakeTick_attr);
}

static void StFakeTick(void * argument)
{
  while(1)
  {
    HAL_IncTick();
    osDelay(1);
  }
}
