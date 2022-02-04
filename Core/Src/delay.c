#include "delay.h"

#define DELAY_MAX_MS_FOR_DELAY_US ( UINT16_MAX/1000 )

void delay_us (uint32_t nus)
{
  __HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
  while (__HAL_TIM_GET_COUNTER(&htim2) < nus);  // wait for the counter to reach the us input in the parameter
}

void delay_ms(uint16_t nms)
{
  // Call delay_us as few times as possible
  if (nms <= DELAY_MAX_MS_FOR_DELAY_US) {
    delay_us(nms*1000);
  }
  else {
    uint16_t nmsRemaining = nms;
    while (nmsRemaining > DELAY_MAX_MS_FOR_DELAY_US) {
      delay_us(DELAY_MAX_MS_FOR_DELAY_US*1000);
      nmsRemaining -= DELAY_MAX_MS_FOR_DELAY_US;
    }
    if (nmsRemaining > 0) {
      delay_us(nmsRemaining*1000);
    }
  }
}


//static uint8_t  fac_us=0;
//static uint16_t fac_ms=0;
//
//void systickInit (uint16_t frequency)
//{
//   RCC_ClocksTypeDef RCC_Clocks;
//   RCC_GetClocksFreq (&RCC_Clocks);
//   (void) SysTick_Config (RCC_Clocks.HCLK_Frequency / frequency);
//}
//
//void delay_init()
//{
//  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
//  fac_us=SystemCoreClock/8000000;
//  fac_ms=(uint16_t)fac_us*1000;
//}
//
//void delay_us(uint32_t nus)
//{
//  uint32_t temp;
//  SysTick->LOAD=nus*fac_us;
//  SysTick->VAL=0x00;
//  SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;
//  do
//  {
//    temp=SysTick->CTRL;
//  }while((temp&0x01)&&!(temp&(1<<16)));
//  SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
//  SysTick->VAL =0X00;
//}
//
//void delay_ms(uint16_t nms)
//{
//  uint32_t temp;
//  SysTick->LOAD=(uint32_t)nms*fac_ms;
//  SysTick->VAL =0x00;
//  SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;
//  do
//  {
//    temp=SysTick->CTRL;
//  }while((temp&0x01)&&!(temp&(1<<16)));
//  SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
//  SysTick->VAL =0X00;
//}







































