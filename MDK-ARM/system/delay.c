#include "system.h"	  
#include "delay.h"	  

void delay_ms(u32 nms)
{
	HAL_Delay(nms);
}

void HAL_Delay_us(uint32_t us)
{       
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000000);
    HAL_Delay(us-1);
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
}

void delay_us(u32 nus)
{
	HAL_Delay_us(nus);
}

