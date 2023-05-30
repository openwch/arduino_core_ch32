
#include "hw_config.h"
#include "clock.h"
#include "core_riscv_ch32yyxx.h"

#ifdef __cplusplus
extern "C" {
#endif

void systick_init(void)
{
    SysTick->SR  = 0;
    SysTick->CTLR= 0;
    SysTick->CNT = 0;
    SysTick->CMP = SystemCoreClock / 1000 - 1;
    SysTick->CTLR= 0xF;
    NVIC_SetPriority(SysTicK_IRQn,0xFF);
    NVIC_EnableIRQ(SysTicK_IRQn);
}


/**
  * @brief  This function performs the global init of the system (HAL, IOs...)
  * @param  None
  * @retval None
  */

void hw_config_init(void)
{
  systick_init();
}
#ifdef __cplusplus
}
#endif


