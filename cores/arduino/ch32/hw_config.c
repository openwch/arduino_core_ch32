
#include "hw_config.h"
#include "clock.h"
#include "core_riscv_ch32yyxx.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CH32V10x
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
#else  //QingKe_V3A registers are different from others 
void systick_init(void)
{
    uint32_t tmp = 
    SystemCoreClock / 1000 / 8 - 1;
    SysTick->CTLR= 0;
    SysTick->CNTL0 = 0;SysTick->CNTL1 = 0;SysTick->CNTL2 = 0;SysTick->CNTL3 = 0;
    SysTick->CNTH0 = 0;SysTick->CNTH1 = 0;SysTick->CNTH2 = 0;SysTick->CNTH3 = 0;
    SysTick->CMPLR0=(uint8_t)(tmp);
    SysTick->CMPLR1=(uint8_t)((tmp)>>8);
    SysTick->CMPLR2=(uint8_t)((tmp)>>16);
    SysTick->CMPLR3=(uint8_t)((tmp)>>24);
    SysTick->CMPHR0=0;SysTick->CMPHR1=0;SysTick->CMPHR2=0;SysTick->CMPHR3=0;
    NVIC_SetPriority(SysTicK_IRQn,0xFF);
    NVIC_EnableIRQ(SysTicK_IRQn);
    SysTick->CTLR=0x1;
}
#endif

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


