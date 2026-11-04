#ifndef _PINAF_CH573_H
#define _PINAF_CH573_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
  AFIO_NONE,
    // ENABLE:
    /* GPIO_Remap_define */
    /* R16_PIN_ALTERNATE */
  AFIO_Remap_SPI0_ENABLE,
  AFIO_Remap_SPI0_DISABLE,

  AFIO_Remap_UART1_ENABLE,
  AFIO_Remap_UART1_DISABLE,

  AFIO_Remap_UART0_ENABLE,
  AFIO_Remap_UART0_DISABLE,

  AFIO_Remap_TMR2_ENABLE,
  AFIO_Remap_TMR2_DISABLE,

  AFIO_Remap_TMR1_ENABLE,
  AFIO_Remap_TMR1_DISABLE,

  AFIO_Remap_TMR0_ENABLE,
  AFIO_Remap_TMR0_DISABLE,
};

static inline void pinV32_DisconnectDebug(PinName pin)
{
  /** Enable this flag gives the possibility to use debug pins without any risk
    * to lose traces
    */
  pin;
  //ch573 seems can onlt change it in the chip config
}


static inline void pin_SetV32AFPin(uint32_t afnum)
{
  // CH573 does not have AFIO clock
  switch (afnum) {
    case AFIO_Remap_SPI0_ENABLE:
      GPIOPinRemap(ENABLE, RB_SPI_CLK);
      break;
    case AFIO_Remap_SPI0_DISABLE:
      GPIOPinRemap(DISABLE, RB_SPI_CLK);
      break;
    case AFIO_Remap_UART0_ENABLE:
      GPIOPinRemap(ENABLE, RB_UART_TXD | RB_UART_RXD);
      break;
    case AFIO_Remap_UART0_DISABLE:
      GPIOPinRemap(DISABLE, RB_UART_TXD | RB_UART_RXD);
      break;
    case AFIO_Remap_TMR0_ENABLE:
      GPIOPinRemap(ENABLE, RB_TMR_PIN);
      break;
    case AFIO_Remap_TMR0_DISABLE:
      GPIOPinRemap(DISABLE, RB_TMR_PIN);
      break;
    default:
    case AFIO_NONE:
      break;
  }
}


#ifdef __cplusplus
}
#endif


#endif