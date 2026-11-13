#ifndef _CH32_DEF_
#define _CH32_DEF_


/**
 * @brief CH32V core version number
 */
#define CH32_CORE_VERSION_MAJOR    (0x01U) /*!< [31:24] major version */
#define CH32_CORE_VERSION_MINOR    (0x00U) /*!< [23:16] minor version */
#define CH32_CORE_VERSION_PATCH    (0x00U) /*!< [15:8]  patch version */
/*
 * Extra label for development:
 * 0: official release
 * [1-9]: release candidate
 * F[0-9]: development
 */
#define CH32_CORE_VERSION_EXTRA    (0x00U) /*!< [7:0]  extra version */
#define CH32_CORE_VERSION          ((CH32_CORE_VERSION_MAJOR << 24U)\
                                        |(CH32_CORE_VERSION_MINOR << 16U)\
                                        |(CH32_CORE_VERSION_PATCH << 8U )\
                                        |(CH32_CORE_VERSION_EXTRA))


/*
 *All defined 
 */
#if defined(CH32V20x) || defined(CH32V203xB) || defined(CH32V208)
  #include "ch32v20x.h"
#elif defined(CH32V30x) || defined(CH32V30x_C)
  #include "ch32v30x.h"
#elif defined(CH32V10x)
  #include "ch32v10x.h"
#elif defined(CH32V00x)
  #include "ch32v00x.h" 
#elif defined(CH32X035)
#include "ch32x035.h" 
#elif defined(CH32L10x)
#include "ch32l103.h"
#elif defined(CH32VM00X)
#include "ch32v00X.h"
#elif defined(CH32V4x7)
  #include "ch32v4x7.h"
#else 
  #error "CH32YYXX chip series is not defined in boards.txt."
#endif

#ifndef F_CPU
  #define F_CPU SystemCoreClock
#endif

#if defined(CH32V4x7)
  // The V4x7 SDK calls the APB1/APB2 buses PB1/PB2
  #define RCC_APB1PeriphClockCmd   RCC_PB1PeriphClockCmd
  #define RCC_APB1PeriphResetCmd   RCC_PB1PeriphResetCmd
  #define RCC_APB2PeriphClockCmd   RCC_PB2PeriphClockCmd
  #define RCC_APB2PeriphResetCmd   RCC_PB2PeriphResetCmd

  #define RCC_APB2Periph_AFIO      RCC_PB2Periph_AFIO
  #define RCC_APB2Periph_GPIOA     RCC_PB2Periph_GPIOA
  #define RCC_APB2Periph_GPIOB     RCC_PB2Periph_GPIOB
  #define RCC_APB2Periph_GPIOC     RCC_PB2Periph_GPIOC
  #define RCC_APB2Periph_GPIOD     RCC_PB2Periph_GPIOD
  #define RCC_APB2Periph_GPIOE     RCC_PB2Periph_GPIOE
  #define RCC_APB2Periph_USART1    RCC_PB2Periph_USART1
  #define RCC_APB2Periph_SPI1      RCC_PB2Periph_SPI1
  #define RCC_APB2Periph_ADC1      RCC_PB2Periph_ADC1
  #define RCC_APB2Periph_ADC2      RCC_PB2Periph_ADC2
  #define RCC_APB2Periph_TIM1      RCC_PB2Periph_TIM1
  #define RCC_APB2Periph_TIM8      RCC_PB2Periph_TIM8

  #define RCC_APB1Periph_USART2    RCC_PB1Periph_USART2
  #define RCC_APB1Periph_USART3    RCC_PB1Periph_USART3
  #define RCC_APB1Periph_USART4    RCC_PB1Periph_USART4
  #define RCC_APB1Periph_USART5    RCC_PB1Periph_USART5
  #define RCC_APB1Periph_USART6    RCC_PB1Periph_USART6
  #define RCC_APB1Periph_USART7    RCC_PB1Periph_USART7
  #define RCC_APB1Periph_USART8    RCC_PB1Periph_USART8
  #define RCC_APB1Periph_USART9    RCC_PB1Periph_USART9
  #define RCC_APB1Periph_USART10   RCC_PB1Periph_USART10
  #define RCC_APB1Periph_SPI2      RCC_PB1Periph_SPI2
  #define RCC_APB1Periph_SPI3      RCC_PB1Periph_SPI3
  #define RCC_APB1Periph_I2C1      RCC_PB1Periph_I2C1
  #define RCC_APB1Periph_TIM2      RCC_PB1Periph_TIM2
  #define RCC_APB1Periph_TIM3      RCC_PB1Periph_TIM3
  #define RCC_APB1Periph_TIM4      RCC_PB1Periph_TIM4
  #define RCC_APB1Periph_TIM5      RCC_PB1Periph_TIM5
  #define RCC_APB1Periph_TIM6      RCC_PB1Periph_TIM6
  #define RCC_APB1Periph_TIM7      RCC_PB1Periph_TIM7
#endif

// Here define some compatibility
#ifndef ADC1
  #define ADC1 ADC
#endif
#ifndef CAN1
  #define CAN1 CAN
#endif
#ifndef DAC1           
  #define DAC1 DAC   //two independent channels
#endif


/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
  #define WEAK __attribute__ ((weak))
#endif

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus


#if defined(NDEBUG)
#if !defined(_Error_Handler)
#define _Error_Handler(str, value) \
  while (1) {\
  }
#endif
#if !defined(Error_Handler)
#define Error_Handler() \
  while (1) {\
  }
#endif
#else
void _Error_Handler(const char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#endif

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif //_CH32_DEF_
