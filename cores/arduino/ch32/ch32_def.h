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
#elif defined(CH32V30x)
  #include "ch32v30x.h"
#elif defined(CH32V10x)
  #include "ch32v10x.h"
#elif defined(CH32V00x)
  #include "ch32v00x.h" 
#else 
  #error "CH32YYXX chip series is not defined in boards.txt."
#endif

#ifndef F_CPU
  #define F_CPU SystemCoreClock
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
