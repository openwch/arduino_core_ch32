#ifndef WIRING_SERIAL_H
#define WIRING_SERIAL_H

#include "variant.h"
#include "HardwareSerial.h"


#if defined(UART_MODULE_ENABLED) && !defined(UART_MODULE_ONLY)

  #if !defined(HWSERIAL_NONE) && defined(SERIAL_UART_INSTANCE)

    #if SERIAL_UART_INSTANCE == 1
      #define ENABLE_HWSERIAL1
      #if !defined(Serial)
        #define Serial Serial1
        // #define serialEvent serialEvent1  //reserved
      #endif  
    #elif SERIAL_UART_INSTANCE == 2
      #define ENABLE_HWSERIAL2
      #if !defined(Serial)
        #define Serial Serial2
        // #define serialEvent serialEvent2
      #endif
    #elif SERIAL_UART_INSTANCE == 3
      #define ENABLE_HWSERIAL3
      #if !defined(Serial)
        #define Serial Serial3
        // #define serialEvent serialEvent3
      #endif
    #elif SERIAL_UART_INSTANCE == 4
      #define ENABLE_HWSERIAL4
      #if !defined(Serial)
        #define Serial Serial4
        // #define serialEvent serialEvent4
      #endif
    #elif SERIAL_UART_INSTANCE == 5
      #define ENABLE_HWSERIAL5
      #if !defined(Serial)
        #define Serial Serial5
        // #define serialEvent serialEvent5
      #endif
    #elif SERIAL_UART_INSTANCE == 6
      #define ENABLE_HWSERIAL6
      #if !defined(Serial)
        #define Serial Serial6
        // #define serialEvent serialEvent6
      #endif
    #elif SERIAL_UART_INSTANCE == 7
      #define ENABLE_HWSERIAL7
      #if !defined(Serial)
        #define Serial Serial7
        // #define serialEvent serialEvent7
      #endif
    #elif SERIAL_UART_INSTANCE == 8
      #define ENABLE_HWSERIAL8
      #if !defined(Serial)
        #define Serial Serial8
        // #define serialEvent serialEvent8
      #endif
    #else
      #if !defined(Serial)
        #warning "No generic 'Serial' defined!"
      #endif

    #endif /* SERIAL_UART_INSTANCE == x */

  #endif /* !HWSERIAL_NONE && SERIAL_UART_INSTANCE */


  #if defined(USART1_BASE)
    #define HAVE_HWSERIAL1
  #endif
  #if defined(USART2_BASE)
    #define HAVE_HWSERIAL2
  #endif
  #if defined(USART3_BASE)
    #define HAVE_HWSERIAL3
  #endif
  #if defined(USART4_BASE) || defined(UART4_BASE)
    #define HAVE_HWSERIAL4
  #endif
  #if defined(UART5_BASE)
    #define HAVE_HWSERIAL5
  #endif
  #if defined(UART6_BASE)
    #define HAVE_HWSERIAL6
  #endif
  #if defined(UART7_BASE)
    #define HAVE_HWSERIAL7
  #endif
  #if defined(UART8_BASE)
    #define HAVE_HWSERIAL8
  #endif

#endif

#endif /* WIRING_SERIAL_H */
