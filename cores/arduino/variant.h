
#pragma once

#include VARIANT_H

// Resolve Board Default (or a platform without a routing menu) to its port.
#if !defined(SERIAL_UART_INSTANCE) || SERIAL_UART_INSTANCE == 0
  #undef SERIAL_UART_INSTANCE
  #if defined(BOARD_DEFAULT_SERIAL)
    #define SERIAL_UART_INSTANCE BOARD_DEFAULT_SERIAL
  #else
    #define SERIAL_UART_INSTANCE 1
  #endif
#endif

// A variant exposes a hardware port by supplying both of its default pins.
// Keep all exposed ports available independently of the Serial alias.
#if !defined(HWSERIAL_NONE) && defined(SERIAL_UART_ENABLE_ALL)
    #if !defined(ENABLE_HWSERIAL1) && (defined(PIN_SERIAL1_RX) && defined(PIN_SERIAL1_TX))
        #define ENABLE_HWSERIAL1
    #endif
    #if !defined(ENABLE_HWSERIAL2) && (defined(PIN_SERIAL2_RX) && defined(PIN_SERIAL2_TX))
        #define ENABLE_HWSERIAL2
    #endif
    #if !defined(ENABLE_HWSERIAL3) && (defined(PIN_SERIAL3_RX) && defined(PIN_SERIAL3_TX))
        #define ENABLE_HWSERIAL3
    #endif
    #if !defined(ENABLE_HWSERIAL4) && (defined(PIN_SERIAL4_RX) && defined(PIN_SERIAL4_TX))
        #define ENABLE_HWSERIAL4
    #endif
    #if !defined(ENABLE_HWSERIAL5) && (defined(PIN_SERIAL5_RX) && defined(PIN_SERIAL5_TX))
        #define ENABLE_HWSERIAL5
    #endif
    #if !defined(ENABLE_HWSERIAL6) && (defined(PIN_SERIAL6_RX) && defined(PIN_SERIAL6_TX))
        #define ENABLE_HWSERIAL6
    #endif
    #if !defined(ENABLE_HWSERIAL7) && (defined(PIN_SERIAL7_RX) && defined(PIN_SERIAL7_TX))
        #define ENABLE_HWSERIAL7
    #endif
    #if !defined(ENABLE_HWSERIAL8) && (defined(PIN_SERIAL8_RX) && defined(PIN_SERIAL8_TX))
        #define ENABLE_HWSERIAL8
    #endif
#endif

// Compatibility pin aliases use the same port as WSerial.h.
#define SERIAL_PIN_NAME_EXPAND(port, direction) PIN_SERIAL##port##_##direction
#define SERIAL_PIN_NAME(port, direction) SERIAL_PIN_NAME_EXPAND(port, direction)
#if defined(SERIAL_UART_INSTANCE) && SERIAL_UART_INSTANCE > 0
    #ifndef PIN_SERIAL_RX
        #define PIN_SERIAL_RX SERIAL_PIN_NAME(SERIAL_UART_INSTANCE, RX)
    #endif
    #ifndef PIN_SERIAL_TX
        #define PIN_SERIAL_TX SERIAL_PIN_NAME(SERIAL_UART_INSTANCE, TX)
    #endif

    // A variant exposes a hardware port by supplying both of its default pins.
    // Keep all exposed ports available independently of the Serial alias.
    #if !defined(HWSERIAL_NONE)
        #if SERIAL_UART_INSTANCE == 1 && !defined(ENABLE_HWSERIAL1)
            #define ENABLE_HWSERIAL1
        #elif SERIAL_UART_INSTANCE == 2 && !defined(ENABLE_HWSERIAL2)
            #define ENABLE_HWSERIAL2
        #elif SERIAL_UART_INSTANCE == 3 && !defined(ENABLE_HWSERIAL3)
            #define ENABLE_HWSERIAL3
        #elif SERIAL_UART_INSTANCE == 4 && !defined(ENABLE_HWSERIAL4)
            #define ENABLE_HWSERIAL4
        #elif SERIAL_UART_INSTANCE == 5 && !defined(ENABLE_HWSERIAL5)
            #define ENABLE_HWSERIAL5
        #elif SERIAL_UART_INSTANCE == 6 && !defined(ENABLE_HWSERIAL6)
            #define ENABLE_HWSERIAL6
        #elif SERIAL_UART_INSTANCE == 7 && !defined(ENABLE_HWSERIAL7)
            #define ENABLE_HWSERIAL7
        #elif SERIAL_UART_INSTANCE == 8 && !defined(ENABLE_HWSERIAL8)
            #define ENABLE_HWSERIAL8
        #endif
    #endif
#endif
