#define ARDUINO_MAIN

#include "Arduino.h"
#include "debug.h"


/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
    pre_init( );
#if defined(USE_TINYUSB)
    TinyUSB_Device_Init(0);
#endif
    setup( );
  
    do {
        loop( );
#if defined(USE_TINYUSB)
      TinyUSB_Device_Task();
      TinyUSB_Device_FlushCDC();
#endif
    } while (1);

    return 0;
}
