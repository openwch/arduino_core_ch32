#define ARDUINO_MAIN

#include "Arduino.h"
#include "debug.h"


/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
    pre_init( );
    setup( );
  
    do {
        loop( );
      
    } while (1);

    return 0;
}
