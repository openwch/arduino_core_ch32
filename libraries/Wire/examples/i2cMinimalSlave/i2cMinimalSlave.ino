/*
  I2C Minimal Slave - CH32 I2C Slave example

  Show the minimal code required to implement an I2C slave device with these features:
     - handle I2C scanning (i.e. acknowledge data-less transaction)
     - handle receiving data
     - return data requested by master
     - blink LED on PA2 according transfered value
  
  Compiled for CH32V003J4M6 (SOP8) using Arduino IDE v2.3.2, CH32 core v1.0.4. 
  Optimize: Smallest (-Os default), Debug symbols: none, no UART: //#define UART_MODULE_ENABLED
  Note - enable I2C slave functionality in /libraries/Wire/src/utility/twi.h
    #define OPT_I2C_SLAVE 1

  Using those options this sketch uses 9424 bytes (57%) of program storage space. Maximum is 16384 bytes.
  Global variables use 580 bytes (28%) of dynamic memory, leaving 1468 bytes for local variables. Maximum is 2048 bytes.
*/

#include <Wire.h>
#define MY_I2C_ADDRESS 0x33
 
#undef LED_BUILTIN
#define LED_BUILTIN PA2
uint32_t _nBlinkSpeed=500;

void ReceiveEvent(int nNumBytes) {
  // receive value is stored to set the speed of blinking
  _nBlinkSpeed=0;
  while(nNumBytes>0)
  {
    _nBlinkSpeed<<=8;
    _nBlinkSpeed|=Wire.read();
    nNumBytes--;
  }
}

void RequestEvent() {
  // returned 32-bit value is the current speed of blinking
  Wire.write((const uint8_t *)&_nBlinkSpeed, sizeof(_nBlinkSpeed));;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize digital pin LED_BUILTIN as an output.
  Wire.begin(MY_I2C_ADDRESS);
  // Note: enable I2C slave functionality in /libraries/Wire/src/utility/twi.h to prevent error message for next two lines
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);
}

void loop() {
  static uint32_t uStart=millis();
  if(millis()-uStart > _nBlinkSpeed)
  { // blink the LED by reversing its state
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    uStart=millis();
  }
}