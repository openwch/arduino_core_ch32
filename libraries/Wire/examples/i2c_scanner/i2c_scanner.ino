
// I2C Scanner
// Written by Nick Gammon
// Date: 20th April 2011

// source: http://arduino-info.wikispaces.com/LCD-Blue-I2C

// MMOLE: CH32 Support I2C scanning using Wire.endTransmission() without sending data
// CH32 changes required in libraries/Wire/src/utility/twi.c
//  - timeout on an addresses should release the bus
//  - allow only sending the address (without actual data)
//  - smaller timeout: I2C_TIMEOUT_TICK 25 (was 100ms)
// Note: Currently there's no support for Wire.setWireTimeout(timeout, reset_on_timeout)
// https://www.arduino.cc/reference/en/language/functions/communication/wire/setwiretimeout/
// Inspiration from https://github.com/mockthebear/easy-ch32v003/tree/main/examples/i2c_scanner



#include <Wire.h>

void setup() {
  Serial.begin (115200);

  // Leonardo: wait for serial port to connect
  while (!Serial) 
    {
    }

  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  
  Wire.begin();
  while(true)
  {

  byte count = 0;
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
  delay(1000);
  Serial.println ("Scanning again...");
  }
}  // end of setup

void loop() {}
