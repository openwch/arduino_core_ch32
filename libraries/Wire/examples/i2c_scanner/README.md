# I2C Scanner
Original I2C Scanner example by Nick Gammon
I2C Scanner - Written by Nick Gammon - Date: 20th April 2011 [source](http://arduino-info.wikispaces.com/LCD-Blue-I2C).

Demonstrates CH32 Support for I2C scanning using Wire.endTransmission() without sending data. 
To enable scanning these changes are required in libraries/Wire/src/utility/twi.c : 
- timeout on an addresses should release the bus
- allow only sending the address (without actual data)
- smaller timeout: I2C_TIMEOUT_TICK 25 (was 100ms)

Note: Currently there's no support for [Wire.setWireTimeout(timeout, reset_on_timeout)](https://www.arduino.cc/reference/en/language/functions/communication/wire/setwiretimeout/).
Inspiration from [i2c_scanner](https://github.com/mockthebear/easy-ch32v003/tree/main/examples/i2c_scanner) by @mockthebear.
