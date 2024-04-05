## **EEPROM Library** for Arduino CH32

### **What is the EEPROM library.**

The EEPROM library provides an easy to use interface to interact with the internal non-volatile storage found in Arduino boards. 
This CH32 version of the library uses the Option bytes area in flash memory to emulate EEPROM and provides a similar API.

Ported by Maxint R&D to CH32, based on multiple sources.
Includes code from Option Data example of CH32V003fun by CNLOHR.
Arduino original copyright (c) 2006 David A. Mellis.  All right reserved. New version by Christopher Andrews 2015.
ESP8266 version copyright (c) 2014 Ivan Grokhotkov. All rights reserved.

## Table of contents
- [CH32V003 emulated EEPROM](#ch32v003-emulated-eeprom)
- [How to use it](#how-to-use-this-library)
- [Library functions](#library-functions)
- [Features & limitations](#features--limitations)
- [Disclaimer](#disclaimer)

### CH32V003 emulated EEPROM 
On the CH32V003 there are 2+24 bytes available. The first two bytes are Option bytes data0 and data1.
All bytes are copied to a 26-byte long byte array in RAM. After changing a value the commit() method is used to write flash.

From the CH32V004 Data Sheet:
  "Built-in 1920 bytes of system storage (System FLASH) for system bootloader storage (factory-cured
  bootloader) 64 bytes are used for the system non-volatile configuration information storage area and 64 bytes
  are used for the user select word storage area."
So next to 16kB of Code FLASH, the chip features 2kB Flash for boot, configuration and storage. That 64 bytes
user select word storage area is the area we can use to emulate on chip EEPROM memory.

From the CH32V003 Reference Manual:
  "The user-option bytes is solidified in FLASH and will be reloaded into the corresponding register after system
  reset, and can be erased and programmed by the user at will. The user option bytes information block has a
  total of 8 bytes (4 bytes for write protection, 1 byte for read protection, 1 byte for configuration options, and
  2 bytes for storing user data), and each bit has its inverse code bit for checksum during loading."
These 8 bytes use 16 bytes of flash. The total storage area page is 64 bytes, leaving 48 bytes available, 
(including required inverse values). This is 24 bytes that can be used plus the two data0 and data1 bytes. 
Layout for uint8_t _data[26]: { ob[4], ob[6], ob[16...62] ].

The first release of this library was made for the CH32V003 and only uses the user select word storage area. 
It was tested using Arduino IDE 2.3.2 and OpenWCH core 1.0.4. 
Future releases of this library may support other CH32 processors and allow for larger memory sizes.

### **How to use this library**
To use the library in your sketch you'll need to reference the library header file. You do this by adding an include directive to the top of your sketch. The library provides a global object variable named `EEPROM`. You use this object to access the library functions that are listed below.
This EEPROM library requires you to call EEPROM.begin() to initialize the object.

```Arduino
#include <EEPROM.h>

void setup(){
  EEPROM.begin();
}

void loop(){

}

```

You can find complete examples [here](examples/).

---


### **Library functions**

#### **`EEPROM.begin()`** 

This method initializes the EEPROM object. It reads the current data from permanent storage into a RAM memory buffer to allow speedy access. After writing data to memory, EEPROM.commit() needs to be called to save the data into permanent storage.

This method does not return any value.

#### **`EEPROM.erase()`** 

This method erases the RAM memory of the EEPROM object. Erased bytes have a default value of 255 (0xFF). After erasing the RAM memory, EEPROM.commit() needs to be called to erase the permanent storage.

This method does not return any value.

#### **`EEPROM.commit()`** [[_example_]](examples/eeprom_counter/eeprom_counter.ino)

The EEPROM object uses a RAM memory buffer to allow speedy access to its data. After writing data to memory, EEPROM.commit() needs to be called to save the data into permanent storage. 

This method returns the `bool` true value to indicate success. The current implementations uses loops to wait for success.

#### **`EEPROM.length()`** [[_example_]](examples/eeprom_counter/eeprom_counter.ino)

This method returns an `size_t` containing the number of elements in the emulated EEPROM. On the CH32V003 there are 2+24 bytes available. _Future releases of this library may allow the begin() method to specify a larger size._

#### **`EEPROM.read( address )`** [[_example_]](examples/eeprom_read/eeprom_read.ino)

This method allows you to read a single byte of data from the EEPROM.
Its only parameter is an `int` which should be set to the address you wish to read.

The method returns the byte (`unsigned char` / `uint8_t`) containing the value read.

#### **`EEPROM.write( address, value )`** [[_example_]](examples/eeprom_write/eeprom_write.ino)

The `write()` method allows you to write a single byte of data to the EEPROM.
Two parameters are needed. The first is an `int` containing the address that is to be written, and the second is a the byte to be written (`unsigned char` / `uint8_t`). After writing data to memory, EEPROM.commit() needs to be called to save the data into permanent storage.

This method does not return any value.

#### **`EEPROM.get( address, variable )`** [[_example_]](examples/eeprom_get/eeprom_get.ino)

This method will retrieve any type of data from the EEPROM.
Two parameters are needed to call this method. The first is an `int` containing the address that is to be written, and the second is the variable you would like to read.

This method returns a reference to the `variable` passed in. It does not need to be used and is only returned for convenience.

#### **`EEPROM.put( address, variable )`** [[_example_]](examples/eeprom_put/eeprom_put.ino)

This method will write any type of data to the EEPROM.
Two parameters are needed to call this method. The first is an `int` containing the address that is to be written, and the second is the variable you would like to write. After writing data to memory, EEPROM.commit() needs to be called to save the data into permanent storage.

This method returns a reference to the `variable` passed in. It does not need to be used and is only returned for convenience.

#### **Subscript operator: `EEPROM[address]`** [[_example_]](examples/eeprom_crc/eeprom_crc.ino)

This operator allows using the `EEPROM` object like an array.  
EEPROM RAM elements can be read _and_ **_written_** directly using this method. After writing data to memory, EEPROM.commit() needs to be called to save the data into permanent storage.

This operator returns a reference to the EEPROM RAM elements.

```c++
uint8_t val;

//Read first EEPROM RAM element.
val = EEPROM[ 0 ];

//Write first EEPROM RAM element.
EEPROM[ 0 ] = val;

//Compare contents
if( val == EEPROM[ 0 ] ){
  //Do something...
}
```

#### **`EEPROM.ReadOptionBytes()`** [[_example_]](examples/eeprom_counter/eeprom_counter.ino)

This method is made available to show how the CH32 stores data in the user select word storage area.
The 8 words in the user option bytes information block are reloaded into their corresponding register after system reset.
For this reason the data0 and data1 bytes within that block can be read even before EEPROM.begin() is called.
After calling EEPROM.begin() these data0 and data1 bytes are copied into addresses 0 and 1 of the EEPROM RAM buffer.

The method returns a `uint32_t` value, containing the data0 and data1 bytes and their inversed values.

---

## Features & limitations
- The first release of this library was made only for the CH32V003 and has been tested on that MCU only. Other memmbers of the CH32 may behave incorrectly or not work at all. 
- The CH32V003 has only 26 bytes available. When addressing more things are likely to go wrong. A future release may allow using more pages from the flash memory.
- Most CH32 EEPROM methods are the same as their equivalent on regular Arduino's. BEWARE: The begin() and end() methods fumction like their ESP8266/ESP, but are very different from the begin() and end() methods of EEPROM v2.0 by Christopher Andrews, who introduced them to support C++ iterators. This library follows the begin() convention introduced by the Serial and Wire classes, i.e. to initialize the object.

## Disclaimer
- All code on this GitHub account, including this library is provided to you on an as-is basis without guarantees and with all liability dismissed. It may be used at your own risk. Unfortunately I have no means to provide support.
