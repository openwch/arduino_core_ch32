/*
  EEPROM - Enables reading and writing to non-volatile storage in the processor.
  
  Uses the Option bytes area in Flash to emulate EEPROM.
  On the CH32V003 there are 2+24 bytes available. 
  The first two bytes are Option bytes data0 and data1.
  All bytes are copied to a 26-byte long byte array in RAM.
  After changing a value the commit() method is used to write flash.

  Ported by Maxint R&D to CH32, based on multiple sources.
  Tested on CH32V003 using Arduino IDE 2.3.2 and OpenWCH core 1.0.4. 
  Includes code from Option Data example of CH32V003fun by CNLOHR.
  Arduino original copyright (c) 2006 David A. Mellis.  All right reserved.
  ESP8266 version copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef EEPROM_h
#define EEPROM_h
#include <Arduino.h>

// Required definitions copied from /system/CH32V00x/SRC/Peripheral/src/ch32v00x_flash.c
/* Flash Control Register bits */
//#define CR_PG_Set                  ((uint32_t)0x00000001)
//#define CR_PG_Reset                ((uint32_t)0xFFFFFFFE)
//#define CR_PER_Set                 ((uint32_t)0x00000002)
//#define CR_PER_Reset               ((uint32_t)0xFFFFFFFD)
//#define CR_MER_Set                 ((uint32_t)0x00000004)
//#define CR_MER_Reset               ((uint32_t)0xFFFFFFFB)
#define CR_OPTPG_Set               ((uint32_t)0x00000010)
#define CR_OPTPG_Reset             ((uint32_t)0xFFFFFFEF)
#define CR_OPTER_Set               ((uint32_t)0x00000020)
#define CR_OPTER_Reset             ((uint32_t)0xFFFFFFDF)
#define CR_STRT_Set                ((uint32_t)0x00000040)
#define CR_LOCK_Set                ((uint32_t)0x00000080)
//#define CR_PAGE_PG                 ((uint32_t)0x00010000)
//#define CR_PAGE_ER                 ((uint32_t)0x00020000)
//#define CR_BUF_LOAD                ((uint32_t)0x00040000)
//#define CR_BUF_RST                 ((uint32_t)0x00080000)

/* FLASH Keys */
//#define RDP_Key                    ((uint16_t)0x00A5)
#define FLASH_KEY1                 ((uint32_t)0x45670123)
#define FLASH_KEY2                 ((uint32_t)0xCDEF89AB)


class EEPROMClass {
  public:
    EEPROMClass(void);
    ~EEPROMClass(void);

    void begin(void);

    uint8_t * getDataPtr();
    uint8_t const * getConstDataPtr() const;
    uint32_t ReadOptionBytes(void);   // return data0 and data1 option bytes including their inversed values

    uint8_t read( int const idx );
    void write( int const idx, uint8_t const val);     // requires commit() to make data stick
    void erase(void);     // requires commit() to make data stick

    bool commit(void);
    bool end(void);

    template<typename T> 
    T &get(int const address, T &t) {
      if (address < 0 || address + sizeof(T) > _size)
        return t;
      memcpy((uint8_t*) &t, _data + address, sizeof(T));
      return t;
    }

    template<typename T> 
    const T &put(int const address, const T &t) {
      if (address < 0 || address + sizeof(T) > _size)
        return t;
      if (memcmp(_data + address, (const uint8_t*)&t, sizeof(T)) != 0) {
        _dirty = true;
        memcpy(_data + address, (const uint8_t*)&t, sizeof(T));
      }
      return t;
    }

    size_t length() {return _size;}

    uint8_t& operator[](int const address) {return getDataPtr()[address];}
    uint8_t const & operator[](int const address) const {return getConstDataPtr()[address];}

  protected:
    uint8_t* _data = nullptr;
    bool _dirty = false;
    size_t _size = 0;
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_EEPROM)
extern EEPROMClass EEPROM;
#endif


#endif		// #ifndef EEPROM_h