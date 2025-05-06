/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

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

  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 3 December 2013 by Matthijs Kooijman
  Modified 1 May 2023 by TempersLee
  Modified 13 October 2023 by Maxint R&D, latest update 6 May 2025
*/

#ifndef HardwareSerial_h
#define HardwareSerial_h

// MMOLE 240619: set OPT_USART_INT to 1 if you want to use interrupts for receiving serial data.
#define OPT_USART_INT 1
#define OPT_PR180 1  // PR180: HardwareSerial: use correct UART HW for TX

#if 1

#include <inttypes.h>
#include "Stream.h"
#include "uart.h"

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.
// NOTE: a "power of 2" buffer size is recommended to dramatically
//       optimize all the modulo operations for ring buffers.
// WARNING: When buffer sizes are increased to > 256, the buffer index
// variables are automatically increased in size, but the extra
// atomicity guards needed for that are not implemented. This will
// often work, but occasionally a race condition can occur that makes
// Serial behave erratically. See https://github.com/arduino/Arduino/issues/2405
#if !defined(SERIAL_TX_BUFFER_SIZE)
  #define SERIAL_TX_BUFFER_SIZE 64
#endif
#if !defined(SERIAL_RX_BUFFER_SIZE)
  #define SERIAL_RX_BUFFER_SIZE 64
#endif
#if (SERIAL_TX_BUFFER_SIZE>256)
  typedef uint16_t tx_buffer_index_t;
#else
  typedef uint8_t tx_buffer_index_t;
#endif
#if  (SERIAL_RX_BUFFER_SIZE>256)
  typedef uint16_t rx_buffer_index_t;
#else
  typedef uint8_t rx_buffer_index_t;
#endif

// A bool should be enough for this
// But it brings an build error due to ambiguous
// call of overloaded HardwareSerial(int, int)
// So defining a dedicated type
typedef enum {
  HALF_DUPLEX_DISABLED,
  HALF_DUPLEX_ENABLED
} HalfDuplexMode_t;

// Define config for Serial.begin(baud, config);
/* databits 8 */
#define SERIAL_8N1      0x02     
#define SERIAL_8N2      0x0A
#define SERIAL_8N0_5    0x06
#define SERIAL_8N1_5    0x0E
#define SERIAL_8E1      0x22
#define SERIAL_8E2      0x2A
#define SERIAL_8E0_5    0x26
#define SERIAL_8E1_5    0x2E
#define SERIAL_8O1      0x32
#define SERIAL_8O2      0x3A
#define SERIAL_8O0_5    0x36
#define SERIAL_8O1_5    0x3E
/* databits 9 */
#define SERIAL_9N1      0x03 
#define SERIAL_9N2      0x0B
#define SERIAL_9N0_5    0x07
#define SERIAL_9N1_5    0x0F
#define SERIAL_9E1      0x23
#define SERIAL_9E2      0x2B
#define SERIAL_9E0_5    0x27 
#define SERIAL_9E1_5    0x2F
#define SERIAL_9O1      0x33 
#define SERIAL_9O2      0x3B
#define SERIAL_9O0_5    0x37
#define SERIAL_9O1_5    0x3F  




class HardwareSerial : public Stream {

#if(OPT_USART_INT==1)
public:
#endif
  serial_t _serial;  
public:
    HardwareSerial(void *peripheral); 
    
    void begin(unsigned long baud)
    {
      begin(baud, SERIAL_8N1);     //SERIAL_9E1_5  SERIAL_8N1
    }
    // MMOLE: reintroduced RX buffer to properly implement read/available/peek methods
    volatile rx_buffer_index_t _rx_buffer_head;
    volatile rx_buffer_index_t _rx_buffer_tail;
    //volatile tx_buffer_index_t _tx_buffer_head;
    //volatile tx_buffer_index_t _tx_buffer_tail;

    // Don't put any members after these buffers, since only the first
    // 32 bytes of this struct can be accessed quickly using the ldd
    // instruction.
    unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
    //unsigned char _tx_buffer[SERIAL_TX_BUFFER_SIZE];
    void begin(unsigned long, uint8_t);
    void end();

    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);


    
    virtual size_t write(uint8_t);
    inline size_t write(unsigned long n)
    {
      return write((uint8_t)n);
    }
    inline size_t write(long n)
    {
      return write((uint8_t)n);
    }
    inline size_t write(unsigned int n)
    {
      return write((uint8_t)n);
    }
    inline size_t write(int n)
    {
      return write((uint8_t)n);
    }
    size_t write(const uint8_t *buffer, size_t size);
    using Print::write; // pull in write(str) from Print
    operator bool()
    {
      return true;
    }

    void setRx(uint32_t _rx);
    void setTx(uint32_t _tx);
    void setRx(PinName _rx);
    void setTx(PinName _tx);

    // Enable HW flow control on RTS, CTS or both
    void setRts(uint32_t _rts);
    void setCts(uint32_t _cts);
    void setRtsCts(uint32_t _rts, uint32_t _cts);
    void setRts(PinName _rts);
    void setCts(PinName _cts);
    void setRtsCts(PinName _rts, PinName _cts);
    void setHandler(void *handler);
  private:
    uint8_t _config;
    unsigned long _baud;
    void init(PinName _rx, PinName _tx, PinName _rts = NC, PinName _cts = NC);
};

#if defined(USART1)
  extern HardwareSerial Serial1;
#endif
#if defined(USART2)
  extern HardwareSerial Serial2;
#endif
#if defined(USART3)
  extern HardwareSerial Serial3;
#endif
#if defined(UART4) || defined(USART4)
  extern HardwareSerial Serial4;
#endif
#if defined(UART5) || defined(USART5)
  extern HardwareSerial Serial5;
#endif
#if defined(UART6) || defined(USART6)
  extern HardwareSerial Serial6;
#endif
#if defined(UART7) || defined(USART7)
  extern HardwareSerial Serial7;
#endif
#if defined(UART8) || defined(USART8)
  extern HardwareSerial Serial8;
#endif



#else  // #if 1







#endif  // #if 1











#endif
