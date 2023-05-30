/*
  HardwareSerial.cpp - Hardware serial library for Wiring
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

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 3  December 2013 by Matthijs Kooijman
  Modified 1 may 2023 by TempersLee
*/

#include <stdio.h>
#include "Arduino.h"
#include "HardwareSerial.h"

#if defined(UART_MODULE_ENABLED) && !defined(UART_MODULE_ONLY)


HardwareSerial::HardwareSerial(void *peripheral)
{
  setHandler(peripheral);

  setRx(PIN_SERIAL_RX);
  
  setTx(PIN_SERIAL_TX);
  
  init(_serial.pin_rx, _serial.pin_tx);
}




void HardwareSerial::init(PinName _rx, PinName _tx, PinName _rts, PinName _cts)
{
  if (_rx == _tx) {
    _serial.pin_rx = NC;
  } else {
    _serial.pin_rx = _rx;
  }
  _serial.pin_tx = _tx;
  _serial.pin_rts = _rts;
  _serial.pin_cts = _cts;
}



// Public Methods //////////////////////////////////////////////////////////////
void HardwareSerial::begin(unsigned long baud, byte config)
{
  uint32_t databits = 0;
  uint32_t stopbits = 0;
  uint32_t parity = 0;

  _baud = baud;
  _config = config;

  // Manage databits
  switch (config & 0x03) {
    case 0x00:
      databits = 6;
      break;
    case 0x01:
      databits = 7;
      break;
    case 0x02:
      databits = 8;
      break;
    case 0x03:
      databits = 9;
      break;
    default:
      databits = 8;
      break;
  }

  if ((config & 0x30) == 0x30) {
    parity = USART_Parity_Odd;
  } else if ((config & 0x20) == 0x20) {
    parity = USART_Parity_Even;
  } else {
    parity = USART_Parity_No;
  }


  switch ( (config & 0x0C) >> 2 ) {
    case 0x00:
      stopbits = USART_StopBits_1;
      break;
    case 0x01:
      stopbits = USART_StopBits_0_5;
      break;
    case 0x02:
      stopbits = USART_StopBits_2;
      break;
    case 0x03:
      stopbits = USART_StopBits_1_5;
      break;
    default:
      stopbits = USART_StopBits_1;
      break;
  }

  switch (databits) 
  {
#ifdef USART_WordLength_6b
    case 6: 
      databits = USART_WordLength_6b;
      break;
#endif 
#ifdef USART_WordLength_7b
    case 7: 
      databits = USART_WordLength_7b;
      break;
#endif
    case 8:
      databits = USART_WordLength_8b;
      break;
    case 9:
      databits = USART_WordLength_9b;
      break;
    default:
    case 0:
      Error_Handler();
      break;
  }
  uart_init(&_serial, (uint32_t)baud, databits, parity, stopbits);
}

void HardwareSerial::end()
{
  uart_deinit(&_serial);
}

int HardwareSerial::available(void)
{
  return -1;
}

int HardwareSerial::peek(void)
{
   return -1;
}

int HardwareSerial::read(void)
{

  unsigned char c;
  if(uart_getc(&_serial, &c) == 0){
    return c;
  }else{
    return -1;
  }
}


size_t HardwareSerial::write(const uint8_t *buffer, size_t size)
{

    return  uart_debug_write((uint8_t *)buffer, size);
}


size_t HardwareSerial::write(uint8_t c)
{
  uint8_t buff = c;
  return write(&buff, 1);
}

void HardwareSerial::setRx(uint32_t _rx)
{
  _serial.pin_rx = digitalPinToPinName(_rx);
}

void HardwareSerial::setTx(uint32_t _tx)
{
  _serial.pin_tx = digitalPinToPinName(_tx);
}

void HardwareSerial::setRx(PinName _rx)
{
  _serial.pin_rx = _rx;
}

void HardwareSerial::setTx(PinName _tx)
{
  _serial.pin_tx = _tx;
}

void HardwareSerial::setRts(uint32_t _rts)
{
  _serial.pin_rts = digitalPinToPinName(_rts);
}

void HardwareSerial::setCts(uint32_t _cts)
{
  _serial.pin_cts = digitalPinToPinName(_cts);
}

void HardwareSerial::setRtsCts(uint32_t _rts, uint32_t _cts)
{
  _serial.pin_rts = digitalPinToPinName(_rts);
  _serial.pin_cts = digitalPinToPinName(_cts);
}

void HardwareSerial::setRts(PinName _rts)
{
  _serial.pin_rts = _rts;
}

void HardwareSerial::setCts(PinName _cts)
{
  _serial.pin_cts = _cts;
}

void HardwareSerial::setRtsCts(PinName _rts, PinName _cts)
{
  _serial.pin_rts = _rts;
  _serial.pin_cts = _cts;
}

void HardwareSerial::setHandler(void *handler)
{
   _serial.uart  = (USART_TypeDef *) handler;
}




#if defined(HAVE_HWSERIAL1) || defined(HAVE_HWSERIAL2) || defined(HAVE_HWSERIAL3) ||\
  defined(HAVE_HWSERIAL4) || defined(HAVE_HWSERIAL5) || defined(HAVE_HWSERIAL6) ||\
  defined(HAVE_HWSERIAL7) || defined(HAVE_HWSERIAL8) 
  // SerialEvent functions are weak, so when the user doesn't define them,
  // the linker just sets their address to 0 (which is checked below).
  #if defined(HAVE_HWSERIAL1)
    HardwareSerial Serial1(USART1);
  #endif

  #if defined(HAVE_HWSERIAL2)
    HardwareSerial Serial2(USART2);
  #endif

  #if defined(HAVE_HWSERIAL3)
    HardwareSerial Serial3(USART3);
  #endif

  #if defined(HAVE_HWSERIAL4)
    #if defined(USART4)
      HardwareSerial Serial4(USART4);
    #else
      HardwareSerial Serial4(UART4);
    #endif
  #endif

  #if defined(HAVE_HWSERIAL5)
    #if defined(UART5)
      HardwareSerial Serial5(UART5);
    #endif
  #endif

  #if defined(HAVE_HWSERIAL6)
    HardwareSerial Serial6(USART6);
  #endif

  #if defined(HAVE_HWSERIAL7)
    #if defined(UART7)
      HardwareSerial Serial7(UART7);
    #endif
  #endif

  #if defined(HAVE_HWSERIAL8)
    #if defined(UART8)
      HardwareSerial Serial8(UART8);
    #endif
  #endif
#endif // HAVE_HWSERIALx



#endif // UART_MODULE_ENABLED && !UART_MODULE_ONLY
