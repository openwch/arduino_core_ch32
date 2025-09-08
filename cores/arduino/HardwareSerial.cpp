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
  Modified 8 September 2025 by Jobit Joseph (Universal CH32 RX Buffer Support)
*/

#include <stdio.h>
#include "Arduino.h"
#include "HardwareSerial.h"

#if defined(UART_MODULE_ENABLED) && !defined(UART_MODULE_ONLY)

// Enable RX buffering for all CH32 series MCUs
#if defined(CH32V003) || defined(CH32V203) || defined(CH32X035) || defined(CH32V103) || defined(CH32V307)
#define ENABLE_RX_BUFFER 1
#else
#define ENABLE_RX_BUFFER 0
#endif

#if ENABLE_RX_BUFFER
// Interrupt handlers for all CH32 series MCUs
#ifdef __cplusplus
extern "C" {
#endif

#if defined(USART1) && defined(HAVE_HWSERIAL1)
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) {
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    
    extern HardwareSerial Serial1;
    HardwareSerial *obj = &Serial1;
    
    unsigned char c = USART_ReceiveData(USART1);
    rx_buffer_index_t next_head = (obj->_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
    
    if (next_head != obj->_rx_buffer_tail) {
      obj->_rx_buffer[obj->_rx_buffer_head] = c;
      obj->_rx_buffer_head = next_head;
    }
  }
}
#endif

#if defined(USART2) && defined(HAVE_HWSERIAL2)
void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART2_IRQHandler(void) {
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    
    extern HardwareSerial Serial2;
    HardwareSerial *obj = &Serial2;
    
    unsigned char c = USART_ReceiveData(USART2);
    rx_buffer_index_t next_head = (obj->_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
    
    if (next_head != obj->_rx_buffer_tail) {
      obj->_rx_buffer[obj->_rx_buffer_head] = c;
      obj->_rx_buffer_head = next_head;
    }
  }
}
#endif

#if defined(USART3) && defined(HAVE_HWSERIAL3)
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void) {
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    
    extern HardwareSerial Serial3;
    HardwareSerial *obj = &Serial3;
    
    unsigned char c = USART_ReceiveData(USART3);
    rx_buffer_index_t next_head = (obj->_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
    
    if (next_head != obj->_rx_buffer_tail) {
      obj->_rx_buffer[obj->_rx_buffer_head] = c;
      obj->_rx_buffer_head = next_head;
    }
  }
}
#endif

#if defined(USART4) && defined(HAVE_HWSERIAL4)
void USART4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART4_IRQHandler(void) {
  if(USART_GetITStatus(USART4, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(USART4, USART_IT_RXNE);
    
    extern HardwareSerial Serial4;
    HardwareSerial *obj = &Serial4;
    
    unsigned char c = USART_ReceiveData(USART4);
    rx_buffer_index_t next_head = (obj->_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
    
    if (next_head != obj->_rx_buffer_tail) {
      obj->_rx_buffer[obj->_rx_buffer_head] = c;
      obj->_rx_buffer_head = next_head;
    }
  }
}
#endif

#if defined(UART4) && defined(HAVE_HWSERIAL4) && !defined(USART4)
void UART4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void UART4_IRQHandler(void) {
  if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    
    extern HardwareSerial Serial4;
    HardwareSerial *obj = &Serial4;
    
    unsigned char c = USART_ReceiveData(UART4);
    rx_buffer_index_t next_head = (obj->_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
    
    if (next_head != obj->_rx_buffer_tail) {
      obj->_rx_buffer[obj->_rx_buffer_head] = c;
      obj->_rx_buffer_head = next_head;
    }
  }
}
#endif

#if defined(UART5) && defined(HAVE_HWSERIAL5)
void UART5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void UART5_IRQHandler(void) {
  if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(UART5, USART_IT_RXNE);
    
    extern HardwareSerial Serial5;
    HardwareSerial *obj = &Serial5;
    
    unsigned char c = USART_ReceiveData(UART5);
    rx_buffer_index_t next_head = (obj->_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
    
    if (next_head != obj->_rx_buffer_tail) {
      obj->_rx_buffer[obj->_rx_buffer_head] = c;
      obj->_rx_buffer_head = next_head;
    }
  }
}
#endif

#if defined(USART6) && defined(HAVE_HWSERIAL6)
void USART6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART6_IRQHandler(void) {
  if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(USART6, USART_IT_RXNE);
    
    extern HardwareSerial Serial6;
    HardwareSerial *obj = &Serial6;
    
    unsigned char c = USART_ReceiveData(USART6);
    rx_buffer_index_t next_head = (obj->_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
    
    if (next_head != obj->_rx_buffer_tail) {
      obj->_rx_buffer[obj->_rx_buffer_head] = c;
      obj->_rx_buffer_head = next_head;
    }
  }
}
#endif

#if defined(UART7) && defined(HAVE_HWSERIAL7)
void UART7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void UART7_IRQHandler(void) {
  if(USART_GetITStatus(UART7, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(UART7, USART_IT_RXNE);
    
    extern HardwareSerial Serial7;
    HardwareSerial *obj = &Serial7;
    
    unsigned char c = USART_ReceiveData(UART7);
    rx_buffer_index_t next_head = (obj->_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
    
    if (next_head != obj->_rx_buffer_tail) {
      obj->_rx_buffer[obj->_rx_buffer_head] = c;
      obj->_rx_buffer_head = next_head;
    }
  }
}
#endif

#if defined(UART8) && defined(HAVE_HWSERIAL8)
void UART8_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void UART8_IRQHandler(void) {
  if(USART_GetITStatus(UART8, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(UART8, USART_IT_RXNE);
    
    extern HardwareSerial Serial8;
    HardwareSerial *obj = &Serial8;
    
    unsigned char c = USART_ReceiveData(UART8);
    rx_buffer_index_t next_head = (obj->_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
    
    if (next_head != obj->_rx_buffer_tail) {
      obj->_rx_buffer[obj->_rx_buffer_head] = c;
      obj->_rx_buffer_head = next_head;
    }
  }
}
#endif

#ifdef __cplusplus
}
#endif
#endif // ENABLE_RX_BUFFER

HardwareSerial::HardwareSerial(void *peripheral)
{
  setHandler(peripheral);

  setRx(PIN_SERIAL_RX);
  setTx(PIN_SERIAL_TX);
  
#if ENABLE_RX_BUFFER
  // Initialize buffer pointers for all CH32 series MCUs
  _rx_buffer_head = 0;
  _rx_buffer_tail = 0;
#endif
  
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

#if ENABLE_RX_BUFFER
  // Clear buffers for all CH32 series MCUs
  _rx_buffer_head = 0;
  _rx_buffer_tail = 0;
#endif

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

#if ENABLE_RX_BUFFER
  // Enable UART RX interrupts for all CH32 series MCUs
  if (_serial.uart == USART1) {
#if defined(USART1) && defined(HAVE_HWSERIAL1)
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);
#endif
  } 
#if defined(USART2) && defined(HAVE_HWSERIAL2)
  else if (_serial.uart == USART2) {
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(USART2_IRQn, 2);
    NVIC_EnableIRQ(USART2_IRQn);
  }
#endif
#if defined(USART3) && defined(HAVE_HWSERIAL3)
  else if (_serial.uart == USART3) {
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(USART3_IRQn, 2);
    NVIC_EnableIRQ(USART3_IRQn);
  }
#endif
#if defined(USART4) && defined(HAVE_HWSERIAL4)
  else if (_serial.uart == USART4) {
    USART_ITConfig(USART4, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(USART4_IRQn, 2);
    NVIC_EnableIRQ(USART4_IRQn);
  }
#endif
#if defined(UART4) && defined(HAVE_HWSERIAL4) && !defined(USART4)
  else if (_serial.uart == UART4) {
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(UART4_IRQn, 2);
    NVIC_EnableIRQ(UART4_IRQn);
  }
#endif
#if defined(UART5) && defined(HAVE_HWSERIAL5)
  else if (_serial.uart == UART5) {
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(UART5_IRQn, 2);
    NVIC_EnableIRQ(UART5_IRQn);
  }
#endif
#if defined(USART6) && defined(HAVE_HWSERIAL6)
  else if (_serial.uart == USART6) {
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(USART6_IRQn, 2);
    NVIC_EnableIRQ(USART6_IRQn);
  }
#endif
#if defined(UART7) && defined(HAVE_HWSERIAL7)
  else if (_serial.uart == UART7) {
    USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(UART7_IRQn, 2);
    NVIC_EnableIRQ(UART7_IRQn);
  }
#endif
#if defined(UART8) && defined(HAVE_HWSERIAL8)
  else if (_serial.uart == UART8) {
    USART_ITConfig(UART8, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(UART8_IRQn, 2);
    NVIC_EnableIRQ(UART8_IRQn);
  }
#endif
#endif
}

void HardwareSerial::end()
{
#if ENABLE_RX_BUFFER
  // Disable interrupts for all CH32 series MCUs
  if (_serial.uart == USART1) {
#if defined(USART1) && defined(HAVE_HWSERIAL1)
    NVIC_DisableIRQ(USART1_IRQn);
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
#endif
  } 
#if defined(USART2) && defined(HAVE_HWSERIAL2)
  else if (_serial.uart == USART2) {
    NVIC_DisableIRQ(USART2_IRQn);
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
  }
#endif
#if defined(USART3) && defined(HAVE_HWSERIAL3)
  else if (_serial.uart == USART3) {
    NVIC_DisableIRQ(USART3_IRQn);
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
  }
#endif
#if defined(USART4) && defined(HAVE_HWSERIAL4)
  else if (_serial.uart == USART4) {
    NVIC_DisableIRQ(USART4_IRQn);
    USART_ITConfig(USART4, USART_IT_RXNE, DISABLE);
  }
#endif
#if defined(UART4) && defined(HAVE_HWSERIAL4) && !defined(USART4)
  else if (_serial.uart == UART4) {
    NVIC_DisableIRQ(UART4_IRQn);
    USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
  }
#endif
#if defined(UART5) && defined(HAVE_HWSERIAL5)
  else if (_serial.uart == UART5) {
    NVIC_DisableIRQ(UART5_IRQn);
    USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
  }
#endif
#if defined(USART6) && defined(HAVE_HWSERIAL6)
  else if (_serial.uart == USART6) {
    NVIC_DisableIRQ(USART6_IRQn);
    USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);
  }
#endif
#if defined(UART7) && defined(HAVE_HWSERIAL7)
  else if (_serial.uart == UART7) {
    NVIC_DisableIRQ(UART7_IRQn);
    USART_ITConfig(UART7, USART_IT_RXNE, DISABLE);
  }
#endif
#if defined(UART8) && defined(HAVE_HWSERIAL8)
  else if (_serial.uart == UART8) {
    NVIC_DisableIRQ(UART8_IRQn);
    USART_ITConfig(UART8, USART_IT_RXNE, DISABLE);
  }
#endif
  
  // Clear buffers
  _rx_buffer_head = _rx_buffer_tail;
#endif

  uart_deinit(&_serial);
}

int HardwareSerial::available(void)
{
#if ENABLE_RX_BUFFER
  // For all CH32 series MCUs: return proper buffer count
  return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail)) % SERIAL_RX_BUFFER_SIZE;
#else
  // For other MCUs: keep original behavior
  return -1;
#endif
}

int HardwareSerial::peek(void)
{
#if ENABLE_RX_BUFFER
  // For all CH32 series MCUs: return next byte without removing it
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  } else {
    return _rx_buffer[_rx_buffer_tail];
  }
#else
  // For other MCUs: keep original behavior
  return -1;
#endif
}

int HardwareSerial::read(void)
{
#if ENABLE_RX_BUFFER
  // For all CH32 series MCUs: read from buffer
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  } else {
    unsigned char c = _rx_buffer[_rx_buffer_tail];
    _rx_buffer_tail = (rx_buffer_index_t)(_rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE;
    return c;
  }
#else
  // For other MCUs: keep original behavior
  unsigned char c;
  if(uart_getc(&_serial, &c) == 0){
    return c;
  }else{
    return -1;
  }
#endif
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
