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

#if ENABLE_RX_BUFFER
// RX ring buffers: rx_owner[] holds the HardwareSerial object that opened each
// UART (set in begin()); the UART's RX interrupt stores the byte in its buffer.
// UART4/5 exist on the V10x but have no interrupt vector there.
static HardwareSerial *rx_owner[8];

static int rx_index(USART_TypeDef *uart, IRQn_Type *irq)
{
#if defined(USART1_BASE)
  if (uart == USART1) { *irq = USART1_IRQn; return 0; }
#endif
#if defined(USART2_BASE)
  if (uart == USART2) { *irq = USART2_IRQn; return 1; }
#endif
#if defined(USART3_BASE)
  if (uart == USART3) { *irq = USART3_IRQn; return 2; }
#endif
#if defined(USART4_BASE)
  if (uart == USART4) { *irq = USART4_IRQn; return 3; }
#endif
#if defined(UART4_BASE) && !defined(CH32V10x)
  if (uart == UART4) { *irq = UART4_IRQn; return 3; }
#endif
#if defined(UART5_BASE) && !defined(CH32V10x)
  if (uart == UART5) { *irq = UART5_IRQn; return 4; }
#endif
#if defined(UART6_BASE)
  if (uart == UART6) { *irq = UART6_IRQn; return 5; }
#endif
#if defined(UART7_BASE)
  if (uart == UART7) { *irq = UART7_IRQn; return 6; }
#endif
#if defined(UART8_BASE)
  if (uart == UART8) { *irq = UART8_IRQn; return 7; }
#endif
  return -1;
}

// With an FPU (V30x) a handler that calls a function must save the FPU
// registers itself (the hardware stacking doesn't cover them), so the body is
// inlined there; without one, all handlers share one copy (smaller)
#if defined(__riscv_flen)
  #define RX_IRQ_INLINE inline __attribute__((always_inline))
#else
  #define RX_IRQ_INLINE __attribute__((noinline))
#endif
static RX_IRQ_INLINE void rx_irq(USART_TypeDef *uart, int idx)
{
  // RXNE, or an overrun: reading STATR then DATAR clears both
  if ((uart->STATR & (USART_FLAG_RXNE | USART_FLAG_ORE)) == 0) {
    return;
  }
  unsigned char c = (unsigned char)uart->DATAR;
  HardwareSerial *obj = rx_owner[idx];
  if (obj == NULL) {
    return;
  }
  rx_buffer_index_t next_head = (rx_buffer_index_t)((obj->_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE);
  if (next_head != obj->_rx_buffer_tail) {   // buffer full: the byte is dropped
    obj->_rx_buffer[obj->_rx_buffer_head] = c;
    obj->_rx_buffer_head = next_head;
  }
}

#ifdef __cplusplus
extern "C" {
#endif

// Weak, like the core's other handlers: a library can still define its own
#define CH32_UART_RX_IRQ(name, uart, idx) \
  void name(void) __attribute__((interrupt("WCH-Interrupt-fast"))); \
  void __attribute__((weak)) name(void) { rx_irq(uart, idx); }

#if defined(USART1_BASE)
CH32_UART_RX_IRQ(USART1_IRQHandler, USART1, 0)
#endif
#if defined(USART2_BASE)
CH32_UART_RX_IRQ(USART2_IRQHandler, USART2, 1)
#endif
#if defined(USART3_BASE)
CH32_UART_RX_IRQ(USART3_IRQHandler, USART3, 2)
#endif
#if defined(USART4_BASE)
CH32_UART_RX_IRQ(USART4_IRQHandler, USART4, 3)
#endif
#if defined(UART4_BASE) && !defined(CH32V10x)
CH32_UART_RX_IRQ(UART4_IRQHandler, UART4, 3)
#endif
#if defined(UART5_BASE) && !defined(CH32V10x)
CH32_UART_RX_IRQ(UART5_IRQHandler, UART5, 4)
#endif
#if defined(UART6_BASE)
CH32_UART_RX_IRQ(UART6_IRQHandler, UART6, 5)
#endif
#if defined(UART7_BASE)
CH32_UART_RX_IRQ(UART7_IRQHandler, UART7, 6)
#endif
#if defined(UART8_BASE)
CH32_UART_RX_IRQ(UART8_IRQHandler, UART8, 7)
#endif

#ifdef __cplusplus
}
#endif
#endif // ENABLE_RX_BUFFER

HardwareSerial::HardwareSerial(void *peripheral)
{
  setHandler(peripheral);

  // The variant's Serial pins belong to one UART only; any other UART
  // (Serial2, HardwareSerial S(USART2), ...) takes its pins from the pin map
  if (pinmap_peripheral(digitalPinToPinName(PIN_SERIAL_TX), PinMap_UART_TX) == peripheral) {
    setRx(PIN_SERIAL_RX);
    setTx(PIN_SERIAL_TX);
  } else {
    setRx(pinmap_pin(peripheral, PinMap_UART_RX));
    setTx(pinmap_pin(peripheral, PinMap_UART_TX));
  }

#if ENABLE_RX_BUFFER
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

  // The USART word length includes the parity bit: 8 data bits + parity
  // need a 9-bit word
  if (parity != USART_Parity_No) {
    databits++;
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
#ifdef USART_WordLength_5b
    case 5:
      databits = USART_WordLength_5b;
      break;
#endif
    case 8:
      databits = USART_WordLength_8b;
      break;
#ifdef USART_WordLength_9b
    case 9:
      databits = USART_WordLength_9b;
      break;
#endif
    default:
    case 0:
      Error_Handler();
      break;
  }
  
  uart_init(&_serial, (uint32_t)baud, databits, parity, stopbits);

#if ENABLE_RX_BUFFER
  // Received bytes go into this object's ring buffer from the UART's RX
  // interrupt, for any UART: Serial, Serial2, or a HardwareSerial(USARTx)
  IRQn_Type irq;
  int idx = rx_index(_serial.uart, &irq);
  if (idx >= 0 && _serial.pin_rx != NC) {
    rx_owner[idx] = this;
    USART_ITConfig(_serial.uart, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(irq);
  }
#endif
}

void HardwareSerial::end()
{
#if ENABLE_RX_BUFFER
  IRQn_Type irq;
  int idx = rx_index(_serial.uart, &irq);
  if (idx >= 0 && rx_owner[idx] == this) {
    NVIC_DisableIRQ(irq);
    USART_ITConfig(_serial.uart, USART_IT_RXNE, DISABLE);
    rx_owner[idx] = NULL;
  }
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

    for (size_t i = 0; i < size; i++) {
      write(buffer[i]);
    }

    return size;
}

size_t HardwareSerial::write(uint8_t c)
{
  return uart_putc(&_serial, c);
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
    HardwareSerial Serial6(UART6);
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
