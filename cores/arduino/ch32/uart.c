/**
 *******************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * This software component is licensed by WCH under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#include "core_debug.h"
#include "uart.h"
#include "Arduino.h"
#include "PinAF_ch32yyxx.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(UART_MODULE_ENABLED) && !defined(UART_MODULE_ONLY)

/* If DEBUG_UART is not defined assume this is the one linked to PIN_SERIAL_TX */
#if !defined(DEBUG_UART)
#if defined(PIN_SERIAL_TX)
#define DEBUG_UART          pinmap_peripheral(digitalPinToPinName(PIN_SERIAL_TX), PinMap_UART_TX)
#define DEBUG_PINNAME_TX    digitalPinToPinName(PIN_SERIAL_TX)
#else
/* No debug UART defined */
#define DEBUG_UART          NP
#define DEBUG_PINNAME_TX    NC
#endif
#endif
#if !defined(DEBUG_UART_BAUDRATE)
#define DEBUG_UART_BAUDRATE 115200
#endif
#if !defined( UART_WORDLENGTH_8B )
#define UART_WORDLENGTH_8B                      USART_WordLength_8b
#endif
#if !defined( UART_PARITY_NONE )
#define UART_PARITY_NONE                        USART_Parity_No
#endif
#if !defined( UART_STOPBITS_1 )
#define UART_STOPBITS_1                         USART_StopBits_1
#endif



/* @brief uart caracteristics */
typedef enum {
#if defined(USART1_BASE)
  UART1_INDEX,
#endif
#if defined(USART2_BASE)
  UART2_INDEX,
#endif
#if defined(USART3_BASE)
  UART3_INDEX,
#endif
#if defined(UART4_BASE) 
  UART4_INDEX,
#endif
#if defined(UART5_BASE) 
  UART5_INDEX,
#endif
#if defined(UART6_BASE) 
  UART6_INDEX,
#endif
#if defined(UART7_BASE) 
  UART7_INDEX,
#endif
#if defined(UART8_BASE) 
  UART8_INDEX,
#endif
  UART_NUM
} uart_index_t;

static UART_HandleTypeDef *uart_handlers[UART_NUM] = {NULL};


static serial_t serial_debug =
{
  .uart = NP,
  .pin_tx = NC,
  .pin_rx = NC,
  .pin_rts = NC,
  .pin_cts = NC,
  .index = UART_NUM,
};

extern uint64_t GetTick(void);

/* Aim of the function is to get serial_s pointer using huart pointer */
/* Highly inspired from magical linux kernel's "container_of" */
serial_t *get_serial_obj(UART_HandleTypeDef *huart)
{
  struct serial_s *obj_s;
  serial_t *obj;

  obj_s = (struct serial_s *)((char *)huart - offsetof(struct serial_s, handle));
  obj = (serial_t *)((char *)obj_s - offsetof(serial_t, uart));

  return (obj);
}


/**
  * @brief  Function called to initialize the uart interface
  * @param  obj : pointer to serial_t structure
  * @retval None
  */
void uart_init(serial_t *obj, uint32_t baudrate, uint32_t databits, uint32_t parity, uint32_t stopbits)
{
  if (obj == NULL) {
    return;
  }
  UART_HandleTypeDef *huart = &(obj->handle);  
  
  /* Determine the U(S)ART peripheral to use (USART1, USART2, ...) */
  USART_TypeDef *uart_tx = pinmap_peripheral(obj->pin_tx, PinMap_UART_TX);
  USART_TypeDef *uart_rx = pinmap_peripheral(obj->pin_rx, PinMap_UART_RX);
  USART_TypeDef *uart_rts = pinmap_peripheral(obj->pin_rts, PinMap_UART_RTS);
  USART_TypeDef *uart_cts = pinmap_peripheral(obj->pin_cts, PinMap_UART_CTS);

  /* Pin Tx must not be NP */
  if (uart_tx == NP) {
    if (obj != &serial_debug) {
      core_debug("ERROR: [U(S)ART] Tx pin has no peripheral!\n");
    }
    return;
  }
  /* Pin Rx must not be NP if not half-duplex */
  if ((obj->pin_rx != NC) && (uart_rx == NP)) {
    if (obj != &serial_debug) {
      core_debug("ERROR: [U(S)ART] Rx pin has no peripheral!\n");
    }
    return;
  }
  /* Pin RTS must not be NP if flow control is enabled */
  if ((obj->pin_rts != NC) && (uart_rts == NP)) {
    if (obj != &serial_debug) {
      core_debug("ERROR: [U(S)ART] RTS pin has no peripheral!\n");
    }
    return;
  }
  /* Pin CTS must not be NP if flow control is enabled */
  if ((obj->pin_cts != NC) && (uart_cts == NP)) {
    if (obj != &serial_debug) {
      core_debug("ERROR: [U(S)ART] CTS pin has no peripheral!\n");
    }
    return;
  }

  /*
   * Get the peripheral name (USART1, USART2, ...) from the pin
   * and assign it to the object
   */
  obj->uart = pinmap_merge_peripheral(uart_tx, uart_rx);
  /* We also merge RTS/CTS and assert all pins belong to the same instance */
  obj->uart = pinmap_merge_peripheral(obj->uart, uart_rts);
  obj->uart = pinmap_merge_peripheral(obj->uart, uart_cts);

  if (obj->uart == NP) {
    if (obj != &serial_debug) {
      core_debug("ERROR: [U(S)ART] Rx/Tx/RTS/CTS pins peripherals mismatch!\n");
    }
    return;
  }

  /* Enable USART clock */
#if defined(USART1_BASE)
  else if (obj->uart == USART1) 
  {
    #if defined(CH32L10x) || defined(CH32VM00X)
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_USART1, ENABLE);
    #else
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    #endif
    obj->index = UART1_INDEX;
    obj->irq = USART1_IRQn;
  }
#endif
#if defined(USART2_BASE)
  else if (obj->uart == USART2) 
  {
    #if defined(CH32L10x) 
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_USART2, ENABLE);
    #elif defined(CH32VM00X)
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_USART2, ENABLE);
    #else
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    #endif
    obj->index = UART2_INDEX;
    obj->irq = USART2_IRQn;
  }
#endif
#if defined(USART3_BASE) 
  else if (obj->uart == USART3) 
  {
    #if defined(CH32L10x)
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_USART3, ENABLE);   
    #else
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    #endif
    obj->index = UART3_INDEX;
    obj->irq = USART3_IRQn;
  }
#endif
#if defined(UART4_BASE) && !defined(CH32V10x) && !defined(CH32L10x)
  else if (obj->uart == UART4) 
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    obj->index = UART4_INDEX;
    obj->irq = UART4_IRQn;
  }
#endif
#if defined(UART5_BASE) && !defined(CH32V10x) && !defined(CH32L10x)
  else if (obj->uart == UART5) 
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    obj->index = UART5_INDEX;
    obj->irq = UART5_IRQn;
  }
#endif
#if defined(UART6_BASE)
  else if (obj->uart == UART6) 
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART6, ENABLE);   
    obj->index = UART6_INDEX;
    obj->irq = UART6_IRQn;
  }
#endif
#if defined(UART7_BASE)
  else if (obj->uart == UART7) 
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE); 
    obj->index = UART7_INDEX;
    obj->irq = UART7_IRQn;
  }
#endif
#if defined(UART8_BASE)
  else if (obj->uart == UART8) 
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8, ENABLE);
    obj->index = UART8_INDEX;
    obj->irq = UART8_IRQn;
  }
#endif

  /* Configure UART GPIO pins */
  pinmap_pinout(obj->pin_tx, PinMap_UART_TX);
  if (uart_rx != NP) {
    pinmap_pinout(obj->pin_rx, PinMap_UART_RX);
  }

  /* Configure flow control */
  uint32_t flow_control = USART_HardwareFlowControl_None;
  if (uart_rts != NP) {
    flow_control |= USART_HardwareFlowControl_RTS;
    pinmap_pinout(obj->pin_rts, PinMap_UART_RTS);
  }
  if (uart_cts != NP) {
    flow_control |= USART_HardwareFlowControl_CTS;
    pinmap_pinout(obj->pin_cts, PinMap_UART_CTS);
  }

  /* Configure uart */
  uart_handlers[obj->index] = huart;
  huart->Instance = (USART_TypeDef *)(obj->uart);
  huart->init.USART_BaudRate   = baudrate; 
  huart->init.USART_WordLength = databits;
  huart->init.USART_StopBits   = stopbits;
  huart->init.USART_Parity     = parity;  
  huart->init.USART_Mode       = (USART_Mode_Rx|USART_Mode_Tx);  
  huart->init.USART_HardwareFlowControl = flow_control;



  if (uart_rx == NP) 
  {
      USART_HalfDuplexCmd(huart->Instance,ENABLE);
  }

  USART_Init(huart->Instance, &(huart->init));
  USART_Cmd(huart->Instance, ENABLE);
}

/**
  * @brief  Function called to deinitialize the uart interface
  * @param  obj : pointer to serial_t structure
  * @retval None
  */
void uart_deinit(serial_t *obj)
{
  /* Reset UART and disable clock */
  switch (obj->index) 
  {
#if defined(USART1_BASE)
    case UART1_INDEX:
      #if defined(CH32L10x) || defined(CH32VM00X)
      RCC_PB2PeriphResetCmd(RCC_PB2Periph_USART1, ENABLE);
      RCC_PB2PeriphResetCmd(RCC_PB2Periph_USART1, DISABLE);
      #else
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
      #endif
      break;
#endif
#if defined(USART2_BASE)
    case UART2_INDEX:
      #if defined(CH32L10x) 
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_USART2, ENABLE);
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_USART2, DISABLE);
      #elif defined(CH32VM00X)
      RCC_PB2PeriphResetCmd(RCC_PB2Periph_USART2, ENABLE);
      RCC_PB2PeriphResetCmd(RCC_PB2Periph_USART2, DISABLE);
      #else
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);
      #endif
      break;
#endif
#if defined(USART3_BASE)
    case UART3_INDEX:
      #if defined(CH32L10x)
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_USART3, ENABLE);
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_USART3, DISABLE);    
      #else
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);    
      #endif
      break;
#endif
#if defined(UART4_BASE)
    case UART4_INDEX:
      #if defined(CH32L10x)
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_UART4, ENABLE);
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_UART4, DISABLE);
      #else
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, DISABLE);
      #endif
      break;
#endif

#if defined(UART5_BASE)
    case UART5_INDEX:
      #if defined(CH32L10x)
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_UART5, ENABLE);
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_UART5, DISABLE);
      #else
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, DISABLE);
      #endif    
      break;
#endif

#if defined(UART6_BASE)
    case UART6_INDEX:
      #if defined(CH32L10x)
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_UART6, ENABLE);
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_UART6, DISABLE);
      #else
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART6, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART6, DISABLE);
      #endif
      break;
#endif

#if defined(UART7_BASE)
    case UART7_INDEX:
      #if defined(CH32L10x)
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_UART7, ENABLE);
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_UART7, DISABLE);
      #else
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART7, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART7, DISABLE);
      #endif
      break;
#endif

#if defined(UART8_BASE)
    case UART8_INDEX:
      #if defined(CH32L10x)
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_UART8, ENABLE);
      RCC_PB1PeriphResetCmd(RCC_PB1Periph_UART8, DISABLE);
      #else
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART8, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART8, DISABLE);
      #endif
      break;
#endif
  }

  /* Release uart debug to ensure init */
  if (serial_debug.index == obj->index) 
  {
    serial_debug.index = UART_NUM;
  }
}


/**
  * @brief  Function called to initialize the debug uart interface
  * @note   Call only if debug U(S)ART peripheral is not already initialized
  *         by a Serial instance
  *         Default config: 8N1
  * @retval None
  */
void uart_debug_init(void)
{
  if (DEBUG_UART != NP) 
  {
#if defined(DEBUG_PINNAME_TX)
    serial_debug.pin_tx = DEBUG_PINNAME_TX;
#else
    serial_debug.pin_tx = pinmap_pin(DEBUG_UART, PinMap_UART_TX);
#endif

    /* serial_debug.pin_rx set by default to NC to configure in half duplex mode */
    uart_init(&serial_debug, DEBUG_UART_BAUDRATE, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_STOPBITS_1);
  }
}

/**
  * @brief  write the data on the uart: used by printf for debug only (syscalls)
  * @param  data : bytes to write
  * @param  size : number of data to write
  * @retval The number of bytes written
  */
size_t uart_debug_write(uint8_t *data, uint32_t size)
{
  int i;
  uint32_t tickstart = GetTick();
  serial_t *obj = NULL;

  if (serial_debug.index >= UART_NUM) 
  {
    if (DEBUG_UART == NP) 
    {
      return 0;
    }

    /* Search if DEBUG_UART already initialized */
    for (serial_debug.index = 0; serial_debug.index < UART_NUM; serial_debug.index++) 
    {
      if (uart_handlers[serial_debug.index] != NULL) {
        if (DEBUG_UART == uart_handlers[serial_debug.index]->Instance) {
          break;
        }
      }
    }

    if (serial_debug.index >= UART_NUM) 
    {
      /* DEBUG_UART not initialized */
      uart_debug_init();
      if (serial_debug.index >= UART_NUM) {
        return 0;
      }
    }
  }

  obj = get_serial_obj(uart_handlers[serial_debug.index]);

  if (!obj) 
  {
    return 0;
  }

  while (serial_tx_active(obj)) 
  {
    if ((GetTick() - tickstart) >= TX_TIMEOUT) 
    {
      return 0;
    }
  }

  for ( i = 0; i < size; i++)
  {
    while (serial_tx_active(obj)) ;
    USART_SendData(uart_handlers[serial_debug.index]->Instance,*data++);
  }
  return size;  //it shouled be 0
}

/**
 * Attempts to determine if the serial peripheral is already in use for RX
 *
 * @param obj The serial object
 * @return Non-zero if the RX transaction is ongoing, 0 otherwise
 */
uint8_t serial_rx_active(serial_t *obj)
{
  return ( USART_GetFlagStatus(uart_handlers[obj->index]->Instance, USART_FLAG_RXNE) == RESET );   
}

/**
 * Attempts to determine if the serial peripheral is already in use for TX
 *
 * @param obj The serial object
 * @return Non-zero if the TX transaction is ongoing, 0 otherwise
 */
uint8_t serial_tx_active(serial_t *obj)
{
  return ( USART_GetFlagStatus(uart_handlers[obj->index]->Instance, USART_FLAG_TC) == RESET ); 
}

/**
  * @brief  Read receive byte from uart
  * @param  obj : pointer to serial_t structure
  * @retval last character received
  */
int uart_getc(serial_t *obj, unsigned char *c)
{
  if (obj == NULL) {
    return -1;
  }

  if (serial_rx_active(obj)) 
  {
    return -1; 
  }

  *c = (unsigned char)USART_ReceiveData(uart_handlers[obj->index]->Instance);

  return 0;
}








#endif /* UART_MODULE_ENABLED  && !UART_MODULE_ONLY */
#ifdef __cplusplus
}
#endif



