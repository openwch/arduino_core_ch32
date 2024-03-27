/**
  ******************************************************************************
  * @file    twi.h
  * @author  WI6LABS
  * @version V1.0.0
  * @date    01-August-2016
  * @brief   Header for twi module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  Modified 6 june 2023 by Temperslee to support wch's risc-v chips
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TWI_H__
#define __TWI_H__

/* Includes ------------------------------------------------------------------*/
#include "ch32_def.h"
#include "PeripheralPins.h"


#ifdef __cplusplus
extern "C" {
#endif

// enabling callback attachment, set slave interrupts, process data transfers
#ifndef OPT_I2C_SLAVE			// allow setting define using Arduino IDE menu choice
#define OPT_I2C_SLAVE 1			// set to 0 to save about 1K FLASH space (812 bytes used with -Os)
#endif


/* Exported types ------------------------------------------------------------*/
/* offsetof is a gcc built-in function, this is the manual implementation */
#define OFFSETOF(type, member) ((uint32_t) (&(((type *)(0))->member)))

/* Interrupt priority */
#ifndef I2C_IRQ_PRIO
#define I2C_IRQ_PRIO        (0xF0)
#endif


/* I2C Tx/Rx buffer size */
#if !defined(I2C_TXRX_BUFFER_SIZE)
#define I2C_TXRX_BUFFER_SIZE    32
#elif (I2C_TXRX_BUFFER_SIZE >= 256)
#error I2C buffer size cannot exceed 255
#endif


//For compatibility
typedef struct _I2C_HandleTypeDef
{
  I2C_TypeDef                *Instance;      /*!< I2C registers base address               */
  I2C_InitTypeDef            Init;           /*!< I2C communication parameters             */
}I2C_HandleTypeDef;

typedef struct i2c_s i2c_t;

struct i2c_s {
  /*  The 1st 2 members I2CName i2c
     *  and I2C_HandleTypeDef handle should
     *  be kept as the first members of this struct
     *  to have get_i2c_obj() function work as expected
     */
  I2C_TypeDef  *i2c;
  I2C_HandleTypeDef handle;
  void *__this;
  PinName sda;
  PinName scl;
  IRQn_Type irq;
  IRQn_Type irqER;

  volatile int slaveRxNbData; // Number of accumulated bytes received in Slave mode
  void (*i2c_onSlaveReceive)(i2c_t *);
  void (*i2c_onSlaveTransmit)(i2c_t *);
  volatile uint8_t i2cTxRxBuffer[I2C_TXRX_BUFFER_SIZE];
  volatile uint8_t i2cTxRxBufferSize;
#if OPT_I2C_SLAVE
  volatile uint8_t slaveMode;
#endif // #if OPT_I2C_SLAVE  
  uint8_t isMaster;
  uint8_t generalCall;
  uint8_t NoStretchMode;
};

///@brief I2C state
typedef enum {
  I2C_OK = 0,
  I2C_DATA_TOO_LONG = 1,
  I2C_NACK_ADDR = 2,
  I2C_NACK_DATA = 3,
  I2C_ERROR = 4,
  I2C_TIMEOUT = 5,
  I2C_BUSY = 6
} i2c_status_e;


/* Exported functions ------------------------------------------------------- */
void i2c_init(i2c_t *obj);
void i2c_custom_init(i2c_t *obj, uint32_t timing, uint32_t addressingMode,uint32_t ownAddress);
void i2c_deinit(i2c_t *obj);
void i2c_setTiming(i2c_t *obj, uint32_t frequency);

i2c_status_e i2c_master_write(i2c_t *obj, uint8_t dev_address, uint8_t *data, uint16_t size,  uint8_t sendstop);
i2c_status_e i2c_slave_write_IT(i2c_t *obj, uint8_t *data, uint16_t size);
i2c_status_e i2c_master_read(i2c_t *obj, uint8_t dev_address, uint8_t *data, uint16_t size);

i2c_status_e i2c_slave_write(i2c_t *obj, uint8_t *data, uint16_t size);
i2c_status_e i2c_slave_read(i2c_t *obj, uint8_t *data, uint16_t size);

// i2c_status_e i2c_IsDeviceReady(i2c_t *obj, uint8_t devAddr, uint32_t trials);
void i2c_attachSlaveRxEvent(i2c_t *obj, void (*function)(i2c_t *));
void i2c_attachSlaveTxEvent(i2c_t *obj, void (*function)(i2c_t *));

// MMOLE 240320: was this method private?
i2c_t *get_i2c_obj(I2C_HandleTypeDef *);

// MMOLE 240323: call this to process slave data transfers
//void i2c_slave_process(i2c_t *obj);


#ifdef __cplusplus
}
#endif

#endif /* __TWI_H__ */

