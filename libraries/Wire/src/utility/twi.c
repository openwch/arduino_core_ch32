/**
  ******************************************************************************
  * @file    twi.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    01-August-2016
  * @brief   provide the TWI interface
  *
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
#include "core_debug.h"
#include "utility/twi.h"
#include "PinAF_ch32yyxx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Private Defines */

/// @brief I2C timeout in tick unit
#ifndef I2C_TIMEOUT_TICK
#define I2C_TIMEOUT_TICK        3      // MMOLE first used 10ms, now even much shorter timeout to facilitate I2C scanning
//#define I2C_TIMEOUT_TICK        100
#endif

#define SLAVE_MODE_TRANSMIT     0
#define SLAVE_MODE_RECEIVE      1
#define SLAVE_MODE_LISTEN       2


/*  Family specific description for I2C */
typedef enum {
#if defined(I2C1_BASE)
  I2C1_INDEX,
#endif
#if defined(I2C2_BASE)
  I2C2_INDEX,
#endif
  I2C_NUM
} i2c_index_t;

/* Private Variables */
static I2C_HandleTypeDef *i2c_handles[I2C_NUM];


/**
* @brief Compute I2C timing according current I2C clock source and
required I2C clock.
* @param  obj : pointer to i2c_t structure
* @param frequency
 Required I2C clock in Hz.
* @retval I2C timing or 0 in case of error.
*/
static uint32_t i2c_getTiming(i2c_t *obj, uint32_t frequency)
{
  uint32_t ret = 0;
  uint32_t i2c_speed = 0;
  if (frequency <= 100000) {
    i2c_speed = 100000;
  } else if (frequency <= 400000) {
    i2c_speed = 400000;
  } else if (frequency <= 1000000) {
    i2c_speed = 1000000;
  }
  ret = i2c_speed;
  return ret;
}

/**
  * @brief  Default init and setup GPIO and I2C peripheral
  * @param  obj : pointer to i2c_t structure
  * @retval none
  */
void i2c_init(i2c_t *obj)
{
  i2c_custom_init(obj, 100000, I2C_AcknowledgedAddress_7bit, 0x33);
}

/**
  * @brief  Initialize and setup GPIO and I2C peripheral
  * @param  obj : pointer to i2c_t structure
  * @param  timing : one of the i2c_timing_e
  * @param  addressingMode : I2C_AcknowledgedAddress_7bit or I2C_AcknowledgedAddress_10bit
  * @param  ownAddress : device address
  * @retval none
  */
void i2c_custom_init(i2c_t *obj, uint32_t timing, uint32_t addressingMode, uint32_t ownAddress)
{
  if (obj != NULL) 
  {
    I2C_HandleTypeDef *handle = &(obj->handle);
    // Determine the I2C to use
    I2C_TypeDef *i2c_sda = pinmap_peripheral(obj->sda, PinMap_I2C_SDA);
    I2C_TypeDef *i2c_scl = pinmap_peripheral(obj->scl, PinMap_I2C_SCL);

    //Pins SDA/SCL must not be NP
    if (i2c_sda == NP || i2c_scl == NP) 
    {
      core_debug("ERROR: at least one I2C pin has no peripheral\n");
    } 
    else 
    {

      obj->i2c = pinmap_merge_peripheral(i2c_sda, i2c_scl);
      if (obj->i2c == NP) 
      {
        core_debug("ERROR: I2C pins mismatch\n");
      } 
      else 
      {
#if defined I2C1_BASE
        // Enable I2C1 clock if not done
        if (obj->i2c == I2C1) 
        {
          RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
          RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
          RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE );

          obj->irq = I2C1_EV_IRQn;
          obj->irqER = I2C1_ER_IRQn;
          i2c_handles[I2C1_INDEX] = handle;
        }
#endif // I2C1_BASE
#if defined I2C2_BASE
        // Enable I2C2 clock if not done
        if (obj->i2c == I2C2) 
        {
          RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
          RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);
          RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE );

          obj->irq = I2C2_EV_IRQn;
          obj->irqER = I2C2_ER_IRQn;
          i2c_handles[I2C2_INDEX] = handle;
        }
#endif // I2C2_BASE


        /* Configure I2C GPIO pins */
        pinmap_pinout(obj->scl, PinMap_I2C_SCL);
        pinmap_pinout(obj->sda, PinMap_I2C_SDA);

        handle->Instance                 = obj->i2c;
        handle->Init.I2C_ClockSpeed      = i2c_getTiming(obj, timing);
        handle->Init.I2C_Mode            = I2C_Mode_I2C;
        /* Standard mode (sm) is up to 100kHz, then it's Fast mode (fm)     */
        /* In fast mode duty cyble bit must be set in CCR register          */
        if (timing > 100000) 
        {
          handle->Init.I2C_DutyCycle     = I2C_DutyCycle_16_9;
        } 
        else 
        {
          handle->Init.I2C_DutyCycle     = I2C_DutyCycle_2;
        }
        handle->Init.I2C_OwnAddress1     = ownAddress;
        handle->Init.I2C_Ack             = I2C_Ack_Enable;
        handle->Init.I2C_AcknowledgedAddress = addressingMode;

#if OPT_I2C_SLAVE // MMOLE: was 0 (all calls were commented out)
		// MMOLE: enable/setup interrupts. Note: all lines within the if were commented out
		if(obj->isMaster==0)
		{
          // MMOLE: Enable I2C interrupts (used value is supported by all CH32's; followed ch32v003fun i2c_slave example by @cnlohr)
          obj->i2c->CTLR2 |= I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN;
		  
          /* Interruption is temporarily not supported */ 
          //NVIC_SetPriority(obj->irq, I2C_IRQ_PRIO, I2C_IRQ_SUBPRIO);
          NVIC_EnableIRQ(obj->irq); // Event interrupt

          //NVIC_SetPriority(obj->irqER, I2C_IRQ_PRIO, I2C_IRQ_SUBPRIO);
          NVIC_EnableIRQ(obj->irqER); // Error interrupt
		}
#endif

        /* Init the I2C */
        I2C_Init(obj->i2c, &(handle->Init));
        I2C_Cmd(obj->i2c,ENABLE);

        /* Initialize default values */
        //obj->slaveRxNbData = 0;		// MMOLE: was commented
        //obj->slaveMode = SLAVE_MODE_LISTEN;		// MMOLE: was commented
      }
    }
  }
}

/**
  * @brief  Initialize and setup GPIO and I2C peripheral
  * @param  obj : pointer to i2c_t structure
  * @retval none
  */
void i2c_deinit(i2c_t *obj)
{
  // MMOLE TODO: Disable I2C interrupts
  // do reverse of this: obj->i2c->CTLR2 |= I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN;
  NVIC_DisableIRQ(obj->irq);
  NVIC_DisableIRQ(obj->irqER);
  I2C_DeInit(obj->i2c);
}

/**
  * @brief  Setup transmission speed. I2C must be configured before.
  * @param  obj : pointer to i2c_t structure
  * @param  frequency : i2c transmission speed
  * @retval none
  */
void i2c_setTiming(i2c_t *obj, uint32_t frequency)
{
  uint32_t f = i2c_getTiming(obj, frequency);
  I2C_Cmd(obj->i2c, DISABLE);


  obj->handle.Init.I2C_ClockSpeed = f;
  /* Standard mode (sm) is up to 100kHz, then it's Fast mode (fm)     */
  /* In fast mode duty cyble bit must be set in CCR register          */
  if (frequency > 100000) 
  {
    obj->handle.Init.I2C_DutyCycle       = I2C_DutyCycle_16_9;
  }
  else 
  {
    obj->handle.Init.I2C_DutyCycle       = I2C_DutyCycle_2;
  }
  I2C_Init(obj->i2c, &(obj->handle.Init));
  I2C_Cmd(obj->i2c, ENABLE);

  if(obj->isMaster) I2C_AcknowledgeConfig( obj->i2c, ENABLE ); //if OWMaddr is 0x01,means masters
}

/**
  * @brief  Write bytes at a given address
  * @param  obj : pointer to i2c_t structure
  * @param  dev_address: specifies the address of the device.
  * @param  data: pointer to data to be write
  * @param  size: number of bytes to be write.
  * @retval read status
  */
i2c_status_e i2c_master_write(i2c_t *obj, uint8_t dev_address,
                              uint8_t *data, uint16_t size, uint8_t sendstop)
{
  // MMOLE: Added support for I2C scanning using Wire.endTransmission() without sending data
  // Arduino examples: 
  //    i2c_scanner by Nick Gammon (broken link: http://arduino-info.wikispaces.com/LCD-Blue-I2C), 
  //    https://playground.arduino.cc/Main/I2cScanner/
  //    https://learn.adafruit.com/scanning-i2c-addresses/arduino
  // Inspiration from https://github.com/mockthebear/easy-ch32v003/tree/main/examples/i2c_scanner


  i2c_status_e  ret = I2C_OK;
  uint32_t tickstart = GetTick();
  {
    I2C_AcknowledgeConfig( obj->handle.Instance, ENABLE ); 
    while(I2C_GetFlagStatus(obj->handle.Instance, I2C_FLAG_BUSY) != RESET)  //wait for busy
    {
        if((GetTick()-tickstart) > I2C_TIMEOUT_TICK) 
        {
/**/
// MMOLE: To allow I2C scanning, when a start condition times out the bus needs to be released
          if(sendstop)  
            I2C_GenerateSTOP(obj->handle.Instance, ENABLE);
/**/
          return I2C_TIMEOUT;
        }
    }
    tickstart = GetTick();
    I2C_GenerateSTART(obj->handle.Instance, ENABLE);
    while(!I2C_CheckEvent(obj->handle.Instance, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if((GetTick()-tickstart) > I2C_TIMEOUT_TICK) 
        {
/**/
// MMOLE: To allow I2C scanning, when a start condition times out the bus needs to be released
          if(sendstop)  
            I2C_GenerateSTOP(obj->handle.Instance, ENABLE);
/**/
          return I2C_TIMEOUT;
        }
    }
    tickstart = GetTick();
    I2C_Send7bitAddress(obj->handle.Instance, dev_address, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(obj->handle.Instance, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))//{}
    {
        if((GetTick()-tickstart) > I2C_TIMEOUT_TICK) 
        {
// MMOLE: To allow I2C scanning, timeout on an addresses should release the bus
          if(sendstop)  
            I2C_GenerateSTOP(obj->handle.Instance, ENABLE);
          return I2C_TIMEOUT;
        }
    }

// MMOLE: Support for I2C scanning: allow only sending the address (without actual data)
if(size)
{  
    while(size)
    {
        if( I2C_GetFlagStatus(obj->handle.Instance, I2C_FLAG_TXE) != RESET )
        {
            I2C_SendData(obj->handle.Instance, *data++);
            size--;
        }
    }

    tickstart = GetTick();
    while(!I2C_CheckEvent(obj->handle.Instance, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if((GetTick()-tickstart) > I2C_TIMEOUT_TICK) 
        {
          return I2C_TIMEOUT;
        }
    }
}   // if(size)

    if(sendstop)  
    {
      I2C_GenerateSTOP(obj->handle.Instance, ENABLE);
    }
  }
  return ret;
}

/**
  * @brief  Write bytes to master
  * @param  obj : pointer to i2c_t structure
  * @param  data: pointer to data to be write
  * @param  size: number of bytes to be write.
  * @retval status
  */
#if OPT_I2C_SLAVE
i2c_status_e i2c_slave_write_IT(i2c_t *obj, uint8_t *data, uint16_t size)
{
 

}
#endif // #if OPT_I2C_SLAVE

#if OPT_I2C_SLAVE

void i2c_ClearErrorFlags(i2c_t *obj)
{   // clear the various error flags that may block further communication

  if(I2C_GetFlagStatus( obj->i2c, I2C_FLAG_AF ) !=  RESET)
  { //  I2C_FLAG_AF - Acknowledge failure flag.
    I2C_ClearFlag( obj->i2c, I2C_FLAG_AF );
  }
  if(I2C_GetFlagStatus( obj->i2c, I2C_FLAG_BERR ) !=  RESET)
  { //  I2C_FLAG_BERR -Bus Error flag.
    I2C_ClearFlag( obj->i2c, I2C_FLAG_BERR );
  }
}

void i2c_ClearStopFlag(i2c_t *obj)
{   // clear the stop flag
  if(I2C_GetFlagStatus( obj->i2c, I2C_FLAG_STOPF ) !=  RESET)
  { // Stop detection flag (Slave mode).
    // STOPF (STOP detection) is cleared by software sequence: a read operation 
    // to I2C_STAR1 register (I2C_GetFlagStatus()) followed by a write operation 
    // to I2C_CTLR1 register (I2C_Cmd() to re-enable the I2C peripheral).
    // -> Since we just read the flag, we only need to (re-)enable.
    I2C_Cmd(obj->i2c, ENABLE);
  }
}

// MMOLE: added function to process I2C slave data transfers
void i2c_slave_process(i2c_t *obj)
{ // Process incoming and outgoing I2C data.
  // When processing the data we can assume there is an address match.
  // We could wait for an address match, but that would be blocking
  // and isn't needed as RX/TX-flags are only set when addressed properly.

  // Process receiving data
  if(I2C_GetFlagStatus( obj->i2c, I2C_FLAG_RXNE ) !=  RESET)
  { // Data register not empty (Receiver) flag; read all available data and store it
    while(I2C_GetFlagStatus( obj->i2c, I2C_FLAG_RXNE ) !=  RESET)
    {
      char c=I2C_ReceiveData( obj->i2c );
      if(obj->i2cTxRxBufferSize<I2C_TXRX_BUFFER_SIZE)
      {
        obj->i2cTxRxBuffer[obj->i2cTxRxBufferSize]=c;
        obj->i2cTxRxBufferSize++;
      }
    }
  }

  // Process end of receiving data, as determined by stop flag
  if(I2C_CheckEvent( obj->i2c, I2C_EVENT_SLAVE_STOP_DETECTED))
  { // When done receiving let's do the callback
    // Note: twi.c::i2c_onSlaveReceive is tied to the TwoWire::onReceiveService().
    obj->slaveRxNbData=obj->i2cTxRxBufferSize; // remember so we can forget
    obj->i2cTxRxBufferSize=0;   // reset the buffer for next incoming data
    if(obj->slaveRxNbData>0 && obj->i2c_onSlaveReceive) obj->i2c_onSlaveReceive(obj);

	// clear the stop flag to be ready for another session
	i2c_ClearStopFlag(obj);
  }

  // Process transmitting data  
  if(I2C_GetFlagStatus( obj->i2c, I2C_FLAG_TXE ) !=  RESET)
  { // Data register empty flag (Transmitter).
    // It seems we need to send something, give the callback opportunity to do so.
    // Note: twi.c::i2c_onSlaveTransmit is tied to the TwoWire::onRequestService().
    // The onRequest callback uses Wire.write(), which calls twi.c::i2c_slave_write(), 
    // which then calls I2C_SendData() and checks if done within timeout */
    if(obj->i2c_onSlaveTransmit) obj->i2c_onSlaveTransmit(obj);
  }

  // Clear error flags (since we don't handle them anyways)
  i2c_ClearErrorFlags(obj);
}
#endif // #if OPT_I2C_SLAVE


/**
  * @brief  Write bytes to master
  * @param  obj : pointer to i2c_t structure
  * @param  data: pointer to data to be write
  * @param  size: number of bytes to be write.
  * @retval status
  */
i2c_status_e i2c_slave_write(i2c_t *obj, uint8_t *data, uint16_t size)
{
#if OPT_I2C_SLAVE
  uint8_t i = 0;
  i2c_status_e ret = I2C_OK;
  uint32_t tickstart = GetTick();

  while(i<size)
  {
    I2C_SendData(obj->handle.Instance,data[i]);
    i++;
    
    while(!I2C_CheckEvent( obj->handle.Instance, I2C_EVENT_SLAVE_BYTE_TRANSMITTED ) )
    {
      if( (GetTick() - tickstart) >= I2C_TIMEOUT_TICK) 
      {
        ret =  I2C_TIMEOUT;
        break;
      }

//#if OPT_I2C_SLAVE
      // don't wait too long. When the controller wants to read immediately there can be a new event (0x00060080) waiting
      uint32_t evt=I2C_GetLastEvent(obj->i2c);
      if(!evt)
        break;
      i2c_ClearErrorFlags(obj);
      i2c_ClearStopFlag(obj);  // really needed here?
//#endif // #if OPT_I2C_SLAVE
    }

    //i2c_ClearErrorFlags(obj);
    //i2c_ClearStopFlag(obj);  // really needed here?
  }
  return ret;
#endif // #if OPT_I2C_SLAVE
}







/**
  * @brief  read bytes in master mode at a given address
  * @param  obj : pointer to i2c_t structure
  * @param  dev_address: specifies the address of the device.
  * @param  data: pointer to data to be read
  * @param  size: number of bytes to be read.
  * @retval read status
  */
i2c_status_e i2c_master_read(i2c_t *obj, uint8_t dev_address, uint8_t *data, uint16_t size)
{
  i2c_status_e ret = I2C_OK;
  uint32_t tickstart = GetTick();

  I2C_GenerateSTART( obj->handle.Instance, ENABLE );
  while( !I2C_CheckEvent( obj->handle.Instance, I2C_EVENT_MASTER_MODE_SELECT ) )
  {
      if((GetTick()-tickstart) > I2C_TIMEOUT_TICK) 
      {
          return I2C_TIMEOUT;
      }    
  }

	I2C_Send7bitAddress( obj->handle.Instance, dev_address, I2C_Direction_Receiver );
  while( !I2C_CheckEvent( obj->handle.Instance, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) )
  {
      if((GetTick()-tickstart) > I2C_TIMEOUT_TICK) 
      {
          return I2C_TIMEOUT;
      }
  }

  I2C_AcknowledgeConfig( obj->handle.Instance, ENABLE );  //MMOLE fix: every byte except last needs to ACK, also on repeated reads
  while(size)
  {
      if(size==1) I2C_AcknowledgeConfig( obj->handle.Instance, DISABLE );  //last data needn't to ACK
      while(I2C_GetFlagStatus( obj->handle.Instance, I2C_FLAG_RXNE ) ==  RESET)
      {
        // MMOLE: at occasion the CH32 would just hang while reading the response. Still unknown why. but adding a timeout may prevent this
        if((GetTick()-tickstart) > I2C_TIMEOUT_TICK) 
        {
            I2C_GenerateSTOP( obj->handle.Instance, ENABLE );
            return I2C_TIMEOUT;
        }
      }
      *data++ = I2C_ReceiveData( obj->handle.Instance );
      size--;
  }
  I2C_GenerateSTOP( obj->handle.Instance, ENABLE );
  return ret;
}


/**
  * @brief  Write bytes to master
  * @param  obj : pointer to i2c_t structure
  * @param  data: pointer to data to be write
  * @param  size: number of bytes to be write.
  * @retval status
  */
i2c_status_e i2c_slave_read(i2c_t *obj, uint8_t *data, uint16_t size)
{
#if OPT_I2C_SLAVE
  uint8_t i = 0;
  i2c_status_e ret = I2C_OK;
  uint32_t tickstart = GetTick();

  while(i<size)
  {
    if(I2C_GetFlagStatus(obj->handle.Instance, I2C_FLAG_RXNE) != RESET)
    {
      data[i] = I2C_ReceiveData(obj->handle.Instance);
      i++;
    }
    if( (GetTick() - tickstart) >= I2C_TIMEOUT_TICK) 
    {
      ret =  I2C_TIMEOUT;
      break;
    }

  }
  return ret;
#endif // #if OPT_I2C_SLAVE
}





/**
  * @brief  Checks if target device is ready for communication
  * @param  obj : pointer to i2c_t structure
  * @param  devAddr: specifies the address of the device.
  * @param  trials : Number of trials.
  * @retval status
  */
i2c_status_e i2c_IsDeviceReady(i2c_t *obj, uint8_t devAddr, uint32_t trials)
{
 
}

/* Aim of the function is to get i2c_s pointer using hi2c pointer */
/* Highly inspired from magical linux kernel's "container_of" */
/* (which was not directly used since not compatible with IAR toolchain) */
i2c_t *get_i2c_obj(I2C_HandleTypeDef *hi2c)
{
  struct i2c_s *obj_s;
  i2c_t *obj;

  obj_s = (struct i2c_s *)((char *)hi2c - offsetof(struct i2c_s, handle));
  obj = (i2c_t *)((char *)obj_s - offsetof(i2c_t, i2c));

  return (obj);
}

/** @brief  sets function called before a slave read operation
  * @param  obj : pointer to i2c_t structure
  * @param  function: callback function to use
  * @retval None
  */
void i2c_attachSlaveRxEvent(i2c_t *obj, void (*function)(i2c_t *))
{
#if OPT_I2C_SLAVE
  // MMOLE 240320: this is where we attach the Wire.onReceiveService to our i2c object
  // The Wire.onReceiveService does the actual callback at the moment all data is receved.
  obj->i2c_onSlaveReceive=function;
#endif
}

/** @brief  sets function called before a slave write operation
  * @param  obj : pointer to i2c_t structure
  * @param  function: callback function to use
  * @retval None
  */
void i2c_attachSlaveTxEvent(i2c_t *obj, void (*function)(i2c_t *))
{
#if OPT_I2C_SLAVE
  // MMOLE 240320: this is where we attach the Wire.onRequestService to our i2c object
  // The Wire.onRequestService does the actual callback at the moment data needs to be transmitted.
  obj->i2c_onSlaveTransmit=function;
#endif
}



#if defined(I2C1_BASE)
/**
* @brief  This function handles I2C1 interrupt.
* @param  None
* @retval None
*/
void I2C1_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_EV_IRQHandler(void)
{
#if OPT_I2C_SLAVE
   I2C_HandleTypeDef *handle = i2c_handles[I2C1_INDEX];  // MMOLE: was commented
   // MMOLE: I2C1_EV_IRQHandler is the event handler, handle is an I2C_HandleTypeDef struct containing parameters and pointer to the registers
   static int _nCounterEV1=1;
   _nCounterEV1++;
   i2c_slave_process(get_i2c_obj(handle));		// process I2C transmissions, for now only events, not errors
#endif
 }
/**
* @brief  This function handles I2C1 interrupt.
* @param  None
* @retval None
*/
void I2C1_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_ER_IRQHandler(void)
{
#if OPT_I2C_SLAVE
   //I2C_HandleTypeDef *handle = i2c_handles[I2C1_INDEX];  // MMOLE: was commented
   // MMOLE: I2C1_ER_IRQHandler is the error handler
   static int _nCounterER1=1;
   _nCounterER1++;
#endif
}
#endif // I2C1_BASE


#if defined(I2C2_BASE)
/**
* @brief  This function handles I2C2 interrupt.
* @param  None
* @retval None
*/
void I2C2_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C2_EV_IRQHandler(void)
{
#if OPT_I2C_SLAVE
   I2C_HandleTypeDef *handle = i2c_handles[I2C2_INDEX];  // MMOLE: was commented
   // MMOLE: I2C2_EV_IRQHandler is the event handler, handle is an I2C_HandleTypeDef struct containing parameters and pointer to the registers
   static int _nCounterEV2=1;
   _nCounterEV2++;
   i2c_slave_process(get_i2c_obj(handle));		// process I2C transmissions, for now only events, not errors
   // MMOLE: tested only using I2C1
#endif
}

/**
* @brief  This function handles I2C2 interrupt.
* @param  None
* @retval None
*/
void I2C2_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C2_ER_IRQHandler(void)
{
#if OPT_I2C_SLAVE
   //I2C_HandleTypeDef *handle = i2c_handles[I2C2_INDEX];  // MMOLE: was commented
   // MMOLE: I2C2_ER_IRQHandler is the error handler
   static int _nCounterER2=1;
   _nCounterER2++;
#endif
}

#endif // I2C2_BASE




#ifdef __cplusplus
}
#endif


