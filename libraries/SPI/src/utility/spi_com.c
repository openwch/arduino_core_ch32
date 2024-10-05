/**
  ******************************************************************************
  * @file    spi_com.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    01-August-2016
  * @brief   provide the SPI interface
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
  modified 30 june 2023 by TempersLee to support wch's MCU
  */
#include "wiring_time.h"
#include "core_debug.h"
#include "ch32_def.h"
#include "utility/spi_com.h"
#include "PinAF_ch32yyxx.h"

#include "pinconfig.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Private Functions */
/**
  * @brief  return clock freq of an SPI instance
  * @param  spi_inst : SPI instance
  * @retval clock freq of the instance else SystemCoreClock
  */
uint32_t spi_getClkFreqInst(SPI_TypeDef *spi_inst)
{
  uint32_t spi_freq = SystemCoreClock;
  RCC_ClocksTypeDef rcc_clocks={0};
  if (spi_inst != NP) 
  {
    RCC_GetClocksFreq( &rcc_clocks);
#if defined(SPI1_BASE)
    if (spi_inst == SPI1) 
    {
      {
        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
        spi_freq = rcc_clocks.PCLK2_Frequency;
        // spi_freq = HAL_RCC_GetPCLK2Freq();
      }
    }
#endif // SPI1_BASE
#if defined(SPI2_BASE)
    if (spi_inst == SPI2) 
    {
      {
        /* SPI_2 and SPI_3. Source CLK is PCKL1 */
        spi_freq = rcc_clocks.PCLK1_Frequency;
        // spi_freq = HAL_RCC_GetPCLK1Freq();
      }
    }
#endif // SPI2_BASE
#if defined(SPI3_BASE)
    if (spi_inst == SPI3) 
    {
      {
        /* SPI_2 and SPI_3. Source CLK is PCKL1 */
        spi_freq = rcc_clocks.PCLK1_Frequency;
        // spi_freq = HAL_RCC_GetPCLK1Freq();
      }
    }
#endif // SPI3_BASE
  }
  return spi_freq;
}

/**
  * @brief  return clock freq of an SPI instance
  * @param  obj : pointer to spi_t structure
  * @retval clock freq of the instance else SystemCoreClock
  */
uint32_t spi_getClkFreq(spi_t *obj)
{
  SPI_TypeDef *spi_inst = NP;
  uint32_t spi_freq = SystemCoreClock;

  if (obj != NULL) 
  {
    spi_inst = pinmap_peripheral(obj->pin_sclk, PinMap_SPI_SCLK);
    if (spi_inst != NP) 
    {
      spi_freq = spi_getClkFreqInst(spi_inst);
    }
  }
  return spi_freq;
}


/**
  * @brief  SPI initialization function
  * @param  obj : pointer to spi_t structure
  * @param  speed : spi output speed
  * @param  mode : one of the spi modes
  * @param  msb : set to 1 in msb first
  * @retval None
  */
void spi_init(spi_t *obj, uint32_t speed, spi_mode_e mode, uint8_t msb)
{
    if (obj == NULL) {
      return;
   }

    SPI_HandleTypeDef *handle = &(obj->handle);
    uint32_t spi_freq = 0;

    // Determine the SPI to use
    SPI_TypeDef *spi_mosi = pinmap_peripheral(obj->pin_mosi, PinMap_SPI_MOSI);
    SPI_TypeDef *spi_miso = pinmap_peripheral(obj->pin_miso, PinMap_SPI_MISO);
    SPI_TypeDef *spi_sclk = pinmap_peripheral(obj->pin_sclk, PinMap_SPI_SCLK);
    SPI_TypeDef *spi_ssel = pinmap_peripheral(obj->pin_ssel, PinMap_SPI_SSEL);
  
    /* Pins MOSI/MISO/SCLK must not be NP. ssel can be NP. */
    if (spi_mosi == NP || spi_miso == NP || spi_sclk == NP) {
      core_debug("ERROR: at least one SPI pin has no peripheral\n");
      return;
    }

    SPI_TypeDef *spi_data = pinmap_merge_peripheral(spi_mosi, spi_miso);
    SPI_TypeDef *spi_cntl = pinmap_merge_peripheral(spi_sclk, spi_ssel);

    obj->spi = pinmap_merge_peripheral(spi_data, spi_cntl);

    // Are all pins connected to the same SPI instance?
    if (spi_data == NP || spi_cntl == NP || obj->spi == NP) {
      core_debug("ERROR: SPI pins mismatch\n");
      return;
    }


  // Configure the SPI pins
  if (obj->pin_ssel != NC) 
  {
    handle->Init.SPI_NSS = SPI_NSS_Hard;
  } 
  else 
  {
    handle->Init.SPI_NSS = SPI_NSS_Soft;
  }

  /* Fill default value */
  handle->Instance               = obj->spi;
  handle->Init.SPI_Mode          = SPI_Mode_Master;

  spi_freq = spi_getClkFreqInst(obj->spi);
  /* For SUBGHZSPI,  'SPI_BAUDRATEPRESCALER_*' == 'SUBGHZSPI_BAUDRATEPRESCALER_*' */
  if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV2_MHZ)) {
    handle->Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV4_MHZ)) {
    handle->Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV8_MHZ)) {
    handle->Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV16_MHZ)) {
    handle->Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV32_MHZ)) {
    handle->Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV64_MHZ)) {
    handle->Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV128_MHZ)) {
    handle->Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
  } else {
    /*
     * As it is not possible to go below (spi_freq / SPI_SPEED_CLOCK_DIV256_MHZ).
     * Set prescaler at max value so get the lowest frequency possible.
     */
    handle->Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  }

#if defined(SPI_IFCR_EOTC)
  // Compute disable delay as baudrate has been modified
  obj->disable_delay = compute_disable_delay(obj);
#endif

  handle->Init.SPI_Direction     = SPI_Direction_2Lines_FullDuplex;

  if ((mode == SPI_MODE_0) || (mode == SPI_MODE_2)) {
    handle->Init.SPI_CPHA        = SPI_CPHA_1Edge;
  } else {
    handle->Init.SPI_CPHA        = SPI_CPHA_2Edge;
  }

  if ((mode == SPI_MODE_0) || (mode == SPI_MODE_1)) {
    handle->Init.SPI_CPOL        = SPI_CPOL_Low;
  } else {
    handle->Init.SPI_CPOL        = SPI_CPOL_High;
  }
  handle->Init.SPI_CRCPolynomial = 7;
  handle->Init.SPI_DataSize      = SPI_DataSize_8b;

  if (msb == 0) {
    #ifndef CH32V00x
    handle->Init.SPI_FirstBit    = SPI_FirstBit_LSB;
    #else
    core_debug("CH32V00x only surport MSB mode\r\n");  
    #endif
  } else {
    handle->Init.SPI_FirstBit    = SPI_FirstBit_MSB;
  }

    /* Configure SPI GPIO pins */
    pinmap_pinout(obj->pin_mosi, PinMap_SPI_MOSI);
    pinmap_pinout(obj->pin_miso, PinMap_SPI_MISO);
    pinmap_pinout(obj->pin_sclk, PinMap_SPI_SCLK);
    pinmap_pinout(obj->pin_ssel, PinMap_SPI_SSEL);


#if defined SPI1_BASE
  // Enable SPI clock
  if (handle->Instance == SPI1) 
  {
    #if defined(CH32L10x)
    RCC_PB2PeriphResetCmd(RCC_PB2Periph_SPI1, ENABLE);
    RCC_PB2PeriphResetCmd(RCC_PB2Periph_SPI1, DISABLE);
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_SPI1, ENABLE );
    #else
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE );
    #endif
  }
#endif

#if defined SPI2_BASE
  if (handle->Instance == SPI2) 
  {
    #if defined(CH32L10x)
    RCC_PB1PeriphResetCmd(RCC_PB1Periph_SPI2, ENABLE);
    RCC_PB1PeriphResetCmd(RCC_PB1Periph_SPI2, DISABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_SPI2, ENABLE );
    #else
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE );
    #endif
  }
#endif

#if defined SPI3_BASE
  if (handle->Instance == SPI3) 
  {
    #if defined(CH32L10x)
    RCC_PB1PeriphResetCmd(RCC_PB1Periph_SPI3, ENABLE);
    RCC_PB1PeriphResetCmd(RCC_PB1Periph_SPI3, DISABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_SPI3, ENABLE );
    #else
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE );    
    #endif
  }
#endif
  SPI_Init( handle->Instance, &handle->Init );
  
  /* In order to set correctly the SPI polarity we need to enable the peripheral */
  SPI_Cmd( handle->Instance, ENABLE );
}

/**
  * @brief This function is implemented to deinitialize the SPI interface
  *        (IOs + SPI block)
  * @param  obj : pointer to spi_t structure
  * @retval None
  */
void spi_deinit(spi_t *obj)
{
  if (obj == NULL) {
    return;
  }

  SPI_HandleTypeDef *handle = &(obj->handle);

  SPI_I2S_DeInit(handle->Instance);

#if defined SPI1_BASE
  // Reset SPI and disable clock
  if (handle->Instance == SPI1) 
  {
    #if defined(CH32L10x)
    RCC_PB2PeriphResetCmd(RCC_PB2Periph_SPI1, ENABLE);
    RCC_PB2PeriphResetCmd(RCC_PB2Periph_SPI1, DISABLE);
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_SPI1, DISABLE);
    #else
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, DISABLE);
    #endif
  }
#endif
#if defined SPI2_BASE
  if (handle->Instance == SPI2) 
  {
    #if defined(CH32L10x)
    RCC_PB1PeriphResetCmd(RCC_PB1Periph_SPI2, ENABLE);
    RCC_PB1PeriphResetCmd(RCC_PB1Periph_SPI2, DISABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_SPI2, DISABLE);
    #else
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, DISABLE);
    #endif
  }
#endif

#if defined SPI3_BASE
  if (handle->Instance == SPI3) 
  {
    #if defined(CH32L10x)
    RCC_PB1PeriphResetCmd(RCC_PB1Periph_SPI3, ENABLE);
    RCC_PB1PeriphResetCmd(RCC_PB1Periph_SPI3, DISABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_SPI3, DISABLE);
    #else
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, DISABLE);
    #endif
  }
#endif
}

/**
  * @brief This function is implemented by user to send data over SPI interface
  * @param  obj : pointer to spi_t structure
  * @param  Data : data to be sent
  * @param  len : length in bytes of the data to be sent
  * @param  Timeout: Timeout duration in tick
  * @retval status of the send operation (0) in case of error
  */
spi_status_e spi_send(spi_t *obj, uint8_t *Data, uint16_t len, uint32_t Timeout)
{
  return spi_transfer(obj, Data, Data, len, Timeout, 1 /* SPI_TRANSMITONLY */);
}

/**
  * @brief This function is implemented by user to send/receive data over
  *         SPI interface
  * @param  obj : pointer to spi_t structure
  * @param  tx_buffer : tx data to send before reception
  * @param  rx_buffer : data to receive
  * @param  len : length in byte of the data to send and receive
  * @param  Timeout: Timeout duration in tick
  * @param  skipReceive: skip receiving data after transmit or not
  * @retval status of the send operation (0) in case of error
  */
spi_status_e spi_transfer(spi_t *obj, uint8_t *tx_buffer, uint8_t *rx_buffer,
                          uint16_t len, uint32_t Timeout, bool skipReceive)
{
  spi_status_e ret = SPI_OK;
  uint32_t tickstart, size = len;
  SPI_TypeDef *_SPI = obj->handle.Instance;

  if ((obj == NULL) || (len == 0) || (Timeout == 0U)) {
    return Timeout > 0U ? SPI_ERROR : SPI_TIMEOUT;
  }
  tickstart = GetTick();

  while (size--) {
    while(SPI_I2S_GetFlagStatus(_SPI, SPI_I2S_FLAG_TXE) == RESET)
    {
      if ((GetTick() - tickstart >= Timeout)) {
        ret = SPI_TIMEOUT;
        break;
      }
    }
    SPI_I2S_SendData(_SPI, *tx_buffer++);

    if (!skipReceive) {
      while(SPI_I2S_GetFlagStatus(_SPI, SPI_I2S_FLAG_RXNE) == RESET)
      {
        if ((GetTick() - tickstart >= Timeout)) {
        ret = SPI_TIMEOUT;
        break;
        }
      }
      *rx_buffer++ = SPI_I2S_ReceiveData(_SPI);
    }

    if ((GetTick() - tickstart >= Timeout)) {
      ret = SPI_TIMEOUT;
      break;
    }
  }

  /* Wait for end of transfer */
  while(SPI_I2S_GetFlagStatus(_SPI, SPI_I2S_FLAG_BSY));

  return ret;
}

#ifdef __cplusplus
}
#endif


