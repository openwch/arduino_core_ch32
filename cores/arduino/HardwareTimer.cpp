/*
  Copyright (c) 2017 Daniel Fekete

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  Copyright (c) 2019 STMicroelectronics
  Modified to support Arduino_Core_STM32

  Modified by TempersLee to support Adruino_Core_CH32
*/

#include "Arduino.h"
#include "HardwareTimer.h"

#if defined(TIM_MODULE_ENABLED) && !defined(TIM_MODULE_ONLY)

/* Private Defines */
#define PIN_NOT_USED 0xFF
#define MAX_RELOAD ((1 << 16) - 1) // Currently even 32b timers are used as 16b to have generic behavior

/* Private Variables */
timerObj_t *HardwareTimer_Handle[TIMER_NUM] = {NULL};

/**
  * @brief  HardwareTimer constructor: make uninitialized timer
  *         Before calling any methods, call setup to select and setup
  *         the timer to be used.
  * @retval None
  */
HardwareTimer::HardwareTimer()
{
  _timerObj.handle.Instance = nullptr;
}

/**
  * @brief  HardwareTimer constructor: set default configuration values
  *         The timer will be usable directly, there is no need to call
  *         setup(). Using this constructor is not recommended for
  *         global variables that are automatically initialized at
  *         startup, since this will happen to early to report any
  *         errors. Better use the argumentless constructor and call the
  *         setup() method during initialization later.
  * @param  Timer instance ex: TIM1, ...
  * @retval None
  */
HardwareTimer::HardwareTimer(TIM_TypeDef *instance)
{
  _timerObj.handle.Instance = nullptr;
  setup(instance);
  
#if defined(TIM1_BASE) 
  NVIC_EnableIRQ(TIM1_UP_IRQn);
  NVIC_EnableIRQ(TIM1_CC_IRQn);
#endif

#ifdef TIM2_BASE 
  #if defined(CH32X035)
    NVIC_EnableIRQ(TIM2_UP_IRQn);
    NVIC_EnableIRQ(TIM2_CC_IRQn);
  #else
    NVIC_EnableIRQ(TIM2_IRQn);
  #endif
#endif

#if defined(TIM3_BASE) && !defined(CH32VM00X)  //v006 has no interruption
  NVIC_EnableIRQ(TIM3_IRQn);  
#endif

#ifdef TIM4_BASE
  NVIC_EnableIRQ(TIM4_IRQn);
#endif

#ifdef CH32V30x

#ifdef TIM5_BASE
  NVIC_EnableIRQ(TIM5_IRQn);
#endif

#ifdef TIM6_BASE
  NVIC_EnableIRQ(TIM6_IRQn);
#endif

#ifdef TIM7_BASE
  NVIC_EnableIRQ(TIM7_IRQn);
#endif

#ifdef TIM8_BASE
  NVIC_EnableIRQ(TIM8_UP_IRQn);
  NVIC_EnableIRQ(TIM8_CC_IRQn);
#endif

#ifdef TIM9_BASE
  NVIC_EnableIRQ(TIM9_UP_IRQn);
  NVIC_EnableIRQ(TIM9_CC_IRQn);
#endif

#ifdef TIM10_BASE
  NVIC_EnableIRQ(TIM10_UP_IRQn);
  NVIC_EnableIRQ(TIM10_CC_IRQn);
#endif

#endif

}

/**
  * @brief  HardwareTimer setup: configuration values. Must be called
  * exactly once before any other methods, except when an instance is
  * passed to the constructor.
  * @param  Timer instance ex: TIM1, ...
  * @retval None
  */
void HardwareTimer::setup(TIM_TypeDef *instance)
{
  uint32_t index = get_timer_index(instance);
  if (index == UNKNOWN_TIMER) {
    Error_Handler();
  }

  // Already initialized?
  if (_timerObj.handle.Instance) {
    Error_Handler();
  }

  HardwareTimer_Handle[index] = &_timerObj;

  _timerObj.handle.Instance = instance;

  _timerObj.__this = (void *)this;
  _timerObj.preemptPriority = TIM_IRQ_PRIO;   
  _timerObj.subPriority = TIM_IRQ_SUBPRIO;
  _timerObj.handle.Init={0}; 
  /* Enable timer clock. Even if it is also done in HAL_TIM_Base_MspInit(),
     it is done there so that it is possible to write registers right now */
  enableTimerClock(&(_timerObj.handle));
  // Initialize NULL callbacks
  for (int i = 0; i < TIMER_CHANNELS + 1 ; i++) {
    callbacks[i] = NULL;
  }

  // Initialize channel mode and complementary
  for (int i = 0; i < TIMER_CHANNELS; i++) {
#if defined(TIM_CC1NE)
    isComplementaryChannel[i] = false;  //如果有互补通道定义
#endif
    _ChannelMode[i] = TIMER_DISABLED;    
  }

  /* Configure timer with some default values */

  _timerObj.handle.Init.TIM_Prescaler     = 0;
  _timerObj.handle.Init.TIM_Period        = MAX_RELOAD;
  _timerObj.handle.Init.TIM_CounterMode   = TIM_CounterMode_Up;
  _timerObj.handle.Init.TIM_ClockDivision = TIM_CKD_DIV1;


#if defined(TIM_RCR_REP)
  _timerObj.handle.Init.TIM_RepetitionCounter = 0;
#endif
  TIM_ARRPreloadConfig( _timerObj.handle.Instance, ENABLE );
  TIM_TimeBaseInit( _timerObj.handle.Instance, &_timerObj.handle.Init);

}


/**
  * @brief  Pause HardwareTimer: stop timer
  * @param  None
  * @retval None
  */
void HardwareTimer::pause()
{
  // Disable all IT
  TIM_ITConfig(_timerObj.handle.Instance, TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, DISABLE);
  // Stop timer. Required to restore HAL State: HAL_TIM_STATE_READY
  TIM_Cmd(_timerObj.handle.Instance,DISABLE);
}

/**
  * @brief  Pause only one channel.
  *         Timer is still running but channel is disabled (output and interrupt)
  * @param  Arduino channel [1..4]
  * @retval None
  */
void HardwareTimer::pauseChannel(uint32_t channel)
{
  int timAssociatedInputChannel;
  int LLChannel = getLLChannel(channel);
  if (LLChannel == -1) {
    Error_Handler();
  }

  int interrupt = getIT(channel);
  if (interrupt == -1) {
    Error_Handler();
  }

  // Disable channel and corresponding interrupt
  TIM_ITConfig(_timerObj.handle.Instance, interrupt, DISABLE);
  TIM_CCxCmd(_timerObj.handle.Instance,LLChannel,TIM_CCx_Disable);

#if defined(TIM_CHANNEL_STATE_SET)
  /* Starting from G4, new Channel state implementation prevents to restart a channel,
     if the channel has not been explicitly be stopped with HAL interface */
#if defined(TIM_CHANNEL_N_STATE_SET)
  if (isComplementaryChannel[channel - 1]) 
  {
    // TIM_CHANNEL_N_STATE_SET(&(_timerObj.handle), getChannel(channel), HAL_TIM_CHANNEL_STATE_READY);
  } 
  else
#endif
  {
    // TIM_CHANNEL_STATE_SET(&(_timerObj.handle), getChannel(channel), HAL_TIM_CHANNEL_STATE_READY);
  }
#endif

  // In case 2 channels are used, disable also the 2nd one
  if (_ChannelMode[channel - 1] == TIMER_INPUT_FREQ_DUTY_MEASUREMENT) 
  {
      // Identify and configure 2nd associated channel
      timAssociatedInputChannel = getAssociatedChannel(channel);
      TIM_ITConfig(_timerObj.handle.Instance, getIT(timAssociatedInputChannel), DISABLE);
      TIM_CCxCmd(_timerObj.handle.Instance,getLLChannel(timAssociatedInputChannel),TIM_CCx_Disable);
  }
}

/**
  * @brief  Start or resume HardwareTimer: all channels are resumed, interrupts are enabled if necessary
  * @param  None
  * @retval None
  */
void HardwareTimer::resume(void)
{
  // Clear flag and enable IT
  if (callbacks[0])  // 0 for update
  {
    TIM_ClearFlag(_timerObj.handle.Instance,TIM_FLAG_Update );  
    TIM_ITConfig(_timerObj.handle.Instance, TIM_IT_Update, ENABLE);
    // Start timer in Time base mode. Required when there is no channel used but only update interrupt.
    TIM_Cmd(_timerObj.handle.Instance, ENABLE );
  }
  // Resume all channels
  resumeChannel(1);
  resumeChannel(2);
  resumeChannel(3);
  resumeChannel(4);
}

/**
  * @brief  Convert arduino channel into HAL channel
  * @param  Arduino channel [1..4]
  * @retval HAL channel. return -1 if arduino channel is invalid
  */
 //CHANNEL 
int HardwareTimer::getChannel(uint32_t channel)
{
  uint32_t return_value;

  switch (channel) {
    case 1:
      return_value = TIM_Channel_1;
      break;
    case 2:
      return_value = TIM_Channel_2;
      break;
    case 3:
      return_value = TIM_Channel_3;
      break;
    case 4:
      return_value = TIM_Channel_4;
      break;
    default:
      return_value = -1;
  }
  return return_value;
}

/**
  * @brief  Convert arduino channel into LL channel
  * @param  Arduino channel [1..4]
  * @retval LL channel. return -1 if arduino channel is invalid
  */
 //CHANNEL P or N
int HardwareTimer::getLLChannel(uint32_t channel)
{
  uint32_t return_value;
#if defined(TIM_CC1NE)
  if (isComplementaryChannel[channel - 1]) {
    // Complementary channel
    switch (channel) {
      case 1:
        return_value = TIM_CHANNEL_CH1N;
        break;
      case 2:
        return_value = TIM_CHANNEL_CH2N;
        break;
      case 3:
        return_value = TIM_CHANNEL_CH3N;
        break;
#if defined(TIM_CHANNEL_CH4N)
      case 4:
        return_value = TIM_CHANNEL_CH4N;
        break;
#endif
      default:
        return_value = -1;
    }
  } 
  else
#endif
  {
    // Regular channel not complementary
    switch (channel) {
      case 1:
        return_value = TIM_CHANNEL_CH1;
        break;
      case 2:
        return_value = TIM_CHANNEL_CH2;
        break;
      case 3:
        return_value = TIM_CHANNEL_CH3;
        break;
      case 4:
        return_value = TIM_CHANNEL_CH4;
        break;
      default:
        return_value = -1;
    }
  }
  return return_value;
}

/**
  * @brief  Convert arduino channel into HAL Interrupt ID
  * @param  Arduino channel [1..4]
  * @retval HAL channel. return -1 if arduino channel is invalid
  */
int HardwareTimer::getIT(uint32_t channel)
{
  uint32_t return_value;

  switch (channel) {
    case 1:
      return_value = TIM_IT_CC1;
      break;
    case 2:
      return_value = TIM_IT_CC2;
      break;
    case 3:
      return_value = TIM_IT_CC3;
      break;
    case 4:
      return_value = TIM_IT_CC4;
      break;
    default:
      return_value = -1;
  }
  return return_value;
}

/**
  * @brief  Get input associated channel
  *         Channel 1 and 2 are associated; channel 3 and 4 are associated
  * @param  Arduino channel [1..4]
  * @retval HAL channel. return -1 if arduino channel is invalid
  */
int HardwareTimer::getAssociatedChannel(uint32_t channel)
{
  int timAssociatedInputChannel = -1;
  switch (channel) {
    case 1:
      timAssociatedInputChannel = 2;
      break;
    case 2:
      timAssociatedInputChannel = 1;
      break;
    case 3:
      timAssociatedInputChannel = 4;
      break;
    case 4:
      timAssociatedInputChannel = 3;
      break;
    default:
      break;
  }
  return timAssociatedInputChannel;
}

/**
  * @brief  Configure specified channel and resume/start timer
  * @param  Arduino channel [1..4]
  * @retval None
  */
void HardwareTimer::resumeChannel(uint32_t channel)
{
  int timChannel = getChannel(channel); 
  int timAssociatedInputChannel;
  if (timChannel == -1) {
    Error_Handler();
  }

  int interrupt = getIT(channel);
  if (interrupt == -1) {
    Error_Handler();
  }

  int LLChannel = getLLChannel(channel);
  if (LLChannel == -1) {
    Error_Handler();
  }

  // Clear flag and enable IT
  if (callbacks[channel]) {
    TIM_ClearFlag(_timerObj.handle.Instance,interrupt );  
    TIM_ITConfig(_timerObj.handle.Instance, interrupt, ENABLE);
  }

  switch (_ChannelMode[channel - 1])
  {
    case TIMER_OUTPUT_COMPARE_PWM1:
    case TIMER_OUTPUT_COMPARE_PWM2: 
    {
#if defined(TIM_CC1NE)
        if (isComplementaryChannel[channel - 1]) 
        {
          TIM_CCxNCmd( _timerObj.handle.Instance, timChannel, TIM_CCxN_Enable );
          _timerObj.handle.Instance->BDTR |= TIM_MOE;   //MOE ENABLE
          TIM_Cmd( _timerObj.handle.Instance, ENABLE );
        } 
        else
#endif
        {
          TIM_CCxCmd( _timerObj.handle.Instance, timChannel, TIM_CCx_Enable );
          _timerObj.handle.Instance->BDTR |= TIM_MOE;   //MOE ENABLE
          TIM_Cmd( _timerObj.handle.Instance, ENABLE );
        }
      }
      break;
    case TIMER_OUTPUT_COMPARE_ACTIVE:
    case TIMER_OUTPUT_COMPARE_INACTIVE:
    case TIMER_OUTPUT_COMPARE_TOGGLE:
    case TIMER_OUTPUT_COMPARE_FORCED_ACTIVE:
    case TIMER_OUTPUT_COMPARE_FORCED_INACTIVE: 
    {
#if defined(TIM_CC1NE)
        if (isComplementaryChannel[channel - 1]) 
        {
          // HAL_TIMEx_OCN_Start(&(_timerObj.handle), timChannel);
          TIM_CCxNCmd( _timerObj.handle.Instance, timChannel, TIM_CCxN_Enable );
          _timerObj.handle.Instance->BDTR |= TIM_MOE;   //MOE ENABLE
          TIM_Cmd( _timerObj.handle.Instance, ENABLE );
        } 
        else
#endif
        {
          // HAL_TIM_OC_Start(&(_timerObj.handle), timChannel);
          TIM_CCxCmd( _timerObj.handle.Instance, timChannel, TIM_CCx_Enable );
          _timerObj.handle.Instance->BDTR |= TIM_MOE;   //MOE ENABLE
          TIM_Cmd( _timerObj.handle.Instance, ENABLE );
        }
      }
      break;
    case TIMER_INPUT_FREQ_DUTY_MEASUREMENT: {
        TIM_CCxCmd( _timerObj.handle.Instance, timChannel, TIM_CCx_Enable );
        // Enable 2nd associated channel
        //两个关联通道，配置为捕获通道   1 & 2  /  3 & 4        
        timAssociatedInputChannel = getAssociatedChannel(channel);
        TIM_CCxCmd( _timerObj.handle.Instance, getLLChannel(timAssociatedInputChannel), TIM_CCx_Enable );
       
        _timerObj.handle.Instance->BDTR |= TIM_MOE;   //MOE ENABLE

         TIM_Cmd( _timerObj.handle.Instance, ENABLE );
        if (callbacks[channel]) 
        {
          TIM_ClearFlag(_timerObj.handle.Instance,getIT(timAssociatedInputChannel));      
          TIM_ITConfig(_timerObj.handle.Instance,getIT(timAssociatedInputChannel),ENABLE);
        }
      }
      break;
    case TIMER_INPUT_CAPTURE_RISING:
    case TIMER_INPUT_CAPTURE_FALLING:
    case TIMER_INPUT_CAPTURE_BOTHEDGE: {

        TIM_CCxCmd( _timerObj.handle.Instance, timChannel, TIM_CCx_Enable );
        _timerObj.handle.Instance->BDTR |= TIM_MOE;   //MOE ENABLE
        TIM_Cmd( _timerObj.handle.Instance, ENABLE );
        // HAL_TIM_IC_Start(&(_timerObj.handle), timChannel);
      }
      break;
    case TIMER_OUTPUT_COMPARE:
    case TIMER_DISABLED:
       if (!(_timerObj.handle.Instance->CTLR1 & TIM_CEN) ) //if not enable 
       { 
          TIM_Cmd( _timerObj.handle.Instance, ENABLE ) ; 
       }
      break;
    case TIMER_NOT_USED:
    default :
      break;
  }
}


/**
  * @brief  Configure comparison output channels
  * @param   timerX, TIM_OCInitTypeDef,  Arduino channel [1..4] 
  * @retval None
  */
void HardwareTimer::TIM_OC_ConfigChannel_Static(TIM_TypeDef *tim, TIM_OCInitTypeDef *sConfig, uint32_t Channel)
{
   switch (Channel)
  {
    case TIM_Channel_1:
    {
      /* Configure the TIM Channel 1 in Output Compare */
      TIM_OC1Init(tim,sConfig);
      break;
    }

    case TIM_Channel_2:
    {
      /* Configure the TIM Channel 2 in Output Compare */
      TIM_OC2Init(tim,sConfig);
      break;
    }

    case TIM_Channel_3:
    {
      /* Configure the TIM Channel 3 in Output Compare */
      TIM_OC3Init(tim,sConfig);
      break;
    }

    case TIM_Channel_4:
    {
      /* Configure the TIM Channel 4 in Output Compare */
      TIM_OC4Init(tim,sConfig);
      break;
    }

    default:
      break;
  }   
}

/**
  * @brief  Configure input capture channels
  * @param   timerX, TIM_ICInitTypeDef,  Arduino channel [1..4] 
  * @retval None
  */
void HardwareTimer::TIM_IC_ConfigChannel_Static(TIM_TypeDef *tim, TIM_ICInitTypeDef *sConfig, uint32_t Channel)
{

  sConfig->TIM_Channel = Channel;

  TIM_ICInit( tim,  sConfig);

  if(Channel == TIM_Channel_1)
  {
      TIM_SetIC1Prescaler(tim, sConfig->TIM_ICPrescaler);
  }
  else if(Channel == TIM_Channel_2)
  {
      TIM_SetIC2Prescaler(tim, sConfig->TIM_ICPrescaler);
  }
  else if(Channel == TIM_Channel_3)
  {
      TIM_SetIC3Prescaler(tim, sConfig->TIM_ICPrescaler);
  }
  else
  {
      TIM_SetIC4Prescaler(tim, sConfig->TIM_ICPrescaler);
  }

}


/**
  * @brief  Retrieve prescaler from hardware register
  * @param  None
  * @retval prescaler factor
  */
uint32_t HardwareTimer::getPrescaleFactor()
{
  // Hardware register correspond to prescaler-1. Example PSC register value 0 means divided by 1
  return (TIM_GetPrescaler( _timerObj.handle.Instance )+1);
}

/**
  * @brief  Configure hardwareTimer prescaler
  * @param  prescaler factor
  * @retval None
  */
void HardwareTimer::setPrescaleFactor(uint32_t prescaler)
{
  // Hardware register correspond to prescaler-1. Example PSC register value 0 means divided by 1
  _timerObj.handle.Instance->PSC = prescaler - 1;
  updateRegistersIfNotRunning(_timerObj.handle.Instance);
}

/**
  * @brief  Retrieve overflow (rollover) value from hardware register
  * @param  format of returned value. If omitted default format is Tick
  * @retval overflow depending on format value:
  *           TICK_FORMAT:     return number of tick for overflow
  *           MICROSEC_FORMAT: return number of microsecondes for overflow
  *           HERTZ_FORMAT:    return frequency in hertz for overflow
  */
uint32_t HardwareTimer::getOverflow(TimerFormat_t format)
{
  // Hardware register correspond to period count-1. Example ARR register value 9 means period of 10 timer cycle
  uint32_t ARR_RegisterValue = _timerObj.handle.Instance->ATRLR;

  uint32_t Prescalerfactor = TIM_GetPrescaler(_timerObj.handle.Instance) + 1;
  uint32_t return_value;
  switch (format) {
    case MICROSEC_FORMAT:
      return_value = (uint32_t)(((ARR_RegisterValue + 1) * Prescalerfactor * 1000000.0) / getTimerClkFreq());
      break;
    case HERTZ_FORMAT:
      return_value = (uint32_t)(getTimerClkFreq() / ((ARR_RegisterValue + 1) * Prescalerfactor));
      break;
    case TICK_FORMAT:
    default :
      return_value = ARR_RegisterValue + 1;
      break;
  }
  return return_value;
}

/**
  * @brief  Set overflow (rollover)
  *
  *         Note that by default, the new value will not be applied
  *         immediately, but become effective at the next update event
  *         (usually the next timer overflow). See setPreloadEnable()
  *         for controlling this behaviour.
  * @param  overflow: depend on format parameter
  * @param  format of overflow parameter. If omitted default format is Tick
  *           TICK_FORMAT:     overflow is the number of tick for overflow
  *           MICROSEC_FORMAT: overflow is the number of microsecondes for overflow
  *           HERTZ_FORMAT:    overflow is the frequency in hertz for overflow
  * @retval None
  */
void HardwareTimer::setOverflow(uint32_t overflow, TimerFormat_t format)
{
  uint32_t ARR_RegisterValue;
  uint32_t PeriodTicks;
  uint32_t Prescalerfactor;
  uint32_t period_cyc;
  // Remark: Hardware register correspond to period count-1. Example ARR register value 9 means period of 10 timer cycle
  switch (format) {
    case MICROSEC_FORMAT:
      period_cyc = overflow * (getTimerClkFreq() / 1000000);
      Prescalerfactor = (period_cyc / 0x10000) + 1;
      _timerObj.handle.Instance->PSC = Prescalerfactor - 1;
      PeriodTicks = period_cyc / Prescalerfactor;
      break;
    case HERTZ_FORMAT:
      period_cyc = getTimerClkFreq() / overflow;
      Prescalerfactor = (period_cyc / 0x10000) + 1;
      _timerObj.handle.Instance->PSC = Prescalerfactor - 1;
      PeriodTicks = period_cyc / Prescalerfactor;
      break;
    case TICK_FORMAT:
    default :
      PeriodTicks = overflow;
      break;
  }

  if (PeriodTicks > 0) {
    // The register specifies the maximum value, so the period is really one tick longer
    ARR_RegisterValue = PeriodTicks - 1;
  } else {
    // But do not underflow in case a zero period was given somehow.
    ARR_RegisterValue = 0;
  }
  _timerObj.handle.Instance->ATRLR = ARR_RegisterValue;
  updateRegistersIfNotRunning(_timerObj.handle.Instance);
}

/**
  * @brief  Retrieve timer counter value
  * @param  format of returned value. If omitted default format is Tick
  * @retval overflow depending on format value:
  *           TICK_FORMAT:     return number of tick for counter
  *           MICROSEC_FORMAT: return number of microsecondes for counter
  *           HERTZ_FORMAT:    return frequency in hertz for counter
  */
uint32_t HardwareTimer::getCount(TimerFormat_t format)
{
  uint32_t CNT_RegisterValue = TIM_GetCounter(_timerObj.handle.Instance);
  uint32_t Prescalerfactor = TIM_GetPrescaler(_timerObj.handle.Instance) + 1;
  uint32_t return_value;
  switch (format) {
    case MICROSEC_FORMAT:
      return_value = (uint32_t)((CNT_RegisterValue * Prescalerfactor * 1000000.0) / getTimerClkFreq());
      break;
    case HERTZ_FORMAT:
      return_value = (uint32_t)(getTimerClkFreq() / (CNT_RegisterValue  * Prescalerfactor));
      break;
    case TICK_FORMAT:
    default :
      return_value = CNT_RegisterValue;
      break;
  }
  return return_value;
}

/**
  * @brief  Set timer counter value
  * @param  counter: depend on format parameter
  * @param  format of overflow parameter. If omitted default format is Tick
  *           TICK_FORMAT:     counter is the number of tick
  *           MICROSEC_FORMAT: counter is the number of microsecondes
  *           HERTZ_FORMAT:    counter is the frequency in hertz
  * @retval None
  */
void HardwareTimer::setCount(uint32_t counter, TimerFormat_t format)
{
  uint32_t CNT_RegisterValue;
  uint32_t Prescalerfactor = TIM_GetPrescaler(_timerObj.handle.Instance) + 1;
  switch (format) {
    case MICROSEC_FORMAT:
      CNT_RegisterValue = ((counter * (getTimerClkFreq() / 1000000)) / Prescalerfactor);
      break;
    case HERTZ_FORMAT:
      CNT_RegisterValue = (uint32_t)(getTimerClkFreq() / (counter * Prescalerfactor));
      break;
    case TICK_FORMAT:
    default :
      CNT_RegisterValue = counter;
      break;
  }
  TIM_SetCounter(_timerObj.handle.Instance, CNT_RegisterValue);
}

/**
  * @brief  Set channel mode
  * @param  channel: Arduino channel [1..4]
  * @param  mode: mode configuration for the channel (see TimerModes_t)
  * @param  pin: Arduino pin number, ex: D1, 1 or PA1
  * @retval None
  */
void HardwareTimer::setMode(uint32_t channel, TimerModes_t mode, uint32_t pin)
{
  setMode(channel, mode, digitalPinToPinName(pin));
}

/**
  * @brief  Set channel mode
  * @param  channel: Arduino channel [1..4]
  * @param  mode: mode configuration for the channel (see TimerModes_t)
  * @param  pin: pin name, ex: PB_0
  * @retval None
  */
void HardwareTimer::setMode(uint32_t channel, TimerModes_t mode, PinName pin)
{
  int timChannel = getChannel(channel);  //get arduino channel-->timer channel
  int timAssociatedInputChannel;      
  TIM_OCInitTypeDef channelOC={0};
  TIM_ICInitTypeDef channelIC={0};

  if (timChannel == -1) {
    Error_Handler();
  }

  /* Configure some default values. Maybe overwritten later */
  channelOC.TIM_OCMode = TIMER_NOT_USED;  //set default value 0xFFFF

  // channelOC.Pulse = __HAL_TIM_GET_COMPARE(&(_timerObj.handle), timChannel);  // keep same value already written in hardware register
  channelOC.TIM_Pulse =  (((timChannel) == TIM_Channel_1) ? (_timerObj.handle.Instance->CH1CVR) :\
                          ((timChannel) == TIM_Channel_2) ? (_timerObj.handle.Instance->CH2CVR) :\
                          ((timChannel) == TIM_Channel_3) ? (_timerObj.handle.Instance->CH3CVR) :\
                          (_timerObj.handle.Instance->CH4CVR));

  channelOC.TIM_OCPolarity = TIM_OCPolarity_High;
#if defined(TIM_OIS1)
  channelOC.TIM_OCIdleState = TIM_OSSIState_Disable;
#endif
#if defined(TIM_CC1NE)
  channelOC.TIM_OCNPolarity = TIM_OCNPolarity_High;
#if defined(TIM_OIS1N)
  channelOC.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
#endif
#endif
  channelIC.TIM_ICPolarity = TIM_ICPolarity_Rising;
  channelIC.TIM_ICSelection = TIM_ICSelection_DirectTI;
  channelIC.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  channelIC.TIM_ICFilter = 0;

  switch (mode) {
    case TIMER_DISABLED:
      channelOC.TIM_OCMode = TIM_OCMode_Timing;

      TIM_OC_ConfigChannel_Static(_timerObj.handle.Instance, &channelOC, timChannel);
      break;
    case TIMER_OUTPUT_COMPARE:
      /* In case of TIMER_OUTPUT_COMPARE, there is no output and thus no pin to
       * configure, and no channel. So nothing to do. For compatibility reason
       * restore TIMER_DISABLED if necessary.
       */
      if (_ChannelMode[channel - 1] != TIMER_DISABLED) {
        _ChannelMode[channel - 1] = TIMER_DISABLED;
        channelOC.TIM_OCMode = TIM_OCMode_Timing;

        TIM_OC_ConfigChannel_Static(_timerObj.handle.Instance, &channelOC, timChannel);
      }
      return;
    case TIMER_OUTPUT_COMPARE_ACTIVE:
      channelOC.TIM_OCMode = TIM_OCMode_Active;
      TIM_OC_ConfigChannel_Static(_timerObj.handle.Instance, &channelOC, timChannel);

      break;
    case TIMER_OUTPUT_COMPARE_INACTIVE:
      channelOC.TIM_OCMode = TIM_OCMode_Inactive;
      TIM_OC_ConfigChannel_Static(_timerObj.handle.Instance, &channelOC, timChannel);
      break;
    case TIMER_OUTPUT_COMPARE_TOGGLE:
      channelOC.TIM_OCMode = TIM_OCMode_Toggle;
      TIM_OC_ConfigChannel_Static(_timerObj.handle.Instance, &channelOC, timChannel);
      break;
    case TIMER_OUTPUT_COMPARE_PWM1:
      channelOC.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OC_ConfigChannel_Static(_timerObj.handle.Instance, &channelOC, timChannel);      
      break;
    case TIMER_OUTPUT_COMPARE_PWM2:
      channelOC.TIM_OCMode = TIM_OCMode_PWM2;
      TIM_OC_ConfigChannel_Static(_timerObj.handle.Instance, &channelOC, timChannel);
      break;
    
    case TIMER_OUTPUT_COMPARE_FORCED_ACTIVE:
      channelOC.TIM_OCMode = 0x0050;   //force high
      TIM_OC_ConfigChannel_Static(_timerObj.handle.Instance, &channelOC, timChannel);
      break;
    case TIMER_OUTPUT_COMPARE_FORCED_INACTIVE:
      channelOC.TIM_OCMode = 0x0040;   //force low
      TIM_OC_ConfigChannel_Static(_timerObj.handle.Instance, &channelOC, timChannel);
      break;
    case TIMER_INPUT_CAPTURE_RISING:
      channelIC.TIM_ICPolarity = TIM_ICPolarity_Rising;
      TIM_IC_ConfigChannel_Static(_timerObj.handle.Instance, &channelIC,timChannel);
      break;
    case TIMER_INPUT_CAPTURE_FALLING:
      channelIC.TIM_ICPolarity = TIM_ICPolarity_Falling;
      TIM_IC_ConfigChannel_Static(_timerObj.handle.Instance, &channelIC,timChannel);
      break;
    case TIMER_INPUT_CAPTURE_BOTHEDGE:
      channelIC.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
      TIM_IC_ConfigChannel_Static(_timerObj.handle.Instance, &channelIC,timChannel);      
      break;
    case TIMER_INPUT_FREQ_DUTY_MEASUREMENT:
      // Configure 1st channel
      channelIC.TIM_ICPolarity = TIM_ICPolarity_Rising;
      channelIC.TIM_ICSelection = TIM_ICSelection_DirectTI;
      TIM_IC_ConfigChannel_Static(_timerObj.handle.Instance, &channelIC, timChannel); 
      // // Identify and configure 2nd associated channel
      timAssociatedInputChannel = getAssociatedChannel(channel);
      _ChannelMode[timAssociatedInputChannel - 1] = mode;
      channelIC.TIM_ICPolarity = TIM_ICPolarity_Falling;
      channelIC.TIM_ICSelection = TIM_ICSelection_IndirectTI;
      TIM_IC_ConfigChannel_Static(_timerObj.handle.Instance, &channelIC, timChannel);    
      break;
    default:
      break;
  }
  // Save channel selected mode to object attribute
  _ChannelMode[channel - 1] = mode;
  if (pin != NC) 
  {
    if ((int)getTimerChannel(pin) == timChannel) 
    {
      /* Configure PWM GPIO pins */
      pinmap_pinout(pin, PinMap_TIM);
      if ((mode == TIMER_INPUT_CAPTURE_RISING) || (mode == TIMER_INPUT_CAPTURE_FALLING) \
          || (mode == TIMER_INPUT_CAPTURE_BOTHEDGE) || (mode == TIMER_INPUT_FREQ_DUTY_MEASUREMENT)) 
      {
          //input alternate function must configure GPIO in input mode
          pinMode(pinNametoDigitalPin(pin), INPUT);  //set input
      }
    } 
    else
     {
      // Pin doesn't match with timer output channels
      Error_Handler();
    }

#if defined(TIM_CC1NE)
     isComplementaryChannel[channel - 1] = CH_PIN_INVERTED(pinmap_function(pin, PinMap_TIM)); //(x>>20)&0x1 
#endif
  }
}

/**
  * @brief  Retrieves channel mode configured
  * @param  channel: Arduino channel [1..4]
  * @retval returns configured mode
  */
TimerModes_t HardwareTimer::getMode(uint32_t channel)
{
  if ((1 <= channel) && (channel <= TIMER_CHANNELS)) {
    return _ChannelMode[channel - 1];
  } else {
    return TIMER_DISABLED;
  }
}

/**
  * @brief  Enable or disable preloading for overflow value
  *         When disabled, changes to the overflow value take effect
  *         immediately. When enabled (the default), the value takes
  *         effect only at the next update event (typically the next
  *         overflow).
  *
  *         Note that the capture/compare register has its own preload
  *         enable bit, which is independent and enabled in PWM modes
  *         and disabled otherwise. If you need more control of that
  *         bit, you can use the HAL functions directly.
  * @param  value: true to enable preloading, false to disable
  * @retval None
  */
void HardwareTimer::setPreloadEnable(bool value)
{
  if (value) {
    TIM_ARRPreloadConfig( _timerObj.handle.Instance, ENABLE );

  } else {
    TIM_ARRPreloadConfig( _timerObj.handle.Instance, DISABLE );
  }
}

/**
  * @brief  Set channel Capture/Compare register
  * @param  channel: Arduino channel [1..4]
  * @param  compare: compare value depending on format
  * @param  format of compare parameter. If omitted default format is Tick
  *           TICK_FORMAT:     compare is the number of tick
  *           MICROSEC_FORMAT: compare is the number of microsecondes
  *           HERTZ_FORMAT:    compare is the frequency in hertz
  * @retval None
  */
void HardwareTimer::setCaptureCompare(uint32_t channel, uint32_t compare, TimerCompareFormat_t format)
{
  int timChannel = getChannel(channel);
  uint32_t Prescalerfactor = TIM_GetPrescaler(_timerObj.handle.Instance) + 1;
  uint32_t CCR_RegisterValue;

  if (timChannel == -1) {
    Error_Handler();
  }
  switch (format) {
    case MICROSEC_COMPARE_FORMAT:
      CCR_RegisterValue = ((compare * (getTimerClkFreq() / 1000000)) / Prescalerfactor);
      break;
    case HERTZ_COMPARE_FORMAT:
      CCR_RegisterValue = getTimerClkFreq() / (compare * Prescalerfactor);
      break;
    // As per Reference Manual PWM reach 100% with CCRx value strictly greater than ARR (So ARR+1 in our case)
    case PERCENT_COMPARE_FORMAT:
      CCR_RegisterValue = ((_timerObj.handle.Instance->ATRLR  + 1) * compare) / 100;
      break;
    case RESOLUTION_1B_COMPARE_FORMAT:
    case RESOLUTION_2B_COMPARE_FORMAT:
    case RESOLUTION_3B_COMPARE_FORMAT:
    case RESOLUTION_4B_COMPARE_FORMAT:
    case RESOLUTION_5B_COMPARE_FORMAT:
    case RESOLUTION_6B_COMPARE_FORMAT:
    case RESOLUTION_7B_COMPARE_FORMAT:
    case RESOLUTION_8B_COMPARE_FORMAT:
    case RESOLUTION_9B_COMPARE_FORMAT:
    case RESOLUTION_10B_COMPARE_FORMAT:
    case RESOLUTION_11B_COMPARE_FORMAT:
    case RESOLUTION_12B_COMPARE_FORMAT:
    case RESOLUTION_13B_COMPARE_FORMAT:
    case RESOLUTION_14B_COMPARE_FORMAT:
    case RESOLUTION_15B_COMPARE_FORMAT:
    case RESOLUTION_16B_COMPARE_FORMAT:
      CCR_RegisterValue = (( _timerObj.handle.Instance->ATRLR + 1 ) * compare ) / ( (1 << format) -1 );
      break;
    case TICK_COMPARE_FORMAT:
    default :
      CCR_RegisterValue = compare;
      break;
  }

  // Special case when ARR is set to the max value, it is not possible to set CCRx to ARR+1 to reach 100%
  // Then set CCRx to max value. PWM is then 1/0xFFFF = 99.998..%
  if( (( _timerObj.handle.Instance->ATRLR ) == MAX_RELOAD) && (CCR_RegisterValue == MAX_RELOAD + 1))
  {
    CCR_RegisterValue = MAX_RELOAD;
  }

    switch (timChannel)
    {
    case TIM_Channel_1:
      TIM_SetCompare1( _timerObj.handle.Instance, CCR_RegisterValue );
      break;
    case TIM_Channel_2:
      TIM_SetCompare2( _timerObj.handle.Instance, CCR_RegisterValue );
      break;
    case TIM_Channel_3:
      TIM_SetCompare3( _timerObj.handle.Instance, CCR_RegisterValue );
      break;
    case TIM_Channel_4:
      TIM_SetCompare4( _timerObj.handle.Instance, CCR_RegisterValue );    
      break;
    default:
      break;
    }
    updateRegistersIfNotRunning(_timerObj.handle.Instance);
}

/**
  * @brief  Retrieve Capture/Compare value
  * @param  channel: Arduino channel [1..4]
  * @param  format of return value. If omitted default format is Tick
  *           TICK_FORMAT:     return value is the number of tick for Capture/Compare value
  *           MICROSEC_FORMAT: return value is the number of microsecondes for Capture/Compare value
  *           HERTZ_FORMAT:    return value is the frequency in hertz for Capture/Compare value
  * @retval Capture/Compare value
  */
uint32_t HardwareTimer::getCaptureCompare(uint32_t channel,  TimerCompareFormat_t format)
{

  int timChannel;
  uint32_t return_value;
  uint32_t CCR_RegisterValue;
  uint32_t Prescalerfactor;

  timChannel = getChannel(channel);
  Prescalerfactor = TIM_GetPrescaler(_timerObj.handle.Instance) + 1;

  if (timChannel == -1) {
    Error_Handler();
  }
  switch(timChannel)
  {
      case TIM_Channel_1:
           CCR_RegisterValue = (_timerObj.handle.Instance->CH1CVR);
           break;
      case TIM_Channel_2:
           CCR_RegisterValue = (_timerObj.handle.Instance->CH2CVR);
           break;
      case TIM_Channel_3:
           CCR_RegisterValue = (_timerObj.handle.Instance->CH3CVR);
           break;
      case TIM_Channel_4:
           CCR_RegisterValue = (_timerObj.handle.Instance->CH4CVR);
           break;
      default:
            break;
  }

  switch (format) {
    case MICROSEC_COMPARE_FORMAT:
      return_value = (uint32_t)((CCR_RegisterValue * Prescalerfactor * 1000000.0) / getTimerClkFreq());
      break;
    case HERTZ_COMPARE_FORMAT:
      return_value = (uint32_t)(getTimerClkFreq() / (CCR_RegisterValue  * Prescalerfactor));
      break;
    case PERCENT_COMPARE_FORMAT:
      return_value = (CCR_RegisterValue * 100) / _timerObj.handle.Instance->ATRLR;
      break;
    case RESOLUTION_1B_COMPARE_FORMAT:
    case RESOLUTION_2B_COMPARE_FORMAT:
    case RESOLUTION_3B_COMPARE_FORMAT:
    case RESOLUTION_4B_COMPARE_FORMAT:
    case RESOLUTION_5B_COMPARE_FORMAT:
    case RESOLUTION_6B_COMPARE_FORMAT:
    case RESOLUTION_7B_COMPARE_FORMAT:
    case RESOLUTION_8B_COMPARE_FORMAT:
    case RESOLUTION_9B_COMPARE_FORMAT:
    case RESOLUTION_10B_COMPARE_FORMAT:
    case RESOLUTION_11B_COMPARE_FORMAT:
    case RESOLUTION_12B_COMPARE_FORMAT:
    case RESOLUTION_13B_COMPARE_FORMAT:
    case RESOLUTION_14B_COMPARE_FORMAT:
    case RESOLUTION_15B_COMPARE_FORMAT:
    case RESOLUTION_16B_COMPARE_FORMAT:
      return_value = (CCR_RegisterValue * ((1 << format) - 1))  / _timerObj.handle.Instance->ATRLR ;
      break;
    case TICK_COMPARE_FORMAT:
    default :
      return_value = CCR_RegisterValue;
      break;
  }
  return return_value;
}

/**
  * @param  channel: Arduino channel [1..4]
  * @param  pin: Arduino pin number, ex D1, 1 or PA1
  * @param  frequency: PWM frequency expressed in hertz
  * @param  dutycycle: PWM dutycycle expressed in percentage
  * @param  PeriodCallback: timer period callback (timer rollover upon update event)
  * @param  CompareCallback: timer compare callback
  * @retval None
  */
void HardwareTimer::setPWM(uint32_t channel, uint32_t pin, uint32_t frequency, uint32_t dutycycle, callback_function_t PeriodCallback, callback_function_t CompareCallback)
{
  setPWM(channel, digitalPinToPinName(pin), frequency, dutycycle, PeriodCallback, CompareCallback);
}

/**
  * @brief  All in one function to configure PWM
  * @param  channel: Arduino channel [1..4]
  * @param  pin: pin name, ex PB_0
  * @param  frequency: PWM frequency expressed in hertz
  * @param  dutycycle: PWM dutycycle expressed in percentage
  * @param  PeriodCallback: timer period callback (timer rollover upon update event)
  * @param  CompareCallback: timer compare callback
  * @retval None
  */
void HardwareTimer::setPWM(uint32_t channel, PinName pin, uint32_t frequency, uint32_t dutycycle, callback_function_t PeriodCallback, callback_function_t CompareCallback)
{
  setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
  setOverflow(frequency, HERTZ_FORMAT);
  setCaptureCompare(channel, dutycycle, PERCENT_COMPARE_FORMAT);
  if (PeriodCallback) {
    attachInterrupt(PeriodCallback);
  }
  if (CompareCallback) {
    attachInterrupt(channel, CompareCallback);
  }
  resume();
}

/**
  * @brief  Set the priority of the interrupt
  * @note   Must be call before resume()
  * @param  preemptPriority: the pre-emption priority for the IRQn channel
  * @param  subPriority: the subpriority level for the IRQ channel.
  * @retval None
  */
void HardwareTimer::setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority)
{
  // Set Update interrupt priority for immediate use
   NVIC_SetPriority( getTimerUpIrq(_timerObj.handle.Instance), ((preemptPriority & 0x1) << 7 ) | subPriority );
  // Set Capture/Compare interrupt priority if timer provides a unique IRQ
  if (getTimerCCIrq(_timerObj.handle.Instance) != getTimerUpIrq(_timerObj.handle.Instance)) {
    NVIC_SetPriority( getTimerCCIrq(_timerObj.handle.Instance), ((preemptPriority & 0x1) << 7 ) | subPriority );
  }
  // Store priority for use if timer is re-initialized
  _timerObj.preemptPriority = preemptPriority;
  _timerObj.subPriority = subPriority;
}

/**
  * @brief  Attach interrupt callback on update (rollover) event
  * @param  callback: interrupt callback
  * @retval None
  */
void HardwareTimer::attachInterrupt(callback_function_t callback)
{
  if (callbacks[0]) {
    // Callback previously configured : do not clear neither enable IT, it is just a change of callback
    callbacks[0] = callback;
  } else {
    callbacks[0] = callback;
    if (callback) {
      // Clear flag before enabling IT
      TIM_ClearFlag(_timerObj.handle.Instance,TIM_FLAG_Update);
      TIM_ITConfig(_timerObj.handle.Instance,TIM_IT_Update,ENABLE);
    }
  }
}

/**
  * @brief  Detach interrupt callback on update (rollover) event
  * @retval None
  */
void HardwareTimer::detachInterrupt()
{
  // Disable update interrupt and clear callback
  TIM_ITConfig(_timerObj.handle.Instance,TIM_IT_Update,DISABLE);
  callbacks[0] = NULL;
}

/**
  * @brief  Attach interrupt callback on Capture/Compare event
  * @param  channel: Arduino channel [1..4]
  * @param  callback: interrupt callback
  * @retval None
  */
void HardwareTimer::attachInterrupt(uint32_t channel, callback_function_t callback)
{
  int interrupt = getIT(channel);
  if (interrupt == -1) {
    Error_Handler();
  }

  if ((channel == 0) || (channel > (TIMER_CHANNELS + 1))) {
    Error_Handler();  // only channel 1..4 have an interrupt
  }
  if (callbacks[channel]) {
    // Callback previously configured : do not clear neither enable IT, it is just a change of callback
    callbacks[channel] = callback;
  } else {
    callbacks[channel] = callback;
    if (callback) {

      // Clear flag before enabling IT
      TIM_ClearFlag( _timerObj.handle.Instance, interrupt);
      // Enable interrupt corresponding to channel, only if callback is valid
      TIM_ITConfig(_timerObj.handle.Instance,interrupt,ENABLE);
    }
  }
}

/**
  * @brief  Detach interrupt callback on Capture/Compare event
  * @param  channel: Arduino channel [1..4]
  * @retval None
  */
void HardwareTimer::detachInterrupt(uint32_t channel)
{
  int interrupt = getIT(channel);
  if (interrupt == -1) {
    Error_Handler();
  }

  if ((channel == 0) || (channel > (TIMER_CHANNELS + 1))) {
    Error_Handler();  // only channel 1..4 have an interrupt
  }

  // Disable interrupt corresponding to channel and clear callback
  TIM_ITConfig(_timerObj.handle.Instance,interrupt,DISABLE);
  callbacks[channel] = NULL;
}

/**
  * @brief  Checks if there's an interrupt callback attached on Rollover event
  * @retval returns true if a timer rollover interrupt has already been set
  */
bool HardwareTimer::hasInterrupt()
{
  return callbacks[0] != NULL;
}

/**
  * @brief  Checks if there's an interrupt callback attached on Capture/Compare event
  * @param  channel: Arduino channel [1..4]
  * @retval returns true if a channel compare match interrupt has already been set
  */
bool HardwareTimer::hasInterrupt(uint32_t channel)
{
  if ((channel == 0) || (channel > (TIMER_CHANNELS + 1))) {
    Error_Handler();  // only channel 1..4 have an interrupt
  }
  return callbacks[channel] != NULL;
}

/**
  * @brief  Generate an update event to force all registers (Autoreload, prescaler, compare) to be taken into account
  * @note   @note Refresh() can only be called after timer has been initialized,
            either by calling setup() function or thanks to constructor with TIM instance parameter.
  *         It is useful while timer is running after some registers update
  * @retval None
  */
void HardwareTimer::refresh()
{
  TIM_GenerateEvent( _timerObj.handle.Instance,TIM_EventSource_Update );
}

/**
  * @brief  Return the timer object handle object for more advanced setup
  * @note   Using this function and editing the Timer handle is at own risk! No support will
  *         be provided whatsoever if the HardwareTimer does not work as expected when editing
  *         the handle using the HAL functionality or other custom coding.
  * @retval TIM_HandleTypeDef address
  */
TIM_HandleTypeDef *HardwareTimer::getHandle()
{
  return &_timerObj.handle;
}

/**
  * @brief  Generic Update (rollover) callback which will call user callback
  * @param  htim: HAL timer handle
  * @retval None
  */
void HardwareTimer::updateCallback(TIM_HandleTypeDef *htim)
{

  if (!htim) {
    Error_Handler();
  }

  if(TIM_GetITStatus(htim->Instance, TIM_IT_Update) && (htim->Instance->DMAINTENR & TIM_IT_Update))
  {

    timerObj_t *obj = get_timer_obj(htim);
    HardwareTimer *HT = (HardwareTimer *)(obj->__this);

    if (HT->callbacks[0]) {
      HT->callbacks[0]();
    }
    TIM_ClearITPendingBit(htim->Instance, TIM_IT_Update);
  }
}

/**
  * @brief  Generic Capture and Compare callback which will call user callback
  * @param  htim: HAL timer handle
  * @retval None
  */
void HardwareTimer::captureCompareCallback(TIM_HandleTypeDef *htim)
{
  if (!htim) {
    Error_Handler();
  }

 uint32_t channel ;

if( (htim->Instance->DMAINTENR & TIM_IT_CC1) || (htim->Instance->DMAINTENR & TIM_IT_CC2) \
 || (htim->Instance->DMAINTENR & TIM_IT_CC3) || (htim->Instance->DMAINTENR & TIM_IT_CC4) )
{
 if( TIM_GetITStatus(htim->Instance, TIM_IT_CC1) ) 
 {
    channel = 1;
    TIM_ClearITPendingBit( htim->Instance, TIM_IT_CC1);
 }
 else if(TIM_GetITStatus(htim->Instance, TIM_IT_CC2) )
 {
    channel = 2;
    TIM_ClearITPendingBit( htim->Instance, TIM_IT_CC2);
 }
 else if(TIM_GetITStatus(htim->Instance, TIM_IT_CC3) )
 {
    channel = 3;
    TIM_ClearITPendingBit( htim->Instance, TIM_IT_CC3);
 }
 else if(TIM_GetITStatus(htim->Instance, TIM_IT_CC4) )
 {
    channel = 4;
    TIM_ClearITPendingBit( htim->Instance, TIM_IT_CC4);
 }

  timerObj_t *obj = get_timer_obj(htim);
  HardwareTimer *HT = (HardwareTimer *)(obj->__this);
  if (HT->callbacks[channel]) {
    HT->callbacks[channel]();
  }
}
}

/**
  * @brief  Check whether HardwareTimer is running (paused or resumed).
  * @retval return true if the HardwareTimer is running
  */
bool HardwareTimer::isRunning()
{
  // return LL_TIM_IsEnabledCounter(_timerObj.handle.Instance);
  return  (((_timerObj.handle.Instance->CTLR1 & TIM_CEN) == TIM_CEN)? 1UL : 0UL) ;
}

/**
  * @brief  Check whether channel is running (paused or resumed).
  * @param  channel: Arduino channel [1..4]
  * @retval return true if HardwareTimer is running and the channel is enabled
  */
bool HardwareTimer::isRunningChannel(uint32_t channel)
{
  int LLChannel = getLLChannel(channel);
  int interrupt = getIT(channel);
  bool ret;

  if (LLChannel == -1) {
    Error_Handler();
  }

  if (interrupt == -1) {
    Error_Handler();
  }

  // channel is running if: timer is running, and either output channel is
  // enabled or interrupt is set
  ret =  ((_timerObj.handle.Instance->CCER & LLChannel) == LLChannel ? 1UL : 0UL) \
        || ((_timerObj.handle.Instance->DMAINTENR & interrupt) == interrupt ? 1UL : 0UL) ;  

  return (isRunning() && ret);
}

/**
  * @brief  Take into account registers update immediately if timer is not running,
  *         (independently from Preload setting)
  * @param  TIMx Timer instance
  * @retval None
  */
void HardwareTimer::updateRegistersIfNotRunning(TIM_TypeDef *TIMx)
{
  if (!isRunning()) {
    if (_timerObj.handle.Instance->DMAINTENR & TIM_IT_Update) 
    {
      // prevent Interrupt generation from refresh()
      TIM_ITConfig(_timerObj.handle.Instance, TIM_IT_Update, DISABLE);
      refresh( );
      TIM_ClearFlag(_timerObj.handle.Instance, TIM_IT_Update); 
      TIM_ITConfig(_timerObj.handle.Instance, TIM_IT_Update, ENABLE);

    } else {
      refresh();
    }
  }
}

/**
  * @brief  HardwareTimer destructor
  * @retval None
  */
HardwareTimer::~HardwareTimer()
{
  uint32_t index = get_timer_index(_timerObj.handle.Instance);
  disableTimerClock(&(_timerObj.handle));
  HardwareTimer_Handle[index] = NULL;
  _timerObj.__this = NULL;
}

/**
  * @brief  return timer index from timer handle
  * @param  htim : one of the defined timer
  * @retval timer index
  */
timer_index_t get_timer_index(TIM_TypeDef *instance)
{
  timer_index_t index = UNKNOWN_TIMER;

#if defined(TIM1_BASE)
  if (instance == TIM1) {
    index = TIMER1_INDEX;
  }
#endif
#if defined(TIM2_BASE)
  if (instance == TIM2) {
    index = TIMER2_INDEX;
  }
#endif
#if defined(TIM3_BASE)
  if (instance == TIM3) {
    index = TIMER3_INDEX;
  }
#endif
#if defined(TIM4_BASE)
  if (instance == TIM4) {
    index = TIMER4_INDEX;
  }
#endif
#if defined(TIM5_BASE)
  if (instance == TIM5) {
    index = TIMER5_INDEX;
  }
#endif
#if defined(TIM6_BASE)
  if (instance == TIM6) {
    index = TIMER6_INDEX;
  }
#endif
#if defined(TIM7_BASE)
  if (instance == TIM7) {
    index = TIMER7_INDEX;
  }
#endif
#if defined(TIM8_BASE)
  if (instance == TIM8) {
    index = TIMER8_INDEX;
  }
#endif
#if defined(TIM9_BASE)
  if (instance == TIM9) {
    index = TIMER9_INDEX;
  }
#endif
#if defined(TIM10_BASE)
  if (instance == TIM10) {
    index = TIMER10_INDEX;
  }
#endif

  return index;
}

/**
  * @brief  This function return the timer clock frequency.
  * @param  None
  * @retval frequency in Hz
  */
uint32_t HardwareTimer::getTimerClkFreq()
{
  RCC_ClocksTypeDef     RCC_ClocksStatus={};
  uint32_t              uwTimclock = 0U, uwAPBxPrescaler = 0U;

  /* Get clock configuration */
  RCC_GetClocksFreq(&RCC_ClocksStatus);

#if !defined(CH32V00x) && !defined(CH32X035) && !defined(CH32VM00X)
  switch (getTimerClkSrc(_timerObj.handle.Instance)) 
  {
    case 1:
      uwAPBxPrescaler = (RCC->CFGR0 & RCC_PPRE1) >> 8;
      uwTimclock = RCC_ClocksStatus.PCLK1_Frequency;
      break;
    case 2:
      uwAPBxPrescaler = (RCC->CFGR0 & RCC_PPRE2) >> 11;
      uwTimclock = RCC_ClocksStatus.PCLK2_Frequency;
      break;
    default:
    case 0: // Unknown timer clock source
      Error_Handler();
      break;
  }

  switch(uwAPBxPrescaler & 0x7)
  {
     case 0x4:
          uwAPBxPrescaler = 2;
          break;
     case 0x5:
          uwAPBxPrescaler = 4;
          break;
     case 0x6:
          uwAPBxPrescaler = 8;
          break;
     case 0x7:
          uwAPBxPrescaler = 16;
          break;
    default:
          uwAPBxPrescaler = 1;
          break;         
  } 

#else //CH32V003 and CH32X035 are equal to AHB CLOCK
      uwAPBxPrescaler = 1;
      uwTimclock = RCC_ClocksStatus.HCLK_Frequency;

#endif

    switch (uwAPBxPrescaler) 
    {
      default:
      case 1:
        uwTimclock*=1;
        break;
      case 2:
      case 4:
      case 8:
      case 16:
        uwTimclock *= 2;
        break;
    }

  return uwTimclock;
}

/**
  * @brief  This function will reset the timer
  * @param  None
  * @retval None
  */
void HardwareTimer::timerHandleDeinit()
{
    TIM_Cmd(_timerObj.handle.Instance, DISABLE);
    TIM_DeInit(_timerObj.handle.Instance);
}





/******************************************************************************/
/*                            TIMx IRQ HANDLER                                */
/******************************************************************************/
extern "C" {

#if defined(TIM1_BASE)
  /**
    * @brief  TIM1 IRQHandler 
    * @param  None
    * @retval None
    */
  void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
  void TIM1_UP_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER1_INDEX]) {
      HardwareTimer::updateCallback(&HardwareTimer_Handle[TIMER1_INDEX]->handle);
    }
  }
  void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
  void TIM1_CC_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER1_INDEX]) {
      HardwareTimer::captureCompareCallback(&HardwareTimer_Handle[TIMER1_INDEX]->handle);
    }
  }
#endif //TIM1_BASE

#if defined(TIM2_BASE)

  #if defined(CH32X035)
     /**
    * @brief  TIM1 IRQHandler 
    * @param  None
    * @retval None
    */
  void TIM2_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
  void TIM2_UP_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER2_INDEX]) {
      HardwareTimer::updateCallback(&HardwareTimer_Handle[TIMER2_INDEX]->handle);
    }
  }

  void TIM2_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
  void TIM2_CC_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER2_INDEX]) {
      HardwareTimer::captureCompareCallback(&HardwareTimer_Handle[TIMER2_INDEX]->handle);
    }
  }

  #else
  /**
    * @brief  TIM2 IRQHandler
    * @param  None
    * @retval None
    */
  void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
  void TIM2_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER2_INDEX]) 
    {
      HardwareTimer::updateCallback(&HardwareTimer_Handle[TIMER2_INDEX]->handle);
      HardwareTimer::captureCompareCallback(&HardwareTimer_Handle[TIMER2_INDEX]->handle);
    }
  }
  #endif

#endif //TIM2_BASE

#if defined(TIM3_BASE)
  /**
    * @brief  TIM3 IRQHandler
    * @param  None
    * @retval None
    */
  void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))); 
  void TIM3_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER3_INDEX]) {
      HardwareTimer::updateCallback(&HardwareTimer_Handle[TIMER3_INDEX]->handle);
      HardwareTimer::captureCompareCallback(&HardwareTimer_Handle[TIMER3_INDEX]->handle);
    }
  }
#endif //TIM3_BASE

#if defined(TIM4_BASE)
  /**
    * @brief  TIM4 IRQHandler
    * @param  None
    * @retval None
    */
  void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))); 
  void TIM4_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER4_INDEX]) {
       HardwareTimer::updateCallback(&HardwareTimer_Handle[TIMER4_INDEX]->handle);
       HardwareTimer::captureCompareCallback(&HardwareTimer_Handle[TIMER4_INDEX]->handle);
    }
  }
#endif //TIM4_BASE

#if defined(TIM5_BASE)
  /**
    * @brief  TIM5 IRQHandler
    * @param  None
    * @retval None
    */
  void TIM5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))); 
  void TIM5_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER5_INDEX]) {
       HardwareTimer::updateCallback(&HardwareTimer_Handle[TIMER5_INDEX]->handle);
       HardwareTimer::captureCompareCallback(&HardwareTimer_Handle[TIMER5_INDEX]->handle);
    }
  }
#endif //TIM5_BASE

#if defined(TIM6_BASE)
  /**
    * @brief  TIM6 IRQHandler
    * @param  None
    * @retval None
    */
  void TIM6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))); 
  void TIM6_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER6_INDEX]) {
       HardwareTimer::updateCallback(&HardwareTimer_Handle[TIMER6_INDEX]->handle);
       HardwareTimer::captureCompareCallback(&HardwareTimer_Handle[TIMER6_INDEX]->handle);
    }
  }
#endif //TIM6_BASE

#if defined(TIM7_BASE)
  /**
    * @brief  TIM7 IRQHandler
    * @param  None
    * @retval None
    */
  void TIM7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))); 
  void TIM7_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER7_INDEX]) {
       HardwareTimer::updateCallback(&HardwareTimer_Handle[TIMER7_INDEX]->handle);
       HardwareTimer::captureCompareCallback(&HardwareTimer_Handle[TIMER7_INDEX]->handle);
    }
  }
#endif //TIM7_BASE

#if defined(TIM8_BASE)
  /**
    * @brief  TIM8 IRQHandler
    * @param  None
    * @retval None
    */
  void TIM8_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))); 
  void TIM8_UP_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER8_INDEX]) {
       HardwareTimer::updateCallback(&HardwareTimer_Handle[TIMER8_INDEX]->handle);
    }
  }
  void TIM8_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))); 
  void TIM8_CC_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER8_INDEX]) {
      HardwareTimer::captureCompareCallback(&HardwareTimer_Handle[TIMER8_INDEX]->handle);
    }
  }
#endif //TIM8_BASE

#if defined(TIM9_BASE)
  /**
    * @brief  TIM9 IRQHandler
    * @param  None
    * @retval None
    */
  void TIM9_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));  
  void TIM9_UP_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER9_INDEX]) {
       HardwareTimer::updateCallback(&HardwareTimer_Handle[TIMER9_INDEX]->handle);
    }
  }
  void TIM9_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));    
  void TIM9_CC_IRQHandler(void)
  {
    if(HardwareTimer_Handle[TIMER9_INDEX]){
        HardwareTimer::captureCompareCallback(&HardwareTimer_Handle[TIMER9_INDEX]->handle);
    }
  }
#endif //TIM9_BASE

#if defined(TIM10_BASE)
  /**
    * @brief  TIM10 IRQHandler
    * @param  None
    * @retval None
    */
  void TIM10_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));   
  void TIM10_UP_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER10_INDEX]) {
       HardwareTimer::updateCallback(&HardwareTimer_Handle[TIMER10_INDEX]->handle);
    }
  }
  void TIM10_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));  
  void TIM10_CC_IRQHandler(void)
  {
    if (HardwareTimer_Handle[TIMER10_INDEX]){
       HardwareTimer::captureCompareCallback(&HardwareTimer_Handle[TIMER10_INDEX]->handle);
    }
  }
#endif //TIM10_BASE
}

#endif // TIM_MODULE_ENABLED && !TIM_MODULE_ONLY
