
#include "interrupt.h"
#include "ch32yyxx_exti.h"

#if !defined(EXTI_MODULE_DISABLED)

/*As we can have only one interrupt/pin id, don't need to get the port info*/
typedef struct {
  IRQn_Type irqnb;
  void (*callback)(void);
} gpio_irq_conf_str;

/* Private_Defines */
#ifdef CH32V00x
#define NB_EXTI   (8) 

#elif defined(CH32X035)
#define NB_EXTI   (26)

#else
#define NB_EXTI   (16)
#endif


/* Private Variables */
static gpio_irq_conf_str gpio_irq_conf[NB_EXTI] = {
#if defined (CH32V00x)
  {.irqnb = EXTI7_0_IRQn,   .callback = NULL}, //GPIO_PIN_0
  {.irqnb = EXTI7_0_IRQn,   .callback = NULL}, //GPIO_PIN_1
  {.irqnb = EXTI7_0_IRQn,   .callback = NULL}, //GPIO_PIN_2
  {.irqnb = EXTI7_0_IRQn,   .callback = NULL}, //GPIO_PIN_3
  {.irqnb = EXTI7_0_IRQn,   .callback = NULL}, //GPIO_PIN_4
  {.irqnb = EXTI7_0_IRQn,   .callback = NULL}, //GPIO_PIN_5
  {.irqnb = EXTI7_0_IRQn,   .callback = NULL}, //GPIO_PIN_6
  {.irqnb = EXTI7_0_IRQn,   .callback = NULL}  //GPIO_PIN_7

#elif defined(CH32X035)
  
  {.irqnb = EXTI7_0_IRQn,    .callback = NULL}, //GPIO_PIN_0
  {.irqnb = EXTI7_0_IRQn,    .callback = NULL}, //GPIO_PIN_1
  {.irqnb = EXTI7_0_IRQn,    .callback = NULL}, //GPIO_PIN_2
  {.irqnb = EXTI7_0_IRQn,    .callback = NULL}, //GPIO_PIN_3
  {.irqnb = EXTI7_0_IRQn,    .callback = NULL}, //GPIO_PIN_4
  {.irqnb = EXTI7_0_IRQn,    .callback = NULL}, //GPIO_PIN_5
  {.irqnb = EXTI7_0_IRQn,    .callback = NULL}, //GPIO_PIN_6
  {.irqnb = EXTI7_0_IRQn,    .callback = NULL},  //GPIO_PIN_7
  {.irqnb = EXTI15_8_IRQn,   .callback = NULL}, //GPIO_PIN_8
  {.irqnb = EXTI15_8_IRQn,   .callback = NULL}, //GPIO_PIN_9
  {.irqnb = EXTI15_8_IRQn,   .callback = NULL}, //GPIO_PIN_10
  {.irqnb = EXTI15_8_IRQn,   .callback = NULL}, //GPIO_PIN_11
  {.irqnb = EXTI15_8_IRQn,   .callback = NULL}, //GPIO_PIN_12
  {.irqnb = EXTI15_8_IRQn,   .callback = NULL}, //GPIO_PIN_13
  {.irqnb = EXTI15_8_IRQn,   .callback = NULL}, //GPIO_PIN_14
  {.irqnb = EXTI15_8_IRQn,   .callback = NULL}, //GPIO_PIN_15
  {.irqnb = EXTI25_16_IRQn,  .callback = NULL}, //GPIO_PIN_16
  {.irqnb = EXTI25_16_IRQn,  .callback = NULL}, //GPIO_PIN_17
  {.irqnb = EXTI25_16_IRQn,  .callback = NULL}, //GPIO_PIN_18
  {.irqnb = EXTI25_16_IRQn,  .callback = NULL}, //GPIO_PIN_19
  {.irqnb = EXTI25_16_IRQn,  .callback = NULL}, //GPIO_PIN_20
  {.irqnb = EXTI25_16_IRQn,  .callback = NULL}, //GPIO_PIN_21
  {.irqnb = EXTI25_16_IRQn,  .callback = NULL}, //GPIO_PIN_22
  {.irqnb = EXTI25_16_IRQn,  .callback = NULL}, //GPIO_PIN_23
  {.irqnb = EXTI25_16_IRQn,  .callback = NULL}, //GPIO_PIN_24
  {.irqnb = EXTI25_16_IRQn,  .callback = NULL}  //GPIO_PIN_25

#else
  {.irqnb = EXTI0_IRQn,     .callback = NULL}, //GPIO_PIN_0
  {.irqnb = EXTI1_IRQn,     .callback = NULL}, //GPIO_PIN_1
  {.irqnb = EXTI2_IRQn,     .callback = NULL}, //GPIO_PIN_2
  {.irqnb = EXTI3_IRQn,     .callback = NULL}, //GPIO_PIN_3
  {.irqnb = EXTI4_IRQn,     .callback = NULL}, //GPIO_PIN_4
  {.irqnb = EXTI9_5_IRQn,   .callback = NULL}, //GPIO_PIN_5
  {.irqnb = EXTI9_5_IRQn,   .callback = NULL}, //GPIO_PIN_6
  {.irqnb = EXTI9_5_IRQn,   .callback = NULL}, //GPIO_PIN_7
  {.irqnb = EXTI9_5_IRQn,   .callback = NULL}, //GPIO_PIN_8
  {.irqnb = EXTI9_5_IRQn,   .callback = NULL}

  #ifndef CH32V10x  
  , //GPIO_PIN_9
  {.irqnb = EXTI15_10_IRQn, .callback = NULL}, //GPIO_PIN_10
  {.irqnb = EXTI15_10_IRQn, .callback = NULL}, //GPIO_PIN_11
  {.irqnb = EXTI15_10_IRQn, .callback = NULL}, //GPIO_PIN_12
  {.irqnb = EXTI15_10_IRQn, .callback = NULL}, //GPIO_PIN_13
  {.irqnb = EXTI15_10_IRQn, .callback = NULL}, //GPIO_PIN_14
  {.irqnb = EXTI15_10_IRQn, .callback = NULL}  //GPIO_PIN_15
  #endif
#endif
};

#if defined(CH32X035)
static const uint32_t exti_lines[NB_EXTI] = {
  EXTI_Line0,  EXTI_Line1,  EXTI_Line2,  EXTI_Line3,
  EXTI_Line4,  EXTI_Line5,  EXTI_Line6,  EXTI_Line7, 
  EXTI_Line8,  EXTI_Line9,  EXTI_Line10, EXTI_Line11,
  EXTI_Line12, EXTI_Line13, EXTI_Line14, EXTI_Line15,
  EXTI_Line16, EXTI_Line17, EXTI_Line19, EXTI_Line19,
  EXTI_Line20, EXTI_Line21, EXTI_Line22, EXTI_Line23,
  EXTI_Line24, EXTI_Line25  
};

#else

static const uint32_t exti_lines[NB_EXTI] = {
  EXTI_Line0,  EXTI_Line1,  EXTI_Line2,  EXTI_Line3,
  EXTI_Line4,  EXTI_Line5,  EXTI_Line6,  EXTI_Line7, 
  #if !defined(CH32V00x)
  EXTI_Line8,  EXTI_Line9,  EXTI_Line10, EXTI_Line11,
  EXTI_Line12, EXTI_Line13, EXTI_Line14, EXTI_Line15
  #endif
};
#endif


/* Private Functions */
/**
  * @brief  This function returns the pin ID function of the HAL PIN definition
  * @param  pin : one of the gpio pin
  * @retval None
  */
static uint8_t get_pin_id(uint16_t pin)
{
  uint8_t id = 0;

  while (pin != 0x0001) {
    pin = pin >> 1;
    id++;
  }

  return id;
}

void ch32_interrupt_enable(GPIO_TypeDef *port, GPIOMode_TypeDef io_mode,uint16_t pin, void (*callback)(void), EXTIMode_TypeDef it_mode, EXTITrigger_TypeDef trigger_mode)
{
    GPIO_InitTypeDef GPIO_InitStruct={0};
    EXTI_InitTypeDef EXTI_InitStruct={0};
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    uint8_t id = get_pin_id(pin);
    uint8_t gpio_port_souce=0;
    GPIO_InitStruct.GPIO_Pin  = pin;
    GPIO_InitStruct.GPIO_Mode = io_mode;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &GPIO_InitStruct);

    #if defined(GPIOA_BASE)
    if(port == GPIOA)  gpio_port_souce = GPIO_PortSourceGPIOA;
    #endif
    #if defined(GPIOB_BASE) 
    if(port == GPIOB)  gpio_port_souce = GPIO_PortSourceGPIOB;
    #endif
    #if defined(GPIOC_BASE)
    if(port == GPIOC)  gpio_port_souce = GPIO_PortSourceGPIOC;
    #endif
    #if defined(GPIOD_BASE)
    if(port == GPIOD)  gpio_port_souce = GPIO_PortSourceGPIOD;
    #endif
    #if defined(GPIOE_BASE)
    if(port == GPIOE)  gpio_port_souce = GPIO_PortSourceGPIOE;
    #endif
    #if defined(GPIOF_BASE)
    if(port == GPIOF)  gpio_port_souce = GPIO_PortSourceGPIOF;
    #endif
    #if defined(GPIOG_BASE)
    if(port == GPIOG)  gpio_port_souce = GPIO_PortSourceGPIOG;
    #endif
    #if defined(GPIOH_BASE)
    if(port == GPIOH)  gpio_port_souce = GPIO_PortSourceGPIOH;
    #endif

    GPIO_EXTILineConfig(gpio_port_souce, id);
    EXTI_InitStruct.EXTI_Line = exti_lines[id];
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = it_mode;
    EXTI_InitStruct.EXTI_Trigger = trigger_mode;
    EXTI_Init(&EXTI_InitStruct);

    gpio_irq_conf[id].callback = callback;

    NVIC_SetPriority(gpio_irq_conf[id].irqnb, EXTI_IRQ_PRIO);
    NVIC_EnableIRQ(gpio_irq_conf[id].irqnb);
}


/**
  * @brief  This function disable the interruption on the selected port/pin
  * @param  port : one of the gpio port
  * @param  pin : one of the gpio pin
  * @retval None
  */
void ch32_interrupt_disable(GPIO_TypeDef *port, uint16_t pin)
{
  uint8_t id = get_pin_id(pin);
  gpio_irq_conf[id].callback = NULL;

  for (int i = 0; i < NB_EXTI; i++) {
    if (gpio_irq_conf[id].irqnb == gpio_irq_conf[i].irqnb
        && gpio_irq_conf[i].callback != NULL) {
      return;
    }
  }
  NVIC_DisableIRQ(gpio_irq_conf[id].irqnb);  
}

/**
  * @brief This function his called by the HAL if the IRQ is valid
  * @param  GPIO_Pin : one of the gpio pin
  * @retval None
  */
void _gpio_exti_callback(uint16_t GPIO_Pin)
{
  uint8_t irq_id = get_pin_id(GPIO_Pin);
  if (gpio_irq_conf[irq_id].callback != NULL) {
    gpio_irq_conf[irq_id].callback();
  }
}



#if defined(CH32V00x)

#ifdef __cplusplus
extern "C" {
#endif

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void)
{
   uint32_t pin;
   for (pin = GPIO_Pin_0; pin <= GPIO_Pin_7; pin = pin << 1) 
   {
      EXTI_ClearITPendingBit(pin);   //0x1 2 4 8 10 20 40 80
      _gpio_exti_callback(pin);
   }
}
#ifdef __cplusplus
}
#endif


#elif defined(CH32X035)


#ifdef __cplusplus
extern "C" {
#endif

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI15_8_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI25_16_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void EXTI7_0_IRQHandler(void)
{
   uint32_t pin;
   for (pin = GPIO_Pin_0; pin <= GPIO_Pin_7; pin = pin << 1) 
   {
      EXTI_ClearITPendingBit(pin);   //0x1 2 4 8 10 20 40 80
      _gpio_exti_callback(pin);
   }
}

void EXTI15_8_IRQHandler(void)
{
   uint32_t pin;
   for (pin = GPIO_Pin_8; pin <= GPIO_Pin_15; pin = pin << 1) 
   {
      EXTI_ClearITPendingBit(pin);   //0x1 2 4 8 10 20 40 80
      _gpio_exti_callback(pin);
   }
}

void EXTI25_16_IRQHandler(void)
{
   uint32_t pin;
   for (pin = GPIO_Pin_16; pin <= GPIO_Pin_23; pin = pin << 1) 
   {
      EXTI_ClearITPendingBit(pin);   //0x1 2 4 8 10 20 40 80
      _gpio_exti_callback(pin);
   }
}


#ifdef __cplusplus
}
#endif


#else

#ifdef __cplusplus
extern "C" {
#endif

void EXTI0_IRQHandler(void)     __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI1_IRQHandler(void)     __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI2_IRQHandler(void)     __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI3_IRQHandler(void)     __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI4_IRQHandler(void)     __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI9_5_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI15_10_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
/**
  * @brief This function handles external line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line0); 
  _gpio_exti_callback(EXTI_Line0);
}

/**
  * @brief This function handles external line 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{
   EXTI_ClearITPendingBit(EXTI_Line1); 
  _gpio_exti_callback(EXTI_Line1);
}

/**
  * @brief This function handles external line 2 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line2); 
  _gpio_exti_callback(EXTI_Line2);
}

/**
  * @brief This function handles external line 3 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line3); 
  _gpio_exti_callback(EXTI_Line3);
}

/**
  * @brief This function handles external line 4 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler(void)
{
   EXTI_ClearITPendingBit(EXTI_Line4); 
   _gpio_exti_callback(EXTI_Line4);
}


/**
  * @brief This function handles external line 5 to 9 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  uint32_t pin;
  for (pin = GPIO_Pin_5; pin <= GPIO_Pin_9; pin = pin << 1) {
    EXTI_ClearITPendingBit(pin); 
    _gpio_exti_callback(pin);
  }
}

/**
  * @brief This function handles external line 10 to 15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  uint32_t pin;
  for (pin = GPIO_Pin_10; pin <= GPIO_Pin_15; pin = pin << 1) {
      EXTI_ClearITPendingBit(pin); 
    _gpio_exti_callback(pin);
  }
}

#ifdef __cplusplus
}
#endif

#endif  /* CH32V00x */

#endif /* EXTI_MODULE_ENABLED */



