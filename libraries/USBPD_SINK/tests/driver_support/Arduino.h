#ifndef TEST_DRIVER_ARDUINO_H
#define TEST_DRIVER_ARDUINO_H

#include <stdint.h>

#define TIM_MODULE_ENABLED
#define ENABLE 1
#define DISABLE 0
#define GPIO_Pin_14 (1u << 14)
#define GPIO_Pin_15 (1u << 15)
#define GPIO_Speed_50MHz 0
#define GPIO_Mode_IN_FLOATING 0
#define RCC_APB2Periph_GPIOC 0
#define RCC_APB2Periph_AFIO 0
#define RCC_AHBPeriph_USBPD 0
#define USBPD_IRQn 0
#define GPIOC 0

typedef struct {
    uint32_t GPIO_Pin;
    uint32_t GPIO_Speed;
    uint32_t GPIO_Mode;
} GPIO_InitTypeDef;

typedef struct {
    uint32_t CONFIG, CONTROL, PORT_CC1, PORT_CC2, STATUS;
    uint32_t DMA, TX_SEL, BMC_TX_SZ, BMC_CLK_CNT, BMC_BYTE_CNT;
} TestUSBPDRegisters;

typedef struct { uint32_t CTLR; } TestAFIORegisters;

static TestUSBPDRegisters testUSBPD;
static TestAFIORegisters testAFIO;
#define USBPD (&testUSBPD)
#define AFIO (&testAFIO)

static inline void RCC_APB2PeriphClockCmd(int peripheral, int enabled)
{ (void)peripheral; (void)enabled; }
static inline void RCC_AHBPeriphClockCmd(int peripheral, int enabled)
{ (void)peripheral; (void)enabled; }
static inline void GPIO_Init(int port, GPIO_InitTypeDef *config)
{ (void)port; (void)config; }
static inline void NVIC_DisableIRQ(int irq) { (void)irq; }
static inline void NVIC_EnableIRQ(int irq) { (void)irq; }
static inline void NVIC_ClearPendingIRQ(int irq) { (void)irq; }
static inline void NVIC_SetPriority(int irq, int priority)
{ (void)irq; (void)priority; }

#endif
