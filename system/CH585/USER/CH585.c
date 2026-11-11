#include "CH585.h"

// https://www.cnblogs.com/gscw/p/18029303
// for c++
void _fini(){}
void _init(){}


/*********************************************************************
 * @fn      GPIO_Init
 *
 * @brief   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *
 * @param   GPIO_InitStruct - pointer to a GPIO_InitTypeDef structure that
 *        contains the configuration information for the specified GPIO peripheral.
 *
 * @return  none
 */
void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct)
{
    if((((uint32_t)GPIO_InitStruct->GPIO_Mode) & ((uint32_t)0x10)) != 0x00) //0x10 OUTPUT
    {
        GPIOx->DIR |= GPIO_InitStruct->GPIO_Pin;
        GPIOx->PD_DRV |= GPIO_InitStruct->GPIO_Pin;
    }else{
        GPIOx->DIR &= ~GPIO_InitStruct->GPIO_Pin;
        switch (GPIO_InitStruct->GPIO_Mode){
        case GPIO_Mode_AIN:
        case GPIO_Mode_IN_FLOATING:
            GPIOx->PU &= ~GPIO_InitStruct->GPIO_Pin;
            GPIOx->PD_DRV &= ~GPIO_InitStruct->GPIO_Pin;
            break;
        case GPIO_Mode_IPD:
            GPIOx->PU &= ~GPIO_InitStruct->GPIO_Pin;
            GPIOx->PD_DRV |= GPIO_InitStruct->GPIO_Pin;
            break;
        case GPIO_Mode_IPU:
            GPIOx->PD_DRV &= ~GPIO_InitStruct->GPIO_Pin;
            GPIOx->PU |= GPIO_InitStruct->GPIO_Pin;
            break;
        default:
            break;
        }
    }
}

/*********************************************************************
 * @fn      GPIO_ReadInputDataBit
 *
 * @brief   GPIOx - where x can be (A..C) to select the GPIO peripheral.
 *
 * @param    GPIO_Pin - specifies the port bit to read.
 *             This parameter can be GPIO_Pin_x where x can be (0..23).
 *
 * @return  The input port pin value.
 */
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
    uint8_t bitstatus = 0x00;

    if((GPIOx->PIN & GPIO_Pin) != (uint32_t)Bit_RESET)
    {
        bitstatus = (uint8_t)Bit_SET;
    }
    else
    {
        bitstatus = (uint8_t)Bit_RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      GPIO_SetBits
 *
 * @brief   Sets the selected data port bits.
 *
 * @param   GPIOx - where x can be (A..C) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bits to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..23).
 *
 * @return  none
 */
void GPIO_SetBits(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
    GPIOx->OUT |= GPIO_Pin;
}

/*********************************************************************
 * @fn      GPIO_ResetBits
 *
 * @brief   Clears the selected data port bits.
 *
 * @param   GPIOx - where x can be (A..C) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bits to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..23).
 *
 * @return  none
 */
void GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
    GPIOx->CLR = GPIO_Pin;
}

void ADC_Cmd(ADC_TypeDef *ADCx, FunctionalState NewState)
{
    (void)ADCx;
    if (NewState != DISABLE) {
        R8_ADC_CFG |= RB_ADC_POWER_ON;
    } else {
        R8_ADC_CFG &= ~RB_ADC_POWER_ON;
    }
}

void ADC_DeInit(ADC_TypeDef *ADCx)
{
    (void)ADCx;
    R8_TEM_SENSOR &= ~RB_TEM_SEN_PWR_ON;
}

void ADC_Init(ADC_TypeDef *ADCx, ADC_InitTypeDef *ADC_InitStruct)
{
    (void)ADCx;
    (void)ADC_InitStruct;
    /* Disable touch key; keep power bit; external default: div, -6dB, buffer on */
    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_TEM_SENSOR &= ~RB_TEM_SEN_PWR_ON;
    R8_ADC_CFG = (R8_ADC_CFG & ~(RB_ADC_BUF_EN | RB_ADC_DIFF_EN | RB_ADC_PGA_GAIN | RB_ADC_CLK_DIV))
                 | (0b10 << 6) | (0b01 << 4) | RB_ADC_BUF_EN;
    R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
}

void ADC_RegularChannelConfig(ADC_TypeDef *ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime)
{
    (void)ADCx;
    (void)Rank;
    (void)ADC_SampleTime;

    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;

    if (ADC_Channel == ADC_Channel_TempSensor) {
        /* Datasheet 15.3.2 / ADC_InterTSSampInit */
        R8_TEM_SENSOR = RB_TEM_SEN_PWR_ON;
        R8_ADC_CHANNEL = ADC_Channel;
        R8_ADC_CFG = (R8_ADC_CFG & ~(RB_ADC_BUF_EN | RB_ADC_DIFF_EN | RB_ADC_PGA_GAIN | RB_ADC_CLK_DIV))
                     | RB_ADC_POWER_ON | RB_ADC_DIFF_EN | (0b11 << 4) | (0b10 << 6);
        R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    } else if (ADC_Channel == ADC_Channel_Vbat) {
        /* ADC_InterBATSampInit: -12dB, buffer on, single-ended */
        R8_TEM_SENSOR &= ~RB_TEM_SEN_PWR_ON;
        R8_ADC_CHANNEL = ADC_Channel;
        R8_ADC_CFG = (R8_ADC_CFG & ~(RB_ADC_BUF_EN | RB_ADC_DIFF_EN | RB_ADC_PGA_GAIN | RB_ADC_CLK_DIV))
                     | RB_ADC_POWER_ON | RB_ADC_BUF_EN | (0b00 << 4) | (0b10 << 6);
        R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    } else {
        /* External single-ended defaults */
        R8_TEM_SENSOR &= ~RB_TEM_SEN_PWR_ON;
        R8_ADC_CHANNEL = ADC_Channel;
        R8_ADC_CFG = (R8_ADC_CFG & ~(RB_ADC_BUF_EN | RB_ADC_DIFF_EN | RB_ADC_PGA_GAIN | RB_ADC_CLK_DIV))
                     | RB_ADC_POWER_ON | RB_ADC_BUF_EN | (0b01 << 4) | (0b10 << 6);
        R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    }
}

void ADC_SoftwareStartConvCmd(ADC_TypeDef *ADCx, FunctionalState NewState)
{
    (void)ADCx;
    if (NewState != DISABLE) {
        R8_ADC_CONVERT |= RB_ADC_START;
    }
}

FlagStatus ADC_GetFlagStatus(ADC_TypeDef *ADCx, uint8_t ADC_FLAG)
{
    (void)ADCx;
    if (ADC_FLAG == ADC_FLAG_EOC) {
        if ((R8_ADC_CONVERT & RB_ADC_START) == 0) {
            return SET;
        }
    }
    return RESET;
}

uint16_t ADC_ConvertPAGValueCH585(void)
{
    uint16_t adcValue = R16_ADC_DATA & RB_ADC_DATA;
    uint8_t pgaSetting = (R8_ADC_CFG & RB_ADC_PGA_GAIN) >> 4;
    switch (pgaSetting) {
        case ADC_PGA_1_4:
            /* Voltage = (ADC/512-3)*1.05V; map to ~10-bit @ 3.3V */
            return ((adcValue * 163 / 256) - 977);
        case ADC_PGA_1_2:
            return ((adcValue * 81 / 256) - 326);
        case ADC_PGA_0:
            return (adcValue * 41 / 256);
        case ADC_PGA_2:
            return ((adcValue * 20 / 256) + 163);
        default:
            return adcValue;
    }
}

