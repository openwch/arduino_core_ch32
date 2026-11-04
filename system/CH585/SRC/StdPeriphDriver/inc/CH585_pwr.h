/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_pwr.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH585_PWR_H__
#define __CH585_PWR_H__

#ifdef __cplusplus
extern "C" {
#endif

#define ROM_CFG_ADR_HW       0x7F00C            // config address for hardware config for LDO&OSC and etc

/**
 * @brief	Peripher CLK control bit define
 */
#define BIT_SLP_CLK_TMR0     (0x00000001) /*!< TMR0 peripher clk bit */
#define BIT_SLP_CLK_TMR1     (0x00000002) /*!< TMR1 peripher clk bit */
#define BIT_SLP_CLK_TMR2     (0x00000004) /*!< TMR2 peripher clk bit */
#define BIT_SLP_CLK_TMR3     (0x00000008) /*!< TMR3 peripher clk bit */
#define BIT_SLP_CLK_UART0    (0x00000010) /*!< UART0 peripher clk bit */
#define BIT_SLP_CLK_UART1    (0x00000020) /*!< UART1 peripher clk bit */
#define BIT_SLP_CLK_UART2    (0x00000040) /*!< UART2 peripher clk bit */
#define BIT_SLP_CLK_UART3    (0x00000080) /*!< UART3 peripher clk bit */
#define BIT_SLP_CLK_SPI0     (0x00000100) /*!< SPI0 peripher clk bit */
//#define BIT_SLP_CLK_SPI1     (0x00000200)  /*!< SPI1 peripher clk bit */
#define BIT_SLP_CLK_PWMX     (0x00000400) /*!< PWMX peripher clk bit */
//#define BIT_SLP_CLK_LCD      (0x00000800)  /*!< LCD peripher clk bit */
#define BIT_SLP_CLK_USB      (0x00001000) /*!< USB peripher clk bit */
//#define BIT_SLP_CLK_ETH      (0x00002000)  /*!< ETH peripher clk bit */
//#define BIT_SLP_CLK_LED      (0x00004000)  /*!< LED peripher clk bit */
#define BIT_SLP_CLK_BLE      (0x00008000) /*!< BLE peripher clk bit */

#define BIT_SLP_CLK_RAMX     (0x10000000) /*!< main SRAM RAM16K peripher clk bit */
#define BIT_SLP_CLK_RAM2K    (0x20000000) /*!< RAM2K peripher clk bit */
#define BIT_SLP_CLK_ALL      (0x3000FFFF) /*!< All peripher clk bit */

/**
 * @brief  unit of controllable power supply
 */
#define UNIT_SYS_LSE         RB_CLK_XT32K_PON   // �ⲿ32K ʱ����
#define UNIT_SYS_LSI         RB_CLK_INT32K_PON  // �ڲ�32K ʱ����
#define UNIT_SYS_HSE         RB_CLK_XT32M_PON   // �ⲿ32M ʱ����
#define UNIT_SYS_PLL         RB_CLK_PLL_PON     // PLL ʱ����

/**
 * @brief  wakeup mode define
 */
typedef enum
{
    Short_Delay = 0,
    Long_Delay,

} WakeUP_ModeypeDef;

/**
 * @brief  wakeup mode define
 */
typedef enum
{
    /* ����ȼ���ʹ�ø߾��ȼ�أ�210uA���� */
    HALevel_1V9 = 0, // 1.7-1.95
    HALevel_2V1,     // 1.9-2.15
    HALevel_2V3,     // 2.1-2.35
    HALevel_2V5,     // 2.3-2.55

    /* ����ȼ���ʹ�õ͹��ļ�أ�1uA���� */
    LPLevel_1V7 = 0x80,
    LPLevel_1V8,
    LPLevel_1V9,
    LPLevel_2V0,
    LPLevel_2V1,
    LPLevel_2V2,
    LPLevel_2V3,
    LPLevel_2V4,

} VolM_LevelypeDef;

/**
 * @brief   �����ڲ�DC/DC��Դ�����ڽ�Լϵͳ����
 *
 * @param   s   - �Ƿ��DCDC��Դ
 */
void PWR_DCDCCfg(FunctionalState s);

/**
 * @brief   �ɿص�Ԫģ��ĵ�Դ����
 *
 * @param   s       - �Ƿ�򿪵�Դ
 * @param   unit    - please refer to unit of controllable power supply
 */
void PWR_UnitModCfg(FunctionalState s, uint8_t unit);

/**
 * @brief   ��ȫ����ʱ�ӿ���λ
 *
 * @param   s       - �Ƿ�򿪶�Ӧ����ʱ��
 * @param   perph   - please refer to SAFE CLK control bit define
 */
void PWR_SafeClkCfg(FunctionalState s, uint16_t perph);

/**
 * @brief   ����ʱ�ӿ���λ
 *
 * @param   s       - �Ƿ�򿪶�Ӧ����ʱ��
 * @param   perph   - please refer to Peripher CLK control bit define
 */
void PWR_PeriphClkCfg(FunctionalState s, uint16_t perph);

/**
 * @brief   ˯�߻���Դ����
 *
 * @param   s       - �Ƿ�򿪴�����˯�߻��ѹ���
 * @param   perph   - ��Ҫ���õĻ���Դ
 *                    RB_SLP_USB_WAKE   -  USB Ϊ����Դ
 *                    RB_SLP_RTC_WAKE   -  RTC Ϊ����Դ
 *                    RB_SLP_GPIO_WAKE  -  GPIO Ϊ����Դ
 *                    RB_SLP_BAT_WAKE   -  BAT Ϊ����Դ
 * @param   mode    - refer to WakeUP_ModeypeDef
 */
void PWR_PeriphWakeUpCfg(FunctionalState s, uint8_t perph, WakeUP_ModeypeDef mode);

/**
 * @brief   ��Դ���
 *
 * @param   s       - �Ƿ�򿪴˹���
 * @param   vl      - refer to VolM_LevelypeDef
 */
void PowerMonitor(FunctionalState s, VolM_LevelypeDef vl);

/**
 * @brief   �͹���-Idleģʽ
 */
void LowPower_Idle(void);

/**
 * @brief   �͹���-Haltģʽ���˵͹����е��ڲ�4Mʱ�����У����Ѻ�ָ�ʱ��Դ�˳�
 */
void LowPower_Halt(void);

/**
 * @brief   �͹���-Haltģʽ���˵͹����е��ڲ�4Mʱ�����У����Ѻ���Ҫ�û��Լ�����ѡ��ϵͳʱ��Դ
 */

void LowPower_Halt_WFE(void);
/**
 * @brief   �͹���-Sleepģʽ���˵͹����л����ڲ�4Mʱ�����У����Ѻ�ָ�ʱ��Դ�˳�
 *          @note ע����ô˺�����DCDC����ǿ�ƹرգ����Ѻ�����ֶ��ٴδ�
 *
 * @param   rm      - ����ģ��ѡ��
 *                    RB_PWR_RAM2K  -   2K retention SRAM ����
 *                    RB_PWR_RAM16K -   16K main SRAM ����
 *                    RB_PWR_EXTEND -   USB �� BLE ��Ԫ�������򹩵�
 *                    RB_PWR_XROM   -   FlashROM ����
 *                    NULL          -   ���ϵ�Ԫ���ϵ�
 */
void LowPower_Sleep(uint16_t rm);

/**
 * @brief   ʹ��WFE���ѵĵ͹���-Sleepģʽ���л����ڲ�4Mʱ�����У����Ѻ���Ҫ�û��Լ�����ѡ��ϵͳʱ��Դ
 *          @note ע����ô˺�����DCDC����ǿ�ƹرգ����Ѻ�����ֶ��ٴδ�
 *
 * @param   rm      - ����ģ��ѡ��
 *                    RB_PWR_RAM2K  -   2K retention SRAM ����
 *                    RB_PWR_RAM16K -   16K main SRAM ����
 *                    RB_PWR_EXTEND -   USB �� BLE ��Ԫ�������򹩵�
 *                    RB_PWR_XROM   -   FlashROM ����
 *                    NULL          -   ���ϵ�Ԫ���ϵ�
 */
void LowPower_Sleep_WFE(uint16_t rm);

/**
 * @brief   �͹���-Shutdownģʽ���˵͹����е�HSI/5ʱ�����У����Ѻ���Ҫ�û��Լ�����ѡ��ϵͳʱ��Դ
 *          @note ע����ô˺�����DCDC����ǿ�ƹرգ����Ѻ�����ֶ��ٴδ�
 *
 * @param   rm      - ����ģ��ѡ��
 *                    RB_PWR_RAM2K  -   2K retention SRAM ����
 *                    RB_PWR_RAM16K -   16K main SRAM ����
 *                    NULL          -   ���ϵ�Ԫ���ϵ�
 */
void LowPower_Shutdown(uint16_t rm);

#ifdef __cplusplus
}
#endif

#endif // __CH585_PWR_H__
