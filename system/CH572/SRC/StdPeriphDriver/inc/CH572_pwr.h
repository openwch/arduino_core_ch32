/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_pwr.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH572_PWR_H__
#define __CH572_PWR_H__

#ifdef __cplusplus
extern "C" {
#endif

#define ROM_CFG_ADR_HW       0x7F00C            // config address for hardware config for LDO&OSC and etc

/**
 * @brief  wakeup mode define, select wakeup delay
 */
typedef enum
{
    Fsys_Delay_3584 = 0,
    Fsys_Delay_512,
    Fsys_Delay_64,
    Fsys_Delay_1,
    Fsys_Delay_8191,
    Fsys_Delay_7168,
    Fsys_Delay_6144,
    Fsys_Delay_4096,

} WakeUP_ModeypeDef;

/**
 * @brief  wakeup mode define
 */
typedef enum
{
    /* ����ȼ���ʹ�õ͹��ļ�أ�1uA���� */
    LPLevel_1V8 = 0,
    LPLevel_2V0,
    LPLevel_2V2,
    LPLevel_2V4,

} VolM_LevelypeDef;

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
 * @brief   �͹���-Haltģʽ���˵͹����е�HSI/5ʱ�����У����Ѻ���Ҫ�û��Լ�����ѡ��ϵͳʱ��Դ
 */
void LowPower_Halt(void);

/**
 * @brief   �͹���-Sleepģʽ���˵͹����е�HSI/5ʱ�����У����Ѻ���Ҫ�û��Լ�����ѡ��ϵͳʱ��Դ
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

#endif // __CH572_PWR_H__
