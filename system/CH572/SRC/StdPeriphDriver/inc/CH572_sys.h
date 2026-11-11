/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_SYS.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH572_SYS_H__
#define __CH572_SYS_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  rtc interrupt event define
 */
typedef enum
{
    RST_STATUS_SW = 0, // ������λ
    RST_STATUS_RPOR,   // �ϵ縴λ
    RST_STATUS_WTR,    // ���Ź���ʱ��λ
    RST_STATUS_MR,     // �ⲿ�ֶ���λ
    RST_STATUS_LRM0,   // ���Ѹ�λ-����λ����
    RST_STATUS_GPWSM,  // �µ�ģʽ���Ѹ�λ
    RST_STATUS_LRM1,   //	���Ѹ�λ-���Ź�����
    RST_STATUS_LRM2,   //	���Ѹ�λ-�ֶ���λ����

} SYS_ResetStaTypeDef;

/**
 * @brief  rtc interrupt event define
 */
typedef enum
{
    INFO_RESET_EN = 0x4,   // RST#�ⲿ�ֶ���λ���빦���Ƿ���
    INFO_BOOT_EN = 0x8,    // ϵͳ�������� BootLoader �Ƿ���
    INFO_RST_PIN = 0x10,   // ��λ����ѡ��1��PA7��0��PA8
    INFO_LOADER  = 0x20,   // ��ǰϵͳ�Ƿ���Bootloader ��
    STA_SAFEACC_ACT = 0x30,// ��ǰϵͳ�Ƿ��ڰ�ȫ����״̬������RWA�������򲻿ɷ���

} SYS_InfoStaTypeDef;

/**
 * @brief  ��ȡоƬID�࣬һ��Ϊ�̶�ֵ
 */
#define SYS_GetChipID()      R8_CHIP_ID

/**
 * @brief  ��ȡ��ȫ����ID��һ��Ϊ�̶�ֵ
 */
#define SYS_GetAccessID()    R8_SAFE_ACCESS_ID

/**
 * @brief   ����ϵͳ����ʱ��
 *
 * @param   sc      - ϵͳʱ��Դѡ�� refer to SYS_CLKTypeDef
 */
void SetSysClock(SYS_CLKTypeDef sc);

/**
 * @brief   ��ȡ��ǰϵͳʱ��
 *
 * @return  Hz
 */
uint32_t GetSysClock(void);

/**
 * @brief   ��ȡ��ǰϵͳ��Ϣ״̬
 *
 * @param   i       - refer to SYS_InfoStaTypeDef
 *
 * @return  �Ƿ���
 */
uint8_t SYS_GetInfoSta(SYS_InfoStaTypeDef i);

/**
 * @brief   ��ȡϵͳ�ϴθ�λ״̬
 *
 * @return  refer to SYS_ResetStaTypeDef
 */
#define SYS_GetLastResetSta()    (R8_RESET_STATUS & RB_RESET_FLAG)

/**
 * @brief   ִ��ϵͳ������λ
 */
void SYS_ResetExecute(void);

/**
 * @brief   ���ø�λ����Ĵ�����ֵ�������ֶ���λ�� ������λ�� ���Ź���λ������ͨ���Ѹ�λ��Ӱ��
 *
 * @param   i       - refer to SYS_InfoStaTypeDef
 */
#define SYS_ResetKeepBuf(d)    (R8_GLOB_RESET_KEEP = d)

/**
 * @brief   �ر������жϣ���������ǰ�ж�ֵ
 *
 * @param   pirqv   - ��ǰ�����ж�ֵ
 */
void SYS_DisableAllIrq(uint32_t *pirqv);

/**
 * @brief   �ָ�֮ǰ�رյ��ж�ֵ
 *
 * @param   irq_status  - ��ǰ�����ж�ֵ
 */
void SYS_RecoverIrq(uint32_t irq_status);

/**
 * @brief   ��ȡ��ǰϵͳ(SYSTICK)����ֵ
 *
 * @return  ��ǰ����ֵ
 */
uint32_t SYS_GetSysTickCnt(void);

/**
 * @brief   ���ؿ��Ź�������ֵ��������
 *
 * @param   c       - ���Ź�������ֵ
 */
#define WWDG_SetCounter(c)    (R8_WDOG_COUNT = c)

/**
 * @brief   ���Ź���ʱ������ж�ʹ��
 *
 * @param   s       - ����Ƿ��ж�
 */
void WWDG_ITCfg(FunctionalState s);

/**
 * @brief   ���Ź���ʱ����λ����
 *
 * @param   s       - ����Ƿ�λ
 */
void WWDG_ResetCfg(FunctionalState s);

/**
 * @brief   ��ȡ��ǰ���Ź���ʱ�������־
 *
 * @return  ���Ź���ʱ�������־
 */
#define WWDG_GetFlowFlag()    (R8_RST_WDOG_CTRL & RB_WDOG_INT_FLAG)

/**
 * @brief   ������Ź��жϱ�־�����¼��ؼ���ֵҲ�����
 */
void WWDG_ClearFlag(void);

/**
 * @brief   uS ��ʱ
 *
 * @param   t       - ʱ�����
 */
void mDelayuS(uint16_t t);

/**
 * @brief   mS ��ʱ
 *
 * @param   t       - ʱ�����
 */
void mDelaymS(uint16_t t);

/**
 * @brief Enter safe access mode.
 * 
 * @NOTE: After enter safe access mode, about 16 system frequency cycles 
 * are in safe mode, and one or more secure registers can be rewritten 
 * within the valid period. The safe mode will be automatically 
 * terminated after the above validity period is exceeded.
 *  if sys_safe_access_enable() is called,
 *  you must call sys_safe_access_disable() before call sys_safe_access_enable() again.
 */
#define sys_safe_access_enable()        do{volatile uint32_t mpie_mie;mpie_mie=__risc_v_disable_irq();SAFEOPERATE;\
                                        R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;SAFEOPERATE;

#define sys_safe_access_disable()       R8_SAFE_ACCESS_SIG = 0;__risc_v_enable_irq(mpie_mie);SAFEOPERATE;}while(0)

#ifdef __cplusplus
}
#endif

#endif // __CH572_SYS_H__
