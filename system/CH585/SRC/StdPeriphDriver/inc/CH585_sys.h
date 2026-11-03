/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_SYS.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH585_SYS_H__
#define __CH585_SYS_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*MachineMode_Call_func)(void);

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
    INFO_ROM_READ = 0, // FlashROM ����������� �Ƿ�ɶ�
    INFO_RESET_EN = 2, // RST#�ⲿ�ֶ���λ���빦���Ƿ���
    INFO_BOOT_EN,      // ϵͳ�������� BootLoader �Ƿ���
    INFO_DEBUG_EN,     // ϵͳ������Խӿ��Ƿ���
    INFO_LOADER,       // ��ǰϵͳ�Ƿ���Bootloader ��
    STA_SAFEACC_ACT,   // ��ǰϵͳ�Ƿ��ڰ�ȫ����״̬������RWA�������򲻿ɷ���

} SYS_InfoStaTypeDef;

/**
 * @brief  IWDG_KR_Key
 */
typedef enum
{
    KEY_UNPROTECT = 0x5555,        //�������
    KEY_RELOADING_COUNT = 0xAAAA,  //��װ�ؼ���ֵ
    KEY_START_IWDG = 0xCCCC        //�������Ź�
} IWDG_KR_Key;

/**
 * @brief  IWDG Prescaler factor
 */
typedef enum
{
    IWDG_PR_4 = 0, // 4��Ƶ
    IWDG_PR_8,     // 8��Ƶ
    IWDG_PR_16,    // 16��Ƶ
    IWDG_PR_32,    // 32��Ƶ
    IWDG_PR_64,    // 64��Ƶ
    IWDG_PR_128,   // 128��Ƶ
    IWDG_PR_256,   // 256��Ƶ
    IWDG_PR_512    // 512��Ƶ
} IWDG_32K_PR;

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
 * @brief   ע���еģʽִ�к��������ڻ�еģʽ�µ���
 *
 * @param   func    -   �����ڻ�еģʽ��ִ�еĺ���
 */
void MachineMode_Call(MachineMode_Call_func func);

/**
 * @brief  ʹ��Ԥȡָ���
 */
void SYS_EnablePI(void);

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
 * @brief   ��ȡд����״̬
 *
 * @return  1:��ֹ������Ӧ�ֶ�; 0:�������
 */
#define  IWDG_WR_Protect()       (R32_IWDG_CFG >> 30 & 0x01)

/**
 * @brief   ��ȡ���Ź�������
 *
 * @return  COUNT
 */
#define  IWDG_Count_Get()        ((R32_IWDG_CFG >> 16) & 0xFFF)

/**
 * @brief   ��ȡ���üĴ������±�־���ر�д����λ��Ч
 *
 * @return  1:�Ĵ�������; 0:�Ĵ���������
 */
#define  IWDG_PVU_Get()          (R32_IWDG_CFG >> 15 & 0x01)

/**
 * @brief   �������Ź�/���������/ι��/��װ�ؼ���ֵ
 *
 * @param   kr       - ����ֵ
 */
void IWDG_KR_Set(IWDG_KR_Key kr);

/**
 * @brief   ����Ԥ��Ƶ���ر�д����λ��Ч
 *
 * @param   pr       - ��Ƶϵ��
 */
uint8_t IWDG_PR_Set(IWDG_32K_PR pr);

/**
 * @brief   ���ü�������װ��ֵ���ر�д����λ��Ч
 *
 * @param   rlr       - ��������װ��ֵ
 */
uint8_t IWDG_RLR_Set(uint16_t rlr);

/**
 * @brief   �������Ź����������ں�ֹͣʹ�ܣ����ڵ���ģʽ����Ч
 *
 * @param   s       - �Ƿ�ʹ��
 */
uint8_t IWDG_FollowCoreStop(FunctionalState s);

/**
 * @brief   �������Ź�ʹ��
 *
 * @param   pr     - Ԥ��Ƶ
 *          rlr    - ��������װ��ֵ
 */
uint8_t IWDG_Enable(IWDG_32K_PR pr, uint16_t rlr);

/**
 * @brief   ϵͳ���붨����װ�ؿ��Ź�����ֵ�Է�ֹ��λ
 *
 * @param   none
 */
void IWDG_Feed(void);

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

#endif // __CH585_SYS_H__
