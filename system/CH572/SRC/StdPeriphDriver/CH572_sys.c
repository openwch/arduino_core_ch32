/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_SYS.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : source file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH572_common.h"

/*********************************************************************
 * @fn      SetSysClock
 *
 * @brief   ����ϵͳ����ʱ��
 *
 * @param   sc      - ϵͳʱ��Դѡ�� refer to SYS_CLKTypeDef
 *
 * @return  none
 */
__HIGH_CODE
void SetSysClock(SYS_CLKTypeDef sc)
{
    uint16_t clk_sys_cfg;
    uint8_t i;
    uint8_t x32M_c;

    if(sc == RB_CLK_SYS_MOD)  // LSI
    {
        sys_safe_access_enable();
        R8_SLP_POWER_CTRL |= 0x40;
        R8_CLK_SYS_CFG |= RB_CLK_SYS_MOD;
        sys_safe_access_disable();
    }
    else
    {
        if(!(R8_HFCK_PWR_CTRL & RB_CLK_XT32M_PON))
        {
            x32M_c = R8_XT32M_TUNE;
            sys_safe_access_enable();
            R8_XT32M_TUNE |= 0x03;
            R8_HFCK_PWR_CTRL |= RB_CLK_XT32M_PON;
            sys_safe_access_disable();
            clk_sys_cfg = R8_CLK_SYS_CFG;
            sys_safe_access_enable();
            R8_CLK_SYS_CFG |= 0xC0;
            sys_safe_access_disable();
            for(i=0; i<9; i++)
            {
                __nop();
            }
            sys_safe_access_enable();
            R8_CLK_SYS_CFG = clk_sys_cfg;
            R8_XT32M_TUNE = x32M_c;
            sys_safe_access_disable();
        }

        if((sc & RB_CLK_SYS_MOD) == 0x40) // PLL div
        {
            sys_safe_access_enable();
            R8_HFCK_PWR_CTRL |= RB_CLK_PLL_PON;
            R8_FLASH_CFG = 0X01;
            R8_FLASH_SCK |= 1<<4; //50M
            sys_safe_access_disable();
        }
        else    // 32M div
        {
            if((sc&0x1F) )
            {
                sys_safe_access_enable();
                R8_FLASH_CFG = 0X02;
                sys_safe_access_disable();
            }
            else
            {
                sys_safe_access_enable();
                R8_FLASH_CFG = 0X07;
                sys_safe_access_disable();
            }
        }

        sys_safe_access_enable();
        R8_SLP_POWER_CTRL |= 0x40;
        R8_CLK_SYS_CFG = sc;
        sys_safe_access_disable();
    }
}

/*********************************************************************
 * @fn      GetSysClock
 *
 * @brief   ��ȡ��ǰϵͳʱ��
 *
 * @param   none
 *
 * @return  Hz
 */
uint32_t GetSysClock(void)
{
    if((R8_CLK_SYS_CFG & RB_CLK_SYS_MOD) == RB_CLK_SYS_MOD)
    { // 32K����Ƶ
        return (Freq_LSI);
    }
    else if((R8_CLK_SYS_CFG & RB_CLK_SYS_MOD) == 0x40)
    {
        return (600000000 / ((R8_CLK_SYS_CFG & 0x1f)?(R8_CLK_SYS_CFG & 0x1f):32));
    }
    else
    { // 32M���з�Ƶ
        return (32000000 / ((R8_CLK_SYS_CFG & 0x1f)?(R8_CLK_SYS_CFG & 0x1f):32));
    }
}

/*********************************************************************
 * @fn      SYS_GetInfoSta
 *
 * @brief   ��ȡ��ǰϵͳ��Ϣ״̬
 *
 * @param   i       - refer to SYS_InfoStaTypeDef
 *
 * @return  �Ƿ���
 */
uint8_t SYS_GetInfoSta(SYS_InfoStaTypeDef i)
{
    if(i == STA_SAFEACC_ACT)
    {
        return (R8_SAFE_ACCESS_SIG & RB_SAFE_ACC_ACT);
    }
    else
    {
        return (R8_GLOB_CFG_INFO & (1 << i));
    }
}

/*********************************************************************
 * @fn      SYS_ResetExecute
 *
 * @brief   ִ��ϵͳ������λ
 *
 * @param   none
 *
 * @return  none
 */
__HIGH_CODE
void SYS_ResetExecute(void)
{
    FLASH_ROM_SW_RESET();
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      SYS_DisableAllIrq
 *
 * @brief   �ر������жϣ���������ǰ�ж�ֵ
 *
 * @param   pirqv   - ��ǰ�����ж�ֵ
 *
 * @return  none
 */
void SYS_DisableAllIrq(uint32_t *pirqv)
{
    *pirqv = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    PFIC->IRER[0] = 0xffffffff;
    PFIC->IRER[1] = 0xffffffff;
    asm volatile("fence.i");
}

/*********************************************************************
 * @fn      SYS_RecoverIrq
 *
 * @brief   �ָ�֮ǰ�رյ��ж�ֵ
 *
 * @param   irq_status  - ��ǰ�����ж�ֵ
 *
 * @return  none
 */
void SYS_RecoverIrq(uint32_t irq_status)
{
    PFIC->IENR[0] = (irq_status << 8);
    PFIC->IENR[1] = (irq_status >> 24);
}

/*********************************************************************
 * @fn      SYS_GetSysTickCnt
 *
 * @brief   ��ȡ��ǰϵͳ(SYSTICK)����ֵ
 *
 * @param   none
 *
 * @return  ��ǰ����ֵ
 */
uint32_t SYS_GetSysTickCnt(void)
{
    return SysTick->CNTL;
}

/*********************************************************************
 * @fn      WWDG_ITCfg
 *
 * @brief   ���Ź���ʱ������ж�ʹ��
 *
 * @param   s       - ����Ƿ��ж�
 *
 * @return  none
 */
void WWDG_ITCfg(FunctionalState s)
{
    uint8_t ctrl = R8_RST_WDOG_CTRL;

    if(s == DISABLE)
    {
        ctrl &= ~RB_WDOG_INT_EN;
    }
    else
    {
        ctrl |= RB_WDOG_INT_EN;
    }

    sys_safe_access_enable();
    R8_RST_WDOG_CTRL = ctrl;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      WWDG_ResetCfg
 *
 * @brief   ���Ź���ʱ����λ����
 *
 * @param   s       - ����Ƿ�λ
 *
 * @return  none
 */
void WWDG_ResetCfg(FunctionalState s)
{
    uint8_t ctrl = R8_RST_WDOG_CTRL;

    if(s == DISABLE)
    {
        ctrl &= ~RB_WDOG_RST_EN;
    }
    else
    {
        ctrl |= RB_WDOG_RST_EN;
    }

    sys_safe_access_enable();
    R8_RST_WDOG_CTRL = ctrl;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      WWDG_ClearFlag
 *
 * @brief   ������Ź��жϱ�־�����¼��ؼ���ֵҲ�����
 *
 * @param   none
 *
 * @return  none
 */
void WWDG_ClearFlag(void)
{
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_WDOG_INT_FLAG;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   Ӳ�������жϣ������ִ�и�λ����λ����Ϊ�ϵ縴λ
 *
 * @param   none
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
__attribute__((weak))
void HardFault_Handler(void)
{
    FLASH_ROM_SW_RESET();
    sys_safe_access_enable();
    R16_INT_LSI_TUNE = 0xFFFF;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    sys_safe_access_disable();
    while(1);
}

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   ���������жϣ���ѹ�����Чʱ����
 *
 * @param   none
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
__attribute__((weak))
void NMI_Handler(void)
{
    while(1);
}

/*********************************************************************
 * @fn      mDelayuS
 *
 * @brief   uS ��ʱ
 *
 * @param   t       - ʱ�����
 *
 * @return  none
 */
__HIGH_CODE
void mDelayuS(uint16_t t)
{
    uint32_t i;
#if(FREQ_SYS == 100000000)
    i = t * 25;
#elif(FREQ_SYS == 75000000)
    i = (t * 25)>>1;
#elif(FREQ_SYS == 60000000)
    i = t * 10;
#elif(FREQ_SYS == 50000000)
    i = (t * 25)>>1;
#elif(FREQ_SYS == 40000000)
    i = t * 10;
#elif(FREQ_SYS == 30000000)
    i = t * 5;
#elif(FREQ_SYS == 25000000)
    i = (t * 25)>>2;
#elif(FREQ_SYS == 24000000)
    i = t * 6;
#elif(FREQ_SYS == 20000000)
    i = t * 5;
#elif(FREQ_SYS == 16000000)
    i = t << 2;
#elif(FREQ_SYS == 8000000)
    i = t << 1;
#elif(FREQ_SYS == 6400000)
    i = t * 8 / 5;
#elif(FREQ_SYS == 4000000)
    i = t;
#elif(FREQ_SYS == 2000000)
    i = t >> 1;
#elif(FREQ_SYS == 1000000)
    i = t >> 2;
#else
    i = t;
#endif

#if((FREQ_SYS == 75000000)||\
    (FREQ_SYS == 60000000)||\
    (FREQ_SYS == 30000000))
    while(--i)
    {
        __nop();
    };
#else
    do
    {
        __nop();
    }while(--i);
#endif
}

/*********************************************************************
 * @fn      mDelaymS
 *
 * @brief   mS ��ʱ
 *
 * @param   t       - ʱ�����
 *
 * @return  none
 */
__HIGH_CODE
void mDelaymS(uint16_t t)
{
    do
    {
        mDelayuS(1000);
    }while(--t);
}

#ifdef DEBUG
int _write(int fd, char *buf, int size)
{
    int i;
    for(i = 0; i < size; i++)
    {
        while(R8_UART_TFC == UART_FIFO_SIZE);                  /* �ȴ����ݷ��� */
            R8_UART_THR = *buf++; /* �������� */
    }
    return size;
}

#endif

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
__attribute__((used))
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || ((uint32_t)curbrk + incr > (__get_SP() - 64)))
    return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}

/*********************************************************************
 * @fn      __wrap_memcpy
 *
 * @brief   wrap memcpy
 *
 * @return  dst
 */
__HIGH_CODE
void *__wrap_memcpy(void *dst, void *src, size_t size)
{
    __MCPY(dst, src, (void *)((uint32_t)src+size));
    return dst;
}
