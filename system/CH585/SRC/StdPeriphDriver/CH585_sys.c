/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_SYS.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : source file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH585_common.h"

volatile MachineMode_Call_func gs_machine_mode_func;

extern uint32_t _vector_base[];

uint32_t chip_info;
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
    chip_info = ((*(uint32_t*)ROM_CFG_CHIP_ID));
    R32_SAFE_MODE_CTRL |= RB_XROM_312M_SEL;
    R8_SAFE_MODE_CTRL &= ~RB_SAFE_AUTO_EN;
    sys_safe_access_enable();
    if(sc == RB_CLK_SYS_MOD)  // 32KHz
    {
        R16_CLK_SYS_CFG |= RB_CLK_SYS_MOD;
    }

    else
    {
        if(sc & RB_OSC32M_SEL)
        {
            if(!(R8_HFCK_PWR_CTRL & RB_CLK_XT32M_PON))
            {
                x32M_c = R8_XT32M_TUNE;
                R8_XT32M_TUNE |= 0x03;
                R8_HFCK_PWR_CTRL |= RB_CLK_XT32M_PON;
                clk_sys_cfg = R16_CLK_SYS_CFG;
                R16_CLK_SYS_CFG |= 0xC0;
                for(i=0; i<9; i++)
                {
                    __nop();
                }
                R16_CLK_SYS_CFG = clk_sys_cfg;
                R8_XT32M_TUNE = x32M_c;
            }
        }
        else
        {
            R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON;
        }

        if(sc & RB_XROM_SCLK_SEL) // PLL div
        {
            R8_HFCK_PWR_CTRL |= RB_CLK_PLL_PON;
            if((sc&0x1F) == 0 )
            {
                R8_FLASH_SCK = R8_FLASH_SCK|(1<<4);
                R8_FLASH_CFG = 0X07;
            }
            else if((sc&0x1F) < 10)
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X01;
            }
            else if((sc&0x1F) < 16)
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X02;
            }
            else
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X07;
            }
        }
        else
        {
            if((sc&0x1F) < 8 )
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X51;
            }
            else
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X57;
            }
        }

        R16_CLK_SYS_CFG = sc|0xC0;
        R16_CLK_SYS_CFG = sc;

        if(sc & RB_OSC32M_SEL)
        {
            if((!((R8_GLOB_CFG_INFO & RB_CFG_DEBUG_EN)|(R8_GLOB_CFG_INFO & RB_CFG_ROM_READ ))) && (R8_SAFE_DEBUG_CTRL & RB_DEBUG_DIS))
            {
                R8_HFCK_PWR_CTRL &= ~RB_CLK_RC16M_PON;
            }
        }
        else
        {
            R8_HFCK_PWR_CTRL &= ~RB_CLK_XT32M_PON;
        }

    }
    R8_SAFE_MODE_CTRL |= RB_SAFE_AUTO_EN;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      highcode_init
 *
 * @brief   ����highcode����
 *
 * @param   none
 *
 * @return  none
 */
__attribute__((section(".highcode_init")))
void highcode_init(void)
{
    R32_SAFE_MODE_CTRL |= RB_XROM_312M_SEL;
    R8_SAFE_MODE_CTRL &= ~RB_SAFE_AUTO_EN;
    sys_safe_access_enable();
    R32_MISC_CTRL |= 5|(3<<25); //
    R8_PLL_CONFIG &= ~(1 << 5); //
    R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON | RB_CLK_PLL_PON;
    R16_CLK_SYS_CFG = CLK_SOURCE_HSI_PLL_62_4MHz;
    R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
    R8_FLASH_CFG = 0X02;
    R8_XT32M_TUNE = (R8_XT32M_TUNE&(~0x03))|0x01;
    R8_CK32K_CONFIG |= RB_CLK_INT32K_PON;
    R8_SAFE_MODE_CTRL |= RB_SAFE_AUTO_EN;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      MachineMode_Call_IRQ
 *
 * @brief   ��еģʽ���ú���ʹ�õ��ж�
 *
 * @param   none
 *
 * @return  none
 */
__HIGH_CODE
__INTERRUPT
void MachineMode_Call_IRQ(void)
{
    if(gs_machine_mode_func != NULL)
    {
        gs_machine_mode_func();
        gs_machine_mode_func = NULL;
    }
}

/*********************************************************************
 * @fn      MachineMode_Call
 *
 * @brief   ע���еģʽִ�к��������ڻ�еģʽ�µ���
 *
 * @param   func    -   �����ڻ�еģʽ��ִ�еĺ���
 *
 * @return  none
 */
__HIGH_CODE
void MachineMode_Call(MachineMode_Call_func func)
{
    uint8_t i;
    uint32_t sw_vtf, sw_irqtable;
    uint32_t irqv;

    /* ����ر������ж� */
    irqv = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    PFIC->IRER[0] = 0xffffffff;
    PFIC->IRER[1] = 0xffffffff;

    /* ����û�ʹ����SW�жϵ�����жϣ�����Ҫȡ���˺�������ע�� */
//    for(i = 0; i < 4; i++)
//    {
//        if(PFIC->VTFIDR[i] == SWI_IRQn)
//        {
//            /* �ҵ����û��Լ�ʹ�õ�SW�жϣ��ر��� */
//            sw_vtf = PFIC->VTFADDR[i];
//            PFIC->VTFADDR[i] = (sw_vtf & 0xFFFFFFFE);
//            break;
//        }
//    }

    sw_irqtable = _vector_base[SWI_IRQn];
    _vector_base[SWI_IRQn] = (uint32_t)MachineMode_Call_IRQ;
    gs_machine_mode_func = func;

    /* ֻ��SWI_IRQn */
    PFIC_EnableIRQ(SWI_IRQn);
    /* ����SWI_IRQn�жϴ������� */
    PFIC_SetPendingIRQ(SWI_IRQn);

    /* �ȴ��������� */
    while(gs_machine_mode_func != NULL);

    PFIC_DisableIRQ(SWI_IRQn);

    _vector_base[SWI_IRQn] = sw_irqtable;

//    if(i != 4)
//    {
//        /* �ָ�ԭ����SW����ж� */
//        PFIC->VTFADDR[i] = sw_vtf;
//    }

    /* ����ָ�ԭ�����ж�ʹ������ */
    PFIC->IENR[0] = (irqv << 8);
    PFIC->IENR[1] = (irqv >> 24);
}

/*********************************************************************
 * @fn      SetPI_func
 *
 * @brief   ���ڻ�еģʽ���õ�ʹ��Ԥȡָ���
 *
 * @param   none
 *
 * @return  none
 */void SetPI_func()
{
    write_csr(0xbc0, 0x25);
}

/*********************************************************************
 * @fn      SYS_EnablePI
 *
 * @brief   ʹ��Ԥȡָ���
 *
 * @param   none
 *
 * @return  null
 */
void SYS_EnablePI()
{
    MachineMode_Call(SetPI_func);
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
    if((R16_CLK_SYS_CFG & RB_CLK_SYS_MOD) == RB_CLK_SYS_MOD)
    { // 32K����Ƶ
        return (CAB_LSIFQ);
    }
    else if(R16_CLK_SYS_CFG & RB_XROM_SCLK_SEL)
    {
        if(!(R16_CLK_SYS_CFG & 0x1f))
        {
            return ((R16_CLK_SYS_CFG & RB_OSC32M_SEL)?32000000:16000000);
        }
        else
        {   // PLL���з�Ƶ
            return (312000000 / (R16_CLK_SYS_CFG & 0x1f));
        }
    }
    else if(R16_CLK_SYS_CFG & RB_OSC32M_SEL)
    { // 32M���з�Ƶ
        return (32000000 / (R16_CLK_SYS_CFG & 0x1f));
    }
    else
    { // 16M���з�Ƶ
        return (16000000 / (R16_CLK_SYS_CFG & 0x1f));
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
__HIGH_CODE
void SYS_DisableAllIrq(uint32_t *pirqv)
{
    *pirqv = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    PFIC->IRER[0] = 0xffffffff;
    PFIC->IRER[1] = 0xffffffff;
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
__HIGH_CODE
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
    R16_INT32K_TUNE = 0xFFFF;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    sys_safe_access_disable();
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
#if(FREQ_SYS == 78000000)
    i = t * 13;
#elif(FREQ_SYS == 62400000)
    i = ((uint32_t)t * 78) / 5;
#elif(FREQ_SYS == 52000000)
    i = t * 13;
#elif(FREQ_SYS == 39000000)
    i = (t * 13)>>1;
#elif(FREQ_SYS == 26000000)
    i = (t * 13)>>1;
#elif(FREQ_SYS == 24000000)
    i = t * 6;
#elif(FREQ_SYS == 19500000)
    i = (t * 13)>>2;
#elif(FREQ_SYS == 32000000)
    i = t << 3;
#elif(FREQ_SYS == 16000000)
    i = t << 2;
#elif(FREQ_SYS == 13000000)
    i = (t * 13)>>2;
#elif(FREQ_SYS == 8000000)
    i = t << 1;
#elif(FREQ_SYS == 4000000)
    i = t;
#elif(FREQ_SYS == 2000000)
    i = t >> 1;
#elif(FREQ_SYS == 1000000)
    i = t >> 2;
#else
    i = t;
#endif

#if((FREQ_SYS == 78000000)||\
    (FREQ_SYS == 39000000)||\
    (FREQ_SYS == 19500000))
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
#if DEBUG == Debug_UART0
        while(R8_UART0_TFC == UART_FIFO_SIZE);                  /* �ȴ����ݷ��� */
        R8_UART0_THR = *buf++; /* �������� */
#elif DEBUG == Debug_UART1
        while(R8_UART1_TFC == UART_FIFO_SIZE);                  /* �ȴ����ݷ��� */
        R8_UART1_THR = *buf++; /* �������� */
#elif DEBUG == Debug_UART2
        while(R8_UART2_TFC == UART_FIFO_SIZE);                  /* �ȴ����ݷ��� */
        R8_UART2_THR = *buf++; /* �������� */
#elif DEBUG == Debug_UART3       
        while(R8_UART3_TFC == UART_FIFO_SIZE);                  /* �ȴ����ݷ��� */
        R8_UART3_THR = *buf++; /* �������� */
#endif
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

/*********************************************************************
 * @fn      IWDG_KR_Set
 *
 * @brief   �������Ź�/���������/ι��/��װ�ؼ���ֵ
 *
 * @param   kr     - IWDG_PR
 *
 * @return  none
 */
void IWDG_KR_Set(IWDG_KR_Key kr)
{
    R32_IWDG_KR = kr;
}

/*********************************************************************
 * @fn      IWDG_PR_Set
 *
 * @brief   ����Ԥ��Ƶ���ر�д����λ��Ч
 *
 * @param   pr
 *
 * @return  none
 */
uint8_t IWDG_PR_Set(IWDG_32K_PR pr)
{
    if(IWDG_WR_Protect())   return 1;
    else
    {
        R32_IWDG_CFG |= (pr << 12);
    }
    return 0;
}

/*********************************************************************
 * @fn      IWDG_RLR_Set
 *
 * @brief   ���ü�������װ��ֵ���ر�д����λ��Ч
 *
 * @param   rlr
 *
 * @return  none
 */
uint8_t IWDG_RLR_Set(uint16_t rlr)
{
    uint32_t cfg;

    if(IWDG_WR_Protect())   return 1;
    else
    {
        cfg = R32_IWDG_CFG;
        cfg = (R32_IWDG_CFG & ~0xFFF) | (rlr & 0xFFF);
        R32_IWDG_CFG = cfg;
    }
    return 0;
}

/*********************************************************************
 * @fn      IWDG_FollowCoreStop
 *
 * @brief   �������Ź����������ں�ֹͣʹ�ܣ����ڵ���ģʽ����Ч
 *
 * @param   s       - �Ƿ�ʹ��
 *
 * @return  none
 */
uint8_t IWDG_FollowCoreStop(FunctionalState s)
{
    if(IWDG_WR_Protect())   return 1;
    else
    {
        if(s == DISABLE)
        {
            R32_IWDG_CFG &= ~(1<<29);
        }
        else
        {
            R32_IWDG_CFG |= (1<<29);
        }
    }
    return 0;
}

/*********************************************************************
 * @fn      IWDG_Enable
 *
 * @brief   �������Ź�ʹ��
 *
 * @param   pr     - Ԥ��Ƶ
 *          rlr    - ��������װ��ֵ�����ֵΪ0xFFF
 *
 * @return  none
 */
uint8_t IWDG_Enable(IWDG_32K_PR pr, uint16_t rlr)
{
    uint8_t state;

    sys_safe_access_enable();
    R8_SAFE_LRST_CTRL |= RB_IWDG_RST_EN;  //���Ź���λΪ�ϵ縴λ������Ϊȫ�ָ�λ���豣֤LSI�����ر�
    sys_safe_access_disable();

    IWDG_KR_Set(KEY_START_IWDG);
    IWDG_KR_Set(KEY_UNPROTECT);
    state = IWDG_PR_Set(pr);
    if(state)  return 1;
    state = IWDG_RLR_Set(rlr);
    if(state)  return 1;

    return 0;
}

/*********************************************************************
 * @fn      IWDG_Feed
 *
 * @brief   ϵͳ���붨����װ�ؿ��Ź�����ֵ�Է�ֹ��λ
 *
 * @param   none
 *
 * @return  none
 */
void IWDG_Feed(void)
{
    IWDG_KR_Set(KEY_RELOADING_COUNT);
}

