/********************************** (C) COPYRIGHT *******************************
* File Name          : system_ch32v4x7.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/12/01
* Description        : CH32V4x7 Device Peripheral Access Layer System Source File.
*                      For HSE = 25Mhz
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v4x7.h" 

/* 
* Uncomment the line corresponding to the desired System clock (SYSCLK) frequency (after 
* reset the HSI is used as SYSCLK source).
* If none of the define below is enabled, the HSI is used as System clock source. 
*/
// #define SYSCLK_HCLK_HSE    HSE_VALUE
// #define SYSCLK_120MHz_HCLK_60MHz_HSE   120000000
// #define SYSCLK_240MHz_HCLK_120MHz_HSE  240000000  
// #define SYSCLK_350MHz_HCLK_175MHz_HSE  350000000 
#define SYSCLK_400MHz_HCLK_200MHz_HSE  400000000 
// #define SYSCLK_HCLK_HSI    HSI_VALUE
// #define SYSCLK_120MHz_HCLK_60MHz_HSI   120000000
// #define SYSCLK_240MHz_HCLK_120MHz_HSI  240000000 
// #define SYSCLK_350MHz_HCLK_175MHz_HSI  350000000
// #define SYSCLK_400MHz_HCLK_200MHz_HSI  400000000  

/*Only suitable for commercial applications, with a temperature not exceeding 70 °C and good heat dissipation*/
/* 
// #define SYSCLK_480MHz_HCLK_240MHz_HSE    480000000
// #define SYSCLK_480MHz_HCLK_240MHz_HSI    480000000 
*/

/* Clock Definitions */
uint32_t HCLKClock;
#ifdef SYSCLK_HCLK_HSE
uint32_t SystemClock         = SYSCLK_HCLK_HSE;                      /* System Clock Frequency */
uint32_t SystemCoreClock     = SYSCLK_HCLK_HSE;        
#elif defined SYSCLK_120MHz_HCLK_60MHz_HSE
uint32_t SystemClock         = SYSCLK_120MHz_HCLK_60MHz_HSE;         /* System Clock Frequency */
uint32_t SystemCoreClock     = SYSCLK_120MHz_HCLK_60MHz_HSE >> 1;
#elif defined SYSCLK_240MHz_HCLK_120MHz_HSE
uint32_t SystemClock         = SYSCLK_240MHz_HCLK_120MHz_HSE;        /* System Clock Frequency */
uint32_t SystemCoreClock     = SYSCLK_240MHz_HCLK_120MHz_HSE >> 1;
#elif defined SYSCLK_350MHz_HCLK_175MHz_HSE
uint32_t SystemClock         = SYSCLK_350MHz_HCLK_175MHz_HSE;        /* System Clock Frequency */
uint32_t SystemCoreClock     = SYSCLK_350MHz_HCLK_175MHz_HSE >> 1;
#elif defined SYSCLK_400MHz_HCLK_200MHz_HSE
uint32_t SystemClock         = SYSCLK_400MHz_HCLK_200MHz_HSE;        /* System Clock Frequency */
uint32_t SystemCoreClock     = SYSCLK_400MHz_HCLK_200MHz_HSE >> 1;
#elif defined SYSCLK_480MHz_HCLK_240MHz_HSE
uint32_t SystemClock         = SYSCLK_480MHz_HCLK_240MHz_HSE;        /* System Clock Frequency */
uint32_t SystemCoreClock     = SYSCLK_480MHz_HCLK_240MHz_HSE >> 1;
#elif defined SYSCLK_120MHz_HCLK_60MHz_HSI
uint32_t SystemClock         = SYSCLK_120MHz_HCLK_60MHz_HSI;         /* System Clock Frequency  */
uint32_t SystemCoreClock     = SYSCLK_120MHz_HCLK_60MHz_HSI >> 1;
#elif defined SYSCLK_240MHz_HCLK_120MHz_HSI
uint32_t SystemClock         = SYSCLK_240MHz_HCLK_120MHz_HSI;        /* System Clock Frequency */
uint32_t SystemCoreClock     = SYSCLK_240MHz_HCLK_120MHz_HSI >> 1;
#elif defined SYSCLK_350MHz_HCLK_175MHz_HSI
uint32_t SystemClock         = SYSCLK_350MHz_HCLK_175MHz_HSI;        /* System Clock Frequency */
uint32_t SystemCoreClock     = SYSCLK_350MHz_HCLK_175MHz_HSI >> 1;
#elif defined SYSCLK_400MHz_HCLK_200MHz_HSI
uint32_t SystemClock         = SYSCLK_400MHz_HCLK_200MHz_HSI;        /* System Clock Frequency  */
uint32_t SystemCoreClock     = SYSCLK_400MHz_HCLK_200MHz_HSI >> 1;
#elif defined SYSCLK_480MHz_HCLK_240MHz_HSI
uint32_t SystemClock         = SYSCLK_480MHz_HCLK_240MHz_HSI;        /* System Clock Frequency  */
uint32_t SystemCoreClock     = SYSCLK_480MHz_HCLK_240MHz_HSI >> 1;
#else
uint32_t SystemClock         = HSI_VALUE;                            /* System Clock Frequency  */
uint32_t SystemCoreClock     = HSI_VALUE;   

#endif

static __I uint8_t HBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static __I uint8_t PLLMULTB[32] = {8,9,10,11,12,25,13,27,14,29,15,31,16,33,17,35,18,37,19,39,20,21,22,24,26,28,30,32,34,36,38,40};

/* system_private_function_proto_types */
static void SetSysClock(void);

#ifdef SYSCLK_HCLK_HSE
static void SetSYSCLK_HCLK_HSE( void );
#elif defined SYSCLK_120MHz_HCLK_60MHz_HSE
static void SetSYSCLK_120MHz_HCLK_60MHz_HSE( void );
#elif defined SYSCLK_240MHz_HCLK_120MHz_HSE
static void SetSYSCLK_240MHz_HCLK_120MHz_HSE( void );
#elif defined SYSCLK_350MHz_HCLK_175MHz_HSE
static void SetSYSCLK_350MHz_HCLK_175MHz_HSE( void );
#elif defined SYSCLK_400MHz_HCLK_200MHz_HSE
static void SetSYSCLK_400MHz_HCLK_200MHz_HSE( void );
#elif defined SYSCLK_480MHz_HCLK_240MHz_HSE
static void SetSYSCLK_480MHz_HCLK_240MHz_HSE( void );
#elif defined SYSCLK_120MHz_HCLK_60MHz_HSI
static void SetSYSCLK_120MHz_HCLK_60MHz_HSI( void );
#elif defined SYSCLK_240MHz_HCLK_120MHz_HSI
static void SetSYSCLK_240MHz_HCLK_120MHz_HSI( void );
#elif defined SYSCLK_350MHz_HCLK_175MHz_HSI
static void SetSYSCLK_350MHz_HCLK_175MHz_HSI( void );
#elif defined SYSCLK_400MHz_HCLK_200MHz_HSI
static void SetSYSCLK_400MHz_HCLK_200MHz_HSI( void );
#elif defined SYSCLK_480MHz_HCLK_240MHz_HSI
static void SetSYSCLK_480MHz_HCLK_240MHz_HSI( void );
#else
static void SetSYSCLK_HCLK_HSI( void );
#endif

/*********************************************************************
 * @fn      SystemInit
 *
 * @brief   Setup the microcontroller system Initialize the Embedded Flash Interface,
 *        the PLL and update the SystemClock variable.
 *
 * @return  none
 */
void SystemInit (void)
{
  RCC->CTLR |= (uint32_t)0x00000001;
  RCC->CFGR0 &= (uint32_t)0x80FF0000;
  while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x00)
  {
  }

  RCC->CTLR &= (uint32_t)0xFEA6FFFF;
  RCC->CTLR &= (uint32_t)0xFFFBFFFF;

  RCC->CFGR0 &= (uint32_t)0xFFC0FFFF;
  RCC->CFGR2 &= (uint32_t)0xFFFFEFFF;
  RCC->CFGR2 &= (uint32_t)0x03F11800;

  RCC->CFGR0 |= (uint32_t)(0x20000000);
  RCC->CFGR3 = 0x00000000;
  RCC->INTR = 0x008F0000;   

  SetSysClock();
}

/*********************************************************************
 * @fn      SystemCoreClockUpdate
 *
 * @brief   Update SystemCoreClock variable according to Clock Register Values.
 *
 * @return  none
 */
void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0, tmp1 = 0, tmp2 = 0, pllmull = 0, pllsource = 0, presc = 0;
  uint8_t Plldiv2 = 0;

  tmp = RCC->CFGR0 & RCC_SWS;
  tmp1 = RCC->CFGR2 & RCC_SYSPLLSRC;

  switch (tmp)
  {
    case 0x00:
      SystemClock = HSI_VALUE;
      break;
    case 0x04:  
      SystemClock = HSE_VALUE;
      break;
    case 0x08: 
      pllsource = RCC->CFGR0 & RCC_PLLSRC; 
      presc = ((RCC->CFGR2 & RCC_SYSPLLDIV)>>8) + 1;
      tmp2 = (RCC->CFGR0 & RCC_PLLMULL)>>17;
      pllmull = PLLMULTB[tmp2];
      if((tmp2 > 4) && (tmp2 < 20) && ((tmp2 % 2) != 0))
      {
        Plldiv2 = 1;
      }

      switch (tmp1) 
      {
        case RCC_SYSPLLSRC_USBHSPLL:
          SystemClock = 480000000;
        break;

        case RCC_SYSPLLSRC_ETHPLL:
          SystemClock = 500000000;
        break;

        case RCC_SYSPLLSRC_USBHSPLL_MUL2_DIV3:
          SystemClock = 320000000;
        break;

        case RCC_SYSPLLSRC_ETHPLL_MUL2_DIV3:
          SystemClock = 333000000;
        break;

        case RCC_SYSPLLSRC_PLL:

          if(pllsource == 0x00) //HSI
          {
            SystemClock = HSI_VALUE*pllmull;
          }
          else 
          {
            SystemClock = HSE_VALUE*pllmull;
          }
          SystemClock /= (1 + Plldiv2);
        break;

        default:
        break;
      }
      SystemClock /= presc;
      break;

      default:
      SystemClock = HSI_VALUE;
      break;
  }
    tmp = HBPrescTable[((RCC->CFGR0 & RCC_HPRE) >> 4)];
    SystemCoreClock = SystemClock >> tmp; 
    HCLKClock =  SystemCoreClock;
}

/*********************************************************************
 * @fn      SetSysClock
 *
 * @brief   Configures the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClock(void)
{
    GPIO_IPD_Unused();
#ifdef SYSCLK_HCLK_HSE
    SetSYSCLK_HCLK_HSE();
#elif defined SYSCLK_120MHz_HCLK_60MHz_HSE
    SetSYSCLK_120MHz_HCLK_60MHz_HSE();
#elif defined SYSCLK_240MHz_HCLK_120MHz_HSE
    SetSYSCLK_240MHz_HCLK_120MHz_HSE();
#elif defined SYSCLK_350MHz_HCLK_175MHz_HSE
    SetSYSCLK_350MHz_HCLK_175MHz_HSE();
#elif defined SYSCLK_400MHz_HCLK_200MHz_HSE
    SetSYSCLK_400MHz_HCLK_200MHz_HSE();    
#elif defined SYSCLK_480MHz_HCLK_240MHz_HSE
    SetSYSCLK_480MHz_HCLK_240MHz_HSE();    
#elif defined SYSCLK_120MHz_HCLK_60MHz_HSI
    SetSYSCLK_120MHz_HCLK_60MHz_HSI();
#elif defined SYSCLK_240MHz_HCLK_120MHz_HSI
    SetSYSCLK_240MHz_HCLK_120MHz_HSI();
#elif defined SYSCLK_350MHz_HCLK_175MHz_HSI
    SetSYSCLK_350MHz_HCLK_175MHz_HSI();
#elif defined SYSCLK_400MHz_HCLK_200MHz_HSI
    SetSYSCLK_400MHz_HCLK_200MHz_HSI();  
#elif defined SYSCLK_480MHz_HCLK_240MHz_HSI
    SetSYSCLK_480MHz_HCLK_240MHz_HSI();  
#else 
    SetSYSCLK_HCLK_HSI();
#endif
 
/* 
  * If none of the define above is enabled, the HSI is used as System clock
  * source (default after reset) 
  */
}


#ifdef SYSCLK_HCLK_HSE

/*********************************************************************
 * @fn      SetSYSCLK_HCLK_HSE
 *
 * @brief   Sets HSE as System clock source and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSYSCLK_HCLK_HSE(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0, FLASH_Temp = 0;
   
  RCC->CTLR |= ((uint32_t)RCC_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CTLR & RCC_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CTLR & RCC_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;      
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;
    
    /* Select FLASH clock frequency*/
    FLASH->KEYR = ((uint32_t)0x45670123);
    FLASH->KEYR = ((uint32_t)0xCDEF89AB);
    FLASH->MODEKEYR = ((uint32_t)0x45670123);
    FLASH->MODEKEYR = ((uint32_t)0xCDEF89AB);

    FLASH_Temp = FLASH->CTLR ;
    FLASH_Temp &= ~((uint32_t)FLASH_CTLR_SCKMOD);
    FLASH_Temp |= FLASH_CTLR_SCKMOD_Div1;
    FLASH->CTLR = FLASH_Temp;
    FLASH->CTLR |= ((uint32_t)0x00008080);

    /* Select HSE as system clock source */
    RCC->CFGR0 |= (uint32_t)RCC_SW_HSE;    

    /* Wait till HSE is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x04)
    {
    }
  }
  else
  { 
      /* 
       * If HSE fails to start-up, the application will have wrong clock
       * configuration. User can add here some code to deal with this error 
       */
  }  
}

#elif defined SYSCLK_120MHz_HCLK_60MHz_HSE

/*********************************************************************
 * @fn      SetSYSCLK_120MHz_HCLK_60MHz_HSE
 *
 * @brief   Sets System clock frequency to 120MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSYSCLK_120MHz_HCLK_60MHz_HSE(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0 , FLASH_Temp = 0;

    RCC->CTLR |= ((uint32_t)RCC_HSEON);

    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
      HSEStatus = RCC->CTLR & RCC_HSERDY;
      StartUpCounter++;
    } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CTLR & RCC_HSERDY) != RESET)
    {
      HSEStatus = (uint32_t)0x01;
    }
    else
    {
      HSEStatus = (uint32_t)0x00;
    }

    if (HSEStatus == (uint32_t)0x01)
    {
      /* HCLK = SYSCLK / 2 */
      RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;
      /* PCLK2 = HCLK */
      RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
      /* PCLK1 = HCLK */
      RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

      /* PLL configuration: HCLK = (USBHSPLL(480M) / 4) / 2 = 60 MHz */
      RCC->CFGR2 |= (uint32_t)(RCC_USBHSPLLSRC_HSE | RCC_USBHSPLLCLK_25M);
      /* Enable USBHSPLL */
      RCC->CTLR |= RCC_USBHSPLLON;
      /* Wait till PLL is ready */
      while((RCC->CTLR & RCC_USBHSPLLRDY) == 0)
      {
      }

      RCC->CFGR2 |= (uint32_t)(RCC_SYSPLLSRC_USBHSPLL | RCC_SYSPLLDIV4);
      /* Wait till USBHSPLL clock is used as SYSPLLSRC clock source */
      while ((RCC->CFGR2 & (uint32_t)RCC_SYSPLLSRC) != (uint32_t)RCC_SYSPLLSRC_USBHSPLL)
      {
      }     

      /* Select FLASH clock frequency*/
      FLASH->KEYR = ((uint32_t)0x45670123);
      FLASH->KEYR = ((uint32_t)0xCDEF89AB);
      FLASH->MODEKEYR = ((uint32_t)0x45670123);
      FLASH->MODEKEYR = ((uint32_t)0xCDEF89AB);

      FLASH_Temp = FLASH->CTLR ;
      FLASH_Temp &= ~((uint32_t)FLASH_CTLR_SCKMOD);
      FLASH_Temp |= FLASH_CTLR_SCKMOD_Div1;
      FLASH->CTLR = FLASH_Temp;
      FLASH->CTLR |= ((uint32_t)0x00008080);

      /* Select PLL as system clock source */
      RCC->CFGR2 |= (uint32_t)RCC_SYSPLL_GATE;
      RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
      /* Wait till PLL is used as system clock source */
      while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
      {
      }
    }
    else
    {
      /*
       * If HSE fails to start-up, the application will have wrong clock
       * configuration. User can add here some code to deal with this error
       */
    }
}

#elif defined SYSCLK_240MHz_HCLK_120MHz_HSE

/*********************************************************************
 * @fn      SetSYSCLK_240MHz_HCLK_120MHz_HSE
 *
 * @brief   Sets System clock frequency to 240MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSYSCLK_240MHz_HCLK_120MHz_HSE(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0 , FLASH_Temp = 0;

    RCC->CTLR |= ((uint32_t)RCC_HSEON);

    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
      HSEStatus = RCC->CTLR & RCC_HSERDY;
      StartUpCounter++;
    } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CTLR & RCC_HSERDY) != RESET)
    {
      HSEStatus = (uint32_t)0x01;
    }
    else
    {
      HSEStatus = (uint32_t)0x00;
    }

    if (HSEStatus == (uint32_t)0x01)
    {
      /* HCLK = SYSCLK / 2 */
      RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;
      /* PCLK2 = HCLK */
      RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
      /* PCLK1 = HCLK */
      RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

      /* PLL configuration: HCLK = (USBHSPLL(480M) / 2) / 2 = 120 MHz */
      RCC->CFGR2 |= (uint32_t)(RCC_USBHSPLLSRC_HSE | RCC_USBHSPLLCLK_25M);
      /* Enable USBHSPLL */
      RCC->CTLR |= RCC_USBHSPLLON;
      /* Wait till PLL is ready */
      while((RCC->CTLR & RCC_USBHSPLLRDY) == 0)
      {
      }

      RCC->CFGR2 |= (uint32_t)(RCC_SYSPLLSRC_USBHSPLL | RCC_SYSPLLDIV2);
      /* Wait till USBHSPLL clock is used as SYSPLLSRC clock source */
      while ((RCC->CFGR2 & (uint32_t)RCC_SYSPLLSRC) != (uint32_t)RCC_SYSPLLSRC_USBHSPLL)
      {
      }     

      /* Select FLASH clock frequency*/
      FLASH->KEYR = ((uint32_t)0x45670123);
      FLASH->KEYR = ((uint32_t)0xCDEF89AB);
      FLASH->MODEKEYR = ((uint32_t)0x45670123);
      FLASH->MODEKEYR = ((uint32_t)0xCDEF89AB);

      FLASH_Temp = FLASH->CTLR ;
      FLASH_Temp &= ~((uint32_t)FLASH_CTLR_SCKMOD);
      FLASH_Temp |= FLASH_CTLR_SCKMOD_Div2;
      FLASH->CTLR = FLASH_Temp;
      FLASH->CTLR |= ((uint32_t)0x00008080);

      /* Select PLL as system clock source */
      RCC->CFGR2 |= (uint32_t)RCC_SYSPLL_GATE;
      RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
      /* Wait till PLL is used as system clock source */
      while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
      {
      }
    }
    else
    {
      /*
       * If HSE fails to start-up, the application will have wrong clock
       * configuration. User can add here some code to deal with this error
       */
    }
}

#elif defined SYSCLK_350MHz_HCLK_175MHz_HSE

/*********************************************************************
 * @fn      SetSYSCLK_350MHz_HCLK_175MHz_HSE
 *
 * @brief   Sets System clock frequency to 350MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSYSCLK_350MHz_HCLK_175MHz_HSE(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0 , FLASH_Temp = 0;

    RCC->CTLR |= ((uint32_t)RCC_HSEON);

    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
      HSEStatus = RCC->CTLR & RCC_HSERDY;
      StartUpCounter++;
    } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CTLR & RCC_HSERDY) != RESET)
    {
      HSEStatus = (uint32_t)0x01;
    }
    else
    {
      HSEStatus = (uint32_t)0x00;
    }

    if (HSEStatus == (uint32_t)0x01)
    {
      /* HCLK = SYSCLK /2 */
      RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;
      /* PCLK2 = HCLK */
      RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
      /* PCLK1 = HCLK */
      RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

      /* PLL configuration: HCLK = (HSE * 14 / 1) / 2 = 175 MHz */
      RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSE | RCC_PLLMULL14);

      /* Enable PLL */
      RCC->CTLR |= RCC_PLLON;
      /* Wait till PLL is ready */
      while((RCC->CTLR & RCC_PLLRDY) == 0)
      {
      }

      RCC->CFGR2 |= (uint32_t)(RCC_SYSPLLSRC_PLL | RCC_SYSPLLDIV1);
      /* Wait till PLL clock is used as SYSPLLSRC clock source */
      while ((RCC->CFGR2 & (uint32_t)RCC_SYSPLLSRC) != (uint32_t)RCC_SYSPLLSRC_PLL)
      {
      }     

      /* Select FLASH clock frequency*/
      FLASH->KEYR = ((uint32_t)0x45670123);
      FLASH->KEYR = ((uint32_t)0xCDEF89AB);
      FLASH->MODEKEYR = ((uint32_t)0x45670123);
      FLASH->MODEKEYR = ((uint32_t)0xCDEF89AB);

      FLASH_Temp = FLASH->CTLR ;
      FLASH_Temp &= ~((uint32_t)FLASH_CTLR_SCKMOD);
      FLASH_Temp |= FLASH_CTLR_SCKMOD_Div2;
      FLASH->CTLR = FLASH_Temp;
      FLASH->CTLR |= ((uint32_t)0x00008080);

      /* Select PLL as system clock source */
      RCC->CFGR2 |= (uint32_t)RCC_SYSPLL_GATE;
      RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
      /* Wait till PLL is used as system clock source */
      while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
      {
      }
    }
    else
    {
      /*
       * If HSE fails to start-up, the application will have wrong clock
       * configuration. User can add here some code to deal with this error
       */
    }
}

#elif defined SYSCLK_400MHz_HCLK_200MHz_HSE

/*********************************************************************
 * @fn      SetSYSCLK_400MHz_HCLK_200MHz_HSE
 *
 * @brief   Sets System clock frequency to 400MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSYSCLK_400MHz_HCLK_200MHz_HSE(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0 , FLASH_Temp = 0;

    RCC->CTLR |= ((uint32_t)RCC_HSEON);

    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
      HSEStatus = RCC->CTLR & RCC_HSERDY;
      StartUpCounter++;
    } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CTLR & RCC_HSERDY) != RESET)
    {
      HSEStatus = (uint32_t)0x01;
    }
    else
    {
      HSEStatus = (uint32_t)0x00;
    }

    if (HSEStatus == (uint32_t)0x01)
    {
      /* HCLK = SYSCLK /2 */
      RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;
      /* PCLK2 = HCLK */
      RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
      /* PCLK1 = HCLK */
      RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

      /* PLL configuration: HCLK = (HSE * 16) / 2 = 200 MHz */
      RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSE | RCC_PLLMULL16);

      /* Enable PLL */
      RCC->CTLR |= RCC_PLLON;
      /* Wait till PLL is ready */
      while((RCC->CTLR & RCC_PLLRDY) == 0)
      {
      }

      RCC->CFGR2 |= (uint32_t)(RCC_SYSPLLSRC_PLL | RCC_SYSPLLDIV1);
      /* Wait till PLL clock is used as SYSPLLSRC clock source */
      while ((RCC->CFGR2 & (uint32_t)RCC_SYSPLLSRC) != (uint32_t)RCC_SYSPLLSRC_PLL)
      {
      }     

      /* Select FLASH clock frequency*/
      FLASH->KEYR = ((uint32_t)0x45670123);
      FLASH->KEYR = ((uint32_t)0xCDEF89AB);
      FLASH->MODEKEYR = ((uint32_t)0x45670123);
      FLASH->MODEKEYR = ((uint32_t)0xCDEF89AB);

      FLASH_Temp = FLASH->CTLR ;
      FLASH_Temp &= ~((uint32_t)FLASH_CTLR_SCKMOD);
      FLASH_Temp |= FLASH_CTLR_SCKMOD_Div2;
      FLASH->CTLR = FLASH_Temp;
      FLASH->CTLR |= ((uint32_t)0x00008080);

      /* Select PLL as system clock source */
      RCC->CFGR2 |= (uint32_t)RCC_SYSPLL_GATE;
      RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
      /* Wait till PLL is used as system clock source */
      while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
      {
      }
    }
    else
    {
      /*
       * If HSE fails to start-up, the application will have wrong clock
       * configuration. User can add here some code to deal with this error
       */
    }
}

#elif defined SYSCLK_480MHz_HCLK_240MHz_HSE

/*********************************************************************
 * @fn      SetSYSCLK_480MHz_HCLK_240MHz_HSE
 *
 * @brief   Sets System clock frequency to 480MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSYSCLK_480MHz_HCLK_240MHz_HSE(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0 , FLASH_Temp = 0, tmpreg = 0;

    /* Set VDDK to 1.23V */
    RCC->PB1PCENR |= RCC_PB1Periph_PWR;
    tmpreg = PWR->CTLR; 
    tmpreg &= ~PWR_CTLR_LDOVDDK;
    tmpreg |= PWR_VDDK_Level4;
    PWR->CTLR = tmpreg;

    RCC->CTLR |= ((uint32_t)RCC_HSEON);

    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
      HSEStatus = RCC->CTLR & RCC_HSERDY;
      StartUpCounter++;
    } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CTLR & RCC_HSERDY) != RESET)
    {
      HSEStatus = (uint32_t)0x01;
    }
    else
    {
      HSEStatus = (uint32_t)0x00;
    }

    if (HSEStatus == (uint32_t)0x01)
    {
      /* HCLK = SYSCLK / 2 */
      RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;
      /* PCLK2 = HCLK */
      RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
      /* PCLK1 = HCLK */
      RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

      /* PLL configuration: HCLK = (USBHSPLL(480M) / 1) / 2 = 120 MHz */
      RCC->CFGR2 |= (uint32_t)(RCC_USBHSPLLSRC_HSE | RCC_USBHSPLLCLK_25M);
      /* Enable USBHSPLL */
      RCC->CTLR |= RCC_USBHSPLLON;
      /* Wait till PLL is ready */
      while((RCC->CTLR & RCC_USBHSPLLRDY) == 0)
      {
      }

      RCC->CFGR2 |= (uint32_t)(RCC_SYSPLLSRC_USBHSPLL | RCC_SYSPLLDIV1);
      /* Wait till USBHSPLL clock is used as SYSPLLSRC clock source */
      while ((RCC->CFGR2 & (uint32_t)RCC_SYSPLLSRC) != (uint32_t)RCC_SYSPLLSRC_USBHSPLL)
      {
      }     

      /* Select FLASH clock frequency*/
      FLASH->KEYR = ((uint32_t)0x45670123);
      FLASH->KEYR = ((uint32_t)0xCDEF89AB);
      FLASH->MODEKEYR = ((uint32_t)0x45670123);
      FLASH->MODEKEYR = ((uint32_t)0xCDEF89AB);

      FLASH_Temp = FLASH->CTLR ;
      FLASH_Temp &= ~((uint32_t)FLASH_CTLR_SCKMOD);
      FLASH_Temp |= FLASH_CTLR_SCKMOD_Div4;
      FLASH->CTLR = FLASH_Temp;
      FLASH->CTLR |= ((uint32_t)0x00008080);

      /* Select PLL as system clock source */
      RCC->CFGR2 |= (uint32_t)RCC_SYSPLL_GATE;
      RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
      /* Wait till PLL is used as system clock source */
      while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
      {
      }
    }
    else
    {
      /*
       * If HSE fails to start-up, the application will have wrong clock
       * configuration. User can add here some code to deal with this error
       */
    }
}

#elif defined SYSCLK_120MHz_HCLK_60MHz_HSI

/*********************************************************************
 * @fn      SetSYSCLK_120MHz_HCLK_60MHz_HSI
 *
 * @brief   Sets System clock frequency to 120MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSYSCLK_120MHz_HCLK_60MHz_HSI(void)
{
    __IO uint32_t FLASH_Temp = 0;

    /* HCLK = SYSCLK / 2 */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: HCLK = (HSI * 12 / 2) / 2 = 60 MHz */
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI | RCC_PLLMULL12);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }

    RCC->CFGR2 |= (uint32_t)(RCC_SYSPLLSRC_PLL | RCC_SYSPLLDIV2);
    /* Wait till PLL clock is used as SYSPLLSRC clock source */
    while ((RCC->CFGR2 & (uint32_t)RCC_SYSPLLSRC) != (uint32_t)RCC_SYSPLLSRC_PLL)
    {
    }     

    /* Select FLASH clock frequency*/
    FLASH->KEYR = ((uint32_t)0x45670123);
    FLASH->KEYR = ((uint32_t)0xCDEF89AB);
    FLASH->MODEKEYR = ((uint32_t)0x45670123);
    FLASH->MODEKEYR = ((uint32_t)0xCDEF89AB);

    FLASH_Temp = FLASH->CTLR ;
    FLASH_Temp &= ~((uint32_t)FLASH_CTLR_SCKMOD);
    FLASH_Temp |= FLASH_CTLR_SCKMOD_Div1;
    FLASH->CTLR = FLASH_Temp;
    FLASH->CTLR |= ((uint32_t)0x00008080);

    /* Select PLL as system clock source */
    RCC->CFGR2 |= (uint32_t)RCC_SYSPLL_GATE;
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
}

#elif defined SYSCLK_240MHz_HCLK_120MHz_HSI

/*********************************************************************
 * @fn      SetSYSCLK_240MHz_HCLK_120MHz_HSI
 *
 * @brief   Sets System clock frequency to 240MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSYSCLK_240MHz_HCLK_120MHz_HSI(void)
{
    __IO uint32_t FLASH_Temp = 0;

    /* HCLK = SYSCLK / 2 */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: HCLK = HSI * 12 / 1 / 2 = 120 MHz */
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI | RCC_PLLMULL12);
    
    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }

    RCC->CFGR2 |= (uint32_t)(RCC_SYSPLLSRC_PLL | RCC_SYSPLLDIV1);
    /* Wait till PLL clock is used as SYSPLLSRC clock source */
    while ((RCC->CFGR2 & (uint32_t)RCC_SYSPLLSRC) != (uint32_t)RCC_SYSPLLSRC_PLL)
    {
    }     

    /* Select FLASH clock frequency*/
    FLASH->KEYR = ((uint32_t)0x45670123);
    FLASH->KEYR = ((uint32_t)0xCDEF89AB);
    FLASH->MODEKEYR = ((uint32_t)0x45670123);
    FLASH->MODEKEYR = ((uint32_t)0xCDEF89AB);

    FLASH_Temp = FLASH->CTLR ;
    FLASH_Temp &= ~((uint32_t)FLASH_CTLR_SCKMOD);
    FLASH_Temp |= FLASH_CTLR_SCKMOD_Div2;
    FLASH->CTLR = FLASH_Temp;
    FLASH->CTLR |= ((uint32_t)0x00008080);

    /* Select PLL as system clock source */
    RCC->CFGR2 |= (uint32_t)RCC_SYSPLL_GATE;
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
}

#elif defined SYSCLK_350MHz_HCLK_175MHz_HSI

/*********************************************************************
 * @fn      SetSYSCLK_350MHz_HCLK_175MHz_HSI
 *
 * @brief   Sets System clock frequency to 350MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSYSCLK_350MHz_HCLK_175MHz_HSI(void)
{
    __IO uint32_t FLASH_Temp = 0;

    /* HCLK = SYSCLK / 2 */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: HCLK = HSI * 17.5 / 1 / 2 = 175 MHz */
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI | RCC_PLLMULL17_5);
    
    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }

    RCC->CFGR2 |= (uint32_t)(RCC_SYSPLLSRC_PLL | RCC_SYSPLLDIV1);
    /* Wait till PLL clock is used as SYSPLLSRC clock source */
    while ((RCC->CFGR2 & (uint32_t)RCC_SYSPLLSRC) != (uint32_t)RCC_SYSPLLSRC_PLL)
    {
    }     

    /* Select FLASH clock frequency*/
    FLASH->KEYR = ((uint32_t)0x45670123);
    FLASH->KEYR = ((uint32_t)0xCDEF89AB);
    FLASH->MODEKEYR = ((uint32_t)0x45670123);
    FLASH->MODEKEYR = ((uint32_t)0xCDEF89AB);

    FLASH_Temp = FLASH->CTLR ;
    FLASH_Temp &= ~((uint32_t)FLASH_CTLR_SCKMOD);
    FLASH_Temp |= FLASH_CTLR_SCKMOD_Div2;
    FLASH->CTLR = FLASH_Temp;
    FLASH->CTLR |= ((uint32_t)0x00008080);

    /* Select PLL as system clock source */
    RCC->CFGR2 |= (uint32_t)RCC_SYSPLL_GATE;
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
}

#elif defined SYSCLK_400MHz_HCLK_200MHz_HSI

/*********************************************************************
 * @fn      SetSYSCLK_400MHz_HCLK_200MHz_HSI
 *
 * @brief   Sets System clock frequency to 400MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSYSCLK_400MHz_HCLK_200MHz_HSI(void)
{
    __IO uint32_t FLASH_Temp = 0;

    /* HCLK = SYSCLK / 2 */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: HCLK = HSI * 20 / 1 / 2 = 200 MHz */
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI | RCC_PLLMULL20);
    
    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }

    RCC->CFGR2 |= (uint32_t)(RCC_SYSPLLSRC_PLL | RCC_SYSPLLDIV1);
    /* Wait till PLL clock is used as SYSPLLSRC clock source */
    while ((RCC->CFGR2 & (uint32_t)RCC_SYSPLLSRC) != (uint32_t)RCC_SYSPLLSRC_PLL)
    {
    }     

    /* Select FLASH clock frequency*/
    FLASH->KEYR = ((uint32_t)0x45670123);
    FLASH->KEYR = ((uint32_t)0xCDEF89AB);
    FLASH->MODEKEYR = ((uint32_t)0x45670123);
    FLASH->MODEKEYR = ((uint32_t)0xCDEF89AB);

    FLASH_Temp = FLASH->CTLR ;
    FLASH_Temp &= ~((uint32_t)FLASH_CTLR_SCKMOD);
    FLASH_Temp |= FLASH_CTLR_SCKMOD_Div2;
    FLASH->CTLR = FLASH_Temp;
    FLASH->CTLR |= ((uint32_t)0x00008080);

    /* Select PLL as system clock source */
    RCC->CFGR2 |= (uint32_t)RCC_SYSPLL_GATE;
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
}

#elif defined SYSCLK_480MHz_HCLK_240MHz_HSI

/*********************************************************************
 * @fn      SetSYSCLK_480MHz_HCLK_240MHz_HSI
 *
 * @brief   Sets System clock frequency to 480MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSYSCLK_480MHz_HCLK_240MHz_HSI(void)
{
    __IO uint32_t FLASH_Temp = 0, tmpreg = 0;

    /* Set VDDK to 1.23V */
    RCC->PB1PCENR |= RCC_PB1Periph_PWR;
    tmpreg = PWR->CTLR; 
    tmpreg &= ~PWR_CTLR_LDOVDDK;
    tmpreg |= PWR_VDDK_Level4;
    PWR->CTLR = tmpreg;

    /* HCLK = SYSCLK / 2 */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* PLL configuration: HCLK = HSI * 24 / 1 / 2 = 240 MHz */
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI | RCC_PLLMULL24);
    
    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }

    RCC->CFGR2 |= (uint32_t)(RCC_SYSPLLSRC_PLL | RCC_SYSPLLDIV1);
    /* Wait till PLL clock is used as SYSPLLSRC clock source */
    while ((RCC->CFGR2 & (uint32_t)RCC_SYSPLLSRC) != (uint32_t)RCC_SYSPLLSRC_PLL)
    {
    }     

    /* Select FLASH clock frequency*/
    FLASH->KEYR = ((uint32_t)0x45670123);
    FLASH->KEYR = ((uint32_t)0xCDEF89AB);
    FLASH->MODEKEYR = ((uint32_t)0x45670123);
    FLASH->MODEKEYR = ((uint32_t)0xCDEF89AB);

    FLASH_Temp = FLASH->CTLR ;
    FLASH_Temp &= ~((uint32_t)FLASH_CTLR_SCKMOD);
    FLASH_Temp |= FLASH_CTLR_SCKMOD_Div4;
    FLASH->CTLR = FLASH_Temp;
    FLASH->CTLR |= ((uint32_t)0x00008080);

    /* Select PLL as system clock source */
    RCC->CFGR2 |= (uint32_t)RCC_SYSPLL_GATE;
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
}

#else 

/*********************************************************************
 * @fn      SetSYSCLK_HCLK_HSI
 *
 * @brief   Sets HSI as System clock source and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSYSCLK_HCLK_HSI(void)
{
    __IO uint32_t FLASH_Temp = 0;

    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* Select FLASH clock frequency*/
    FLASH->KEYR = ((uint32_t)0x45670123);
    FLASH->KEYR = ((uint32_t)0xCDEF89AB);
    FLASH->MODEKEYR = ((uint32_t)0x45670123);
    FLASH->MODEKEYR = ((uint32_t)0xCDEF89AB);

    FLASH_Temp = FLASH->CTLR ;
    FLASH_Temp &= ~((uint32_t)FLASH_CTLR_SCKMOD);
    FLASH_Temp |= FLASH_CTLR_SCKMOD_Div1;
    FLASH->CTLR = FLASH_Temp;
    FLASH->CTLR |= ((uint32_t)0x00008080);

    /* Select HSI as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_HSI;
    /* Wait till HSI is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x00)
    {
    }
}

#endif
