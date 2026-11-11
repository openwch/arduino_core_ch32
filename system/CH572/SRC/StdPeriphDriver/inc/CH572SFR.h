/* Define for CH572         */
/* Website:  http://wch.cn  */
/* Email:    tech@wch.cn    */
/* Author:   RDL14 2024.11.22   */
/* V1.0 SpecialFunctionRegister */
/* V1.01 Update breif and define of RAM ROM FLASH PLL  */
/* V1.02 Update interupt define and number for keyscan and encoder  */
/* V1.03 Update breif and define  for PWM_DATA_REG  */
/* V1.04 Update name and breif for RTC PWM LSI */

// multi-blocks: __BASE_TYPE__, __CH572SFR_H__, __CH572USBSFR_H__, __USB_TYPE__...

#ifndef __BASE_TYPE__
#define __BASE_TYPE__

#ifdef __cplusplus
extern "C" {
#endif

/* ********************************************************************************************************************* */
/* Base types & constants */

#ifndef TRUE
#define TRUE                    1
#define FALSE                   0
#endif
#ifndef NULL
#define NULL                    0
#endif

#ifndef VOID
#define VOID                    void
#endif
#ifndef CONST
#define CONST                   const
#endif
#ifndef BOOL
typedef unsigned char           BOOL;
#endif
#ifndef BOOLEAN
typedef unsigned char           BOOLEAN;
#endif
#ifndef CHAR
typedef char                    CHAR;
#endif
#ifndef INT8
typedef char                    INT8;
#endif
#ifndef INT16
typedef short                   INT16;
#endif
#ifndef INT32
typedef long                    INT32;
#endif
#ifndef UINT8
typedef unsigned char           UINT8;
#endif
#ifndef UINT16
typedef unsigned short          UINT16;
#endif
#ifndef UINT32
typedef unsigned long           UINT32;
#endif
#ifndef UINT64
typedef unsigned long long      UINT64;
#endif
#ifndef UINT8V
typedef unsigned char volatile  UINT8V;
#endif
#ifndef UINT16V
typedef unsigned short volatile UINT16V;
#endif
#ifndef UINT32V
typedef unsigned long volatile  UINT32V;
#endif
#ifndef UINT64V
typedef unsigned long long volatile  UINT64V;
#endif

#ifndef PVOID
typedef void                    *PVOID;
#endif
#ifndef PCHAR
typedef char                    *PCHAR;
#endif
#ifndef PCHAR
typedef const char              *PCCHAR;
#endif
#ifndef PINT8
typedef char                    *PINT8;
#endif
#ifndef PINT16
typedef short                   *PINT16;
#endif
#ifndef PINT32
typedef long                    *PINT32;
#endif
#ifndef PUINT8
typedef unsigned char           *PUINT8;
#endif
#ifndef PUINT16
typedef unsigned short          *PUINT16;
#endif
#ifndef PUINT32
typedef unsigned long           *PUINT32;
#endif
#ifndef PUINT8V
typedef volatile unsigned char  *PUINT8V;
#endif
#ifndef PUINT16V
typedef volatile unsigned short *PUINT16V;
#endif
#ifndef PUINT32V
typedef volatile unsigned long  *PUINT32V;
#endif
#ifndef PUINT64V
typedef volatile unsigned long long  *PUINT64V;
#endif

/* ********************************************************************************************************************* */
/* Base macros */

#ifndef min
#define min(a,b)                (((a) < (b)) ? (a) : (b))
#endif
#ifndef max
#define max(a,b)                (((a) > (b)) ? (a) : (b))
#endif

/* Calculate the byte offset of a field in a structure of type */
#define FIELD_OFFSET(Type, Field)    ((UINT16)&(((Type *)0)->Field))

/* Calculate the size of a field in a structure of type */
#define FIELD_SIZE(Type, Field)      (sizeof(((Type *)0)->Field))

/* An expression that yields the type of a field in a struct */
#define FIELD_TYPE(Type, Field)      (((Type *)0)->Field)

/* Return the number of elements in a statically sized array */
#define NUMBER_OF(Array)             (sizeof(Array)/sizeof((Array)[0]))
#define NUMBER_OF_FIELD(Type, Field) (NUMBER_OF(FIELD_TYPE(Type, Field)))

#ifdef __cplusplus
}
#endif

#endif  // __BASE_TYPE__


#ifndef __CH572SFR_H__
#define __CH572SFR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ********************************************************************************************************************* */

// Address Space
//    CODE:   00000000H - 0003BFFFH   240KB
//    BOOT:   0003C000H - 0003DFFFH   8KB
//    INFO:   0003E000H - 0003FFFFH   8KB
//    DATA:   20000000H - 20002FFFH   12KB
//    SFR:    40000000H - 4000FFFFH   64KB
//
//    SFR:    40000000H - 4000FFFFH,  64KB
//      SYS:    +1000H - 17FFH, include base configuration, interrupt, GPIO, etc...
//    FLASH:    +1800H - 23FFH
//      TMR:    +2400H - 27FFH
//     UART:    +3400H - 37FFH
//      SPI:    +4000H - 43FFH
//      I2C:    +4800H - 4BFFH
//     PWMx:    +5000H - 53FFH
//      LCD:    +6000H - 63FFH
//      USB:    +8000H - 83FFH
//      BLE:    +C000H - D3FFH

// Register Bit Attribute / Bit Access Type
//   RF:    Read only for Fixed value
//   RO:    Read Only (internal change)
//   RZ:    Read only with auto clear Zero
//   WO:    Write Only (read zero or different)
//   WA:    Write only under safe Accessing mode (read zero or different)
//   WZ:    Write only with auto clear Zero
//   RW:    Read / Write
//   RWA:   Read / Write under safe Accessing mode
//   RW1:   Read / Write 1 to Clear

/* Register name rule:
   R32_* for 32 bits register (UINT32,ULONG)
   R16_* for 16 bits register (UINT16,USHORT)
   R8_*  for  8 bits register (UINT8,UCHAR)
   RB_*  for bit or bit mask of 8 bit register
   BA_*  for base address point
   b*    for GPIO bit mask
   Others for register address offset */

/* ********************************************************************************************************************* */

/* Independent watch-dog register */
#define R32_IWDG_KR         (*((PUINT32V)0x40001000)) // WO, watch-dog key register
#define R32_IWDG_CFG        (*((PUINT32V)0x40001004)) // RW, watch-dog configuration

/* System: safe accessing register */
#define R32_SAFE_ACCESS     (*((PUINT32V)0x40001040)) // RW, safe accessing
#define R8_SAFE_ACCESS_SIG  (*((PUINT8V)0x40001040))  // WO, safe accessing sign register, must write SAFE_ACCESS_SIG1 then SAFE_ACCESS_SIG2 to enter safe accessing mode
#define  RB_SAFE_ACC_MODE   0x03                      // RO, current safe accessing mode: 11=safe/unlocked (SAM), other=locked (00..01..10..11)
#define  RB_SAFE_ACC_ACT    0x04                      // RO, indicate safe accessing on
#define  RB_SAFE_ACC_TIMER  0xF8                      // RO, safe accessing mode closed
#define SAFE_ACCESS_SIG1    0x57                      // WO: safe accessing sign value step 1
#define SAFE_ACCESS_SIG2    0xA8                      // WO: safe accessing sign value step 2
#define SAFE_ACCESS_SIG0    0x00                      // WO: safe accessing sign value for disable
#define R8_CHIP_ID          (*((PUINT8V)0x40001041))  // RF, chip ID register, always is ID_CH57*
#define R8_SAFE_ACCESS_ID   (*((PUINT8V)0x40001042))  // RF, safe accessing ID register, always 0x0C
#define R8_WDOG_COUNT       (*((PUINT8V)0x40001043))  // RW, watch-dog count, count by clock frequency Fsys/131072

/* System: global configuration register */
#define R32_GLOBAL_CONFIG   (*((PUINT32V)0x40001044)) // RW, global configuration
#define R8_RESET_STATUS     (*((PUINT8V)0x40001044))  // RO, reset status
#define  RB_RESET_FLAG      0x07                      // RO: recent reset flag
#define  RST_FLAG_SW        0x00
#define  RST_FLAG_RPOR      0x01
#define  RST_FLAG_WTR       0x02
#define  RST_FLAG_MR        0x03
//#define  RST_FLAG_GPWSM     0x04                      // RO, power on reset flag during sleep/shutdown: 0=no power on reset during sleep/shutdown, 1=power on reset occurred during sleep/shutdown
#define  RST_FLAG_GPWSM     0x05
// RB_RESET_FLAG: recent reset flag
//   000 - SR, software reset, by RB_SOFTWARE_RESET=1 @RB_WDOG_RST_EN=0
//   001 - RPOR, real power on reset
//   010 - WTR, watch-dog timer-out reset
//   011 - MR, external manual reset by RST pin input low
//   101 - GRWSM, global reset by waking under shutdown mode
//   1?? - LRW, power on reset occurred during sleep
#define R8_GLOB_ROM_CFG     R8_RESET_STATUS           // RWA, flash ROM configuration, SAM
#define  RB_ROM_CODE_WE     0xC0                      // RWA, enable flash ROM code area being erase/write: X0=code writing protect, 01=enable 129~240K area program and erase, 11=enable 0~240K area program and erase
#define  RB_ROM_CTRL_EN     0x20                      // RWA, enable flash ROM control interface enable: 0=disable access, 1=enable access control register
#define  RB_ROM_CODE_OFS    0x10                      // RWA, code offset address selection in Flash ROM: 0=start address 0x000000, 1=start address 0x008000
#define R8_GLOB_CFG_INFO    (*((PUINT8V)0x40001045))  // RO, global configuration information and status
#define  RB_BOOT_LOADER     0x20                      // RO, indicate boot loader status: 0=application status (by software reset), 1=boot loader status
#define  RB_CFG_RST_PIN     0x10                      // RO, reset pin sel,1=PA7,0=PA8
#define  RB_CFG_BOOT_EN     0x08                      // RO, boot-loader enable status
#define  RB_CFG_RESET_EN    0x04                      // RO, manual reset input enable status
#define R8_RST_WDOG_CTRL    (*((PUINT8V)0x40001046))  // RWA, reset and watch-dog control, SAM
#define  RB_WDOG_INT_FLAG   0x10                      // RW1, watch-dog timer overflow interrupt flag, cleared by RW1 or reload watch-dog count or __SEV(Send-Event)
#define  RB_WDOG_INT_EN     0x04                      // RWA, watch-dog timer overflow interrupt enable: 0=disable, 1=enable
#define  RB_WDOG_RST_EN     0x02                      // RWA, enable watch-dog reset if watch-dog timer overflow: 0=as timer only, 1=enable reset if timer overflow
#define  RB_BOOT_LOAD_MAN   0x01                      // RO, manual boot loader flag, high action
#define  RB_SOFTWARE_RESET  0x01                      // WA/WZ, global software reset, high action, auto clear
#define R8_GLOB_RESET_KEEP  (*((PUINT8V)0x40001047))  // RW, value keeper during global reset

/* System: clock configuration register */
#define R32_CLOCK_CONFIG    (*((PUINT32V)0x40001008)) // RWA, clock configuration, SAM
#define R8_CLK_SYS_CFG      (*((PUINT8V)0x40001008))  // RWA, system clock configuration, SAM
#define  RB_CLK_SYS_MOD     0xC0                      // RWA, system clock source mode: 00/10=divided from 32MHz, 01=divided from PLL-600MHz,11=directly from LSI
#define  RB_CLK_PLL_DIV     0x1F                      // RWA, output clock divider from PLL or CK32M
#define R8_HFCK_PWR_CTRL    (*((PUINT8V)0x4000100A))  // RWA, power configuration for system high clock, SAM
#define  RB_CLK_PLL_PON     0x10                      // RWA, PLL power control
#define  RB_CLK_XT32M_KEEP  0x08                      // RWA, RWA, disable auto closing when in halt mode
#define  RB_CLK_XT32M_PON   0x04                      // RWA, extern 32MHz HSE power contorl

// FckLSI = RC_LSI
// Fpll = XT_32MHz * 18.75 = 600MHz
// Fsys = RB_CLK_SYS_MOD==3 ? FckLSI : ( ( RB_CLK_SYS_MOD[0] ? Fpll : XT_32MHz ) / RB_CLK_PLL_DIV )
// default: Fsys = XT_32MHz / RB_CLK_PLL_DIV = 32MHz / 5 = 6.4MHz
//   range: 24~42KHz, 1MHz~16MHz, 18.75MHz~100MHz

/* System: sleep control register */
#define R32_SLEEP_CONTROL   (*((PUINT32V)0x4000100C)) // RWA, sleep control, SAM
#define R8_SLP_CLK_OFF0     (*((PUINT8V)0x4000100C))  // RWA, sleep clock off control byte 0, SAM
#define  RB_SLP_KEYSCAN_WAKE  0x80                    // RWA, enable key_scan waking
#define  RB_SLP_CLK_UART    0x10                      // RWA, set 1 close UART clock
#define  RB_SLP_CLK_CMP     0x02                      // RWA, set 1 close CMP clock
#define  RB_SLP_CLK_TMR     0x01                      // RWA, set 1 close TMR clock
#define R8_SLP_CLK_OFF1     (*((PUINT8V)0x4000100D))  // RWA, sleep clock off control byte 1, SAM
#define  RB_SLP_CLK_BLE     0x80                      // RWA, set 1 close BLE clock
#define  RB_SLP_CLK_USB     0x10                      // RWA, set 1 close USB clock
#define  RB_SLP_CLK_I2C     0x08                      // RWA, set 1 close I2C clock
#define  RB_SLP_CLK_PWMX    0x04                      // RWA, set 1 close PWMx clock
#define  RB_CLK_OFF_AESCCM  0x02                      // RWA, set 1 close AES_CCM clock
#define  RB_SLP_CLK_SPI     0x01                      // RWA, set 1 close SPI clock
#define R8_SLP_WAKE_CTRL    (*((PUINT8V)0x4000100E))  // RWA, wake control, SAM
#define  RB_GPIO_WAKE_MODE  0x80                      // RWA, GPIO wakeup mode: RB_SLP_GPIO_EDGE_MODE=1,1=all edge , RB_SLP_GPIO_EDGE_MODE=0,1=rise edge;0=high level
#define  RB_WAKE_EV_MODE    0x40                      // RWA, event wakeup mode: 0=event keep valid for long time, 1=short pulse event
#define  RB_SLP_BAT_WAKE    0x20                      // RWA, enable BAT waking
#define  RB_SLP_GPIO_WAKE   0x10                      // RWA, enable GPIO waking
#define  RB_SLP_RTC_WAKE    0x08                      // RWA, enable RTC waking
#define  RB_SLP_GPIO_EDGE_MODE    0x04                // RWA, GPIO waking edge select:1=all edge,0=pos edge
#define  RB_SLP_ENC_WAKE    0x02                      // RWA, enable encoder waking
#define  RB_SLP_USB_WAKE    0x01                      // RWA, enable USB waking
#define R8_SLP_POWER_CTRL   (*((PUINT8V)0x4000100F))  // RWA, peripherals power down control, SAM
#define  RB_RAM_RET_LV      0xC0                      // RWA, SRAM retention voltage selection: 00=disable, 01=low power mode 1, 10=low power mode 2, 11=low power mode 3,
#define  RB_SLP_CLK_RAMX    0x10                      // RWA, 1=close main SRAM clock, 0=enable main SRAM clock
#define  RB_WAKE_DLY_MOD    0x07                      // RWA, wakeup wait time selection,FCLK + 48cycle == SCLK
// RB_WAKE_DLY_MOD select wakeup delay
//  000: short time, 3584 cycles+TSUHSE
//  001: short time, 512 cycles+TSUHSE
//  010: short time, 64 cycles+TSUHSE
//  011: short time, 1 cycles+TSUHSE
//  100: long time,  8191 cycles+TSUHSE
//  101: long time,  7168 cycles+TSUHSE
//  110: long time,  6144 cycles+TSUHSE
//  111: long time,  4096 cycles+TSUHSE

/* System: I/O pin configuration register */
#define R16_PIN_ALTERNATE   (*((PUINT16V)0x40001018)) // RW, function pin alternate configuration low word
#define  RB_PIN_DEBUG_EN     0x4000                    // RW, debug interface enable  
#define  RB_PIN_USB_EN      0x2000                    // RW, USB analog I/O enable: 0=analog I/O disable, 1=analog I/O enable  
#define  RB_UDP_PU_EN         0x1000                    // RW, USB UDP internal pullup resistance enable: 0=enable/disable by RB_UC_DEV_PU_EN, 1=enable pullup, replace RB_UC_DEV_PU_EN under sleep mode 
#define  RB_PA_DI_DIS         0x0FFF                    // RW, 1=disable PA input , 0=enable PA input
#define R16_PIN_ALTERNATE_H (*((PUINT16V)0x4000101A)) // RW, function pin alternate configuration high word
#define  RB_25M_EN            0x1000                    // RW, CLK25M OUPUT enable
#define  RB_SPI_CLK            0x0800                    // RW, SPI CLK alternate pin enable
#define  RB_I2C_PIN            0x0600                    // RW, I2C alternate pin enable
#define  RB_SPI_CS            0x0100                    // RW, SPI CS alternate pin enable
#define  RB_TMR_PIN         0x00C0                    // RW, TIMER alternate pin enable
#define  RB_UART_TXD        0x0038                    // RW, TXD alternate pin enable
#define  RB_UART_RXD        0x0007                    // RW, RXD alternate pin enable
#define R16_SLP_WAKE_CFG     (*((PUINT16V)0x4000101E)) // RWA, sleep clock aux register, SAM
#define  RB_ACAUTO_ENABLE   0x0100                    // RWA, 1=enable safe register acess auto off,1=disable safe register acess auto off  
#define  RB_PRECLK_CNT_SEL  0x0060                    // RWA, preclk count value sel,11=2048,101024,01= 512,00=256(actual time value = cnt * Fsys)
#define  RB_PRECLK_CNT_EN   0x0010                    // RWA, 1=need to wait until reach to wait time to release flck when wake up   
#define  RB_OSCCLK_RDY_KEEP 0x0001                    // RWA, 1=force OSC READY when halt sleep , 0= OSC not READY when halt sleep   
#define R8_SLP_CLK_OFF2     (*((PUINT8V)0x4000101D))  // RWA, sleep clock off control byte 2, SAM
#define  RB_CLK_OFF_HCLK    0x10                      // RW, 1=XROM hclk off  
#define  RB_CLK_OFF_DEBUG   0x02                      // RW, 1=2wire debug clk off  
#define  RB_CLK_OFF_XROM    0x01                      // RW, 1=XROM 64M or 600M clk off 
#define R8_LONG_RST_CFG     (*((PUINT8V)0x4000101C))  // RWA, long reset config, SAM
#define  RB_LONG_TIM_SEL    0x06                      // RW, long reset time value selecet 11=32.768ms,10=25.000ms,01=20.000msm,00=15.000ms
#define  RB_LONG_RST_EN     0x01                      // RW, long reset enable

/* System: power management register */
#define R32_POWER_MANAG     (*((PUINT32V)0x40001020)) // RWA, power management register, SAM
#define R16_POWER_PLAN      (*((PUINT16V)0x40001020)) // RWA, power plan before sleep instruction, SAM
#define  RB_PWR_PLAN_EN     0x8000                    // RWA/WZ, power plan enable, auto clear after sleep executed
#define  RB_XT_PRE_EN       0x4000                    // RWA, extern 32MHz HSE early wake up enable, must be used with LSI
#define  RB_PWR_MUST_0      0x2000                    // RWA, must write 0
// #define  RB_XT_PRE_CFG      0x1800                    // RWA, extern 32MHz HSE early wake up time configuration
#define  RB_PWR_LDO5V_EN    0x0100                    // RWA, internal LDO5v enable,1=power by V5,0=power by VDD33 
#define  RB_PWR_SYS_EN      0x80                      // RWA, power for system
#define  RB_MAIN_ACT        0x40                      // RWA, main power chose
#define  RB_PWR_EXTEND      0x08                      // RWA, power retention for USB and BLE
#define  RB_PWR_CORE        0x04                      // RWA, power retention for core and base peripherals
#define  RB_PWR_RAM12K      0x02                      // RWA, power for retention 12KB SRAM
#define  RB_PWR_XROM        0x01                      // RWA, power for flash ROM
#define R16_AUX_POWER_ADJ   (*((PUINT16V)0x40001022))  // RWA, aux power adjust control, SAM
#define  RB_CFG_IVREF       0x0F00                    // RWA, I/V reference config data
#define  RB_ULPLDO_ADJ      0x0007                    // RWA, Ultra-Low-Power LDO voltage adjust

/* System: battery detector register */
#define R32_BATTERY_CTRL    (*((PUINT32V)0x40001024)) // RWA, battery voltage detector, SAM
#define R8_BAT_DET_CTRL     (*((PUINT8V)0x40001024))  // RWA, battery voltage detector control, SAM
#define  RB_BAT_LOW_IE      0x08                      // RWA, interrupt enable for battery low voltage
#define  RB_BAT_MON_EN      0x02                      // RWA, battery voltage monitor enable under sleep mode
#define  RB_PWR_LDO_EN      0x01                      // RWA, enable LDO
// request NMI interrupt if both RB_BAT_LOWER_IE and RB_BAT_LOW_IE enabled
#define R8_BAT_DET_CFG      (*((PUINT8V)0x40001025))  // RWA, battery voltage detector configuration, SAM
#define  RB_BAT_LOW_VTH     0x03                      // RWA, select detector/monitor threshold voltage of battery voltage low
#define R8_BAT_STATUS       (*((PUINT8V)0x40001026))  // RO, battery status
#define  RB_BAT_STAT_LOW    0x02                      // RO, battery low voltage status for detector/monitor, high action

/* System: LSI oscillator control register */
#define R32_OSC_LSI_CTRL     (*((PUINT32V)0x4000102C)) // RWA, LSI oscillator control, SAM
#define R16_INT_LSI_TUNE     (*((PUINT16V)0x4000102C)) // RWA, LSI oscillator tune control, SAM
#define  RB_INT_LSI_TUNE     0x1FFF                    // RWA, LSI oscillator frequency tune
#define R8_LSI_CONFIG       (*((PUINT8V)0x4000102F))   // RWA, LSI oscillator configure
#define  RB_LSI_CLK_PIN     0x80                       // RO, LSI oscillator clock pin status
#define  RB_CLK_LSI_PON  0x01                       // RWA, LSI oscillator power on

/* System: real-time clock register */
#define R32_RTC_CTRL        (*((PUINT32V)0x40001030)) // RWA, RTC control, SAM
#define R8_RTC_FLAG_CTRL    (*((PUINT8V)0x40001030))  // RW, RTC flag and clear control
#define  RB_RTC_TRIG_FLAG   0x80                      // RO, RTC trigger action flag
#define  RB_RTC_TMR_FLAG    0x40                      // RO, RTC timer action flag
#define  RB_RTC_TRIG_CLR    0x20                      // RW, set 1 to clear RTC trigger action flag, auto clear
#define  RB_RTC_TMR_CLR     0x10                      // RW, set 1 to clear RTC timer action flag, auto clear
#define R8_RTC_MODE_CTRL    (*((PUINT8V)0x40001031))  // RWA, RTC mode control, SAM
#define  RB_RTC_LOAD_HI     0x80                      // RWA, set 1 to load RTC count high word R32_RTC_CNT_DAY, auto clear after loaded
#define  RB_RTC_LOAD_LO     0x40                      // RWA, set 1 to load RTC count low word R32_RTC_CNT_LSI, auto clear after loaded
#define  RB_RTC_TRIG_EN     0x20                      // RWA, RTC trigger mode enable
#define  RB_RTC_TMR_EN      0x10                      // RWA, RTC timer mode enable
#define  RB_RTC_IGNORE_B0   0x08                      // RWA, force ignore bit0 for trigger mode: 0=compare bit0, 1=ignore bit0
#define  RB_RTC_TMR_MODE    0x07                      // RWA, RTC timer mode(unit=cycle): 000: 1; 001: 4096; 010: 12288; 011: 28672 ; 100:61440 ;101: 122880 ;110: 245760; 111: 491520
#define R32_RTC_TRIG        (*((PUINT32V)0x40001034)) // RWA, RTC trigger value, SAM
#define R32_RTC_CNT_LSI     (*((PUINT32V)0x40001038)) // RO, RTC count based LSI
#define R16_RTC_CNT_LSI     (*((PUINT16V)0x40001038)) // RO, RTC count based LSI
#define R16_RTC_CNT_DIV1    (*((PUINT16V)0x4000103A)) // RO, RTC count based 65536 frequency division period
#define R32_RTC_CNT_DIV2    (*((PUINT32V)0x4000103C)) // RO, RTC count based 2764800000 frequency division period

/*System: Miscellaneous Control register */
#define R32_MISC_CTRL       (*((PUINT32V)0x40001048)) // RWA, miscellaneous control register
#define R8_PLL_CONFIG       (*((PUINT8V)0x4000104B))  // RWA, PLL configuration control, SAM
#define  RB_PLL_CFG_DAT     0x7F                      // RWA, PLL configuration control, SAM

/* System: 32MHz oscillator control register */
#define R8_XT32M_TUNE       (*((PUINT8V)0x4000104E))  // RWA, external 32MHz oscillator tune control, SAM
#define  RB_XT32M_C_LOAD    0x70                      // RWA, external 32MHz oscillator load capacitor tune: Cap = RB_XT32M_C_LOAD * 2 + 10pF
#define  RB_XT32M_I_BIAS    0x03                      // RWA, external 32MHz oscillator bias current tune: 00=75% current, 01=standard current, 10=125% current, 11=150% current

/* System: oscillator frequency calibration register */
#define R32_OSC_CALIB       (*((PUINT32V)0x40001050)) // RWA, oscillator frequency calibration, SAM
#define R16_OSC_CAL_CNT     (*((PUINT16V)0x40001050)) // RO, system clock count value for LSI multi-cycles
#define  RB_OSC_CAL_IF      0x8000                    // RW1, interrupt flag for oscillator capture end, set 1 to clear
#define  RB_OSC_CAL_OV_CLR  0x4000                    // RW1, indicate R8_OSC_CAL_OV_CNT not zero, set 1 to clear R8_OSC_CAL_OV_CNT
#define  RB_OSC_CAL_CNT     0x3FFF                    // RO, system clock count value for LSI multi-cycles
#define R8_OSC_CAL_OV_CNT   (*((PUINT8V)0x40001052))  // RO, oscillator frequency calibration overflow times
#define R8_OSC_CAL_CTRL     (*((PUINT8V)0x40001053))  // RWA, oscillator frequency calibration control, SAM
#define  RB_CNT_CLR         0x80                      // RWA,  reset RB_OSC_CAL_CNT
#define  RB_OSC_CNT_END     0x40                      // RWA, select oscillator capture end mode: 0=normal, 1=append 2 cycles
#define  RB_OSC_CNT_EN      0x20                      // RWA, calibration counter enable
#define  RB_OSC_CAL_IE      0x10                      // RWA, interrupt enable for oscillator capture end
#define  RB_OSC_CNT_HALT    0x08                      // RO, calibration counter halt status: 0=counting, 1=halt for reading count value
#define  RB_OSC_CNT_TOTAL   0x07                      // RWA, total cycles mode for oscillator capture
// RB_OSC_CNT_TOTAL: select total cycles for oscillator capture
//    000: 1
//    001: 2
//    010: 4
//    011: 32
//    100: 64
//    101: 128
//    110: 1024
//    111: 2047

/*system: CMP*/
#define R32_CMP_CTRL       (*((PUINT32V)0x40001054)) // RW, configuration for comparator, 
#define R8_CMP_CTRL_0      (*((PUINT8V)0x40001054))  // RW, configuration for comparator0, 
#define  RB_CMP_NREF_LEVEL 0xF0                      // RW, comparator negative end point Vref sel:1111=800mv,0000=50mv       
#define  RB_CMP_SW         0x0C                      // RW, [0]comparator -channel input sel:1=PA7,0=PA3 ; [1]comparator +channel input sel: 1=COMP_VERF,0=PA2
#define  RB_CMP_CAP        0x02                      // RW, connect COMP_output to be TIM_cap1_input
#define  RB_CMP_EN         0x01                      // RW, enable comparators    
#define R8_CMP_CTRL_1      (*((PUINT8V)0x40001055))  // RW, configuration for comparator1, 
#define  RB_CMP_OUT_SEL    0x0C                      // RW, comparator output sel:11=rise edge,10=fall edge,01=low,00=high 
#define  RB_CMP_IE         0x01                      // RW, comparator interupt enable 
#define R8_CMP_CTRL_2      (*((PUINT8V)0x40001056))  // RW, configuration for comparator2, 
#define  RB_CMP_IF         0x01                      // RW1Z, comparator interupt flag     
#define R8_CMP_CTRL_3      (*((PUINT8V)0x40001057))  // RW, configuration for comparator3,
#define  RB_CMP_REAL_SIG   0x01                      // RO, comparator current real siginal 
#define  RB_APR_OUT_CMP    0x02                      // RO, comparator current output siginal 
#define R32_SAFE_ACCESS_SIG2  (*((PUINT32V)0x40001058)) // RO, safe accessing sign register2,
#define  RB_FUN_MODE        0x07000000                // RO, function enable
#define  RB_FLASH_HALTED    0x00800000                // RO, 2wire flash prohibited operation flag,1=prohibit operation; 0=allow operation
#define  RB_MANU_CFG_LOCK   0x00400000                // RO, vendor configuration word lock flag,1=locked; 0= not locked
#define  RB_RD_PROTECT      0x00200000                // RO, flash read protecet flag,1=enable read project; 0=disable read project
#define  RB_SAFE_AC_DIS     0x00100000                // RO, safe register auto disable flag,1=disable safe access auto off; 0=enable safe access auto off


/* System: Flash ROM control register */
#define R32_FLASH_DATA      (*((PUINT32V)0x40001800)) // RO/WO, flash ROM data
#define R32_FLASH_CONTROL   (*((PUINT32V)0x40001804)) // RWA, flash ROM control,byte1 and byte3 need RWA
#define R8_FLASH_DATA       (*((PUINT8V)0x40001804))  // RO/WO, flash ROM data buffer
#define R8_FLASH_SCK        (*((PUINT8V)0x40001805))  // RW, flash ROM sck time config
#define R8_FLASH_CTRL       (*((PUINT8V)0x40001806))  // RW, flash ROM access control
#define R8_FLASH_CFG        (*((PUINT8V)0x40001807))  // RWA, flash ROM access config, SAM

/* System: GPIO interrupt control register */
#define R32_GPIO_INT_EN     (*((PUINT32V)0x40001090)) // RW, GPIO interrupt enable
#define R16_PA_INT_EN       (*((PUINT16V)0x40001090)) // RW, GPIO PA interrupt enable
#define R32_GPIO_INT_MODE   (*((PUINT32V)0x40001094)) // RW, GPIO interrupt mode: 0=level action, 1=edge action
#define R16_PA_INT_MODE     (*((PUINT16V)0x40001094)) // RW, GPIO PA interrupt mode: 0=level action, 1=edge action
#define R16_PA_INT_EDGE_TYPE (*((PUINT16V)0x40001096)) // RW, GPIO interrupt edge mode: 0=according to por, 1= don't according to por	
#define R32_GPIO_INT_IF     (*((PUINT32V)0x4000109C)) // RW1, GPIO interrupt flag
#define R16_PA_INT_IF       (*((PUINT16V)0x4000109C)) // RW1, GPIO PA interrupt flag

/* GPIO PA register */
#define R32_PA_DIR          (*((PUINT32V)0x400010A0)) // RW, GPIO PA I/O direction: 0=in, 1=out
#define R8_PA_DIR_0         (*((PUINT8V)0x400010A0))  // RW, GPIO PA I/O direction byte 0
#define R8_PA_DIR_1         (*((PUINT8V)0x400010A1))  // RW, GPIO PA I/O direction byte 1
#define R32_PA_PIN          (*((PUINT32V)0x400010A4)) // RO, GPIO PA input
#define R8_PA_PIN_0         (*((PUINT8V)0x400010A4))  // RO, GPIO PA input byte 0
#define R8_PA_PIN_1         (*((PUINT8V)0x400010A5))  // RO, GPIO PA input byte 1
#define R32_PA_OUT          (*((PUINT32V)0x400010A8)) // RW, GPIO PA output
#define R8_PA_OUT_0         (*((PUINT8V)0x400010A8))  // RW, GPIO PA output byte 0
#define R8_PA_OUT_1         (*((PUINT8V)0x400010A9))  // RW, GPIO PA output byte 1
#define R32_PA_CLR          (*((PUINT32V)0x400010AC)) // WZ, GPIO PA clear output: 0=keep, 1=clear
#define R8_PA_CLR_0         (*((PUINT8V)0x400010AC))  // WZ, GPIO PA clear output byte 0
#define R8_PA_CLR_1         (*((PUINT8V)0x400010AD))  // WZ, GPIO PA clear output byte 1
#define R32_PA_PU           (*((PUINT32V)0x400010B0)) // RW, GPIO PA pullup resistance enable
#define R8_PA_PU_0          (*((PUINT8V)0x400010B0))  // RW, GPIO PA pullup resistance enable byte 0
#define R8_PA_PU_1          (*((PUINT8V)0x400010B1))  // RW, GPIO PA pullup resistance enable byte 1
#define R32_PA_PD_DRV       (*((PUINT32V)0x400010B4)) // RW, PA pulldown for input or PA driving capability for output
#define R8_PA_PD_DRV_0      (*((PUINT8V)0x400010B4))  // RW, PA pulldown for input or PA driving capability for output byte 0
#define R8_PA_PD_DRV_1      (*((PUINT8V)0x400010B5))  // RW, PA pulldown for input or PA driving capability for output byte 1
#define R32_PA_SET          (*((PUINT32V)0x400010B8)) // WZ, PA set high for output ,1=set output high,0=IDLE
/* KEYSCAN register */
#define R16_KEY_SCAN_CTRL    (*((PUINT16V)0x40001064))// KEY SCAN control register
#define  RB_CLR_WAKEUP_EN   0x4000                   // RW, claer wake_up siginal after chip wakeing up, 1=enable 0=disable 
#define  RB_SCAN_1END_WAKE_EN    0x2000              // RW, wake up chip after 1 round of key scanning, 1=enable 0=disable 
#define  RB_PIN_SCAN_EN     0x1F00                   // RW, select which pin could be scaned, 1=enable 0=disable
#define  RB_SCAN_CLK_DIV    0x00F0                   // RW, divider value of scanning clock
#define  RB_SCAN_CNT_END    0x000E                   // RW, set the times of the same key_scan value 
#define  RB_SCAN_START_EN   0x0001                   // RW, start key scan enable,1=enable 0=disable
#define R8_KEY_SCAN_INT_EN (*((PUINT8V)0x40001066))  // KEY SCAN interupt enable register
#define  RB_SCAN_1END_IE    0x02                     // RW, key scan 1 round end interupt enable
#define  RB_KEY_PRESSED_IE  0x01                     // RW, detect key pressed interupt enable                  
#define R8_KEY_SCAN_INT_FLAG (*((PUINT8V)0x40001067))// KEY SCAN interupt flag register 
#define  RB_SCAN_1END_IF    0x02                     // RW1, key scan 1 round end flag enable
#define  RB_KEY_PRESSED_IF  0x01                     // RW1, detect key pressed flag enable                   
#define R32_KEY_SCAN_NUMB  (*((PUINT32V)0x40001068))// SCAN_KEY number address now register
#define  RB_KEY_SCAN_CNT    0x700000                 // current SCAN_KEY times
#define  RB_KEY_SCAN_NUMB   0x0FFFFF                 // SCAN_KEY number address now 

/* GPIO register address offset and bit define */
#define BA_PA               ((PUINT8V)0x400010A0)     // point GPIO PA base address
#define GPIO_DIR            0x00
#define GPIO_DIR_0          0x00
#define GPIO_DIR_1          0x01
#define GPIO_PIN            0x04
#define GPIO_PIN_0          0x04
#define GPIO_PIN_1          0x05
#define GPIO_OUT            0x08
#define GPIO_OUT_0          0x08
#define GPIO_OUT_1          0x09
#define GPIO_CLR            0x0C
#define GPIO_CLR_0          0x0C
#define GPIO_CLR_1          0x0D
#define GPIO_PU             0x10
#define GPIO_PU_0           0x10
#define GPIO_PU_1           0x11
#define GPIO_PD_DRV         0x14
#define GPIO_PD_DRV_0       0x14
#define GPIO_PD_DRV_1       0x15

/* GPIO alias name */
#define bTIO            (1<<0)          //PA0
#define bTXD_3          (1<<0)          //PA0
#define bRXD_2          (1<<0)          //PA0
#define bSCL_1          (1<<0)          //PA0
#define bUDM            (1<<0)          //PA0

#define bTCK            (1<<1)          //PA1
#define bTXD_2          (1<<1)          //PA1
#define bRXD_3          (1<<1)          //PA1
#define bSDA_1          (1<<1)          //PA1
#define bUDP            (1<<1)          //PA1

#define bTXD_1          (1<<2)          //PA2
#define bRXD_0          (1<<2)          //PA2
#define bSDA_2          (1<<2)          //PA2
#define bPWM2           (1<<2)          //PA2    
#define bPWM0_1         (1<<2)          //PA2    
#define bCAP1_1         (1<<2)          //PA2
#define bCAP2_0         (1<<2)          //PA2
#define bSCS_           (1<<2)          //PA2
#define bKEY0           (1<<2)          //PA2
#define bCMP0           (1<<2)          //PA2

#define bTXD_0          (1<<3)          //PA3
#define bRXD_1          (1<<3)          //PA3
#define bSCL_2          (1<<3)          //PA3
#define bPWM3           (1<<3)          //PA3    
#define bSCK_           (1<<3)          //PA3
#define bKEY1           (1<<3)          //PA3
#define bCMP1           (1<<3)          //PA3

#define bPWM4           (1<<4)          //PA4    
#define bPWM0_2         (1<<4)          //PA4    
#define bSCS            (1<<4)          //PA4
#define bCAP1_2         (1<<4)          //PA4
#define bCAP2_3         (1<<4)          //PA4
#define bXM25MO         (1<<4)          //PA4

#define bSCL_3          (1<<5)          //PA5
#define bSCK            (1<<5)          //PA5

#define bSDA_6          (1<<6)          //PA6
#define bMISO           (1<<6)          //PA6
#define bRXD_4          (1<<6)          //PA6

#define bTXD_4          (1<<7)          //PA7
#define bMOSI           (1<<7)          //PA7
#define bPWM1           (1<<7)          //PA7    
#define bPWM0_0         (1<<7)          //PA7    
#define bRST_           (1<<7)          //PA7
#define bCAP1_0         (1<<7)          //PA7
#define bCAP2_1         (1<<7)          //PA7
#define bCMP2           (1<<7)          //PA7

#define bRST            (1<<8)          //PA8
#define bTXD_5          (1<<8)          //PA8
#define bSCL_0          (1<<8)          //PA8
#define bPWM5           (1<<8)          //PA8    
#define bKEY2           (1<<8)          //PA8

#define bSDA_0          (1<<9)          //PA9
#define bPWM0_3         (1<<9)          //PA9    
#define bRXD_5          (1<<9)          //PA9
#define bCAP1_3         (1<<9)          //PA9
#define bCAP2_2         (1<<9)          //PA9

#define bTXD_7          (1<<10)         //PA10
#define bRXD_6          (1<<10)         //PA10
#define bKEY3           (1<<10)         //PA10

#define bTXD_6          (1<<11)         //PA11
#define bRXD_7          (1<<11)         //PA11
#define bKEY4           (1<<11)         //PA11

/* Timer1 register */
#define R32_TMR_CONTROL    (*((PUINT32V)0x40002400)) // RW, TMR control
#define R8_TMR_CTRL_MOD    (*((PUINT8V)0x40002400))  // RW, TMR mode control
#define R8_TMR_CTRL_DMA    (*((PUINT8V)0x40002401))  // RW, TMR DMA control
#define R8_TMR_INTER_EN    (*((PUINT8V)0x40002402))  // RW, TMR interrupt enable
// #define R32_TMR_STATUS     (*((PUINT32V)0x40002404)) // RW, TMR status
#define R8_TMR_INT_FLAG    (*((PUINT8V)0x40002406))  // RW1, TMR interrupt flag
#define R8_TMR_FIFO_COUNT  (*((PUINT8V)0x40002407))  // RO, TMR FIFO count status
#define R32_TMR_COUNT      (*((PUINT32V)0x40002408)) // RO, TMR current count
#define R16_TMR_COUNT      (*((PUINT16V)0x40002408)) // RO, TMR current count
#define R8_TMR_COUNT       (*((PUINT8V)0x40002408))  // RO, TMR current count
#define R32_TMR_CNT_END    (*((PUINT32V)0x4000240C)) // RW, TMR end count value, only low 26 bit
#define R32_TMR_FIFO       (*((PUINT32V)0x40002410)) // RO/WO, TMR FIFO register, only low 26 bit
#define R16_TMR_FIFO       (*((PUINT16V)0x40002410)) // RO/WO, TMR FIFO register
#define R8_TMR_FIFO        (*((PUINT8V)0x40002410))  // RO/WO, TMR FIFO register
#define R32_TMR_DMA_NOW    (*((PUINT32V)0x40002414)) // RW, TMR DMA current address
#define R16_TMR_DMA_NOW    (*((PUINT16V)0x40002414)) // RW, TMR DMA current address
#define R32_TMR_DMA_BEG    (*((PUINT32V)0x40002418)) // RW, TMR DMA begin address
#define R16_TMR_DMA_BEG    (*((PUINT16V)0x40002418)) // RW, TMR DMA begin address
#define R32_TMR_DMA_END    (*((PUINT32V)0x4000241C)) // RW, TMR DMA end address
#define R16_TMR_DMA_END    (*((PUINT16V)0x4000241C)) // RW, TMR DMA end address
/* ENCODER register */
#define R32_ENC_REG_CTRL    (*((PUINT32V)0x40002420))
#define R8_ENC_REG_CTRL        (*((PUINT8V)0x40002420))  // RW, ENCODER control register
// #define  RB_WAKEUP_CLR_EN   0x10                      // RW, clear wake_up siginal after chip wake up enable   
#define  RB_ENC_DIR         0x20                      // RO, encoder director,0=forward,1=backward  
#define  RB_RD_CLR_EN       0x08                      // RW, clear encoder count value after R32_ENC_REG_CCNT be read
#define  RB_SMS_MODE        0x06                      // RW, SMS mode value,10=T1EDGE,01=T2EDGE,11=T12EDGE
#define  RB_START_ENC_EN    0x01                      // RW, start encode enable   
#define R8_ENC_INTER_EN        (*((PUINT8V)0x40002421))  // RW, ENCODER interupt enable register
#define  RB_IE_DIR_DEC      0x02                      // RW, encode decrease interupt enable
#define  RB_IE_DIR_INC      0x01                      // RW, encode increase interupt enable 
#define R8_ENC_INT_FLAG        (*((PUINT8V)0x40002422))  // RW, ENCODER interupt flag register
#define  RB_IF_DIR_DEC      0x02                      // RWA, encode decrease interupt flag
#define  RB_IF_DIR_INC      0x01                      // RWA, encode increase interupt flag 
#define R32_ENC_REG_CEND    (*((PUINT32V)0x40002424))
#define R32_ENC_REG_CCNT    (*((PUINT32V)0x40002428))

/* Timer register address offset and bit define */
#define TMR_FIFO_SIZE       8                         // timer FIFO size (depth)
#define BA_TMR             ((PUINT8V)0x40002400)     // point TMR base address
#define TMR_CTRL_MOD        0
#define  RB_TMR_PWM_REPEAT  0xC0                      // RW, timer PWM repeat mode: 00=1, 01=4, 10=8, 11-16
#define  RB_TMR_CAP_EDGE    0xC0                      // RW, timer capture edge mode: 00=disable, 01=edge change, 10=fall to fall, 11-rise to rise
#define  RB_TMR_OUT_POLAR   0x10                      // RW, timer PWM output polarity: 0=default low and high action, 1=default high and low action
#define  RB_TMR_CAP_COUNT   0x10                      // RW, count sub-mode if RB_TMR_MODE_IN=1: 0=capture, 1=count
#define  RB_TMR_OUT_EN      0x08                      // RW, timer output enable
#define  RB_TMR_COUNT_EN    0x04                      // RW, timer count enable
#define  RB_TMR_ALL_CLEAR   0x02                      // RW, force clear timer FIFO and count
#define  RB_TMR_MODE_IN     0x01                      // RW, timer in mode: 0=timer/PWM, 1=capture/count
#define TMR_CTRL_DMA        1
#define  RB_TMR_DMA_LOOP    0x04                      // RW, timer1/2 DMA address loop enable
#define  RB_TMR_DMA_ENABLE  0x01                      // RW, timer1/2 DMA enable
#define TMR_INTER_EN        2
#define  RB_TMR_IE_FIFO_OV  0x10                      // RW, enable interrupt for timer FIFO overflow
#define  RB_TMR_IE_DMA_END  0x08                      // RW, enable interrupt for timer1/2 DMA completion
#define  RB_TMR_IE_FIFO_HF  0x04                      // RW, enable interrupt for timer FIFO half (capture fifo >=4 or PWM fifo <=3)
#define  RB_TMR_IE_DATA_ACT 0x02                      // RW, enable interrupt for timer capture input action or PWM trigger
#define  RB_TMR_IE_CYC_END  0x01                      // RW, enable interrupt for timer capture count timeout or PWM cycle end
#define TMR_INT_FLAG        6
#define  RB_TMR_IF_FIFO_OV  0x10                      // RW1, interrupt flag for timer FIFO overflow
#define  RB_TMR_IF_DMA_END  0x08                      // RW1, interrupt flag for timer1/2 DMA completion
#define  RB_TMR_IF_FIFO_HF  0x04                      // RW1, interrupt flag for timer FIFO half (capture fifo >=4 or PWM fifo <=3)
#define  RB_TMR_IF_DATA_ACT 0x02                      // RW1, interrupt flag for timer capture input action or PWM trigger
#define  RB_TMR_IF_CYC_END  0x01                      // RW1, interrupt flag for timer capture count timeout or PWM cycle end
#define TMR_FIFO_COUNT      7
#define TMR_DMA_END         0x1C
#define TMR_DMA_BEG         0x18
#define TMR_DMA_NOW         0x14
#define TMR_FIFO            0x10
#define TMR_CNT_END         0x0C
#define TMR_COUNT           0x08

/* UART register */
#define R32_UART_CTRL      (*((PUINT32V)0x40003400)) // RW, UART control
#define R8_UART_MCR        (*((PUINT8V)0x40003400))  // RW, UART modem control
#define R8_UART_IER        (*((PUINT8V)0x40003401))  // RW, UART interrupt enable
#define R8_UART_FCR        (*((PUINT8V)0x40003402))  // RW, UART FIFO control
#define R8_UART_LCR        (*((PUINT8V)0x40003403))  // RW, UART line control
#define R8_UART_IIR        (*((PUINT8V)0x40003404))  // RO, UART interrupt identification
#define R8_UART_LSR        (*((PUINT8V)0x40003405))  // RO, UART line status
#define R8_UART_RBR        (*((PUINT8V)0x40003408))  // RO, UART receiver buffer, receiving byte
#define R8_UART_THR        (*((PUINT8V)0x40003408))  // WO, UART transmitter holding, transmittal byte
#define R8_UART_RFC        (*((PUINT8V)0x4000340A))  // RO, UART receiver FIFO count
#define R8_UART_TFC        (*((PUINT8V)0x4000340B))  // RO, UART transmitter FIFO count
#define R16_UART_DL        (*((PUINT16V)0x4000340C)) // RW, UART divisor latch
#define R8_UART_DLL        (*((PUINT8V)0x4000340C))  // RW, UART divisor latch LSB byte
#define R8_UART_DLM        (*((PUINT8V)0x4000340D))  // RW, UART divisor latch MSB byte
#define R8_UART_DIV        (*((PUINT8V)0x4000340E))  // RW, UART pre-divisor latch byte, only low 7 bit, from 1 to 0/128

/* UART register address offset and bit define */
#define UART_FIFO_SIZE      8                         // UART FIFO size (depth)
#define UART_RECV_RDY_SZ    7                         // the max FIFO trigger level for UART receiver data available
#define BA_UART            ((PUINT8V)0x40003400)     // point UART base address
#define UART_MCR            0
#define  RB_MCR_OUT2        0x08                      // RW, UART control OUT2
#define  RB_MCR_INT_OE      0x08                      // RW, UART interrupt output enable
#define UART_IER            1
#define  RB_IER_TXD_EN      0x40                      // RW, UART TXD pin enable
#define  RB_IER_LINE_STAT   0x04                      // RW, UART interrupt enable for receiver line status
#define  RB_IER_THR_EMPTY   0x02                      // RW, UART interrupt enable for THR empty
#define  RB_IER_RECV_RDY    0x01                      // RW, UART interrupt enable for receiver data ready
#define UART_FCR            2
#define  RB_FCR_FIFO_TRIG   0xC0                      // RW, UART receiver FIFO trigger level: 00-1byte, 01-2bytes, 10-4bytes, 11-7bytes
#define  RB_FCR_FIFO_EN     0x01                      // RW, UART FIFO enable
#define UART_LCR            3
#define  RB_LCR_DLAB        0x80                      // RW, UART reserved bit
#define  RB_LCR_GP_BIT      0x80                      // RW, UART general purpose bit
#define  RB_LCR_BREAK_EN    0x40                      // RW, UART break control enable
#define  RB_LCR_PAR_MOD     0x30                      // RW, UART parity mode: 00-odd, 01-even, 10-mark, 11-space
#define  RB_LCR_PAR_EN      0x08                      // RW, UART parity enable
#define  RB_LCR_STOP_BIT    0x04                      // RW, UART stop bit length: 0-1bit, 1-2bit
#define  RB_LCR_WORD_SZ     0x03                      // RW, UART word bit length: 00-5bit, 01-6bit, 10-7bit, 11-8bit
#define UART_IIR            4
#define  RB_IIR_FIFO_ID     0xC0                      // RO, UART FIFO enabled flag
#define  RB_IIR_INT_MASK    0x0F                      // RO, UART interrupt flag bit mask
#define  RB_IIR_NO_INT      0x01                      // RO, UART no interrupt flag: 0=interrupt action, 1=no interrupt
#define UART_LSR            5
#define  RB_LSR_ERR_RX_FIFO 0x80                      // RO, indicate error in UART receiver fifo
#define  RB_LSR_TX_ALL_EMP  0x40                      // RO, UART transmitter all empty status
#define  RB_LSR_TX_FIFO_EMP 0x20                      // RO, UART transmitter fifo empty status
#define  RB_LSR_BREAK_ERR   0x10                      // RZ, UART receiver break error
#define  RB_LSR_FRAME_ERR   0x08                      // RZ, UART receiver frame error
#define  RB_LSR_PAR_ERR     0x04                      // RZ, UART receiver parity error
#define  RB_LSR_OVER_ERR    0x02                      // RZ, UART receiver overrun error
#define  RB_LSR_DATA_RDY    0x01                      // RO, UART receiver fifo data ready status
#define UART_MSR            6
#define UART_RBR            8
#define UART_THR            8
#define UART_RFC            0x0A
#define UART_TFC            0x0B
#define UART_DLL            0x0C
// #define UART_DLM            0x0D
#define UART_DIV            0x0E
#define UART_ADR            0x0F

/* UART interrupt identification values for IIR bits 3:0 */
#define UART_II_RECV_TOUT   0x0C                      // RO, UART interrupt by receiver fifo timeout
#define UART_II_LINE_STAT   0x06                      // RO, UART interrupt by receiver line status
#define UART_II_RECV_RDY    0x04                      // RO, UART interrupt by receiver data available
#define UART_II_THR_EMPTY   0x02                      // RO, UART interrupt by THR empty
#define UART_II_NO_INTER    0x01                      // RO, no UART interrupt is pending

/* SPI register */
#define R32_SPI_CONTROL    (*((PUINT32V)0x40004000)) // RW, SPI control
#define R8_SPI_CTRL_MOD    (*((PUINT8V)0x40004000))  // RW, SPI mode control
#define R8_SPI_CTRL_CFG    (*((PUINT8V)0x40004001))  // RW, SPI configuration control
#define R8_SPI_INTER_EN    (*((PUINT8V)0x40004002))  // RW, SPI interrupt enable
#define R8_SPI_CLOCK_DIV   (*((PUINT8V)0x40004003))  // RW, SPI master clock divisor
#define R8_SPI_SLAVE_PRE   (*((PUINT8V)0x40004003))  // RW, SPI slave preset value
#define R32_SPI_STATUS     (*((PUINT32V)0x40004004)) // RW, SPI status
#define R8_SPI_BUFFER      (*((PUINT8V)0x40004004))  // RO, SPI data buffer
#define R8_SPI_RUN_FLAG    (*((PUINT8V)0x40004005))  // RO, SPI work flag
#define R8_SPI_INT_FLAG    (*((PUINT8V)0x40004006))  // RW1, SPI interrupt flag
#define R8_SPI_FIFO_COUNT  (*((PUINT8V)0x40004007))  // RO, SPI FIFO count status
#define R32_SPI_INTER_CFG1 (*((PUINT32V)0x40004008))  // RO, SPI interrupt configuration1
#define R16_SPI_TOTAL_CNT  (*((PUINT16V)0x4000400C)) // RW, SPI total byte count, only low 12 bit
#define R32_SPI_FIFO       (*((PUINT32V)0x40004010)) // RW, SPI FIFO register
#define R8_SPI_FIFO        (*((PUINT8V)0x40004010))  // RO/WO, SPI FIFO register
#define R8_SPI_FIFO_COUNT1 (*((PUINT8V)0x40004013))  // RO, SPI FIFO count status
#define R16_SPI_DMA_NOW    (*((PUINT16V)0x40004014)) // RW, SPI DMA current address
#define R16_SPI_DMA_BEG    (*((PUINT16V)0x40004018)) // RW, SPI DMA begin address
#define R16_SPI_DMA_END    (*((PUINT16V)0x4000401C)) // RW, SPI DMA end address

/* SPI register address offset and bit define */
#define SPI_FIFO_SIZE       8                         // SPI FIFO size (depth)
#define BA_SPI             ((PUINT8V)0x40004000)     // point SPI base address
#define SPI_CTRL_MOD        0
#define  RB_SPI_MISO_OE     0x80                      // RW, SPI MISO output enable
#define  RB_SPI_MOSI_OE     0x40                      // RW, SPI MOSI output enable
#define  RB_SPI_SCK_OE      0x20                      // RW, SPI SCK output enable
#define  RB_SPI_FIFO_DIR    0x10                      // RW, SPI FIFO direction: 0=out(write @master mode), 1=in(read @master mode)
#define  RB_SPI_SLV_CMD_MOD 0x08                      // RW, SPI slave command mode: 0=byte stream, 1=first byte command
#define  RB_SPI_MST_SCK_MOD 0x08                      // RW, SPI master clock mode: 0=mode 0, 1=mode 3
#define  RB_SPI_2WIRE_MOD   0x04                      // RW, SPI enable 2 wire mode for slave: 0=3wire(SCK0,MOSI,MISO), 1=2wire(SCK0,MISO=MXSX)
#define  RB_SPI_ALL_CLEAR   0x02                      // RW, force clear SPI FIFO and count
#define  RB_SPI_MODE_SLAVE  0x01                      // RW, SPI slave mode: 0=master/host, 1=slave/device
#define SPI_CTRL_CFG        1
#define  RB_SPI_MST_DLY_EN  0x40                      // RW, SPI master input delay enable
#define  RB_SPI_BIT_ORDER   0x20                      // RW, SPI bit data order: 0=MSB first, 1=LSB first
#define  RB_SPI_AUTO_IF     0x10                      // RW, enable buffer/FIFO accessing to auto clear RB_SPI_IF_BYTE_END interrupt flag
#define  RB_SPI_DMA_LOOP    0x04                      // RW, SPI DMA address loop enable
#define  RB_MST_CLK_SEL     0x02                      // RW, hclk polarity reversal,1= polarity reversal,0=IDLE 
#define  RB_SPI_DMA_ENABLE  0x01                      // RW, SPI DMA enable
#define SPI_INTER_EN        2
#define  RB_SPI_IE_FST_BYTE 0x80                      // RW, enable interrupt for SPI slave mode first byte received
#define  RB_SPI_IE_FIFO_OV  0x10                      // RW, enable interrupt for SPI FIFO overflow
#define  RB_SPI_IE_DMA_END  0x08                      // RW, enable interrupt for SPI DMA completion
#define  RB_SPI_IE_FIFO_HF  0x04                      // RW, enable interrupt for SPI FIFO half
#define  RB_SPI_IE_BYTE_END 0x02                      // RW, enable interrupt for SPI byte exchanged
#define  RB_SPI_IE_CNT_END  0x01                      // RW, enable interrupt for SPI total byte count end
#define SPI_CLOCK_DIV       3
#define SPI_SLAVE_PRESET    3
#define SPI_BUFFER          4
#define SPI_RUN_FLAG        5
#define  RB_SPI_SLV_SELECT  0x80                      // RO, SPI slave selection status
#define  RB_SPI_SLV_CS_LOAD 0x40                      // RO, SPI slave chip-select loading status
#define  RB_SPI_FIFO_READY  0x20                      // RO, SPI FIFO ready status
#define  RB_SPI_SLV_CMD_ACT 0x10                      // RO, SPI slave first byte / command flag
#define SPI_INT_FLAG        6
#define  RB_SPI_IF_FST_BYTE 0x80                      // RW1, interrupt flag for SPI slave mode first byte received
#define  RB_SPI_FREE        0x40                      // RO, current SPI free status
#define  RB_SPI_IF_FIFO_OV  0x10                      // RW1, interrupt flag for SPI FIFO overflow
#define  RB_SPI_IF_DMA_END  0x08                      // RW1, interrupt flag for SPI DMA completion
#define  RB_SPI_IF_FIFO_HF  0x04                      // RW1, interrupt flag for SPI FIFO half (RB_SPI_FIFO_DIR ? >=4bytes : <4bytes)
#define  RB_SPI_IF_BYTE_END 0x02                      // RW1, interrupt flag for SPI byte exchanged
#define  RB_SPI_IF_CNT_END  0x01                      // RW1, interrupt flag for SPI total byte count end
#define SPI_FIFO_COUNT      7
#define SPI_INTER_CFG1      8
#define  RB_SPI_IF_FIFO_FULL   0x020000               // RW, fifo full interupt flag    
#define  RB_SPI_IF_FIFO_EMPTY  0x010000               // RW, fifo empty interupt flag   
#define  RB_SPI_IE_FIFO_FULL   0x0200                 // RW, fifo full interupt enable    
#define  RB_SPI_IE_FIFO_EMPTY  0x0100                 // RW, fifo empty interupt enable  
#define  RB_SPI_INT_TYPE       0x001F                 // RW, interupt trig mode select,1=edge,0=level  
#define SPI_TOTAL_CNT       0x0C
#define SPI_FIFO            0x10
#define SPI_DMA_NOW         0x14
#define SPI_DMA_BEG         0x18

/* I2C register */
#define R16_I2C_CTRL1       (*((PUINT16V)0x40004800)) // RW, I2C control 1
#define R16_I2C_CTRL2       (*((PUINT16V)0x40004804)) // RW, I2C control 2
#define R16_I2C_OADDR1      (*((PUINT16V)0x40004808)) // RW, I2C own address register 1
#define R16_I2C_OADDR2      (*((PUINT16V)0x4000480C)) // RW, I2C own address register 2
#define R16_I2C_DATAR       (*((PUINT16V)0x40004810)) // RW, I2C data register
#define R16_I2C_STAR1       (*((PUINT16V)0x40004814)) // R0, I2C stauts register 1
#define R16_I2C_STAR2       (*((PUINT16V)0x40004818)) // R0, I2C status register 2
// #define R8_I2C_PEC          (*((PUINT8V) 0x40004819)) // R0, I2C Packet error checking register 
#define R16_I2C_CKCFGR      (*((PUINT16V)0x4000481C)) // RW, I2C clock control register
#define R16_I2C_RTR         (*((PUINT16V)0x40004820)) // RW, I2C trise register

/* I2C register address offset and bit define */
#define BA_I2C              ((PUINT8V)0x40004800)     // point I2C base address
#define I2C_CTRL1           0
#define  RB_I2C_SWRST       0x8000                    // RW, Software reset
#define  RB_I2C_ALERT       0x2000                    // RW, SMBus alert: 0=Releases SMBA pin high, 1=Drives SMBA pin low.
#define  RB_I2C_PEC         0x1000                    // RW, Packet error checking: 0=No PEC transfer, 1=PEC transfer (in Tx or Rx mode)
#define  RB_I2C_POS         0x0800                    // RW, Acknowledge/PEC Position (for data reception)
#define  RB_I2C_ACK         0x0400                    // RW, Acknowledge enable
#define  RB_I2C_STOP        0x0200                    // RW, Stop generation: master mode: 0=no stop, 1=stop after the current byte transfer or after the current Start condition is sent; slave mode: 0=no stop, 1=Release the SCL and SDA lines after the current byte transfer
#define  RB_I2C_START       0x0100                    // RW, Start generation: master mode: 0=no start, 1=repeated start; slave mode: 0=no start, 1=start at bus free
#define  RB_I2C_NOSTRETCH   0x0080                    // RW, Clock stretching disable (Slave mode)
#define  RB_I2C_ENGC        0x0040                    // RW, General call enable
#define  RB_I2C_ENPEC       0x0020                    // RW, PEC ebable
#define  RB_I2C_EBARP       0x0010                    // RW, ARP enable
#define  RB_I2C_SMBTYPE     0x0008                    // RW, SMBus type: 0=Device, 1=Host
#define  RB_I2C_SMBUS       0x0002                    // RW, SMBUS mode: 0=I2C mode, 1=SMBUS mode
#define  RB_I2C_PE          0x0001                    // RW, Peripheral enable
#define I2C_CTRL2           4
#define  RB_I2C_ITBUFEN     0x0400                    // RW, Buffer interrupt enable
#define  RB_I2C_ITEVTEN     0x0200                    // RW, Event interrupt enable
#define  RB_I2C_ITERREN     0x0100                    // RW, Error interrupt enable
#define  RB_I2C_FREQ        0x003F                    // RW, Peripheral clock frequency, The minimum allowed frequency is 2 MHz,the maximum frequency is 36 MHz
#define I2C_OADDR1          8
#define  RB_I2C_ADDMODE     0x8000                    // RW, Addressing mode (slave mode): 0=7-bit slave address, 1=10-bit slave address
#define  RB_I2C_ADD9_8      0x0300                    // RW, bit[9:8] of address in 10-bit addressing mode
#define  RB_I2C_ADD7_1      0x00FE                    // RW, bit[7:1] of address
#define  RB_I2C_ADD0        0x0001                    // RW, bit0 of address in 10-bit addressing mode
#define I2C_OADDR2          12
#define  RB_I2C_ADD2        0x00FE                    // RW, bit[7:1] of address2
#define  RB_I2C_ENDUAL      0x0001                    // RW, Dual addressing mode enable
#define I2C_DATAR           16              
#define I2C_STAR1           20
#define  RB_I2C_SMBALERT    0x8000                    // RW0, SMBus alert flag
#define  RB_I2C_TIMEOUT     0x4000                    // RW0, Timeout or Tlow error flag
#define  RB_I2C_PECERR      0x1000                    // RW0, PEC Error flag in reception
#define  RB_I2C_OVR         0x0800                    // RW0, Overrun/Underrun flag
#define  RB_I2C_AF          0x0400                    // RW0, Acknowledge failure flag
#define  RB_I2C_ARLO        0x0200                    // RW0, Arbitration lost flag (master mode)
#define  RB_I2C_BERR        0x0100                    // RW0, Bus error flag
#define  RB_I2C_TxE         0x0080                    // RO, Data register empty flag (transmitters)
#define  RB_I2C_RxNE        0x0040                    // RO, Data register not empty flag (receivers)
#define  RB_I2C_STOPF       0x0010                    // RO, Stop detection flag (slave mode)
#define  RB_I2C_ADD10       0x0008                    // RO, 10-bit header sent flag (Master mode)
#define  RB_I2C_BTF         0x0004                    // RO, Byte transfer finished flag
#define  RB_I2C_ADDR        0x0002                    // RW0, Address sent (master mode)/matched (slave mode) flag
#define  RB_I2C_SB          0x0001                    // RW0, Start bit flag (Master mode)
#define I2C_STAR2           24
#define  RB_I2C_PECX        0xFF00                    // RO, Packet error checking register
#define  RB_I2C_DUALF       0x0080                    // RO, Dual flag (Slave mode): 0=Received address matched with OAR1, 1=Received address matched with OAR2
#define  RB_I2C_SMBHOST     0x0040                    // RO, SMBus host header (Slave mode) received flag
#define  RB_I2C_SMBDEFAULT  0x0020                    // RO, SMBus device default address (Slave mode) received flag
#define  RB_I2C_GENCALL     0x0010                    // RO, General call address (Slave mode) received flag
#define  RB_I2C_TRA         0x0004                    // RO, Trans flag: 0=data bytes received, 1=data bytes transmitted
#define  RB_I2C_BUSY        0x0002                    // RO, Bus busy flag
#define  RB_I2C_MSL         0x0001                    // RO, Mode statu: 0=Slave mode, 1=Master mode
#define I2C_CKCFGR          28
#define  RB_I2C_F_S         0x8000                    // RW, I2C master mode selection: 0=standard mode, 1=fast mode
#define  RB_I2C_DUTY        0x4000                    // RW, Fm mode duty cycle: 0=L/H=2, 1=L/H=16/9
#define  RB_I2C_CCR         0x0FFF                    // RW, Controls the SCL clock in Fm/Sm mode (Master mode)
#define I2C_RTR             32
#define  RB_I2C_TRISE       0x003F                    // RW, Maximum rise time in Fm/Sm mode (Master mode)

/* PWM1/2/3/4/5/register */
#define R32_PWM_CONTROL     (*((PUINT32V)0x40005000)) // RW, PWM control
#define R8_PWM_OUT_EN       (*((PUINT8V)0x40005000))  // RW, PWM output enable control
#define R8_PWM_POLAR        (*((PUINT8V)0x40005001))  // RW, PWM output polarity control
#define R8_PWM_CONFIG       (*((PUINT8V)0x40005002))  // RW, PWM configuration
#define R8_PWM_DMA_CTRL    (*((PUINT8V)0x40005003))   // RW, PWM DMA control
#define R32_PWM1_3_DATA     (*((PUINT32V)0x40005004)) // RW, PWM1-3 data holding
#define R16_PWM1_DATA       (*((PUINT16V)0x40005004)) // RW, PWM1 data (16 bit) holding
#define R16_PWM2_DATA       (*((PUINT16V)0x40005006)) // RW, PWM2 data (16 bit) holding
#define R8_PWM1_DATA        (*((PUINT8V)0x40005004))  // RW, PWM1 data (8 bit) holding
#define R8_PWM2_DATA        (*((PUINT8V)0x40005005))  // RW, PWM2 data (8 bit) holding
#define R8_PWM3_DATA        (*((PUINT8V)0x40005006))  // RW, PWM3 data (8 bit) holding
#define R16_PWM3_DATA       (*((PUINT16V)0x40005008)) // RW, PWM3 data (16 bit) holding
#define R32_PWM4_5_DATA     (*((PUINT32V)0x40005010)) // RW, PWM4-5 data register
#define R16_PWM4_DATA       (*((PUINT16V)0x40005010))// RW, PWM4 data (16 bit) holding
#define R16_PWM5_DATA       (*((PUINT16V)0x40005012))// RW, PWM5 data (16 bit) holding
#define R8_PWM4_DATA        (*((PUINT8V)0x40005010)) // RW, PWM4 data (8 bit) holding
#define R8_PWM5_DATA        (*((PUINT8V)0x40005011)) // RW, PWM5 data (8 bit) holding
#define R8_PWM_INT_EN       (*((PUINT8V)0x4000500C))  // RW, PWM interrupt enable
#define  RB_PWM_IE_OVER     0x40                     // RW, enable interrupt for fifo overflow  
#define  RB_PWM_IE_DMA      0x20                     // RW, enable interrupt for DMA transmision end
#define  RB_PWM_IE_FIFO     0x10                     // RW, enable interrupt for fifo count < 4  
#define  RB_PWM1_IE_CYC     0x04                     // RW, enable interrupt for PWM4\5 cycle end
#define  RB_PWM_CYC_PRE     0x02                     // RW, select PWM cycle interrupt point: 0=after count 0xFE (0x7E for 7 bits mode...), 1=after count 0xF0 (0x70 for 7 bits mode...) 
#define  RB_PWM_IE_CYC      0x01                     // RW, enable interrupt for PWM1\2\3 cycle end
#define R8_PWM_INT_FLAG     (*((PUINT8V)0x4000500D))  // RW1, PWM interrupt flag
#define  RB_PWM_IF_OVER     0x10                     // RW1, interrupt flag for fifo overflow 
#define  RB_PWM_IF_DMA      0x08                     // RW1, interrupt flag for DMA transmision end
#define  RB_PWM_IF_FIFO     0x04                     // RW1, interrupt flag for fifo count < 4 
#define  RB_PWM1_IF_CYC     0x02                     // RW1, interrupt flag for PWM4\5 cycle end 
#define  RB_PWM_IF_CYC      0x01                     // RW1, interrupt flag for PWM1\2\3 cycle end
#define R16_PWM_CYC_VALUE  (*((PUINT16V)0x40005014)) // RW, PWM1\2\3 cycle value for 16bit
#define R16_PWM_CYC1_VALUE   (*((PUINT16V)0x40005016)) // RW, PWM4\5 cycle value for 16bit
#define R16_PWM_CLOCK_DIV   (*((PUINT16V)0x40005018)) // RW, PWM clock division
#define R32_PWM_DMA_NOW     (*((PUINT32V)0x4000501C)) // RW, PWM DMA addr for now
#define R32_PWM_DMA_BEG     (*((PUINT32V)0x40005020)) // RW, PWM DMA addr of begining
#define R32_PWM_DMA_END     (*((PUINT32V)0x40005024)) // RW, PWM DMA addr of end

/* PWM1/2/3/4/5 register address offset and bit define */
#define BA_PWMX             ((PUINT8V)0x40005000)     // point PWM1/2/3/4/5 base address
#define PWM_OUT_EN          0
#define  RB_PWM5_OUT_EN     0x10                      // RW, PWM5 output enable
#define  RB_PWM4_OUT_EN     0x08                      // RW, PWM4 output enable
#define  RB_PWM3_OUT_EN     0x04                      // RW, PWM3 output enable
#define  RB_PWM2_OUT_EN     0x02                      // RW, PWM2 output enable
#define  RB_PWM1_OUT_EN     0x01                      // RW, PWM1 output enable
#define PWM_POLAR           1
#define  RB_PWM5_POLAR      0x10                      // RW, PWM5 output polarity: 0=default low and high action, 1=default high and low action
#define  RB_PWM4_POLAR      0x08                      // RW, PWM4 output polarity: 0=default low and high action, 1=default high and low action
#define  RB_PWM3_POLAR      0x04                      // RW, PWM3 output polarity: 0=default low and high action, 1=default high and low action
#define  RB_PWM2_POLAR      0x02                      // RW, PWM2 output polarity: 0=default low and high action, 1=default high and low action
#define  RB_PWM1_POLAR      0x01                      // RW, PWM1 output polarity: 0=default low and high action, 1=default high and low action
#define PWM_CONFIG          2
#define  RB_PWM_SYNC_EN     0x80                      // RW, enable sync
#define  RB_PWM_SYNC_START  0x40                      // RW, enable sync start when RB_PWM_SYN_EN=1
#define  RB_PWM4_5_CH       0x10                      // RO, 1=PWM4 channel output 0=PWM5 channel output
#define  RB_PWM4_5_STAG_EN  0x08                      // RW, PWM4/5 stagger output enable: 0=independent output, 1=stagger output
#define  RB_PWM_CYC_MOD     0x06                      // RW, PWM data width mode: 00=8 bits data, 01=7 bits data, 10=6 bits data, 11=16 bits data
#define  RB_PWM_CYCLE_SEL   0x01                      // RW, PWM cycle selection: 0=256/128/64/32 clocks, 1=255/127/63/31 clocks
#define PWM_DMA_CTRL        3
#define  RB_DMA_SEL         0x04                      // RW, RB_PWM_SYN_EN=0: 1=DMA choose 1/2/3 channel output , 0=DMA choose 4/5 channel output ,RB_PWM_SYN_EN=1:  1=DMA choose 1/2/3/4/5 channel output
#define  RB_DMA_ADDR_LOOP   0x02                      // RW, DMA mode:1=DMA loop,0=DMA no loop  
#define  RB_DMA_ENABLE      0x01                      // RW, DMA enable(only 16bit data)  

#define PWM1_DATA_HOLD      4
#define PWM2_DATA_HOLD      5
#define PWM3_DATA_HOLD      6
#define PWM4_DATA_HOLD      7
#define PWM5_DATA_HOLD      8

/* Address space define */
#define BA_CODE             ((PUINT32)0x00000000)     // point code base address
#define SZ_CODE             0x00080000                // code size
#define BA_SFR              ((PUINT32)0x40000000)     // point SFR base address
#define SZ_SFR              0x00010000                // SFR size
#define BA_RAM              ((PUINT32)0x20000000)     // point RAM base address
#define SZ_RAM              0x00003000                // RAM size

/* Special Program Space */
#define CODE_FLASH_SIZE     0x00000                    // size of Data-Flash
#define BOOT_LOAD_ADDR      0x3C000                   // start address of boot loader program
#define BOOT_LOAD_SIZE      0x2000                    // size of boot loader program
#define BOOT_LOAD_CFG       0x3E000                   // start address of configuration information for boot loader program
#define ROM_CFG_ADDR        0x3F000                   // chip configuration information address

/*----- Reference Information --------------------------------------------*/
#define ID_CH572            0x72                      // chip ID
#define ID_CH570            0x70                      // chip ID

/* Interrupt routine address and interrupt number */
// #define INT_ID_TMR0         0                         // interrupt number for Timer0
#define INT_ID_GPIO_A       17                        // interrupt number for GPIO port A
#define INT_ID_SPI          19                        // interrupt number for SPI
#define INT_ID_BLEB         20                        // interrupt number for BLEBB
#define INT_ID_BLEL         21                        // interrupt number for BLELLE
#define INT_ID_USB          22                        // interrupt number for USB
#define INT_ID_TMR          24                        // interrupt number for Timer
#define INT_ID_UART         27                        // interrupt number for UART
#define INT_ID_RTC          28                        // interrupt number for RTC
#define INT_ID_CMP          29      
#define INT_ID_I2C          30      
#define INT_ID_PWMX         31                        // interrupt number for PWM1~5
#define INT_ID_KEYSCAN      33                        // interrupt number for KEYSCAN  
#define INT_ID_ENCODE       34                        // interrupt number for ENCODER  
#define INT_ID_WDOG_BAT     35                        // interrupt number for Watch-Dog timer and Battery low voltage
#define INT_VEC_ENTRY_SZ    4                         // size of each interrupt vector entry
// #define INT_ADDR_TMR0       (INT_ID_TMR0*INT_VEC_ENTRY_SZ+64)    // interrupt vector address for Timer0
#define INT_ADDR_GPIO_A     (INT_ID_GPIO_A*INT_VEC_ENTRY_SZ+64)  // interrupt vector address for GPIO port A
#define INT_ADDR_SPI        (INT_ID_SPI*INT_VEC_ENTRY_SZ+64)    // interrupt vector address for SPI
#define INT_ADDR_BLEB       (INT_ID_BLEB*INT_VEC_ENTRY_SZ+64)    // interrupt vector address for BLEBB
#define INT_ADDR_BLEL       (INT_ID_BLEL*INT_VEC_ENTRY_SZ+64)    // interrupt vector address for BLELLE
#define INT_ADDR_USB        (INT_ID_USB*INT_VEC_ENTRY_SZ+64)     // interrupt vector address for USB
#define INT_ADDR_TMR        (INT_ID_TMR*INT_VEC_ENTRY_SZ+64)    // interrupt vector address for Timer1
#define INT_ADDR_UART       (INT_ID_UART*INT_VEC_ENTRY_SZ+64)   // interrupt vector address for UART
#define INT_ADDR_RTC        (INT_ID_RTC*INT_VEC_ENTRY_SZ+64)     // interrupt vector address for RTC
#define INT_ADDR_CMP        (INT_ID_CMP*INT_VEC_ENTRY_SZ+64)      
#define INT_ADDR_I2C        (INT_ID_I2C*INT_VEC_ENTRY_SZ+64)      
#define INT_ADDR_PWMX       (INT_ID_PWMX*INT_VEC_ENTRY_SZ+64)    // interrupt vector address for PWM1~5
#define INT_ADDR_KEYSCAN    (INT_ID_KEYSCAN*INT_VEC_ENTRY_SZ+64)   // interrupt vector address for KEYSCAN
#define INT_ADDR_ENCODE     (INT_ID_ENCODE*INT_VEC_ENTRY_SZ+64)   // interrupt vector address for ENCODER
#define INT_ADDR_WDOG_BAT   (INT_ID_WDOG_BAT*INT_VEC_ENTRY_SZ+64) // interrupt vector address for Watch-Dog timer and Battery low voltage

#ifndef TABLE_IRQN
#define __PFIC_PRIO_BITS          2 /*!< uses 8 Bits for the Priority Levels    */
#define __Vendor_SysTickConfig    0 /*!< Set to 1 if different SysTick Config is used */
typedef enum IRQn
{
  Reset_IRQn                    =  0,       
  NMI_IRQn                      =  2,       
  EXC_IRQn                      =  3,       
  ECALL_M_IRQn                  =  5,       
  ECALL_U_IRQn                  =  8,       
  BREAKPOINT_IRQn               =  9,       
  SysTick_IRQn                  =  12,      
  SWI_IRQn                      =  14,      
  GPIO_A_IRQn                   =  17,      
  SPI_IRQn                      =  19,      
  BLEB_IRQn                     =  20,      
  BLEL_IRQn                     =  21,      
  USB_IRQn                      =  22,      
  TMR_IRQn                      =  24,      
  UART_IRQn                     =  27,      
  RTC_IRQn                      =  28,      
  CMP_IRQn                      =  29,      
  I2C_IRQn                      =  30,      
  PWMX_IRQn                     =  31,      
  KEYSCAN_IRQn                  =  33,      
  ENCODE_IRQn                   =  34,      
  WDOG_BAT_IRQn                 =  35       
} IRQn_Type;
#endif


#ifdef __cplusplus
}
#endif

#endif  // __CH572SFR_H__


#ifndef __CH572USBSFR_H__
#define __CH572USBSFR_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* usb addresses         
//      USB:     +8000H - 83FFH                                                    */
#define USB_BASE_ADDR              (0x40008000)
#define BA_USB              ((PUINT8V)0x40008000)     // point USB base address

/*       USB  */
#define R32_USB_CONTROL     (*((PUINT32V)0x40008000)) // USB control & interrupt enable & device address
#define R8_USB_CTRL         (*((PUINT8V)0x40008000))  // USB base control
#define  RB_UC_HOST_MODE    0x80      // enable USB host mode: 0=device mode, 1=host mode
#define  RB_UC_LOW_SPEED    0x40      // enable USB low speed: 0=12Mbps, 1=1.5Mbps
#define  RB_UC_DEV_PU_EN    0x20      // USB device enable and internal pullup resistance enable
#define  RB_UC_SYS_CTRL1    0x20      // USB system control high bit
#define  RB_UC_SYS_CTRL0    0x10      // USB system control low bit
#define  MASK_UC_SYS_CTRL   0x30      // bit mask of USB system control
// bUC_HOST_MODE & bUC_SYS_CTRL1 & bUC_SYS_CTRL0: USB system control
//   0 00: disable USB device and disable internal pullup resistance
//   0 01: enable USB device and disable internal pullup resistance, need RB_PIN_USB_DP_PU=1 or need external pullup resistance
//   0 1x: enable USB device and enable internal pullup resistance
//   1 00: enable USB host and normal status
//   1 01: enable USB host and force UDP/UDM output SE0 state
//   1 10: enable USB host and force UDP/UDM output J state
//   1 11: enable USB host and force UDP/UDM output resume or K state
#define  RB_UC_INT_BUSY     0x08      // enable automatic responding busy for device mode or automatic pause for host mode during interrupt flag UIF_TRANSFER valid
#define  RB_UC_RESET_SIE    0x04      // force reset USB SIE, need software clear
#define  RB_UC_CLR_ALL      0x02      // force clear FIFO and count of USB
#define  RB_UC_DMA_EN       0x01      // DMA enable and DMA interrupt enable for USB

#define R8_UDEV_CTRL        (*((PUINT8V)0x40008001))  // USB device physical prot control
#define  RB_UD_PD_DIS       0x80      // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define  RB_UD_DP_PIN       0x20      // ReadOnly: indicate current UDP pin level
#define  RB_UD_DM_PIN       0x10      // ReadOnly: indicate current UDM pin level
#define  RB_UD_LOW_SPEED    0x04      // enable USB physical port low speed: 0=full speed, 1=low speed
#define  RB_UD_GP_BIT       0x02      // general purpose bit
#define  RB_UD_PORT_EN      0x01      // enable USB physical port I/O: 0=disable, 1=enable

#define R8_UHOST_CTRL       R8_UDEV_CTRL              // USB host physical prot control
#define  RB_UH_PD_DIS       0x80      // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define  RB_UH_DP_PIN       0x20      // ReadOnly: indicate current UDP pin level
#define  RB_UH_DM_PIN       0x10      // ReadOnly: indicate current UDM pin level
#define  RB_UH_LOW_SPEED    0x04      // enable USB port low speed: 0=full speed, 1=low speed
#define  RB_UH_BUS_RESET    0x02      // control USB bus reset: 0=normal, 1=force bus reset
#define  RB_UH_PORT_EN      0x01      // enable USB port: 0=disable, 1=enable port, automatic disabled if USB device detached

#define R8_USB_INT_EN       (*((PUINT8V)0x40008002))  // USB interrupt enable
#define  RB_UIE_DEV_SOF     0x80      // enable interrupt for SOF received for USB device mode
#define  RB_UIE_DEV_NAK     0x40      // enable interrupt for NAK responded for USB device mode
#define  RB_MOD_1_WIRE      0x20      // enable single wire mode
#define  RB_UIE_FIFO_OV     0x10      // enable interrupt for FIFO overflow
#define  RB_UIE_HST_SOF     0x08      // enable interrupt for host SOF timer action for USB host mode
#define  RB_UIE_SUSPEND     0x04      // enable interrupt for USB suspend or resume event
#define  RB_UIE_TRANSFER    0x02      // enable interrupt for USB transfer completion
#define  RB_UIE_DETECT      0x01      // enable interrupt for USB device detected event for USB host mode
#define  RB_UIE_BUS_RST     0x01      // enable interrupt for USB bus reset event for USB device mode

#define R8_USB_DEV_AD       (*((PUINT8V)0x40008003))  // USB device address
#define  RB_UDA_GP_BIT      0x80      // general purpose bit
#define  MASK_USB_ADDR      0x7F      // bit mask for USB device address

#define R32_USB_STATUS      (*((PUINT32V)0x40008004)) // USB miscellaneous status & interrupt flag & interrupt status
#define R8_USB_MIS_ST       (*((PUINT8V)0x40008005))  // USB miscellaneous status
#define  RB_UMS_SOF_PRES    0x80      // RO, indicate host SOF timer presage status
#define  RB_UMS_SOF_ACT     0x40      // RO, indicate host SOF timer action status for USB host
#define  RB_UMS_SIE_FREE    0x20      // RO, indicate USB SIE free status
#define  RB_UMS_R_FIFO_RDY  0x10      // RO, indicate USB receiving FIFO ready status (not empty)
#define  RB_UMS_BUS_RESET   0x08      // RO, indicate USB bus reset status
#define  RB_UMS_SUSPEND     0x04      // RO, indicate USB suspend status
#define  RB_UMS_DM_LEVEL    0x02      // RO, indicate UDM level saved at device attached to USB host
#define  RB_UMS_DEV_ATTACH  0x01      // RO, indicate device attached status on USB host

#define R8_USB_INT_FG       (*((PUINT8V)0x40008006))  // USB interrupt flag
#define  RB_U_IS_NAK        0x80    // RO, indicate current USB transfer is NAK received
#define  RB_U_TOG_OK        0x40    // RO, indicate current USB transfer toggle is OK
#define  RB_U_SIE_FREE      0x20    // RO, indicate USB SIE free status
#define  RB_UIF_FIFO_OV     0x10    // FIFO overflow interrupt flag for USB, direct bit address clear or write 1 to clear
#define  RB_UIF_HST_SOF     0x08    // host SOF timer interrupt flag for USB host, direct bit address clear or write 1 to clear
#define  RB_UIF_SUSPEND     0x04    // USB suspend or resume event interrupt flag, direct bit address clear or write 1 to clear
#define  RB_UIF_TRANSFER    0x02    // USB transfer completion interrupt flag, direct bit address clear or write 1 to clear
#define  RB_UIF_DETECT      0x01    // device detected event interrupt flag for USB host mode, direct bit address clear or write 1 to clear
#define  RB_UIF_BUS_RST     0x01    // bus reset event interrupt flag for USB device mode, direct bit address clear or write 1 to clear

#define R8_USB_INT_ST       (*((PUINT8V)0x40008007))  // USB interrupt status
#define  RB_UIS_SETUP_ACT   0x80      // RO, indicate SETUP token & 8 bytes setup request received for USB device mode
#define  RB_UIS_TOG_OK      0x40      // RO, indicate current USB transfer toggle is OK
#define  RB_UIS_TOKEN1      0x20      // RO, current token PID code bit 1 received for USB device mode
#define  RB_UIS_TOKEN0      0x10      // RO, current token PID code bit 0 received for USB device mode
#define  MASK_UIS_TOKEN     0x30      // RO, bit mask of current token PID code received for USB device mode
#define  UIS_TOKEN_OUT      0x00
#define  UIS_TOKEN_SOF      0x10
#define  UIS_TOKEN_IN       0x20
#define  UIS_TOKEN_SETUP    0x30
// bUIS_TOKEN1 & bUIS_TOKEN0: current token PID code received for USB device mode, keep last status during SETUP token, clear RB_UIF_TRANSFER ( RB_UIF_TRANSFER from 1 to 0 ) to set free
//   00: OUT token PID received
//   01: SOF token PID received
//   10: IN token PID received
//   11: free
#define  MASK_UIS_ENDP      0x0F      // RO, bit mask of current transfer endpoint number for USB device mode
#define  MASK_UIS_H_RES     0x0F      // RO, bit mask of current transfer handshake response for USB host mode: 0000=no response, time out from device, others=handshake response PID received

#define R8_USB_RX_LEN       (*((PUINT8V)0x40008008))  // USB receiving length
#define R32_USB_BUF_MODE    (*((PUINT32V)0x4000800C)) // USB endpoint buffer mode
#define R8_UEP4_1_MOD       (*((PUINT8V)0x4000800C))  // endpoint 4/1 mode
#define  RB_UEP1_RX_EN      0x80      // enable USB endpoint 1 receiving (OUT)
#define  RB_UEP1_TX_EN      0x40      // enable USB endpoint 1 transmittal (IN)
#define  RB_UEP1_BUF_MOD    0x10      // buffer mode of USB endpoint 1
// bUEPn_RX_EN & bUEPn_TX_EN & bUEPn_BUF_MOD: USB endpoint 1/2/3 buffer mode, buffer start address is UEPn_DMA
//   0 0 x:  disable endpoint and disable buffer
//   1 0 0:  64 bytes buffer for receiving (OUT endpoint)
//   1 0 1:  dual 64 bytes buffer by toggle bit bUEP_R_TOG selection for receiving (OUT endpoint), total=128bytes
//   0 1 0:  64 bytes buffer for transmittal (IN endpoint)
//   0 1 1:  dual 64 bytes buffer by toggle bit bUEP_T_TOG selection for transmittal (IN endpoint), total=128bytes
//   1 1 0:  64 bytes buffer for receiving (OUT endpoint) + 64 bytes buffer for transmittal (IN endpoint), total=128bytes
//   1 1 1:  dual 64 bytes buffer by bUEP_R_TOG selection for receiving (OUT endpoint) + dual 64 bytes buffer by bUEP_T_TOG selection for transmittal (IN endpoint), total=256bytes
#define  RB_UEP4_RX_EN      0x08      // enable USB endpoint 4 receiving (OUT)
#define  RB_UEP4_TX_EN      0x04      // enable USB endpoint 4 transmittal (IN)
// bUEP4_RX_EN & bUEP4_TX_EN: USB endpoint 4 buffer mode, buffer start address is UEP0_DMA
//   0 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
//   1 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 receiving (OUT endpoint), total=128bytes
//   0 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=128bytes
//   1 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
//           + 64 bytes buffer for endpoint 4 receiving (OUT endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=192bytes

#define R8_UEP2_3_MOD       (*((PUINT8V)0x4000800D))   // endpoint 2/3 mode
#define  RB_UEP3_RX_EN      0x80      // enable USB endpoint 3 receiving (OUT)
#define  RB_UEP3_TX_EN      0x40      // enable USB endpoint 3 transmittal (IN)
#define  RB_UEP3_BUF_MOD    0x10      // buffer mode of USB endpoint 3
#define  RB_UEP2_RX_EN      0x08      // enable USB endpoint 2 receiving (OUT)
#define  RB_UEP2_TX_EN      0x04      // enable USB endpoint 2 transmittal (IN)
#define  RB_UEP2_BUF_MOD    0x01      // buffer mode of USB endpoint 2

#define R8_UEP567_MOD       (*((PUINT8V)0x4000800E))   // endpoint 5/6/7 mode
#define  RB_UEP7_RX_EN      0x20      // enable USB endpoint 7 receiving (OUT)
#define  RB_UEP7_TX_EN      0x10      // enable USB endpoint 7 transmittal (IN)
#define  RB_UEP6_RX_EN      0x08      // enable USB endpoint 6 receiving (OUT)
#define  RB_UEP6_TX_EN      0x04      // enable USB endpoint 6 transmittal (IN)
#define  RB_UEP5_RX_EN      0x02      // enable USB endpoint 5 receiving (OUT)
#define  RB_UEP5_TX_EN      0x01      // enable USB endpoint 5 transmittal (IN)
// bUEPn_RX_EN & bUEPn_TX_EN: USB endpoint 5/6/7 buffer mode, buffer start address is UEPn_DMA
//   0 0:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for receiving (OUT endpoint)
//   0 1:  64 bytes buffer for transmittal (IN endpoint)
//   1 1:  64 bytes buffer for receiving (OUT endpoint) + 64 bytes buffer for transmittal (IN endpoint), total=128bytes

#define R8_UH_EP_MOD        R8_UEP2_3_MOD             //host endpoint mode
#define  RB_UH_EP_TX_EN     0x40      // enable USB host OUT endpoint transmittal
#define  RB_UH_EP_TBUF_MOD  0x10      // buffer mode of USB host OUT endpoint
// bUH_EP_TX_EN & bUH_EP_TBUF_MOD: USB host OUT endpoint buffer mode, buffer start address is UH_TX_DMA
//   0 x:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for transmittal (OUT endpoint)
//   1 1:  dual 64 bytes buffer by toggle bit bUH_T_TOG selection for transmittal (OUT endpoint), total=128bytes
#define  RB_UH_EP_RX_EN     0x08      // enable USB host IN endpoint receiving
#define  RB_UH_EP_RBUF_MOD  0x01      // buffer mode of USB host IN endpoint
// bUH_EP_RX_EN & bUH_EP_RBUF_MOD: USB host IN endpoint buffer mode, buffer start address is UH_RX_DMA
//   0 x:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for receiving (IN endpoint)
//   1 1:  dual 64 bytes buffer by toggle bit bUH_R_TOG selection for receiving (IN endpoint), total=128bytes

#define R16_UEP0_DMA        (*((PUINT16V)0x40008010)) // endpoint 0 DMA buffer address
#define R16_UEP1_DMA        (*((PUINT16V)0x40008014)) // endpoint 1 DMA buffer address
#define R16_UEP2_DMA        (*((PUINT16V)0x40008018)) // endpoint 2 DMA buffer address
#define R16_UH_RX_DMA       R16_UEP2_DMA              // host rx endpoint buffer address
#define R16_UEP3_DMA        (*((PUINT16V)0x4000801C)) // endpoint 3 DMA buffer address
#define R16_UH_TX_DMA       R16_UEP3_DMA              // host tx endpoint buffer address
#define R16_UEP5_DMA        (*((PUINT16V)0x40008054)) // endpoint 5 DMA buffer address
#define R16_UEP6_DMA        (*((PUINT16V)0x40008058)) // endpoint 6 DMA buffer address
#define R16_UEP7_DMA        (*((PUINT16V)0x4000805C)) // endpoint 7 DMA buffer address
#define R32_USB_EP0_CTRL    (*((PUINT32V)0x40008020)) // endpoint 0 control & transmittal length
#define R8_UEP0_T_LEN       (*((PUINT8V)0x40008020))  // endpoint 0 transmittal length
#define R8_UEP0_CTRL        (*((PUINT8V)0x40008022))  // endpoint 0 control
#define R32_USB_EP1_CTRL    (*((PUINT32V)0x40008024)) // endpoint 1 control & transmittal length
#define R8_UEP1_T_LEN       (*((PUINT8V)0x40008024))  // endpoint 1 transmittal length
#define R8_UEP1_CTRL        (*((PUINT8V)0x40008026))  // endpoint 1 control
#define  RB_UEP_R_TOG       0x80      // expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
#define  RB_UEP_T_TOG       0x40      // prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
#define  RB_UEP_AUTO_TOG    0x10      // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle
#define  RB_UEP_R_RES1      0x08      // handshake response type high bit for USB endpoint X receiving (OUT)
#define  RB_UEP_R_RES0      0x04      // handshake response type low bit for USB endpoint X receiving (OUT)
#define  MASK_UEP_R_RES     0x0C      // bit mask of handshake response type for USB endpoint X receiving (OUT)
#define  UEP_R_RES_ACK      0x00
#define  UEP_R_RES_TOUT     0x04
#define  UEP_R_RES_NAK      0x08
#define  UEP_R_RES_STALL    0x0C
// RB_UEP_R_RES1 & RB_UEP_R_RES0: handshake response type for USB endpoint X receiving (OUT)
//   00: ACK (ready)
//   01: no response, time out to host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)
#define  RB_UEP_T_RES1      0x02      // handshake response type high bit for USB endpoint X transmittal (IN)
#define  RB_UEP_T_RES0      0x01      // handshake response type low bit for USB endpoint X transmittal (IN)
#define  MASK_UEP_T_RES     0x03      // bit mask of handshake response type for USB endpoint X transmittal (IN)
#define  UEP_T_RES_ACK      0x00
#define  UEP_T_RES_TOUT     0x01
#define  UEP_T_RES_NAK      0x02
#define  UEP_T_RES_STALL    0x03
// bUEP_T_RES1 & bUEP_T_RES0: handshake response type for USB endpoint X transmittal (IN)
//   00: DATA0 or DATA1 then expecting ACK (ready)
//   01: DATA0 or DATA1 then expecting no response, time out from host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)

#define R8_UH_SETUP         R8_UEP1_CTRL              // host aux setup
#define  RB_UH_PRE_PID_EN   0x80      // USB host PRE PID enable for low speed device via hub
#define  RB_UH_SOF_EN       0x40      // USB host automatic SOF enable

#define R32_USB_EP2_CTRL    (*((PUINT32V)0x40008028)) // endpoint 2 control & transmittal length
#define R8_UEP2_T_LEN       (*((PUINT8V)0x40008028))  // endpoint 2 transmittal length
#define R8_UEP2_CTRL        (*((PUINT8V)0x4000802A))  // endpoint 2 control

#define R8_UH_EP_PID        R8_UEP2_T_LEN             // host endpoint and PID
#define  MASK_UH_TOKEN      0xF0      // bit mask of token PID for USB host transfer
#define  MASK_UH_ENDP       0x0F      // bit mask of endpoint number for USB host transfer

#define R8_UH_RX_CTRL       R8_UEP2_CTRL              // host receiver endpoint control
#define  RB_UH_R_TOG        0x80      // expected data toggle flag of host receiving (IN): 0=DATA0, 1=DATA1
#define  RB_UH_R_AUTO_TOG   0x10      // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
#define  RB_UH_R_RES        0x04      // prepared handshake response type for host receiving (IN): 0=ACK (ready), 1=no response, time out to device, for isochronous transactions

#define R32_USB_EP3_CTRL    (*((PUINT32V)0x4000802c)) // endpoint 3 control & transmittal length
#define R8_UEP3_T_LEN       (*((PUINT8V)0x4000802c))  // endpoint 3 transmittal length
#define R8_UEP3_CTRL        (*((PUINT8V)0x4000802e))  // endpoint 3 control
#define R8_UH_TX_LEN        R8_UEP3_T_LEN             // host transmittal endpoint transmittal length

#define R8_UH_TX_CTRL       R8_UEP3_CTRL              // host transmittal endpoint control
#define  RB_UH_T_TOG        0x40      // prepared data toggle flag of host transmittal (SETUP/OUT): 0=DATA0, 1=DATA1
#define  RB_UH_T_AUTO_TOG   0x10      // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
#define  RB_UH_T_RES        0x01      // expected handshake response type for host transmittal (SETUP/OUT): 0=ACK (ready), 1=no response, time out from device, for isochronous transactions

#define R32_USB_EP4_CTRL    (*((PUINT32V)0x40008030)) // endpoint 4 control & transmittal length
#define R8_UEP4_T_LEN       (*((PUINT8V)0x40008030))  // endpoint 4 transmittal length
#define R8_UEP4_CTRL        (*((PUINT8V)0x40008032))  // endpoint 4 control

#define R32_USB_EP5_CTRL    (*((PUINT32V)0x40008064)) // endpoint 5 control & transmittal length
#define R8_UEP5_T_LEN       (*((PUINT8V)0x40008064))  // endpoint 5 transmittal length
#define R8_UEP5_CTRL        (*((PUINT8V)0x40008066))  // endpoint 5 control

#define R32_USB_EP6_CTRL    (*((PUINT32V)0x40008068)) // endpoint 6 control & transmittal length
#define R8_UEP6_T_LEN       (*((PUINT8V)0x40008068))  // endpoint 6 transmittal length
#define R8_UEP6_CTRL        (*((PUINT8V)0x4000806A))  // endpoint 6 control

#define R32_USB_EP7_CTRL    (*((PUINT32V)0x4000806C)) // endpoint 7 control & transmittal length
#define R8_UEP7_T_LEN       (*((PUINT8V)0x4000806C))  // endpoint 7 transmittal length
#define R8_UEP7_CTRL        (*((PUINT8V)0x4000806E))  // endpoint 7 control

#ifdef __cplusplus
}
#endif

#endif //__CH572USBSFR_H__


#ifndef __USB_TYPE__
#define __USB_TYPE__

#ifdef __cplusplus
extern "C" {
#endif

/*----- USB constant and structure define --------------------------------*/

/* USB PID */
#ifndef USB_PID_SETUP
#define USB_PID_NULL            0x00    /* reserved PID */
#define USB_PID_SOF             0x05
#define USB_PID_SETUP           0x0D
#define USB_PID_IN              0x09
#define USB_PID_OUT             0x01
#define USB_PID_ACK             0x02
#define USB_PID_NAK             0x0A
#define USB_PID_STALL           0x0E
#define USB_PID_DATA0           0x03
#define USB_PID_DATA1           0x0B
#define USB_PID_PRE             0x0C
#endif

/* USB standard device request code */
#ifndef USB_GET_DESCRIPTOR
#define USB_GET_STATUS          0x00
#define USB_CLEAR_FEATURE       0x01
#define USB_SET_FEATURE         0x03
#define USB_SET_ADDRESS         0x05
#define USB_GET_DESCRIPTOR      0x06
#define USB_SET_DESCRIPTOR      0x07
#define USB_GET_CONFIGURATION   0x08
#define USB_SET_CONFIGURATION   0x09
#define USB_GET_INTERFACE       0x0A
#define USB_SET_INTERFACE       0x0B
#define USB_SYNCH_FRAME         0x0C
#endif

/* USB hub class request code */
#ifndef HUB_GET_DESCRIPTOR
#define HUB_GET_STATUS          0x00
#define HUB_CLEAR_FEATURE       0x01
#define HUB_GET_STATE           0x02
#define HUB_SET_FEATURE         0x03
#define HUB_GET_DESCRIPTOR      0x06
#define HUB_SET_DESCRIPTOR      0x07
#endif

/* USB HID class request code */
#ifndef HID_GET_REPORT
#define HID_GET_REPORT          0x01
#define HID_GET_IDLE            0x02
#define HID_GET_PROTOCOL        0x03
#define HID_SET_REPORT          0x09
#define HID_SET_IDLE            0x0A
#define HID_SET_PROTOCOL        0x0B
#endif

/* Bit define for USB request type */
#ifndef USB_REQ_TYP_MASK
#define USB_REQ_TYP_IN          0x80            /* control IN, device to host */
#define USB_REQ_TYP_OUT         0x00            /* control OUT, host to device */
#define USB_REQ_TYP_READ        0x80            /* control read, device to host */
#define USB_REQ_TYP_WRITE       0x00            /* control write, host to device */
#define USB_REQ_TYP_MASK        0x60            /* bit mask of request type */
#define USB_REQ_TYP_STANDARD    0x00
#define USB_REQ_TYP_CLASS       0x20
#define USB_REQ_TYP_VENDOR      0x40
#define USB_REQ_TYP_RESERVED    0x60
#define USB_REQ_RECIP_MASK      0x1F            /* bit mask of request recipient */
#define USB_REQ_RECIP_DEVICE    0x00
#define USB_REQ_RECIP_INTERF    0x01
#define USB_REQ_RECIP_ENDP      0x02
#define USB_REQ_RECIP_OTHER     0x03
#endif

/* USB request type for hub class request */
#ifndef HUB_GET_HUB_DESCRIPTOR
#define HUB_CLEAR_HUB_FEATURE   0x20
#define HUB_CLEAR_PORT_FEATURE  0x23
#define HUB_GET_BUS_STATE       0xA3
#define HUB_GET_HUB_DESCRIPTOR  0xA0
#define HUB_GET_HUB_STATUS      0xA0
#define HUB_GET_PORT_STATUS     0xA3
#define HUB_SET_HUB_DESCRIPTOR  0x20
#define HUB_SET_HUB_FEATURE     0x20
#define HUB_SET_PORT_FEATURE    0x23
#endif

/* Hub class feature selectors */
#ifndef HUB_PORT_RESET
#define HUB_C_HUB_LOCAL_POWER   0
#define HUB_C_HUB_OVER_CURRENT  1
#define HUB_PORT_CONNECTION     0
#define HUB_PORT_ENABLE         1
#define HUB_PORT_SUSPEND        2
#define HUB_PORT_OVER_CURRENT   3
#define HUB_PORT_RESET          4
#define HUB_PORT_POWER          8
#define HUB_PORT_LOW_SPEED      9
#define HUB_C_PORT_CONNECTION   16
#define HUB_C_PORT_ENABLE       17
#define HUB_C_PORT_SUSPEND      18
#define HUB_C_PORT_OVER_CURRENT 19
#define HUB_C_PORT_RESET        20
#endif

/* USB descriptor type */
#ifndef USB_DESCR_TYP_DEVICE
#define USB_DESCR_TYP_DEVICE    0x01
#define USB_DESCR_TYP_CONFIG    0x02
#define USB_DESCR_TYP_STRING    0x03
#define USB_DESCR_TYP_INTERF    0x04
#define USB_DESCR_TYP_ENDP      0x05
#define USB_DESCR_TYP_QUALIF    0x06
#define USB_DESCR_TYP_SPEED     0x07
#define USB_DESCR_TYP_OTG       0x09
#define USB_DESCR_TYP_HID       0x21
#define USB_DESCR_TYP_REPORT    0x22
#define USB_DESCR_TYP_PHYSIC    0x23
#define USB_DESCR_TYP_CS_INTF   0x24
#define USB_DESCR_TYP_CS_ENDP   0x25
#define USB_DESCR_TYP_HUB       0x29
#endif

/* USB device class */
#ifndef USB_DEV_CLASS_HUB
#define USB_DEV_CLASS_RESERVED  0x00
#define USB_DEV_CLASS_AUDIO     0x01
#define USB_DEV_CLASS_COMMUNIC  0x02
#define USB_DEV_CLASS_HID       0x03
#define USB_DEV_CLASS_MONITOR   0x04
#define USB_DEV_CLASS_PHYSIC_IF 0x05
#define USB_DEV_CLASS_POWER     0x06
#define USB_DEV_CLASS_PRINTER   0x07
#define USB_DEV_CLASS_STORAGE   0x08
#define USB_DEV_CLASS_HUB       0x09
#define USB_DEV_CLASS_VEN_SPEC  0xFF
#endif

/* USB endpoint type and attributes */
#ifndef USB_ENDP_TYPE_MASK
#define USB_ENDP_DIR_MASK       0x80
#define USB_ENDP_ADDR_MASK      0x0F
#define USB_ENDP_TYPE_MASK      0x03
#define USB_ENDP_TYPE_CTRL      0x00
#define USB_ENDP_TYPE_ISOCH     0x01
#define USB_ENDP_TYPE_BULK      0x02
#define USB_ENDP_TYPE_INTER     0x03
#endif

#ifndef USB_DEVICE_ADDR
#define USB_DEVICE_ADDR         0x02    /*default addr of USB */
#endif
#ifndef DEFAULT_ENDP0_SIZE
#define DEFAULT_ENDP0_SIZE      8       /* default maximum packet size for endpoint 0 */
#endif
#ifndef MAX_PACKET_SIZE
#define MAX_PACKET_SIZE         64      /* maximum packet size */
#endif
#ifndef USB_BO_CBW_SIZE
#define USB_BO_CBW_SIZE         0x1F    /* total length of CBW command block */
#define USB_BO_CSW_SIZE         0x0D    /* total length of CSW command state block */
#endif
#ifndef USB_BO_CBW_SIG
#define USB_BO_CBW_SIG          0x43425355    /* identification mark of CBW command block 'USBC' */
#define USB_BO_CSW_SIG          0x53425355    /* identification mark of CSW command state block 'USBC'USBS' */
#endif

#ifndef __PACKED
#define __PACKED               __attribute__((packed))
#endif

typedef struct __PACKED _USB_SETUP_REQ {
    UINT8 bRequestType;
    UINT8 bRequest;
    UINT16 wValue;
    UINT16 wIndex;
    UINT16 wLength;
} USB_SETUP_REQ, *PUSB_SETUP_REQ;

typedef struct __PACKED _USB_DEVICE_DESCR {
    UINT8 bLength;
    UINT8 bDescriptorType;
    UINT16 bcdUSB;
    UINT8 bDeviceClass;
    UINT8 bDeviceSubClass;
    UINT8 bDeviceProtocol;
    UINT8 bMaxPacketSize0;
    UINT16 idVendor;
    UINT16 idProduct;
    UINT16 bcdDevice;
    UINT8 iManufacturer;
    UINT8 iProduct;
    UINT8 iSerialNumber;
    UINT8 bNumConfigurations;
} USB_DEV_DESCR, *PUSB_DEV_DESCR;

typedef struct __PACKED _USB_CONFIG_DESCR {
    UINT8 bLength;
    UINT8 bDescriptorType;
    UINT16 wTotalLength;
    UINT8 bNumInterfaces;
    UINT8 bConfigurationValue;
    UINT8 iConfiguration;
    UINT8 bmAttributes;
    UINT8 MaxPower;
} USB_CFG_DESCR, *PUSB_CFG_DESCR;

typedef struct __PACKED _USB_INTERF_DESCR {
    UINT8 bLength;
    UINT8 bDescriptorType;
    UINT8 bInterfaceNumber;
    UINT8 bAlternateSetting;
    UINT8 bNumEndpoints;
    UINT8 bInterfaceClass;
    UINT8 bInterfaceSubClass;
    UINT8 bInterfaceProtocol;
    UINT8 iInterface;
} USB_ITF_DESCR, *PUSB_ITF_DESCR;

typedef struct __PACKED _USB_ENDPOINT_DESCR {
    UINT8 bLength;
    UINT8 bDescriptorType;
    UINT8 bEndpointAddress;
    UINT8 bmAttributes;
    UINT16 wMaxPacketSize;
    UINT8 bInterval;
} USB_ENDP_DESCR, *PUSB_ENDP_DESCR;

typedef struct __PACKED _USB_CONFIG_DESCR_LONG {
    USB_CFG_DESCR   cfg_descr;
    USB_ITF_DESCR   itf_descr;
    USB_ENDP_DESCR  endp_descr[1];
} USB_CFG_DESCR_LONG, *PUSB_CFG_DESCR_LONG;

typedef USB_CFG_DESCR_LONG *PXUSB_CFG_DESCR_LONG;

typedef struct __PACKED  _USB_HUB_DESCR {
    UINT8 bDescLength;
    UINT8 bDescriptorType;
    UINT8 bNbrPorts;
    UINT8 wHubCharacteristicsL;
    UINT8 wHubCharacteristicsH;
    UINT8 bPwrOn2PwrGood;
    UINT8 bHubContrCurrent;
    UINT8 DeviceRemovable;
    UINT8 PortPwrCtrlMask;
} USB_HUB_DESCR, *PUSB_HUB_DESCR;

typedef USB_HUB_DESCR  *PXUSB_HUB_DESCR;

typedef struct __PACKED  _USB_HID_DESCR {
    UINT8 bLength;
    UINT8 bDescriptorType;
    UINT16 bcdHID;
    UINT8 bCountryCode;
    UINT8 bNumDescriptors;
    UINT8 bDescriptorTypeX;
    UINT8 wDescriptorLengthL;
    UINT8 wDescriptorLengthH;
} USB_HID_DESCR, *PUSB_HID_DESCR;

typedef USB_HID_DESCR *PXUSB_HID_DESCR;

typedef struct __PACKED  _UDISK_BOC_CBW {         /* command of BulkOnly USB-FlashDisk */
    UINT32 mCBW_Sig;
    UINT32 mCBW_Tag;
    UINT32 mCBW_DataLen;                /* uppest byte of data length, always is 0 */
    UINT8 mCBW_Flag;                    /* transfer direction and etc. */
    UINT8 mCBW_LUN;
    UINT8 mCBW_CB_Len;                  /* length of command block */
    UINT8 mCBW_CB_Buf[16];              /* command block buffer */
} UDISK_BOC_CBW, *PUDISK_BOC_CBW;

typedef UDISK_BOC_CBW  *PXUDISK_BOC_CBW;

typedef struct __PACKED  _UDISK_BOC_CSW {         /* status of BulkOnly USB-FlashDisk */
    UINT32 mCSW_Sig;
    UINT32 mCSW_Tag;
    UINT32 mCSW_Residue;                /* return: remainder bytes */
    UINT8 mCSW_Status;                  /* return: result status */
} UDISK_BOC_CSW, *PUDISK_BOC_CSW;

typedef UDISK_BOC_CSW  *PXUDISK_BOC_CSW;

#ifdef __cplusplus
}
#endif

#endif  // __USB_TYPE__
