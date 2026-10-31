
// clang-format off
#include <stdint.h>
#include <stdbool.h>
#include "SimpleUsbCdc.h"
#include "USBconstant.h"
#include "USBhandler.h"
// clang-format on

// clang-format off
extern uint8_t Ep0Buffer[];
extern uint8_t Ep2Buffer[];
// clang-format on

#define LINE_CODEING_SIZE 7
uint8_t LineCoding[LINE_CODEING_SIZE] = {
    0x00, 0xe1, 0x00, 0x00,
    0x00, 0x00, 0x08}; // Initialize for baudrate 57600, 1 stopbit, No parity,
                       // eight data bits

#if defined(CH585) || defined(CH32V30x)
volatile uint16_t USBByteCountEP2 =
    0; // Bytes of received data on USB endpoint
volatile uint16_t USBBufOutPointEP2 = 0; // Data pointer for fetching
#else
volatile uint8_t USBByteCountEP2 =
    0; // Bytes of received data on USB endpoint
volatile uint8_t USBBufOutPointEP2 = 0; // Data pointer for fetching
#endif

volatile uint8_t UpPoint2BusyFlag = 0; // Flag of whether upload pointer is busy
volatile uint8_t controlLineState = 0;

#if defined(CH585) || defined(CH32V30x)
uint16_t usbWritePointer = 0;
#else
uint8_t usbWritePointer = 0;
#endif

void __attribute__((weak)) preAppJumpBoot(void) {
  // empty weak function, can be overridden by user code
}

#if defined (CH573)
// This function can not be in RAM, as we are copying bootloader code to RAM.
__attribute__((noinline, used)) static void jump_isprom_strip()
{
  R8_USB_CTRL = RB_UC_RESET_SIE | RB_UC_CLR_ALL;
  for (volatile uint32_t i = 0; i < 1000; ++i) {
    __nop();
  }
  R8_USB_INT_EN = 0x00;
  R8_USB_CTRL   = 0x00;
  R8_UDEV_CTRL  = RB_UD_PD_DIS;
  PFIC_DisableIRQ(USB_IRQn);

  PFIC_DisableIRQ(SysTick_IRQn);
  SysTick->CTLR = 0;

  for (volatile uint32_t i = 0; i < 10000; ++i) {
    __nop();
  }

  // after rebase dumped bootloader to 0x0, Ghidra shows:
  // gp = 0x20005e10;
  // puVar1 = &DAT_ram_000000a8;
  // puVar2 = (undefined4 *)&DAT_ram_20003800;
  volatile uint32_t *dst = (volatile uint32_t *)0x20003800;
  const uint32_t *src = (const uint32_t *)(0x00078000 + 0xa8);
  const uint32_t words = (0x1EC0-0xA8) >> 2;
  for (uint32_t i = 0; i < words; ++i) {
      dst[i] = src[i];
  }

  //function reading PB22 is at 00078146, we need to edit     ram:00078154 05 89           c.andi     a0,0x1
  *(int16_t*)(0x200038AC) = 0x4505; //li a0,1
  
  // DAT_ram_20005618 -> 0x200059a0
  volatile uint32_t *clr = (volatile uint32_t *)0x20005618;
  for (uint32_t i = 0; i < ((0x388) >> 2); ++i) {
      clr[i] = 0;
  }

  //0x20004948 should be 0x000791f0, it is the bootloader_main
  asm( "la gp, 0x20005e10\n"
      ".option arch, +zicsr\n"
      "li t0, 0x20004948\n"
      "jr t0\n");
    // "csrw mepc, t0\n" // __set_MEPC is not available here
    // "mret\n");
}

__attribute__((section(".highcode")))
void APPJumpBoot(void)   //this section of code must run in RAM
{
  if (*((const uint16_t *)(0x00078154)) == 0x8905u) {
    jump_isprom_strip();
  } else {
    while(FLASH_EEPROM_CMD(0x01, 0, NULL, 4096) != 0x00) {
      ;//ROM erase 4K size at address 0
    }
    FLASH_EEPROM_CMD(0x04, 0, NULL, 0);   //reset flash
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;
    //SAFEOPERATE;
    asm("nop");
    asm("nop");
    R16_INT32K_TUNE = 0xFFFF;
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    R8_SAFE_ACCESS_SIG = 0;//run to execute reset, reset type will be power-up reset.
  }
  while(1);//Make bootloader think the chip is empty (first 4 bytes are 0xFF)
}
#elif defined (CH572)

// This function can not be in RAM, as we are copying bootloader code to RAM.
__attribute__((noinline, used)) static void jump_isprom_strip()
{
  R8_USB_CTRL = RB_UC_RESET_SIE | RB_UC_CLR_ALL;
  for (volatile uint32_t i = 0; i < 1000; ++i) {
    __nop();
  }
  R8_USB_INT_EN = 0x00;
  R8_USB_CTRL   = 0x00;
  R8_UDEV_CTRL  = RB_UD_PD_DIS;
  PFIC_DisableIRQ(USB_IRQn);

  PFIC_DisableIRQ(SysTick_IRQn);
  SysTick->CTLR = 0;

  for (volatile uint32_t i = 0; i < 10000; ++i) {
    __nop();
  }

  /* Word copy like WCH FLASH_ROM_READ: flash ROM reads are naturally 32-bit. */
  // RAM 0x20000000 is from 0x0003c0c0 in flash

  // after rebase dumped bootloader to 0x0, Ghidra shows:
  // gp = 0x20002398;
  // puVar1 = &DAT_ram_000000c0;
  // puVar2 = (undefined4 *)&DAT_ram_20000000;
  volatile uint32_t *dst = (volatile uint32_t *)0x20000000;
  const uint32_t *src = (const uint32_t *)(0x0003c000 + 0xc0);
  const uint32_t words = 0x2000u >> 2;
  for (uint32_t i = 0; i < words; ++i) {
      dst[i] = src[i];
  }

  *(int32_t*)(0x20000100 + 0xc) = 0x00014505; // nop \n li a0,1, patch PA1 detection
  
  // DAT_ram_20001ba0 -> 0x20002014
  volatile uint32_t *clr = (volatile uint32_t *)0x20001ba0;
  for (uint32_t i = 0; i < (0x0474u >> 2); ++i) {
      clr[i] = 0;
  }

  //0x20001196 should be 0x3D256, it is the bootloader_main
  asm( "la gp, 0x20002398\n"
      ".option arch, +zicsr\n"
      "li t0, 0x20001196\n"
      "jr t0\n");
    // "csrw mepc, t0\n" // __set_MEPC is not available here
    // "mret\n");
}

__attribute__((section(".highcode")))
void APPJumpBoot(void)   //this section of code must run in RAM
{

  R32_PA_DIR &= ~(1<<2);
  R32_PA_PU &= ~(1<<2);
  R16_PIN_ALTERNATE &= ~(1<<2); //RB_PA_DI_DIS
  R32_PA_PD_DRV |= (1<<2);
  //Discharge PA2(RXD) to prevent falling edge in bootloader, and a unexpected 0 in UART
  //flash erase will take time, maybe no delay is needed

  if (*((const uint32_t *)(0x0003c000 + 0xc0 + 0x10c)) == 0x00153513) { //    ram:0003c1cc 13 35 15 00     sltiu      a0,a0,0x1
    jump_isprom_strip();
  }else{
    while(FLASH_EEPROM_CMD(0x01, 0, NULL, 4096) != 0x00) {
      ;//ROM erase 4K size at address 0
    }
    FLASH_EEPROM_CMD(0x04, 0, NULL, 0);   //reset flash

    {
      volatile uint32_t mpie_mie;
      mpie_mie=__risc_v_disable_irq();
      asm volatile("fence.i");
      R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;
      R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;
      asm volatile("fence.i");

      R16_INT_LSI_TUNE = 0xFFFF;

      R8_SAFE_ACCESS_SIG = 0;
      __risc_v_enable_irq(mpie_mie);
      asm volatile("fence.i");
    }

    {
      volatile uint32_t mpie_mie;
      mpie_mie=__risc_v_disable_irq();
      asm volatile("fence.i");
      R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;
      R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;
      asm volatile("fence.i");

      R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET; //run to execute reset, reset type will be power-up reset.
      
      R8_SAFE_ACCESS_SIG = 0;
      __risc_v_enable_irq(mpie_mie);
      asm volatile("fence.i");
    }
  }

  while(1);//Make bootloader think the chip is empty (first 4 bytes are 0xFF)
}
#elif defined (CH585)

// HS detach must be visible on the wire before the FS bootloader attaches.
// A-to-C / USB-A watches D+/D- and accepts a ~100us drop; a Mac USB-C C-C
// port is already in HS and will not re-enumerate from that.
static void usbhs_force_host_disconnect(void)
{
  const uint32_t usb2_dp_dm = bU2DM | bU2DP;

  R8_USB2_INT_EN = 0x00;
  PFIC_DisableIRQ(USB2_DEVICE_IRQn);

  // Drop LPM and device enable, keep PHY awake so HS 45ohm terms actually
  // release (RB_UD_DEV_EN / RB_UD_PHY_SUSPENDM).
  R8_USB2_CTRL = USBHS_UD_PHY_SUSPENDM;
  for (volatile uint32_t i = 0; i < 20000; ++i) {
    __nop();
  }

  // Reset the HS link state machine, then stop advertising HS.
  R8_USB2_CTRL = USBHS_UD_RST_LINK | USBHS_UD_PHY_SUSPENDM;
  R8_USB2_BASE_MODE = USBHS_UD_SPEED_FULL;

  // Disconnect USBHS analog from PB12/PB13 and float the pins.
  R16_PIN_CONFIG &= (uint16_t)~RB_PIN_USB2_EN;
  R32_PB_DIR &= ~usb2_dp_dm;
  R32_PB_PU &= ~usb2_dp_dm;
  R32_PB_PD_DRV &= ~usb2_dp_dm;

  R8_USB2_DEV_AD = 0x00;
  R16_U2EP_TX_EN = 0;
  R16_U2EP_RX_EN = 0;

  // Analog HS disconnect is ~2.5us; Mac C-C debounce is tens of ms.
  for (volatile uint32_t i = 0; i < 1000000; ++i) {
    __nop();
  }

  // Hold SIE/LINK in reset, PHY suspended, PLL off (POR CTRL is 0x07).
  R8_USBHS_PLL_CTRL = 0x00;
  R8_USB2_CTRL = USBHS_UD_RST_SIE | USBHS_UD_CLR_ALL | USBHS_UD_RST_LINK;
}

// This function can not be in RAM, as we are copying bootloader code to RAM.
__attribute__((noinline, used)) static void jump_isprom_strip()
{
  usbhs_force_host_disconnect();

  PFIC_DisableIRQ(SysTick_IRQn);
  SysTick->CTLR = 0;

  volatile uint32_t *dst = (volatile uint32_t *)0x20000000;
  const uint32_t *src = (const uint32_t *)(0x00078000 + 0x88);
  const uint32_t words = 0x2500u >> 2;
  for (uint32_t i = 0; i < words; ++i) {
    dst[i] = src[i];
  }

  //0x78192 is ReadPB11. 0007819c: 0589 c.andi a0,0x1
  *(int16_t*)(0x2000010a + 0xa) = 0x4505; //li a0,1, patch PB11 (option byte is not read correctly) detection
  
  volatile uint32_t *clr = (volatile uint32_t *)0x200024f0;
  for (uint32_t i = 0; i < (0x0560u >> 2); ++i) {
    clr[i] = 0;
  }

  //0x200018c2 -> 0x7994A, main of bootloader 
  asm( "la gp, 0x20002ce8\n"
      ".option arch, +zicsr\n"
      "li t0, 0x200018c2\n"
      "jr t0\n");
  //   "csrw mepc, t0\n" // __set_MEPC is not available here
  //   "mret\n");
}

__attribute__((section(".highcode")))
void APPJumpBoot(void)   //this section of code must run in RAM
{
  //disable tick interrupt and usb interrupt
  PFIC_DisableIRQ(SysTick_IRQn);
  PFIC_DisableIRQ(USB2_DEVICE_IRQn);

  if (*((const uint16_t *)(0x00078000 + 0x88 + 0x10a + 0xa)) == 0x8905) { //    0007819c: 0589 c.andi a0,0x1
    jump_isprom_strip();
  }else{

    while(FLASH_EEPROM_CMD( 0x01, 0, NULL, 4096 ) != 0x00); //ROM erase 4K size at address 0
    FLASH_EEPROM_CMD( 0x04, 0, NULL, 0 );   //reset flash

    {
      volatile uint32_t mpie_mie;
      mpie_mie=__risc_v_disable_irq();
      asm volatile("fence.i");
      R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;
      R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;
      asm volatile("fence.i");

      R16_INT32K_TUNE = 0xFFFF;

      R8_SAFE_ACCESS_SIG = 0;
      __risc_v_enable_irq(mpie_mie);
      asm volatile("fence.i");
    }

    {
      volatile uint32_t mpie_mie;
      mpie_mie=__risc_v_disable_irq();
      asm volatile("fence.i");
      R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;
      R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;
      asm volatile("fence.i");

      R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET; //run to execute reset, reset type will be power-up reset.
      
      R8_SAFE_ACCESS_SIG = 0;
      __risc_v_enable_irq(mpie_mie);
      asm volatile("fence.i");
    }
  }

  while(1);//Make bootloader think the chip is empty (first 4 bytes are 0xFF)
}
#elif defined (CH32X035)
void APPJumpBoot(void)   //this section of code must run in RAM
{
    FLASH->BOOT_MODEKEYR = 0x45670123;
    FLASH->BOOT_MODEKEYR = 0xCDEF89AB;
    FLASH->STATR |= (1<<14);
    //R32_PFIC_SCTLR
    *((uint32_t *)0xE000ED10) = (1<<31);
}
#elif defined (CH32V30x)
void APPJumpBoot(void){
  //Not verified but maybe not doable.
}
#endif


void USBInit(void){
#if defined (CH32X035)
  //USBFS_RCC_Init
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBFS, ENABLE );
  //USBFS_Device_Init( ENABLE , PWR_VDD_SupplyVoltage());
  //GPIO_USB_INIT();
  GPIO_InitTypeDef GPIO_InitStructure = {0};
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_16;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_17;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //PWR_VDD_SupplyVoltage
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR, ENABLE);
  PWR_PVDLevelConfig(PWR_PVDLevel_4V0);
  // Delay_Us(10); // roughly 10us delay
  asm volatile (
    "addi  t0, zero, 33    \n\t" // set loop count (must fit 12-bit signed imm: -2048..+2047)
    "1:                     \n\t"
    "add   zero,zero,zero   \n\t" // 32-bit nop (preferred on CH573/CH32V cores)
    "addi  t0, t0, -1       \n\t" // decrement
    "bnez  t0, 1b           \n\t" // branch if not zero
    :
    :
    : "t0"
  );
  if( PWR_GetFlagStatus(PWR_FLAG_PVDO) == (uint32_t)RESET)
  {
    //VDD > 4.0V
    AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK | USB_PHY_V33)) | UDP_PUE_10K | USB_IOEN;
  }else{
    //VDD < 4.0V
    AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK )) | USB_PHY_V33 | UDP_PUE_1K5 | USB_IOEN;
  }
  PWR_PVDLevelConfig(PWR_PVDLevel_2V1);
#elif defined (CH32V30x)
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );

  RCC_USBCLK48MConfig( RCC_USBCLK48MCLKSource_USBPHY );
  RCC_USBHSPLLCLKConfig( RCC_HSBHSPLLCLKSource_HSE );
  RCC_USBHSConfig( RCC_USBPLL_Div2 );
  RCC_USBHSPLLCKREFCLKConfig( RCC_USBHSPLLCKREFCLK_4M );
  RCC_USBHSPHYPLLALIVEcmd( ENABLE );
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBHS, ENABLE );
#endif

  USBInitForCdc();
}

void resetCDCParameters() {
    USBByteCountEP2 = 0; // Bytes of received data on USB endpoint
    UpPoint2BusyFlag = 0;
}
  
void setLineCodingHandler() {
  uint8_t receiveLen;
#if defined (CH573) || defined (CH572)
  receiveLen = R8_USB_RX_LEN;
#elif defined (CH585)
  receiveLen = R16_U2EP0_RX_LEN;
#elif defined (CH32X035)
  receiveLen = USBFSD->RX_LEN;
#elif defined (CH32V30x)
  receiveLen = USBHSD->RX_LEN;
#endif
    for (uint8_t i = 0;
         i < ((LINE_CODEING_SIZE <= receiveLen) ? LINE_CODEING_SIZE : receiveLen);
         i++) {
      LineCoding[i] = Ep0Buffer[i];
    }
}

uint16_t getLineCodingHandler() {
  uint16_t returnLen;

  returnLen = LINE_CODEING_SIZE;
  for (uint8_t i = 0; i < returnLen; i++) {
    Ep0Buffer[i] = LineCoding[i];
  }

  return returnLen;
}
  
void setControlLineStateHandler() {
  controlLineState = Ep0Buffer[2];

  uint32_t baud = *((uint32_t *)LineCoding); // both linecoding and riscv gcc are little-endian

  // We check DTR state to determine if host port is open (bit 0 of lineState).
  if (((controlLineState & 0x01) == 0) &&
      (baud == 1200)) { 
          preAppJumpBoot();
          APPJumpBoot();
  }
}



uint8_t USBSerial_wait_UpPoint2BusyFlag_clear() {
  uint16_t waitWriteCount = 0;
  while (UpPoint2BusyFlag) { // wait for 250ms or give up, on my mac it takes
                              // about 256us
    waitWriteCount++;
    //delayMicroseconds(5);
    //TODO: the following is a busy wait for CH573, need more test for CH32X035
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    if (waitWriteCount >= 50000)
      return 0;
  }
  return 1;
}
  
bool USBSerial() {
  bool result = false;
  if (controlLineState > 0)
    result = true;
  // delay(10); not doing it for now
  return result;
}
  
void USBSerial_flush(void) {
  if (!UpPoint2BusyFlag && usbWritePointer > 0) {
      uint16_t usbIntCopy;
#if defined(CH573) || defined(CH572)
      usbIntCopy = R8_USB_INT_EN;
      R8_USB_INT_EN &= ~RB_UIE_TRANSFER;
      R8_UEP2_T_LEN = usbWritePointer;
      UpPoint2BusyFlag = 1;
      R8_UEP2_CTRL = R8_UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // Respond ACK
      R8_USB_INT_EN = usbIntCopy;
#elif defined(CH585)
      usbIntCopy = R8_USB2_INT_EN;
      R8_USB2_INT_EN &= ~USBHS_UDIE_TRANSFER;
      R16_U2EP2_T_LEN = usbWritePointer;
      UpPoint2BusyFlag = 1;
      R8_U2EP2_TX_CTRL = (R8_U2EP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
      R8_USB2_INT_EN = usbIntCopy;
#elif defined(CH32X035)
      usbIntCopy = USBFSD->INT_EN;
      USBFSD->INT_EN &= ~USBFS_UIE_TRANSFER;
      USBFSD->UEP2_TX_LEN = usbWritePointer;
      UpPoint2BusyFlag = 1;
      USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~USBFS_UEP_T_RES_MASK | USBFS_UEP_T_RES_ACK; // Respond ACK
      USBFSD->INT_EN = usbIntCopy;
#elif defined (CH32V30x)
      usbIntCopy = USBHSD->INT_EN;
      USBHSD->INT_EN &= ~USBHS_UIE_TRANSFER;
      USBHSD->UEP2_TX_LEN = usbWritePointer;
      UpPoint2BusyFlag = 1;
      USBHSD->UEP2_TX_CTRL = USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK | USBHS_UEP_T_RES_ACK; // Respond ACK
      USBHSD->INT_EN = usbIntCopy;
#endif

    if (usbWritePointer ==
        EP2_MAX_PACKET_SIZE) { // write empty packet for end transmission. Needed
                            // for windows.
      if (USBSerial_wait_UpPoint2BusyFlag_clear()) {
#if defined(CH573) || defined(CH572)
        usbIntCopy = R8_USB_INT_EN;
        R8_USB_INT_EN &= ~RB_UIE_TRANSFER;
        R8_UEP2_T_LEN = 0;
        UpPoint2BusyFlag = 1;
        R8_UEP2_CTRL = R8_UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // Respond ACK
        R8_USB_INT_EN = usbIntCopy;
#elif defined(CH585)
        usbIntCopy = R8_USB2_INT_EN;
        R8_USB2_INT_EN &= ~USBHS_UDIE_TRANSFER;
        R16_U2EP2_T_LEN = 0;
        UpPoint2BusyFlag = 1;
        R8_U2EP2_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
        R8_U2EP2_TX_CTRL = (R8_U2EP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
        R8_U2EP2_TX_CTRL &= ~USBHS_UEP_T_DONE;
        R8_USB2_INT_EN = usbIntCopy;
#elif defined(CH32X035)
        usbIntCopy = USBFSD->INT_EN;
        USBFSD->INT_EN &= ~USBFS_UIE_TRANSFER;
        USBFSD->UEP2_TX_LEN = 0;
        UpPoint2BusyFlag = 1;
        USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~USBFS_UEP_T_RES_MASK | USBFS_UEP_T_RES_ACK; // Respond ACK
        USBFSD->INT_EN = usbIntCopy;
#elif defined (CH32V30x)
        usbIntCopy = USBHSD->INT_EN;
        USBHSD->INT_EN &= ~USBHS_UIE_TRANSFER;
        USBHSD->UEP2_TX_LEN = 0;
        UpPoint2BusyFlag = 1;
        USBHSD->UEP2_TX_CTRL = USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK | USBHS_UEP_T_RES_ACK; // Respond ACK
        USBHSD->INT_EN = usbIntCopy;
#endif
      }
    }
    usbWritePointer = 0;
  }
}
  
uint8_t USBSerial_write( char c) { // 3 bytes generic pointer
  while (true) {
    if (USBSerial_wait_UpPoint2BusyFlag_clear() == 0)
      return 0;
    if (usbWritePointer < EP2_MAX_PACKET_SIZE) {
      Ep2Buffer[EP2_MAX_PACKET_SIZE + usbWritePointer] = c;
      usbWritePointer++;
      return 1;
    } else {
      USBSerial_flush(); // go back to first while
    }
  }
  return 0;
}

uint8_t USBSerial_print_n(uint8_t *buf,int len) { // 3 bytes generic pointer, not using
                                      // USBSerial_write for a bit efficiency
  if (controlLineState > 0) {
    while (len > 0) {
      if (USBSerial_wait_UpPoint2BusyFlag_clear() == 0)
        return 0;
      while (len > 0) {
        if (usbWritePointer < EP2_MAX_PACKET_SIZE) {
          Ep2Buffer[EP2_MAX_PACKET_SIZE + usbWritePointer] = *buf++;
          usbWritePointer++;
          len--;
        } else {
          USBSerial_flush(); // go back to first while
          break;
        }
      }
    }
  }
  return 0;
}

#if defined(CH585) || defined(CH32V30x)
uint16_t USBSerial_available() { return USBByteCountEP2; }
#else
uint8_t USBSerial_available() { return USBByteCountEP2; }
#endif
  
char USBSerial_read() {
  if (USBByteCountEP2 == 0)
    return 0;
  char data = Ep2Buffer[USBBufOutPointEP2];
  USBBufOutPointEP2++;
  USBByteCountEP2--;
  if (USBByteCountEP2 == 0) {
#if defined(CH573) || defined(CH572)
    R8_UEP2_CTRL = R8_UEP2_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_ACK;
#elif defined(CH585)
    R8_U2EP2_RX_CTRL = (R8_U2EP2_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
    R8_U2EP2_RX_CTRL &= ~USBHS_UEP_R_DONE;
#elif defined(CH32X035)
    USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~USBFS_UEP_R_RES_MASK | USBFS_UEP_R_RES_ACK;
#elif defined (CH32V30x)
    USBHSD->UEP2_RX_CTRL = (USBHSD->UEP2_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
#endif
  }
  return data;
}

bool USBSerial_bool() {
  bool result = false;
  if (controlLineState > 0)
    result = true;
  // delay(10); not doing it for now
  return result;
}
  

void USB_EP2_IN() {
#if defined(CH573) || defined(CH572)
  R8_UEP2_T_LEN = 0; // No data to send anymore
  R8_UEP2_CTRL = R8_UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // Respond NAK by default
#elif defined(CH585)
  R16_U2EP2_T_LEN = 0;
  R8_U2EP2_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
  R8_U2EP2_TX_CTRL = (R8_U2EP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
  R8_U2EP2_TX_CTRL &= ~USBHS_UEP_T_DONE;
#elif defined(CH32X035)
  USBFSD->UEP2_TX_LEN = 0; // No data to send anymore
  USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~USBFS_UEP_T_RES_MASK | USBFS_UEP_T_RES_NAK; // Respond NAK by default
#elif defined (CH32V30x)
  USBHSD->UEP2_TX_LEN = 0;
  USBHSD->UEP2_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
  USBHSD->UEP2_TX_CTRL = USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK | USBHS_UEP_T_RES_NAK;
#endif
  UpPoint2BusyFlag = 0;                            // Clear busy flag
}
  
void USB_EP2_OUT() {
#if defined(CH573) || defined(CH572)
  if (R8_USB_INT_FG & RB_U_TOG_OK){ // Discard unsynchronized packets
    USBByteCountEP2 = R8_USB_RX_LEN;
    USBBufOutPointEP2 = 0; // Reset Data pointer for fetching
    if (USBByteCountEP2)
    R8_UEP2_CTRL = R8_UEP2_CTRL & ~MASK_UEP_R_RES |
                  UEP_R_RES_NAK; // Respond NAK after a packet. Let main code
                                 // change response after handling.
  }
#elif defined(CH585)
  { // CH585 does not have RB_U_TOG_OK
    USBByteCountEP2 = R16_U2EP2_RX_LEN;
    USBBufOutPointEP2 = 0; // Reset Data pointer for fetching
    if (USBByteCountEP2){
      R8_U2EP2_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
      R8_U2EP2_RX_CTRL = (R8_U2EP2_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
      R8_U2EP2_RX_CTRL &= ~USBHS_UEP_R_DONE;
    }
  }
#elif defined(CH32X035)
  if (USBFSD->INT_FG & USBFS_U_TOG_OK){ // Discard unsynchronized packets
    USBByteCountEP2 = USBFSD->RX_LEN;
    USBBufOutPointEP2 = 0; // Reset Data pointer for fetching
    if (USBByteCountEP2)
    USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~USBFS_UEP_R_RES_MASK |
                  USBFS_UEP_R_RES_NAK; // Respond NAK after a packet. Let main code
                                 // change response after handling.
  }
#elif defined (CH32V30x)
  if (USBHSD->INT_ST & USBHS_UIS_TOG_OK){ // Discard unsynchronized packets
    USBByteCountEP2 = USBHSD->RX_LEN;
    USBBufOutPointEP2 = 0; // Reset Data pointer for fetching
    if (USBByteCountEP2){
      USBHSD->UEP2_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
      USBHSD->UEP2_RX_CTRL = (USBHSD->UEP2_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
    }
  }
#endif
}
