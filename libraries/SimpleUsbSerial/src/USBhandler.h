#ifndef __USB_HANDLER_H__
#define __USB_HANDLER_H__

// clang-format off
#include <stdint.h>
#if defined (CH573)
#include "CH573SFR.h"
#include "core_riscv.h"
#elif defined (CH572)
#include "CH572SFR.h"
#include "core_riscv.h"
#elif defined (CH585)
#include "CH585SFR.h"
#include "core_riscv.h"
#define DEF_USBD_UEP0_SIZE           64     /* usb hs/fs device end-point 0 size */
#define DEF_USBD_UEP1_SIZE           512     /* usb hs/fs device end-point 1 size */
#define DEF_USBD_UEP2_SIZE           512     /* usb hs/fs device end-point 2 size */

#elif defined (CH32X035)
#include "ch32x035_usb_const.h"
#elif defined (CH32V30x)
#include "ch32v30x.h"
#include "ch32v30x_usb.h"
#define DEF_USBD_UEP0_SIZE           64     /* usb hs/fs device end-point 0 size */
#define DEF_USBD_UEP1_SIZE           512     /* usb hs/fs device end-point 1 size */
#define DEF_USBD_UEP2_SIZE           512     /* usb hs/fs device end-point 2 size */
#endif
#include "USBconstant.h"
// clang-format on

// // clang-format off
// extern __xdata __at (EP0_ADDR) uint8_t Ep0Buffer[];
// extern __xdata __at (EP1_ADDR) uint8_t Ep1Buffer[];
// extern __xdata __at (EP2_ADDR) uint8_t Ep2Buffer[];
// // clang-format on

// extern __data uint16_t SetupLen;
// extern __data uint8_t SetupReq;
// volatile extern __xdata uint8_t UsbConfig;
// extern const __code uint8_t *__data pDescr;

typedef struct _USB_SETUP_REQ_ {
    uint8_t bRequestType;
    uint8_t bRequest;
    uint8_t wValueL;
    uint8_t wValueH;
    uint8_t wIndexL;
    uint8_t wIndexH;
    uint8_t wLengthL;
    uint8_t wLengthH;
} USB_SETUP_REQ_t;

#define UsbSetupBuf ((USB_SETUP_REQ_t *)Ep0Buffer)

// Out
#define EP0_OUT_Callback USB_EP0_OUT
#define EP1_OUT_Callback NOP_Process
#define EP2_OUT_Callback USB_EP2_OUT
#define EP3_OUT_Callback NOP_Process
#define EP4_OUT_Callback NOP_Process

// SOF
#define EP0_SOF_Callback NOP_Process
#define EP1_SOF_Callback NOP_Process
#define EP2_SOF_Callback NOP_Process
#define EP3_SOF_Callback NOP_Process
#define EP4_SOF_Callback NOP_Process

// IN
#define EP0_IN_Callback USB_EP0_IN
#define EP1_IN_Callback USB_EP1_IN
#define EP2_IN_Callback USB_EP2_IN
#define EP3_IN_Callback NOP_Process
#define EP4_IN_Callback NOP_Process

// SETUP
#define EP0_SETUP_Callback USB_EP0_SETUP
#define EP1_SETUP_Callback NOP_Process
#define EP2_SETUP_Callback NOP_Process
#define EP3_SETUP_Callback NOP_Process
#define EP4_SETUP_Callback NOP_Process

// void USBInterrupt(void);
// void USBDeviceCfg();
// void USBDeviceIntCfg();
// void USBDeviceEndPointCfg();

void USBInitForCdc();

#endif