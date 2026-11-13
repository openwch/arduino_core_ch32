/********************************** (C) COPYRIGHT *******************************
* File Name          : system_ch32v4x7.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/12/01
* Description        : CH32V4x7 Device Peripheral Access Layer System Header File.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef __CH32V4x7_USB_H
#define __CH32V4x7_USB_H

#ifdef __cplusplus
 extern "C" {
#endif

/*******************************************************************************/
/* Header File */
#include "stdint.h"

/*******************************************************************************/
/* USB Communication Related Macro Definition */
/* USB Endpoint0 Size */
#ifndef DEFAULT_ENDP0_SIZE
#define DEFAULT_ENDP0_SIZE          8          // default maximum packet size for endpoint 0
#endif
#ifndef MAX_PACKET_SIZE
#define MAX_PACKET_SIZE             64         // maximum packet size
#endif

/* USB PID */
#ifndef USB_PID_SETUP
#define USB_PID_NULL                0x00
#define USB_PID_SOF                 0x05
#define USB_PID_SETUP               0x0D
#define USB_PID_IN                  0x09
#define USB_PID_OUT                 0x01
#define USB_PID_NYET                0x06
#define USB_PID_ACK                 0x02
#define USB_PID_NAK                 0x0A
#define USB_PID_STALL               0x0E
#define USB_PID_DATA0               0x03
#define USB_PID_DATA1               0x0B
#define USB_PID_DATA2               0x07
#define USB_PID_MDATA               0x0F
#define USB_PID_PRE                 0x0C
#endif

/* USB standard device request code */
#ifndef USB_GET_DESCRIPTOR
#define USB_GET_STATUS              0x00
#define USB_CLEAR_FEATURE           0x01
#define USB_SET_FEATURE             0x03
#define USB_SET_ADDRESS             0x05
#define USB_GET_DESCRIPTOR          0x06
#define USB_SET_DESCRIPTOR          0x07
#define USB_GET_CONFIGURATION       0x08
#define USB_SET_CONFIGURATION       0x09
#define USB_GET_INTERFACE           0x0A
#define USB_SET_INTERFACE           0x0B
#define USB_SYNCH_FRAME             0x0C
#endif

#define DEF_STRING_DESC_LANG        0x00
#define DEF_STRING_DESC_MANU        0x01
#define DEF_STRING_DESC_PROD        0x02
#define DEF_STRING_DESC_SERN        0x03

/* USB hub class request code */
#ifndef HUB_GET_DESCRIPTOR
#define HUB_GET_STATUS              0x00
#define HUB_CLEAR_FEATURE           0x01
#define HUB_GET_STATE               0x02
#define HUB_SET_FEATURE             0x03
#define HUB_GET_DESCRIPTOR          0x06
#define HUB_SET_DESCRIPTOR          0x07
#endif

/* USB HID class request code */
#ifndef HID_GET_REPORT
#define HID_GET_REPORT              0x01
#define HID_GET_IDLE                0x02
#define HID_GET_PROTOCOL            0x03
#define HID_SET_REPORT              0x09
#define HID_SET_IDLE                0x0A
#define HID_SET_PROTOCOL            0x0B
#endif

/* USB CDC Class request code */
#ifndef CDC_GET_LINE_CODING
#define CDC_GET_LINE_CODING         0x21                                      /* This request allows the host to find out the currently configured line coding */
#define CDC_SET_LINE_CODING         0x20                                      /* Configures DTE rate, stop-bits, parity, and number-of-character */
#define CDC_SET_LINE_CTLSTE         0x22                                      /* This request generates RS-232/V.24 style control signals */
#define CDC_SEND_BREAK              0x23                                      /* Sends special carrier modulation used to specify RS-232 style break */
#endif

/* Bit Define for USB Request Type */
#ifndef USB_REQ_TYP_MASK
#define USB_REQ_TYP_IN              0x80
#define USB_REQ_TYP_OUT             0x00
#define USB_REQ_TYP_READ            0x80
#define USB_REQ_TYP_WRITE           0x00
#define USB_REQ_TYP_MASK            0x60
#define USB_REQ_TYP_STANDARD        0x00
#define USB_REQ_TYP_CLASS           0x20
#define USB_REQ_TYP_VENDOR          0x40
#define USB_REQ_TYP_RESERVED        0x60
#define USB_REQ_RECIP_MASK          0x1F
#define USB_REQ_RECIP_DEVICE        0x00
#define USB_REQ_RECIP_INTERF        0x01
#define USB_REQ_RECIP_ENDP          0x02
#define USB_REQ_RECIP_OTHER         0x03
#define USB_REQ_FEAT_REMOTE_WAKEUP  0x01
#define USB_REQ_FEAT_ENDP_HALT      0x00
#endif

/* USB Descriptor Type */
#ifndef USB_DESCR_TYP_DEVICE
#define USB_DESCR_TYP_DEVICE        0x01
#define USB_DESCR_TYP_CONFIG        0x02
#define USB_DESCR_TYP_STRING        0x03
#define USB_DESCR_TYP_INTERF        0x04
#define USB_DESCR_TYP_ENDP          0x05
#define USB_DESCR_TYP_QUALIF        0x06
#define USB_DESCR_TYP_SPEED         0x07
#define USB_DESCR_TYP_OTG           0x09
#define USB_DESCR_TYP_BOS           0X0F
#define USB_DESCR_TYP_HID           0x21
#define USB_DESCR_TYP_REPORT        0x22
#define USB_DESCR_TYP_PHYSIC        0x23
#define USB_DESCR_TYP_CS_INTF       0x24
#define USB_DESCR_TYP_CS_ENDP       0x25
#define USB_DESCR_TYP_HUB           0x29
#endif

/* USB Device Class */
#ifndef USB_DEV_CLASS_HUB
#define USB_DEV_CLASS_RESERVED      0x00
#define USB_DEV_CLASS_AUDIO         0x01
#define USB_DEV_CLASS_COMMUNIC      0x02
#define USB_DEV_CLASS_HID           0x03
#define USB_DEV_CLASS_MONITOR       0x04
#define USB_DEV_CLASS_PHYSIC_IF     0x05
#define USB_DEV_CLASS_POWER         0x06
#define USB_DEV_CLASS_IMAGE         0x06
#define USB_DEV_CLASS_PRINTER       0x07
#define USB_DEV_CLASS_STORAGE       0x08
#define USB_DEV_CLASS_HUB           0x09
#define USB_DEV_CLASS_VEN_SPEC      0xFF
#endif

/* USB Hub Class Request */
#ifndef HUB_GET_HUB_DESCRIPTOR
#define HUB_CLEAR_HUB_FEATURE       0x20
#define HUB_CLEAR_PORT_FEATURE      0x23
#define HUB_GET_BUS_STATE           0xA3
#define HUB_GET_HUB_DESCRIPTOR      0xA0
#define HUB_GET_HUB_STATUS          0xA0
#define HUB_GET_PORT_STATUS         0xA3
#define HUB_SET_HUB_DESCRIPTOR      0x20
#define HUB_SET_HUB_FEATURE         0x20
#define HUB_SET_PORT_FEATURE        0x23
#endif

/* Hub Class Feature Selectors */
#ifndef HUB_PORT_RESET
#define HUB_C_HUB_LOCAL_POWER       0
#define HUB_C_HUB_OVER_CURRENT      1
#define HUB_PORT_CONNECTION         0
#define HUB_PORT_ENABLE             1
#define HUB_PORT_SUSPEND            2
#define HUB_PORT_OVER_CURRENT       3
#define HUB_PORT_RESET              4
#define HUB_PORT_POWER              8
#define HUB_PORT_LOW_SPEED          9
#define HUB_C_PORT_CONNECTION       16
#define HUB_C_PORT_ENABLE           17
#define HUB_C_PORT_SUSPEND          18
#define HUB_C_PORT_OVER_CURRENT     19
#define HUB_C_PORT_RESET            20
#endif


/* USB UDisk */
#ifndef USB_BO_CBW_SIZE
#define USB_BO_CBW_SIZE             0x1F
#define USB_BO_CSW_SIZE             0x0D
#endif
#ifndef USB_BO_CBW_SIG0
#define USB_BO_CBW_SIG0             0x55
#define USB_BO_CBW_SIG1             0x53
#define USB_BO_CBW_SIG2             0x42
#define USB_BO_CBW_SIG3             0x43
#define USB_BO_CSW_SIG0             0x55
#define USB_BO_CSW_SIG1             0x53
#define USB_BO_CSW_SIG2             0x42
#define USB_BO_CSW_SIG3             0x53
#endif

/*******************************************************************************/
/* USBHS Related Register Macro Definition */

/* R8_USB_CTRL */
#define  USBHS_UD_LPM_EN            0x80
#define  USBHS_UD_DEV_EN            0x20
#define  USBHS_UD_DMA_EN            0x10
#define  USBHS_UD_PHY_SUSPENDM      0x08
#define  USBHS_UD_CLR_ALL           0x04
#define  USBHS_UD_RST_SIE           0x02
#define  USBHS_UD_RST_LINK          0x01

/* R8_USB_BASE_MODE */
#define  USBHS_UD_SPEED_FULL        0x00
#define  USBHS_UD_SPEED_HIGH        0x01
#define  USBHS_UD_SPEED_LOW         0x02
#define  USBHS_UD_SPEED_TYPE        0x03       

/* R8_USB_INT_EN */
#define  USBHS_UDIE_FIFO_OVER       0x80
#define  USBHS_UDIE_LINK_RDY        0x40
#define  USBHS_UDIE_SOF_ACT         0x20
#define  USBHS_UDIE_TRANSFER        0x10
#define  USBHS_UDIE_LPM_ACT         0x08
#define  USBHS_UDIE_BUS_SLEEP       0x04
#define  USBHS_UDIE_SUSPEND         0x02
#define  USBHS_UDIE_BUS_RST         0x01

/* R8_USB_DEV_AD */
#define  USBHS_UD_DEV_ADDR          0x7F

/* R8_USB_WAKE_CTRL */
#define  USBHS_UD_REMOTE_WKUP       0x01

/* R8_USB_TEST_MODE */
#define  USBHS_UD_TEST_EN           0x80
#define  USBHS_UD_TEST_SE0NAK       0x08
#define  USBHS_UD_TEST_PKT          0x04
#define  USBHS_UD_TEST_K            0x02
#define  USBHS_UD_TEST_J            0x01

/* R16_USB_LPM_DATA */
#define  USBHS_UD_LPM_BUSY          0x8000
#define  USBHS_UD_LPM_DATA          0x07FF

/* R8_USB_INT_FG */
#define  USBHS_UDIF_FIFO_OV         0x80
#define  USBHS_UDIF_LINK_RDY        0x40
#define  USBHS_UDIF_RX_SOF          0x20
#define  USBHS_UDIF_TRANSFER        0x10
#define  USBHS_UDIF_RTX_ACT         0x10
#define  USBHS_UDIF_LPM_ACT         0x08
#define  USBHS_UDIF_BUS_SLEEP       0x04
#define  USBHS_UDIF_SUSPEND         0x02
#define  USBHS_UDIF_BUS_RST         0x01

/* R8_USB_INT_ST */
#define  USBHS_UDIS_EP_DIR          0x10
#define  USBHS_UDIS_EP_ID_MASK      0x07

/* R8_USB_MIS_ST */
#define  USBHS_UDMS_HS_MOD          0x80
#define  USBHS_UDMS_SUSP_REQ        0x10
#define  USBHS_UDMS_SIE_FREE        0x08
#define  USBHS_UDMS_SLEEP           0x04
#define  USBHS_UDMS_SUSPEND         0x02
#define  USBHS_UDMS_READY           0x01

/* R16_USB_FRAME_NO */
#define  USBHS_UD_MFRAME_NO         0xE000
#define  USBHS_UD_FRAME_NO          0x07FF

/* R16_USB_BUS */
#define  USBHS_USB_DM_ST            0x08
#define  USBHS_USB_DP_ST            0x04
#define  USB_WAKEUP                 0x01

/* R16_UEP_TX_EN */
#define USBHS_UEP0_T_EN             0x0001
#define USBHS_UEP1_T_EN             0x0002
#define USBHS_UEP2_T_EN             0x0004
#define USBHS_UEP3_T_EN             0x0008
#define USBHS_UEP4_T_EN             0x0010
#define USBHS_UEP5_T_EN             0x0020
#define USBHS_UEP6_T_EN             0x0040
#define USBHS_UEP7_T_EN             0x0080
#define USBHS_UEP8_T_EN             0x0100
#define USBHS_UEP9_T_EN             0x0200
#define USBHS_UEP10_T_EN            0x0400
#define USBHS_UEP11_T_EN            0x0800
#define USBHS_UEP12_T_EN            0x1000
#define USBHS_UEP13_T_EN            0x2000
#define USBHS_UEP14_T_EN            0x4000
#define USBHS_UEP15_T_EN            0x8000

/* R16_UEP_RX_EN */
#define USBHS_UEP0_R_EN             0x0001
#define USBHS_UEP1_R_EN             0x0002
#define USBHS_UEP2_R_EN             0x0004
#define USBHS_UEP3_R_EN             0x0008
#define USBHS_UEP4_R_EN             0x0010
#define USBHS_UEP5_R_EN             0x0020
#define USBHS_UEP6_R_EN             0x0040
#define USBHS_UEP7_R_EN             0x0080
#define USBHS_UEP8_R_EN             0x0100
#define USBHS_UEP9_R_EN             0x0200
#define USBHS_UEP10_R_EN            0x0400
#define USBHS_UEP11_R_EN            0x0800
#define USBHS_UEP12_R_EN            0x1000
#define USBHS_UEP13_R_EN            0x2000
#define USBHS_UEP14_R_EN            0x4000
#define USBHS_UEP15_R_EN            0x8000

/* R16_UEP_T_TOG_AUTO */
#define USBHS_UEP_T_TOG_AUTO        0xFF

/* R16_UEP_R_TOG_AUTO */
#define USBHS_UEP_R_TOG_AUTO        0xFF

/* R8_UEP_T_BURST */
#define USBHS_UEP_T_BURST_EN        0xFF

/* R8_UEP_T_BURST_MODE */
#define USBHS_UEP_T_BURST_MODE      0xFF

/* R8_UEP_R_BURST */
#define USBHS_UEP_R_BURST_EN        0xFF

/* R8_UEP_R_RES_MODE */
#define USBHS_UEP_R_RES_MODE        0xFF

/* R32_UEP_AF_MODE */
#define USBHS_UEP_T_AF              0xFE

/* R32_UEP0_DMA */
#define UEPn_DMA                    0xFFFFFF

/* R32_UEPn_RX_DMA */
#define UEPn_RX_DMA                 0xFFFFFF

/* R32_UEPn_TX_DMA */
#define UEPn_TX_DMA                 0xFFFFFF

/* R32_UEPn_MAX_LEN */
#define USBHS_UEP0_MAX_LEN          0x007F
#define USBHS_UEPn_MAX_LEN          0x07FF

/* R16_UEPn_RX_LEN */
#define USBHS_UEP0_RX_LEN           0x007F

/* R16_UEPn_RX_LEN */
#define USBHS_UEPn_RX_LEN           0xFFFF

/* R16_UEPn_R_SIZE */
#define USBHS_UEPn_R_SIZE           0xFFFF

/* R16_UEP0_T_LEN */
#define USBHS_UEP0_T_LEN            0x7F

/**R16_UEPn_T_LEN**/ 
#define USBHS_UEPn_T_LEN            0xFFFF

/**R8_UEPn_TX_CTRL**/ 
#define USBHS_UEP_T_DONE            0x80
#define USBHS_UEP_T_NAK_ACT         0x40
#define USBHS_UEP_T_TOG_MASK        0x0C
#define USBHS_UEP_T_TOG_MDATA       0x0C
#define USBHS_UEP_T_TOG_DATA2       0x08
#define USBHS_UEP_T_TOG_DATA1       0x04
#define USBHS_UEP_T_TOG_DATA0       0x00
#define USBHS_UEP_T_RES_MASK        0x03
#define USBHS_UEP_T_RES_ACK         0x02
#define USBHS_UEP_T_RES_STALL       0x01
#define USBHS_UEP_T_RES_NAK         0x00

/**R8_UEP0_RX_CTRL**/ 

/**R8_UEPn_RX_CTRL**/ 
#define USBHS_UEP_R_DONE            0x80
#define USBHS_UEP_R_NAK_ACT         0x40
#define USBHS_UEP_R_NAK_TOG         0x20
#define USBHS_UEP_R_TOG_MATCH       0x10
#define USBHS_UEP_R_SETUP_IS        0x08
#define USBHS_UEP_R_TOG_MASK        0x0C
#define USBHS_UEP_R_TOG_MDATA       0x0C
#define USBHS_UEP_R_TOG_DATA2       0x08
#define USBHS_UEP_R_TOG_DATA1       0x04
#define USBHS_UEP_R_TOG_DATA0       0x00
#define USBHS_UEP_R_RES_MASK        0x03
#define USBHS_UEP_R_RES_ACK         0x02
#define USBHS_UEP_R_RES_STALL       0x01
#define USBHS_UEP_R_RES_NAK         0x00
 
/* R16_UEP_T_ISO */
#define  USBHS_UEP1_T_FIFO_EN       0x0200
#define  USBHS_UEP2_T_FIFO_EN       0x0400
#define  USBHS_UEP3_T_FIFO_EN       0x0800
#define  USBHS_UEP4_T_FIFO_EN       0x1000
#define  USBHS_UEP5_T_FIFO_EN       0x2000
#define  USBHS_UEP6_T_FIFO_EN       0x4000
#define  USBHS_UEP7_T_FIFO_EN       0x8000
#define  USBHS_UEP1_T_ISO_EN        0x0002
#define  USBHS_UEP2_T_ISO_EN        0x0004
#define  USBHS_UEP3_T_ISO_EN        0x0008
#define  USBHS_UEP4_T_ISO_EN        0x0010
#define  USBHS_UEP5_T_ISO_EN        0x0020
#define  USBHS_UEP6_T_ISO_EN        0x0040
#define  USBHS_UEP7_T_ISO_EN        0x0080

/* R16_UEP_R_ISO */
#define  USBHS_UEP1_R_FIFO_EN       0x0200
#define  USBHS_UEP2_R_FIFO_EN       0x0400
#define  USBHS_UEP3_R_FIFO_EN       0x0800
#define  USBHS_UEP4_R_FIFO_EN       0x1000
#define  USBHS_UEP5_R_FIFO_EN       0x2000
#define  USBHS_UEP6_R_FIFO_EN       0x4000
#define  USBHS_UEP7_R_FIFO_EN       0x8000
#define  USBHS_UEP1_R_ISO_EN        0x0002
#define  USBHS_UEP2_R_ISO_EN        0x0004
#define  USBHS_UEP3_R_ISO_EN        0x0008
#define  USBHS_UEP4_R_ISO_EN        0x0010
#define  USBHS_UEP5_R_ISO_EN        0x0020
#define  USBHS_UEP6_R_ISO_EN        0x0040
#define  USBHS_UEP7_R_ISO_EN        0x0080
 
/* R32_UEPn_RX_FIFO */
#define  USBHS_UEP_RX_FIFO_E        0xFF00
#define  USBHS_UEP_RX_FIFO_S        0x00FF

/* R32_UEPn_TX_FIFO */
#define  USBHS_UEP_TX_FIFO_E        0xFF00
#define  USBHS_UEP_TX_FIFO_S        0x00FF


/* USB high speed host register  */
/* R8_UH_CFG */
#define  USBHS_UH_LPM_EN            0x80
#define  USBHS_UH_FORCE_FS          0x40
#define  USBHS_UH_SOF_EN            0x20
#define  USBHS_UH_DMA_EN            0x10
#define  USBHS_UH_PHY_SUSPENDM      0x08
#define  USBHS_UH_CLR_ALL           0x04
#define  USBHS_RST_SIE              0x02
#define  USBHS_RST_LINK             0x01

/* R8_UH_INT_EN */
#define  USBHS_UHIE_FIFO_OVER       0x80
#define  USBHS_UHIE_TX_HALT         0x40
#define  USBHS_UHIE_SOF_ACT         0x20
#define  USBHS_UHIE_TRANSFER        0x10
#define  USBHS_UHIE_RESUME_ACT      0x08
#define  USBHS_UHIE_WKUP_ACT        0x04

/* R8_UH_DEV_AD */
#define  USBHS_UH_DEV_ADDR          0x7F

/* R32_UH_CONTROL */
#define  USBHS_UH_RX_NO_RES         0x800000
#define  USBHS_UH_TX_NO_RES         0x400000
#define  USBHS_UH_RX_NO_DATA        0x200000
#define  USBHS_UH_TX_NO_DATA        0x100000
#define  USBHS_UH_PRE_PID_EN        0x080000
#define  USBHS_UH_SPLIT_VALID       0x040000
#define  USBHS_UH_LPM_VALID         0x020000
#define  USBHS_UH_HOST_ACTION       0x010000
#define  USBHS_UH_BUF_MODE          0x0400
#define  USBHS_UH_T_TOG_MASK        0x0300
#define  USBHS_UH_T_TOG_MDATA       0x0300
#define  USBHS_UH_T_TOG_DATA2       0x0200
#define  USBHS_UH_T_TOG_DATA1       0x0100
#define  USBHS_UH_T_TOG_DATA0       0x0000
#define  USBHS_UH_T_ENDP_MASK       0xF0
#define  USBHS_UH_T_TOKEN_MASK      0x0F

/* R8_UH_INT_FLAG */
#define  USBHS_UHIF_FIFO_OVER       0x80
#define  USBHS_UHIF_TX_HALT         0x40
#define  USBHS_UHIF_SOF_ACT         0x20
#define  USBHS_UHIF_TRANSFER        0x10
#define  USBHS_UHIF_RESUME_ACT      0x08
#define  USBHS_UHIF_WKUP_ACT        0x04

/* R8_UH_INT_ST */
#define  USBHS_UHIF_PORT_RX_RESUME  0x10
#define  USBHS_UH_R_TOKEN_MASK      0x0F

/* R8_UH_MIS_ST */
#define  USBHS_UHMS_BUS_SE0         0x80
#define  USBHS_UHMS_BUS_J           0x40
#define  USBHS_UHMS_LINESTATE       0x30
#define  USBHS_UHMS_USB_WAKEUP      0x08
#define  USBHS_UHMS_SOF_ACT         0x04
#define  USBHS_UHMS_SOF_PRE         0x02
#define  USBHS_UHMS_SOF_FREE        0x01

/* R32_UH_LPM_DATA */
#define  USBHS_UH_LPM_DATA          0x07FF

/* R32_UH_SPLIT_DATA */
#define  USBHS_UH_SPLIT_DATA        0x07FFFF

/* R32_UH_FRAME */
#define  USBHS_UH_SOF_CNT_CLR       0x02000000
#define  USBHS_UH_SOF_CNT_EN        0x01000000
#define  USBHS_UH_MFRAME_NO         0x070000
#define  USBHS_UH_FRAME_NO          0x07FF

/* R32_UH_TX_LEN */
#define  USBHS_UH_TX_LEN            0x07FF

/* R32_UH_RX_LEN */
#define  USBHS_UH_RX_LEN            0x07FF

/* R32_UH_RX_MAX_LEN */
#define  USBHS_UH_RX_MAX_LEN        0x07FF

/* R32_UH_RX_DMA */
#define  USBHS_R32_UH_RX_DMA        0x01FFFF

/* R32_UH_TX_DMA */
#define  USBHS_R32_UH_TX_DMA        0x01FFFF

/* R32_UH_PORT_CTRL */
#define  USBHS_UH_BUS_RST_LONG      0x010000
#define  USBHS_UH_PORT_SLEEP_BESL   0xF000
#define  USBHS_UH_CLR_PORT_SLEEP    0x0100
#define  USBHS_UH_CLR_PORT_CONNECT  0x20
#define  USBHS_UH_CLR_PORT_EN       0x10
#define  USBHS_UH_SET_PORT_SLEEP    0x08
#define  USBHS_UH_CLR_PORT_SUSP     0x04
#define  USBHS_UH_SET_PORT_SUSP     0x02
#define  USBHS_UH_SET_PORT_RESET    0x01

/* R8_UH_PORT_CFG */
#define  USBHS_UH_PD_EN             0x80
#define  USBHS_UH_HOST_EN           0x01

/* R8_UH_PORT_INT_EN */
#define  USBHS_UHIE_PORT_SLP        0x20
#define  USBHS_UHIE_PORT_RESET      0x10
#define  USBHS_UHIE_PORT_SUSP       0x04
#define  USBHS_UHIE_PORT_EN         0x02
#define  USBHS_UHIE_PORT_CONNECT    0x01

/* R8_UH_PORT_TEST_CT */
#define  USBHS_UH_TEST_SE0_NAK      0x10
#define  USBHS_UH_TEST_PACKET       0x08
#define  USBHS_UH_TEST_FORCE_EN     0x04
#define  USBHS_UH_TEST_K            0x02
#define  USBHS_UH_TEST_J            0x01

/* R16_UH_PORT_ST */
#define  USBHS_UHIS_PORT_TEST       0x0800
#define  USBHS_UHIS_PORT_SPEED_MASK 0x0600
#define  USBHS_UHIS_PORT_HS         0x0400
#define  USBHS_UHIS_PORT_LS         0x0200
#define  USBHS_UHIS_PORT_FS         0x0000
#define  USBHS_UHIS_PORT_SLP        0x20
#define  USBHS_UHIS_PORT_RST        0x10
#define  USBHS_UHIS_PORT_SUSP       0x04
#define  USBHS_UHIS_PORT_EN         0x02
#define  USBHS_UHIS_PORT_CONNECT    0x01

/* R8_UH_PORT_CHG */
#define  USBHS_UHIF_PORT_SLP        0x20
#define  USBHS_UHIF_PORT_RESET      0x10
#define  USBHS_UHIF_PORT_SUSP       0x04
#define  USBHS_UHIF_PORT_EN         0x02
#define  USBHS_UHIF_PORT_CONNECT    0x01

/* R32_UH_BC_CTRL */
#define  UDM_VSRC_ACT               0x0400
#define  UDM_BC_VSRC                0x0200
#define  UDP_BC_VSRC                0x0100
#define  BC_AUTO_MODE               0x40
#define  UDM_BC_CMPE                0x20
#define  UDP_BC_CMPE                0x10
#define  UDM_BC_CMPO                0x02
#define  UDP_BC_CMPO                0x01

/*******************************************************************************/
/* Struct Definition */

/* USB Setup Request */
typedef struct __attribute__((packed)) _USB_SETUP_REQ
{
    uint8_t  bRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} USB_SETUP_REQ, *PUSB_SETUP_REQ;

/* USB Device Descriptor */
typedef struct __attribute__((packed)) _USB_DEVICE_DESCR
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} USB_DEV_DESCR, *PUSB_DEV_DESCR;

/* USB Configuration Descriptor */
typedef struct __attribute__((packed)) _USB_CONFIG_DESCR
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  MaxPower;
} USB_CFG_DESCR, *PUSB_CFG_DESCR;

/* USB Interface Descriptor */
typedef struct __attribute__((packed)) _USB_INTERF_DESCR
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
} USB_ITF_DESCR, *PUSB_ITF_DESCR;

/* USB Endpoint Descriptor */
typedef struct __attribute__((packed)) _USB_ENDPOINT_DESCR
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint8_t  wMaxPacketSizeL;
    uint8_t  wMaxPacketSizeH;
    uint8_t  bInterval;
} USB_ENDP_DESCR, *PUSB_ENDP_DESCR;

/* USB Configuration Descriptor Set */
typedef struct __attribute__((packed)) _USB_CONFIG_DESCR_LONG
{
    USB_CFG_DESCR  cfg_descr;
    USB_ITF_DESCR  itf_descr;
    USB_ENDP_DESCR endp_descr[ 1 ];
} USB_CFG_DESCR_LONG, *PUSB_CFG_DESCR_LONG;

/* USB HUB Descriptor */
typedef struct __attribute__((packed)) _USB_HUB_DESCR
{
    uint8_t  bDescLength;
    uint8_t  bDescriptorType;
    uint8_t  bNbrPorts;
    uint8_t  wHubCharacteristicsL;
    uint8_t  wHubCharacteristicsH;
    uint8_t  bPwrOn2PwrGood;
    uint8_t  bHubContrCurrent;
    uint8_t  DeviceRemovable;
    uint8_t  PortPwrCtrlMask;
} USB_HUB_DESCR, *PUSB_HUB_DESCR;

/* USB HID Descriptor */
typedef struct __attribute__((packed)) _USB_HID_DESCR
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdHID;
    uint8_t  bCountryCode;
    uint8_t  bNumDescriptors;
    uint8_t  bDescriptorTypeX;
    uint8_t  wDescriptorLengthL;
    uint8_t  wDescriptorLengthH;
} USB_HID_DESCR, *PUSB_HID_DESCR;

/* USB UDisk */
typedef struct __attribute__((packed)) _UDISK_BOC_CBW
{
    uint32_t mCBW_Sig;
    uint32_t mCBW_Tag;
    uint32_t mCBW_DataLen;
    uint8_t  mCBW_Flag;
    uint8_t  mCBW_LUN;
    uint8_t  mCBW_CB_Len;
    uint8_t  mCBW_CB_Buf[ 16 ];
} UDISK_BOC_CBW, *PXUDISK_BOC_CBW;

/* USB UDisk */
typedef struct __attribute__((packed)) _UDISK_BOC_CSW
{
    uint32_t mCBW_Sig;
    uint32_t mCBW_Tag;
    uint32_t mCSW_Residue;
    uint8_t  mCSW_Status;
} UDISK_BOC_CSW, *PXUDISK_BOC_CSW;

#ifdef __cplusplus
}
#endif

#endif /* __CH32V4x7_USB_H */
