#ifndef __CONST_DATA_H__
#define __CONST_DATA_H__

#include <stdint.h>

#if defined (CH585) || defined (CH32V30x)
#define DEFAULT_ENDP0_SIZE 64
#define EP2_MAX_PACKET_SIZE 512
#else
#define DEFAULT_ENDP0_SIZE 8
#define EP2_MAX_PACKET_SIZE 64
#endif

#define  SET_LINE_CODING                0X20            // Configures DTE rate, stop-bits, parity, and number-of-character
#define  GET_LINE_CODING                0X21            // This request allows the host to find out the currently configured line coding.
#define  SET_CONTROL_LINE_STATE         0X22            // This request generates RS-232/V.24 style control signals.

#if defined (CH585) || defined (CH32V30x)
#define USBHS_SPEED_TYPE_MASK         ((uint8_t)(0x03))
#define USBHS_SPEED_LOW               ((uint8_t)(0x02))
#define USBHS_SPEED_FULL              ((uint8_t)(0x00))
#define USBHS_SPEED_HIGH              ((uint8_t)(0x01))
#endif

extern const uint8_t DevDesc[];
extern const uint8_t CfgDesc[];
#if defined(CH585) || defined(CH32V30x)
extern const uint8_t CfgHsDesc[];
extern const uint8_t QualDesc[];
extern uint8_t CfgOtherDesc[];
#endif
extern const uint8_t LangDes[];
extern const uint16_t SerDes[];
extern const uint16_t Prod_Des[];
extern const uint16_t CDC_Des[];
extern const uint16_t Manuf_Des[];



extern const uint16_t DevDescLen;
extern const uint16_t CfgDescLen;
#if defined(CH585) || defined(CH32V30x)
extern const uint16_t CfgHsDescLen;
extern const uint16_t QualDescLen;
#endif
extern const uint16_t LangDesLen;
extern const uint16_t SerDesLen;
extern const uint16_t Prod_DesLen;
extern const uint16_t CDC_DesLen;
extern const uint16_t Manuf_DesLen;

#if defined (CH32X035)
#include "ch32x035_usb_const.h"
#include "ch32x035.h"
#endif

#endif
