#ifndef _USBPD_SINK_H
#define _USBPD_SINK_H

#ifdef __cplusplus
extern "C" {
#endif /* end of __cplusplus */

#include "usbpd_def.h"

// Register Bit Definition
//USBPD->CONFIG
#define PD_ALL_CLR          (1<<1)              //PD mode clears all interrupt flags, 0: invalid, 1: clear interrupt flags
#define CC_SEL              (1<<2)              //Select current PD communication port,0: use CC1 port to communicate,1: use CC2 port to communicate
#define PD_DMA_EN           (1<<3)              // Enable DMA for USBPD, this bit must be set to 1 in normal transfer mode,1: Enable DMA function and DMA interrupt,0: Disable DMA.
#define PD_RST_EN           (1<<4)              //PD mode reset command enable,0: invalid,1: reset
#define WAKE_POLAR          (1<<5)              //PD port wake-up level, 0: active low, 1: active high
#define IE_PD_IO            (1<<10)             //PD IO interrupt enable
#define IE_RX_BIT           (1<<11)             //Receive bit interrupt enable
#define IE_RX_BYTE          (1<<12)             // Receive byte interrupt enable
#define IE_RX_ACT           (1<<13)             //Receive completion interrupt enable
#define IE_RX_RESET         (1<<14)             //Receive reset interrupt enable
#define IE_TX_END           (1<<15)             //End-of-send interrupt enable

//USBPD->CONTROL
#define PD_TX_EN            (1<<0)              // USBPD transceiver mode and transmit enable,0: PD receive enable,1: PD transmit enable
#define BMC_START           (1<<1)              //BMC sends start signal
#define DATA_FLAG           (1<<5)              //Cache data valid flag bit
#define TX_BIT_BACK         (1<<6)              // Indicates the current bit status of the BMC when sending the code,0: idle;,1: indicates that the BMC bytes are being sent
#define BMC_BYTE_HI         (1<<7)              // Indicates the current half-byte status of the PD data being sent and received, 0: the lower 4 bits are being processed, 1: the upper 4 bits are being processed

//USBPD->TX_SEL
#define TX_SEL1             (0<<0)
    #define TX_SEL1_SYNC1           (0<<0)               //0：SYNC1
    #define TX_SEL1_RST1            (1<<0)               //1：RST1
#define TX_SEL2             (0<<2)
    #define TX_SEL2_SYNC1           (0<<2)               //00:SYNC1
    #define TX_SEL2_SYNC3           (1<<2)               //01:SYNC3
    #define TX_SEL2_RST1            (2<<2)               //1x:RST1
#define TX_SEL3             (0<<4)
    #define TX_SEL3_SYNC1           (0<<4)               //00:SYNC1
    #define TX_SEL3_SYNC3           (1<<4)               //01:SYNC3
    #define TX_SEL3_RST1            (2<<4)               //1x:RST1
#define TX_SEL4             (0<<6)
    #define TX_SEL4_SYNC2           (0<<6)               //00:SYNC2
    #define TX_SEL4_SYNC3           (1<<6)               //01:SYNC3
    #define TX_SEL4_RST2            (2<<6)               //1x:RST2

//USBPD->STATUS
#define BMC_AUX            (3<<0)               //BMC auxiliary information, when doing receive SOP:,when doing send CRC: CRC byte counter
    #define BMC_AUX_INVALID         (0<<0)               //00: not valid
    #define BMC_AUX_SOP0            (1<<0)               //01：SOP0
    #define BMC_AUX_SOP1_HRST       (2<<0)               //10：SOP1 hard reset
    #define BMC_AUX_SOP2_CRST       (3<<0)               //11：SOP2 cable reset
#define BUF_ERR            (1<<2)              //BUFFER or DMA error interrupt flag, write 1 to clear 0, write 0 to void
#define IF_RX_BIT          (1<<3)              //Receive bit or 5bit interrupt flag, write 1 to clear 0, write 0 to void
#define IF_RX_BYTE         (1<<4)              // Receive byte or SOP interrupt flag, write 1 to clear 0, write 0 to void
#define IF_RX_ACT          (1<<5)              //Receive completion interrupt flag, write 1 to clear 0, write 0 to void
#define IF_RX_RESET        (1<<6)              // Receive reset interrupt flag, write 1 to clear 0, write 0 to void
#define IF_TX_END          (1<<7)              //Transfer completion interrupt flag, write 1 to clear 0, write 0 to void

//USBPD->PORT_CC1
//USBPD->PORT_CC2
#define PA_CC_AI          (1<<0)               //CC1 port comparator analogue input
#define CC_PD             (1<<1)               //CC1 port down resistor enable,0: disable pull down resistor ,1: enable 5.1KΩ pull down resistor
#define CC_PU_CLR             (3<<2)               //CC1 port pull-up current selection
    #define CC_NO_PU                (0<<2)               //00: Pull-up current forbidden
    #define CC_PU_330               (1<<2)               //01：330uA
    #define CC_PU_180               (2<<2)               //10：180uA
    #define CC_PU_80                (3<<2)               //11:80uA
#define CC_LV0            (1<<4)               //CC1 port output low voltage enable,0: normal voltage VDD weak drive output,1: low voltage drive output
#define CC_CE             (7<<5)               //Enable of voltage comparator on port /CC1,001: Reserved
    #define CC_NO_CMP               (0<<5)               //000: closed
    #define CC_CMP_22               (2<<5)               //010：0.22V
    #define CC_CMP_45               (3<<5)               //011：0.45V
    #define CC_CMP_55               (4<<5)               //100：0.55V
    #define CC_CMP_66               (5<<5)               //101：0.66V
    #define CC_CMP_95               (6<<5)               //110：0.95V
    #define CC_CMP_123              (7<<5)               //111：1.23V

#define USBPD_IN_HVT      (1<<9)
/*********************************************************
 * PD pin PC14/PC15 high threshold input mode:
 * 1: High threshold input, ~2.2V typical, reduces PD pass
 * I/O power consumption during signalling;
 * 0: Normal GPIO threshold input. *
 * *******************************************************/
#define USBPD_PHY_V33     (1<<8)
/**********************************************************
* PD transceiver PHY pull-up limit configuration bits:
* 1: direct VDD, output voltage up to VDD, for VDD
* for applications with 3.3V;
* 0: LDO buck enabled, limited to approx. 3.3V, for applications with VDD
* applications with more than 4V.
* ********************************************************/



/* PD Revision */
#define DEF_PD_REVISION_10         0x00
#define DEF_PD_REVISION_20         0x01
#define DEF_PD_REVISION_30         0x02


#define UPD_TMR_TX_48M    (80-1)                                                                // timer value for USB PD BMC transmittal @Fsys=48MHz
#define UPD_TMR_RX_48M    (120-1)                                                               // timer value for USB PD BMC receiving @Fsys=48MHz
#define UPD_TMR_TX_24M    (40-1)                                                                // timer value for USB PD BMC transmittal @Fsys=24MHz
#define UPD_TMR_RX_24M    (60-1)                                                                // timer value for USB PD BMC receiving @Fsys=24MHz
#define UPD_TMR_TX_12M    (20-1)                                                                // timer value for USB PD BMC transmittal @Fsys=12MHz
#define UPD_TMR_RX_12M    (30-1)                                                                // timer value for USB PD BMC receiving @Fsys=12MHz

#define MASK_PD_STAT      0x03                                                                  // ReadOnly: bit mask for current PD status
#define PD_RX_SOP0        0x01                                                                  // SOP0 received for rx
#define PD_RX_SOP1_HRST   0x02                                                                  // SOP1 or Hard Reset received for rx
#define PD_RX_SOP2_CRST   0x03                                                                  // SOP2 or Cable Reset received for rx

#define USBPD_SOP0          ( TX_SEL1_SYNC1 | TX_SEL2_SYNC1 | TX_SEL3_SYNC1 | TX_SEL4_SYNC2 )     // Start of Packet Sequence
#define USBPD_SOP1          ( TX_SEL1_SYNC1 | TX_SEL2_SYNC1 | TX_SEL3_SYNC3 | TX_SEL4_SYNC3 )     // Start of Packet Sequence Prime
#define USBPD_SOP2          ( TX_SEL1_SYNC1 | TX_SEL2_SYNC3 | TX_SEL3_SYNC1 | TX_SEL4_SYNC3 )     // Start of Packet Sequence Double Prime
#define USBPD_HARD_RESET    ( TX_SEL1_RST1  | TX_SEL2_RST1  | TX_SEL3_RST1  | TX_SEL4_RST2  )     // Hard Reset
#define USBPD_CABLE_RESET   ( TX_SEL1_RST1  | TX_SEL2_SYNC1 | TX_SEL3_RST1  | TX_SEL4_SYNC3 )     // Cable Reset




#define PIN_CC1                    GPIO_Pin_14
#define PIN_CC2                    GPIO_Pin_15

typedef enum
{
    REQUEST_5v = 0,
    REQUEST_9v,
    REQUEST_12v,
    REQUEST_15v,
    REQUEST_20v,
}Request_voltage_t;

// #define REQUEST_5V   5u
// #define REQUEST_9v   9u
// #define REQUEST_12v  12u
// #define REQUEST_15v  15u
// #define REQUEST_20v  20u

typedef struct 
{
    uint16_t Current;
    uint16_t Voltage;
}FixedSourceCap_t;

typedef struct 
{
    uint16_t MinVoltage;
    uint16_t MaxVoltage;
    uint16_t Current;
}PPSSourceCap_t;


typedef enum
{
    CC_IDLE = 0u,
    CC_CHECK_CONNECT,
    CC_CONNECT,
    CC_SOURCE_CAP,
    CC_SEND_REQUEST,
    CC_WAIT_ACCEPT,
    CC_ACCEPT,
    CC_WAIT_PS_RDY,
    CC_PS_RDY,
    CC_GET_SOURCE_CAP,
}cc_state_t;

typedef struct 
{
    volatile cc_state_t cc_State;
    volatile cc_state_t cc_LastState;
    FixedSourceCap_t *cc_FixedSourceCap;
    PPSSourceCap_t*   cc_PPSSourceCap;
    volatile uint8_t    cc1_ConnectTimes;
    volatile uint8_t    cc2_ConnectTimes;
    volatile uint8_t    cc_NoneTimes;
    volatile uint8_t    cc_SourcePDONum;
    volatile uint8_t    cc_SourcePPSNum;
    volatile uint8_t    cc_PD_Version;
    volatile uint16_t   cc_WaitTime;
    volatile uint8_t    cc_SetPDONum;
    volatile uint8_t    cc_LastSetPDONum;
    volatile uint8_t    cc_USBPD_READY;

    volatile uint8_t    cc_SourceMessageID;
    volatile uint8_t    cc_SinkMessageID;
    volatile uint8_t    cc_SinkGoodCRCOver;
    volatile uint8_t    cc_SourceGoodCRCOver;

}pd_control_t;




void usbpd_sink_init(void);
void usbpd_sink_process(void);

uint8_t usbpd_sink_get_ready(void);
void usbpd_sink_clear_ready(void);

void usbpd_sink_set_request_fixed_voltage(Request_voltage_t requestVoltage);


#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif /* end of usbpd_sink.h */

