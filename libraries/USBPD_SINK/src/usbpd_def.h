#ifndef _USBPD_DEF_H
#define _USBPD_DEF_H

#ifdef __cplusplus
extern "C" {
#endif /* end of __cplusplus */

#include <stdint.h>



/*  get values from PDO representation */
#define POWER_DECODE_50MV(value)      ((uint16_t)(((value) * 50)))     /* From 50mV  multiples to mV */
#define POWER_DECODE_100MV(value)     ((uint16_t)(((value) * 100)))    /* From 100mV multiples to mV */
#define POWER_DECODE_10MA(value)      ((uint16_t)(((value) * 10)))     /* From 10mA  multiples to mA */
#define POWER_DECODE_50MA(value)      ((uint16_t)(((value) * 50)))     /* From 50mA  multiples to mA */


/* PD PHY Channel */
#define USBPD_CCNONE  0x00u
#define USBPD_CC1     0x01u
#define USBPD_CC2     0x02u

/* USB PD Revision */
#define USBPD_SPECIFICATION_REV1        0x00u  // Revision 1.0  
#define USBPD_SPECIFICATION_REV2        0x01u  // Revision 2.0    
#define USBPD_SPECIFICATION_REV3        0x02u  // Revision 3.0     


typedef struct 
{
    uint32_t MaxCurrentIn10mAunits      : 10u;
    uint32_t VolatageIn50mVunits        : 10u;
    uint32_t PeakCurrent                : 2u;
    uint32_t Reserved_22bit             : 1u;
    uint32_t EPRModeCapable             : 1u;
    uint32_t UnchunkedExterndedMessage  : 1u;
    uint32_t DualRoleData               : 1u;
    uint32_t USBCommunicationsCapable   : 1u;
    uint32_t UnconstrainedPowe          : 1u;
    uint32_t USBSuspendSupported        : 1u;
    uint32_t DualRolePower              : 1u;
    uint32_t FixedSupply                : 2u;
}USBPD_SourceFixedSupplyPDO_t;

typedef struct  
{
    uint32_t MaxCurrentIn50mAincrements : 7u;
    uint32_t Reserved_7bit              : 1u; // shall br set to zero
    uint32_t MinVlotageIn100mVincrements: 8u;
    uint32_t Reserved_16bit             : 1u; // shall br set to zero
    uint32_t MaxVlotageIn100mVincrements: 8u;
    uint32_t Reserved_25_26bit          : 2u; // shall br set to zero
    uint32_t PPSpowerLimited            : 1u; 
    uint32_t SPRprogrammablePowerSupply : 2u; // 00b
    uint32_t AugmentedPowerDataObject   : 2u; // 11b
}USBPD_SourcePPSSupplyPDO_t;

typedef union 
{
    uint32_t d32;
    USBPD_SourceFixedSupplyPDO_t SourceFixedPDO;
    USBPD_SourcePPSSupplyPDO_t   SourcePPSPDO;
}USBPD_PDO_t;

typedef enum
{
    USBPD_CONTROL_MSG_GOODCRC               = 0x01u,
    USBPD_CONTROL_MSG_GOTOMIN               = 0x02u, 
    USBPD_CONTROL_MSG_ACCEPT                = 0x03u,  
    USBPD_CONTROL_MSG_REJECT                = 0x04u,  
    USBPD_CONTROL_MSG_PING                  = 0x05u,  
    USBPD_CONTROL_MSG_PS_RDY                = 0x06u,  
    USBPD_CONTROL_MSG_GET_SRC_CAP           = 0x07u,  
    USBPD_CONTROL_MSG_GET_SNK_CAP           = 0x08u,  
    USBPD_CONTROL_MSG_DR_SWAP               = 0x09u,  
    USBPD_CONTROL_MSG_PR_SWAP               = 0x0Au,  
    USBPD_CONTROL_MSG_VCONN_SWAP            = 0x0Bu, 
    USBPD_CONTROL_MSG_WAIT                  = 0x0Cu, 
    USBPD_CONTROL_MSG_SOFT_RESET            = 0x0Du,  
    USBPD_CONTROL_MSG_DATA_RESET            = 0x0Eu,  
    USBPD_CONTROL_MSG_DATA_RESET_COMPLETE   = 0x0Fu,  
    USBPD_CONTROL_MSG_NOT_SUPPORTED         = 0x10u, 
    USBPD_CONTROL_MSG_GET_SRC_CAPEXT        = 0x11u,  
    USBPD_CONTROL_MSG_GET_STATUS            = 0x12u,  
    USBPD_CONTROL_MSG_FR_SWAP               = 0x13u,  
    USBPD_CONTROL_MSG_GET_PPS_STATUS        = 0x14u, 
    USBPD_CONTROL_MSG_GET_COUNTRY_CODES     = 0x15u,  
    USBPD_CONTROL_MSG_GET_SNK_CAPEXT        = 0x16u,
    USBPD_CONTROL_MSG_GET_SRC_INFO          = 0x17u,  
    USBPD_CONTROL_MSG_GET_REVISION          = 0x18u,  
}USBPD_ControlMessage_t;

typedef enum
{
    USBPD_DATA_MSG_SRC_CAP                  = 0x01u,
    USBPD_DATA_MSG_REQUEST                  = 0x02u, 
    USBPD_DATA_MSG_BIST                     = 0x03u, 
    USBPD_DATA_MSG_SNK_CAP                  = 0x04u, 
    USBPD_DATA_MSG_BATTERY_STATUS           = 0x05u,  
    USBPD_DATA_MSG_ALERT                    = 0x06u,  
    USBPD_DATA_MSG_GET_COUNTRY_INFO         = 0x07u,  
    USBPD_DATA_MSG_ENTER_USB                = 0x08u,
    USBPD_DATA_MSG_EPR_REUEST               = 0x09u,
    USBPD_DATA_MSG_EPR_MODE                 = 0x0Au,
    USBPD_DATA_MSG_SRC_INFO                 = 0x0Bu,
    USBPD_DATA_MSG_REVISION                 = 0x0Cu,
    USBPD_DATA_MSG_VENDOR_DEFINED           = 0x0Fu  
}USBPD_DataMessage_t;

typedef union 
{
    uint16_t d16;
    USBPD_DataMessage_t DataMessage;
    USBPD_ControlMessage_t ControlMessage;
}USBPD_MessageType_t;


typedef struct 
{
    uint16_t MessageType               : 5u;
    uint16_t PortDataRole              : 1u; // 0b->UFP, 1b->DFP
    uint16_t SpecificationRevision     : 2u; // 00b->v1.0, 01b->v2.0, 10b->v3.0
    uint16_t PortPowerRole             : 1u; // 0b->sink, 1b->source
    uint16_t MessageID                 : 3u;
    uint16_t NumberOfDataObjects       : 3u;
    uint16_t Extended                  : 1u;
}USBPD_MessageHeader_tt;

typedef union 
{
    uint16_t d16;
    USBPD_MessageHeader_tt MessageHeader;
}USBPD_MessageHeader_t;

typedef struct 
{
    uint32_t MaxOperatingCurrent10mAunits : 10u;
    uint32_t OperatingCurrentIn10mAunits  : 10u;
    uint32_t Reserved20_21                : 2u; // 00b
    uint32_t EPRModeCapable               : 1u; 
    uint32_t UnchunkedExtendedMessage     : 1u;
    uint32_t NoUSBSuspend                 : 1u;
    uint32_t USBCommunicationsCapable     : 1u;
    uint32_t CapabilityMismatch           : 1u;
    uint32_t GiveBackFlag                 : 1u;
    uint32_t ObjectPosition               : 4u;
}USBPD_SinkFixedVariableRDO_t;

typedef struct 
{
    uint32_t OperatingCurrentIN50mAuints  : 7u;
    uint32_t Reserved7_8                  : 2u; // shall be set to zero
    uint32_t OutputVoltagein20mVunits     : 12u;
    uint32_t Reserved21                   : 1u; // shall be set to zero
    uint32_t EPRModeCapable               : 1u;
    uint32_t UnchunkedExtendedMessage     : 1u;
    uint32_t NoUSBSuspend                 : 1u;
    uint32_t USBCommunicationsCapable     : 1u;
    uint32_t CapabilityMismatch           : 1u;
    uint32_t Rserved27                    : 1u; // shall be set to zero
    uint32_t ObjectPosition               : 4u;
}USBPD_SinkPPSRDO_t;


typedef union 
{
    uint32_t d32;
    USBPD_SinkFixedVariableRDO_t SinkFixedVariableRDO;
    USBPD_SinkPPSRDO_t           SinkPPSRDO;
}USBPD_SINKRDO_t;


#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif /* end of usbpd_def.h*/

