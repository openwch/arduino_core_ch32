/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v4x7_i3c.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/12/01
* Description        : This file contains all the functions prototypes for the
*                      I3C firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32V4x7_I3C_H
#define __CH32V4x7_I3C_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32v4x7.h"

/* I3C controller bus Init structure definition */
typedef struct
{
    uint32_t SDAHoldTime; /* Specifies the I3C SDA hold time.
                                This parameter must be a value of @ref SDAHoldTime */

    uint32_t WaitTime; /* Specifies the time that the main and the new controllers should wait before
                                issuing a start.
                                This parameter must be a value of @ref WaitTime */

    uint8_t BusFreeDuration; /* Specifies the I3C controller duration in number of kernel clock cycles, after
                                a stop and before a start.
                                This parameter must be a number  from 0 to 0xFF. */

    uint8_t BusIdleDuration; /* Specifies the I3C controller duration in number of kernel clock cycles to be
                                elapsed, after that both SDA and SCL are continuously high and stable
                                before issuing a hot-join event.
                                This parameter must be a number  from 0 to 0xFF. */

    uint8_t SCLPPLowDuration; /* Specifies the I3C SCL low duration in number of kernel clock cycles
                                in I3C push-pull phases.
                                This parameter must be a number from 0 to 0xFF. */

    uint8_t SCLI3CHighDuration; /* Specifies the I3C SCL high duration in number of kernel clock cycles,
                                used for I3C messages for I3C open-drain and push pull phases.
                                This parameter must be a number from 0 to 0xFF. */

    uint8_t SCLODLowDuration; /* Specifies the I3C SCL low duration in number of kernel clock cycles in
                                open-drain phases, used for legacy I2C commands and for I3C open-drain phases.
                                This parameter must be a number  from 0 to 0xFF. */

    uint8_t SCLI2CHighDuration; /* Specifies the I3C SCL high duration in number of kernel clock cycles, used
                                for legacy I2C commands.
                                This parameter must be a number  from 0 to 0xFF. */
} I3C_Ctrl_BusTypeDef;

/* I3C controller configration Init structure definition */
typedef struct
{
    uint8_t DynamicAddr; /* Specifies the dynamic address of the controller when goes in target mode.
                                This parameter must be a number  from 0 to 0x7F. */

    uint8_t StallTime; /*  Specifies the controller clock stall time in number of kernel clock cycles.
                                This parameter must be a number  from 0 to 0xFF. */

    FunctionalState PullUp; /*  Specifies the Enable/Disable state of the controller Pull Up.
                                This parameter can be set to ENABLE or DISABLE */

    FunctionalState HotJoinAllowed_EN; /*  Specifies the Enable/Disable state of the controller Hot Join acknowledgement
                                when receiving a hot join request from target.
                                This parameter can be set to ENABLE or DISABLE */

    FunctionalState ACKStallState; /*  Specifies the Enable/Disable state of the controller clock stall
                                on the ACK phase.
                                This parameter can be set to ENABLE or DISABLE */

    FunctionalState CCCStallState; /*  Specifies the Enable/Disable state of the controller clock stall on the
                                T bit phase of a CCC communication to allow the target to decode command.
                                This parameter can be set to ENABLE or DISABLE */

    FunctionalState TxStallState; /*  Specifies the Enable/Disable state of the controller clock stall on
                                parity phase of data to allow the target to read received data.
                                This parameter can be set to ENABLE or DISABLE */

    FunctionalState RxStallState; /*  Specifies the Enable/Disable state of the controller clock stall on the T bit
                                phase of data enable to allow the target to prepare data to be sent.
                                This parameter can be set to ENABLE or DISABLE */
} I3C_CtrlConfTypeDef;

/* I3C device configration Init structure definition */
typedef struct
{
    uint8_t DeviceIndex; /*  Specifies the index value of the device in the DEVRx register.
                                    This parameter must be a a value of @ref DeviceIndex */

    uint8_t TargetDynamicAddr; /*  Specifies the dynamic address of the target x (1 to 4) connected on the bus.
                                    This parameter must be a number from 0 to 0x7F.  */

    FunctionalState IBIAck; /*  Specifies the Enable/Disable state of the controller's ACK when receiving
                                    an IBI from a target x (1 to 4) connected on the bus.
                                    This parameter can be set to ENABLE or DISABLE */

    FunctionalState IBIPayload; /*  Specifies the Enable/Disable state of the controller's receiving IBI payload
                                    after acknowledging an IBI requested from a target x (1 to 4) connected
                                    on the bus.
                                    This parameter can be set to ENABLE or DISABLE */

    FunctionalState CtrlStopTransfer; /*  Specifies the Enable/Disable state of the controller's stop transfer after
                                    receiving an IBI request from a target x (1 to 4) connected on the bus.
                                    This parameter can be set to ENABLE or DISABLE */

} I3C_DeviceConfTypeDef;

/* I3C_FIFO_Configration structure definition */
typedef struct
{
    uint32_t RxFifoThreshold; /*  Specifies the I3C Rx FIFO threshold level.
                            This parameter must be a value of @ref I3C_RX_FIFO_THRESHOLD */

    uint32_t TxFifoThreshold; /*  Specifies the I3C Tx FIFO threshold level.
                            This parameter must be a value of @ref I3C_TX_FIFO_THRESHOLD */

    uint32_t ControlFifo; /*  Specifies the I3C control FIFO enable/disable state.
                            This parameter is configured only with controller mode and it
                            must be a value of @ref I3C_CONTROL_FIFO_STATE */

} I3C_FifoConfTypeDef;

/* I3C_SDAHoldTime*/
#define I3C_SDAHoldTime_0_5              ((uint32_t)0x00000000)
#define I3C_SDAHoldTime_1_5              ((uint32_t)0x10000000)

/* I3C_WaitTime */
#define I3C_WaitTime_State_0             ((uint32_t)0x00000000)
#define I3C_WaitTime_State_1             ((uint32_t)0x00000100)
#define I3C_WaitTime_State_2             ((uint32_t)0x00000200)
#define I3C_WaitTime_State_3             ((uint32_t)0x00000300)

/* I3C_IBIPayloadSize */
#define I3C_IBIPayloadSize_None          ((uint32_t)0x00000000)
#define I3C_IBIPayloadSize_1B            ((uint32_t)0x00010000)
#define I3C_IBIPayloadSize_2B            ((uint32_t)0x00020000)
#define I3C_IBIPayloadSize_3B            ((uint32_t)0x00030000)
#define I3C_IBIPayloadSize_4B            ((uint32_t)0x00040000)

/* I3C_DataTurnAroundDuration */
#define I3C_DataTurnAroundDuration_Mode0 ((uint32_t)0x00000000)
#define I3C_DataTurnAroundDuration_Mode1 ((uint32_t)0x01000000)

/* I3C_MaxDataSpeed */
#define I3C_MaxDataSpeed_Format_Mode0    ((uint32_t)0x00000000)
#define I3C_MaxDataSpeed_Format_Mode1    ((uint32_t)0x00000100)
#define I3C_MaxDataSpeed_Format_Mode2    ((uint32_t)0x00000200)
#define I3C_MaxDataSpeed_Format_Mode3    ((uint32_t)0x00000300)

/* I3C_HandOffActivityState */
#define I3C_HandOffActivityState_0       ((uint32_t)0x00000000)
#define I3C_HandOffActivityState_1       ((uint32_t)0x00000001)
#define I3C_HandOffActivityState_2       ((uint32_t)0x00000002)
#define I3C_HandOffActivityState_3       ((uint32_t)0x00000003)

/* I3C_DeviceIndex */
#define I3C_DeviceIndex_1                ((uint32_t)0x00000001)
#define I3C_DeviceIndex_2                ((uint32_t)0x00000002)
#define I3C_DeviceIndex_3                ((uint32_t)0x00000003)
#define I3C_DeviceIndex_4                ((uint32_t)0x00000004)

/* I3C_Direction */
#define I3C_Direction_WR                 ((uint32_t)0x00000000)
#define I3C_Direction_RD                 ((uint32_t)0x00010000)

/* I3C_CONTROLLER MessageType */
#define I3C_CONTROLLER_MTYPE_RELEASE     ((uint32_t)0x00000000)
#define I3C_CONTROLLER_MTYPE_HEADER      I3C_CTLR_MTYPE_0
#define I3C_CONTROLLER_MTYPE_PRIVATE     I3C_CTLR_MTYPE_1
#define I3C_CONTROLLER_MTYPE_DIRECT      (I3C_CTLR_MTYPE_1 | I3C_CTLR_MTYPE_0)
#define I3C_CONTROLLER_MTYPE_LEGACY_I2C  I3C_CTLR_MTYPE_2
#define I3C_CONTROLLER_MTYPE_CCC         (I3C_CTLR_MTYPE_2 | I3C_CTLR_MTYPE_1)

/* I3C_EndMode */
#define I3C_GENERATE_STOP                I3C_CTLR_MEND
#define I3C_GENERATE_RESTART             ((uint32_t)0x00000000)

/* I3C_PeripheralMode */
#define PeripheralMode_CONTROLLER        ((uint32_t)0x00000000)

/* I3C_RX_FIFO_THRESHOLD */
#define I3C_RXFIFO_THRESHOLD_1_4         ((uint32_t)0x00000000)
#define I3C_RXFIFO_THRESHOLD_4_4         I3C_CFGR_RXTHRES

/* I3C_TX_FIFO_THRESHOLD */
#define I3C_TXFIFO_THRESHOLD_1_4         ((uint32_t)0x00000000)
#define I3C_TXFIFO_THRESHOLD_4_4         I3C_CFGR_TXTHRES

/* I3C_CONTROL_FIFO_STATE */
#define I3C_CONTROLFIFO_DISABLE          ((uint32_t)0x00000000)
#define I3C_CONTROLFIFO_ENABLE           I3C_CFGR_TMODE

/* I3C_flags_definition */
#define I3C_FLAG_CFEF                    ((uint32_t)0x00000001)
#define I3C_FLAG_TXFEF                   ((uint32_t)0x00000002)
#define I3C_FLAG_CFNFF                   ((uint32_t)0x00000004)
#define I3C_FLAG_TXFNFF                  ((uint32_t)0x00000010)
#define I3C_FLAG_RXFNEF                  ((uint32_t)0x00000020)
#define I3C_FLAG_TXLASTF                 ((uint32_t)0x00000040)
#define I3C_FLAG_RXLASTF                 ((uint32_t)0x00000080)
#define I3C_FLAG_FCF                     ((uint32_t)0x00000200)
#define I3C_FLAG_RXTGTENDF               ((uint32_t)0x00000400)
#define I3C_FLAG_ERRF                    ((uint32_t)0x00000800)
#define I3C_FLAG_IBIF                    ((uint32_t)0x00008000)
#define I3C_FLAG_HJF                     ((uint32_t)0x00080000)

/* I3C_interrupts_definition */
#define I3C_IT_CFNFIE                    ((uint32_t)0x00000004)
#define I3C_IT_TXFNEIE                   ((uint32_t)0x00000010)
#define I3C_IT_RXFNEIE                   ((uint32_t)0x00000020)
#define I3C_IT_FCIE                      ((uint32_t)0x00000200)
#define I3C_IT_RXTGTENDIE                ((uint32_t)0x00000400)
#define I3C_IT_ERRIE                     ((uint32_t)0x00000800)
#define I3C_IT_IBIIE                     ((uint32_t)0x00008000)
#define I3C_IT_HJIE                      ((uint32_t)0x00080000)

/* I3C_ERROR */
#define I3C_ERROR_CE0                    ((uint32_t)0x00000000)
#define I3C_ERROR_CE1                    ((uint32_t)0x00000001)
#define I3C_ERROR_CE2                    ((uint32_t)0x00000002)
#define I3C_ERROR_CE3                    ((uint32_t)0x00000003)
#define I3C_ERROR_PERR                   ((uint32_t)0x00000010)
#define I3C_ERROR_DOVR                   ((uint32_t)0x00000040)
#define I3C_ERROR_COVR                   ((uint32_t)0x00000080)
#define I3C_ERROR_ADDRESS_NACK           ((uint32_t)0x00000100)
#define I3C_ERROR_DATA_NACK              ((uint32_t)0x00000200)
#define I3C_ERROR_DATA_HAND_OFF          ((uint32_t)0x00000400)

void I3C_DeInit(void);
void I3C_Ctrl_Init(I3C_Ctrl_BusTypeDef* I3C_InitStruct);
void I3C_Ctrl_Config(I3C_CtrlConfTypeDef* I3C_CtrlConf);
void I3C_Cmd(FunctionalState NewState);
void I3C_ArbitrationHeaderCmd(FunctionalState NewState);
void I3C_HJAckCmd(FunctionalState NewState);
void I3C_DMAReq_RXCmd(FunctionalState NewState);
void I3C_DMAReq_TXCmd(FunctionalState NewState);
void I3C_DMAReq_ControlCmd(FunctionalState NewState);
void I3C_ExitPatternCmd(FunctionalState NewState);
void I3C_RequestTransfer(void);
void I3C_TxPreloadConfig(uint16_t TxDataCount);
void I3C_CONTROLLER_ResetCmd(FunctionalState NewState);
void I3C_SetModeConfig(uint32_t PeripheralMode);
void I3C_Ctrl_ConfigBusDevices(I3C_DeviceConfTypeDef* pDesc);
void I3C_Ctrl_SetConfigResetPattern(FunctionalState resetPattern);
FunctionalState I3C_Ctrl_GetConfigResetPattern(void);
void I3C_FlushAllFifo(void);
void I3C_FlushControlFifo(void);
void I3C_SetConfigFifo(I3C_FifoConfTypeDef* I3C_FifoConf);
void I3C_ClearConfigFifo(void);
uint8_t I3C_ReadByte(void);
void I3C_WriteByte(uint8_t Byte);
void I3C_IBIDataConfig(uint32_t Data);
void I3C_ControllerHandleMessage(uint32_t TargetAddr, uint32_t TransferSize, uint32_t Direction, uint32_t MessageType, uint32_t EndMode);
void I3C_ControllerHandleCCC(uint32_t CCCValue, uint32_t AddByteSize, uint32_t EndMode);
uint8_t I3C_GetMessageDirection(void);
uint8_t I3C_GetTargetAbortPrivateRead(void);
uint16_t I3C_GetXferDataCount(void);
uint8_t I3C_GetMessageIdentifier(void);
uint8_t I3C_GetIBITargetAddr(void);
uint8_t I3C_GetReceiveCommandCode(void);
uint8_t I3C_GetNbIBIAddData(void);
uint8_t I3C_GetResetAction(void);
FunctionalState I3C_GetAllowedPayloadUpdate(uint32_t DeviceIndex);
void I3C_ITConfig(uint32_t I3C_IT, FunctionalState NewState);
ITStatus I3C_GetITStatus(uint32_t I3C_IT);
void I3C_ClearITPendingBit(uint32_t I3C_IT);
FlagStatus I3C_GetFlagStatus(uint32_t I3C_FLAG);
FlagStatus I3C_GetErrorStatus(uint32_t I3C_Error);
void I3C_ClearFlag(uint32_t I3C_FLAG);

#ifdef __cplusplus
}
#endif

#endif
