/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v4x7_psram.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/12/01
* Description        : This file contains all the functions prototypes for the PSRAM
*                      firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32V4x7_PSRAM_H
#define __CH32V4x7_PSRAM_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32v4x7.h"


/* PSRAM Time Init structure definition */
typedef struct
{
  uint32_t PSRAM_trc;       /* Defines minimum time of read-write cycle. 
                               This parameter must range from 0 to 0XFF. */

  uint32_t PSRAM_tcph;      /* Defines the minimum interval between two consecutive bursts.
                                This parameter must range from 0 to 0XFF. */

  uint32_t PSRAM_txlpd;      /* Defines exit low power CE_n continuous pulse width. 
                                This parameter must range from 0 to 0XFF. */

}PSRAMTimingInitTypeDef;


/* PSRAM Init structure definition */
typedef struct
{
  uint32_t PSRAM_cfifo;       /* Clear read FIFO mode selection
                               This parameter can be a value of @ref PSRAM_CFIFO_selsct*/

  uint32_t PSRAM_cap_cfg;      /* PSRAM capacity configuration 
                               This parameter can be a value of @ref PSRAM_CAP_selsct*/

  uint32_t PSRAM_exti_lpmd;    /* PSRAM exit current low power mode enable 
                                 This parameter can be set to ENABLE or DISABLE. */

  PSRAMTimingInitTypeDef* PSRAMTimingStruct; /* Timing Parameters for write and read access if the  ExtendedMode is not used*/

}PSRAMInitTypeDef;

/* PSRAM DMA Init structure definition */
typedef struct
{
  uint32_t PSRAM_DMA_DAT_DIR;       /* PSRAM DMA DIR 
                                     This parameter can be a value of @ref PSRAM_DMA_read_write_select*/

  uint32_t PSRAM_DMA_MEMORY_SIZE;   /* PSRAM memory size 
                                     This parameter can be a value of @ref PSRAM_DMA_DMA_SIZE*/

  uint32_t PSRAM_DMA_DMA_ONEBL;    /* PSRAM DMA burst length 
                                     This parameter can be a value of @ref PSRAM_DMA_DMA_ONEBL_select*/

 uint32_t PSRAM_DMA_TOWBL_TOUT;    /* PSRAM pause tine 
                                   This parameter can be a value of @ref PSRAM_DMA_TOWBL_TOUT_select */

 uint32_t PSRAM_DMA_DATA_NUM;     /* PSRAM pause tine 
                                   This parameter must range from 0 to 0XFFFF. */                                   
                                  

}PSRAMDMATypeDef;


/*PSRAM_CFIFO_selsct */
#define  PSRAM_CFIFO_WRADD                               ((uint32_t)0x00000010)
#define  PSRAM_CFIFO_BTWWRRD                             ((uint32_t)0x00000000)

/*PSRAM_DFIFO_selsct */
#define  PSRAM_DFIFO_CLSIM                               ((uint32_t)0x00000020)
#define  PSRAM_DFIFO_CLNSIM                              ((uint32_t)0x00000000)

/*PSRAM_CAP_selsct */
#define  PSRAM_CAP_32M                                   ((uint32_t)0x00000100)
#define  PSRAM_CAP_64M                                   ((uint32_t)0x00000300)
#define  PSRAM_CAP_128M                                  ((uint32_t)0x00000500)
#define  PSRAM_CAP_256M                                  ((uint32_t)0x00000700)

/*PSRAM operation select*/
#define CMD_MW_Write                                     ((uint32_t)0x00000002)
#define CMD_MW_Read                                      ((uint32_t)0x00000000)

/*PSRAM_wait_BUSY*/
#define CMD_BUSY                                         ((uint32_t)0x00000001)

/* PSRAM_state_machine_Flags */
#define  Idle_state                                      ((uint32_t)0x00000000)
#define  SdAdd_state                                     ((uint32_t)0x00000001)
#define  WrReg_state                                     ((uint32_t)0x00000002)
#define  ReReg_state                                     ((uint32_t)0x00000003)
#define  WrDelay_state                                   ((uint32_t)0x00000004)
#define  WrDat_state                                     ((uint32_t)0x00000006)
#define  RdDelay_state                                   ((uint32_t)0x00000007)
#define  WtCEM_Completion                                ((uint32_t)0x0000000F)

/*PSRAM_DMA_read_write_select*/
#define DMA_DIR_PSRAM                                    ((uint32_t)0x00000001)
#define DMA_DIR_MEM                                      ((uint32_t)0x00000000)

/*PSRAM_DMA_DMA_SIZE */
#define PSRAM_DMA_SIZE_16                                ((uint32_t)0x00000400)
#define PSRAM_DMA_SIZE_32                                ((uint32_t)0x00000800)

/*PSRAM_DMA_DMA_ONEBL_select*/
#define DMA_BRST_NUM2                                    ((uint32_t)0x00000000)
#define DMA_BRST_NUM4                                    ((uint32_t)0x00000001)
#define DMA_BRST_NUM6                                    ((uint32_t)0x00000002)
#define DMA_BRST_NUM8                                    ((uint32_t)0X00000003)
#define DMA_BRST_NUM16                                   ((uint32_t)0X00000008)

/*PSRAM_DMA_TOWBL_TOUT_select*/
#define DMA_PAUSE_TIM0                                   ((uint32_t)0X00000000)
#define DMA_PAUSE_TIM1                                   ((uint32_t)0X00000001)
#define DMA_PAUSE_TIM2                                   ((uint32_t)0X00000002)
#define DMA_PAUSE_TIM3                                   ((uint32_t)0X00000003)
#define DMA_PAUSE_TIM4                                   ((uint32_t)0X00000004)
#define DMA_PAUSE_TIM5                                   ((uint32_t)0X00000005)
#define DMA_PAUSE_TIM6                                   ((uint32_t)0X00000006)
#define DMA_PAUSE_TIM7                                   ((uint32_t)0X00000007)
#define DMA_PAUSE_TIM8                                   ((uint32_t)0X00000008)
#define DMA_PAUSE_TIM9                                   ((uint32_t)0X00000009)
#define DMA_PAUSE_TIM10                                  ((uint32_t)0X0000000A)
#define DMA_PAUSE_TIM11                                  ((uint32_t)0X0000000B)
#define DMA_PAUSE_TIM12                                  ((uint32_t)0X0000000C)
#define DMA_PAUSE_TIM13                                  ((uint32_t)0X0000000D)
#define DMA_PAUSE_TIM14                                  ((uint32_t)0X0000000E)
#define DMA_PAUSE_TIM15                                  ((uint32_t)0X0000000F)

/*PSRAM_DMA_enable_select*/
#define PSRAM_DMA_Enable                                 ((uint32_t)0X00000001)
#define PSRAM_DMA_Disnable                               ((uint32_t)0X00000000)


void PSRAMDeInit();
void PSRAMInit(PSRAMInitTypeDef *PSRAMInitStruct);
void PSRAM_StructInit(PSRAMInitTypeDef *PSRAMInitStruct);
void PSRAMSetWrLatency(uint32_t Write_Latency);
void PSRAMSetRdLatency(uint32_t Read_Latency);
void PSRAMSetData(uint16_t data);
uint16_t PSRAMGetData();
void PSRAM_Set_Busy(FunctionalState NewState);
void PSRAM_Set_MW(FunctionalState NewState);
void PSRAM_Set_CMD(uint8_t CMD);
void PSRAM_Set_MR_ADDR(uint8_t ADDR );
void PSRAMDMASet(uint32_t *pBuffer,uint32_t WrReAddr,PSRAMDMATypeDef *PSRAMDMAStruct);
void PSRAMClearFifoMode(u8 CFIFO);
void PSRAM_DMA_Cmd(FunctionalState NewState);
void PSRAM_DMA_DIR(uint32_t DIR);
void PSRAM_DMA_CFIFO(FunctionalState NewState);
void PSRAM_CEPW_Cmd(FunctionalState NewState);
FlagStatus PSRAM_GetFlagStatus(uint32_t PSRAM_FLAG);
void PSRAM_ITConfig(uint32_t PSRAM_IT, FunctionalState NewState);
FlagStatus PSRAM_GetTranFlagStatus(uint32_t PSRAM_TranFLAG);
void PSRAM_ClearITPendingBit(uint32_t PSRAM_IT);
void PSRAM_SyncBur_write_Cmd_Set(uint8_t CMD);
void PSRAM_SyncBur_Read_Cmd_Set(uint8_t CMD);
void PSRAM_Sync_write_Cmd_Set(uint8_t CMD);
void PSRAM_Sync_Read_Cmd_Set(uint8_t CMD);
void PSRAM_Refresh_Cmd_Set(uint8_t CMD);
void PSRAM_Global_RST_Cmd_Set(uint8_t CMD);
void PSRAM_WRReg_Cmd_Set(uint8_t CMD);
void PSRAM_RDReg_Cmd_Set(uint8_t CMD);
#ifdef __cplusplus
}
#endif

#endif 
