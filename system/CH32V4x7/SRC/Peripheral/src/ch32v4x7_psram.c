/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v4x7_psram.c
* Author             : WCH
* Version            : V1.0.1
* Date               : 2026/06/29
* Description        : This file provides all the PSRAM firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v4x7_psram.h"

/*********************************************************************
 * @fn      PSRAMDeInit
 *
 * @brief   Deinitializes the PSRAM registers to their default
 *          reset values.
 *
 * @return  none
 */
void PSRAMDeInit()
{
    RCC_HBPeriphResetCmd(RCC_HBPeriph_PSRAM, ENABLE);
    RCC_HBPeriphResetCmd(RCC_HBPeriph_PSRAM, DISABLE);
}

/*********************************************************************
 * @fn      PSRAMInit
 *
 * @brief   Initializes the PSRAM according to the specified
 *        parameters in the PSRAMInitStruct.
 *
 * @param   PSRAMInitTypeDef:pointer to a PSRAMInitTypeDef
 *        structure that contains the configuration information for the PSRAM.
 *
 * @return  none
 */
void PSRAMInit(PSRAMInitTypeDef *PSRAMInitStruct)
{
    PSRAM->CTLR=(uint32_t)PSRAMInitStruct->PSRAM_cfifo|
                PSRAMInitStruct->PSRAM_cap_cfg|
                PSRAMInitStruct->PSRAM_exti_lpmd;
    PSRAM->TIMING=(uint32_t)PSRAMInitStruct->PSRAMTimingStruct->PSRAM_trc|
                    ((PSRAMInitStruct->PSRAMTimingStruct->PSRAM_tcph)<<16)|
                    ((PSRAMInitStruct->PSRAMTimingStruct->PSRAM_txlpd)<<24);   
}


/*********************************************************************
 * @fn      PSRAM_StructInit
 *
 * @brief   Fills each PSRAMInitTypeDef member with its default value.
 *
 * @param   PSRAMInitStruct - pointer to an PSRAMInitTypeDef structure which
 *          will be initialized.
 *
 * @return  none
 */
void PSRAM_StructInit(PSRAMInitTypeDef *PSRAMInitStruct)
{
    PSRAMInitStruct->PSRAMTimingStruct->PSRAM_tcph=0x0C;
    PSRAMInitStruct->PSRAMTimingStruct->PSRAM_trc=0x14;
    PSRAMInitStruct->PSRAMTimingStruct->PSRAM_txlpd=0;
    PSRAMInitStruct->PSRAM_cfifo=0;
    PSRAMInitStruct->PSRAM_cap_cfg=PSRAM_CAPCFG_32M;
    PSRAMInitStruct->PSRAM_exti_lpmd=0;
}

/*********************************************************************
 * @fn      PSRAMSetWrLatency
 *
 * @brief   PSRAM set write Latency
 *
 * @param   Write_Latency - Write Latency.
 *
 * @return  none
 */
void PSRAMSetWrLatency(uint32_t Write_Latency)
{
    uint32_t temp;
    temp=PSRAM->LATENCY;
    PSRAM->LATENCY=temp&0xffff00ff|(Write_Latency<<8);
 
}

/*********************************************************************
 * @fn      PSRAMSetRdLatency
 *
 * @brief   PSRAM set read Latency
 *
 * @param   Write_Latency - read Latency.
 *
 * @return  none
 */
void PSRAMSetRdLatency(uint32_t Read_Latency)
{
    uint32_t temp;
    temp=PSRAM->LATENCY;
    PSRAM->LATENCY=temp&0xffffff00|Read_Latency;
}
/*********************************************************************
 * @fn      PSRAMSetData
 *
 * @brief   PSRAM set data.
 *
 * @param   data -PARAM data
 *
 * @return  none
 */
void PSRAMSetData(uint16_t data)
{
    PSRAM->DATA=data;
}
/*********************************************************************
 * @fn      PSRAMGetData
 *
 * @brief   PSRAM get data.
 *
 * @param   none.
 *
 * @return  tmpreg-PARAM data
 */
uint16_t PSRAMGetData()
{
    uint16_t tmpreg = 0;
    tmpreg=PSRAM->DATA;
    return tmpreg;
}

 /*********************************************************************
 * @fn      PSRAM_Set_Busy
 *
 * @brief   PSRAM Busy ENABLE.
 *
 * @param    NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void PSRAM_Set_Busy(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        PSRAM->ADDR|=(1<<0);
    }
    else
    {
        PSRAM->ADDR&=~(1<<0);
    }
}

 /*********************************************************************
 * @fn      PSRAM_Set_MW
 *
 * @brief   PSRAM MW ENABLE.
 *
 * @param    NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void PSRAM_Set_MW(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        PSRAM->ADDR|=(1<<1);
    }
    else
    {
        PSRAM->ADDR&=~(1<<1);
    }
}

 /*********************************************************************
 * @fn      PSRAM_Set_CMD
 *
 * @brief   PSRAM set CMD.
 *
 * @param    CMD - cmd
 *
 * @return  none
 */
void PSRAM_Set_CMD(uint8_t CMD)
{
    PSRAM->ADDR&=~(0xFF<<8);
    PSRAM->ADDR|=(CMD<<8);
}

 /*********************************************************************
 * @fn      PSRAM_Set_MR_ADDR 
 *
 * @brief   PSRAM set Mode reg address.
 *
 * @param    CMD - cmd
 *
 * @return  none
 */
void PSRAM_Set_MR_ADDR(uint8_t ADDR)
{
    PSRAM->ADDR&=~(0xF<<16);
    PSRAM->ADDR|=(ADDR<<16);
}

/*********************************************************************
 * @fn      PSRAMDMARead
 *
 * @brief   PSRAM write data by DMA 
 *
 * @param   pBuffer - data
 *          WrReAddr - PSRAM address.
 *          dataWidth-  Data bit width
 *          PSRAMDMAStruct-pointer to a PSRAMDMATypeDef
 *        structure that contains the configuration information for the PSRAM DMA. 
 *
 * @return  none
 */
void PSRAMDMASet(uint32_t *pBuffer,uint32_t WrReAddr,PSRAMDMATypeDef *PSRAMDMAStruct)
{
    PSRAM->DMA_MEM_ADDR =(volatile u32)pBuffer;
    PSRAM->DMA_PSRAM_ADDR=(volatile u32)(WrReAddr);
    PSRAM->DATA_NUM=PSRAMDMAStruct->PSRAM_DMA_DATA_NUM;
    PSRAM->DMACR=((PSRAMDMAStruct->PSRAM_DMA_DAT_DIR)<<4)|(PSRAMDMAStruct->PSRAM_DMA_MEMORY_SIZE)|((PSRAMDMAStruct->PSRAM_DMA_DMA_ONEBL)<<16)|((PSRAMDMAStruct->PSRAM_DMA_TOWBL_TOUT)<<20);
}

/*********************************************************************
 * @fn      ClearFifoMode
 *
 * @brief   PSRAM clear FIFO mode selection 
 *
 * @param   CFIFO - PSRAM_CFIFO_BTWWRRD :The read FIFO is cleared only when the written address matches the address in the read buffer
 *                  PSRAM_CFIFO_WRADD ¡êowriting to any PSRAM address clears the read FIFO 
 *
 * @return  none
 */
void ClearFifoMode(u8 CFIFO)
{
    PSRAM->CTLR&=~(1<<4);
    PSRAM->CTLR|=CFIFO;
}

/*********************************************************************
 * @fn      PSRAM_DMA_Cmd
 *
 * @brief   PSRAM DMA ENABLE.
 *
 * @param    NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void PSRAM_DMA_Cmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        PSRAM->DMACR|=(1<<0);
    }
    else
    {
        PSRAM->DMACR&=~(1<<0);
    }
}
/*********************************************************************
 * @fn      PSRAM_DMA_CFIFO
 *
 * @brief   PSRAM DMA Clear FIFO.
 *
 * @param    NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void PSRAM_DMA_CFIFO(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        PSRAM->CTLR|=(1<<5);
    }
    else
    {
        PSRAM->CECFG&=~(1<<5);
    }
}

/*********************************************************************
 * @fn      PSRAM_DMA_DIR
 *
 * @brief   PSRAM DMA DIR select.
 *
 * @param    DIR-DMA_DIR_PSRAM:MEM to PSRAM
                 DMA_DIR_MEM:PSRAM to MEM
 *
 * @return  none
 */
void PSRAM_DMA_DIR(uint32_t DIR)
{
    if(DIR==DMA_DIR_PSRAM)
    {
        PSRAM->DMACR|=(1<<4);
    }
    else
    {
        PSRAM->DMACR&=~(1<<4);
    }
}

/*********************************************************************
 * @fn      PSRAM_CEPW_Cmd
 *
 * @brief   PSRAM CEPW ENABLE.
 *
 * @param    NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void PSRAM_CEPW_Cmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        PSRAM->CECFG|=(1<<16);
    }
    else
    {
        PSRAM->CECFG&=~(1<<16);
    }
}

/*********************************************************************
 * @fn      PSRAM_DMA_Cmd
 *
 * @brief   PSRAM DMA ENABLE.
 *
 * @param    NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void PSRAM_Exti_LP(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        PSRAM->CTLR|=(1<<16);
    }
    else
    {
        PSRAM->CTLR&=~(1<<16);
    }
}

/*********************************************************************
 * @fn      PSRAM_GetFlagStatus
 *
 * @brief   Checks whether the specified state machine flag is set or not.
 *
 * @param   PSRAM_GetFlagStatus - specifies the flag to check.
                    Idle_state:Idle state
                    SdAdd_state:Send Command Address state        
                    WrReg_state:Write Register state        
                    ReReg_state:Read Register state        
                    WrDelay_state:Write Delay state       
                    WrDat_state:Write Data state        
                    RdDelay_state:Read Delay state        
                    WtCEM_Completion:Wait for Shortest CEM Completion state   
 *
 * @return  ITStatus - SET or RESET
 */
FlagStatus PSRAM_GetFlagStatus(uint32_t PSRAM_FLAG)
{
    FlagStatus bitstatus = RESET;

    if((PSRAM->STATUS & PSRAM_FLAG) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }
    return bitstatus;
}

/*********************************************************************
 * @fn      PSRAM_ITConfig
 *
 * @brief   Enables or disables the PSRAM interrupts.
 *
 * @param   PSRAM_IT - specifies the SDIO interrupt sources to be enabled or disabled.
            PSRAM_DMATEN:  DMA bus transfer complete         
            PSRAM_CENEIEN: Generate CEM error interrupt;                          
            PSRAM_AEIEN:Address error interrupt flag, the address exceeds the size of MEM capacity       
            NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void PSRAM_ITConfig(uint32_t PSRAM_IT, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
       PSRAM->IER |= PSRAM_IT;
    }
    else
    {
        PSRAM->IER &= ~PSRAM_IT;
    }
}

/*********************************************************************
 * @fn      PSRAM_GetTranFlagStatus
 *
 * @brief   Checks whether the specified PSRAM transmission flag is set or not.
 *
 * @param   PSRAM_TranFLAG - specifies the flag to check.
            PSRAM_DMATF:  DMA bus transfer complete         
            PSRAM_CENEIF: Generate CEM error interrupt;                          
            PSRAM_AEIF:Address error interrupt flag, the address exceeds the size of MEM capacity                                
 *
 * @return  ITStatus - SET or RESET
 */
FlagStatus PSRAM_GetTranFlagStatus(uint32_t PSRAM_TranFLAG)
{
    FlagStatus bitstatus = RESET;

    if((PSRAM->ISR & PSRAM_TranFLAG) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }
    return bitstatus;
}

/*********************************************************************
 * @fn      PSRAM_ClearITPendingBit
 *
 * @brief   Clears the PSRAM's pending flags.
 *
 * @param   PSRAM_IT - specifies the flag to clear.
            PSRAM_DMATF:  DMA bus transfer complete         
            PSRAM_CENEIF: Generate CEM error interrupt;                          
 * @return  none
 */
void PSRAM_ClearITPendingBit(uint32_t PSRAM_IT)
{
    PSRAM->ISR = PSRAM_IT;
}

/*********************************************************************
 * @fn      PSRAM_SyncBur_write_Cmd_Set
 *
 * @brief   Set Sync write (linear burst) command.
 *
 * @param     CMD-command

 * @return  none
 */
void PSRAM_SyncBur_write_Cmd_Set(uint8_t CMD)
{
    PSRAM->CMD0_CFG&=~(0xFF<<24);
    PSRAM->CMD0_CFG|=CMD<<24;
}

/*********************************************************************
 * @fn      PSRAM_SyncBur_Read_Cmd_Set
 *
 * @brief   Set Sync Read (linear burst) command.
 *
 * @param     CMD-command

 * @return  none
 */
void PSRAM_SyncBur_Read_Cmd_Set(uint8_t CMD)
{
    PSRAM->CMD0_CFG&=~(0xFF<<16);
    PSRAM->CMD0_CFG|=CMD<<16;
}

/*********************************************************************
 * @fn      PSRAM_Sync_write_Cmd_Set
 *
 * @brief   Set Sync write command.
 *
 * @param     CMD-command

 * @return  none
 */
void PSRAM_Sync_write_Cmd_Set(uint8_t CMD)
{
    PSRAM->CMD0_CFG&=~(0xFF<<8);
    PSRAM->CMD0_CFG|=CMD<<8;
}

/*********************************************************************
 * @fn      PSRAM_Sync_Read_Cmd_Set
 *
 * @brief   Set Sync Read (linear burst) command.
 *
 * @param     CMD-command

 * @return  none
 */
void PSRAM_Sync_Read_Cmd_Set(uint8_t CMD)
{
    PSRAM->CMD0_CFG&=~(0xFF<<0);
    PSRAM->CMD0_CFG|=CMD<<0;
}

/*********************************************************************
 * @fn      PSRAM_Refresh_Cmd_Set
 *
 * @brief   Set Refresh command.
 *
 * @param     CMD-command

 * @return  none
 */
void PSRAM_Refresh_Cmd_Set(uint8_t CMD)
{
    PSRAM->CMD1_CFG&=~(0xFF<<24);
    PSRAM->CMD1_CFG|=CMD<<24;
}

/*********************************************************************
 * @fn      PSRAM_Global_RST_Cmd_Set
 *
 * @brief   Set Global_RST command.
 *
 * @param     CMD-command

 * @return  none
 */
void PSRAM_Global_RST_Cmd_Set(uint8_t CMD)
{
    PSRAM->CMD1_CFG&=~(0xFF<<16);
    PSRAM->CMD1_CFG|=CMD<<16;
}

/*********************************************************************
 * @fn      PSRAM_WRReg_Cmd_Set
 *
 * @brief   Set Register write command.
 *
 * @param     CMD-command

 * @return  none
 */
void PSRAM_WRReg_Cmd_Set(uint8_t CMD)
{
    PSRAM->CMD1_CFG&=~(0xFF<<8);
    PSRAM->CMD1_CFG|=CMD<<8;
}

/*********************************************************************
 * @fn      PSRAM_RDReg_Cmd_Set
 *
 * @brief   Set Read write command.
 *
 * @param    CMD-command

 * @return  none
 */
void PSRAM_RDReg_Cmd_Set(uint8_t CMD)
{
    PSRAM->CMD1_CFG&=~(0xFF<<0);
    PSRAM->CMD1_CFG|=CMD<<0;
}
