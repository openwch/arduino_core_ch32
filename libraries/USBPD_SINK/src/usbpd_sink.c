#include "usbpd_sink.h"
#include "debug.h"
#include <string.h>

void my_delayus(uint16_t us)
{
    for(uint16_t i=0; i<us; i++)
    {
        for(uint8_t j=0; j<10; j++)
        {
            __asm volatile ("nop");
        }
    }
}

static pd_control_t  pdControl_g =
{
    .cc_State = CC_IDLE,
    .cc1_ConnectTimes = 0,
    .cc2_ConnectTimes = 0,
};

FixedSourceCap_t fixedSourceCap[5];
PPSSourceCap_t   ppsSourceCap[2];


__attribute__ ((aligned(4))) uint8_t usbpdRxBuffer[34];       // PD receive buffer 
__attribute__ ((aligned(4))) uint8_t usbpdTxBuffer[34] ;       // PD send buffer 
__attribute__ ((aligned(4))) uint8_t storageSourceCap[28];    

uint8_t usbpd_sink_get_ready(void)
{
    return pdControl_g.cc_USBPD_READY;
}

void usbpd_sink_clear_ready(void)
{
    pdControl_g.cc_USBPD_READY = 0;
}

void usbpd_sink_set_request_fixed_voltage(Request_voltage_t requestVoltage)
{
    uint16_t targetVoltage;
    switch (requestVoltage)
    {
        case REQUEST_5v:
            targetVoltage = 5000;
            break;

        case REQUEST_9v:
            targetVoltage = 9000;
            break;

        case REQUEST_12v:
            targetVoltage = 12000;
            break;

        case REQUEST_15v:
            targetVoltage = 15000;
            break;

        case REQUEST_20v:
            targetVoltage = 20000;
            break;
        
        default:
            targetVoltage = 5000;
            break;
    }
    // pdControl_g.cc_LastSetPDONum = pdControl_g.cc_SetPDONum;
    for(uint8_t i=0; i<5; i++)
    {
        if(pdControl_g.cc_FixedSourceCap[i].Voltage == targetVoltage)
        { 
            pdControl_g.cc_SetPDONum = i+1;
            return;
        }
    }
    pdControl_g.cc_SetPDONum = (pdControl_g.cc_SourcePDONum - pdControl_g.cc_SourcePPSNum);
    
}

void timer3_init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit( TIM3, &TIM_TimeBaseInitStructure);

    TIM_ClearITPendingBit( TIM3, TIM_IT_Update );

    TIM_ITConfig( TIM3, TIM_IT_Update , ENABLE );
    NVIC_SetPriority(TIM3_IRQn, 0x10);
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM_Cmd( TIM3, ENABLE );
}


void usbpd_sink_rx_mode(void)
{
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN ;
    USBPD->DMA = (uint32_t)usbpdRxBuffer;
    USBPD->CONTROL &= ~PD_TX_EN;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
    USBPD->CONTROL |= BMC_START ;
    // NVIC_EnableIRQ( USBPD_IRQn );  
}

void usbpd_sink_reset(void)
{
    USBPD->PORT_CC1 = CC_CMP_66;
    USBPD->PORT_CC2 = CC_CMP_66;
    pdControl_g.cc1_ConnectTimes = 0;
    pdControl_g.cc2_ConnectTimes = 0;
    pdControl_g.cc_NoneTimes = 0;
    pdControl_g.cc_SourcePDONum = 0;
    pdControl_g.cc_SourcePPSNum = 0;
    pdControl_g.cc_FixedSourceCap = fixedSourceCap;
    pdControl_g.cc_PPSSourceCap = ppsSourceCap;
    pdControl_g.cc_State = CC_IDLE;
    pdControl_g.cc_LastState = CC_IDLE;
    pdControl_g.cc_SinkMessageID = 0;
    pdControl_g.cc_SinkGoodCRCOver = 0;
    pdControl_g.cc_SourceGoodCRCOver = 0;
    pdControl_g.cc_PD_Version = DEF_PD_REVISION_20;
    pdControl_g.cc_USBPD_READY = 0;
    pdControl_g.cc_SetPDONum = 1;
    pdControl_g.cc_LastSetPDONum = 1;
}

void usbpd_sink_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);               //enable PD I/O clock, AFIO clock and PD clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;

    USBPD->CONFIG = PD_DMA_EN;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;// Clear all interrupt flags


    timer3_init(5000-1,48-1); // 5ms

}

void usbpd_sink_phy_send_data(uint8_t* pBuf, uint8_t length, uint8_t sop)
{
    // pdControl_g.cc_SourceGoodCRCOver = 0;
    USBPD->CONFIG |= IE_TX_END ;

    if ((USBPD->CONFIG & CC_SEL) == CC_SEL )
    {
        USBPD->PORT_CC2 |= CC_LV0;
    }
    else
    {
        USBPD->PORT_CC1 |= CC_LV0;
    }

    USBPD->BMC_CLK_CNT = UPD_TMR_TX_48M;

    USBPD->DMA = (uint32_t)usbpdTxBuffer;                         //dma address

    USBPD->TX_SEL = sop;
    USBPD->BMC_TX_SZ = length;                                     //length
    USBPD->CONTROL |= PD_TX_EN ;                                //tx mode
    USBPD->STATUS &= BMC_AUX_INVALID;                           //BMC_AUX clear 0
    USBPD->CONTROL |= BMC_START ;                               //BMC_START
}


uint8_t usbpd_sink_check_cc_connect(void)
{
    uint8_t ccLine = USBPD_CCNONE;

    USBPD->PORT_CC1 &= ~( CC_CE | PA_CC_AI );
    USBPD->PORT_CC1 |= CC_CMP_22;
    // Delay_Us(2);
    if( USBPD->PORT_CC1 & PA_CC_AI )
    {
        ccLine = USBPD_CC1;
    }


    USBPD->PORT_CC2 &= ~( CC_CE | PA_CC_AI );
    USBPD->PORT_CC2 |= CC_CMP_22;
    // Delay_Us(2);
    if( USBPD->PORT_CC2 & PA_CC_AI )
    {
        ccLine = USBPD_CC2;
    }

    return ccLine;
}

void usbpd_sink_pdo_analyse(uint8_t* pdoData, pd_control_t* pdControl)
{
    USBPD_PDO_t test;
    pdControl->cc_SourcePPSNum = 0;

    for(uint8_t i=0; i<pdControl->cc_SourcePDONum; i++)
    { 
        test.d32 = *(uint32_t*)(&pdoData[i*4]);
        if((test.SourcePPSPDO.AugmentedPowerDataObject==3u) && (test.SourcePPSPDO.SPRprogrammablePowerSupply==0)) //PPS
        {
            pdControl->cc_PPSSourceCap[pdControl->cc_SourcePPSNum].MaxVoltage = POWER_DECODE_100MV(test.SourcePPSPDO.MaxVlotageIn100mVincrements);
            pdControl->cc_PPSSourceCap[pdControl->cc_SourcePPSNum].MinVoltage = POWER_DECODE_100MV(test.SourcePPSPDO.MinVlotageIn100mVincrements);
            pdControl->cc_PPSSourceCap[pdControl->cc_SourcePPSNum].Current = POWER_DECODE_50MA(test.SourcePPSPDO.MaxCurrentIn50mAincrements);


            pdControl->cc_SourcePPSNum++;
        }
        else // fixed
        {
            pdControl->cc_FixedSourceCap[i].Current = POWER_DECODE_10MA(test.SourceFixedPDO.MaxCurrentIn10mAunits);
            pdControl->cc_FixedSourceCap[i].Voltage = POWER_DECODE_50MV(test.SourceFixedPDO.VolatageIn50mVunits);
        }
        
    }
}

void usbpd_sink_fixed_pdo_request(FixedSourceCap_t* sourceCap, uint8_t pdoNum, pd_control_t* pdControl, uint8_t* txData)
{
    USBPD_SINKRDO_t sinkFixedRDO;
    USBPD_MessageHeader_t messageHeader;
    messageHeader.d16 = 0u;
    sinkFixedRDO.d32 = 0u;

    if(pdoNum > (pdControl->cc_SourcePDONum - pdControl->cc_SourcePPSNum))
    {
        pdoNum = (pdControl->cc_SourcePDONum - pdControl->cc_SourcePPSNum);
    }

    messageHeader.MessageHeader.MessageID = pdControl->cc_SinkMessageID ;
    messageHeader.MessageHeader.MessageType = USBPD_DATA_MSG_REQUEST;
    messageHeader.MessageHeader.NumberOfDataObjects = 1u;
    messageHeader.MessageHeader.SpecificationRevision = pdControl->cc_PD_Version;

    sinkFixedRDO.SinkFixedVariableRDO.MaxOperatingCurrent10mAunits = sourceCap[pdoNum-1].Current/10;
    sinkFixedRDO.SinkFixedVariableRDO.OperatingCurrentIn10mAunits = sourceCap[pdoNum-1].Current/10;
    sinkFixedRDO.SinkFixedVariableRDO.ObjectPosition = pdoNum;
    sinkFixedRDO.SinkFixedVariableRDO.USBCommunicationsCapable = 1u;
    sinkFixedRDO.SinkFixedVariableRDO.NoUSBSuspend = 1u;

    // debug_log("sinkFIx = %08lx, curren = %d\r\n",sinkFixedRDO.d32,sourceCap[pdoNum-1].Current/100);

    *(uint16_t*)&txData[0] =  messageHeader.d16;
    txData[2] = sinkFixedRDO.d32 & 0xff;
    txData[3] =(sinkFixedRDO.d32>>8) & 0xff;
    txData[4] =(sinkFixedRDO.d32>>16) & 0xff; 
    txData[5] =(sinkFixedRDO.d32>>24) & 0xff;     
}

void usbpd_sink_pps_pdo_request(PPSSourceCap_t* sourceCap, uint8_t pdoNum, uint16_t voltage, pd_control_t* pdControl, uint8_t* txData)
{
    USBPD_SINKRDO_t sinkPPSRDO;
    USBPD_MessageHeader_t messageHeader;
    messageHeader.d16 = 0u;
    sinkPPSRDO.d32 = 0u;

    if(pdoNum > pdControl->cc_SourcePDONum)
    {
        // debug_log("not support this pps \r\n");
        pdoNum = pdControl->cc_SourcePDONum - pdControl->cc_SourcePPSNum;
    }

    messageHeader.MessageHeader.MessageID = pdControl->cc_SinkMessageID;
    messageHeader.MessageHeader.MessageType = USBPD_DATA_MSG_REQUEST;
    messageHeader.MessageHeader.NumberOfDataObjects = 1u;
    messageHeader.MessageHeader.SpecificationRevision = pdControl->cc_PD_Version;

    sinkPPSRDO.SinkPPSRDO.ObjectPosition = pdoNum;
    sinkPPSRDO.SinkPPSRDO.OutputVoltagein20mVunits = voltage/20;
    sinkPPSRDO.SinkPPSRDO.OperatingCurrentIN50mAuints = sourceCap[pdoNum+pdControl->cc_SourcePPSNum-pdControl->cc_SourcePDONum - 1].Current/50;
    sinkPPSRDO.SinkPPSRDO.NoUSBSuspend = 1u;
    sinkPPSRDO.SinkPPSRDO.USBCommunicationsCapable = 1u;

    *(uint16_t*)&txData[0] =  messageHeader.d16;
    txData[2] = sinkPPSRDO.d32 & 0xff;
    txData[3] =(sinkPPSRDO.d32>>8) & 0xff;
    txData[4] =(sinkPPSRDO.d32>>16) & 0xff; 
    txData[5] =(sinkPPSRDO.d32>>24) & 0xff;  

}


void usbpd_sink_process(void)
{
    
    cc_state_t temp = pdControl_g.cc_State;
    USBPD_MessageHeader_t messageHeader;
    switch (pdControl_g.cc_State)
    {
        case CC_IDLE:
        {
            NVIC_DisableIRQ( USBPD_IRQn );  
            usbpd_sink_reset();
               
            pdControl_g.cc_State = CC_CHECK_CONNECT;
            break;
        }

        case CC_CHECK_CONNECT:
            break;

        case CC_CONNECT:
        {
            if(pdControl_g.cc_LastState != pdControl_g.cc_State)
            {
                usbpd_sink_rx_mode();
                NVIC_SetPriority(USBPD_IRQn, 0x00);
                NVIC_EnableIRQ( USBPD_IRQn );  
                // Delay_Ms(1);
            }
            break;
        }

        case CC_SOURCE_CAP:
        {
            if(pdControl_g.cc_SinkGoodCRCOver)
            {
                // printf("pdo analyse\r\n");
                pdControl_g.cc_SinkGoodCRCOver = 0;
                NVIC_DisableIRQ( USBPD_IRQn );
                usbpd_sink_pdo_analyse(storageSourceCap, &pdControl_g);
                NVIC_EnableIRQ( USBPD_IRQn );

                pdControl_g.cc_State = CC_SEND_REQUEST;
            }
            break;
        }

        case CC_SEND_REQUEST:
        {
  
            if(pdControl_g.cc_LastState != pdControl_g.cc_State)
            {
                // Delay_Ms(2);
                // printf("request\r\n");

                usbpd_sink_fixed_pdo_request(fixedSourceCap, pdControl_g.cc_SetPDONum, &pdControl_g,usbpdTxBuffer);        

                usbpd_sink_phy_send_data(usbpdTxBuffer, 6, USBPD_SOP0);
            }

            if(pdControl_g.cc_SourceGoodCRCOver)
            {
                pdControl_g.cc_SourceGoodCRCOver = 0;
                pdControl_g.cc_State = CC_WAIT_ACCEPT;
                
            }

            break;
        }

        case CC_WAIT_PS_RDY:
        {
            break;
        }

        case CC_PS_RDY:
        {
            if(pdControl_g.cc_SinkGoodCRCOver)
            {
                pdControl_g.cc_SinkGoodCRCOver = 0;
                pdControl_g.cc_State = CC_GET_SOURCE_CAP;
                pdControl_g.cc_WaitTime = 0;
                
            }
            break;
        }

        case CC_GET_SOURCE_CAP:
        {
            pdControl_g.cc_USBPD_READY = 1; 
            if(pdControl_g.cc_SetPDONum != pdControl_g.cc_LastSetPDONum)
            {
                pdControl_g.cc_LastSetPDONum = pdControl_g.cc_SetPDONum;
                pdControl_g.cc_USBPD_READY = 0; 
                // Delay_Ms(5);
                messageHeader.d16 = 0u;
                messageHeader.MessageHeader.MessageID = pdControl_g.cc_SinkMessageID;
                messageHeader.MessageHeader.MessageType = USBPD_CONTROL_MSG_GET_SRC_CAP;
                messageHeader.MessageHeader.NumberOfDataObjects = 0u;
                messageHeader.MessageHeader.SpecificationRevision = pdControl_g.cc_PD_Version;
                *(uint16_t*)&usbpdTxBuffer[0] =  messageHeader.d16;


                usbpd_sink_phy_send_data(  usbpdTxBuffer, 2, USBPD_SOP0 );

                pdControl_g.cc_State = CC_GET_SOURCE_CAP+1;
            }
            
            break;
        }

        default:
            
            break;
    }
    
    pdControl_g.cc_LastState = temp;
    
}

void usbpd_sink_protocol_analysis(USBPD_MessageHeader_t* messageHeader, pd_control_t* pdControl)
{
    uint8_t sendGoodCRCFlag = 1;
    if(messageHeader->MessageHeader.Extended == 0u)
    {
        if(messageHeader->MessageHeader.NumberOfDataObjects == 0u) // control message
        {
            switch(messageHeader->MessageHeader.MessageType)
            {
                case USBPD_CONTROL_MSG_GOODCRC:
                {
                    sendGoodCRCFlag = 0;
                    pdControl->cc_SourceGoodCRCOver = 1;
                    pdControl->cc_SinkMessageID++;
                    // debug_log("receive good crc\r\n");   
                    break;
                }
                case USBPD_CONTROL_MSG_ACCEPT:
                {
                    pdControl->cc_State = CC_WAIT_PS_RDY;
                    // debug_log("accept\r\n");
                    break;
                }
                case USBPD_CONTROL_MSG_PS_RDY:
                {
                    pdControl->cc_State = CC_PS_RDY;
                    // debug_log("ps RDY\r\n");
                    break;
                }

                default:
                    // pdControl->cc_State = CC_IDLE;
                    break;
            }
        }
        else // data message
        {
            switch (messageHeader->MessageHeader.MessageType)
            {
                case USBPD_DATA_MSG_SRC_CAP:
                {
                    pdControl->cc_State = CC_SOURCE_CAP;
                    pdControl->cc_SourcePDONum = messageHeader->MessageHeader.NumberOfDataObjects;
                    // pdControl->cc_PD_Version = messageHeader->MessageHeader.SpecificationRevision;
                    memcpy(storageSourceCap,&usbpdRxBuffer[2],28);
                    // debug_log("cc source cap\r\n");
                    
                    break;
                }

                default:
                    break;
            }
        }
    }

    if(sendGoodCRCFlag) // send goodcrd
    {
        // Delay_Us(30); // Delay 30us, answer GoodCRC
        // GPIOC->BSHR = GPIO_Pin_3; //toggle pc3
        my_delayus(20);
        // GPIOC->BCR = GPIO_Pin_3; //toggle pc3
        pdControl_g.cc_SinkGoodCRCOver = 0;

        USBPD_MessageHeader_t my_messageHeader;
        my_messageHeader.d16 = 0u;
        my_messageHeader.MessageHeader.MessageID = messageHeader->MessageHeader.MessageID;
        my_messageHeader.MessageHeader.MessageType = USBPD_CONTROL_MSG_GOODCRC;
        my_messageHeader.MessageHeader.SpecificationRevision = pdControl->cc_PD_Version;
        *(uint16_t*)&usbpdTxBuffer[0] =  my_messageHeader.d16;
        usbpd_sink_phy_send_data( usbpdTxBuffer, 2, USBPD_SOP0 );
    } 
}

void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBPD_IRQHandler(void)
{

    USBPD_MessageHeader_t messageHeader;

    if(USBPD->STATUS & IF_RX_ACT)
    {
        messageHeader.d16 = *(uint16_t*)usbpdRxBuffer;
        
        if( ( USBPD->STATUS & MASK_PD_STAT ) == PD_RX_SOP0 )
        {
            if( USBPD->BMC_BYTE_CNT >= 6 )
            {
                usbpd_sink_protocol_analysis(&messageHeader, &pdControl_g);
            }

        }
        USBPD->STATUS |= IF_RX_ACT;
    }

    if(USBPD->STATUS & IF_TX_END)// Transmission completion interrupt flag
    {
        /* Packet send completion interrupt (GoodCRC send completion interrupt only) */
        USBPD->PORT_CC1 &= ~CC_LV0;
        USBPD->PORT_CC2 &= ~CC_LV0;

        usbpd_sink_rx_mode();
        pdControl_g.cc_SinkGoodCRCOver = 1;

        USBPD->STATUS |= IF_TX_END;
    }

}

void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM3_IRQHandler(void)
{ 
    
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    uint8_t ccLine = usbpd_sink_check_cc_connect();


    pdControl_g.cc_WaitTime++;

    if(pdControl_g.cc_State == CC_CHECK_CONNECT) // check connect
    {
        if(ccLine == USBPD_CC1)
        {
            pdControl_g.cc2_ConnectTimes = 0;
            pdControl_g.cc1_ConnectTimes++;
            if(pdControl_g.cc1_ConnectTimes > 5)
            {
                pdControl_g.cc1_ConnectTimes = 0;
                pdControl_g.cc_State = CC_CONNECT;
                USBPD->CONFIG &= ~CC_SEL; // select cc1 line to communication
                // debug_log("cc1 connect\r\n");
            }
        }
        else if(ccLine == USBPD_CC2)
        {
            pdControl_g.cc1_ConnectTimes = 0;
            pdControl_g.cc2_ConnectTimes++;
            if(pdControl_g.cc2_ConnectTimes > 5)
            {
                pdControl_g.cc2_ConnectTimes = 0;
                pdControl_g.cc_State = CC_CONNECT;
                USBPD->CONFIG |= CC_SEL; // select cc2 line to communication
                // debug_log("cc2 connect\r\n");
            }
        }
        else
        {
            pdControl_g.cc1_ConnectTimes = 0;
            pdControl_g.cc2_ConnectTimes = 0;
        }
    }

    if( pdControl_g.cc_State > CC_CHECK_CONNECT) // check disconnect
    {

        if(ccLine == USBPD_CCNONE)
        {
            pdControl_g.cc_NoneTimes++;
            if(pdControl_g.cc_NoneTimes > 5)
            {
                pdControl_g.cc_NoneTimes = 0;
                pdControl_g.cc_State = CC_IDLE;
                NVIC_DisableIRQ( USBPD_IRQn );  
                // debug_log("cc  disconnect\r\n");
            }
        } 
        else
        {
            pdControl_g.cc_NoneTimes = 0;
        }       
    }

    usbpd_sink_process();
}

