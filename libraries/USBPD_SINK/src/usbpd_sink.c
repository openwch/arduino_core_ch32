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

FixedSourceCap_t fixedSourceCap[USBPD_MAX_SOURCE_CAPABILITIES];
PPSSourceCap_t   ppsSourceCap[USBPD_MAX_SOURCE_CAPABILITIES];
__attribute__ ((aligned(4))) uint8_t usbpdRxBuffer[34];
__attribute__ ((aligned(4))) uint8_t usbpdTxBuffer[34];
__attribute__ ((aligned(4))) uint8_t storageSourceCap[28];

static USBPD_Capability_t sourceCapabilities[USBPD_MAX_SOURCE_CAPABILITIES];
static volatile uint8_t sourceCapabilityCount;
static volatile uint8_t sourceCapabilitiesReady;
static volatile uint8_t apiManaged;
static volatile uint8_t requestPending;
static volatile uint8_t requestByVoltage;
static volatile uint8_t requestedCapability;
static volatile uint16_t requestedVoltage;
static volatile uint16_t requestedCurrent;
static volatile uint16_t negotiatedVoltage;
static volatile uint16_t negotiatedCurrent;
static volatile USBPD_Status_t apiStatus = USBPD_STATUS_NO_SOURCE;
static volatile uint8_t connected;
static volatile uint16_t negotiationWait;
static volatile uint16_t sourceWait;

static uint16_t usbpd_request_current_for(const USBPD_Capability_t *capability)
{
    uint16_t step = capability->programmable ? 50 : 10;
    return ((requestedCurrent + step - 1) / step) * step;
}

static void usbpd_start_requested_contract(void)
{
    uint8_t selected = 0xff;
    uint8_t voltageMatch = 0;
    uint8_t currentMatch = 0;

    if (!requestPending || !sourceCapabilitiesReady)
        return;

    if (requestByVoltage)
    {
        for (uint8_t i = 0; i < sourceCapabilityCount; ++i)
        {
            USBPD_Capability_t *cap = &sourceCapabilities[i];
            if (!cap->available || requestedVoltage < cap->minVoltage ||
                requestedVoltage > cap->maxVoltage)
                continue;

            voltageMatch = 1;
            if (usbpd_request_current_for(cap) > cap->current)
                continue;

            currentMatch = 1;
            if (selected == 0xff ||
                cap->current < sourceCapabilities[selected].current)
                selected = i;
        }
    }
    else if (requestedCapability < sourceCapabilityCount &&
             sourceCapabilities[requestedCapability].available)
    {
        USBPD_Capability_t *cap = &sourceCapabilities[requestedCapability];
        voltageMatch = 1;
        if (usbpd_request_current_for(cap) <= cap->current)
        {
            currentMatch = 1;
            selected = requestedCapability;
            if (cap->programmable)
                requestedVoltage = cap->minVoltage;
            else
                requestedVoltage = cap->maxVoltage;
        }
    }

    if (selected == 0xff)
    {
        apiStatus = !voltageMatch ? USBPD_STATUS_VOLTAGE_UNAVAILABLE :
                    (!currentMatch ? USBPD_STATUS_CURRENT_UNAVAILABLE :
                     USBPD_STATUS_ERROR);
        requestPending = 0;
        pdControl_g.cc_State = CC_HAVE_CAPABILITIES;
        return;
    }

    pdControl_g.cc_SetPDONum = selected + 1;
    pdControl_g.cc_USBPD_READY = 0;
    apiStatus = USBPD_STATUS_NEGOTIATING;
    negotiationWait = 0;
    requestPending = 0;
    pdControl_g.cc_State = CC_SEND_REQUEST;
}

static void usbpd_update_source_capabilities(void)
{
    memset(sourceCapabilities, 0, sizeof(sourceCapabilities));
    usbpd_decode_source_capabilities(storageSourceCap,
                                     pdControl_g.cc_SourcePDONum,
                                     fixedSourceCap, ppsSourceCap);

    sourceCapabilityCount = pdControl_g.cc_SourcePDONum;
    if (sourceCapabilityCount > USBPD_MAX_SOURCE_CAPABILITIES)
        sourceCapabilityCount = USBPD_MAX_SOURCE_CAPABILITIES;

    for (uint8_t i = 0; i < sourceCapabilityCount; ++i)
    {
        if (fixedSourceCap[i].Voltage)
        {
            sourceCapabilities[i].minVoltage = fixedSourceCap[i].Voltage;
            sourceCapabilities[i].maxVoltage = fixedSourceCap[i].Voltage;
            sourceCapabilities[i].current = fixedSourceCap[i].Current;
            sourceCapabilities[i].available = 1;
        }
        else if (ppsSourceCap[i].MaxVoltage)
        {
            sourceCapabilities[i].minVoltage = ppsSourceCap[i].MinVoltage;
            sourceCapabilities[i].maxVoltage = ppsSourceCap[i].MaxVoltage;
            sourceCapabilities[i].current = ppsSourceCap[i].Current;
            sourceCapabilities[i].programmable = 1;
            sourceCapabilities[i].available = 1;
        }
    }

    sourceCapabilitiesReady = 1;
}

void usbpd_api_begin(void)
{
    apiManaged = 1;
    usbpd_sink_init();
}

bool usbpd_api_request_voltage(uint16_t voltage, uint16_t current)
{
    if (voltage < 5 || voltage > 21 || current == 0)
        return false;

    requestedVoltage = voltage * 1000u;
    requestedCurrent = current;
    requestByVoltage = 1;
    requestPending = 1;
    apiStatus = USBPD_STATUS_NEGOTIATING;
    pdControl_g.cc_USBPD_READY = 0;
    usbpd_start_requested_contract();
    return true;
}

bool usbpd_api_request_capability(uint8_t index, uint16_t current)
{
    if (index >= USBPD_MAX_SOURCE_CAPABILITIES || current == 0)
        return false;
    if (sourceCapabilitiesReady && index >= sourceCapabilityCount)
        return false;

    requestedCapability = index;
    requestedCurrent = current;
    requestByVoltage = 0;
    requestPending = 1;
    apiStatus = USBPD_STATUS_NEGOTIATING;
    pdControl_g.cc_USBPD_READY = 0;
    usbpd_start_requested_contract();
    return true;
}

bool usbpd_api_is_ready(void)
{
    return apiStatus == USBPD_STATUS_READY;
}

USBPD_Status_t usbpd_api_get_status(void)
{
    return apiStatus;
}

uint16_t usbpd_api_get_voltage(void)
{
    return apiStatus == USBPD_STATUS_READY ? negotiatedVoltage : 0;
}

uint16_t usbpd_api_get_current(void)
{
    return apiStatus == USBPD_STATUS_READY ? negotiatedCurrent : 0;
}

bool usbpd_api_has_capabilities(void)
{
    return sourceCapabilitiesReady != 0;
}

uint8_t usbpd_api_get_capability_count(void)
{
    return sourceCapabilitiesReady ? sourceCapabilityCount : 0;
}

bool usbpd_api_get_capability(uint8_t index, USBPD_Capability_t *capability)
{
    if (!capability || !sourceCapabilitiesReady ||
        index >= sourceCapabilityCount || !sourceCapabilities[index].available)
        return false;

    *capability = sourceCapabilities[index];
    return true;
}


uint8_t usbpd_sink_get_ready(void)
{
    return pdControl_g.cc_USBPD_READY;
}

void usbpd_sink_clear_ready(void)
{
    pdControl_g.cc_USBPD_READY = 0;
}

bool usbpd_sink_set_request_fixed_voltage(Request_voltage_t requestVoltage)
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
    if (!sourceCapabilitiesReady)
        return false;
    for(uint8_t i=0; i<pdControl_g.cc_SourcePDONum; i++)
    {
        if(pdControl_g.cc_FixedSourceCap[i].Voltage == targetVoltage)
        { 
            if (pdControl_g.cc_SetPDONum == i + 1 &&
                apiStatus == USBPD_STATUS_READY)
                return true;
            pdControl_g.cc_SetPDONum = i+1;
            requestedCurrent = pdControl_g.cc_FixedSourceCap[i].Current;
            requestedVoltage = targetVoltage;
            requestByVoltage = 0;
            requestPending = 1;
            usbpd_start_requested_contract();
            return true;
        }
    }
    
    // unsupported voltage
    return false;
}

void timer3_init(uint16_t arr, uint16_t psc)
{
#if defined(TIM_MODULE_ENABLED)
    // Init of HardwareTimer is done in main sketch
	(void)arr;
	(void)psc;
#else
    // keep original bypass of CH32 core API
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
#endif // defined(TIM_MODULE_ENABLED)
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
    memset(fixedSourceCap, 0, sizeof(fixedSourceCap));
    memset(ppsSourceCap, 0, sizeof(ppsSourceCap));
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
    negotiationWait = 0;
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
    usbpd_decode_source_capabilities(pdoData, pdControl->cc_SourcePDONum,
                                     pdControl->cc_FixedSourceCap,
                                     pdControl->cc_PPSSourceCap);
}

void usbpd_sink_fixed_pdo_request(FixedSourceCap_t* sourceCap, uint8_t pdoNum, pd_control_t* pdControl, uint8_t* txData)
{
    USBPD_SINKRDO_t sinkFixedRDO;
    USBPD_MessageHeader_t messageHeader;
    messageHeader.d16 = 0u;
    sinkFixedRDO.d32 = 0u;

    messageHeader.MessageHeader.MessageID = pdControl->cc_SinkMessageID ;
    messageHeader.MessageHeader.MessageType = USBPD_DATA_MSG_REQUEST;
    messageHeader.MessageHeader.NumberOfDataObjects = 1u;
    messageHeader.MessageHeader.SpecificationRevision = pdControl->cc_PD_Version;

    uint16_t current = usbpd_request_current_for(&sourceCapabilities[pdoNum - 1]) / 10;
    if (current > sourceCap[pdoNum-1].Current / 10)
        current = sourceCap[pdoNum-1].Current / 10;
    sinkFixedRDO.SinkFixedVariableRDO.MaxOperatingCurrent10mAunits = current;
    sinkFixedRDO.SinkFixedVariableRDO.OperatingCurrentIn10mAunits = current;
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

    messageHeader.MessageHeader.MessageID = pdControl->cc_SinkMessageID;
    messageHeader.MessageHeader.MessageType = USBPD_DATA_MSG_REQUEST;
    messageHeader.MessageHeader.NumberOfDataObjects = 1u;
    messageHeader.MessageHeader.SpecificationRevision = pdControl->cc_PD_Version;

    sinkPPSRDO.SinkPPSRDO.ObjectPosition = pdoNum;
    sinkPPSRDO.SinkPPSRDO.OutputVoltagein20mVunits = voltage/20;
    uint16_t current = usbpd_request_current_for(&sourceCapabilities[pdoNum - 1]) / 50;
    if (current > sourceCap[pdoNum-1].Current / 50)
        current = sourceCap[pdoNum-1].Current / 50;
    sinkPPSRDO.SinkPPSRDO.OperatingCurrentIN50mAuints = current;
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
                usbpd_update_source_capabilities();
                NVIC_EnableIRQ( USBPD_IRQn );

                pdControl_g.cc_State = CC_HAVE_CAPABILITIES;
                if (!apiManaged && !requestPending)
                {
                    requestedCapability = 0;
                    requestByVoltage = 0;
                    requestedCurrent = sourceCapabilities[0].current;
                    requestPending = 1;
                }
                usbpd_start_requested_contract();
            }
            break;
        }

        case CC_SEND_REQUEST:
        {
  
            if(pdControl_g.cc_LastState != pdControl_g.cc_State)
            {
                // Delay_Ms(2);
                // printf("request\r\n");

                uint8_t index = pdControl_g.cc_SetPDONum - 1;
                if (sourceCapabilities[index].programmable)
                    usbpd_sink_pps_pdo_request(ppsSourceCap,
                                               pdControl_g.cc_SetPDONum,
                                               requestedVoltage, &pdControl_g,
                                               usbpdTxBuffer);
                else
                    usbpd_sink_fixed_pdo_request(fixedSourceCap,
                                                 pdControl_g.cc_SetPDONum,
                                                 &pdControl_g, usbpdTxBuffer);

                usbpd_sink_phy_send_data(usbpdTxBuffer, 6, USBPD_SOP0);
                negotiationWait = 0;
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
                uint8_t index = pdControl_g.cc_SetPDONum - 1;
                negotiatedVoltage = sourceCapabilities[index].programmable ?
                                    requestedVoltage :
                                    sourceCapabilities[index].maxVoltage;
                negotiatedCurrent =
                    usbpd_request_current_for(&sourceCapabilities[index]);
                apiStatus = USBPD_STATUS_READY;
                pdControl_g.cc_USBPD_READY = 1;
                pdControl_g.cc_State = CC_ACTIVE;
            }
            break;
        }

        case CC_HAVE_CAPABILITIES:
        {
            usbpd_start_requested_contract();
            break;
        }

        case CC_ACTIVE:
        {
            break;
        }

        case CC_REQUEST_FAILED:
            break;

        case CC_WAIT_ACCEPT:
            break;

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
                    if (pdControl->cc_State == CC_SEND_REQUEST ||
                        pdControl->cc_State == CC_WAIT_ACCEPT)
                    {
                        pdControl->cc_SourceGoodCRCOver = 0;
                        pdControl->cc_State = CC_WAIT_PS_RDY;
                    }
                    // debug_log("accept\r\n");
                    break;
                }
                case USBPD_CONTROL_MSG_REJECT:
                {
                    if (pdControl->cc_State == CC_SEND_REQUEST ||
                        pdControl->cc_State == CC_WAIT_ACCEPT)
                    {
                        pdControl->cc_SourceGoodCRCOver = 0;
                        pdControl->cc_USBPD_READY = 0;
                        apiStatus = USBPD_STATUS_REJECTED;
                        pdControl->cc_State = CC_REQUEST_FAILED;
                    }
                    break;
                }
                case USBPD_CONTROL_MSG_WAIT:
                {
                    if (pdControl->cc_State == CC_SEND_REQUEST ||
                        pdControl->cc_State == CC_WAIT_ACCEPT)
                    {
                        pdControl->cc_SourceGoodCRCOver = 0;
                        pdControl->cc_USBPD_READY = 0;
                        apiStatus = USBPD_STATUS_ERROR;
                        pdControl->cc_State = CC_REQUEST_FAILED;
                    }
                    break;
                }
                case USBPD_CONTROL_MSG_PS_RDY:
                {
                    if (pdControl->cc_State == CC_WAIT_PS_RDY)
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

#if defined(TIM_MODULE_ENABLED)
    // Use of HardwareTimer in main sketch. All we have here is the callback that does all things needed
void USBPD_Timer_Callback(void)
#else
    #warning TIM_MODULE_ENABLED is not defined. Please enable HardwareTimer module.
    // keep original bypass of CH32 core API
void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM3_IRQHandler(void)
#endif // defined(TIM_MODULE_ENABLED)
{ 
#if defined(TIM_MODULE_ENABLED)
    // C++ HardwareTimer management is in main sketch; it calls timer to resume after handling this
#else
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
#endif // defined(TIM_MODULE_ENABLED)

    uint8_t ccLine = usbpd_sink_check_cc_connect();

    pdControl_g.cc_WaitTime++;

    if(pdControl_g.cc_State == CC_CHECK_CONNECT) // check connect
    {
        if(ccLine == USBPD_CC1)
        {
            sourceWait = 0;
            pdControl_g.cc2_ConnectTimes = 0;
            pdControl_g.cc1_ConnectTimes++;
            if(pdControl_g.cc1_ConnectTimes > 5)
            {
                pdControl_g.cc1_ConnectTimes = 0;
                pdControl_g.cc_State = CC_CONNECT;
                connected = 1;
                apiStatus = requestPending ? USBPD_STATUS_NEGOTIATING :
                            USBPD_STATUS_IDLE;
                USBPD->CONFIG &= ~CC_SEL; // select cc1 line to communication
                // debug_log("cc1 connect\r\n");
            }
        }
        else if(ccLine == USBPD_CC2)
        {
            sourceWait = 0;
            pdControl_g.cc1_ConnectTimes = 0;
            pdControl_g.cc2_ConnectTimes++;
            if(pdControl_g.cc2_ConnectTimes > 5)
            {
                pdControl_g.cc2_ConnectTimes = 0;
                pdControl_g.cc_State = CC_CONNECT;
                connected = 1;
                apiStatus = requestPending ? USBPD_STATUS_NEGOTIATING :
                            USBPD_STATUS_IDLE;
                USBPD->CONFIG |= CC_SEL; // select cc2 line to communication
                // debug_log("cc2 connect\r\n");
            }
        }
        else
        {
            pdControl_g.cc1_ConnectTimes = 0;
            pdControl_g.cc2_ConnectTimes = 0;
            if (sourceWait < 400)
                ++sourceWait;
            if (sourceWait >= 400 && requestPending)
                apiStatus = USBPD_STATUS_NO_SOURCE;
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
                pdControl_g.cc_USBPD_READY = 0;
                sourceCapabilitiesReady = 0;
                sourceCapabilityCount = 0;
                negotiatedVoltage = 0;
                negotiatedCurrent = 0;
                apiStatus = connected ? USBPD_STATUS_DISCONNECTED :
                            USBPD_STATUS_NO_SOURCE;
                connected = 0;
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

    if (pdControl_g.cc_State == CC_SEND_REQUEST ||
        pdControl_g.cc_State == CC_WAIT_ACCEPT ||
        pdControl_g.cc_State == CC_WAIT_PS_RDY ||
        pdControl_g.cc_State == CC_PS_RDY)
    {
        if (++negotiationWait > 400)
        {
            pdControl_g.cc_USBPD_READY = 0;
            apiStatus = USBPD_STATUS_ERROR;
            pdControl_g.cc_State = CC_REQUEST_FAILED;
        }
    }

    usbpd_sink_process();
}
