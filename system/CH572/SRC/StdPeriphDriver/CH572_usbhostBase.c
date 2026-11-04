/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_usbhost.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : source file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH572_common.h"
#if DISK_LIB_ENABLE
  #include "CHRV3UFI.H"
#endif

uint8_t       UsbDevEndp0Size; // USB�豸�Ķ˵�0�������ߴ�
uint8_t       FoundNewDev;
_RootHubDev   ThisUsbDev;                  //ROOT��
_DevOnHubPort DevOnHubPort[HUB_MAX_PORTS]; // �ٶ�:������1���ⲿHUB,ÿ���ⲿHUB������HUB_MAX_PORTS���˿�(���˲���)

uint8_t *pHOST_RX_RAM_Addr;
uint8_t *pHOST_TX_RAM_Addr;

/*��ȡ�豸������*/
__attribute__((aligned(4))) const uint8_t SetupGetDevDescr[] = {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00,
                                                                USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof(USB_DEV_DESCR), 0x00};
/*��ȡ����������*/
__attribute__((aligned(4))) const uint8_t SetupGetCfgDescr[] = {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00,
                                                                USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00};
/*����USB��ַ*/
__attribute__((aligned(4))) const uint8_t SetupSetUsbAddr[] = {USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00,
                                                               0x00, 0x00, 0x00, 0x00};
/*����USB����*/
__attribute__((aligned(4))) const uint8_t SetupSetUsbConfig[] = {USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00,
                                                                 0x00, 0x00, 0x00};
/*����USB�ӿ�����*/
__attribute__((aligned(4))) const uint8_t SetupSetUsbInterface[] = {USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00,
                                                                    0x00, 0x00, 0x00, 0x00};
/*����˵�STALL*/
__attribute__((aligned(4))) const uint8_t SetupClrEndpStall[] = {USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE,
                                                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*********************************************************************
 * @fn      DisableRootHubPort
 *
 * @brief   �ر�ROOT-HUB�˿�,ʵ����Ӳ���Ѿ��Զ��ر�,�˴�ֻ�����һЩ�ṹ״̬
 *
 * @param   none
 *
 * @return  none
 */
void DisableRootHubPort(void)
{
#ifdef FOR_ROOT_UDISK_ONLY
    CHRV3DiskStatus = DISK_DISCONNECT;
#endif
#ifndef DISK_BASE_BUF_LEN
    ThisUsbDev.DeviceStatus = ROOT_DEV_DISCONNECT;
    ThisUsbDev.DeviceAddress = 0x00;
#endif
}

/*********************************************************************
 * @fn      AnalyzeRootHub
 *
 * @brief   ����ROOT-HUB״̬,����ROOT-HUB�˿ڵ��豸����¼�
 *          ����豸�γ�,�����е���DisableRootHubPort()����,���˿ڹر�,�����¼�,����Ӧ�˿ڵ�״̬λ
 *
 * @param   none
 *
 * @return  ����ERR_SUCCESSΪû�����,����ERR_USB_CONNECTΪ��⵽������,����ERR_USB_DISCONΪ��⵽�Ͽ�
 */
uint8_t AnalyzeRootHub(void)
{
    uint8_t s;

    s = ERR_SUCCESS;

    if(R8_USB_MIS_ST & RB_UMS_DEV_ATTACH)
    { // �豸����
#ifdef DISK_BASE_BUF_LEN
        if(CHRV3DiskStatus == DISK_DISCONNECT
#else
        if(ThisUsbDev.DeviceStatus == ROOT_DEV_DISCONNECT // ��⵽���豸����
#endif
           || (R8_UHOST_CTRL & RB_UH_PORT_EN) == 0x00)
        {                         // ��⵽���豸����,����δ����,˵���Ǹղ���
            DisableRootHubPort(); // �رն˿�
#ifdef DISK_BASE_BUF_LEN
            CHRV3DiskStatus = DISK_CONNECT;
#else
            ThisUsbDev.DeviceSpeed = R8_USB_MIS_ST & RB_UMS_DM_LEVEL ? 0 : 1;
            ThisUsbDev.DeviceStatus = ROOT_DEV_CONNECTED; //�����ӱ�־
#endif
            PRINT("USB dev in\n");
            s = ERR_USB_CONNECT;
        }
    }

#ifdef DISK_BASE_BUF_LEN
    else if(CHRV3DiskStatus >= DISK_CONNECT)
    {
#else
    else if(ThisUsbDev.DeviceStatus >= ROOT_DEV_CONNECTED)
    { //��⵽�豸�γ�
#endif
        DisableRootHubPort(); // �رն˿�
        PRINT("USB dev out\n");
        if(s == ERR_SUCCESS)
        {
            s = ERR_USB_DISCON;
        }
    }
    //	R8_USB_INT_FG = RB_UIF_DETECT;                                                  // ���жϱ�־
    return (s);
}

/*********************************************************************
 * @fn      SetHostUsbAddr
 *
 * @brief   ����USB������ǰ������USB�豸��ַ
 *
 * @param   addr    - USB�豸��ַ
 *
 * @return  none
 */
void SetHostUsbAddr(uint8_t addr)
{
    R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | (addr & MASK_USB_ADDR);
}

/*********************************************************************
 * @fn      SetUsbSpeed
 *
 * @brief   ���õ�ǰUSB�ٶ�
 *
 * @param   FullSpeed   - USB�ٶ�
 *
 * @return  none
 */
void SetUsbSpeed(uint8_t FullSpeed)
{
#ifndef DISK_BASE_BUF_LEN
    if(FullSpeed) // ȫ��
    {
        R8_USB_CTRL &= ~RB_UC_LOW_SPEED;  // ȫ��
        R8_UH_SETUP &= ~RB_UH_PRE_PID_EN; // ��ֹPRE PID
    }
    else
    {
        R8_USB_CTRL |= RB_UC_LOW_SPEED; // ����
    }
#endif
    (void)FullSpeed;
}

/*********************************************************************
 * @fn      ResetRootHubPort
 *
 * @brief   ��⵽�豸��,��λ����,Ϊö���豸׼��,����ΪĬ��Ϊȫ��
 *
 * @param   none
 *
 * @return  none
 */
void ResetRootHubPort(void)
{
    UsbDevEndp0Size = DEFAULT_ENDP0_SIZE; //USB�豸�Ķ˵�0�������ߴ�
    SetHostUsbAddr(0x00);
    R8_UHOST_CTRL &= ~RB_UH_PORT_EN;                                      // �ص��˿�
    SetUsbSpeed(1);                                                       // Ĭ��Ϊȫ��
    R8_UHOST_CTRL = (R8_UHOST_CTRL & ~RB_UH_LOW_SPEED) | RB_UH_BUS_RESET; // Ĭ��Ϊȫ��,��ʼ��λ
    mDelaymS(15);                                                         // ��λʱ��10mS��20mS
    R8_UHOST_CTRL = R8_UHOST_CTRL & ~RB_UH_BUS_RESET;                     // ������λ
    mDelayuS(250);
    R8_USB_INT_FG = RB_UIF_DETECT; // ���жϱ�־
}

/*********************************************************************
 * @fn      EnableRootHubPort
 *
 * @brief   ʹ��ROOT-HUB�˿�,��Ӧ��bUH_PORT_EN��1�����˿�,�豸�Ͽ����ܵ��·���ʧ��
 *
 * @param   none
 *
 * @return  ����ERR_SUCCESSΪ��⵽������,����ERR_USB_DISCONΪ������
 */
uint8_t EnableRootHubPort(void)
{
#ifdef DISK_BASE_BUF_LEN
    if(CHRV3DiskStatus < DISK_CONNECT)
        CHRV3DiskStatus = DISK_CONNECT;
#else
    if(ThisUsbDev.DeviceStatus < ROOT_DEV_CONNECTED)
        ThisUsbDev.DeviceStatus = ROOT_DEV_CONNECTED;
#endif
    if(R8_USB_MIS_ST & RB_UMS_DEV_ATTACH)
    { // ���豸
#ifndef DISK_BASE_BUF_LEN
        if((R8_UHOST_CTRL & RB_UH_PORT_EN) == 0x00)
        { // ��δʹ��
            ThisUsbDev.DeviceSpeed = (R8_USB_MIS_ST & RB_UMS_DM_LEVEL) ? 0 : 1;
            if(ThisUsbDev.DeviceSpeed == 0)
            {
                R8_UHOST_CTRL |= RB_UH_LOW_SPEED; // ����
            }
        }
#endif
        R8_UHOST_CTRL |= RB_UH_PORT_EN; //ʹ��HUB�˿�
        return (ERR_SUCCESS);
    }
    return (ERR_USB_DISCON);
}

#ifndef DISK_BASE_BUF_LEN
/*********************************************************************
 * @fn      SelectHubPort
 *
 * @brief   ѡ����Ҫ������HUB��
 *
 * @param   HubPortIndex    - ѡ�����ָ����ROOT-HUB�˿ڵ��ⲿHUB��ָ���˿�
 *
 * @return  None
 */
void SelectHubPort(uint8_t HubPortIndex)
{
    if(HubPortIndex) // ѡ�����ָ����ROOT-HUB�˿ڵ��ⲿHUB��ָ���˿�
    {
        SetHostUsbAddr(DevOnHubPort[HubPortIndex - 1].DeviceAddress); // ����USB������ǰ������USB�豸��ַ
        SetUsbSpeed(DevOnHubPort[HubPortIndex - 1].DeviceSpeed);      // ���õ�ǰUSB�ٶ�
        if(DevOnHubPort[HubPortIndex - 1].DeviceSpeed == 0)           // ͨ���ⲿHUB�����USB�豸ͨѶ��Ҫǰ��ID
        {
            R8_UEP1_CTRL |= RB_UH_PRE_PID_EN; // ����PRE PID
            mDelayuS(100);
        }
    }
    else
    {
        SetHostUsbAddr(ThisUsbDev.DeviceAddress); // ����USB������ǰ������USB�豸��ַ
        SetUsbSpeed(ThisUsbDev.DeviceSpeed);      // ����USB�豸���ٶ�
    }
}
#endif

/*********************************************************************
 * @fn      WaitUSB_Interrupt
 *
 * @brief   �ȴ�USB�ж�
 *
 * @param   none
 *
 * @return  ����ERR_SUCCESS ���ݽ��ջ��߷��ͳɹ�,����ERR_USB_UNKNOWN ���ݽ��ջ��߷���ʧ��
 */
uint8_t WaitUSB_Interrupt(void)
{
    uint16_t i;
    for(i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0; i--)
    {
        ;
    }
    return ((R8_USB_INT_FG & RB_UIF_TRANSFER) ? ERR_SUCCESS : ERR_USB_UNKNOWN);
}

/*********************************************************************
 * @fn      USBHostTransact
 *
 * @brief   ��������,����Ŀ�Ķ˵��ַ/PID����,ͬ����־,��20uSΪ��λ��NAK������ʱ��(0������,0xFFFF��������),����0�ɹ�,��ʱ/��������
 *          ���ӳ���������������,����ʵ��Ӧ����,Ϊ���ṩ�����ٶ�,Ӧ�öԱ��ӳ����������Ż�
 *
 * @param   endp_pid    - ���ƺ͵�ַ, ��4λ��token_pid����, ��4λ�Ƕ˵��ַ
 * @param   tog         - ͬ����־
 * @param   timeout     - ��ʱʱ��
 *
 * @return  ERR_USB_UNKNOWN ��ʱ������Ӳ���쳣
 *          ERR_USB_DISCON  �豸�Ͽ�
 *          ERR_USB_CONNECT �豸����
 *          ERR_SUCCESS     �������
 */
uint8_t USBHostTransact(uint8_t endp_pid, uint8_t tog, uint32_t timeout)
{
    uint8_t TransRetry;

    uint8_t  s, r;
    uint16_t i;

    R8_UH_RX_CTRL = R8_UH_TX_CTRL = tog;
    TransRetry = 0;

    do
    {
        R8_UH_EP_PID = endp_pid; // ָ������PID��Ŀ�Ķ˵��
        R8_USB_INT_FG = RB_UIF_TRANSFER;
        for(i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0; i--)
        {
            DelayUs(1);
        }
        R8_UH_EP_PID = 0x00; // ֹͣUSB����
        if((R8_USB_INT_FG & RB_UIF_TRANSFER) == 0)
        {
            return (ERR_USB_UNKNOWN);
        }

        if(R8_USB_INT_FG & RB_UIF_DETECT)
        { // USB�豸����¼�
            //			mDelayuS( 200 );                                                       // �ȴ��������
            R8_USB_INT_FG = RB_UIF_DETECT;
            s = AnalyzeRootHub(); // ����ROOT-HUB״̬

            if(s == ERR_USB_CONNECT)
                FoundNewDev = 1;
#ifdef DISK_BASE_BUF_LEN
            if(CHRV3DiskStatus == DISK_DISCONNECT)
            {
                return (ERR_USB_DISCON);
            } // USB�豸�Ͽ��¼�
            if(CHRV3DiskStatus == DISK_CONNECT)
            {
                return (ERR_USB_CONNECT);
            } // USB�豸�����¼�
#else
            if(ThisUsbDev.DeviceStatus == ROOT_DEV_DISCONNECT)
            {
                return (ERR_USB_DISCON);
            } // USB�豸�Ͽ��¼�
            if(ThisUsbDev.DeviceStatus == ROOT_DEV_CONNECTED)
            {
                return (ERR_USB_CONNECT);
            } // USB�豸�����¼�
#endif
            mDelayuS(200); // �ȴ��������
        }

        if(R8_USB_INT_FG & RB_UIF_TRANSFER) // ��������¼�
        {
            if(R8_USB_INT_ST & RB_UIS_TOG_OK)
            {
                return (ERR_SUCCESS);
            }
            r = R8_USB_INT_ST & MASK_UIS_H_RES; // USB�豸Ӧ��״̬
            if(r == USB_PID_STALL)
            {
                return (r | ERR_USB_TRANSFER);
            }
            if(r == USB_PID_NAK)
            {
                if(timeout == 0)
                {
                    return (r | ERR_USB_TRANSFER);
                }
                if(timeout < 0xFFFFFFFF)
                {
                    timeout--;
                }
                --TransRetry;
            }
            else
                switch(endp_pid >> 4)
                {
                    case USB_PID_SETUP:
                    case USB_PID_OUT:
                        if(r)
                        {
                            return (r | ERR_USB_TRANSFER);
                        }      // ���ǳ�ʱ/����,����Ӧ��
                        break; // ��ʱ����
                    case USB_PID_IN:
                        if(r == USB_PID_DATA0 || r == USB_PID_DATA1)
                        { // ��ͬ�����趪��������
                        } // ��ͬ������
                        else if(r)
                        {
                            return (r | ERR_USB_TRANSFER);
                        }      // ���ǳ�ʱ/����,����Ӧ��
                        break; // ��ʱ����
                    default:
                        return (ERR_USB_UNKNOWN); // �����ܵ����
                        break;
                }
        }
        else
        {                         // �����ж�,��Ӧ�÷��������
            R8_USB_INT_FG = 0xFF; /* ���жϱ�־ */
        }
        mDelayuS(15);
    } while(++TransRetry < 3);
    return (ERR_USB_TRANSFER); // Ӧ��ʱ
}

/*********************************************************************
 * @fn      HostCtrlTransfer
 *
 * @brief   ִ�п��ƴ���,8�ֽ���������pSetupReq��,DataBufΪ��ѡ���շ�������
 *
 * @param   DataBuf     - �����Ҫ���պͷ�������,��ôDataBuf��ָ����Ч���������ڴ�ź�������
 * @param   RetLen      - ʵ�ʳɹ��շ����ܳ��ȱ�����RetLenָ����ֽڱ�����
 *
 * @return  ERR_USB_BUF_OVER    IN״̬�׶γ���
 *          ERR_SUCCESS         ���ݽ����ɹ�
 */
uint8_t HostCtrlTransfer(uint8_t *DataBuf, uint8_t *RetLen)
{
    uint16_t RemLen = 0;
    uint8_t  s, RxLen, RxCnt, TxCnt;
    uint8_t *pBuf;
    uint8_t *pLen;

    pBuf = DataBuf;
    pLen = RetLen;
    mDelayuS(200);
    if(pLen)
    {
        *pLen = 0; // ʵ�ʳɹ��շ����ܳ���
    }

    R8_UH_TX_LEN = sizeof(USB_SETUP_REQ);
    s = USBHostTransact(USB_PID_SETUP << 4 | 0x00, 0x00, 200000 / 20); // SETUP�׶�,200mS��ʱ
    if(s != ERR_SUCCESS)
    {
        return (s);
    }
    R8_UH_RX_CTRL = R8_UH_TX_CTRL = RB_UH_R_TOG | RB_UH_R_AUTO_TOG | RB_UH_T_TOG | RB_UH_T_AUTO_TOG; // Ĭ��DATA1
    R8_UH_TX_LEN = 0x01;                                                                             // Ĭ�������ݹ�״̬�׶�ΪIN
    RemLen = pSetupReq->wLength;
    PRINT("wLength: %x\n", RemLen);
    if(RemLen && pBuf) // ��Ҫ�շ�����
    {
        PRINT("bRequestType: %x\n", pSetupReq->bRequestType);
        if(pSetupReq->bRequestType & USB_REQ_TYP_IN) // ��
        {
            while(RemLen)
            {
                mDelayuS(200);
                s = USBHostTransact(USB_PID_IN << 4 | 0x00, R8_UH_RX_CTRL, 200000 / 20); // IN����
                if(s != ERR_SUCCESS)
                {
                    return (s);
                }
                RxLen = R8_USB_RX_LEN < RemLen ? R8_USB_RX_LEN : RemLen;
                RemLen -= RxLen;
                if(pLen)
                {
                    *pLen += RxLen; // ʵ�ʳɹ��շ����ܳ���
                }
                for(RxCnt = 0; RxCnt != RxLen; RxCnt++)
                {
                    *pBuf = pHOST_RX_RAM_Addr[RxCnt];
                    pBuf++;
                }
                if(R8_USB_RX_LEN == 0 || (R8_USB_RX_LEN & (UsbDevEndp0Size - 1)))
                {
                    break; // �̰�
                }
            }
            R8_UH_TX_LEN = 0x00; // ״̬�׶�ΪOUT
        }
        else // ��
        {
            while(RemLen)
            {
                mDelayuS(200);
                R8_UH_TX_LEN = RemLen >= UsbDevEndp0Size ? UsbDevEndp0Size : RemLen;
                for(TxCnt = 0; TxCnt != R8_UH_TX_LEN; TxCnt++)
                {
                    pHOST_TX_RAM_Addr[TxCnt] = *pBuf;
                    pBuf++;
                }
                s = USBHostTransact(USB_PID_OUT << 4 | 0x00, R8_UH_TX_CTRL, 200000 / 20); // OUT����
                if(s != ERR_SUCCESS)
                {
                    return (s);
                }
                RemLen -= R8_UH_TX_LEN;
                if(pLen)
                {
                    *pLen += R8_UH_TX_LEN; // ʵ�ʳɹ��շ����ܳ���
                }
            }
            PRINT("Send: %d\n", *pLen);
            //          R8_UH_TX_LEN = 0x01;                                                     // ״̬�׶�ΪIN
        }
    }
    mDelayuS(200);
    s = USBHostTransact((R8_UH_TX_LEN ? USB_PID_IN << 4 | 0x00 : USB_PID_OUT << 4 | 0x00), RB_UH_R_TOG | RB_UH_T_TOG, 200000 / 20); // STATUS�׶�
    if(s != ERR_SUCCESS)
    {
        return (s);
    }
    if(R8_UH_TX_LEN == 0)
    {
        return (ERR_SUCCESS); // ״̬OUT
    }
    if(R8_USB_RX_LEN == 0)
    {
        return (ERR_SUCCESS); // ״̬IN,���IN״̬�������ݳ���
    }
    return (ERR_USB_BUF_OVER); // IN״̬�׶δ���
}

/*********************************************************************
 * @fn      CopySetupReqPkg
 *
 * @brief   ���ƿ��ƴ���������
 *
 * @param   pReqPkt     - �����������ַ
 *
 * @return  none
 */
void CopySetupReqPkg(const uint8_t *pReqPkt) // ���ƿ��ƴ���������
{
    uint8_t i;
    for(i = 0; i != sizeof(USB_SETUP_REQ); i++)
    {
        ((uint8_t *)pSetupReq)[i] = *pReqPkt;
        pReqPkt++;
    }
}

/*********************************************************************
 * @fn      CtrlGetDeviceDescr
 *
 * @brief   ��ȡ�豸������,������ pHOST_TX_RAM_Addr ��
 *
 * @param   none
 *
 * @return  ERR_USB_BUF_OVER    ���������ȴ���
 *          ERR_SUCCESS         �ɹ�
 */
uint8_t CtrlGetDeviceDescr(void)
{
    uint8_t s;
    uint8_t len;

    UsbDevEndp0Size = DEFAULT_ENDP0_SIZE;
    CopySetupReqPkg(SetupGetDevDescr);
    s = HostCtrlTransfer(Com_Buffer, &len); // ִ�п��ƴ���
    if(s != ERR_SUCCESS)
    {
        return (s);
    }
    UsbDevEndp0Size = ((PUSB_DEV_DESCR)Com_Buffer)->bMaxPacketSize0; // �˵�0��������,���Ǽ򻯴���,����Ӧ���Ȼ�ȡǰ8�ֽں���������UsbDevEndp0Size�ټ���
    if(len < ((PUSB_SETUP_REQ)SetupGetDevDescr)->wLength)
    {
        return (ERR_USB_BUF_OVER); // ���������ȴ���
    }
    return (ERR_SUCCESS);
}

/*********************************************************************
 * @fn      CtrlGetConfigDescr
 *
 * @brief   ��ȡ����������,������ pHOST_TX_RAM_Addr ��
 *
 * @param   none
 *
 * @return  ERR_USB_BUF_OVER    ���������ȴ���
 *          ERR_SUCCESS         �ɹ�
 */
uint8_t CtrlGetConfigDescr(void)
{
    uint8_t s;
    uint8_t len;

    CopySetupReqPkg(SetupGetCfgDescr);
    s = HostCtrlTransfer(Com_Buffer, &len); // ִ�п��ƴ���
    if(s != ERR_SUCCESS)
    {
        return (s);
    }
    if(len < ((PUSB_SETUP_REQ)SetupGetCfgDescr)->wLength)
    {
        return (ERR_USB_BUF_OVER); // ���س��ȴ���
    }

    len = ((PUSB_CFG_DESCR)Com_Buffer)->wTotalLength;
    CopySetupReqPkg(SetupGetCfgDescr);
    pSetupReq->wLength = len;               // �����������������ܳ���
    s = HostCtrlTransfer(Com_Buffer, &len); // ִ�п��ƴ���
    if(s != ERR_SUCCESS)
    {
        return (s);
    }

#ifdef DISK_BASE_BUF_LEN
    if(len > 64)
        len = 64;
    memcpy(TxBuffer, Com_Buffer, len); //U�̲���ʱ����Ҫ������TxBuffer
#endif

    return (ERR_SUCCESS);
}

/*********************************************************************
 * @fn      CtrlSetUsbAddress
 *
 * @brief   ����USB�豸��ַ
 *
 * @param   addr    - �豸��ַ
 *
 * @return  ERR_SUCCESS     �ɹ�
 */
uint8_t CtrlSetUsbAddress(uint8_t addr)
{
    uint8_t s;

    CopySetupReqPkg(SetupSetUsbAddr);
    pSetupReq->wValue = addr;         // USB�豸��ַ
    s = HostCtrlTransfer(NULL, NULL); // ִ�п��ƴ���
    if(s != ERR_SUCCESS)
    {
        return (s);
    }
    SetHostUsbAddr(addr); // ����USB������ǰ������USB�豸��ַ
    mDelaymS(10);         // �ȴ�USB�豸��ɲ���
    return (ERR_SUCCESS);
}

/*********************************************************************
 * @fn      CtrlSetUsbConfig
 *
 * @brief   ����USB�豸����
 *
 * @param   cfg     - ����ֵ
 *
 * @return  ERR_SUCCESS     �ɹ�
 */
uint8_t CtrlSetUsbConfig(uint8_t cfg)
{
    CopySetupReqPkg(SetupSetUsbConfig);
    pSetupReq->wValue = cfg;               // USB�豸����
    return (HostCtrlTransfer(NULL, NULL)); // ִ�п��ƴ���
}

/*********************************************************************
 * @fn      CtrlClearEndpStall
 *
 * @brief   ����˵�STALL
 *
 * @param   endp    - �˵��ַ
 *
 * @return  ERR_SUCCESS     �ɹ�
 */
uint8_t CtrlClearEndpStall(uint8_t endp)
{
    CopySetupReqPkg(SetupClrEndpStall); // ����˵�Ĵ���
    pSetupReq->wIndex = endp;                     // �˵��ַ
    return (HostCtrlTransfer(NULL, NULL));        // ִ�п��ƴ���
}

/*********************************************************************
 * @fn      CtrlSetUsbIntercace
 *
 * @brief   ����USB�豸�ӿ�
 *
 * @param   cfg     - ����ֵ
 *
 * @return  ERR_SUCCESS     �ɹ�
 */
uint8_t CtrlSetUsbIntercace(uint8_t cfg)
{
    CopySetupReqPkg(SetupSetUsbInterface);
    pSetupReq->wValue = cfg;               // USB�豸����
    return (HostCtrlTransfer(NULL, NULL)); // ִ�п��ƴ���
}

/*********************************************************************
 * @fn      USB_HostInit
 *
 * @brief   USB�������ܳ�ʼ��
 *
 * @param   none
 *
 * @return  none
 */
void USB_HostInit(void)
{
    R8_USB_CTRL = RB_UC_HOST_MODE;
    R8_UHOST_CTRL = 0;
    R8_USB_DEV_AD = 0x00;

    R8_UH_EP_MOD = RB_UH_EP_TX_EN | RB_UH_EP_RX_EN;
    R16_UH_RX_DMA = (uint32_t)pHOST_RX_RAM_Addr;
    R16_UH_TX_DMA = (uint32_t)pHOST_TX_RAM_Addr;

    R8_UH_RX_CTRL = 0x00;
    R8_UH_TX_CTRL = 0x00;
    R8_USB_CTRL = RB_UC_HOST_MODE | RB_UC_INT_BUSY | RB_UC_DMA_EN;
    R8_UH_SETUP = RB_UH_SOF_EN;
    R8_USB_INT_FG = 0xFF;
    DisableRootHubPort();
    R8_USB_INT_EN = RB_UIE_TRANSFER | RB_UIE_DETECT;

    FoundNewDev = 0;
}
