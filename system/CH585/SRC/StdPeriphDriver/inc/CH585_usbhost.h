/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_usbhost.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH585_USBHOST_H__
#define __CH585_USBHOST_H__

#ifdef __cplusplus
extern "C" {
#endif

#if DISK_LIB_ENABLE
  #if DISK_WITHOUT_USB_HUB
  /* ��ʹ��U���ļ�ϵͳ�����U�̹���USBhub���棬��Ҫ�ر����涨�� */
    #define FOR_ROOT_UDISK_ONLY
  #endif
  /* ʹ��U���ļ�ϵͳ�⣬��Ҫ�������涨��, ��ʹ����ر� */
  #define DISK_BASE_BUF_LEN    512  /* Ĭ�ϵĴ������ݻ�������СΪ512�ֽ�,����ѡ��Ϊ2048����4096��֧��ĳЩ��������U��,Ϊ0���ֹ��.H�ļ��ж��建��������Ӧ�ó�����pDISK_BASE_BUF��ָ�� */
#endif

// ���ӳ��򷵻�״̬��
#define ERR_SUCCESS            0x00  // �����ɹ�
#define ERR_USB_CONNECT        0x15  /* ��⵽USB�豸�����¼�,�Ѿ����� */
#define ERR_USB_DISCON         0x16  /* ��⵽USB�豸�Ͽ��¼�,�Ѿ��Ͽ� */
#define ERR_USB_BUF_OVER       0x17  /* USB��������������������̫�໺������� */
#define ERR_USB_DISK_ERR       0x1F  /* USB�洢������ʧ��,�ڳ�ʼ��ʱ������USB�洢����֧��,�ڶ�д�����п����Ǵ����𻵻����Ѿ��Ͽ� */
#define ERR_USB_TRANSFER       0x20  /* NAK/STALL�ȸ����������0x20~0x2F */
#define ERR_USB_UNSUPPORT      0xFB  /* ��֧�ֵ�USB�豸*/
#define ERR_USB_UNKNOWN        0xFE  /* �豸��������*/
#define ERR_AOA_PROTOCOL       0x41  /* Э��汾���� */

/*USB�豸�����Ϣ��,���֧��1���豸*/
#define ROOT_DEV_DISCONNECT    0
#define ROOT_DEV_CONNECTED     1
#define ROOT_DEV_FAILED        2
#define ROOT_DEV_SUCCESS       3
#define DEV_TYPE_KEYBOARD      (USB_DEV_CLASS_HID | 0x20)
#define DEV_TYPE_MOUSE         (USB_DEV_CLASS_HID | 0x30)
#define DEF_AOA_DEVICE         0xF0
#define DEV_TYPE_UNKNOW        0xFF

/*
Լ��: USB�豸��ַ�������(�ο�USB_DEVICE_ADDR)
��ֵַ  �豸λ��
0x02    ����Root-HUB�µ�USB�豸���ⲿHUB
0x1x    ����Root-HUB�µ��ⲿHUB�Ķ˿�x�µ�USB�豸,xΪ1~n
*/
#define HUB_MAX_PORTS          4
#define WAIT_USB_TOUT_200US    800   // �ȴ�USB�жϳ�ʱʱ��

typedef struct
{
    uint8_t  DeviceStatus;  // �豸״̬,0-���豸,1-���豸����δ��ʼ��,2-���豸����ʼ��ö��ʧ��,3-���豸�ҳ�ʼ��ö�ٳɹ�
    uint8_t  DeviceAddress; // �豸�������USB��ַ
    uint8_t  DeviceSpeed;   // 0Ϊ����,��0Ϊȫ��
    uint8_t  DeviceType;    // �豸����
    uint16_t DeviceVID;
    uint16_t DevicePID;
    uint8_t  GpVar[4];     // ͨ�ñ�������Ŷ˵�
    uint8_t  GpHUBPortNum; // ͨ�ñ���,�����HUB����ʾHUB�˿���
} _RootHubDev;

typedef struct
{
    uint8_t  DeviceStatus;  // �豸״̬,0-���豸,1-���豸����δ��ʼ��,2-���豸����ʼ��ö��ʧ��,3-���豸�ҳ�ʼ��ö�ٳɹ�
    uint8_t  DeviceAddress; // �豸�������USB��ַ
    uint8_t  DeviceSpeed;   // 0Ϊ����,��0Ϊȫ��
    uint8_t  DeviceType;    // �豸����
    uint16_t DeviceVID;
    uint16_t DevicePID;
    uint8_t  GpVar[4]; // ͨ�ñ���
} _DevOnHubPort;     // �ٶ�:������1���ⲿHUB,ÿ���ⲿHUB������HUB_MAX_PORTS���˿�(���˲���)

extern _RootHubDev   ThisUsbDev;
extern _DevOnHubPort DevOnHubPort[HUB_MAX_PORTS]; // �ٶ�:������1���ⲿHUB,ÿ���ⲿHUB������HUB_MAX_PORTS���˿�(���˲���)
extern uint8_t       UsbDevEndp0Size;             // USB�豸�Ķ˵�0�������ߴ� */
extern uint8_t       FoundNewDev;

extern uint8_t *pHOST_RX_RAM_Addr;
extern uint8_t *pHOST_TX_RAM_Addr;

extern _RootHubDev   ThisUsb2Dev;
extern _DevOnHubPort DevOnU2HubPort[HUB_MAX_PORTS]; // �ٶ�:������1���ⲿHUB,ÿ���ⲿHUB������HUB_MAX_PORTS���˿�(���˲���)
extern uint8_t       Usb2DevEndp0Size;              // USB�豸�Ķ˵�0�������ߴ� */
extern uint8_t       FoundNewU2Dev;

extern uint8_t *pU2HOST_RX_RAM_Addr;
extern uint8_t *pU2HOST_TX_RAM_Addr;

#define pSetupReq      ((PUSB_SETUP_REQ)pHOST_TX_RAM_Addr)
#define pU2SetupReq    ((PUSB_SETUP_REQ)pU2HOST_TX_RAM_Addr)
extern uint8_t Com_Buffer[];
extern uint8_t U2Com_Buffer[];

/* ����ΪUSB��������� */
extern const uint8_t SetupGetDevDescr[];     // ��ȡ�豸������*/
extern const uint8_t SetupGetCfgDescr[];     // ��ȡ����������*/
extern const uint8_t SetupSetUsbAddr[];      // ����USB��ַ*/
extern const uint8_t SetupSetUsbConfig[];    // ����USB����*/
extern const uint8_t SetupSetUsbInterface[]; // ����USB�ӿ�����*/
extern const uint8_t SetupClrEndpStall[];    // ����˵�STALL*/

extern const uint8_t SetupGetU2DevDescr[];    // ��ȡ�豸������*/
extern const uint8_t SetupGetU2CfgDescr[];    // ��ȡ����������*/
extern const uint8_t SetupSetUsb2Addr[];      // ����USB��ַ*/
extern const uint8_t SetupSetUsb2Config[];    // ����USB����*/
extern const uint8_t SetupSetUsb2Interface[]; // ����USB�ӿ�����*/
extern const uint8_t SetupClrU2EndpStall[];   // ����˵�STALL*/

/**
 * @brief   �ر�ROOT-HUB�˿�,ʵ����Ӳ���Ѿ��Զ��ر�,�˴�ֻ�����һЩ�ṹ״̬
 */
void DisableRootHubPort(void);

/**
 * @brief   ����ROOT-HUB״̬,����ROOT-HUB�˿ڵ��豸����¼�
 *          ����豸�γ�,�����е���DisableRootHubPort()����,���˿ڹر�,�����¼�,����Ӧ�˿ڵ�״̬λ
 *
 * @return  ����ERR_SUCCESSΪû�����,����ERR_USB_CONNECTΪ��⵽������,����ERR_USB_DISCONΪ��⵽�Ͽ�
 */
uint8_t AnalyzeRootHub(void);

/**
 * @brief   ����USB������ǰ������USB�豸��ַ
 *
 * @param   addr    - USB�豸��ַ
 */
void SetHostUsbAddr(uint8_t addr);

/**
 * @brief   ���õ�ǰUSB�ٶ�
 *
 * @param   FullSpeed   - USB�ٶ�
 */
void SetUsbSpeed(uint8_t FullSpeed);

/**
 * @brief   ��⵽�豸��,��λ����,Ϊö���豸׼��,����ΪĬ��Ϊȫ��
 */
void ResetRootHubPort(void);

/**
 * @brief   ʹ��ROOT-HUB�˿�,��Ӧ��bUH_PORT_EN��1�����˿�,�豸�Ͽ����ܵ��·���ʧ��
 *
 * @return  ����ERR_SUCCESSΪ��⵽������,����ERR_USB_DISCONΪ������
 */
uint8_t EnableRootHubPort(void);

/**
 * @brief   �ȴ�USB�ж�
 *
 * @return  ����ERR_SUCCESS ���ݽ��ջ��߷��ͳɹ�,����ERR_USB_UNKNOWN ���ݽ��ջ��߷���ʧ��
 */
uint8_t WaitUSB_Interrupt(void);

/**
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
uint8_t USBHostTransact(uint8_t endp_pid, uint8_t tog, uint32_t timeout);

/**
 * @brief   ִ�п��ƴ���,8�ֽ���������pSetupReq��,DataBufΪ��ѡ���շ�������
 *
 * @param   DataBuf     - �����Ҫ���պͷ�������,��ôDataBuf��ָ����Ч���������ڴ�ź�������
 * @param   RetLen      - ʵ�ʳɹ��շ����ܳ��ȱ�����RetLenָ����ֽڱ�����
 *
 * @return  ERR_USB_BUF_OVER    IN״̬�׶γ���
 *          ERR_SUCCESS         ���ݽ����ɹ�
 */
uint8_t HostCtrlTransfer(uint8_t *DataBuf, uint8_t *RetLen);

/**
 * @brief   ���ƿ��ƴ���������
 *
 * @param   pReqPkt     - �����������ַ
 */
void CopySetupReqPkg(const uint8_t *pReqPkt);

/**
 * @brief   ��ȡ�豸������,������ pHOST_TX_RAM_Addr ��
 *
 * @return  ERR_USB_BUF_OVER    ���������ȴ���
 *          ERR_SUCCESS         �ɹ�
 */
uint8_t CtrlGetDeviceDescr(void);

/**
 * @brief   ��ȡ����������,������ pHOST_TX_RAM_Addr ��
 *
 * @return  ERR_USB_BUF_OVER    ���������ȴ���
 *          ERR_SUCCESS         �ɹ�
 */
uint8_t CtrlGetConfigDescr(void);

/**
 * @brief   ����USB�豸��ַ
 *
 * @param   addr    - �豸��ַ
 *
 * @return  ERR_SUCCESS     �ɹ�
 */
uint8_t CtrlSetUsbAddress(uint8_t addr);

/**
 * @brief   ����USB�豸����
 *
 * @param   cfg     - ����ֵ
 *
 * @return  ERR_SUCCESS     �ɹ�
 */
uint8_t CtrlSetUsbConfig(uint8_t cfg);

/**
 * @brief   ����˵�STALL
 *
 * @param   endp    - �˵��ַ
 *
 * @return  ERR_SUCCESS     �ɹ�
 */
uint8_t CtrlClearEndpStall(uint8_t endp);

/**
 * @brief   ����USB�豸�ӿ�
 *
 * @param   cfg     - ����ֵ
 *
 * @return  ERR_SUCCESS     �ɹ�
 */
uint8_t CtrlSetUsbIntercace(uint8_t cfg);

/**
 * @brief   USB�������ܳ�ʼ��
 */
void USB_HostInit(void);
uint8_t EnumAllHubPort(void);// ö������ROOT-HUB�˿����ⲿHUB��Ķ���USB�豸
void SelectHubPort(uint8_t HubPortIndex); // HubPortIndex=0ѡ�����ָ����ROOT-HUB�˿�,����ѡ�����ָ����ROOT-HUB�˿ڵ��ⲿHUB��ָ���˿�
uint16_t SearchTypeDevice(uint8_t type); // ��ROOT-HUB�Լ��ⲿHUB���˿�������ָ�����͵��豸���ڵĶ˿ں�,����˿ں�Ϊ0xFFFF��δ������.
uint8_t SETorOFFNumLock(uint8_t *buf); // NumLock�ĵ���ж�

/*************************************************************/

/**
 * @brief   ��ʼ��ָ��ROOT-HUB�˿ڵ�USB�豸
 *
 * @return  ������
 */
uint8_t InitRootDevice(void);

/**
 * @brief   ��ȡHID�豸����������,������TxBuffer��
 *
 * @return  ������
 */
uint8_t CtrlGetHIDDeviceReport(uint8_t infc);

/**
 * @brief   ��ȡHUB������,������Com_Buffer��
 *
 * @return  ������
 */
uint8_t CtrlGetHubDescr(void);

/**
 * @brief   ��ѯHUB�˿�״̬,������Com_Buffer��
 *
 * @param   HubPortIndex    - �˿ں�
 *
 * @return  ������
 */
uint8_t HubGetPortStatus(uint8_t HubPortIndex);

/**
 * @brief   ����HUB�˿�����
 *
 * @param   HubPortIndex    - �˿ں�
 * @param   FeatureSelt     - �˿�����
 *
 * @return  ������
 */
uint8_t HubSetPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt);

/**
 * @brief   ���HUB�˿�����
 *
 * @param   HubPortIndex    - �˿ں�
 * @param   FeatureSelt     - �˿�����
 *
 * @return  ������
 */
uint8_t HubClearPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt);

#ifdef __cplusplus
}
#endif

#endif // __CH585_USBHOST_H__
