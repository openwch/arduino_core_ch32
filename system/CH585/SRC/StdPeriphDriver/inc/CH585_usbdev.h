/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_usbdev.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH585_USBDEV_H__
#define __CH585_USBDEV_H__

#ifdef __cplusplus
extern "C" {
#endif

/* HID������ */
#define DEF_USB_GET_IDLE           0x02                                         /* get idle for key or mouse */
#define DEF_USB_GET_PROTOCOL       0x03                                         /* get protocol for bios type */
#define DEF_USB_SET_REPORT         0x09                                         /* set report for key */
#define DEF_USB_SET_IDLE           0x0A                                         /* set idle for key or mouse */
#define DEF_USB_SET_PROTOCOL       0x0B                                         /* set protocol for bios type */

/* ���»�������USBģ���շ�ʹ�õ����ݻ��������ܹ�9��ͨ����9�黺�棩���û��ɸ���ʵ��ʹ�õ�ͨ����������Ӧ������ */
extern uint8_t *pEP0_RAM_Addr; //ep0(64)+ep4_out(64)+ep4_in(64)
extern uint8_t *pEP1_RAM_Addr; //ep1_out(64)+ep1_in(64)
extern uint8_t *pEP2_RAM_Addr; //ep2_out(64)+ep2_in(64)
extern uint8_t *pEP3_RAM_Addr; //ep3_out(64)+ep3_in(64)

#define pSetupReqPak          ((PUSB_SETUP_REQ)pEP0_RAM_Addr)
#define pEP0_DataBuf          (pEP0_RAM_Addr)
#define pEP1_OUT_DataBuf      (pEP1_RAM_Addr)
#define pEP1_IN_DataBuf       (pEP1_RAM_Addr + 64)
#define pEP2_OUT_DataBuf      (pEP2_RAM_Addr)
#define pEP2_IN_DataBuf       (pEP2_RAM_Addr + 64)
#define pEP3_OUT_DataBuf      (pEP3_RAM_Addr)
#define pEP3_IN_DataBuf       (pEP3_RAM_Addr + 64)
#define pEP4_OUT_DataBuf      (pEP0_RAM_Addr + 64)
#define pEP4_IN_DataBuf       (pEP0_RAM_Addr + 128)

/**
 * @brief   USB�豸���ܳ�ʼ����4���˵㣬8��ͨ����
 */
void USB_DeviceInit(void);

/**
 * @brief   USB�豸Ӧ���䴦��
 */
void USB_DevTransProcess(void);

/**
 * @brief   �˵�1�´����ݴ���
 *
 * @param   l   - ���������ݳ���(<64B)
 */
void DevEP1_OUT_Deal(uint8_t l);

/**
 * @brief   �˵�2�´����ݴ���
 *
 * @param   l   - ���������ݳ���(<64B)
 */
void DevEP2_OUT_Deal(uint8_t l);

/**
 * @brief   �˵�3�´����ݴ���
 *
 * @param   l   - ���������ݳ���(<64B)
 */
void DevEP3_OUT_Deal(uint8_t l);

/**
 * @brief   �˵�4�´����ݴ���
 *
 * @param   l   - ���������ݳ���(<64B)
 */
void DevEP4_OUT_Deal(uint8_t l);

/**
 * @brief   �˵�1�����ϴ�
 *
 * @param   l   - �ϴ����ݳ���(<64B)
 */
void DevEP1_IN_Deal(uint8_t l);

/**
 * @brief   �˵�2�����ϴ�
 *
 * @param   l   - �ϴ����ݳ���(<64B)
 */
void DevEP2_IN_Deal(uint8_t l);

/**
 * @brief   �˵�3�����ϴ�
 *
 * @param   l   - �ϴ����ݳ���(<64B)
 */
void DevEP3_IN_Deal(uint8_t l);

/**
 * @brief   �˵�4�����ϴ�
 *
 * @param   l   - �ϴ����ݳ���(<64B)
 */
void DevEP4_IN_Deal(uint8_t l);

/**
 * @brief   ��ѯ�˵�1�Ƿ��ϴ����
 *
 * @return  0-δ���  (!0)-�����
 */
#define EP1_GetINSta()    (R8_UEP1_CTRL & UEP_T_RES_NAK)

/**
 * @brief   ��ѯ�˵�2�Ƿ��ϴ����
 *
 * @return  0-δ���  (!0)-�����
 */
#define EP2_GetINSta()    (R8_UEP2_CTRL & UEP_T_RES_NAK)

/**
 * @brief   ��ѯ�˵�3�Ƿ��ϴ����
 *
 * @return  0-δ���  (!0)-�����
 */
#define EP3_GetINSta()    (R8_UEP3_CTRL & UEP_T_RES_NAK)

/**
 * @brief   ��ѯ�˵�4�Ƿ��ϴ����
 *
 * @return  0-δ���  (!0)-�����
 */
#define EP4_GetINSta()    (R8_UEP4_CTRL & UEP_T_RES_NAK)

/**
 * @brief   �ر�USB��������
 */
#define USB_DisablePin()  (R16_PIN_ANALOG_IE &= ~(RB_PIN_USB_IE | RB_PIN_USB_DP_PU))

/**
 * @brief   �ر�USB
 */
#define USB_Disable()     (R32_USB_CONTROL = 0)

#ifdef __cplusplus
}
#endif

#endif // __CH585_USBDEV_H__
