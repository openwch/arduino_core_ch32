/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_keyscan.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH572_KEYSCAN_H__
#define __CH572_KEYSCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief Keyscan Pin Configuration
 */
#define   KEYSCAN_PA2     0x100
#define   KEYSCAN_PA3     0x200
#define   KEYSCAN_PA8     0x400
#define   KEYSCAN_PA10    0x800
#define   KEYSCAN_PA11    0x1000
#define   KEYSCAN_ALL     0x1F00

/*
 * @brief Keyscan Clock Division
 */
#define   KEYSCAN_DIV1     0x00
#define   KEYSCAN_DIV2     0x10
#define   KEYSCAN_DIV4     0x30
#define   KEYSCAN_DIV8     0x70
#define   KEYSCAN_DIV16    0xF0

/*
 * @brief Keyscan repetition times
 */
#define   KEYSCAN_REP1     0x02
#define   KEYSCAN_REP2     0x04
#define   KEYSCAN_REP3     0x06
#define   KEYSCAN_REP4     0x08
#define   KEYSCAN_REP5     0x0A
#define   KEYSCAN_REP6     0x0C
#define   KEYSCAN_REP7     0x0E

/*
 * @brief Keyscan Configuration
 */
void  KeyScan_Cfg(uint8_t s, uint16_t keyScanPin, uint16_t ClkDiv, uint16_t Rep);

/*
 * @brief Keypress Wakeup Enable
 */
void  KeyPress_Wake(uint8_t s);

/*
 * @brief Key Values Return
 */
#define   KeyValue   (R32_KEY_SCAN_NUMB & RB_KEY_SCAN_NUMB)

/*
 * @brief Keyscan Count
 */
#define   KeyScan_Cnt   (R32_KEY_SCAN_NUMB >> 20)

/**
 * @brief   KeyScan�ж�����
 *
 * @param   s       - ʹ��/�ر�
 * @param   f       - refer to ENC interrupt bit define
 */
#define KeyScan_ITCfg(s, f)       ((s) ? (R8_KEY_SCAN_INT_EN |= f) : (R8_KEY_SCAN_INT_EN &= ~f))

/**
 * @brief   ���ENC�жϱ�־
 *
 * @param   f       - refer to ENC interrupt bit define
 */
#define KeyScan_ClearITFlag(f)    (R8_KEY_SCAN_INT_FLAG = f)

/**
 * @brief   ��ѯ�жϱ�־״̬
 *
 * @param   f       - refer to ENC interrupt bit define
 */
#define KeyScan_GetITFlag(f)      (R8_KEY_SCAN_INT_FLAG & f)

#ifdef __cplusplus
}
#endif

#endif // __CH572_KEYSCAN_H__
