/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_lcd.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2022/12/05
 * Description        : head file(ch585/ch584)
 ********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/


#ifndef __CH585_LCD_H__
#define __CH585_LCD_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <CH585SFR.h>

/**
  * @brief  Configuration LCD driver power
  */
typedef enum
{
	LCD_PS_3V3 = 0,					// 3.3V ����
	LCD_PS_2V5,						// 2.5V ����
}LCDDrvPowerTypeDef; 

/**
  * @brief  Configuration LCD bias
  */
typedef enum
{
	LCD_1_2_Bias = 0,				// 2����ѹ
	LCD_1_3_Bias,					// 3����ѹ
}LCDBiasTypeDef;

/**
  * @brief  Configuration LCD duty
  */
typedef enum
{
	LCD_1_2_Duty = 0,				// COM0-COM1
	LCD_1_3_Duty,					// COM0-COM2
	LCD_1_4_Duty,					// COM0-COM3
}LCDDutyTypeDef;

/**
  * @brief  Configuration LCD scan clk
  */
typedef enum
{
	LCD_CLK_256 = 0,				// 256Hz
	LCD_CLK_512,					// 512Hz
	LCD_CLK_1000,					// 1KHz
	LCD_CLK_128						// 128Hz
}LCDSCANCLKTypeDef;
	 
/* LCD��ʽ��������ʼ������ */
void LCD_Init(LCDDutyTypeDef duty, LCDBiasTypeDef bias);

#define	LCD_PowerDown()			(R32_LCD_CMD &= ~(RB_LCD_ON | RB_LCD_SYS_EN))		/* LCD����ģ��ر� */
#define	LCD_PowerOn()			(R32_LCD_CMD |= (RB_LCD_ON | RB_LCD_SYS_EN))		/* LCD����ģ�鿪�� */

// ����ֵ�ο� LCDDrvPowerTypeDef
#define LCD_PowerCfg( d )		(R32_LCD_CMD = (R32_LCD_CMD & ~RB_LCD_VLCD_SEL) | (d<<7))			/* ����LCD�� �����ѹѡ�� */
// ����ֵ�ο� LCDSCANCLKTypeDef
#define LCD_ScanCLKCfg( d )		(R32_LCD_CMD = (R32_LCD_CMD & ~RB_LCD_SCAN_CLK) | (d<<5))			/* ����LCD�� ɨ��ʱ��ѡ�� */
// ����ֵ�ο� LCDDutyTypeDef
#define LCD_DutyCfg( d )		(R32_LCD_CMD = (R32_LCD_CMD & ~RB_LCD_DUTY) | (d<<3))				/* ����LCD�� dutyѡ�� */
// ����ֵ�ο� LCDBiasTypeDef
#define LCD_BiasCfg( d )		(R32_LCD_CMD = (R32_LCD_CMD & ~RB_LCD_BIAS) | (d<<2))				/* ����LCD�� biasѡ�� */
	 
#define LCD_WriteData0( d )		(R32_LCD_RAM0 = (R32_LCD_RAM0 & 0xffffff00) | ((uint32_t)d))			/* ���LCD0������ֵ */
#define LCD_WriteData1( d )		(R32_LCD_RAM0 = (R32_LCD_RAM0 & 0xffff00ff) | ((uint32_t)d<<8))		/* ���LCD1������ֵ */
#define LCD_WriteData2( d )		(R32_LCD_RAM0 = (R32_LCD_RAM0 & 0xff00ffff) | ((uint32_t)d<<16))		/* ���LCD2������ֵ */
#define LCD_WriteData3( d )		(R32_LCD_RAM0 = (R32_LCD_RAM0 & 0x00ffffff) | ((uint32_t)d<<24))		/* ���LCD3������ֵ */
	 
#define LCD_WriteData4( d )		(R32_LCD_RAM1 = (R32_LCD_RAM1 & 0xffffff00) | ((uint32_t)d))			/* ���LCD4������ֵ */
#define LCD_WriteData5( d )		(R32_LCD_RAM1 = (R32_LCD_RAM1 & 0xffff00ff) | ((uint32_t)d<<8))		/* ���LCD5������ֵ */
#define LCD_WriteData6( d )		(R32_LCD_RAM1 = (R32_LCD_RAM1 & 0xff00ffff) | ((uint32_t)d<<16))		/* ���LCD6������ֵ */
#define LCD_WriteData7( d )		(R32_LCD_RAM1 = (R32_LCD_RAM1 & 0x00ffffff) | ((uint32_t)d<<24))		/* ���LCD7������ֵ */
	 
#define LCD_WriteData8( d )		(R32_LCD_RAM2 = (R32_LCD_RAM2 & 0xffffff00) | ((uint32_t)d))			/* ���LCD8������ֵ */
#define LCD_WriteData9( d )		(R32_LCD_RAM2 = (R32_LCD_RAM2 & 0xffff00ff) | ((uint32_t)d<<8))		/* ���LCD9������ֵ */
#define LCD_WriteData10( d )	(R32_LCD_RAM2 = (R32_LCD_RAM2 & 0xff00ffff) | ((uint32_t)d<<16))		/* ���LCD10������ֵ */
#define LCD_WriteData11( d )    (R32_LCD_RAM2 = (R32_LCD_RAM2 & 0x00ffffff) | ((uint32_t)d<<24))      /* ���LCD11������ֵ */

#define LCD_WriteData12( d )    (R32_LCD_RAM3 = (R32_LCD_RAM3 & 0xffffff00) | ((uint32_t)d))          /* ���LCD12������ֵ */
#define LCD_WriteData13( d )    (R32_LCD_RAM3 = (R32_LCD_RAM3 & 0xffff00ff) | ((uint32_t)d<<8))       /* ���LCD13������ֵ */



#ifdef __cplusplus
}
#endif

#endif  // __CH585_LCD_H__

