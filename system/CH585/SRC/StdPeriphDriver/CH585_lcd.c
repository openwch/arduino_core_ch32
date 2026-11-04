/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_lcd.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/15
 * Description        : source file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH585_common.h"
#include "CH585_lcd.h"

/*******************************************************************************
* Function Name  : LCD_DefInit
* Description    : LCD��ʽ��������ʼ������
* Input          : duty ռ�ձ�
*                  bias ƫѹ��
* Return         : None
*******************************************************************************/
void LCD_Init(LCDDutyTypeDef duty, LCDBiasTypeDef bias)
{
    R32_PIN_IN_DIS |= 0x0000238F; // �ر���������
    R32_PIN_IN_DIS |= RB_PBLx_IN_DIS; // �ر���������
    R16_PIN_CONFIG |= RB_PBHx_IN_DIS; // ����LCDʱ����ر�debug
    R32_LCD_SEG_EN = 0x0FFFFFFF;
    R8_LCD_CMD = RB_LCD_SYS_EN | RB_LCD_ON |
                   (LCD_CLK_128 << 5)  |
                   (duty << 3) |
                   (bias << 2);
}
