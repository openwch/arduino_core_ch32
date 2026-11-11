/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_cmp.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH572_CMP_H__
#define __CH572_CMP_H__

#ifdef __cplusplus
extern "C" {
#endif
/* comparator -channel input sel */
typedef enum
{
    cmp_sw_0 = 0,          // CMP_P0(PA3),CMP_N(PA2)
    cmp_sw_1,              // CMP_P0(PA3),CMP_N(CMP_VERF)
    cmp_sw_2,              // CMP_P0(PA7),CMP_N(PA2)
    cmp_sw_3               // CMP_P0(PA7),CMP_N(CMP_VERF)
} CMPSwTypeDef;

//GPIOModeTypeDef
/* comparator negative end point Vref sel */
typedef enum
{
    cmp_nref_level_50 = 0,          // 50mv
    cmp_nref_level_100,             // 100mv
    cmp_nref_level_150,             // 150mv
    cmp_nref_level_200,             // 200mv
    cmp_nref_level_250,             // 250mv
    cmp_nref_level_300,             // 300mv
    cmp_nref_level_350,             // 350mv
    cmp_nref_level_400,             // 400mv
    cmp_nref_level_450,             // 450mv
    cmp_nref_level_500,             // 500mv
    cmp_nref_level_550,             // 550mv
    cmp_nref_level_600,             // 600mv
    cmp_nref_level_650,             // 650mv
    cmp_nref_level_700,             // 700mv
    cmp_nref_level_750,             // 750mv
    cmp_nref_level_800,             // 800mv
} CMPNrefLevelTypeDef;

/* comparator output sel */
typedef enum
{
    cmp_out_sel_high = 0,          // high
    cmp_out_sel_low,               // low
    cmp_out_sel_fall,              // fall edge
    cmp_out_sel_rise               // rise edge
} CMPOutSelTypeDef;

#define CMP_GetITStatus() (R8_CMP_CTRL_2 & RB_CMP_IF)

#define CMP_ClearITStatus() (R8_CMP_CTRL_2 |= RB_CMP_IF)

#define CMP_ReadAPROut() (R8_CMP_CTRL_3 & RB_APR_OUT_CMP)

#define CMP_Enable() (R8_CMP_CTRL_0 |= RB_CMP_EN)

#define CMP_Disable() (R8_CMP_CTRL_0 &= ~RB_CMP_EN)

void CMP_Init (CMPSwTypeDef s, CMPNrefLevelTypeDef v);

void CMP_OutToTIMCAPCfg(FunctionalState s);

void CMP_INTCfg (CMPOutSelTypeDef sel, FunctionalState s);

#ifdef __cplusplus
}
#endif

#endif // __CH572_CMP_H__
