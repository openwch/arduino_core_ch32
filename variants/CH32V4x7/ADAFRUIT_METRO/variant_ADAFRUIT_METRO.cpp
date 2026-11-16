// SPDX-FileCopyrightText: Copyright (c) 2026 Adafruit Industries
// SPDX-License-Identifier: BSD-3-Clause
#include "ch32_def.h"
#include "pins_arduino.h"

const PinName digitalPin[] = {
    PB_7,  PB_6,  PB_8, PB_9,  PC_0,  PC_1,  PC_2,  PC_3,  PA_0, PA_1,
    PA_2,  PA_8,  PA_9, PB_12, PA_4,  PA_3,  PB_0,  PB_1,  PC_4, PC_5,
    PB_11, PB_10, PA_7, PA_6,  PA_5,  PC_14, PC_15, PA_15, PB_3, PB_4,
    PB_5,  PC_12, PD_2, PB_14, PB_15, PC_10, PC_11, PA_10};
const uint32_t analogInputPin[] = {14, 15, 16, 17, 18, 19};
static_assert(sizeof(digitalPin) / sizeof(digitalPin[0]) == NUM_DIGITAL_PINS,
              "Metro pin count must match NUM_DIGITAL_PINS");

extern "C" void ch32TinyUSBEnterBootloader(void) {
  RCC_PB1PeriphClockCmd(RCC_PB1Periph_BKP | RCC_PB1Periph_PWR, ENABLE);
  PWR->CTLR |= PWR_CTLR_DBP;
  // TinyUF2's backup-register request survives the software reset.
  *(volatile uint16_t *)0x40006C28 = 0xF016;
  NVIC_SystemReset();
}
