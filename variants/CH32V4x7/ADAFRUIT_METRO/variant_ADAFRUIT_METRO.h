// SPDX-FileCopyrightText: Copyright (c) 2026 Adafruit Industries
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#define ADC_MODULE_ENABLED
#define DAC_MODULE_ENABLED
#define UART_MODULE_ENABLED
#define SPI_MODULE_ENABLED
#define I2C_MODULE_ENABLED
#define TIM_MODULE_ENABLED

// Metro rev A with the separate Ethernet transformer and RJ45.
#define NUM_DIGITAL_PINS 38
#define NUM_ANALOG_INPUTS 6
#define ADC_RESOLUTION 12
#define PB7 0
#define PB6 1
#define PB8 2
#define PB9 3
#define PC0 4
#define PC1 5
#define PC2 6
#define PC3 7
#define PA0 8
#define PA1 9
#define PA2 10
#define PA8 11
#define PA9 12
#define PB12 13
#define PA4 14
#define PA3 15
#define PB0 16
#define PB1 17
#define PC4 18
#define PC5 19
#define PB11 20
#define PB10 21
#define PA7 22
#define PA6 23
#define PA5 24
#define PC14 25
#define PC15 26
#define PA15 27
#define PB3 28
#define PB4 29
#define PB5 30
#define PC12 31
#define PD2 32
#define PB14 33
#define PB15 34
#define PC10 35
#define PC11 36
#define PA10 37

#define LED_BUILTIN 13
#define PIN_NEOPIXEL PC14
#define PIN_BUTTON PC15
#define USER_BTN PIN_BUTTON
#define PIN_SPI_SS 10
#define PIN_SPI_MOSI PA7
#define PIN_SPI_MISO PA6
#define PIN_SPI_SCK PA5
#define PIN_WIRE_SDA PB11
#define PIN_WIRE_SCL PB10
#define PIN_FLASH_CS PA15
#define PIN_FLASH_SCK PB3
#define PIN_FLASH_MISO PB4
#define PIN_FLASH_MOSI PB5
#define PIN_SD_DETECT PA10
#define PIN_SD_CLK PC12
#define PIN_SD_CMD PD2
#define PIN_SD_D0 PB14
#define PIN_SD_D1 PB15
#define PIN_SD_D2 PC10
#define PIN_SD_D3 PC11
#define NEOPIXEL_NUM 1
#define BOARD_DEFAULT_SERIAL 1
#define PIN_SERIAL1_RX 0
#define PIN_SERIAL1_TX 1
#define TIMER_TONE TIM6
#define TIMER_SERVO TIM7

// PB10/PB11 use the I3C hardware controller in legacy I2C mode.
// PD8/VDD18 is connected to supply capacitors and is deliberately not exposed.
#if defined(CH32V467)
#define USB_PRODUCT "Adafruit Metro CH32V467"
#else
#define USB_PRODUCT "Adafruit Metro CH32V407"
#endif
#define SerialUSB SerialTinyUSB
