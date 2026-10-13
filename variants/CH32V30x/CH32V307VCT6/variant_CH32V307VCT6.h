/*
 *******************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * This software component is licensed by WCH under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#pragma once

/* ENABLE Peripherals */
#define                         ADC_MODULE_ENABLED
#define                         UART_MODULE_ENABLED
#define                         DAC_MODULE_ENABLED
#define                         SPI_MODULE_ENABLED  
#define                         I2C_MODULE_ENABLED
#define                         TIM_MODULE_ENABLED    

/* CH32V307VCT6 Pins */
#define PA0                     PIN_A0
#define PA1                     PIN_A1
#define PA2                     PIN_A2
#define PA3                     PIN_A3
#define PA4                     PIN_A4
#define PA5                     PIN_A5

#define PA10                    6
#define PA9                     7
#define PA8                     8                     
#define PA7                     9
#define PA6                     10
#define PB5                     11 
#define PB8                     12
#define PB9                     13
#define PB1                     14
#define PB0                     15
#define PB12                    16
#define PB15                    17       
#define PB14                    18
#define PB13                    19
#define PB11                    20
#define PB10                    21 


// Alternate pins number
#define PA0_ALT1                (PA0  | ALT1) 
#define PA1_ALT1                (PA1  | ALT1)
#define PA2_ALT1                (PA2  | ALT1)
#define PA3_ALT1                (PA3  | ALT1)
#define PA4_ALT1                (PA4  | ALT1)
#define PA5_ALT1                (PA5  | ALT1)

#define PA4_ALT2                (PA4  | ALT2) //for DAC_Channel1
#define PA5_ALT2                (PA5  | ALT2) //for DAC_Channel2



// MxRD 261012: TODO: the number of pins below and the definitions above are very much incomplete!
// Most CH32V30x variants have more than 22 pins (ranging from 17 for V305FB to 80 for 303VC/307VC packages).
// See also V10x issue openwch#210, resolved by PR #22 (openwch#211), which was very much the same thing.
// These numbers/definitions were not updated since the first release in 2023. 
// Probably there are more variants suffering from the same issue. TODO: need to check those!
#define NUM_DIGITAL_PINS        22
#define NUM_ANALOG_INPUTS       6
// #define ADC_CTLR_ADCAL          
#define ADC_RESOLUTION          12


// On-board LED pin number
#ifndef LED_BUILTIN
  #define LED_BUILTIN           PNUM_NOT_DEFINED
#endif



// On-board user button
#ifndef USER_BTN
  #define USER_BTN              PNUM_NOT_DEFINED
#endif

// SPI definitions
#ifndef PIN_SPI_SS
  #define PIN_SPI_SS            PA4
#endif
#ifndef PIN_SPI_SS1
  #define PIN_SPI_SS1           PNUM_NOT_DEFINED
#endif
#ifndef PIN_SPI_SS2
  #define PIN_SPI_SS2           PNUM_NOT_DEFINED
#endif
#ifndef PIN_SPI_SS3
  #define PIN_SPI_SS3           PNUM_NOT_DEFINED
#endif

#ifndef PIN_SPI_MOSI
  #define PIN_SPI_MOSI          PA7
#endif
#ifndef PIN_SPI_MISO
  #define PIN_SPI_MISO          PA6
#endif
#ifndef PIN_SPI_SCK
  #define PIN_SPI_SCK           PA5
#endif

// I2C definitions
  #define PIN_WIRE_SDA          PB11
  #define PIN_WIRE_SCL          PB10

// Timer Definitions
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM3
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM2
#endif


// UART Definitions
// CH32V30x: Serial1-Serial8 on V307 amd V303RC/VC, only 1-3 on V303CB/RB, only 1/3 on V305FB, 1-5 ON V305GB/CC/RB.
//    RX: PA10=RX1, PA3=RX2, PB11=RX3, PC11=RX4, PD2=RX5, PC1=RX6, PC3=RX7, PC5=RX8
//    TX: PA9=TX1, PA2=TX2, PB10=TX3, PC10=TX4, PC12=TX5, PC0=TX6, PC2=TX7, PC4=TX8
#ifndef SERIAL_UART_INSTANCES
  #define SERIAL_UART_INSTANCES 1
#endif

#if (SERIAL_UART_INSTANCES == 1)
  //  Using only one Serial port
  #ifndef SERIAL_UART_INSTANCE
    #define SERIAL_UART_INSTANCE 1   // use Serial1 as default for Serial
  #endif
#else
  // Using multple Serial ports
  // When having multiple Serial instances SERIAL_UART_INSTANCE cannot be defined as it will skip defining the other ports.
  #undef SERIAL_UART_INSTANCE
  #define ENABLE_HWSERIAL1 1
  #define ENABLE_HWSERIAL2 1
  #define ENABLE_HWSERIAL3 1
  #define ENABLE_HWSERIAL4 1
  #define ENABLE_HWSERIAL5 1
  #define ENABLE_HWSERIAL6 1
  #define ENABLE_HWSERIAL7 1
  #define ENABLE_HWSERIAL8 1
  #define Serial Serial1
#endif

// Default pin used for generic 'Serial' instance
// Mandatory for Firmata, no longer used by HardwareSerial.
#if SERIAL_UART_INSTANCE==1 || SERIAL_UART_INSTANCES>1
    #ifndef PIN_SERIAL_RX
        #define PIN_SERIAL_RX     PA10
    #endif
    #ifndef PIN_SERIAL_TX
        #define PIN_SERIAL_TX     PA9
    #endif
#elif  SERIAL_UART_INSTANCE==2
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX       PA3
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX       PA2
    #endif
#elif  SERIAL_UART_INSTANCE==3
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX       PB11
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX       PB10
    #endif
#elif  SERIAL_UART_INSTANCE==4
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX       PC11
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX       PC10
    #endif
#elif  SERIAL_UART_INSTANCE==5
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX       PD2
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX       PC12
    #endif
#elif  SERIAL_UART_INSTANCE==6
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX       PC1
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX       PC0
    #endif
#elif  SERIAL_UART_INSTANCE==7
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX       PC3
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX       PC2
    #endif
#elif  SERIAL_UART_INSTANCE==8
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX       PC5
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX       PC4
    #endif
//    RX: PA10=RX1, PA3=RX2, PB11=RX3, PC11=RX4, PD2=RX5, PC1=RX6, PC3=RX7, PC5=RX8
//    TX: PA9=TX1, PA2=TX2, PB10=TX3, PC10=TX4, PC12=TX5, PC0=TX6, PC2=TX7, PC4=TX8
#endif


/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
  // These serial port names are intended to allow libraries and architecture-neutral
  // sketches to automatically default to the correct port name for a particular type
  // of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
  // the first hardware serial port whose RX/TX pins are not dedicated to another use.
  //
  // SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
  //
  // SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
  //
  // SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
  //
  // SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
  //
  // SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
  //                            pins are NOT connected to anything by default.
  #ifndef SERIAL_PORT_MONITOR
    #define SERIAL_PORT_MONITOR   Serial
  #endif
  #ifndef SERIAL_PORT_HARDWARE
    #define SERIAL_PORT_HARDWARE  Serial
  #endif
#endif
