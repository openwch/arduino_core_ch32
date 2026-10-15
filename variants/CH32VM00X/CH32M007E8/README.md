## CH32M007E8R6 ##


### QSOP24 PINOUT ###
Arduino pin numbers D0-D20, A1-A7 (A0 is unavailable because PA2 is occupied by LO2).

CH32M007E8R6 QSOP24 PINOUT - 6-24V
```
                   +-----v-----+
       SWIO D18  1-+PD1     PA1+-24 ~D17 A1
     SCK/A3 D19~ 2-+PD2     PC5+-23  D16 RST
    MOSI/A4 D20~ 3-+PD3     PC4+-22 ~D15 SS/A2
          A7 D0  4-+PD4     PC2+-21  D14 SCL 
       A5 TX/D1  5-+PD5     PC1+-20  D13 SDA
       A6 RX/D2  6-+PD6     PB3+-19  D12 MISO
             D3~ 7-+PD7     PD0+-18  D11 LO3  
         TX2/D4  8-+PA4     PB1+-17  D10 HO3
         RX2/D5  9-+PA5     PA2+-16  D9  LO2
            VDD 10-+VDD     PB0+-15  D8  HO2
            VSS 11-+VSS     PA0+-14  D7  LO1
          6-24V 12-+VHV     PA3+-13  D6  HO1
                   +-----------+
```


### MAIN FEATURES/LIMITATIONS ###

 - Max 15 I/O pins, PWM/ADC/I2C/2xUART/SPI
 - I2C uses SDA/SCL=PC1/PC2
 - SPI uses pin mapping SPI_2: MOSI/MISO/SCK/SS are PD3/PB3/PD2/PC4
 - Serial1 (USART1) uses: TX/RX=PD5/PD6
 - Serial2 (USART2) uses pin mapping UART2_1: TX2/RX2=PA4/PA5
 - Requires updated core and tools. See [these notes](../README.md#known-issues--limitations)
