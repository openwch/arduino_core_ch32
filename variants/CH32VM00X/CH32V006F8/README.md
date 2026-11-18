### CH32V006F8P6 TSSOP20 PINOUT ###
Arduino pin numbers D0-D17, A0-A7.
Pinout for CH32V006F8P6 (same layout as CH32V003F4P6 / CH32V002F4P6).
Extra pins for Serial2
```
                +-----v-----+
      A7/D14  1-+PD4     PD3+-20 D13/A4/RX2
   TX/A5/D15  2-+PD5     PD2+-19 D12~/A3/TX2
   RX/A6/D16  3-+PD6     PD1+-18 D11/SWIO
     RST/D17~ 4-+PD7*    PC7+-17 D9/MISO
       A1/D0~ 5-+PA1     PC6+-16 D8/MOSI
       A0/D1  6-+PA2     PC5+-15 D7/SCK
         GND  7-+VSS    'PC4+-14 D6~/A2/SS (software CS)
         D10  8-+PD0     PC3+-13 D5~
         VCC  9-+VDD     PC2+-12 D4/SCL
         D2~ 10-+PC0    'PC1+-11 D3/SDA/NSS (hardware SSEL)
                +-----------+    
```
- Note *: On V006F8 PD7 is shorted to PA4
- Note ': PC4 is the default software chip-select pin (PIN_SPI_SS). Control this pin as a GPIO. Hardware SPI NSS/SSEL uses PC1, which is also the default I2C SDA pin. These two peripheral functions cannot use PC1 at the same time. Do not pass PC4 as the hardware SSEL argument to SPIClass.
Using alternative pin mappings now requires changing PeripheralPins.c, but may get core API support in a future release. See issue #61. 

Alternate pins for V006F8:

```
SPI           _1  _2  _3  _4  _5  _6
SPI_NSS  PC1 PC0 PC4 PB0 PD3 PC1 PC4
SPI_SCK  PC5 PC5 PD2 PB1 PD4 PA1 PB5
SPI_MISO PC7 PC7 PB3 PB2 PD5 PB5 PC7
SPI_MOSI PC6 PC6 PD3 PC0 PD6 PA2 PB4 
No port B on V006F8 ==> SPI_1 or SPI_4 as alternatives.


USART     _1  _2  _3  _4  _5  _6  _7  _8  _9
TX1  PD5 PD6 PD0 PC0 PD1 PB3 PC5 PB5 PA0 PA0 
RX1  PD6 PD5 PD1 PC1 PB3 PD1 PC6 PB6 PA1 PC4 
CTS1 PD3 PC6 PC3 PC6 PD7 PD7 PC7 PC7 PD2 PD5 
RTS1 PC2 PC7 PC2 PC7 PA5 PA5 PB4 PB4 PD3 PD4 
TX2  PA7 PA4 PA2 PD2 PB0 PC4 PA6 ... ... ... 
RX2  PB3 PA5 PA3 PD3 PB1 PD1 PA5 ... ... ... 
CTS2 PA4 PA7 PA0 PA0 PB6 PA4 PA7 ... ... ... 
RTS2 PA5 PB3 PA1 PA1 PA1 PA1 PB3 ... ... ... 
No port B or PA0/3-7 on V006F8 ==> USART1_1/2/3 and only USART2_3/5 (without CTS) as alternatives

I2C          _1  _2  _3  _4
I2C_SCL PC2 PD1 PC5 PB5 PB3
I2C_SDA PC1 PD0 PC6 PB6 PD1
No port B on V006F8 ==> I2C_1/2 as alternatives
```
