

### CH32V006E8R6 QSOP24 PINOUT ###
Arduino pin numbers D0-D21, A0-A7.

The E8 (QSOP24) package has more pins than F8 (QFN20/TSSOP20), but also some other pin-differences: 
- F8: PA1/2, PC0-7, PD0-7
- E8: PA0-5, PB0/1/3, PC0-2/4/5, PD0-7

Pinout for CH32V006E8R6 QSOP24:

```
                   +-----v-----+
       SWIO D18~ 1-+PD1     PB3+-24  D17 MISO
            D19~ 2-+PC0     PB1+-23  D16
        SDA D20  3-+PC1     PB0+-22  D15
        SCL D0   4-+PC2     PA3+-21  D14 
            GND  5-+VSS     PD0+-20 ~D13 
            VCC  6-+VDD     PA2+-19 ~D12 A0 
        RST D21  7-+PC5     PA0+-18  D11  
      SS/A2 D1~  8-+PC4    *PA1+-17  D10 A1  
     SCK/A3 D2~  9-+PD2     PA5+-16  D9  RX2
    MOSI/A4 D3~ 10-+PD3     PA4+-15  D8  TX2
         A7 D4  11-+PD4     PD7+-14 ~D7
      TX/A5 D5  12-+PD5     PD6+-13  D6 RX/A6
                   +-----------+        
```
*Note: on CH32V006E8R6 (and CH32V005E6R6) pins PA1 and PA6 are shorted => pin 17 only usable as output pin when PA6 is unassigned or set as input.

Alternate pins for V006E8:
```
SPI           _1  _2  _3  _4  _5  _6
SPI_NSS  PC1 PC0 PC4 PB0 PD3 PC1 PC4
SPI_SCK  PC5 PC5 PD2 PB1 PD4 PA1 PB5
SPI_MISO PC7 PC7 PB3 PB2 PD5 PB5 PC7
SPI_MOSI PC6 PC6 PD3 PC0 PD6 PA2 PB4 
Only B0/1/3, no PC6/7 ==> use SPI_2 on MISO=24/MOSI=10/SCK=9/SS=8 (_4 clashes with UART1)

USART     _1  _2  _3  _4  _5  _6  _7  _8  _9
TX1  PD5 PD6 PD0 PC0 PD1 PB3 PC5 PB5 PA0 PA0 
RX1  PD6 PD5 PD1 PC1 PB3 PD1 PC6 PB6 PA1 PC4 
CTS1 PD3 PC6 PC3 PC6 PD7 PD7 PC7 PC7 PD2 PD5 
RTS1 PC2 PC7 PC2 PC7 PA5 PA5 PB4 PB4 PD3 PD4 
TX2  PA7 PA4 PA2 PD2 PB0 PC4 PA6 ... ... ... 
RX2  PB3 PA5 PA3 PD3 PB1 PD1 PA5 ... ... ... 
CTS2 PA4 PA7 PA0 PA0 PB6 PA4 PA7 ... ... ... 
RTS2 PA5 PB3 PA1 PA1 PA1 PA1 PB3 ... ... ... 
Only B0/1/3, no PC6/7 or PA6/7 ==> USART1_2/4/5/8/9 and USART2_2/3/5 as alternatives

I2C          _1  _2  _3  _4
I2C_SCL PC2 PD1 PC5 PB5 PB3
I2C_SDA PC1 PD0 PC6 PB6 PD1
Only B0/1/3, no PC6/7 => I2C_1/4 as alternatives
```
