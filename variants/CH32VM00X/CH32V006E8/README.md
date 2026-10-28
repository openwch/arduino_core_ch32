

### CH32V006 QSOP24 PINOUT ###
- Arduino pin numbers D0-D21, A0-A7.
- named pins QSOP24: PA0-5, PB0/1/3, PC0-2/4/5, PD0-7 (TSSOP20: PA1/2, PC0-7, PD0-7)

CH32V006E8R6 Pinout 
```
                   +-----v-----+
       SWIO D18~ 1-+PD1     PB3+-24  D17 MISO
            D19~ 2-+PC0     PB1+-23  D16
        SDA D20  3-+PC1     PB0+-22  D15
        SCL D0   4-+PC2     PA3+-21  D14 
            GND  5-+VSS     PD0+-20 ~D13 
            VCC  6-+VDD     PA2+-19 ~D12 A0 
        RST D21  7-+PC5     PA0+-18  D11  
      SS/A2 D1~  8-+PC4     PA1+-17  D10* A1  
     SCK/A3 D2~  9-+PD2     PA5+-16  D9  RX2
    MOSI/A4 D3~ 10-+PD3     PA4+-15  D8  TX2
         A7 D4  11-+PD4     PD7+-14 ~D7
      TX/A5 D5  12-+PD5     PD6+-13  D6 RX/A6
                   +-----------+        
```
*Note: on CH32V006E8R6 (and CH32V005E6R6) pins PA1 and PA6 are shorted => not usable as output pin?


### CH32V006 QSOP24 UART/I2C/SPI pins ###
```
        _1  _2  _3  _4 _5 _6
MISO  x  x  24   x  12  x  x    
MOSI  x  x  10   2  13 19  x
SCK   7  7   9  23  11 17* x
SS    3  2   8  22  10  3  8
CH32V006E8R6 ==> use SPI_2 on MISO=24/MOSI=10/SCK=9/SS=8

        _1  _2  _3  _4 _5 _6 _7 _8 _9
TX1  12 13  20   2   1 24  7  x 18 18
RX1  13 12   1   3  24  1  x  x 17  8
TX2   x 15  19   9  22  8 17*
RX2  24 16  21  10  23  1 16

        _1  _2  _3  _4
SDA  3  20   x   x   1
SCL  4   1   7   x  24
```
