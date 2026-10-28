## CH32V002A4M6 ##


### PINOUT ###

CH32V002A4M6 SOP16 PINOUT
```
              +-----v-----+
RX'/SDA D3  1-+PC1     PC0+-16 D2~/TX'
    SCL D4  2-+PC2     VDD+-15 VCC
        D5~ 3-+PC3     VSS+-14 GND
        D6~ 4-+PC4     PA2+-13 D1/A0
        D8  5-+PC6     PA1+-12 D0~/A1
     SS/D9  6-+PC7     PD7+-11 RST
      SWIO  7-+PD1     PD6+-10 RX/A6/MOSI
SCK/D14/A7  8-+PD4     PD5+-9  TX/A5/MISO
              +-----------+
```              

### MAIN FEATURES/LIMITATIONS ###

 - Max 14 I/O pins, PWM/ADC/I2C/UART
 - SPI is available (with software SS on PC7)
 - SPI pins MOSI/MISO are shared with serial TX/RX pins when I2C is enabled
 - TX'/RX' - when I2C is enabled, TX/RX=PD5/PD6, when not enabled TX/RX=PC0/PC1
 - Requires updated core and tools. See [these notes](../README.md#known-issues--limitations)
