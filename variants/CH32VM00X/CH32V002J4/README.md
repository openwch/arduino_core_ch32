## CH32V002J4M6 ##


### PINOUT ###

CH32V002J4M6 SOP8 PINOUT
```
                  +-----v-----+
  TX'/A6/RX/D8~ 1-+PD6     PD1+-8 ~D5/SWIO
         A1/D0~   +PA1   PD5/4+   ~D7/A5/TX ~D6/A7/RX'
           GND  2-+VSS     PC4+-7 ~D4/A2
                  +           +
         A0/D1~ 3-+PA2     PC2+-6 ~D3/SCL
                  +           +
           VCC  4-+VDD     PC1+-5 ~D2/SDA
                  +-----------+
```

### MAIN FEATURES/LIMITATIONS ###

 - Max 6 I/O pins, PWM/ADC/I2C/UART. No SPI!
 - Pinout is similar to CH32V003J4M6, but there are functional differences.
 - Pin 8 combines many pins, including SWIO. Beware of bricking! 
 - Pin 1 also combines multiple pins. Only one can be used as output!
 - Don't use more than one output for combined pins. PD6=A1, PD1=PD4=PD5.
 - TX'/RX' - Serial1_1 is board default to allow Serial.print without unbricking procedure.
 - Serial1_2 and Serial1_3 are not available on CH32V002J4M6.
 - I2C slave mode can be used to make an 8-pin I2C device.
 - Hardware SPI is unavailable. Software SPI is possible.
 - Do not call `setRx()/setTx()/setRts()/setCts()` without validating pin combinations against
   the V002 Datasheet or Reference Manual. For example: don't use `setRts(PC2)` with the
   board-default Serial1_1 mapping. Configuring PC2 as RTS disables the USART1 remap, 
   changing TX/RX from PD6/PD5 to PD5/PD6. PD5 also shares the SWIO pin.

#### SWIO pin usage - Beware of bricking!

 - Using either PD1, PD4 or PD5 will block the SWIO function.
 - When selecting Serial=Serial1 (TX/RX=PD5/PD6), using Serial will block the SWIO function. 
 - To unbrick use WCH Link Utility option "Clear All Code by Power Off".
