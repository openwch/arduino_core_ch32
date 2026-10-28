## CH32V002D4U6 ##


### PINOUT ###

V002 QFN12->DIP14 DEV-BOARD PINOUT
```
               +-----v------+
     D0~/A1 1 -+PA1      PD7+- 14  D10~/RST
      D1/A0 2 -+PA2      PD4+- 13  D9/A3
  TX/SDA/D7 3 -+PD0      PD1+- 12  SWIO/D8/SCL/RX
        3V3 4 -+VDD  /\  PC7+- 11  D6
         D2 5 -+PC0  \/  PC6+- 10  D5
        D3~ 6 -+PC3      PC4+- 9   D4~/A2
        3V3 7 -+VDD+     VSS+- 8   GND
               +------------+        
```

### MAIN FEATURES/LIMITATIONS ###

 - Up to 11 I/O pins, PWM/ADC/I2C/UART
 - SPI is not available (no SCK pin)
 - Shared UART/I2C pins (SCL and RX are shared with SWIO pin)
 - Requires updated core and tools. See [these notes](../README.md#known-issues--limitations)
