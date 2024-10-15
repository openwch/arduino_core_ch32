## CH32X033F8P6 ##

### CH32X035/X033 Main Features ###
-	48MHz RC oscillator
-	20KB SRAM
-	62KB Flash + 3328B System Flash
-	256B system config + 256B user storage
-	2x OPA, 3x CMP, 12-bit 14-chn ADC, 14-chn touch
-	3x 16-bit timer, 2x WD timer, systick
-	4x USART, I2C, SPI, USB2, USB PD, 2-wire debug


### TSSOP20 Pinout ###
- See [datasheet](https://www.wch.cn/downloads/CH32X035DS0_PDF.html) for complete pin function list.
```
                  +------v------+
    MISO/A6 D6~ 1-+PA6       PA5+-20  D5 SCK/A5
MOSI/A8/TX4 D7  2-+PA7=PB0   PA4+-19  D4~ CS/A4
     A9/RX4 D8  3-+PB1      PC19+-18  D17 SWCLK
        RST D9  4-+PB7       PA3+-17  D3~ RX2/A3*
     USBDM D10  5-+PC16=PC11 PA2+-16  D2~ TX2/A2
     USBDP D11  6-+PC17=PC10 PA1+-15  D1~ A1
           GND  7-+VSS       PA0+-14  D0~ A0
     SWDIO D16  8-+PC18      PC3+-13  D15~ A13
           VCC  9-+VDD      PA10+-12  D14/SCL
           D12 10-+PA9      PA11+-11  D13/SDA
                  +-------------+
*A3 and I2C don't work on CH32X033F8P6 0-series (lot number with the penultimate bit 5 being 0).
```


### Tested features ###
- digitalWrite()/digitalRead()
- analogWrite() - 12-bit resolution, marked with ~ in pinout above.
- analogRead() - very stable 12-bit resolution
- EEPROM library - may need improvement (first test showed corrupted data written)

### Known issues/limitations ###
- Pins PA7/PB0, PC16/PC11 and PC17/PC10 cannot be used for output
- Any signal on A0 seems to show on other ADC pins when disconnected.
- A3 and I2C don't work on CH32X033F8P6 0-series (lot number with the penultimate bit 5 being 0)

### References ###
- [datasheet](https://www.wch.cn/downloads/CH32X035DS0_PDF.html)
- [reference manual](https://www.wch.cn/downloads/CH32X035RM_PDF.html)
- [PIOC](https://github.com/openwch/ch32x035/tree/main/EVT/EXAM/PIOC) [User manual](https://github.com/openwch/ch32x035/blob/main/EVT/EXAM/PIOC/PIOC%20UserManual.pdf)), [Reference manual](https://github.com/openwch/ch32x035/blob/main/EVT/EXAM/PIOC/PIOC-EN.pdf)), [Instruction set](https://github.com/openwch/ch32x035/blob/main/EVT/EXAM/PIOC/CHRISC8B-EN.pdf)
- [X033/X035 ](https://github.com/openwch/ch32x035), [WCH examples](https://github.com/openwch/ch32x035/blob/main/EVT/EXAM)
- [PIOC](https://github.com/openwch/ch32x035/tree/main/EVT/EXAM/PIOC) [User manual](https://github.com/openwch/ch32x035/blob/main/EVT/EXAM/PIOC/PIOC%20UserManual.pdf)), [Reference manual](https://github.com/openwch/ch32x035/blob/main/EVT/EXAM/PIOC/PIOC-EN.pdf)), [Instruction set](https://github.com/openwch/ch32x035/blob/main/EVT/EXAM/PIOC/CHRISC8B-EN.pdf)
