# Arduino core support for CH32 EVT Boards

* [Introduction](https://github.com/openwch/arduino_core_ch32#Introduction)<br>
* [How to use](https://github.com/openwch/arduino_core_ch32#How-to-use)<br>
* [Supported boards](https://github.com/openwch/arduino_core_ch32#Supported-boards)<br>
* [OS support](https://github.com/openwch/arduino_core_ch32#OS-support)<br>
* [Submit bugs](https://github.com/openwch/arduino_core_ch32#Submit-bugs)<br>

## Introduction

This repo adds the support of CH32 MCU in Arduino IDE.<br>

The file includes:
* [Arduino_Core_CH32](https://github.com/openwch/arduino_core_ch32):Public library files.
* [openocd](https://github.com/openwch/openocd_wch):can directly use WCH-LINKE to download and debug wch chips.
* [riscv-none-embed-gcc](https://github.com/openwch/risc-none-embed-gcc):A toolchain that supports WCH custom half word and byte compression instruction extensions and hardware stack push/pop functions.

## How to use

You can add this software package directly on the IDE through the [Arduino Boards Manager](https://www.arduino.cc/en/guide/cores).

Add the following link in the "*Additional Boards Managers URLs*" field:

https://github.com/openwch/board_manager_files/raw/main/package_ch32v_index.json

Then you can search for "**wch**" through the "**board manager**", find the installation package, and install it.

## Supported boards

It will be a long-term support and maintenance project, unless we encounter force majeure factors.The current version supports the following development boards:

- [CH32V00x EVT Boards](#CH32V00x-EVT-Boards)
- [CH32V10x EVT Boards](#CH32V10x-EVT-Boards)
- [CH32V20x EVT Boards](#CH32V20x-EVT-Boards)
- [CH32V30x EVT Boards](#CH32V30x-EVT-Boards)
- [CH32X035 EVT Boards](#CH32X035-EVT-Boards)

### CH32V00x EVT Boards

| Status | Boards name | Peripherals | Release | Notes |
| :----: |     ----    |     ----    | :-----: | :---- |
| :heavy_check_mark: | CH32V003F4P | ADC,DAC,USART,GPIO,EXTI,SysTick | 1.0.0 | SPI,I2C_Master since 1.0.2 |

### CH32V20x EVT Boards

| Status | Boards name | Peripherals | Release | Notes |
| :----: |     ----    |     ----    | :-----: | :---- |
| :heavy_check_mark: | CH32V203G8U | ADC,DAC,USART,GPIO,EXTI,SysTick | 1.0.0 | SPI,I2C_Master since 1.0.2 |

### CH32X035 EVT Boards

| Status | Boards name | Peripherals | Release | Notes |
| :----: |     ----    |     ----    | :-----: | :---- |
| :heavy_check_mark: | CH32X035G8U | ADC,DAC,USART,GPIO,EXTI,SysTick | 1.0.1 | SPI,I2C_Master since 1.0.2 | 

### CH32V10x EVT Boards

| Status | Boards name | Peripherals | Release | Notes |
| :----: |     ----    |     ----    | :-----: | :---- |
| :heavy_check_mark: | CH32V103R8T6_BLACK | ADC,DAC,USART,GPIO,EXTI,SysTick,SPI,I2C_Master | 1.0.3 | - |

### CH32V30x EVT Boards

| Status | Boards name | Peripherals | Release | Notes |
| :----: |     ----    |     ----    | :-----: | :---- |
| :heavy_check_mark: | CH32V307VCT6_BLACK | ADC,DAC,USART,GPIO,EXTI,SysTick,SPI,I2C_Master | 1.0.3 | - |


## OS support

Adopting toolchain and openocd under [MRS](http://www.mounriver.com/), supporting HPE, custom byte and half-word compression extensions,"upload" via WCH_LINKE. 

**Most importantly, the version of Arduino IDE is 2.0+.**

### Win

If you encounter an error during upload, please confirm that the version of your WCH-LINKE is consistent with the latest version under MRS. 
WCH-LINKE related information can [refer to this link](https://github.com/openwch/ch32v307/tree/main/WCH-Link). 

### Linux

For Linux, after installing the support package for the first time, to ensure the normal upload function, 
please open the packages installation path of the Arduino IDE, run the script, and automatically configure the environment.

Usually, it can be operated as follows:<br>

```bash
cd ~/.arduino15/packages/WCH/tools/beforeinstall/1.0.0
./start.sh
```
After authorization, it will copy or generate some necessary libraries and rules:

```text
Copy Libs
[sudo] password for temperslee: 
Register new Libs
copy rules
Reload rules
DONE
```

### MAC

For MAC, please install the "libusb" library before starting to use it.
```bash
brew install libusb
```
If "libusb" related errors still occur when "uploading" firmware after installing the libusb library, 
please contact the **MRS team** for assistance through "*support@mounriver.com*".


## Submit bugs

If you have any questions, you could contact me through the email "*yy@wch.cn*".
Or you could [file an issue on GitHub](https://github.com/openwch/arduino_core_ch32/issues/new).


