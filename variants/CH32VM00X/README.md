## CH32VM00X Boards: CH32V002, CH32V006, etc. ##

Currently supported board variants (listed in **bold**):
|        |  QFN32  |QSOP24 |QFN20|TSSOP20 |SOP16|QFN12|SOP8 | |
|--------|--------|--------|-----|--------|------|-----|-----|---|
|CH32V002|_n/a_   |_n/a_   |**F4U6**|**F4P6**|**A4M6**|**D4U6**| J4M6 | |
|CH32V004|_n/a_   |_n/a_   | F6U1 | F6P1 |_n/a_ |_n/a_|_n/a_| |
|CH32V005|_n/a_   | E6R6   | F6U6 | F6P6 |_n/a_ | D6U6 |_n/a_| |
|CH32V006|**K8U6**|**E8R6**|**F8U6**|**F8P6**|_n/a_ |_n/a_|_n/a_| |
|CH32V007| K8U    | E8R    |_n/a_| _n/a_  |_n/a_ |_n/a_|_n/a_| |
|CH32M007| K8U7   |**E8R6**|_n/a_| _n/a_  |_n/a_ |_n/a_|_n/a_| * |

 - See [notes below](#known-issues--limitations) for known issues and installation instructions.
 - CH32M007 also available in QFN26/QSOP28 (E8U7/G8R6 - no specific support yet).

------------------

### TSSOP20 PINOUT ###
Arduino pin numbers D0-D17, A0-A7.
Shared pinout for CH32V003F4P6 / CH32V002F4P6 / CH32V006F8P6.

```
                +-----v-----+
      A7/D14  1-+PD4     PD3+-20 D13/A4
   TX/A5/D15  2-+PD5     PD2+-19 D12~/A3
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
- Note *: On V006F8 PD7 is shorted to PA4. Event though PA4 has no pin, they cannot both be configured as output.
- Note ': PC4 is the default software chip-select pin (PIN_SPI_SS). Control this pin as a GPIO. Hardware SPI NSS/SSEL uses PC1, which is also the default I2C SDA pin. These two peripheral functions cannot use PC1 at the same time. Do not pass PC4 as the hardware SSEL argument to SPIClass. Using alternative pin mappings now requires changing PeripheralPins.c, but may get core API support in a future release. See issue #61.

#### More pinout diagrams
|  |  |  |  |
| -- | -- | -- | -- |
| V002 | [A4M6 SOP16](./CH32V002A4/README.md#pinout) | [D4U6 QFN12->DIP14](./CH32V002D4/README.md#pinout) | [J4M6 SOP8](./CH32V002J4/README.md#pinout)  |
| V006 | [E8R6 QSOP24](./CH32V006E8/README.md#ch32v006e8r6-qsop24-pinout) | [F8P6 TSSOP20](./CH32V006F8/README.md#ch32v006f8p6-tssop20-pinout) | |
| VM007 | [M007E8R6 QSOP24](./CH32M007E8/README.md#qsop24-pinout) | | |

### Feature comparison with V003 (TSSOP20) ###

| Feature | V003   | V002   | V004   | V005   | V006   |
| ------- | ------ | ------ | ------ | ------ | ------ |
| Flash   | 16KB   | 16KB   | 32KB   | 32KB   | 62KB   |
| RAM     | 2KB    | 4KB    | 6KB    | 6KB    | 8KB    |
| UserMem | 64B    | 256B   | 256B   | 256B   | 256B   |
| ADC     | 10-bit | 12-bit | 12-bit | 12-bit | 12-bit |
| OPA     | 1-ch   |        |        | 1-ch   | 3-ch   |
| Touch   |        |        |        |        | 8x     |
| Serial  | 1x     | 1x     | 1x     | 1x     | 2x     |
| Timers  | 2+2 WD | 2+2 WD | 2+2 WD | 2+2 WD | 3+2 WD |
| VCC     | 2v7~5v5| 2~5V   | 2~5V   | 2~5V   | 2~5V   |
| ISA     | RV32EC | RV32EmC| RV32EmC| RV32EmC| RV32EmC|

### Documentation / Links ###
- Datasheets: [CH32V002](https://www.wch-ic.com/downloads/CH32V002DS0_PDF.html), [CH32V004](https://www.wch-ic.com/downloads/CH32V004DS0_PDF.html), [CH32V005/CH32V006](https://www.wch-ic.com/downloads/CH32V006DS0_PDF.html),  [CH32V007/CH32M007](https://www.wch-ic.com/downloads/CH32V007DS0_PDF.html), 
- Reference manual: [CH32V00X](https://www.wch-ic.com/downloads/CH32V00XRM_PDF.html)

### Known Issues & Limitations ###
- The boards file has provisions for selecting peripherals from the menu. This feature is aimed at chips with limited flash (think V003/V002) and is not implemented on all boards. (To support other boards the menu options need to be copied and `#ifndef IDE_MENU_PERIPHERALS` should be added to the variant specific file variant_CH32XnnnYYZZ.h) [TODO, see #7] 
- To fix the clock capitalization bug, minimal changes were made. Not all changes from the latest SDK were incorporated yet. [WIP: #15 #70 #71 #72]
- To see the updated boards menu after installation of WCH core v1.0.4, you may need to [clear the IDE 2.x application cache](https://github.com/ch32-community/arduino_core_ch32/wiki/Clearing-Arduino-2.x-IDE-application-cache).
- Uploading and debugging the new boards using the Arduino IDE 2.x, requires an [updated version of OpenOCD](#updating-openocd).

------------------

### Fresh installation with support for CH32VM00X (V002/V006/etc) ###

1. Install [latest WCH Link Utility](https://www.wch.cn/downloads/WCH-LinkUtility_ZIP.html), update LinkE device when prompted. Test connection to Link Utility and CH32 chip using Query Chip Info. (This step installs drivers and support for CH32V006 and other newer chips).
2. Install latest Arduino IDE (v2.3.10).
3. Run Arduino IDE as administrator, Add [board json](https://github.com/ch32-community/board_manager_files/raw/main/package_ch32-commmunity_index.json) to preferences:
    `https://github.com/ch32-community/board_manager_files/raw/main/package_ch32-commmunity_index.json`
Open Tools → Board → Boards Manager, search for wch, and click Install. Exit IDE.
5. Optional: For latest sources download [zip of latest master branch](https://github.com/ch32-community/arduino_core_ch32/archive/refs/heads/main.zip). Unzip and copy/overwrite all contents to this location: 
   - `C:\Users\<username>\AppData\Local\Arduino15\packages\ch32-community\hardware\ch32v\[release number]`
6. Optional: review [unmerged PRs](https://github.com/ch32-community/arduino_core_ch32/pulls) for possibly required fixes and manually copy changed files.
7. Optional: for uploading to new chip: [update OpenOCD](#updating-openocd) obtained from MounRiver Studio.
8. Restart IDE. If new boards are not visible: [clear IDE application cache](https://github.com/ch32-community/arduino_core_ch32/wiki/Clearing-Arduino-2.x-IDE-application-cache).
9. Select your CH32 board. Test uploading simple serial example. Test debugging.

### Updating OpenOCD ###
To support the new CH32VM00X boards (CH32V002, CH32V006, etc), the installation done when installing CH32 Arduino core v 1.0.4 may need to be updated.
This can be done by copying it from a recent installation of MounRiver Studio.

These are the steps I took:

After installation of MounRiver Studio (in Windows 11 x64 under VMware Workstation Player) following all defaults, WCH OpenOCD v2.3 was found here:
`C:\MounRiver\MounRiver_Studio2\resources\app\resources\win32\components\WCH\OpenOCD`

I copied the contents of that folder to a new 2.3.0 folder (next to the 1.0.0 folder) in 
-  `C:\Users\<username>\AppData\Local\Arduino15\packages\WCH\tools\openocd`

Then I changed this file:
-  `C:\Users\<username>\AppData\Local\Arduino15\packages\WCH\hardware\ch32v\1.0.4\platform.txt`
Use search and replace to change "openocd-1.0.0" into "openocd-2.3.0" in the sections `# Uploader tool` and `# Debugger configuration`.

After restarting the Arduino IDE (v2.3.6) I could successfully upload my sketch and use the interactive debugger. Yippie! 
Thanks @tako0910 for pointing out this OpenOCD update.
