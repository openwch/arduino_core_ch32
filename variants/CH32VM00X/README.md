## CH32VM00X Boards: CH32V002, CH32V006, etc. ##

Currently supported board variants (listed in **bold**):
|        |  QFN32 |QSOP24|QFN20|TSSOP20 |SSOP16|QFN12|SOP8 | |
|--------|--------|------|-----|--------|------|-----|-----|---|
|CH32V002|_n/a_   |_n/a_ |**F4U6**|**F4P6**|**A4M6**|**D4U6**|     | |
|CH32V004|_n/a_   |_n/a_ |     |        |_n/a_ |_n/a_|_n/a_| |
|CH32V005|_n/a_   |      |     |        |_n/a_ |     |_n/a_| |
|CH32V006|**K8U6**|**E8R6**|**F8U6**|**F8P6**|_n/a_ |_n/a_|_n/a_| |
|CH32V007|        |      |_n/a_| _n/a_  |_n/a_ |_n/a_|_n/a_| |
|CH32M007|_n/a_   |**E8R6**|_n/a_| _n/a_  |_n/a_ |_n/a_|_n/a_| * |

 - See [notes below](#known-issues--limitations) for known issues and installation instructions.
 - CH32M007 also available in QFN26/QSOP28 (no specfic support yet).

------------------

### TSSOP20 PINOUT ###
Arduino pin numbers D0-D17, A0-A7.
Shared pinout for CH32V003F4P6 / CH32V002F4P6 / CH32V006F8P6.

```
                +-----v-----+
      A7/D14  1-+PD4     PD3+-20 D13/A4
   TX/A5/D15  2-+PD5     PD2+-19 D12~/A3
   RX/A6/D16  3-+PD6     PD1+-18 D11/SWIO
     RST/D17~ 4-+PD7     PC7+-17 D9/MISO
       A1/D0~ 5-+PA1     PC6+-16 D8/MOSI
       A0/D1  6-+PA2     PC5+-15 D7/SCK
         GND  7-+VSS     PC4+-14 D6~/A2/SS
         D10  8-+PD0     PC3+-13 D5~
         VCC  9-+VDD     PC2+-12 D4/SCL
         D2~ 10-+PC0     PC1+-11 D3/SDA
                +-----------+    
```

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
- The boards file in this PR has provisions for selecting peripherals from the menu. This feature is not (yet) implemented for all boards (currenly only for  V003/V002/V006/X035). To support other boards the menu options need to be copied and `#ifndef IDE_MENU_PERIPHERALS` should be added to the variant specific file variant_CH32XnnnYYZZ.h
- To fix the clock capitalization bug (see #66), minimal changes were made. Not all changes from the latest SDK were incorporated.
- This PR requires CH32 Arduino core with latest updates from the master branch (see [installation notes](#fresh-installation-with-support-for-ch32vm00x-v002v006etc)).
- To see the updated boards menu after installation of core v1.0.4, you may need to [clear the IDE 2.x application cache](#clearing-arduino-2x-ide-application-cache).
- Uploading and debugging the new boards using the Arduino IDE 2.x, requires an [updated version of OpenOCD](#updating-openocd).

------------------

### Fresh installation with support for CH32VM00X (V002/V006/etc) ###

1. Install [latest WCH Link Utility](https://www.wch.cn/downloads/WCH-LinkUtility_ZIP.html), update LinkE device when prompted. Test connection to Link Utility and CH32 chip using Query Chip Info. (This step installs drivers and support for CH32V006 and other newer chips).
2. Install latest Arduino IDE (v2.3.6).
3. Run Arduino IDE as administrator, Add [board json](https://github.com/openwch/board_manager_files/raw/main/package_ch32v_index.json) to preferences. Install WCH CH32 core v1.0.4. Exit IDE.
4. Download [zip of latest master branch](https://github.com/openwch/arduino_core_ch32/archive/refs/heads/main.zip). Unzip and copy/overwrite all contents to this location: 
   - `C:\Users\<username>\AppData\Local\Arduino15\packages\WCH\hardware\ch32v\1.0.4`
5. Optional: review [unmerged PRs](https://github.com/openwch/arduino_core_ch32/pulls) for possibly required fixes and manually copy changed files.
6. Optional: for uploading to new chip: [update OpenOCD](#updating-openocd) obtained from MounRiver Studio.
7. Restart IDE. If new boards are not visible: [clear IDE application cache](#clearing-arduino-2x-ide-application-cache).
8. Select CH32 board. Test uploading simple serial example. Test debugging.

### Clearing Arduino 2.x IDE application cache ###
When newly added menu items are not visible in the Arduino IDE v2.x, the cause can be an application cache that is not automatically refreshed.
Assuming you are using the Arduino IDE v2.x under Windows, the menu can be cached in such a way that restarting the IDE is insufficient. For instance when I added my definition for the X033 board (and later again for the the V002), it would not show the board to be selected. I had to exit the IDE, then remove this folder prior to restarting the IDE:
 - `C:\Users\<username>\AppData\Roaming\arduino-ide`

I read that this is a [known IDE 2.x issue](https://github.com/arduino/arduino-ide/issues/1030) and that it's okay to [delete the entire folder](https://github.com/arduino/arduino-ide/issues/1030#issuecomment-1152005617). This [Arduino post](https://support.arduino.cc/hc/en-us/articles/4415103213714-Find-sketches-libraries-board-cores-and-other-files-on-your-computer) gives an overview of all folders used by the IDE. Apparently that `arduino-ide` folder contains some state information. After deleting it and restarting the IDE, you need to select the board again and then the new menu should be shown.
(Issue [#110](https://github.com/openwch/arduino_core_ch32/issues/110) shares some frustration, followed by [this solution](https://github.com/openwch/arduino_core_ch32/issues/110#issuecomment-2768025106) ).

### Updating OpenOCD ###
To support the new CH32VM00X boards (CH32V002, CH32V006, etc), the installation done when installing CH32 Arduino core v 1.0.4 needs to be updated.
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

