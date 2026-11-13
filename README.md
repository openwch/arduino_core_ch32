<div align="center">

# Arduino Core for WCH CH32

**Program WCH's RISC-V CH32 microcontrollers with the Arduino IDE, Arduino CLI and the Arduino API you already know.**

[![Release](https://img.shields.io/github/v/tag/openwch/arduino_core_ch32?label=release&sort=semver)](https://github.com/openwch/arduino_core_ch32/tags)
[![Open issues](https://img.shields.io/github/issues/openwch/arduino_core_ch32)](https://github.com/openwch/arduino_core_ch32/issues)
[![Arduino IDE 2](https://img.shields.io/badge/Arduino%20IDE-2.x-00979D?logo=arduino&logoColor=white)](https://www.arduino.cc/en/software)
[![Toolchain](https://img.shields.io/badge/toolchain-WCH%20GCC%2015.2-blue)](#toolchain)

[Install](#installation) · [Boards](#supported-chips-and-boards) · [Features](#features) · [Getting started](#getting-started) · [Upload](#upload) · [Changelog](CHANGELOG.md) · [Contributing](CONTRIBUTING.md)

</div>

---

## Why this core

- **One core for the CH32V family**: V003 and the V002/V004/V005/V006/V007 family, X035, V103, V203, V307, L103 and V407, from 16 KB to 512 KB of flash, with the same sketch code.
- **Standard Arduino API**: `digitalWrite`, `analogRead`, `analogWrite`, `attachInterrupt`, `Serial`, `Wire`, `SPI`, `tone`, `HardwareTimer`, plus the WCH peripheral drivers for everything else.
- **WCH's own toolchain**: GCC 15.2 with the WCH extensions (`xw` compressed instructions, hardware stack push/pop) and OpenOCD 2.12 for the WCH-LinkE, both installed by the Boards Manager.
- **Tuning menus**: clock source and speed, optimization (LTO for the 16 KB V003), C library (nano, float printf/scanf), USB stack, upload method.
- **Checked on hardware**: releases are run on the EVT boards of each series (GPIO, interrupts, ADC, PWM, timers, I2C, SPI, Serial) before they go to the Boards Manager.

## Installation

### Arduino IDE 2 (Boards Manager)

1. **File → Preferences → Additional boards manager URLs**, add:

   ```
   https://github.com/openwch/board_manager_files/raw/main/package_ch32v_index.json
   ```

2. **Tools → Board → Boards Manager**, search **CH32**, install **CH32 MCU EVT Boards**.
   The IDE also downloads the toolchain (WCH GCC 15.2) and OpenOCD 2.12.
3. **Tools → Board → CH32 MCU EVT Boards**: pick your series, then your chip under **Tools → Board Select**.

### Arduino CLI

```bash
arduino-cli config add board_manager.additional_urls https://github.com/openwch/board_manager_files/raw/main/package_ch32v_index.json
arduino-cli core update-index
arduino-cli core install WCH:ch32v
arduino-cli compile -b WCH:ch32v:CH32V20x_EVT:pnum=CH32V203C8 MySketch
```

### Development version (git)

Clone into your sketchbook to use the latest `main`:

```bash
mkdir -p ~/Documents/Arduino/hardware/WCH
cd ~/Documents/Arduino/hardware/WCH
git clone --recursive https://github.com/openwch/arduino_core_ch32.git ch32v
```

The clone uses the same toolchain as the release: install the release once from the Boards Manager (it brings the tools), or point `wch.gcc.dir` and `wch.openocd.dir` at your own tool folders in a `platform.local.txt` next to `platform.txt`.

### Operating systems

| OS | Status | Notes |
|---|---|---|
| Windows 10/11 x64 | ✅ | WCH-LinkE driver: install [WCH-LinkUtility](https://www.wch-ic.com/downloads/WCH-LinkUtility_ZIP.html) once |
| Linux x86_64 | ✅ | run once after installing, so the WCH-LinkE (and other debug probes) work without sudo: `~/.arduino15/packages/WCH/tools/beforeinstall/2.0.0/start.sh` |
| macOS, Apple silicon | ✅ | macOS 14 or newer |
| macOS, Intel | ⚠️ | WCH's toolchain for macOS runs on Apple silicon only, so the Boards Manager offers 1.0.4 there |

**Intel Mac with 1.0.4:** install the `libusb` library before the first upload:

```bash
# with brew
brew install libusb
# or with macport
sudo port install libusb
```

If uploads still fail with `libusb` errors, contact the MounRiver Studio team at support@mounriver.com (they build the toolchain and OpenOCD).

## Supported chips and boards

| Series (Tools → Board) | Chips / boards (Board Select) | Core | Max clock | Flash / RAM | Since |
|---|---|---|---|---|---|
| **CH32V00x** | CH32V003F4 EVT | QingKe V2A | 48 MHz | 16 KB / 2 KB | 1.0.0 |
| **CH32VM00X** | CH32V006K8 EVT | QingKe V2C | 48 MHz | 62 KB / 8 KB | 1.0.5 |
| **CH32X035** | CH32X035G8U EVT | QingKe V4C | 48 MHz | 62 KB / 20 KB | 1.0.1 |
| **CH32V10x** | CH32V103R8T6 EVT | QingKe V3A | 72 MHz | 64 KB / 20 KB | 1.0.3 |
| **CH32V20x** | CH32V203C6, C8, G6, G8, RB EVT, Adafruit QT Py CH32V203 | QingKe V4B | 144 MHz | up to 128 KB / 64 KB | 1.0.0 |
| **CH32V30x** | CH32V307VCT6 EVT | QingKe V4F (FPU) | 144 MHz | 256 KB / 64 KB | 1.0.3 |
| **CH32L10x** | CH32L103C8T6 EVT | QingKe V4C | 96 MHz | 64 KB / 20 KB | 1.0.5 |
| **CH32V4x7** | CH32V407VET6 EVT | QingKe V5F | 200 MHz | 512 KB / 200 KB | 1.0.7 |

More packages of the V002/V004/V005/V006/V007 family and the CH32X033 are on the way ([#200](https://github.com/openwch/arduino_core_ch32/pull/200), [#171](https://github.com/openwch/arduino_core_ch32/pull/171)). A board that isn't listed can usually run the variant of the same chip; see [adding a board](CONTRIBUTING.md#adding-a-chip-or-board).

## Features

| | V00x | VM00X | X035 | V10x | V20x | V30x | L10x | V4x7 |
|---|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| GPIO, pull-up/down, open drain | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| `attachInterrupt` (EXTI) | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| `analogRead` (ADC) | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| `analogWrite` (PWM), `tone` | ✅ | ✅ | ✅ ¹ | ✅ | ✅ | ✅ | ✅ | ✅ |
| `HardwareTimer` | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✓ |
| `Serial` (U(S)ART) | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ⚠️ ² | ✅ |
| `Wire` (I2C master) | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✓ | ✅ |
| `SPI` | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✓ | ✅ |
| LTO (Tools → Optimize) | ✅ | ✅ | ✅ | ⚠️ ⁴ | ⚠️ ⁴ | ⚠️ ⁴ | ⚠️ ⁴ | |
| `EEPROM` (option bytes) | ✓ | | | ⚠️ ³ | | | | |
| USB device (Adafruit TinyUSB) | | | | | ✓ | ✓ | | |
| USB-PD sink (`USBPD_SINK`) | | | ✓ | | | | | |

✅ checked on an EVT board · ✓ available, not yet checked on a board · ⚠️ known problem
¹ not on complementary (N) timer outputs yet, [#183](https://github.com/openwch/arduino_core_ch32/issues/183) · ² [#149](https://github.com/openwch/arduino_core_ch32/issues/149) · ³ [#213](https://github.com/openwch/arduino_core_ch32/issues/213) · ⁴ LTO drops the core's interrupt handlers on these series ([#204](https://github.com/openwch/arduino_core_ch32/issues/204)); use the plain optimize options

Open problems are in the [issues](https://github.com/openwch/arduino_core_ch32/issues).

## Getting started

```cpp
// Toggle a pin and print over Serial (the WCH-Link's COM port, 115200)
#define LED_PIN PA8   // pins use the chip's names: PA8, PD4, PC1, ...

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  Serial.print("millis: ");
  Serial.println(millis());
  delay(500);
}
```

Analog inputs are also `A0`, `A1`, ... The default `Serial` is the USART wired to the WCH-Link on the EVT board. Other UARTs: `HardwareSerial Serial2(USART2);`, with the pins from the variant's pin map (USART2 on PA2/PA3 for the V20x and X035 boards; the other variants list only the `Serial` UART so far).

Library examples are under **File → Examples → (Examples for CH32 MCU EVT Boards)**: EEPROM, SPI (SPI flash), Wire (I2C scanner, AT24C02, I2C slave), USBPD_SINK.

### Using the WCH peripheral drivers

The WCH SDK drivers (`ch32*_*.h`) are available in every sketch, so peripherals the Arduino API doesn't cover (DMA, CAN, OPA, RTC, DAC, ...) can be used next to Arduino code:

```cpp
void setup() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitTypeDef gpio = {0};
  gpio.GPIO_Pin = GPIO_Pin_0;
  gpio.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio);
}
```

## Upload

| Tools → Upload method | Hardware |
|---|---|
| **WCH-SWD** (default) | WCH-LinkE (RV mode) or the WCH-Link on the EVT board; SWDIO/SWCLK, or SWIO (PD1) on V00x/VM00X |
| **WCH-ISP** | the chip's USB/UART bootloader (BOOT0 high while resetting), on chips that have one |

- **WCH-LinkE in ARM mode?** Switch it to RV mode with WCH-LinkUtility.
- **Chip no longer connects** (a sketch turned the debug pins into GPIO)? Upload with BOOT0 held high, or erase the chip with WCH-LinkUtility.

## Toolchain

| Tool | Version | Installed by |
|---|---|---|
| WCH GCC (`riscv32-wch-elf`) | 15.2.0 | Boards Manager (`riscv-wch-gcc`) |
| WCH OpenOCD | 2.12 | Boards Manager (`openocd-wch`) |

GCC 15.2 and OpenOCD 2.12 are the builds from [MounRiver Studio](http://www.mounriver.com/) (3.0.0 for OpenOCD and for macOS). Releases up to 1.0.4 used GCC 8.2 ([openwch/risc-none-embed-gcc](https://github.com/openwch/risc-none-embed-gcc)) and OpenOCD 1.0.0 ([openwch/openocd_wch](https://github.com/openwch/openocd_wch)); they stay installable from the Boards Manager.

## Project layout

```
cores/arduino/       Arduino API (HardwareSerial, HardwareTimer, wiring_*, Tone, ...)
cores/arduino/ch32/  glue between the API and the WCH SDK (uart, analog, interrupt, clock, timer)
variants/<series>/   one folder per chip or board: pin names, pin maps, enabled peripherals
system/<series>/     WCH SDK: startup, linker scripts, peripheral drivers
libraries/           Wire, SPI, EEPROM, USBPD_SINK, Adafruit TinyUSB
boards.txt           boards and Tools-menu options
platform.txt         build recipes, toolchain, upload tools
```

## Contributing

Bug reports, fixes and new boards are welcome. [CONTRIBUTING.md](CONTRIBUTING.md) covers how to report a bug so it can be reproduced, how to build and test a change, and how to add a chip or board.

- Bugs and feature requests: [open an issue](https://github.com/openwch/arduino_core_ch32/issues/new/choose)
- Release notes: [CHANGELOG.md](CHANGELOG.md)
- Everyone who has contributed fixes, boards and tests: [contributors](https://github.com/openwch/arduino_core_ch32/graphs/contributors)

## Related projects

- [WCH-LinkUtility](https://www.wch-ic.com/downloads/WCH-LinkUtility_ZIP.html): flash, erase and set up the WCH-Link
- [MounRiver Studio](http://www.mounriver.com/): WCH's IDE, source of the toolchain
- [WCH product pages](https://www.wch-ic.com/): datasheets and reference manuals

## Contact

- Issues: [github.com/openwch/arduino_core_ch32/issues](https://github.com/openwch/arduino_core_ch32/issues)
- Email: yy@wch.cn
