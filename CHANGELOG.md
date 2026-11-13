# Changelog

All releases are installable from the Arduino Boards Manager (`package_ch32v_index.json`); older versions stay available there.

## 1.0.7 (2026-11-13)

### New

- **CH32V4x7** series, CH32V407VET6 EVT board
- **macOS on Apple silicon** (macOS 14 or newer): GCC 15.2 and OpenOCD 2.12 from the Boards Manager. Intel Macs keep 1.0.4, as WCH's macOS toolchain is Apple silicon only
- OpenOCD 2.12 on Windows, Linux and macOS

### Fixed

- `SPI.setClockDivider()`, `setDataMode()` and `setBitOrder()` called before `SPI.begin()` were ignored ([#42](https://github.com/openwch/arduino_core_ch32/issues/42))
- `analogWrite()` on the V003's PD0, PA2 and PD1 (TIM1 complementary outputs) ([#183](https://github.com/openwch/arduino_core_ch32/issues/183))
- `DEBUG` was defined in every sketch ([#125](https://github.com/openwch/arduino_core_ch32/issues/125))

## 1.0.6 (2026-11-07)

The Boards Manager package includes the Adafruit TinyUSB library again (Tools → USB support).

### Fixed

- Serial receives through an interrupt into a 64-byte buffer on every UART and every series: `available()`, `peek()` and `read()` work, and bytes that arrive while the sketch is busy are kept. Covers `Serial`, `Serial2`.. and `HardwareSerial(USARTx)` made in a sketch ([#284](https://github.com/openwch/arduino_core_ch32/pull/284), based on [#225](https://github.com/openwch/arduino_core_ch32/pull/225) by @jobitjoseph; fixes [#251](https://github.com/openwch/arduino_core_ch32/issues/251), [#142](https://github.com/openwch/arduino_core_ch32/issues/142), [#93](https://github.com/openwch/arduino_core_ch32/issues/93))
- `delayMicroseconds()` was too long on the slow chips (`delayMicroseconds(10)` took 46-64 us on V003/V006, 20 us on X035) and `micros()` could step back 1 ms, which ended `delay()` early; both use 32-bit math now and sketches are 0.3-1.8 KB smaller ([#284](https://github.com/openwch/arduino_core_ch32/pull/284))
- `Serial6` uses UART6 (the V30x has no USART6)

## 1.0.5 (2026-11-03)

First release since 1.0.4 (2023-10). It brings two new series, the WCH GCC 15 toolchain and a year of fixes from the community.

### New

- **CH32VM00X** series (V002/V004/V005/V006/V007 family), CH32V006K8 EVT board
- **CH32L10x** series, CH32L103C8T6 EVT board
- Boards: CH32V203G6 EVT, CH32V203C6/C8, CH32V203RB, Adafruit QT Py CH32V203
- Toolchain **WCH GCC 15.2** (`riscv-wch-gcc`) and **OpenOCD 2.10** (`openocd-wch`), installed by the Boards Manager
- Tools menu: clock select (internal/external, all speeds), Optimize (with LTO), C runtime (nano, float printf/scanf), USB support (Adafruit TinyUSB), upload method (WCH-SWD, WCH-ISP)
- `HardwareTimer`, `tone()`, I2C slave mode, `EEPROM` (user option bytes), `USBPD_SINK` library, WCharacter functions
- `Wire` and `SPI` on VM00X and L10x ([#199](https://github.com/openwch/arduino_core_ch32/pull/199))
- I2C pin remapping ([#152](https://github.com/openwch/arduino_core_ch32/pull/152)); SPI3 and SDIO pins on V30x ([#161](https://github.com/openwch/arduino_core_ch32/pull/161))
- V103R8T6: all 16 analog inputs (A0-A15) and SPI1 on PA5-PA7 ([#211](https://github.com/openwch/arduino_core_ch32/pull/211), [#277](https://github.com/openwch/arduino_core_ch32/pull/277))
- AVR compatibility headers `util/delay.h` and `util/atomic.h` for AVR libraries; `ARDUINO_<board>` and `ARDUINO_ARCH_CH32` defines
- `pinMode(pin, OUTPUT_AF_PP / OUTPUT_AF_OD)` ([#233](https://github.com/openwch/arduino_core_ch32/issues/233))

### Fixed

- `micros()` could step backward ([#250](https://github.com/openwch/arduino_core_ch32/pull/250))
- `attachInterrupt()` called the handler once right after attaching; timer and EXTI handlers are now weak so libraries can override them ([#271](https://github.com/openwch/arduino_core_ch32/pull/271), [#273](https://github.com/openwch/arduino_core_ch32/pull/273))
- Serial parity and word length ([#270](https://github.com/openwch/arduino_core_ch32/pull/270)); UART TX on the right peripheral ([#180](https://github.com/openwch/arduino_core_ch32/pull/180)); `Serial2`..`Serial8` and `HardwareSerial(USARTx)` use their own pins; `Serial.write()` returns 1 ([#277](https://github.com/openwch/arduino_core_ch32/pull/277), [#278](https://github.com/openwch/arduino_core_ch32/pull/278)); `print(0)` prints "0" ([#189](https://github.com/openwch/arduino_core_ch32/pull/189))
- `analogRead(PADC_VREF)` on V00x ([#156](https://github.com/openwch/arduino_core_ch32/pull/156)); `analogReadResolution()` scales from the ADC's real resolution
- VM00X system clock and PWM timing ([#198](https://github.com/openwch/arduino_core_ch32/pull/198))
- X035 TIM2 remap ([#272](https://github.com/openwch/arduino_core_ch32/pull/272))
- V10x: `noInterrupts()`/`interrupts()` and `ATOMIC_BLOCK` work (`main()` now runs in machine mode)
- LTO builds on V00x, VM00X and X035 ([#215](https://github.com/openwch/arduino_core_ch32/pull/215))
- `random()` no longer pulls in malloc and printf with GCC 15 (the 16 KB V003 had run out of flash)
- SPI2, I2C slave on V30x, flash and RAM sizes of several variants, a hard fault without the TinyUSB header

### Package update (2026-11-05)

- `beforeinstall` 2.0.0: on Linux it installs udev rules for the WCH-LinkE, the WCH-ISP bootloader and the other debug probes OpenOCD 2.10 supports, and no longer copies old libraries into `/usr/lib`; Windows and macOS get an empty archive, which fixes the install error on Windows ([#228](https://github.com/openwch/arduino_core_ch32/issues/228), [#209](https://github.com/openwch/arduino_core_ch32/issues/209), [#138](https://github.com/openwch/arduino_core_ch32/issues/138)). Reinstall 1.0.5 to get it.

### Known problems

- `Serial.available()` returns -1 when nothing has been received (fixed on `main` by [#284](https://github.com/openwch/arduino_core_ch32/pull/284))
- No serial output on the CH32L103 ([#149](https://github.com/openwch/arduino_core_ch32/issues/149))
- LTO on V10x, V20x, V30x and L10x drops the core's interrupt handlers ([#204](https://github.com/openwch/arduino_core_ch32/issues/204))
- No PWM on complementary timer outputs ([#183](https://github.com/openwch/arduino_core_ch32/issues/183))
- macOS: the 1.0.5 tools aren't published for macOS yet; the Boards Manager offers 1.0.4 there
- The 1.0.5 package is missing the Adafruit TinyUSB library (Tools -> USB support): the tag zip doesn't include git submodules

## 1.0.4 (2023-10-23)

CH32V00x, CH32X035, CH32V10x, CH32V20x and CH32V30x EVT boards with GCC 8.2 (`riscv-none-embed-gcc`) and OpenOCD 1.0.0; Linux and macOS support.

## 1.0.0 – 1.0.3 (2023)

First releases: CH32V003, CH32V203 and CH32X035 EVT boards, then CH32V103 and CH32V307; SPI and I2C master since 1.0.2.
