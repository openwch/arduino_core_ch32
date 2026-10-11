<div align="center">

# Arduino Core for CH32

**Write Arduino sketches for WCH's low-cost RISC-V microcontrollers.**
Community-maintained, Arduino IDE 2.x ready.

[![Compile examples](https://img.shields.io/github/actions/workflow/status/ch32-community/arduino_core_ch32/compile_examples.yml?branch=main&style=flat-square&label=compile%20examples)](https://github.com/ch32-community/arduino_core_ch32/actions/workflows/compile_examples.yml)
[![Stars](https://img.shields.io/github/stars/ch32-community/arduino_core_ch32?style=flat-square)](https://github.com/ch32-community/arduino_core_ch32/stargazers)
[![Issues](https://img.shields.io/github/issues/ch32-community/arduino_core_ch32?style=flat-square)](https://github.com/ch32-community/arduino_core_ch32/issues)
[![Pull requests](https://img.shields.io/github/issues-pr/ch32-community/arduino_core_ch32?style=flat-square)](https://github.com/ch32-community/arduino_core_ch32/pulls)
[![Last commit](https://img.shields.io/github/last-commit/ch32-community/arduino_core_ch32?style=flat-square)](https://github.com/ch32-community/arduino_core_ch32/commits/main)
[![MemBrowse](https://membrowse.com/badge.svg)](https://membrowse.com/public/ch32-community/arduino_core_ch32)

[Install](#-installation) · [Supported boards](#-supported-boards) · [Platform setup](#-platform-setup) · [Contributing](#-contributing) · [Get help](#-getting-help)

</div>

---

## About

This project brings the CH32 family of RISC-V microcontrollers to the Arduino IDE, so you can use the familiar `setup()` / `loop()` workflow, Arduino-style APIs and your favourite libraries on cheap, capable WCH chips.

The original core was published by WCH ([openwch/arduino_core_ch32](https://github.com/openwch/arduino_core_ch32)) but is no longer actively maintained. **This repository is the community continuation:** we're keeping it building, fixing bugs, reviewing pull requests and extending support for more chips and peripherals.

### What's inside

| Component | What it does |
| :-- | :-- |
| [Arduino core](https://github.com/ch32-community/arduino_core_ch32) | Arduino API implementation, variants and libraries for CH32 |
| [openocd_wch](https://github.com/ch32-community/openocd_wch) | OpenOCD build that uploads and debugs over WCH-LinkE |
| [riscv-none-embed-gcc](https://github.com/ch32-community/risc-none-embed-gcc) | Toolchain with WCH's custom half-word/byte compression extensions and hardware stack push/pop |

## 🚀 Installation

> **Requires Arduino IDE 2.0 or newer.**

1. Open **File → Preferences** (or **Arduino IDE → Settings** on macOS).
2. Paste this URL into **Additional boards manager URLs**:

   ```
   https://github.com/ch32-community/board_manager_files/raw/main/package_ch32v_index.json
   ```

3. Open **Tools → Board → Boards Manager**, search for **wch**, and click **Install**.
4. Pick your board under **Tools → Board**, plug in a WCH-LinkE, and hit **Upload**.

Linux and macOS need a one-time extra step. See [Platform setup](#-platform-setup).

### Quick test

```cpp
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
```

If your board variant doesn't define `LED_BUILTIN`, substitute a GPIO pin that has an LED attached.

## 🧩 Supported boards

| Family | Tested Board | ADC | DAC | USART | GPIO | EXTI |SysTick | SPI (Master) | SPI (Slave) | I2C (Master) | I2C (Slave) |
| :-- | :-- | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: |
| CH32V00x | CH32V003F4P        | ✅ | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ | | ✅ | ✅ |
| CH32VM00X | CH32V006K8        | ✅ | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ | | ✅ | |
| CH32V10x | CH32V103R8T6_BLACK | ✅ | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ | | ✅ | |
| CH32V20x | CH32V203G8U        | ✅ | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ | | ✅ | |
| CH32V30x | CH32V307VCT6_BLACK | ✅ |   | ✅ | ✅ | ✅ | ✅ | ✅ | | ✅ | |
| CH32X035 | CH32X035G8U        | ✅ | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ | | ✅ | |


| Family | Board |
| :-: | :-- |
| ✅ | Supported|
| 🟡 | Partial support |
| ❌ | Not supported by hardware|
| \[empty\] | Unknown |

Missing your chip? [Open an issue](https://github.com/ch32-community/arduino_core_ch32/issues/new) or send a pull request. New variants are very welcome.

## 🖥️ Platform setup

Uploads go through a **WCH-LinkE** programmer using the OpenOCD and toolchain builds from [MounRiver Studio (MRS)](http://www.mounriver.com/).

<details>
<summary><b>Windows</b></summary>

If uploads fail, make sure your WCH-LinkE firmware matches the latest version shipped with MRS. Firmware and tooling details are in [WCH's WCH-Link repository](https://github.com/openwch/ch32v307/tree/main/WCH-Link).

</details>

<details>
<summary><b>Linux</b></summary>

After installing the board package for the first time, run the setup script once so the udev rules and libraries get installed:

```bash
cd ~/.arduino15/packages/WCH/tools/beforeinstall/1.0.0
./start.sh
```

You'll be asked for your sudo password. The script copies the required libraries, registers them, installs the udev rules and reloads them. Expect output similar to:

```text
Copy Libs
Register new Libs
copy rules
Reload rules
DONE
```

</details>

<details>
<summary><b>macOS</b></summary>

Install `libusb` before your first upload:

```bash
brew install libusb
```

If you still see libusb errors on upload, please [open an issue](https://github.com/ch32-community/arduino_core_ch32/issues/new) with your macOS version and the full error output.

</details>

## 🤝 Contributing

This is a volunteer project and help is appreciated, whether that's testing on real hardware, fixing a peripheral driver, adding a variant or improving the docs.

- **Found a bug?** [Open an issue](https://github.com/ch32-community/arduino_core_ch32/issues/new) with your board, core version, Arduino IDE version, OS and a minimal sketch that reproduces it.
- **Fixed something or added a board?** [Send a pull request](https://github.com/ch32-community/arduino_core_ch32/pulls). Small, focused PRs are easiest to review.
- **Have an idea or question?** Start a thread in [Discussions](https://github.com/ch32-community/arduino_core_ch32/discussions).

## 💬 Getting help

Please use [GitHub Issues](https://github.com/ch32-community/arduino_core_ch32/issues) for bugs and [Discussions](https://github.com/ch32-community/arduino_core_ch32/discussions) for questions. Issues with the MRS toolchain on macOS can also be directed to the MRS team at <support@mounriver.com>.

## 🙏 Credits

- **WCH**, for the original [Arduino core](https://github.com/openwch/arduino_core_ch32), the CH32 SDK and the chips themselves.
- **The MounRiver Studio team**, for the toolchain and OpenOCD work this core builds on.
- Everyone who has [contributed](https://github.com/ch32-community/arduino_core_ch32/graphs/contributors) fixes, boards and bug reports.

*This is an independent community project and is not affiliated with or endorsed by WCH.*
