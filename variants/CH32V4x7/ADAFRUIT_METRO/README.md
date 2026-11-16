# Adafruit Metro CH32V407 / CH32V467

These variants target the rev A PCB with a separate Ethernet transformer and
RJ45 jack. Both use the same pin numbering and a 25 MHz crystal. CPU and bus
clocks default to 100 MHz. The **Clock select** menu also offers 200 MHz.
Both options retain the 200 MHz system PLL.

In the Arduino IDE, choose **Board > CH32V4x7**, then choose
**Board select > Adafruit Metro CH32V407** or **Adafruit Metro CH32V467** to
match the fitted chip. **Clock Select** and **Upload method > TinyUF2 (USB drive)**
are shared by both variants.

| Arduino pins | MCU pins, in the same order |
| --- | --- |
| D0–D7 | PB7, PB6, PB8, PB9, PC0, PC1, PC2, PC3 |
| D8–D13 | PA0, PA1, PA2, PA8, PA9, PB12 |
| A0–A5 | PA4, PA3, PB0, PB1, PC4, PC5 |
| SDA, SCL | PB11, PB10 |
| MOSI, MISO, SCK | PA7, PA6, PA5 |
| NeoPixel, USER button | PC14, PC15 |

`Serial` / `SerialUSB` is USB CDC; `Serial1` uses D0 RX and D1 TX.

The core bundles a tested default in `libraries/Adafruit_TinyUSB_Arduino`.
A separately installed **Adafruit TinyUSB Library** can override it through
normal Arduino library selection, including updates from Library Manager.
The selected library must include CH32V4x7 support and retain TinyUSB API
version `30000`, matching the core's bundled declarations. An incompatible
copy produces a compile-time error with recovery instructions instead of
missing USB symbols at link time. Update the library or board package as
directed, or remove the sketchbook TinyUSB installation to use the bundled copy.
Release **3.7.7** predates CH32V4x7 support; until a newer release includes it,
use the bundled default or a compatible upstream development version.

The USER button is active low. A0 supports DAC output. Pin remapping is selected
through the variant's peripheral tables; the header UART uses USART1's PB6/PB7
remap. `Wire` uses the I3C hardware controller in legacy I2C mode on PB10/PB11.
External I2C pull-ups are required; controller mode only. The default is the
400 kHz setting at 100 MHz HCLK; other clock settings are currently rejected.
Select **100 MHz (hardware I2C)** to use `Wire`. At the 200 MHz CPU setting,
the controller cannot meet the Fast-mode SCL-low minimum, so transfers fail.

This controller's legacy mode requires at least one data byte and a STOP after
every message. `endTransmission(false)` and zero-byte scanner probes return 4;
reads without STOP and combined internal-address reads return zero bytes. Use
separate write-with-STOP and read-with-STOP calls for devices that permit them.
The driver never substitutes a STOP or dummy data for an unsupported request.
After an unsupported `setClock()` value, transfers fail until
`Wire.setClock(400000)` or `Wire.begin()` restores the supported configuration.

## TinyUF2

Production Metro CH32V407 and CH32V467 boards ship with TinyUF2 preinstalled.
Normal sketch uploads need only USB. A debug probe is needed only for a blank
board or to replace the bootloader; bootloader self-update over UF2 is not
supported.

UF2 conversion and uploads use Python 3.
Boards Manager installs a private interpreter on **Windows and macOS**, following
Arduino-Pico's approach; no separate Python installation or `PATH` change is
needed. On **Linux**, the packaged launcher uses system `python3`, which must be
installed through your distribution's package manager. No extra Python packages
are required.

Updating core files alone does not install the Python tool. Update the package
index and reinstall the CH32 package in Boards Manager if the tool is missing,
then restart Arduino IDE. See the [UF2 Python setup notes](../../../tools/uf2/README.md#python-dependency)
for Git checkouts and an older-core workaround. An `exec: "python": executable
file not found` error comes from UF2 conversion after linking.

For initial installation or recovery:

1. Download the bootloader for the chip fitted to the board:
   [Metro CH32V407](https://github.com/adafruit/tinyuf2/actions/runs/43249115554/artifacts/13957828280)
   or [Metro CH32V467](https://github.com/adafruit/tinyuf2/actions/runs/43249115554/artifacts/13957853111).
   These are official TinyUF2 `master` build artifacts at `d4a8530`; GitHub sign-in
   is required. Extract `tinyuf2.hex` from the matching ZIP. If the artifacts
   have expired, use a newer matching artifact from the
   [TinyUF2 Build workflow](https://github.com/adafruit/tinyuf2/actions/workflows/build.yml?query=branch%3Amaster),
   or follow the [CH32V4x7 build instructions](https://github.com/adafruit/tinyuf2/tree/master/ports/ch32v4x7#build-and-initial-installation)
   with `BOARD=metro_ch32v407` or `BOARD=metro_ch32v467`.
2. Power the Metro from USB. Connect a CH32V4x7-compatible WCH single-wire debug
   (SWIO) probe: SWIO to PA13, reset to NRST, and ground to GND. Use 3.3 V signal
   levels; do not connect a second power output from the probe. A standard ARM
   SWD probe is not sufficient unless its firmware explicitly supports WCH SWIO.
3. In the probe's programming software, select the matching CH32V407 or CH32V467,
   load `tinyuf2.hex`, and program with sector erase and readback verification.
   Preserve any existing application first. Do not use a whole-chip erase or
   change option bytes/write protection. The bootloader occupies the first
   32 KiB; use the addresses encoded in the HEX file.
4. Disconnect the probe and reset the Metro. Double-tap RESET within 500 ms and
   check that the `METRO407` or `METRO467` USB drive appears. Application uploads
   can now use the USB procedure below.

Sketches start at flash address `0x8000`, preserving the bootloader's first
32 KiB. The two variants use the CH32V4x7 UF2 family ID `0x7b8c32a7`.

Build/export produces a `.uf2` file. Double-tap the Metro's RESET button, then
copy that file to `METRO407` or `METRO467`. The upload recipe also accepts the
mounted drive as an explicit port, for example:

```
arduino-cli upload --fqbn <package>:ch32v:CH32V4x7_BOARDS:pnum=ADAFRUIT_METRO_CH32V467 --port D: --input-dir <build-output>
```

Replace the package name, drive and output directory with your installation's
values; select `pnum=ADAFRUIT_METRO_CH32V407` for the other chip. Automatic
serial-port-to-drive discovery is not implemented. The USB
port supports a 1200-baud DTR drop to request the bootloader.

The TinyUSB submodule revision includes the CH32V4x7 USBHS1 device port. That
port is a dependency of these board definitions.
