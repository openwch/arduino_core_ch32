# Contributing

Thanks for helping. This page covers reporting a bug, sending a fix, and adding a chip or board.

## Reporting a bug

A report we can reproduce gets fixed fastest. Please include:

- **Core version** (Boards Manager version, or the git commit for a clone) and **Arduino IDE / CLI version**
- **Board and menu settings**: Tools → Board, Board Select, Clock, Optimize
- **The smallest sketch** that shows the problem, and the **wiring** it needs
- **What you expected and what happened**: the Serial output, or the full build/upload log (File → Preferences → "Show verbose output during compile/upload")
- For upload problems: the WCH-LinkE firmware version (shown in the upload log) and the OS

Open it with the [bug report form](https://github.com/openwch/arduino_core_ch32/issues/new/choose). Questions about using the core are welcome too; check the README and the existing issues first.

## Sending a fix

1. Fork the repo and branch from `main`.
2. Keep the change to one problem, and keep the other series working: code for one series goes inside `#if defined(CH32V20x)` (or the series you're fixing), never by removing another series' code.
3. Build it for every series (below), and run it on a board if you have one.
4. Open a pull request: what was wrong, what you changed, which boards you tested on, and `Fixes #N` for the issue it closes.

Commit messages: a short first line saying what changed (`HardwareSerial: take a UART's own pins when it is not the Serial UART`), then why, if it isn't obvious.

### Build check

Before opening a pull request, build for every series, not only the one you changed: a change outside an `#if defined(CH32...)` block reaches all of them. Clone the core into your sketchbook (see the README, "Development version") and build a sketch for each series:

```bash
for fqbn in CH32V00x_EVT:pnum=CH32V003F4 CH32VM00X_EVT:pnum=CH32V006K8 \
            CH32X035_EVT:pnum=CH32X035G8U CH32V10x_EVT:pnum=CH32V103R8T6 \
            CH32V20x_EVT:pnum=CH32V203C8 CH32V30x_EVT:pnum=CH32V307VCT6 \
            CH32L10x_EVT:pnum=CH32L103C8T6; do
  arduino-cli compile -b WCH:ch32v:$fqbn libraries/Wire/examples/i2c_scanner || echo "FAILED: $fqbn"
done
```

Watch the flash size on the CH32V003: it has 16 KB, and a change that adds a few KB to every sketch is a problem there.

### Testing on a board

The maintainers run each release on the EVT board of every series. For a fix, a short sketch that prints the result over Serial (`[PASS]` / `[FAIL]` per check) is the best evidence: put it in the pull request description with its output from main and from your branch.

## Adding a chip or board

A new board of a supported chip only needs a variant; a new chip of a supported series also needs its `boards.txt` entry and linker script.

1. **Variant**: copy the closest folder in `variants/<series>/` to `variants/<series>/<NAME>/` and adjust:
   - `variant_<NAME>.h`: pin numbers, `NUM_DIGITAL_PINS`, `NUM_ANALOG_INPUTS`, the `*_MODULE_ENABLED` switches, the default `Serial`, `Wire` and `SPI` pins, `LED_BUILTIN`
   - `variant_<NAME>.cpp`: `digitalPin[]` and `analogInputPin[]`
   - `PeripheralPins.c`: which pins can be ADC, PWM (timer channel), UART, I2C and SPI pins
2. **boards.txt**: a `menu.pnum` entry under the series, with `upload.maximum_size` and `upload.maximum_data_size` matching the chip's flash and RAM, `build.variant`, `build.march` (as the other chips of the series) and, if the memory differs, `build.ldscript`. Keep `tools/makeboards.py` in step if you use it.
3. **Check**: build the examples for the new entry, then run a sketch on the board: GPIO, `Serial`, one ADC pin, one PWM pin, and `Wire`/`SPI` if they're enabled.
4. **Don't change the debug pins** (SWDIO/SWCLK or SWIO) in `pre_init()`: if a variant turns them into GPIO, users can't upload again without BOOT0.

## Code style

Follow the file you're editing: the Arduino API files use the Arduino style; the WCH SDK files in `system/` keep WCH's style and should only change when there is a real SDK fix. No unrelated reformatting in a fix.
