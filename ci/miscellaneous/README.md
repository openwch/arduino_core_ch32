# Miscellaneous compiler coverage

Run `python tools/build_examples.py Miscellaneous` from the repository checkout.
The `Miscellaneous - Compile Sketches` workflow step runs this same suite.

`tools/miscellaneous_builds.py` supplies the build list to both example CI and
the MemBrowse target generator. It selects these representative processors from
`boards.txt`, using the largest package/peripheral set in each family:

| Family | Processor |
| --- | --- |
| V00x | CH32V003F4 |
| VM00X | CH32V006K8, plus CH32V002F4 for small-device I2C coverage |
| X035 | CH32X035G8U |
| V10x | CH32V103R8T6 |
| V20x | CH32V203RB |
| V30x | CH32V307VCT6 |
| L10x | CH32L103C8T6 |

The suite has 45 configurations and runs up to four concurrently. Smaller
package variants and alternate boards such as QTPy and LANA TNY are excluded.

Edit `BUILD_CASES` near the top of `tools/miscellaneous_builds.py` to change the
sketches and compiler configurations. `DEFAULT_BOARD_OPTIONS` sets optimization
and debug options for all cases; each case can override them. For example, add:

```python
{
    'label': 'O2_TIM',
    'sketch': 'ci/libraries/ArduinoFadeExample',
    'peripherals': ['TIM'],
    'board_options': {'opt': 'o2lto'},  # -O2 with LTO
    'extra_flags': ['-DMY_DEFINE=1'],  # additional C/C++ compiler flags
},
```

`peripherals: []` disables all optional peripheral modules. Supported modules
are ADC, DAC, UART, SPI, I2C, and TIM (subject to processor support). Optional
`processors` restricts a case to specific selected processors. `each_serial: True`
expands it for every supported serial port, substituting `{port}` in the label.
Use `build_properties` for other Arduino properties, such as
`{'compiler.cpp.std': 'gnu++17'}`. Give each configuration a unique label; changing
flags does not automatically change the label. CI and MemBrowse share this list.

| Sketch | Configuration | Processors |
| --- | --- | --- |
| ArduinoBlinkExample | `-Os`, LTO, no optional peripherals | All selected processors |
| ArduinoFadeExample | `-Os`, LTO, only TIM | All selected processors |
| i2c_scanner | `-Os`, LTO, only I2C and UART | V003 and V002 |
| SimpleSerialRead | `-Os`, LTO, only UART, each supported Serial selection | All selected processors |

Serial ports are discovered from each variant's `PIN_SERIALn_RX` and
`PIN_SERIALn_TX` definitions. Family menu choices that a processor does not
support are excluded (for example Serial2 on V002 and Serial3 on X033).
V003 has no serial menu, so its only UART is selected with a compiler definition.

The suite force-includes `ci/miscellaneous/peripherals.h` in each C/C++ source
file. This loads the original variant, clears its default module definitions,
and enables only the requested modules via `CI_ENABLE_*` compiler definitions.
Variant files are unchanged. Builds retain debug symbols for memory analysis.
USB remains at its default disabled setting.

MemBrowse report and onboarding workflows include the suite automatically.
The report workflow processes it as the `Miscellaneous` group. Targets and their
build directories have unique names, for example:

```
Miscellaneous/CH32V307VCT6_Serial3-SimpleSerialRead
Miscellaneous/CH32V003F4_LTO_NoPeripherals-ArduinoBlinkExample
Miscellaneous/CH32V002F4_LTO_I2C_UART-i2c_scanner
```

V307 Serial6–8 currently expose missing PC0–PC5 definitions in the original
variant. These configurations remain in the suite and fail until the variant
is fixed separately.
