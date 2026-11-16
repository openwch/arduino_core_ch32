## Workflows

### MemBrowse

[MemBrowse](https://membrowse.com/public/ch32-community/arduino_core_ch32) is a handy tool to track changes in used flash/ram memory as result of a PR.
- For each board the i2c_scanner example is used as basis for the report.
- To add new boards or variants, edit the targets configuration: /.github/membrowse-targets.json
- As `target_name` use  board family name (e.g. `ch32v20x`) or variant name (eg. `ch32vm00x-v002`)
- When using a variant, make sure to include the pnum in the fqbn parameter of the `build_cmd`: `--fqbn ch32-community:ch32v:CH32VM00X_EVT:pnum=CH32V002F4,dbg=enable_sym`

### Autmatic build of examples 

CH32V4x7 builds cover both the Adafruit Metro CH32V407 and CH32V467 at the
default 100 MHz clock, including TinyUSB examples. The miscellaneous builds
also check Serial on both variants at 100 and 200 MHz. The USB-PD example is
skipped because these boards do not support PD. MemBrowse tracks both variants
with separate build directories and reports.

As of PR #69, we can now test specific variants:
- Build step compile_examples.yml builds the V006K8 and V002F4 variants as specified.
- It gave build errors only for V002F4 when the sketch was too large
- It skipped the example for that variant after adding a .CH32VM00X-CH32V002F4.test.skip file.

We can still test the default variant per board like before, but now we can also build a specific variant:
Marker in the example directory | Effect
-- | --
.CH32VM00X.test.skip | Skip all configured VM00X variants
.CH32VM00X-CH32V002F4.test.skip | Skip only the V002F4 build
.CH32VM00X.test.only | Allow all configured VM00X variants
.CH32VM00X-CH32V006K8.test.only | Allow only the V006K8 build

The same names work as individual lines in `.skip.txt`. For any example to skip, add this line to its `.skip.txt`:
- `CH32VM00X-CH32V002F4`
