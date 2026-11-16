# UF2 converter

`uf2conv.py` is from Microsoft's [UF2 tools](https://github.com/microsoft/uf2/tree/84444ddb9d2914edf7f6d9e89a7ce41b53d64d98),
commit `84444ddb9d2914edf7f6d9e89a7ce41b53d64d98`. See `LICENSE.txt`.

The platform provides optional UF2 conversion and upload recipes. Existing
boards keep their current output formats and upload tools.

## Python dependency

Boards Manager installs the pinned `pqt-python3` tool
(`1.0.1-base-3a57aed-1`) from the
[CH32 package index](https://github.com/ch32-community/board_manager_files).
It reuses the Python archives distributed by
[Arduino-Pico](https://github.com/earlephilhower/arduino-pico):

- **Windows and macOS:** a private Python interpreter is installed with the
  board package. No separate Python installation or `PATH` change is needed.
- **Linux:** the tool is a wrapper around the system's `python3`. Install
  Python 3 with your distribution's package manager if it is missing.

No additional Python packages are needed. Both UF2 recipes select the tool's
interpreter explicitly and run it in isolated mode (`-I`). The upload wrapper
uses the same interpreter and isolation for its converter subprocess.

The core recipes and the package-index dependency must be updated together.
After updating the index, reinstall the CH32 package in Boards Manager if an
existing installation lacks the tool; copying core files alone does not install
tool dependencies. Restart Arduino IDE after installation. For an older core
that still reports `exec: "python": executable file not found`, installing
Python 3 from [python.org](https://www.python.org/downloads/) with `python` on
`PATH` remains a workaround.

For a Git checkout, install the CH32 package's tool dependencies first, or
override `tools.uf2.python` with an absolute Python 3 executable path in
`platform.local.txt` or an Arduino CLI `--build-property` argument. The example
build runner supplies the interpreter that is running `tools/build_examples.py`.

## Board configuration

A board opts in through its `boards.txt` properties. For example, a future
CH32V4x7 board with a 32 KiB TinyUF2 reservation would use:

```ini
example.build.uf2_family=0x7b8c32a7
example.build.flash_offset=0x8000
example.recipe.objcopy.uf2.pattern={tools.uf2.convert.pattern}
example.build.preferred_out_format=uf2
example.upload.tool=uf2
example.upload.tool.default=uf2
```

If a board has an upload-method menu, select `uf2` in that menu entry's
`upload.tool` property as well; menu properties override board defaults.

The family ID must match the installed bootloader. The application must also be
linked at the specified flash offset, with its maximum size reduced to reserve
the bootloader; conversion does not relocate executable code. These recipes
install neither board definitions nor a bootloader.

## Export and upload

For an opted-in board, compiling produces `.uf2` alongside `.bin` and `.hex`.
The `build.preferred_out_format` setting selects UF2 for Export Compiled Binary.
To upload, enter the board's bootloader and pass its mounted volume path as the
port, for example:

```sh
arduino-cli upload --fqbn <package>:ch32v:<board> --port D: --input-dir <build-output>
```

On Linux or macOS, use the volume's mount path instead of `D:`. Select the
intended board's volume explicitly; automatic serial-port-to-volume discovery
and reset are not provided. The upload wrapper requires a mounted volume with
an `INFO_UF2.TXT` file before writing `NEW.UF2` there. Missing drives, ordinary
directories, and stale mount directories are rejected. On Windows, a bare
drive letter such as `D:` selects the drive's root.

Both recipes use conversion-only mode with an explicit output path, avoiding
the converter's automatic deployment to every UF2 drive. The vendor converter
remains unmodified; volume validation is implemented separately in `upload.py`.
