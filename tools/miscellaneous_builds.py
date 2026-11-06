"""Shared compiler/peripheral coverage for example CI and MemBrowse."""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]

# Use the largest package/peripheral set in each family, plus V002 for its
# separate small-device I2C coverage. Avoid recompiling smaller package variants.
REPRESENTATIVE_PROCESSORS = {
    'CH32V003F4',
    'CH32V002F4',
    'CH32V006K8',
    'CH32V103R8T6',
    'CH32V203RB',
    'CH32V307VCT6',
    'CH32X035G8U',
    'CH32L103C8T6',
}

# Defaults for every case. Per-case board_options override these Arduino menu
# choices (for example opt=osstd for -Os without LTO, or opt=o2lto for -O2+LTO).
DEFAULT_BOARD_OPTIONS = {'opt': 'oslto', 'dbg': 'enable_sym'}

# Edit this list to add/remove builds or change their compiler arguments.
# Optional fields:
#   board_options: Arduino menu overrides, e.g. {'opt': 'o2lto'}
#   extra_flags: additional C/C++ flags, e.g. ['-DMY_DEFINE=1']
#   build_properties: Arduino build overrides, e.g. {'compiler.cpp.std': 'gnu++17'}
#   processors: restrict this case to these exact processor names; omit for all
#   each_serial: expand once per supported port; use {port} in the label
# Peripherals may contain ADC, DAC, UART, SPI, I2C, and TIM; [] disables them all.
# Labels form the build/MemBrowse names; give new configurations distinct labels.
BUILD_CASES = [
    {
        'label': 'LTO_NoPeripherals',
        'sketch': 'ci/libraries/ArduinoBlinkExample',
        'peripherals': [],
    },
    {
        'label': 'LTO_TIM',
        'sketch': 'ci/libraries/ArduinoFadeExample',
        'peripherals': ['TIM'],
    },
    {
        'label': 'LTO_I2C_UART',
        'sketch': 'libraries/Wire/examples/i2c_scanner',
        'peripherals': ['I2C', 'UART'],
        'processors': ['CH32V003F4', 'CH32V002F4'],
    },
    {
        'label': 'Serial{port}',
        'sketch': 'ci/miscellaneous/SimpleSerialRead',
        'peripherals': ['UART'],
        'each_serial': True,
    },
]


def miscellaneous_builds(package='local:ch32v'):
    """Expand representative processors and the serial ports each exposes."""
    properties = {}
    for line in (ROOT / 'boards.txt').read_text(encoding='utf-8').splitlines():
        key, separator, value = line.partition('=')
        if separator and not key.lstrip().startswith('#'):
            properties[key.strip()] = value.strip()

    for key in properties:
        match = re.fullmatch(r'(.+)\.menu\.pnum\.([^.]+)', key)
        if not match:
            continue
        board, processor = match.groups()
        if processor not in REPRESENTATIVE_PROCESSORS:
            continue
        prefix = key + '.'
        series = properties[prefix + 'build.series']
        variant = properties[prefix + 'build.variant']
        header = ROOT / 'variants' / variant / f'variant_{processor}.h'
        source = header.read_text(encoding='utf-8')
        rx = set(re.findall(r'^\s*#\s*define\s+PIN_SERIAL([1-8])_RX\b', source, re.M))  # find all defined serial instances in the variant file
        tx = set(re.findall(r'^\s*#\s*define\s+PIN_SERIAL([1-8])_TX\b', source, re.M))
        ports = sorted(rx & tx)
        if not ports:
            raise ValueError(f'No serial ports found for {processor}')

        for case in BUILD_CASES:
            if processor not in case.get('processors', REPRESENTATIVE_PROCESSORS):
                continue
            selected_ports = ports if case.get('each_serial') else [None]
            for port in selected_ports:
                yield make_build(case, processor, board, port, package, properties, prefix, series)


def make_build(case, processor, board, port, package, properties, prefix, series):
    label = case['label'].format(port=port)
    sketch = case['sketch']
    modules = case['peripherals']
    unknown_modules = set(modules) - {'ADC', 'DAC', 'UART', 'SPI', 'I2C', 'TIM'}
    if unknown_modules:
        raise ValueError(f'Unknown peripherals in {label}: {sorted(unknown_modules)}')
    options = {'pnum': processor, **DEFAULT_BOARD_OPTIONS, **case.get('board_options', {})}
    flags = ['-include "{runtime.platform.path}/ci/miscellaneous/peripherals.h"']
    flags.extend(f'-DCI_ENABLE_{module}' for module in modules)
    if port is not None:
        # V003 has no serial menu; route its only UART via a build flag.
        if f'{board}.menu.xserial.serial{port}' in properties:
            options['xserial'] = f'serial{port}'
        else:
            flags.append(f'-DSERIAL_UART_INSTANCE={port}')
    elif 'UART' not in modules:
        flags.append('-DHWSERIAL_NONE')
    flags.extend(case.get('extra_flags', []))
    build_properties = dict(case.get('build_properties', {}))
    # Keep the peripheral override when a case also supplies build.extra_flags.
    if build_properties.get('build.extra_flags'):
        flags.append(build_properties['build.extra_flags'])
    build_properties['build.extra_flags'] = ' '.join(flags)

    name = f'{processor}_{label}-{Path(sketch).name}'
    return {
        'name': name,
        'fqbn': f'{package}:{board}:' + ','.join(f'{key}={value}' for key, value in options.items()),
        'sketch': sketch,
        'build_properties': build_properties,
        'build_dir': f'build/Miscellaneous/{name}',
        'ld': f"system/{series}/SRC/Ld/{properties.get(prefix + 'build.ldscript', 'Link.ld')}",
        'linker_vars': (
            f"__FLASH_SIZE={properties[prefix + 'upload.maximum_size']} "
            f"__RAM_SIZE={properties[prefix + 'upload.maximum_data_size']}"
        ),
    }
