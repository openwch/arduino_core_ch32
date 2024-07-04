#!/usr/bin/env python3


# mcu: march, mabi, math_lib_gcc, IQ_math_RV32, ch_extra_lib
mcu_list = {
    'QingKe-V2A': {'march': 'rv32ecxw', 'mabi': 'ilp32e', 'ch_extra_lib': '-lprintf'},
    'QingKe-V2C': {'march': 'rv32ecxw', 'mabi': 'ilp32e', 'ch_extra_lib': '-lprintf'},
    'QingKe-V3A': {'march': 'rv32imac', 'mabi': 'ilp32', 'ch_extra_lib': '-lprintf'},
    'QingKe-V4B': {'march': 'rv32imacxw', 'mabi': 'ilp32', 'ch_extra_lib': '-lprintf'},
    'QingKe-V4C': {'march': 'rv32imacxw', 'mabi': 'ilp32', 'ch_extra_lib': '-lprintf'},
    'QingKe-V4F': {'march': 'rv32imafcxw', 'mabi': 'ilp32f', 'ch_extra_lib': '-lprintfloat'},
}

usb_list = {
    'tinyusb_usbd': {
        'name': 'Adafruit TinyUSB with USBD',
        'usb_flags': '-DUSBCON -DUSE_TINYUSB -DCFG_TUD_WCH_USBIP_FSDEV=1 "-I{runtime.platform.path}/libraries/Adafruit_TinyUSB_Arduino/src/arduino"'
    },
    'tinyusb_usbfs': {
        'name': 'Adafruit TinyUSB with USBFS',
        'usb_flags': '-DUSBCON -DUSE_TINYUSB -DCFG_TUD_WCH_USBIP_USBFS=1 "-I{runtime.platform.path}/libraries/Adafruit_TinyUSB_Arduino/src/arduino"'
    },
    'tinyusb_usbhs': {
        'name': 'Adafruit TinyUSB with USBHS',
        'usb_flags': '-DUSBCON -DUSE_TINYUSB -DCFG_TUD_WCH_USBIP_USBHS=1 "-I{runtime.platform.path}/libraries/Adafruit_TinyUSB_Arduino/src/arduino"'
    },
}

# series: name, pnums
board_list = {
    'CH32V00x': {
        'name': 'CH32V00x_EVT',
        'info': '',
        'usb': [],
        'hsi': [48, 24, 8],
        'hse': [48, 24, 8],
        'pnums': {
            'CH32V003F4': {'name': 'CH32V003F4 EVT', 'maximum_size': 16384, 'maximum_data_size': 2048, 'mcu': 'QingKe-V2A', 'chip': 'CH32V003F4'},
        }
    },
    'CH32VM00X': {
        'name': 'CH32VM00X_EVT',
        'info': 'including V/M 002 004 005 006 007',
        'usb': [],
        'hsi': [48, 24, 8],
        'hse': [48, 24, 8],
        'pnums': {
            'CH32V006K8': {'name': 'CH32V006K8 EVT', 'maximum_size': 63488, 'maximum_data_size': 8192, 'mcu': 'QingKe-V2C', 'chip': 'CH32V006K8'},
        }
    },
    'CH32X035': {
        'name': 'CH32X035_EVT',
        'info': '',
        'usb': [],
        'hsi': [48, 24, 16, 12, 8],
        'hse': [],
        'pnums': {
            'CH32X035G8U': {'name': 'CH32X035G8U EVT', 'maximum_size': 63488, 'maximum_data_size': 20480, 'mcu': 'QingKe-V4C', 'chip': 'CH32X035G8U'},
        }
    },
    'CH32V10x': {
        'name': 'CH32V10x_EVT',
        'info': '-lprintf, CH32V10x_3V3: 3.3V power supply  CH32V10x_5V: 5V power supply',
        'usb': [],
        'hsi': [72, 56, 48, 8],
        'hse': [72, 56, 48, 8],
        'pnums': {
            'CH32V103R8T6': {'name': 'CH32V103R8T6 EVT', 'maximum_size': 65536, 'maximum_data_size': 20480, 'mcu': 'QingKe-V3A', 'chip': 'CH32V10x_3V3'},
        }
    },
    'CH32V20x': {
        'name': 'CH32V20x_EVT',
        'info': '',
        'usb': ['tinyusb_usbd', 'tinyusb_usbfs'],
        'hsi': [144, 120, 96, 72, 56, 48, 0],
        'hse': [144, 120, 96, 72, 56, 48, 0],
        'pnums': {
            'CH32V203RB': {'name': 'CH32V203RB EVT', 'maximum_size': 131072, 'maximum_data_size': 65536, 'mcu': 'QingKe-V4C', 'chip': 'CH32V203'},
            'CH32V203G8': {'name': 'CH32V203G8 EVT', 'maximum_size': 65536, 'maximum_data_size': 20480, 'mcu': 'QingKe-V4B', 'chip': 'CH32V203'},
            'CH32V203G6': {'name': 'CH32V203G6 EVT', 'maximum_size': 32768, 'maximum_data_size': 10240, 'mcu': 'QingKe-V4B', 'chip': 'CH32V203'},
            'CH32V203C8': {'name': 'CH32V203C8', 'maximum_size': 65536, 'maximum_data_size': 20480, 'mcu': 'QingKe-V4B', 'chip': 'CH32V203'},
            'CH32V203C6': {'name': 'CH32V203C6', 'maximum_size': 32768, 'maximum_data_size': 10240, 'mcu': 'QingKe-V4B', 'chip': 'CH32V203'},
            'CH32V203G6_ADAFRUIT_QTPY': {'name': 'Adafruit QTPy CH32V203G6', 'maximum_size': 32768, 'maximum_data_size': 10240, 'mcu': 'QingKe-V4B', 'chip': 'CH32V203'},
        }
    },
    'CH32V30x': {
        'name': 'CH32V30x_EVT',
        'info': '-lprintfloat, CH32V30x_C: connected product_line  CH32V30x: normal product_line',
        'usb': ['tinyusb_usbhs', 'tinyusb_usbfs'],
        'hsi': [144, 120, 96, 72, 56, 48, 0],
        'hse': [144, 120, 96, 72, 56, 48, 0],
        'pnums': {
            'CH32V307VCT6': {'name': 'CH32V307VCT6 EVT', 'maximum_size': 262144, 'maximum_data_size': 65536, 'mcu': 'QingKe-V4F', 'chip': 'CH32V30x_C'},
        }
    },
    'CH32L10x': {
        'name': 'CH32L10x_EVT',
        'info': '-lprintf',
        'usb': [],
        'hsi': [96, 72, 56, 48, 0, 'HSI_LP'],
        'hse': [96, 72, 56, 48, 0],
        'pnums': {
            'CH32L103C8T6': {'name': 'CH32L103C8T6 EVT', 'maximum_size': 65536, 'maximum_data_size': 20480, 'mcu': 'QingKe-V4C', 'chip': 'CH32L10x'},
        }
    }
}


def build_global_menu():
    print("""# See: https://arduino.github.io/arduino-cli/latest/platform-specification/

menu.pnum=Board Select
menu.clock=Clock Select
menu.xserial=U(S)ART support
menu.usb=USB support (if available)
menu.xusb=USB speed (if available)
menu.virtio=Virtual serial support

menu.opt=Optimize
menu.dbg=Debug symbols and core logs
menu.rtlib=C Runtime Library
menu.upload_method=Upload method""")


def build_header(series, values):
    print()
    print()
    print()
    print('#'*78)
    name = values["name"]
    print(f'##{name} Board   {values["info"]}')
    print()
    print(f'{name}.name={series}')
    print(f'{name}.build.core=arduino')
    print(f'{name}.build.board={name}')
    print(f'{name}.upload.maximum_size=0')
    print(f'{name}.upload.maximum_data_size=0')
    print(f'{name}.build.variant_h=variant_{{build.board}}.h')
    print(f'{name}.debug.tool=gdb-WCH_LinkE')
    print()


def build_pnum(series, values):
    print()
    for p, pv in values['pnums'].items():
        menu = f'{values["name"]}.menu.pnum.{p}'
        mcu = pv["mcu"]
        print(f'#{pv["name"]} Board')
        print(f'{menu}={pv["name"]}')
        print(f'{menu}.node={p.replace("CH32", "NODE_")}')
        print(f'{menu}.upload.maximum_size={pv["maximum_size"]}')
        print(f'{menu}.upload.maximum_data_size={pv["maximum_data_size"]}')
        print(f'{menu}.build.mcu={mcu}')
        print(f'{menu}.build.board={p}')
        print(f'{menu}.build.series={series}')
        print(f'{menu}.build.variant={series}/{p}')
        print(f'{menu}.build.chip={pv["chip"]}')
        print(f'{menu}.build.march={mcu_list[mcu]["march"]}')
        print(f'{menu}.build.mabi={mcu_list[mcu]["mabi"]}')
        print(f'{menu}.build.math_lib_gcc=-lm')
        print(f'{menu}.build.IQ_math_RV32=')
        print(f'{menu}.build.ch_extra_lib={mcu_list[mcu]["ch_extra_lib"]}')
        print()


def build_usb(series, values):
    if len(values['usb']) == 0:
        return
    print()
    print("# USB support")
    name = values["name"]
    menu = f'{name}.menu.usb'
    print(f'{menu}.none=None')
    print(f'{menu}.none.build.usb_flags=')
    for usb in values['usb']:
        print(f'{menu}.{usb}={usb_list[usb]["name"]}')
        print(f'{menu}.{usb}.build.usb_flags={usb_list[usb]["usb_flags"]}')


def build_upload(series, values):
    print()
    print("# Upload menu")
    name = values["name"]
    menu = f'{name}.menu.upload_method'

    menu_swd = f'{menu}.swdMethod'
    print(f'{menu_swd}=WCH-SWD')
    print(f'{menu_swd}.upload.protocol=')
    print(f'{menu_swd}.upload.options=')
    print(f'{menu_swd}.upload.tool=WCH_linkE')

    menu_isp = f'{menu}.ispMethod'
    print(f'{menu_isp}=WCH-ISP')
    print(f'{menu_isp}.upload.protocol=')
    print(f'{menu_isp}.upload.options=')
    print(f'{menu_isp}.upload.tool=wchisp')

    print()


def build_optimization(series, values):
    print()
    print("# Optimizations")
    name = values["name"]
    menu = f'{name}.menu.opt'

    print(f'{menu}.osstd=Smallest (-Os default)')
    print(f'{menu}.osstd.build.flags.optimize=-Os')
    print(f'{menu}.oslto=Smallest (-Os) with LTO')
    print(f'{menu}.oslto.build.flags.optimize=-Os -flto')

    print(f'{menu}.o1std=Fast (-O1)')
    print(f'{menu}.o1std.build.flags.optimize=-O1')
    print(f'{menu}.o1lto=Fast (-O1) with LTO')
    print(f'{menu}.o1lto.build.flags.optimize=-O1 -flto')

    print(f'{menu}.o2std=Faster (-O2)')
    print(f'{menu}.o2std.build.flags.optimize=-O2')
    print(f'{menu}.o2lto=Faster (-O2) with LTO')
    print(f'{menu}.o2lto.build.flags.optimize=-O2 -flto')

    print(f'{menu}.o3std=Fastest (-O3)')
    print(f'{menu}.o3std.build.flags.optimize=-O3')
    print(f'{menu}.o3lto=Fastest (-O3) with LTO')
    print(f'{menu}.o3lto.build.flags.optimize=-O3 -flto')

    print(f'{menu}.ogstd=Debug (-Og)')
    print(f'{menu}.ogstd.build.flags.optimize=-Og')
    print(f'{menu}.o0std=No Optimization (-O0)')
    print(f'{menu}.o0std.build.flags.optimize=-O0')
    print()


def build_clock(series, values):
    print()
    print("# Clock Select")
    name = values["name"]
    menu = f'{name}.menu.clock'
    for hsi in values['hsi']:
        if hsi == 0:
            print(f'{menu}.HSI=HSI Internal')
            print(f'{menu}.HSI.build.flags.clock=-DSYSCLK_FREQ_HSI=HSI_VALUE -DF_CPU=HSI_VALUE')
        elif hsi == 'HSI_LP':
            print(f'{menu}.HSI_LP=HSI_LP Internal')
            print(f'{menu}.HSI_LP.build.flags.clock=-DSYSCLK_FREQ_HSI_LP=HSI_LP_VALUE -DF_CPU=HSI_LP_VALUE')
        else:
            print(f'{menu}.{hsi}MHz_HSI={hsi}MHz Internal')
            print(f'{menu}.{hsi}MHz_HSI.build.flags.clock=-DSYSCLK_FREQ_{hsi}MHz_HSI={hsi}000000 -DF_CPU={hsi}000000')
    for hse in values['hse']:
        if hse == 0:
            print(f'{menu}.HSE=HSE External')
            print(f'{menu}.HSE.build.flags.clock=-DSYSCLK_FREQ_HSE=HSE_VALUE -DF_CPU=HSE_VALUE')
        else:
            print(f'{menu}.{hse}MHz_HSE={hse}MHz External')
            print(f'{menu}.{hse}MHz_HSE.build.flags.clock=-DSYSCLK_FREQ_{hse}MHz_HSE={hse}000000 -DF_CPU={hse}000000')
    print()


def build_debug(series, values):
    print()
    print("# Debug information")
    name = values["name"]
    menu = f'{name}.menu.dbg'
    print(f'{menu}.none=None')
    print(f'{menu}.none.build.flags.debug=-DNDEBUG')
    print(f'{menu}.enable_sym=Symbols Enabled (-g)')
    print(f'{menu}.enable_sym.build.flags.debug=-g -DNDEBUG')
    print(f'{menu}.enable_log=Core logs Enabled')
    print(f'{menu}.enable_log.build.flags.debug=')
    print(f'{menu}.enable_all=Core Logs and Symbols Enabled (-g)')
    print(f'{menu}.enable_all.build.flags.debug=-g')
    print()


def build_runtimelib(series, values):
    print()
    print("# C Runtime Library")
    name = values["name"]
    menu = f'{name}.menu.rtlib'
    print(f'{menu}.nano=Newlib Nano (default)')
    print(f'{menu}.nano.build.flags.ldflags=--specs=nano.specs --specs=nosys.specs')
    print(f'{menu}.nanofp=Newlib Nano + Float Printf')
    print(f'{menu}.nanofp.build.flags.ldflags=--specs=nano.specs --specs=nosys.specs -u _printf_float')
    print(f'{menu}.nanofs=Newlib Nano + Float Scanf')
    print(f'{menu}.nanofs.build.flags.ldflags=--specs=nano.specs --specs=nosys.specs -u _scanf_float')
    print(f'{menu}.nanofps=Newlib Nano + Float Printf/Scanf')
    print(f'{menu}.nanofps.build.flags.ldflags=--specs=nano.specs --specs=nosys.specs -u _printf_float -u _scanf_float')
    print(f'{menu}.full=Newlib Standard')
    print(f'{menu}.full.build.flags.ldflags=--specs=nosys.specs')
    print()


def make_board(series, values):
    build_header(series, values)
    build_pnum(series, values)
    build_usb(series, values)
    build_upload(series, values)
    build_clock(series, values)
    build_optimization(series, values)
    build_debug(series, values)
    build_runtimelib(series, values)

# ------------------------------
# main
# ------------------------------
build_global_menu()

for k, v in board_list.items():
    make_board(k, v)
