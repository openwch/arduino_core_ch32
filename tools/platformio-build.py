# Copyright 2023-present Maximilian Gerhardt <maximilian.gerhardt@rub.de>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Arduino

Arduino Wiring-based Framework allows writing cross-platform software to
control devices attached to a wide range of Arduino boards to create all
kinds of creative coding, interactive objects, spaces or physical experiences.

http://arduino.cc/en/Reference/HomePage
"""

from os.path import isdir, join
import sys

from SCons.Script import DefaultEnvironment

IS_MAC = sys.platform.startswith("darwin")

env = DefaultEnvironment()
platform = env.PioPlatform()
board = env.BoardConfig()
mcu = env.BoardConfig().get("build.mcu")
chip_series: str = board.get("build.series", "")[0:-1].upper() + "x"
variant_h = board.get("build.arduino.openwch.variant_h")

FRAMEWORK_DIR = platform.get_package_dir("framework-arduino-openwch-ch32")
assert isdir(FRAMEWORK_DIR)

machine_flags = [
    "-march=%s" % board.get("build.march"),
    "-mabi=%s" % board.get("build.mabi"),
    "-msmall-data-limit=8",
    "-msave-restore",
]

env.Append(
    ASFLAGS=machine_flags,
    ASPPFLAGS=[
        "-x", "assembler-with-cpp"
    ],

    CFLAGS=[
        "-std=gnu99"
    ],

    CCFLAGS=machine_flags + [
        "-Os",
        "-Wall",
        "-fmessage-length=0",
        "-fsigned-char",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-common",
        #"-flto",
    ],

    CXXFLAGS=[
        "-fno-threadsafe-statics",
        "-fno-rtti",
        "-fno-exceptions",
        "-fno-use-cxa-atexit",
        "-fpermissive",
        "-std=gnu++14"
    ],

    LINKFLAGS=machine_flags + [
        "-Os",
        "-fmessage-length=0",
        "-fsigned-char",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-common",
        "-Wl,--gc-sections",
        #"-flto",
        "--specs=nosys.specs",
        "--specs=nano.specs",
        "-nostartfiles",
        '-Wl,-Map="%s"' % join("${BUILD_DIR}", "${PROGNAME}.map")
    ],

    CPPDEFINES= [
        ("ARDUINO", 10808),
        ("VARIANT_H", env.StringifyMacro(variant_h)),
        chip_series
    ],

    # LIBS is handled in _LIBFLAGS below

    LIBSOURCE_DIRS=[
        join(FRAMEWORK_DIR, "libraries")
    ],

    CPPPATH=[
        join(FRAMEWORK_DIR, "cores", "arduino"),
        join(FRAMEWORK_DIR, "cores", "arduino", "avr"),
        join(FRAMEWORK_DIR, "cores", "arduino", "ch32"),
        join(FRAMEWORK_DIR, "cores", "arduino", "ch32", "lib"),
        join(FRAMEWORK_DIR, "system", chip_series, "USER"),
        join(FRAMEWORK_DIR, "system", chip_series, "SRC", "Core"),
        join(FRAMEWORK_DIR, "system", chip_series, "SRC", "Debug"),
        join(FRAMEWORK_DIR, "system", chip_series, "SRC", "Startup"),
        join(FRAMEWORK_DIR, "system", chip_series, "SRC", "Peripheral", "inc"),
        join(FRAMEWORK_DIR, "system", chip_series, "SRC", "Peripheral", "src"),
    ],
)
# Use exact same size regexes as the real core
env.Replace(
    SIZEPROGREGEX=r"^(?:\.text|\.data|\.rodata)\s+([0-9]+).*",
    # excludes stack
    SIZEDATAREGEXP=r"^(?:\.data|\.bss|\.noinit)\s+(\d+).*",
)

#
# Target: Build Core Library
#

libs = []

if "build.variant" in board:
    variants_dir = join(
        "$PROJECT_DIR", board.get("build.variants_dir")) if board.get(
            "build.variants_dir", "") else join(FRAMEWORK_DIR, "variants")

    variant = board.get("build.arduino.openwch.variant", board.get("build.variant"))
    env.Append(
        CPPPATH=[
            join(variants_dir, variant)
        ],
        LDSCRIPT_PATH=join(FRAMEWORK_DIR, "system", chip_series, "SRC", "Ld", "Link.ld"),

    )
    env.BuildSources(
        join("$BUILD_DIR", "FrameworkArduinoVariant"),
        join(variants_dir, variant)
    )

# Startup files and debug.c require this to be built using BuildSources or with -Wl,-whole-archive
pre_libs = "-lprintf" if not IS_MAC else ""
env.Prepend(_LIBFLAGS="%s -Wl,--whole-archive " % pre_libs)
env.Append(_LIBFLAGS=" -Wl,--no-whole-archive -lc")

libs.append(env.BuildLibrary(
    join("$BUILD_DIR", "FrameworkArduino"),
    join(FRAMEWORK_DIR, "cores", "arduino"),
))

env.Prepend(LIBS=libs)