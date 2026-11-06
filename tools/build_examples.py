import os
import sys
import subprocess
from subprocess import PIPE
import time
import glob
import json
import re
import shutil
import tempfile
from contextlib import contextmanager
from multiprocessing import Pool
from miscellaneous_builds import miscellaneous_builds

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# USB device modes implemented by the current CH32 TinyUSB port. These apply
# only to TinyUSB examples; other sketches retain their configured FQBN.
TINYUSB_OPTIONS = {
    'CH32V20x': 'tinyusb_usbd',
    'CH32V30x': 'tinyusb_usbfs',
}

SUCCEEDED = "\033[32msucceeded\033[0m"
FAILED = "\033[31mfailed\033[0m"
SKIPPED = "\033[35mskipped\033[0m"
WARNING = "\033[33mwarnings\033[0m "

build_format = '| {:35} | {:9} | {:6} |'
build_separator = '-' * 59

# ci-arduino naming, local checkout fqbn, [family name]
all_boards = [
    ['CH32V00x' , 'local:ch32v:CH32V00x_EVT' ],
    ['CH32VM00X-CH32V002F4', 'local:ch32v:CH32VM00X_EVT:pnum=CH32V002F4', 'CH32VM00X'],
    ['CH32VM00X-CH32V006K8', 'local:ch32v:CH32VM00X_EVT:pnum=CH32V006K8', 'CH32VM00X'],
    ['CH32L10x' , 'local:ch32v:CH32L10x_EVT' ],
    ['CH32V10x' , 'local:ch32v:CH32V10x_EVT' ],
    ['CH32V20x' , 'local:ch32v:CH32V20x_EVT' ],
    ['CH32V30x' , 'local:ch32v:CH32V30x_EVT' ],
    ['CH32X035' , 'local:ch32v:CH32X035_EVT' ],
]


@contextmanager
def local_core_build():
    """Expose the checkout as a local platform without changing installed cores."""
    cli = shutil.which('arduino-cli')
    if cli is None:
        raise RuntimeError('arduino-cli was not found on PATH')
    directory_result = subprocess.run(
        [cli, 'config', 'get', 'directories', '--json'],
        check=True, stdout=PIPE, stderr=PIPE, text=True,
    )
    directories = json.loads(directory_result.stdout)

    # Local platforms have no Boards Manager tool dependencies. Resolve the
    # compiler required by this checkout from the existing tool installation.
    with open(os.path.join(REPO_ROOT, 'platform.txt'), encoding='utf-8') as f:
        toolchain = re.search(
            r'^toolchain_dir=\{runtime\.tools\.(.+)-([0-9][^{}]*)\.path\}\s*$',
            f.read(), re.MULTILINE,
        )
    if toolchain is None:
        raise RuntimeError('Cannot determine the compiler dependency from platform.txt')
    tool_name, tool_version = toolchain.groups()
    compiler_dir = os.path.join(
        directories['data'], 'packages', 'ch32-community', 'tools',
        tool_name, tool_version,
    )
    if not os.path.isdir(compiler_dir):
        raise RuntimeError(
            f'Missing compiler: {compiler_dir}. Install ch32-community:ch32v '
            'with arduino-cli core install to provide the compiler tools.'
        )

    compile_options = [
        '--build-property', f'toolchain_dir={compiler_dir}',
        '--libraries', os.path.join(REPO_ROOT, 'libraries'),
    ]
    user_libraries = os.path.join(directories['user'], 'libraries')
    if os.path.isdir(user_libraries):
        compile_options.extend(['--libraries', user_libraries])

    with tempfile.TemporaryDirectory(prefix='ch32-local-core-') as sketchbook:
        hardware = os.path.join(sketchbook, 'hardware', 'local')
        os.makedirs(hardware)
        core_link = os.path.join(hardware, 'ch32v')
        try:
            os.symlink(REPO_ROOT, core_link, target_is_directory=True)
        except OSError:
            if os.name != 'nt':
                raise
            # Junctions work without Windows developer mode/admin privileges.
            import _winapi
            _winapi.CreateJunction(REPO_ROOT, core_link)
        try:
            yield {
                'cli': cli,
                'sketchbook': sketchbook,
                'compile_options': compile_options,
            }
        finally:
            # Remove only the link before TemporaryDirectory cleans its own tree.
            if os.path.islink(core_link):
                os.unlink(core_link)
            else:
                os.rmdir(core_link)


def is_tinyusb_example(sketch):
    examples = os.path.join(REPO_ROOT, 'libraries', 'Adafruit_TinyUSB_Arduino', 'examples')
    try:
        return os.path.commonpath([examples, os.path.abspath(sketch)]) == examples
    except ValueError:  # Paths on different Windows drives cannot match.
        return False


def sketch_fqbn(variant, sketch):
    fqbn = variant[1]
    family = variant[2] if len(variant) > 2 else variant[0]
    if is_tinyusb_example(sketch) and family in TINYUSB_OPTIONS:
        parts = fqbn.split(':', 3)
        options = parts[3].split(',') if len(parts) == 4 else []
        if not any(option.startswith('usb=') for option in options):
            options.append('usb=' + TINYUSB_OPTIONS[family])
        return ':'.join(parts[:3]) + ':' + ','.join(options)
    return fqbn


def sketch_skip_directories(sketch):
    """Read sketch markers plus inherited metadata owned by this repository."""
    sketchdir = os.path.dirname(os.path.abspath(sketch))
    yield sketchdir
    try:
        relative = os.path.relpath(sketchdir, REPO_ROOT)
    except ValueError:
        return
    if relative == os.pardir or relative.startswith(os.pardir + os.sep):
        return
    metadata_root = os.path.join(REPO_ROOT, 'ci', 'skips')
    directory = os.path.join(metadata_root, relative)
    while True:
        yield directory
        if directory == metadata_root:
            break
        directory = os.path.dirname(directory)


def sketch_is_skipped(variant, sketch):
    name = variant[0]
    family = variant[2] if len(variant) > 2 else name
    # Upstream TinyUSB uses board IDs such as CH32V20x_EVT in its markers.
    skip_names = {family, name, variant[1].split(':')[2]}
    if is_tinyusb_example(sketch):
        options = sketch_fqbn(variant, sketch).split(':')[-1].split(',')
        if 'usb=none' in options:
            return True

    for directory in sketch_skip_directories(sketch):
        skip_marker = any(
            os.path.exists(os.path.join(directory, f".{key}.test.skip"))
            for key in skip_names
        )
        only_markers = glob.glob(os.path.join(directory, ".*.test.only"))
        matches_only = any(
            os.path.exists(os.path.join(directory, f".{key}.test.only"))
            for key in skip_names
        )
        if skip_marker or (only_markers and not matches_only):
            return True

        skip_txt = os.path.join(directory, '.skip.txt')
        if os.path.exists(skip_txt):
            with open(skip_txt, encoding='utf-8') as f:
                names = {line.partition('#')[0].strip() for line in f}
            if '*' in names or names & skip_names:
                return True
    return False


# return [succeeded, failed, skipped], output
def build_sketch(variant, sketch, local_build, build_properties=None, display_name=None):
    ret = [0, 0, 0]
    fqbn = sketch_fqbn(variant, sketch)
    start_time = time.monotonic()

    if sketch_is_skipped(variant, sketch):
        ret[2] = 1
        success = SKIPPED
    else:
        build_env = os.environ.copy()
        build_env['ARDUINO_DIRECTORIES_USER'] = local_build['sketchbook']
        property_options = []
        for key, value in (build_properties or {}).items():
            property_options.extend(['--build-property', f'{key}={value}'])
        build_result = subprocess.run(
            [local_build['cli'], 'compile', '--warnings', 'all', '--fqbn', fqbn,
             *local_build['compile_options'], *property_options, os.path.abspath(sketch)],
            env=build_env, stdout=PIPE, stderr=PIPE,
        )
        # get stderr into a form where warning/error was output to stderr
        if build_result.returncode != 0:
            success = FAILED
            ret[1] = 1
        else:
            ret[0] = 1
            if build_result.stderr:
                success = WARNING
            else:
                success = SUCCEEDED

    build_duration = time.monotonic() - start_time
    output = [build_format.format(
        display_name or os.path.basename(sketch), success, '{:5.2f}s'.format(build_duration)
    )]

    if success != SKIPPED:
        # Build failed
        if build_result.returncode != 0:
            output.append(build_result.stdout.decode("utf-8"))

        # Build with warnings
        if build_result.stderr:
            output.append(build_result.stderr.decode("utf-8"))
    return ret, "\n".join(output)


def build_sketch_task(task):
    index, variant, sketch, local_build = task
    ret, output = build_sketch(variant, sketch, local_build)
    return index, ret, output


# return [succeeded, failed, skipped]
def build_variant(variant, local_build):
    print()
    print(build_separator)
    print('| {:^56} |'.format('Board ' + variant[0]))
    print(build_separator)
    print(build_format.format('Example', '\033[39mResult\033[0m', 'Time'))
    print(build_separator)

    family = variant[2] if len(variant) > 2 else variant[0]
    options = variant[1].split(':', 3)
    usb_disabled = len(options) == 4 and 'usb=none' in options[3].split(',')
    tinyusb_supported = family in TINYUSB_OPTIONS and not usb_disabled
    examples = [
        sketch for sketch in all_examples
        if tinyusb_supported or not is_tinyusb_example(sketch)
    ]
    # Unsupported TinyUSB examples do not belong in this board's results.
    if not examples:
        return [0, 0, 0]

    ordered_examples = sorted(
        examples,
        key=lambda sketch: (
            not sketch_is_skipped(variant, sketch),
            os.path.basename(sketch).casefold(),
            sketch.casefold(),
        ),
    )
    results = [None] * len(ordered_examples)
    pending_output = {}
    next_to_print = 0
    with Pool(processes=os.cpu_count()) as pool:
        pool_args = (
            (index, variant, sketch, local_build)
            for index, sketch in enumerate(ordered_examples)
        )
        for index, ret, output in pool.imap_unordered(
            build_sketch_task, pool_args, chunksize=1
        ):
            results[index] = ret
            pending_output[index] = output
            while next_to_print in pending_output:
                print(pending_output.pop(next_to_print), flush=True)
                next_to_print += 1

    # Sum each result column (succeeded, failed, skipped).
    return list(map(sum, zip(*results)))


def build_miscellaneous_task(task):
    case, local_build = task
    sketch_name = os.path.basename(case['sketch'])
    # Arduino's default build directory is shared by repeated builds of a sketch.
    # Different peripheral flags must never write into the same directory.
    local_build = {
        **local_build,
        'compile_options': [
            *local_build['compile_options'], '--build-path',
            os.path.join(local_build['sketchbook'], 'builds', case['name']),
        ],
    }
    ret, output = build_sketch(
        [case['name'], case['fqbn']],
        os.path.join(case['sketch'], sketch_name + '.ino'), local_build,
        case['build_properties'],
        display_name=case['name'].removesuffix('-' + sketch_name),
    )
    return ret, output


def build_miscellaneous(local_build):
    result = [0, 0, 0]
    cases = sorted(miscellaneous_builds(),
                   key=lambda case: os.path.basename(case['sketch']).casefold())
    current_sketch = None
    # Each CLI also compiles in parallel; bound concurrent configurations.
    with Pool(processes=min(4, os.cpu_count() or 1)) as pool:
        tasks = ((case, local_build) for case in cases)
        results = pool.imap(build_miscellaneous_task, tasks, chunksize=1)
        for case, (ret, output) in zip(cases, results):
            sketch_name = os.path.basename(case['sketch'])
            if sketch_name != current_sketch:
                print()
                print(build_separator)
                print('| {:^56} |'.format('Miscellaneous/' + sketch_name))
                print(build_separator)
                print(build_format.format('Example', '\033[39mResult\033[0m', 'Time'))
                print(build_separator)
                current_sketch = sketch_name
            print(output, flush=True)
            result = list(map(sum, zip(result, ret)))
    return result

if __name__ == '__main__':
    # Resolve all sketch paths from the repository, even when invoked elsewhere.
    os.chdir(REPO_ROOT)

    # build all variants if input not existed
    if len(sys.argv) > 1:
        build_boards = list(filter(lambda x: sys.argv[1] in x[0], all_boards))
    else:
        build_boards = all_boards

    all_examples = []
    for library_dir in ('libraries', os.path.join('ci', 'libraries')):
        if os.path.isdir(library_dir):
            all_examples.extend(
                sketch for sketch in glob.iglob(
                    os.path.join(library_dir, '**', '*.ino'), recursive=True
                )
                if (
                    os.path.splitext(os.path.basename(sketch))[0]
                    == os.path.basename(os.path.dirname(sketch))
                )
            )
    all_examples.sort(key=lambda sketch: (os.path.basename(sketch).casefold(), sketch.casefold()))

    total_time = time.monotonic()
    total_result = [0, 0, 0]
    with local_core_build() as local_build:
        print(f'Building against local core: {REPO_ROOT}', flush=True)
        if len(sys.argv) > 1 and sys.argv[1] == 'Miscellaneous':
            total_result = build_miscellaneous(local_build)
        for b in build_boards:
            ret = build_variant(b, local_build)
            total_result = list(map(lambda x, y: x + y, total_result, ret))

    # Build Summary
    total_time = time.monotonic() - total_time
    print(build_separator)
    print(F"Build Summary: {total_result[0]} {SUCCEEDED}, {total_result[1]} {FAILED}, {total_result[2]} {SKIPPED} and took {total_time:.2f}s")
    print(build_separator)
    sys.exit(total_result[1])
