"""Upload through the unmodified UF2 converter to one validated volume."""

import argparse
import os
from pathlib import Path
import subprocess
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--volume', required=True, help='mounted UF2 volume')
    parser.add_argument('--family', required=True, help='UF2 family ID')
    parser.add_argument('--base', required=True, help='application flash address')
    parser.add_argument('input', help='compiled BIN file')
    args = parser.parse_args(argv)

    volume_name = args.volume
    if os.name == 'nt':
        drive, tail = os.path.splitdrive(volume_name)
        if drive and not tail:
            # Arduino accepts D: as a port; use its root, not its current folder.
            volume_name += os.sep
    try:
        volume = Path(volume_name).resolve()
        if not os.path.ismount(volume) or not (volume / 'INFO_UF2.TXT').is_file():
            parser.error(f'Not a mounted UF2 volume: {volume}. Enter the bootloader '
                         'and select its mounted volume.')
    except OSError as error:
        parser.error(f'Cannot access UF2 volume: {error}')

    # Conversion-only mode writes exclusively to the explicitly selected volume.
    return subprocess.run([
        sys.executable, '-I', str(Path(__file__).with_name('uf2conv.py')),
        '-c', '-f', args.family, '-b', args.base,
        '-o', str(volume / 'NEW.UF2'), args.input,
    ]).returncode


if __name__ == '__main__':
    sys.exit(main())
