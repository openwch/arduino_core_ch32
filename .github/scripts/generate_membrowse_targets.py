#!/usr/bin/env python3
"""Expand the concise MCU and example lists into a MemBrowse matrix."""

import argparse
import json
import os
import shlex
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / ".github" / "membrowse-targets.json"
sys.path.insert(0, str(ROOT / "tools"))
from miscellaneous_builds import miscellaneous_builds

CORE_INSTALL_CMD = (
    'CORE_DIR="$(arduino-cli config get directories.data)/packages/'
    'ch32-community/hardware/ch32v" && '
    'rm -rf "$CORE_DIR"/* && '
    'mkdir -p "$CORE_DIR/0.0.0" && '
    'rsync -a --exclude=.git --exclude=build ./ "$CORE_DIR/0.0.0/"'
)


def example_is_skipped(example: str, skip_keys: list[str]) -> bool:
    sketch_dir = ROOT / example
    if sketch_dir.is_file():
        sketch_dir = sketch_dir.parent
    skip_keys = set(skip_keys)

    if any((sketch_dir / f".{key}.test.skip").exists() for key in skip_keys):
        return True

    only_markers = list(sketch_dir.glob(".*.test.only"))
    if only_markers and not any(
        (sketch_dir / f".{key}.test.only").exists() for key in skip_keys
    ):
        return True

    skip_file = sketch_dir / ".skip.txt"
    if skip_file.exists():
        listed_keys = {
            line.strip() for line in skip_file.read_text(encoding="utf-8").splitlines()
        }
        if skip_keys & listed_keys:
            return True

    return False


def generate_targets(config: dict) -> list[dict]:
    setup_cmd = config["setup_cmd"]
    targets = []

    for mcu in config["mcus"]:
        for example in config["examples"]:
            example_path = ROOT / example
            if example_path.is_dir():
                compile_path = example
                sketch = example_path.name
            elif example_path.is_file() and example_path.suffix.lower() == ".ino":
                allowed_roots = (ROOT / "libraries", ROOT / "ci" / "libraries")
                if not any(example_path.is_relative_to(root) for root in allowed_roots):
                    raise ValueError(
                        f"Sketch file must be under libraries or ci/libraries: {example}"
                    )
                compile_path = example
                sketch = example_path.stem
            else:
                raise ValueError(
                    f"Example must be an existing directory or .ino file: {example}"
                )

            skip_keys = mcu.get("skip_keys", [mcu["name"]])
            if example_is_skipped(example, skip_keys):
                continue

            build_dir = f"{mcu['build_dir']}/{sketch}"
            build_cmd = (
                CORE_INSTALL_CMD + ' && '
                f'arduino-cli compile --fqbn {mcu["fqbn"]} '
                f'--build-path "$PWD/{build_dir}" {compile_path}'
            )
            targets.append(
                {
                    "target_name": f"{mcu['name']}/{sketch}",
                    "setup_cmd": setup_cmd,
                    "build_cmd": build_cmd,
                    "elf": f"{build_dir}/{sketch}.ino.elf",
                    "ld": mcu["ld"],
                    "map_file": f"{build_dir}/{sketch}.ino.map",
                    "linker_vars": mcu["linker_vars"],
                }
            )

    for case in miscellaneous_builds('ch32-community:ch32v'):
        sketch = Path(case['sketch']).name
        compile_args = ['arduino-cli', 'compile', '--fqbn', case['fqbn']]
        for key, value in case['build_properties'].items():
            compile_args.extend(['--build-property', f'{key}={value}'])
        build_dir = case['build_dir']
        build_cmd = (
            CORE_INSTALL_CMD + ' && ' + shlex.join(compile_args)
            + f' --build-path "$PWD/{build_dir}" ' + shlex.quote(case['sketch'])
        )
        targets.append({
            'target_name': f"Miscellaneous/{case['name']}",
            'setup_cmd': setup_cmd,
            'build_cmd': build_cmd,
            'elf': f'{build_dir}/{sketch}.ino.elf',
            'ld': case['ld'],
            'map_file': f'{build_dir}/{sketch}.ino.map',
            'linker_vars': case['linker_vars'],
        })

    names = [target["target_name"] for target in targets]
    if len(names) != len(set(names)):
        raise ValueError("Generated MemBrowse target names are not unique")

    return targets


def generate_mcu_matrix(config: dict) -> list[dict]:
    targets = generate_targets(config)
    target_names = [target["target_name"] for target in targets]
    return [
        {"mcu": mcu["name"]}
        for mcu in config["mcus"]
        if any(name.startswith(f"{mcu['name']}/") for name in target_names)
    ] + [{'mcu': 'Miscellaneous'}]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mcu-matrix",
        action="store_true",
        help="Output one matrix entry per configured MCU instead of per target",
    )
    args = parser.parse_args()

    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    matrix = generate_mcu_matrix(config) if args.mcu_matrix else generate_targets(config)
    compact_json = json.dumps(matrix, separators=(",", ":"))

    github_output = os.environ.get("GITHUB_OUTPUT")
    if github_output:
        with open(github_output, "a", encoding="utf-8") as output:
            output.write(f"matrix={compact_json}\n")
    else:
        print(compact_json)


if __name__ == "__main__":
    try:
        main()
    except (KeyError, OSError, ValueError, json.JSONDecodeError) as error:
        print(f"Error generating MemBrowse targets: {error}", file=sys.stderr)
        sys.exit(1)
