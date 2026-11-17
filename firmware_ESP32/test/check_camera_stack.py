"""Compile the real ESP32 sources and check per-function stack frames.

Run after `idf.py build`: python3 test/check_camera_stack.py
Uses the same compiler/flags as build/compile_commands.json; only temporary
objects are produced. This is not a measurement of worst-case call-chain stack
or a replacement for the on-device high-water marks after capture/snapshot.
"""
import json
from pathlib import Path
import shlex
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
BUDGETS = {
    'mqtt_lib.c': {'cal_result_handler': 768},
    'camera_lib.c': {
        'camera_cal_report': 128,
        'camera_handle_calibration': 1280,
        'camera_task': 768,
    },
}


def main():
    commands = json.loads((ROOT / 'build/compile_commands.json').read_text())
    checked = set()
    with tempfile.TemporaryDirectory(prefix='selfiq-stack-') as directory:
        for entry in commands:
            name = Path(entry['file']).name
            if name not in BUDGETS:
                continue
            args = shlex.split(entry['command'])
            obj = Path(directory) / (name + '.o')
            args[args.index('-o') + 1] = str(obj)
            subprocess.run(args + ['-fstack-usage'], cwd=entry['directory'], check=True)
            frames = {}
            for line in obj.with_suffix('.su').read_text().splitlines():
                symbol, size, kind = line.split('\t')
                frames[symbol.rsplit(':', 1)[-1]] = (int(size), kind)
            for symbol, limit in BUDGETS[name].items():
                if symbol not in frames:
                    raise RuntimeError(f'Missing stack report for {symbol}')
                size, kind = frames[symbol]
                if kind != 'static' or size > limit:
                    raise RuntimeError(f'{symbol}: {size} bytes ({kind}), limit {limit}')
                print(f'{symbol}: {size} bytes <= {limit}')
                checked.add((name, symbol))
    expected = {(name, symbol) for name, limits in BUDGETS.items() for symbol in limits}
    if checked != expected:
        raise RuntimeError('Incomplete build compile database; run idf.py build first')
    print('Stack frame budgets OK; physical capture/snapshot margin still needs validation.')


if __name__ == '__main__':
    main()
