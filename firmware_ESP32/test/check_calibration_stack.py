#!/usr/bin/env python3
"""Check known Xtensa stack frames in the actual ESP32 ELF (not a PC build).

Run after idf.py build, with the IDF environment active:
    python test/check_calibration_stack.py build/firmware_ESP32.elf
This is a regression guard, not a full bound on NVS/libc/MQTT/interrupt usage.
The runtime stack_min_free log remains mandatory for physical validation.
"""
import argparse
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path)
    parser.add_argument('--objdump', default='xtensa-esp-elf-objdump')
    args = parser.parse_args()
    text = subprocess.check_output([args.objdump, '-d', str(args.elf)], text=True)
    frames = {}
    for name, body in re.findall(r'<([^>]+)>:\n(.*?)(?=\n[0-9a-f]+ <|\Z)', text, re.S):
        entry = re.search(r'entry\s+a1,\s*(0x[0-9a-f]+|\d+)', body)
        if entry:
            frames[name] = int(entry[1], 0)
    header = Path(__file__).resolve().parents[1] / 'main/modules/MQTT/mqtt_lib.h'
    budget = int(re.search(r'#define MQTT_CAL_TASK_STACK (\d+)U', header.read_text())[1])
    paths = {
        'status': ['cal_worker', 'calibration_store_status_body', 'get_active', 'load_blob', 'decode'],
        'commit': ['cal_worker', 'calibration_store_commit', 'write_blob', 'load_blob', 'decode'],
        'cancel': ['cal_worker', 'calibration_store_cancel', 'load_blob', 'decode'],
    }
    # Missing/renamed/inlined symbols require review, not silent success.
    missing = sorted({name for path in paths.values() for name in path} - frames.keys())
    if missing:
        raise SystemExit(f'Cannot verify frames; inspect ELF for: {missing}')
    if frames['cal_worker'] > 512:
        raise SystemExit(f"cal_worker local frame grew to {frames['cal_worker']} B (limit 512)")
    reserve = 2048  # Allowance, not a measured upper bound on library calls.
    for action, path in paths.items():
        known = sum(frames[name] for name in path)
        print(f'{action}: known frames={known} B, library allowance={reserve} B, budget={budget} B')
        if known + reserve > budget:
            raise SystemExit(f'Insufficient stack allowance for {action}')
    print(f"cal_worker frame={frames['cal_worker']} B; check PASSED; verify stack_min_free on board")


if __name__ == '__main__':
    main()
