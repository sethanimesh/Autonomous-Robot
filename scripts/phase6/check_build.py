#!/usr/bin/env python3
"""Compile robot code and run offline regressions; no ROS or hardware startup."""
from pathlib import Path
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[2]
    files = sorted((root / 'robot').rglob('*.py'))
    files += sorted((root / 'scripts').rglob('*.py'))
    for path in files:
        compile(path.read_bytes(), str(path), 'exec')
    print('Compiled {} Python files. Running offline tests.'.format(len(files)), flush=True)
    return subprocess.run(
        [sys.executable, '-m', 'unittest', 'discover', '-s', 'tests'],
        cwd=root, check=False,
    ).returncode


if __name__ == '__main__':
    raise SystemExit(main())
