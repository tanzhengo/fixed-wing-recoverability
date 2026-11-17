#!/usr/bin/env python3
"""Compile frozen recovered evaluators without modifying any original source."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parent
TARGETS = {
    'procedural': ('source/procedural/benchmark.cpp', []),
    'forest': ('source/forest/benchmark.cpp', []),
    'urban_current': ('source/urban_replay/current/replay.cpp', []),
    'urban_instrumented': ('source/urban_replay/instrumented/replay.cpp', ['-DREPLAY_INSTRUMENTED']),
    'urban_frozen': ('source/urban_replay/frozen/replay.cpp', ['-DFROZEN_BASELINE']),
}

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--targets', nargs='+', choices=sorted(TARGETS), default=['procedural', 'forest'])
    p.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    p.add_argument('--eigen-include', type=Path, default=Path('/usr/include/eigen3'))
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'roscpp', 'tf'], text=True))
    report = {}
    for target in a.targets:
        rel, defines = TARGETS[target]
        source = ROOT / rel
        binary = a.out.resolve() / target
        cmd = [a.compiler, '-std=c++14', '-O2'] + defines + ['-I' + str(source.parent),
            '-I' + str(a.eigen_include), str(source), '-o', str(binary)] + flags
        subprocess.run(cmd, check=True)
        report[target] = {'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                         'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(), 'command': cmd}
        print('Built', target, flush=True)
    (a.out / 'recovered_build.json').write_text(json.dumps(report, indent=2) + '\n')

if __name__ == '__main__':
    main()
