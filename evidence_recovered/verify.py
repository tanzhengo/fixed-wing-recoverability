#!/usr/bin/env python3
"""Verify lightweight originals and optionally full archived evidence by SHA-256."""
import argparse
import hashlib
import json
from pathlib import Path
PACKAGE = Path(__file__).resolve().parent

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--artifact-root', type=Path, help='Historical workspace root preserving listed relative paths')
    p.add_argument('--raw-root', type=Path, help='Alternative archive location, preserving paths or unique basenames')
    p.add_argument('--require-external', action='store_true')
    p.add_argument('--out', type=Path, help='Optional JSON report')
    a = p.parse_args()
    report = {'package': [], 'external': []}
    for r in json.loads((PACKAGE / 'COPIED_FILES.json').read_text())['files']:
        actual = digest(PACKAGE / r['path'])
        assert actual == r['sha256'], r['path']
        report['package'].append({'path': r['path'], 'sha256': actual, 'matched': True})
    if a.artifact_root or a.raw_root or a.require_external:
        external = json.loads((PACKAGE / 'EXTERNAL_ARTIFACTS.json').read_text())['files']
        names = [Path(r['path']).name for r in external]
        for r in external:
            candidates = []
            if a.artifact_root:
                candidates.append(a.artifact_root / r['path'])
            if a.raw_root:
                candidates.append(a.raw_root / r['path'])
                if names.count(Path(r['path']).name) == 1:
                    candidates.append(a.raw_root / Path(r['path']).name)
            location = next((x for x in candidates if x.is_file()), None)
            status = dict(path=r['path'], present=location is not None)
            if location:
                status['sha256'] = digest(location)
                status['matched'] = status['sha256'] == r['sha256']
                assert status['matched'], r['path']
            elif a.require_external:
                raise FileNotFoundError(r['path'])
            report['external'].append(status)
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps(report, indent=2) + '\n')
    print('Verified {} original package files; {}/{} external files available and verified.'.format(
        len(report['package']), sum(r['present'] for r in report['external']), len(report['external'])))

if __name__ == '__main__':
    main()
