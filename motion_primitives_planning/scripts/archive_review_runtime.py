#!/usr/bin/env python3
"""Losslessly archive completed runtime CSVs and retain both content hashes."""
from pathlib import Path
import argparse
import datetime
import gzip
import hashlib
import json
import shutil
from review_runtime_io import raw_sha256


def stored_sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--minimum-bytes', type=int, default=1000000)
    args = parser.parse_args()
    root = args.directory.resolve()
    manifest = root / 'CSV_ARCHIVE_MANIFEST.json'
    entries = json.loads(manifest.read_text())['files'] if manifest.exists() else []
    for source in sorted(root.rglob('*.csv')):
        if source.stat().st_size < args.minimum_bytes:
            continue
        target = Path(str(source) + '.gz')
        if target.exists():
            raise FileExistsError(target)
        raw_hash = stored_sha(source)
        raw_size = source.stat().st_size
        with source.open('rb') as original, target.open('wb') as archive:
            with gzip.GzipFile(filename='', mode='wb', fileobj=archive,
                               compresslevel=6, mtime=0) as compressed:
                shutil.copyfileobj(original, compressed)
        if raw_sha256(target) != raw_hash:
            raise RuntimeError('Archive round-trip hash mismatch: ' + str(source))
        entries.append({'raw_path': str(source.relative_to(root)),
                        'raw_sha256': raw_hash, 'raw_bytes': raw_size,
                        'archive_path': str(target.relative_to(root)),
                        'archive_sha256': stored_sha(target),
                        'archive_bytes': target.stat().st_size})
        # Each archive is verified and recorded before removing its raw copy.
        manifest.write_text(json.dumps({
            'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'archive_format': 'gzip; deterministic mtime=0; lossless raw SHA256 verified',
            'files': entries}, indent=2) + '\n')
        source.unlink()
    print(json.dumps({'files': len(entries),
                      'raw_bytes': sum(e['raw_bytes'] for e in entries),
                      'archive_bytes': sum(e['archive_bytes'] for e in entries)}))


if __name__ == '__main__':
    main()
