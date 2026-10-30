#!/usr/bin/env python3
"""Read raw or losslessly archived revision CSVs with stable logical hashes."""
from pathlib import Path
import csv
import gzip
import hashlib


def csv_source(path):
    path = Path(path)
    if path.is_file():
        return path
    archived = Path(str(path) + '.gz')
    if archived.is_file():
        return archived
    raise FileNotFoundError(path)


def csv_exists(path):
    return Path(path).is_file() or Path(str(path) + '.gz').is_file()


def open_csv(path):
    source = csv_source(path)
    return gzip.open(source, 'rt', newline='') if source.suffix == '.gz' else source.open(newline='')


def csv_rows(path):
    with open_csv(path) as stream:
        return list(csv.DictReader(stream))


def raw_sha256(path):
    source = csv_source(path)
    opener = gzip.open if source.suffix == '.gz' else open
    digest = hashlib.sha256()
    with opener(source, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def logical_csv_paths(directory):
    paths = set(Path(directory).glob('*.csv'))
    paths.update(Path(str(p)[:-3]) for p in Path(directory).glob('*.csv.gz'))
    return sorted(paths)
