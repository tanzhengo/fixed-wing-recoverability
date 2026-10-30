#!/usr/bin/env python3
"""Check compact-export structure and sequential-float32 parity fixtures."""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def read_network(path):
    tokens = path.read_text().split()
    magic, inputs, first, second, outputs = tokens[:5]
    dims = list(map(int, (inputs, first, second, outputs)))
    values = np.asarray(tokens[5:], dtype=np.float32)
    assert np.isfinite(values).all()
    arrays, offset = [], 0
    for i, j in zip(dims[:-1], dims[1:]):
        arrays.append(values[offset:offset+i*j].reshape(j, i))
        offset += i*j
        arrays.append(values[offset:offset+j])
        offset += j
    assert offset == len(values)
    return magic, dims, arrays


def forward(features, arrays):
    x = features.astype(np.float32)
    for layer in range(3):
        w, b = arrays[2*layer:2*layer+2]
        y = np.broadcast_to(b, (len(x),len(b))).copy()
        for j in range(x.shape[1]):
            y += x[:,j,None]*w[None,:,j]
        x = np.maximum(y, np.float32(0)) if layer < 2 else y
    return x


def main():
    root = Path(__file__).resolve().parent
    rows = []
    for report_path in sorted(root.glob('*/*/training.json')):
        directory = report_path.parent
        report = json.loads(report_path.read_text())
        p = directory/'network_compact.txt'
        assert hashlib.sha256(p.read_bytes()).hexdigest() == report['model_compact_sha256']
        magic,dims,compact = read_network(p)
        _,original_dims,original = read_network(directory/'network_v1.txt')
        assert magic == 'RECOVERY_MLP_V2' and dims[1:] == [64,32,1]
        assert original_dims == [103,64,32,2]
        assert np.array_equal(compact[0],original[0][:,:dims[0]])
        for index in [1,2,3]:
            assert np.array_equal(compact[index],original[index])
        assert np.array_equal(compact[4],original[4][:1])
        assert np.array_equal(compact[5],original[5][:1])
        if dims[0] == 28:
            assert np.count_nonzero(original[0][:,28:]) == 0
        with (directory/'compact_parity.csv').open() as stream:
            reader = csv.reader(stream)
            header = next(reader)
            values = np.asarray(list(reader),dtype=np.float32)
        assert len(header)==dims[0]+1
        errors=np.abs(forward(values[:,:dims[0]],compact)[:,0]-values[:,-1])
        assert np.max(errors)<1e-4
        rows.append(dict(cell=report['cell'],seed=report['seed'],
                         dimensions=dims,fixture_rows=len(values),
                         maximum_absolute_error=float(np.max(errors)),
                         mean_absolute_error=float(np.mean(errors))))
    assert len(rows)==12
    result=dict(checked_models=len(rows),structural_compaction_exact=True,
                arithmetic='sequential float32 accumulation versus PyTorch fixture',
                tolerance=1e-4,maximum_absolute_error=max(r['maximum_absolute_error'] for r in rows),
                models=rows)
    (root/'export_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
