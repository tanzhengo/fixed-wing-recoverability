#!/usr/bin/env python3
"""Portable fresh-output runner for original frozen closed-loop evaluators."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client

PACKAGE = Path(__file__).resolve().parent
MODELS = {'legacy': 'legacy_state_aux_v1.txt', 'state': 'aligned_state_aux.txt',
          'bce': 'selected_obs_bce.txt', 'aux': 'obs_aux.txt'}
METHODS = {
    'Cost': ('cost', 'legacy', .5), 'Diverse': ('diverse', 'legacy', .5),
    'Legacy': ('learned', 'legacy', .5), 'AlignedState': ('learned', 'state', .5),
    'ObsBCE': ('learned', 'bce', .5), 'ObsAux': ('learned', 'aux', .5),
    'Selected': ('learned_coverage', 'bce', .75),
    'ObservedCoverage': ('observed_coverage', 'legacy', .25), 'Full': ('full', 'legacy', .5)}

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('suite', choices=['procedural', 'forest', 'failure_case'])
    p.add_argument('--build', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True, help='New directory; original evidence is never overwritten')
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--methods', nargs='+', choices=sorted(METHODS))
    p.add_argument('--master', default=os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
    p.add_argument('--plan-only', action='store_true')
    a = p.parse_args()
    if a.out.exists() and any(a.out.iterdir()):
        raise RuntimeError('Choose a new empty --out directory.')
    if a.workers < 1:
        raise ValueError('--workers must be positive')
    a.out.mkdir(parents=True, exist_ok=True)
    is_forest = a.suite == 'forest'
    binary = a.build.resolve() / ('forest' if is_forest else 'procedural')
    if is_forest:
        methods = a.methods or ['Cost', 'Diverse', 'ObsAux', 'Selected', 'Full']
        cases = [(f, i, 0, 2402100) for i in range(8) for f in [100, 200, 300]]
    elif a.suite == 'failure_case':
        methods = a.methods or ['Selected', 'Diverse', 'Full']
        cases = [(2, 0, 0, 1301000)]
    else:
        methods = a.methods or list(METHODS)
        cases = [(f, i, c, 1301000) for i in range(8) for f in range(4) for c in range(4)]
    plan = []
    for f, i, c, base in cases:
        for label in methods:
            order, modelkey, alpha = METHODS[label]
            model = PACKAGE / 'models' / MODELS[modelkey]
            family = 'forest_d{}_L600'.format(f) if is_forest else ['staggered', 'wall', 'culdesac', 'spatial'][f]
            key = '{}_{}_m{}_c{}'.format(family, i, label, c)
            command = [str(binary), str(a.out.resolve()), str(model), str(f), str(i), order,
                       str(c), str(base), str(alpha), '3' if is_forest else '1', label]
            if is_forest:
                command.append('600')
            plan.append({'case': key, 'command': command, 'model_sha256': sha(model)})
    protocol = {'suite': a.suite, 'jobs': plan, 'binary_sha256': sha(binary),
        'master_uri': a.master, 'source': 'recovered original C++ evaluator',
        'results_scope': 'fresh rerun; compare decisions and work to originals; CPU timing is machine dependent'}
    (a.out / 'run_protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    if a.plan_only:
        print('Prepared', len(plan), 'jobs in', a.out)
        return
    if xmlrpc.client.ServerProxy(a.master).getPid('/recovered_evidence_runner')[0] != 1:
        raise RuntimeError('Start a ROS master before executing the evaluators.')
    env = dict(os.environ, ROS_MASTER_URI=a.master, ROS_LOG_DIR=str(a.out.resolve() / 'roslogs'))
    def run(job):
        with (a.out / (job['case'] + '.log')).open('w') as f:
            subprocess.run(job['command'], env=env, stdout=f, stderr=subprocess.STDOUT, check=True)
        with (a.out / (job['case'] + '_summary.csv')).open() as f:
            row = next(csv.DictReader(f))
        assert row['case'] == job['case']
        return row
    start = time.time()
    rows = []
    with ThreadPoolExecutor(max_workers=a.workers) as pool:
        for future in as_completed([pool.submit(run, job) for job in plan]):
            row = future.result()
            rows.append(row)
            print('{}/{} {} {}'.format(len(rows), len(plan), row['case'], row['outcome']), flush=True)
    rows.sort(key=lambda r: r['case'])
    with (a.out / 'episodes.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (a.out / 'completion.json').write_text(json.dumps({'episodes': len(rows), 'wall_seconds': time.time() - start,
        'episodes_csv_sha256': sha(a.out / 'episodes.csv')}, indent=2) + '\n')

if __name__ == '__main__':
    main()
