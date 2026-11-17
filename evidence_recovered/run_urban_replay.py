#!/usr/bin/env python3
"""Run frozen replay against user-supplied original observation folders."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import xmlrpc.client
PACKAGE = Path(__file__).resolve().parent
RUNS = ['learned3', 'cost1', 'diverse1']

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs-root', type=Path, required=True, help='Parent of learned3, cost1 and diverse1 observation folders')
    p.add_argument('--build', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--mode', choices=['current', 'instrumented', 'frozen'], default='current')
    p.add_argument('--budget-grid', action='store_true')
    p.add_argument('--master', default=os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
    p.add_argument('--cpu', type=int, help='Optional CPU affinity; original quiet pass used logical CPU 10')
    p.add_argument('--plan-only', action='store_true')
    a = p.parse_args()
    if a.out.exists() and any(a.out.iterdir()):
        raise RuntimeError('Use a fresh output directory.')
    if a.budget_grid and a.mode != 'current':
        raise ValueError('Budget-grid source is the current non-instrumented variant.')
    a.out.mkdir(parents=True, exist_ok=True)
    records = json.loads((PACKAGE / 'EXTERNAL_ARTIFACTS.json').read_text())['files']
    hashes = {r['path'].split('/unknown_replay/', 1)[1]: r['sha256'] for r in records if '/unknown_replay/' in r['path']}
    for run in RUNS:
        for name in ['poses.csv', 'queries.csv', 'accepted_scans.csv', 'params.yaml', 'rays.bin']:
            key = run + '/' + name
            assert digest(a.inputs_root / key) == hashes[key], key
    # The original TSV parser splits whitespace. Relative model names avoid any
    # dependency on spaces in the chosen repository path; replay cwd is PACKAGE.
    spec = [('AlignedState', 'learned', 'models/aligned_state_aux.txt', 0.0),
            ('ObsBCE', 'learned', 'models/selected_obs_bce.txt', 0.0),
            ('ObsAux', 'learned', 'models/obs_aux.txt', 0.0),
            ('Selected', 'learned_coverage', 'models/selected_obs_bce.txt', .75),
            ('ObservedCoverage', 'observed_coverage', '-', .25)]
    lines = ['\t'.join(str(v) for v in row) for row in spec]
    if a.budget_grid:
        for method, order, model, alpha in [('Cost','cost','-',0), ('Diverse','diverse','-',0),
                ('Selected','learned_coverage','models/selected_obs_bce.txt',.75),
                ('ObservedCoverage','observed_coverage','-',.25)]:
            for k in [1,2,4,8,12,24]:
                lines.append('\t'.join(str(v) for v in [method+'_K'+str(k),order,model,alpha,k]))
    specfile = a.out.resolve() / 'selected_methods.tsv'
    specfile.write_text('\n'.join(lines) + '\n')
    env = dict(os.environ, ROS_MASTER_URI=a.master, REPLAY_METHODS_TSV=str(specfile),
               ROS_LOG_DIR=str(a.out.resolve() / 'roslogs'))
    env.pop('REPLAY_REPEATS', None)
    if a.budget_grid:
        env['REPLAY_REPEATS'] = '1'
    jobs = []
    for run in RUNS:
        ns = '/recovered_replay/' + run
        inputdir = (a.inputs_root / run).resolve()
        outputdir = (a.out / run).resolve()
        outputdir.mkdir()
        suffix = 'budget_grid' if a.budget_grid else 'quiet' if a.mode == 'current' else a.mode
        command = [str(a.build.resolve() / ('urban_' + a.mode)), str(inputdir), str(outputdir), ns,
            str(PACKAGE / 'models/legacy_state_aux_compact.txt'), suffix]
        if a.cpu is not None:
            command = ['taskset', '-c', str(a.cpu)] + command
        jobs.append({'run':run,'namespace':ns,'command':command,
            'load':['rosparam','load',str(inputdir / 'params.yaml'),ns],
            'model_override':['rosparam','set',ns+'/recovery/model_path',str(PACKAGE / 'models/legacy_state_aux_v1.txt')]})
    (a.out / 'run_protocol.json').write_text(json.dumps({'mode':a.mode,'budget_grid':a.budget_grid,
        'jobs':jobs,'model_tsv_sha256':digest(specfile),'cpu':a.cpu,
        'input_hashes_verified':True,'binary_sha256':digest(a.build / ('urban_'+a.mode))},indent=2)+'\n')
    if a.plan_only:
        print('Prepared',len(jobs),'hash-verified replay jobs in',a.out)
        return
    if xmlrpc.client.ServerProxy(a.master).getPid('/recovered_replay_runner')[0] != 1:
        raise RuntimeError('Start a ROS master before replay.')
    for job in jobs:
        subprocess.run(job['load'],env=env,check=True)
        subprocess.run(job['model_override'],env=env,check=True)
        try:
            with (a.out / job['run'] / 'replay.log').open('w') as log:
                subprocess.run(job['command'],env=env,cwd=PACKAGE,stdout=log,stderr=subprocess.STDOUT,check=True)
        finally:
            subprocess.run(['rosparam','delete',job['namespace']],env=env,check=True)
        print('Completed',job['run'],flush=True)

if __name__ == '__main__':
    main()
