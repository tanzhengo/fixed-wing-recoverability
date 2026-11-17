#!/usr/bin/env python3
from pathlib import Path
import argparse,sys,subprocess,json,time,shutil
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description='Replay all six frozen validation maps without changing original records.')
p.add_argument('--build-dir',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--port',type=int,default=11989);a=p.parse_args()
sys.path.insert(0,str(HERE.parent));from portable_support import fresh,copy_inputs,master,sha
OUT=fresh(a.output_dir);copy_inputs(HERE/'raw',OUT/'raw')
for name in ['frozen_pairs_final.json','selection_protocol.json','selection_amendment.json']:shutil.copyfile(HERE/name,OUT/name)
report={'frozen_pairs_sha256':sha(HERE/'frozen_pairs_final.json'),'command':[str(a.build_dir.resolve()/'pair_replay'),str(OUT/'raw')],'scope':'sensor-map replay only; no production control changes'}
try:
    with master(OUT,a.port) as env:
        begin=time.monotonic()
        with (OUT/'raw/stdout.log').open('w') as stdout,(OUT/'raw/trace.log').open('w') as stderr:result=subprocess.run(report['command'],env=env,stdout=stdout,stderr=stderr)
        report.update(returncode=result.returncode,wall_seconds=time.monotonic()-begin)
        if result.returncode:raise RuntimeError('Replay failed; inspect fresh logs')
finally:(OUT/'execution.json').write_text(json.dumps(report,indent=2)+'\n')
print(OUT)
