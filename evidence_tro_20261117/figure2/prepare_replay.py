#!/usr/bin/env python3
"""Regenerate the frozen six-case instrument from the included original pair tool."""
from pathlib import Path
import argparse,json,shutil,hashlib
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--pair-source',type=Path,default=HERE.parent/'mechanisms/source/pair');a=p.parse_args()
OUT=a.output_dir.resolve()
if OUT.exists() and any(OUT.iterdir()):raise RuntimeError('Use a fresh output directory')
SRC=OUT/'source/validation_pair';SRC.mkdir(parents=True,exist_ok=True)
for f in a.pair_source.glob('*'):
    if f.is_file():shutil.copyfile(f,SRC/f.name)
frozen=json.loads((HERE/'frozen_pairs_final.json').read_text());families=['staggered','wall','culdesac','spatial','forest_sparse','forest_dense'];specs=[]
for group in frozen['groups']:
    for item in group['pair']:
        st=item['state'];index=int(st['scene'].rsplit('_',1)[1]);assert st['sampler']=='0' and float(st['t'])==.8 and st['query']=='4';specs.append((families.index(st['family']),index,item['candidate']))
p=SRC/'pair.cpp';s=p.read_text();s=s.replace('int sceneindex=0;for(int family:{2,0}){Scene sc=scene(family,0,1201000);','struct Spec{int family,index,candidate;};int sceneindex=0;for(const Spec spec:std::vector<Spec>{'+','.join('{%d,%d,%d}'%x for x in specs)+'}){Scene sc=scene(spec.family,spec.index,1201000);');s=s.replace('if(c.id==69||firstpositive)','if(c.id==spec.candidate||firstpositive)').replace('v.x<=135&&std::abs(v.y)<=75','v.x<=145&&std::abs(v.y)<=85');p.write_text(s)
assert p.read_bytes()==(HERE/'source/validation_pair/pair.cpp').read_bytes(),'Generated instrument differs from frozen source'
(OUT/'source_manifest.json').write_text(json.dumps({'matches_frozen_instrument':True,'files':{str(p.relative_to(OUT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in SRC.iterdir() if p.is_file()}},indent=2)+'\n');print('Prepared',specs)
