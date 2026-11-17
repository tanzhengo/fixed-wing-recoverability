#!/usr/bin/env python3
from pathlib import Path
import argparse,sys
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description='Compile the six-map frozen Figure 2 replay.');p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
sys.path.insert(0,str(HERE.parent));from portable_support import build
build([('pair_replay',HERE/'source/validation_pair/pair.cpp')],a.output_dir)
