#!/usr/bin/env python3
from pathlib import Path
import argparse,sys
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description='Compile the recovered pair and urban instrumented tools.');p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
sys.path.insert(0,str(HERE.parent));from portable_support import build
build([('pair',HERE/'source/pair/pair.cpp'),('urban',HERE/'source/urban/replay.cpp')],a.output_dir)
