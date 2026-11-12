#!/usr/bin/env python3
"""Run reconstruction verification and save actual results and source hashes."""
import argparse,io,json,platform,sys,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from fwcontinuation.records import source_hashes,write_json

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True)
    args=parser.parse_args();root=Path(__file__).resolve().parents[1]
    suite=unittest.defaultTestLoader.discover(str(root/'tests'))
    stream=io.StringIO();start=time.perf_counter()
    result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    duration=time.perf_counter()-start
    payload={'evidence_kind':'reconstruction_software_checks','python':platform.python_version(),
             'platform':platform.platform(),'tests_run':result.testsRun,
             'failures':len(result.failures),'errors':len(result.errors),'skipped':len(result.skipped),
             'elapsed_s':duration,'source_sha256':source_hashes(),
             'historical_experiments_reproduced':False,'ros_gazebo_executed':False,
             'log':stream.getvalue()}
    write_json(args.output,payload)
    print(stream.getvalue());sys.exit(0 if result.wasSuccessful() else 1)

if __name__=='__main__':main()
