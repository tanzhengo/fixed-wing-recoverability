#!/usr/bin/env python3
"""Record the updated city route, including real lidar voxels and execution."""
import argparse,hashlib,json,os,signal,socket,subprocess,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--scratch',type=Path,required=True)
    args=p.parse_args();repo=Path(__file__).resolve().parents[2]
    if args.output.exists() and any(args.output.iterdir()):p.error('Output directory is not empty')
    args.output.mkdir(parents=True,exist_ok=True);args.scratch.mkdir(parents=True,exist_ok=True)
    for port in (11478,11479):
        with socket.socket() as s:assert s.connect_ex(('127.0.0.1',port))!=0,'Port in use'
    task=repo/'motion_primitives_planning/launch/config/task_end_city_dense.yaml'
    env=os.environ.copy();env.update(ROS_MASTER_URI='http://127.0.0.1:11478',GAZEBO_MASTER_URI='http://127.0.0.1:11479',GAZEBO_MODEL_DATABASE_URI='',ROS_LOG_DIR=str(args.scratch/'roslog'))
    commands={
        'mapping':['python3',str(repo/'docs/recovery/lidar_voxel_map/verify_lidar.py'),'--output-dir',str(args.output/'mapping'),'--sim-seconds','180','--wall-seconds','500','--stop-after-cycle','--resolution','2.0'],
        'flight':['python3',str(repo/'motion_primitives_planning/scripts/verify_end_world.py'),'--task-config',str(task),'--output-dir',str(args.output/'flight'),'--sim-seconds','180','--wall-seconds','500','--stop-after-cycle','--stop-on-failure'],
        'launch':['roslaunch','motion_primitives_planning','terminal_recovery_end_world_dense_lidar.launch','gui:=false','rviz:=false','paused:=false'],
    }
    sources=[task,repo/'motion_primitives_planning/models/recovery_mlp.txt',repo/'motion_primitives_planning/src/runtime_motion_primitives.h',repo/'simple-airplane/simple_airplane_gazebo/worlds/end_city_dense_lidar.world',Path(__file__).resolve()]
    manifest={'commands':commands,'started_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'source_sha256':{str(f.relative_to(repo)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},'only_requested_waypoint_xy_changed':True,'figure_model':'original Geometry (same as the previous Figure 8)','horizontal_gate_m':48.825,'height_gate_m':3.0,'voxel_resolution_m':2.0,'max_voxels':200000}
    (args.output/'run.json').write_text(json.dumps(manifest,indent=2)+'\n')
    processes={};logs={};start=time.monotonic()
    try:
        for name,cmd in commands.items():
            logs[name]=(args.scratch/(name+'.log')).open('w')
            processes[name]=subprocess.Popen(cmd,env=env,stdout=logs[name],stderr=subprocess.STDOUT,start_new_session=True)
        while processes['flight'].poll() is None and time.monotonic()-start<530:
            if processes['launch'].poll() is not None:break
            time.sleep(1)
        result=args.output/'flight/result.json'
        if result.exists():print(json.dumps(json.loads(result.read_text()),indent=2),flush=True)
        if processes['mapping'].poll() is None:
            try:processes['mapping'].wait(timeout=3)
            except subprocess.TimeoutExpired:pass
    finally:
        for name in ('mapping','flight','launch'):
            proc=processes.get(name)
            if proc and proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=8)
        for stream in logs.values():stream.close()
        manifest['ended_utc']=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())
        manifest['elapsed_wall_s']=time.monotonic()-start
        manifest['source_unchanged']=all(hashlib.sha256((repo/f).read_bytes()).hexdigest()==h for f,h in manifest['source_sha256'].items())
        (args.output/'run.json').write_text(json.dumps(manifest,indent=2)+'\n')

if __name__=='__main__':main()
