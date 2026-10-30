#!/usr/bin/env python3
"""Observe an isolated Gazebo episode without commanding existing simulators.

Source ROS and the built catkin workspace before running. Example:
  python3 gazebo_regression.py --tag course --workspace /path/to/catkin_ws \
      --output-dir /tmp/recovery-check
  python3 gazebo_regression.py --tag climb --case multiturn --goal-height 110 \
      --workspace /path/to/catkin_ws --output-dir /tmp/recovery-check \
      --scenario-dir /path/to/scenarios/multiturn_110

--port and --port+1 must both be unused. The observer shuts down only its own
roslaunch process group. The output includes raw observations and source hashes.
"""
import argparse,os,subprocess,time,signal,json,math,hashlib,threading,socket,xml.etree.ElementTree as ET
from pathlib import Path
from collections import Counter
P=argparse.ArgumentParser();P.add_argument('--tag',required=True);P.add_argument('--sim-seconds',type=float,default=180);P.add_argument('--wall-seconds',type=float,default=120);P.add_argument('--port',type=int,default=11418);P.add_argument('--launch',default='terminal_recovery_learned.launch');P.add_argument('--case',choices=['course','multiturn'],default='course');P.add_argument('--workspace',type=Path,required=True);P.add_argument('--output-dir',type=Path,required=True);P.add_argument('--scenario-dir',type=Path);P.add_argument('--goal-x',type=float,default=50);P.add_argument('--goal-y',type=float,default=0);P.add_argument('--goal-height',type=float,default=70);args=P.parse_args()
if not 1024 <= args.port <= 65534:
 P.error('--port must be between 1024 and 65534')
if Path(args.tag).name != args.tag or args.tag in ('.','..'):
 P.error('--tag must be one directory name')
for port in (args.port,args.port+1):
 with socket.socket() as connection:
  connection.settimeout(.3)
  if connection.connect_ex(('127.0.0.1',port)) == 0:
   P.error('Port %d is already in use; choose another --port' % port)
out=args.output_dir.expanduser().resolve()/args.tag
if (out/'result.json').exists():
 P.error('This tag already has results; choose a new --tag')
out.mkdir(parents=True,exist_ok=True)
workspace=args.workspace.expanduser().resolve()
import rospkg
repo=Path(rospkg.RosPack().get_path('motion_primitives_planning')).parent
assert (workspace/'devel/setup.bash').is_file(), 'Source and select a built catkin workspace'
scenario=args.scenario_dir.expanduser().resolve() if args.scenario_dir else None
assert args.case=='course' or scenario is not None, 'multiturn requires --scenario-dir'
os.environ['ROS_MASTER_URI']='http://127.0.0.1:'+str(args.port)
os.environ['GAZEBO_MASTER_URI']='http://127.0.0.1:'+str(args.port+1)
os.environ['ROS_LOG_DIR']=str(out/'roslog');os.environ['GAZEBO_MODEL_DATABASE_URI']=''
world=(repo/'simple-airplane/simple_airplane_gazebo/worlds/vertical_avoidance.world') if args.case=='course' else scenario/'multiturn.world'
obstacles=[]
for model in ET.parse(world).getroot().find('world').findall('model'):
 if not model.attrib.get('name','').startswith('static_obs'):continue
 pose=[float(v) for v in model.findtext('pose').split()];cyl=model.find('link/collision/geometry/cylinder')
 obstacles.append({'name':model.attrib['name'],'x':pose[0],'y':pose[1],'z':pose[2],'r':float(cyl.findtext('radius')),'h':float(cyl.findtext('length'))})
assert len(obstacles)==(38 if args.case=='course' else 0)
records={'odom':[],'command':[],'goal':[],'status':[]};lock=threading.Lock();result={};transitions=[]
command=(['roslaunch','motion_primitives_planning',args.launch] if args.case=='course' else ['roslaunch',str(scenario/'multiturn.launch')])+['gui:=false','rviz:=false','paused:=false']
binary=workspace/'devel/lib/motion_primitives_planning/motion_primitives_test'
binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest()
source_paths=['motion_primitives_planning/src/runtime_motion_primitives.h',
 'motion_primitives_planning/src/motion_primitives_test.cpp',
 'motion_primitives_planning/scripts/generate_urban_planner.py',
 'motion_primitives_planning/models/recovery_mlp.txt']
source_sha256={name:hashlib.sha256((repo/name).read_bytes()).hexdigest()
 for name in source_paths if (repo/name).is_file()}
log=(out/'launch.log').open('w');start=time.monotonic();p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def cyl_distance(s,o,inflated):
 radial=math.hypot(s['x']-o['x'],s['y']-o['y'])-o['r']*(1.2 if inflated else 1.)
 axial=abs(s['z']-o['z'])-o['h']/2-(1. if inflated else 0.)
 return math.hypot(max(radial,0.),max(axial,0.))+min(max(radial,axial),0.)
def rpy(q):
 x,y,z,w=q
 return math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(max(-1,min(1,2*(w*y-z*x)))),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
try:
 import xmlrpc.client
 deadline=start+20
 while time.monotonic()<deadline:
  if p.poll() is not None:raise RuntimeError('launch exited early')
  try:
   if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/isolated_gazebo_check')[0]==1:break
  except OSError:pass
  time.sleep(.2)
 import rospy
 from nav_msgs.msg import Odometry
 from geometry_msgs.msg import Twist
 from visualization_msgs.msg import Marker
 from std_msgs.msg import String
 rospy.init_node('isolated_gazebo_regression',anonymous=True,disable_signals=True)
 def odom(m):
  q=m.pose.pose.orientation;roll,pitch,yaw=rpy([q.x,q.y,q.z,q.w]);pos=m.pose.pose.position
  row={'t':m.header.stamp.to_sec(),'x':pos.x,'y':pos.y,'z':pos.z,'V':m.twist.twist.linear.x,'gamma':m.twist.twist.linear.y,'sigma':m.twist.twist.linear.z,'roll':-roll,'attack':-pitch-m.twist.twist.linear.y,'T':m.twist.twist.angular.x,'d_sigma':m.twist.twist.angular.y}
  row['physical_center_clearance']=min((cyl_distance(row,o,False) for o in obstacles),default=None)
  row['inflated_clearance']=min((cyl_distance(row,o,True) for o in obstacles),default=None)
  with lock:records['odom'].append(row)
 def cmd(m):
  with lock:records['command'].append({'t':rospy.Time.now().to_sec(),'roll':m.linear.x,'attack':m.linear.y,'T':m.linear.z})
 def goal(m):
  pos=m.pose.position;row={'t':m.header.stamp.to_sec(),'x':pos.x,'y':pos.y,'z':pos.z}
  with lock:
   previous=records['goal'][-1] if records['goal'] else None
   records['goal'].append(row)
   if previous is None or (row['x'],row['y'],row['z'])!=(previous['x'],previous['y'],previous['z']):
    transition=dict(row)
    if records['odom']:transition['aircraft']=dict(records['odom'][-1])
    if previous and records['odom']:
     s=records['odom'][-1];transition['previous_goal']=[previous[k] for k in ('x','y','z')];transition['previous_goal_horizontal_error']=math.hypot(s['x']-previous['x'],s['y']-previous['y']);transition['previous_goal_height_error']=abs(s['z']-previous['z'])
    transitions.append(transition)
 def status(m):
  row=json.loads(m.data);row['t']=rospy.Time.now().to_sec()
  with lock:records['status'].append(row)
 subs=[rospy.Subscriber('/suav1/odom',Odometry,odom,queue_size=100),rospy.Subscriber('/suav1/cmd_vel',Twist,cmd,queue_size=100),rospy.Subscriber('/suav1/goal',Marker,goal,queue_size=100),rospy.Subscriber('/suav1/planning_status',String,status,queue_size=100)]
 deadline=time.monotonic()+args.wall_seconds;last_progress=0;stop='wall_timeout'
 while time.monotonic()<deadline:
  if p.poll() is not None:stop='launch_exited';break
  with lock:
   t=records['odom'][-1]['t'] if records['odom'] else 0
   seq=[(r['x'],r['y'],r['z']) for r in transitions]
  if args.case=='course' and len(seq)>=4 and seq[:4]==[(170.,0.,8.),(350.,0.,24.),(520.,0.,8.),(170.,0.,8.)]:stop='completed_first_waypoint_cycle';break
  if args.case=='multiturn' and records['odom']:
   last=records['odom'][-1]
   if math.hypot(last['x']-args.goal_x,last['y']-args.goal_y)<=48.825 and abs(last['z']-args.goal_height)<=3:stop='multiturn_goal_attained';break
  if t>=args.sim_seconds:stop='simulation_limit';break
  if time.monotonic()-last_progress>20:
   print(json.dumps({'tag':args.tag,'sim_time':t,'goal_sequence':seq}),flush=True);last_progress=time.monotonic()
  time.sleep(.2)
 with lock:
  od=records['odom'];st=records['status']
  climbs=[]
  for first,last in zip(od,od[1:]):
   dt=last['t']-first['t']
   if dt>0 and last['z']>first['z'] and abs(first['roll'])>math.radians(10):climbs.append((last['z']-first['z'],dt))
  banked_window_gain=0.;banked_window=None
  for i,first in enumerate(od):
   j=i
   while j+1<len(od) and od[j+1]['t']-first['t']<=5.:j+=1
   segment=od[i:j+1]
   if segment and segment[-1]['t']-first['t']>=4.5 and sum(abs(r['roll'])>math.radians(10) for r in segment)>=len(segment)*.5:
    gain=segment[-1]['z']-first['z']
    if gain>banked_window_gain:banked_window_gain=gain;banked_window=[first['t'],segment[-1]['t'],first['z'],segment[-1]['z']]
  result={'max_banked_climb_gain_over_5s_m':banked_window_gain,'best_banked_climb_window':banked_window,'tag':args.tag,'stop_reason':stop,'wall_seconds':time.monotonic()-start,'last_sim_time':od[-1]['t'] if od else None,'odom_count':len(od),'command_count':len(records['command']),'status_count':len(st),'goal_transitions':transitions,'status_counts':dict(Counter(r['status'] for r in st)),'last_odom':od[-1] if od else None,'minimum_physical_center_clearance_m':min((r['physical_center_clearance'] for r in od if r['physical_center_clearance'] is not None),default=None),'minimum_inflated_clearance_m':min((r['inflated_clearance'] for r in od if r['inflated_clearance'] is not None),default=None),'altitude_min_max':[min((r['z'] for r in od),default=None),max((r['z'] for r in od),default=None)],'banked_positive_climb_m':sum(h for h,dt in climbs),'banked_positive_climb_duration_s':sum(dt for h,dt in climbs),'planner_parameters':rospy.get_param('/suav1/motion_primitive',{}),'recovery_parameters':rospy.get_param('/suav1/recovery',{}),'command':command,'world_sha256':hashlib.sha256(world.read_bytes()).hexdigest(),'planner_binary_sha256':binary_sha256,'planner_source_sha256':source_sha256,'observer_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'point_clearance_note':'Clearance is sampled for aircraft reference-point position against analytic physical/inflated static cylinders; not an aircraft mesh-contact certificate.'}
  turns=[math.atan2(math.sin(b['sigma']-a['sigma']),math.cos(b['sigma']-a['sigma'])) for a,b in zip(od,od[1:])]
  result.update(case=args.case,obstacle_count=len(obstacles),absolute_heading_turns=sum(abs(d) for d in turns)/(2*math.pi),signed_heading_turns=sum(turns)/(2*math.pi),speed_min_max=[min(r['V'] for r in od),max(r['V'] for r in od)],max_abs_gamma_deg=max(abs(r['gamma']) for r in od)*180/math.pi,max_abs_roll_deg=max(abs(r['roll']) for r in od)*180/math.pi)
  result['observer_cli']={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}
  result['scenario_source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in scenario.glob('multiturn.*')} if scenario else {}
  (out/'observations.json').write_text(json.dumps(records)+'\n')
except Exception as exc:
 result={'tag':args.tag,'error':repr(exc)}
 raise
finally:
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 log.close();result['network_fallback_warning']='using diverse scheduling' in (out/'launch.log').read_text(errors='replace')
 result['waypoint_gate_checks_passed']=all(r.get('previous_goal_horizontal_error',0)<=49.325 and r.get('previous_goal_height_error',0)<=3.2 for r in result.get('goal_transitions',[]))
 result['passed']=(result.get('stop_reason')=='completed_first_waypoint_cycle' if args.case=='course' else result.get('stop_reason')=='multiturn_goal_attained' and abs(result.get('signed_heading_turns',0))>=2) and (result.get('minimum_inflated_clearance_m',-1)>0 if obstacles else True) and result['waypoint_gate_checks_passed'] and result.get('banked_positive_climb_m',0)>3 and not result['network_fallback_warning']
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
