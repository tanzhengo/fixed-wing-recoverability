from pathlib import Path
import os,sys,subprocess,signal,time,socket,json,copy,xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[2];repo=root/'outputs/two_phase_ws/src/Two-phase-motion-planning';out=Path(__file__).resolve().parent
mode=sys.argv[1] if len(sys.argv)>1 else 'static'
assert mode in ('static','moving','empty')
sdf=E.Element('sdf',version='1.6');world=E.SubElement(sdf,'world',name='default')
physics=E.SubElement(world,'physics',name='default_physics',type='ode');E.SubElement(physics,'max_step_size').text='.01';E.SubElement(physics,'real_time_update_rate').text='100'
E.SubElement(world,'gravity').text='0 0 0'
def addbox(name,pose,size):
 m=E.SubElement(world,'model',name=name);E.SubElement(m,'static').text='true';E.SubElement(m,'pose').text=pose;l=E.SubElement(m,'link',name='link');c=E.SubElement(l,'collision',name='collision');g=E.SubElement(c,'geometry');E.SubElement(E.SubElement(g,'box'),'size').text=size
if mode!='empty':
 addbox('ground','0 0 -.1 0 0 0','500 500 .2');addbox('wall','10 0 45 0 0 0','.2 100 100')
m=E.SubElement(world,'model',name='calibration_sensor');E.SubElement(m,'static').text='false' if mode=='moving' else 'true';E.SubElement(m,'pose').text='0 0 5 .1 .2 .3';link=E.SubElement(m,'link',name='base_link')
if mode=='moving':
 inertial=E.SubElement(link,'inertial');E.SubElement(inertial,'mass').text='1';inertia=E.SubElement(inertial,'inertia')
 for k in ('ixx','iyy','izz'):E.SubElement(inertia,k).text='.1'
sensor=E.parse(repo/'simple-airplane/simple_airplane_gazebo/worlds/end_city_lidar.world').getroot().find("world/model[@name='suav1']/link[@name='base_link']/sensor[@name='lidar_3d']")
link.append(copy.deepcopy(sensor));E.ElementTree(sdf).write(out/'calibration.world',encoding='utf-8',xml_declaration=True)
for port in (11468,11469):
 with socket.socket() as s:assert s.connect_ex(('127.0.0.1',port))!=0,'port occupied'
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11468',GAZEBO_MASTER_URI='http://127.0.0.1:11469',ROS_LOG_DIR=str(out/'roslog'),GAZEBO_MODEL_DATABASE_URI='')
log=(out/('launch_'+mode+'.log')).open('w');p=subprocess.Popen(['roslaunch','gazebo_ros','empty_world.launch','world_name:='+str(out/'calibration.world'),'gui:=false','paused:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
native=subprocess.Popen([str(out/'native_scan'),str(out/('native_'+mode+'.jsonl'))],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True) if mode!='empty' else None
try:
 import rospy
 from sensor_msgs.msg import PointCloud2
 from sensor_msgs import point_cloud2
 rospy.init_node('calibrate_lidar_exact_rays',anonymous=True,disable_signals=True)
 messages=[]
 def receive(m):messages.append(m)
 sub=rospy.Subscriber('/suav1/lidar/points_raw',PointCloud2,receive,queue_size=20)
 if mode=='moving':
  from gazebo_msgs.srv import SetModelState
  from gazebo_msgs.msg import ModelState
  rospy.wait_for_service('/gazebo/set_model_state',timeout=20)
  state=ModelState();state.model_name='calibration_sensor';state.pose.position.z=5
  q=Rotation.from_euler('xyz',[.1,.2,.3]).as_quat();state.pose.orientation.x=q[0];state.pose.orientation.y=q[1];state.pose.orientation.z=q[2];state.pose.orientation.w=q[3];state.twist.linear.x=2
  assert rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)(state).success
 deadline=time.monotonic()+40
 while len(messages)<18 and time.monotonic()<deadline:
  if p.poll() is not None:raise RuntimeError('Gazebo exited')
  time.sleep(.1)
 assert len(messages)>=18,'No lidar messages'
 time.sleep(.3)
 selected=messages[-8:];m=selected[-1]
 xyz=np.array(list(point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=False))).reshape((-1,3))
 stamps=np.array([x.header.stamp.to_sec() for x in selected])
 result={'mode':mode,'cloud_count':len(selected),'point_count':len(xyz),'total_rays':512*32,'frame':m.header.frame_id,'fields':[f.name for f in m.fields],'stamp_intervals_s':np.diff(stamps).tolist(),'mean_hz':float(1/np.diff(stamps).mean()),'all_finite':bool(np.isfinite(xyz).all())}
 if mode=='empty':
  result['passed']=bool(result['all_finite'] and result['frame']=='suav1/lidar' and all(x.width*x.height==0 and len(x.data)==0 for x in selected))
 else:
  rows=[json.loads(line) for line in (out/('native_'+mode+'.jsonl')).read_text().splitlines() if line.strip()];poses={round(row['t'],6):row for row in rows};errors=[];wrong=[];matched=0
  for cloud in selected:
   stamp=round(cloud.header.stamp.to_sec(),6);native_pose=poses[stamp];points=np.array(list(point_cloud2.read_points(cloud,field_names=('x','y','z'),skip_nans=False)))
   rotation=Rotation.from_quat(native_pose['quaternion']).as_matrix();world_points=points@rotation.T+native_pose['position'];err=np.minimum(abs(world_points[:,2]),abs(world_points[:,0]-9.9));errors.extend(err.tolist());matched+=1
   old=poses.get(round(stamp-.2,6))
   if old:
    bad=points@Rotation.from_quat(old['quaternion']).as_matrix().T+old['position'];wrong.extend(np.minimum(abs(bad[:,2]),abs(bad[:,0]-9.9)).tolist())
  result.update(native_stamp_matches=matched,max_range_m=float(np.linalg.norm(xyz,axis=1).max()),ground_hits=int((abs(world_points[:,2])<.03).sum()),wall_hits=int((abs(world_points[:,0]-9.9)<.03).sum()),max_surface_error_m=float(max(errors)),p99_surface_error_m=float(np.percentile(errors,99)),previous_stamp_p99_error_m=float(np.percentile(wrong,99)) if wrong else None)
  result['passed']=bool(result['all_finite'] and result['frame']=='suav1/lidar' and len(xyz)<512*32 and result['max_range_m']<120 and result['p99_surface_error_m']<.03 and result['ground_hits']>100 and result['wall_hits']>100)
  np.savez_compressed(out/('calibration_points_'+mode+'.npz'),sensor_points=xyz,world_points=world_points)
 (out/('calibration_result_'+mode+'.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
 assert result['passed']
finally:
 if native is not None and native.poll() is None:
  os.killpg(native.pid,signal.SIGINT)
  native.wait(timeout=5)
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=12)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 log.close()
