from pathlib import Path
import os,subprocess,socket,time,signal,json,sys
port=11488
with socket.socket() as s: assert s.connect_ex(('127.0.0.1',port))!=0
root=Path(__file__).resolve().parents[2];repo=root/'outputs/two_phase_ws/src/Two-phase-motion-planning';out=root/'work/lidar_mapper'
env=os.environ.copy();env['ROS_MASTER_URI']='http://127.0.0.1:'+str(port);env['ROS_LOG_DIR']=str(out/'roslog');os.environ.update(env)
log=(out/'roscore.log').open('w');node_log=(out/'mapper.log').open('w')
master=subprocess.Popen(['roscore','-p',str(port)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);node=None
try:
 import rospy
 from geometry_msgs.msg import Point
 from nav_msgs.msg import Odometry
 from sensor_msgs.msg import PointCloud2,PointField
 from sensor_msgs import point_cloud2
 from std_msgs.msg import Header,String
 from visualization_msgs.msg import Marker
 import xmlrpc.client
 for _ in range(100):
  try:
   if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/test')[0]==1:break
  except OSError:pass
  time.sleep(.05)
 rospy.init_node('mapper_smoke',anonymous=True,disable_signals=True)
 rospy.set_param('/lidar_voxel_map/max_pose_gap',2.)
 rospy.set_param('/lidar_voxel_map/sensor_offset',[0.,0.,.15])
 node=subprocess.Popen(['python3',str(repo/'motion_primitives_planning/scripts/lidar_voxel_map.py')],env=env,stdout=node_log,stderr=subprocess.STDOUT,start_new_session=True)
 got={'points':[],'map':[],'stats':[]}
 subs=[rospy.Subscriber('/suav1/lidar/points',PointCloud2,lambda m:got['points'].append(m)),rospy.Subscriber('/suav1/voxel_map',Marker,lambda m:got['map'].append(m)),rospy.Subscriber('/suav1/voxel_map/status',String,lambda m:got['stats'].append(json.loads(m.data)))]
 odom=rospy.Publisher('/suav1/odom',Odometry,queue_size=10);cloud=rospy.Publisher('/suav1/lidar/points_raw',PointCloud2,queue_size=10)
 for _ in range(100):
  if odom.get_num_connections() and cloud.get_num_connections():break
  time.sleep(.05)
 def pose(t,x):
  m=Odometry();m.header.stamp=rospy.Time.from_sec(t);m.header.frame_id='world';m.pose.pose.position.x=x;m.pose.pose.orientation.w=1.;return m
 odom.publish(pose(1.,0.));time.sleep(.1)
 fields=[PointField('x',0,PointField.FLOAT32,1),PointField('y',4,PointField.FLOAT32,1),PointField('z',8,PointField.FLOAT32,1),PointField('intensity',12,PointField.FLOAT32,1)]
 msg=point_cloud2.create_cloud(Header(stamp=rospy.Time.from_sec(1.5),frame_id='suav1/lidar'),fields,[(1.,0.,0.,2.),(float('nan'),0.,0.,0.)])
 cloud.publish(msg);time.sleep(.15)
 assert not got['points'],'Map must await bracketing odometry'
 odom.publish(pose(2.,10.))
 deadline=time.monotonic()+4
 while time.monotonic()<deadline and not (got['points'] and any(m.points for m in got['map'])):time.sleep(.05)
 assert node.poll() is None,(out/'mapper.log').read_text()
 assert got['points'] and any(m.points for m in got['map']),got
 m=got['points'][-1];pts=list(point_cloud2.read_points(m,field_names=('x','y','z')))
 assert len(pts)==1 and abs(pts[0][0]-6.)<1e-6 and abs(pts[0][2]-.15)<1e-6,pts
 assert m.header.frame_id=='world' and abs(m.header.stamp.to_sec()-1.5)<1e-9
 occupied=next(m for m in reversed(got['map']) if m.points)
 assert occupied.type==Marker.CUBE_LIST and len(occupied.points)==1
 assert abs(occupied.points[0].x-6.5)<1e-9 and occupied.pose.orientation.w==1.
 odom.publish(pose(0.,0.));time.sleep(.8)
 assert len(got['map'][-1].points)==0,'Reset should clear latched CUBE_LIST'
 result={'passed':True,'output_type':'sensor_msgs/PointCloud2','map_type':'visualization_msgs/Marker CUBE_LIST','point_world_xyz':pts[0],'scan_stamp_preserved':m.header.stamp.to_sec(),'reset_cleared_map':True,'last_stats':got['stats'][-1]}
 (out/'smoke_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
finally:
 for p in (node,master):
  if p and p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 log.close();node_log.close()
