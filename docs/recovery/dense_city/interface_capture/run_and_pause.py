import rospy,json,time
from pathlib import Path
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import String
rospy.init_node("paper_capture_run",anonymous=True)
goals=[];last={};status={}
def goal(m):
 v=[m.pose.position.x,m.pose.position.y,m.pose.position.z]
 if not goals or goals[-1]["target"]!=v:goals.append({"t":m.header.stamp.to_sec(),"target":v})
def odom(m):
 last.update(t=m.header.stamp.to_sec(),position=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z])
def mapper(m):
 status.clear();status.update(json.loads(m.data))
rospy.Subscriber("/suav1/goal",Marker,goal,queue_size=100)
rospy.Subscriber("/suav1/odom",Odometry,odom,queue_size=10)
rospy.Subscriber("/suav1/voxel_map/status",String,mapper,queue_size=2)
rospy.wait_for_service("/gazebo/unpause_physics",timeout=20)
rospy.ServiceProxy("/gazebo/unpause_physics",Empty)()
start=time.monotonic()
while time.monotonic()-start<240 and not rospy.is_shutdown():
 if len(goals)>=9 and goals[-1]["target"]==goals[0]["target"]:break
 time.sleep(.05)
rospy.ServiceProxy("/gazebo/pause_physics",Empty)()
time.sleep(2)
result={"goals":goals,"last_odometry":last,"map_status":status,"sim_time":rospy.Time.now().to_sec(),"purpose":"Separate run for native Gazebo/RViz interface screenshots; archived flight1 quantitative results retained."}
Path("work/interface_screenshots/capture_run.json").write_text(json.dumps(result,indent=2)+"\n")
print(json.dumps(result,indent=2))
