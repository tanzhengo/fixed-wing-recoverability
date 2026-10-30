#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <fstream>
#include <iomanip>
#include <atomic>
std::ofstream stream;std::atomic<int> count(0);
void scan(ConstLaserScanStampedPtr& m){auto p=m->scan().world_pose();stream<<std::setprecision(14)<<"{\"t\":"<<m->time().sec()+m->time().nsec()*1e-9<<",\"position\":["<<p.position().x()<<","<<p.position().y()<<","<<p.position().z()<<"],\"quaternion\":["<<p.orientation().x()<<","<<p.orientation().y()<<","<<p.orientation().z()<<","<<p.orientation().w()<<"]}\n";stream.flush();++count;}
int main(int argc,char**argv){if(argc<2)return 1;stream.open(argv[1]);gazebo::client::setup();gazebo::transport::NodePtr n(new gazebo::transport::Node);n->Init("default");auto sub=n->Subscribe("~/calibration_sensor/base_link/lidar_3d/scan",scan);for(int i=0;i<300&&count<30;++i)gazebo::common::Time::MSleep(100);gazebo::client::shutdown();stream.close();}
