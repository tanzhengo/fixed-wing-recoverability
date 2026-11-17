#pragma once
#include "runtime_motion_primitives.h"
#include "evaluation_plant.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <memory>
#include <cassert>
#include <sys/stat.h>
using P=RuntimeMotionPrimitives;
using Pt=sensor_occupancy::Point;
struct Ob{int type;double x,y,z,r,height;};
struct Scene{std::string family,name;unsigned seed;double low=16,high=24,goal_x=250;int tree_count=0,bush_count=0;std::vector<Ob> obs;};
struct Gen{unsigned state;double range(double a,double b){state=1664525u*state+1013904223u;return a+(b-a)*(double(state)/4294967296.);}};
Scene scene(int family,int index,unsigned base,double length){
 if((family!=100&&family!=200&&family!=300)||index<0||index>7||length!=600)throw std::runtime_error("Invalid fixed-length forest scene");
 Scene s;s.family="forest_d"+std::to_string(family);s.name=s.family+"_L600_"+std::to_string(index);s.seed=base+index;s.low=5;s.high=45;s.goal_x=length;
 Gen g{s.seed};Gen bushes{s.seed^0x51ed270bu};s.tree_count=family;s.bush_count=33;
 std::vector<Ob> trees;for(int j=0;j<300;++j){double x=g.range(40,length-10),y=g.range(-80,80),r=g.range(.8,1.4),h=g.range(24,36),c=g.range(4,6);trees.push_back({0,x,y,h/2,r,h});trees.push_back({1,x,y,h,c,0});}
 s.obs.insert(s.obs.end(),trees.begin(),trees.begin()+2*s.tree_count);for(int j=0;j<s.bush_count;++j){double x=bushes.range(40,length-10),y=bushes.range(-80,80),r=bushes.range(3,5);s.obs.push_back({1,x,y,r,r,0});}return s;
}
// Nearest physical ray intersection; these geometric objects never enter the planner.
double hit(const Pt&o,const Pt&d,const Ob&b,double limit){
 double x=o.x-b.x,y=o.y-b.y,z=o.z-b.z;double best=limit;
 if(b.type==1){double bb=x*d.x+y*d.y+z*d.z,cc=x*x+y*y+z*z-b.r*b.r,disc=bb*bb-cc;if(disc>=0){double t=-bb-std::sqrt(disc);if(t>1e-7&&t<best)best=t;}return best;}
 double aa=d.x*d.x+d.y*d.y,bb=x*d.x+y*d.y,cc=x*x+y*y-b.r*b.r,disc=bb*bb-aa*cc;
 if(aa>1e-12&&disc>=0)for(double t:{(-bb-std::sqrt(disc))/aa,(-bb+std::sqrt(disc))/aa})if(t>1e-7&&t<best&&std::abs(z+t*d.z)<=b.height/2)best=t;
 if(std::abs(d.z)>1e-12)for(double side:{-1.,1.}){double t=(side*b.height/2-z)/d.z;if(t>1e-7&&t<best&&std::pow(x+t*d.x,2)+std::pow(y+t*d.y,2)<=b.r*b.r)best=t;}
 return best;
}
double clearance(const Pt&p,const Ob&o){double x=p.x-o.x,y=p.y-o.y,z=p.z-o.z;if(o.type==1)return std::sqrt(x*x+y*y+z*z)-o.r;double dr=std::hypot(x,y)-o.r,dh=std::abs(z)-o.height/2;return std::hypot(std::max(dr,0.),std::max(dh,0.))+std::min(std::max(dr,dh),0.);}
double chord_clearance(const Pt&a,const Pt&b,const Ob&o){
 auto point=[&](double t){return Pt{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)};};
 double lo=0,hi=1;for(int i=0;i<36;++i){double m1=(2*lo+hi)/3,m2=(lo+2*hi)/3;if(clearance(point(m1),o)<clearance(point(m2),o))hi=m2;else lo=m1;}
 return std::min({clearance(a,o),clearance(b,o),clearance(point((lo+hi)/2),o)});
}
std::vector<Pt> directions(){std::vector<Pt> v;v.reserve(512*32);for(int j=0;j<32;++j){double el=(-40.+60.*j/31)*M_PI/180;for(int i=0;i<512;++i){double az=-M_PI+2*M_PI*i/512;v.push_back({std::cos(el)*std::cos(az),std::cos(el)*std::sin(az),std::sin(el)});}}return v;}
void scan(P&planner,const Scene&sc,const P::s_t&s,double time,const std::vector<Pt>&dirs){
 tf::Quaternion q;q.setRPY(-s.roll,-s.attack-s.gamma,s.sigma);tf::Matrix3x3 R(q);tf::Vector3 origin=tf::Vector3(s.pos[0],s.pos[1],s.h)+R*tf::Vector3(0,0,.15);Pt o{origin.x(),origin.y(),origin.z()};
 std::vector<sensor_occupancy::Ray> rays;rays.reserve(dirs.size());
 for(const Pt&l:dirs){auto dv=R*tf::Vector3(l.x,l.y,l.z);Pt d{dv.x(),dv.y(),dv.z()};double t=120;if(d.z<0){double ground=-o.z/d.z;if(ground>0)t=std::min(t,ground);}for(const auto&ob:sc.obs)t=hit(o,d,ob,t);if(t<1)continue;rays.push_back({{o.x+t*d.x,o.y+t*d.y,o.z+t*d.z},t<120?1:0});}
 if(!planner.sensor_map->map_->integrate_scan(o,rays,1+time))throw std::runtime_error("map integration failed");planner.sensor_map->last_stamp_=1+time;
}
P::s_t trim(const P&p){double speed=15.5,lo=-20*M_PI/180,hi=25*M_PI/180;for(int i=0;i<60;++i){double a=(lo+hi)/2,r=1.86*speed*speed*(p.C_D(a)*std::tan(a)+p.C_L(a))-9.81;if(r>0)hi=a;else lo=a;}P::s_t s;s.h=20;s.V=speed;s.attack=(lo+hi)/2;s.T=1.86*speed*speed*p.C_D(s.attack)/std::cos(s.attack);return s;}
void cfg(ros::NodeHandle&nh,const Scene&sc,const std::string&model){
 ros::param::del(nh.getNamespace());
 nh.setParam("sensor_map/enabled",true);nh.setParam("sensor_map/resolution",2.);nh.setParam("sensor_map/max_cells",8000000);nh.setParam("sensor_map/clearance",2.);nh.setParam("sensor_map/planning_reserve",1.);
 nh.setParam("sensor_map/frame","suav1/lidar");nh.setParam("execution/planning_lookahead",0.);nh.setParam("execution/compensate_latency",false);
 nh.setParam("motion_primitive/horizon_time",3.);nh.setParam("motion_primitive/V",15.5);nh.setParam("motion_primitive/num",29);nh.setParam("motion_primitive/vertical_num",7);nh.setParam("motion_primitive/state_relative_altitudes",true);nh.setParam("motion_primitive/min_altitude",sc.low);nh.setParam("motion_primitive/max_altitude",sc.high);
 nh.setParam("recovery/enabled",true);nh.setParam("recovery/strict_execution",true);nh.setParam("recovery/reuse_verified_plan",true);nh.setParam("recovery/stop_on_rejection",true);nh.setParam("recovery/require_terminal",true);nh.setParam("recovery/horizon_time",3.);nh.setParam("recovery/min_execution_time",.1);nh.setParam("recovery/max_candidates",12);nh.setParam("recovery/candidate_order","learned");nh.setParam("recovery/model_path",model);nh.setParam("recovery/state_only_features",true);nh.setParam("recovery/learned_work_power",0.);
 nh.setParam("waypoints/num",1);nh.setParam("waypoints/wp1/x",sc.goal_x);nh.setParam("waypoints/wp1/y",0.);nh.setParam("waypoints/wp1/h",20.);
 for(const std::string k:{"static_obstacles","sphere_obstacles","horizontal_obstacles","dynamic_obstacles","box_obstacles"})nh.setParam(k+"/num",0);
}
