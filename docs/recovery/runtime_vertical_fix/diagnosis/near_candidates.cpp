#include "motion_primitives.h"
#include <fstream>
#include <iomanip>
#include <map>
using MP=MotionPrimitives;
int main(int argc,char**argv){
 ros::init(argc,argv,"codex_candidate_diagnostic",ros::init_options::AnonymousName);
 const std::string ns="/codex_candidate_diagnostic_isolated";
 XmlRpc::XmlRpcValue params;if(!ros::param::get("/suav1",params)){std::cerr<<"No live parameters\n";return 2;}
 ros::param::set(ns,params);ros::NodeHandle nh(ns);
 nh.setParam("waypoints/num",1);nh.setParam("waypoints/wp1/x",350.);nh.setParam("waypoints/wp1/y",0.);nh.setParam("waypoints/wp1/h",24.);
 std::ifstream f(argv[1]);std::ofstream out(argv[2]);out<<std::setprecision(10);
 out<<"sample,candidate,wp_h,theta,feasible,valid,collision,cost,end_h,end_x,end_y,end_sigma,end_gamma,roll,attack,thrust,backup_verified,heading_cost,altitude_cost,distance_cost,control_cost\n";
 {MP p(nh); std::string line;
 while(std::getline(f,line)){
  std::replace(line.begin(),line.end(),',',' ');std::stringstream row(line);int sample;MP::s_t s;
  row>>sample>>s.pos[0]>>s.pos[1]>>s.h>>s.V>>s.gamma>>s.sigma>>s.roll>>s.attack>>s.T>>s.d_sigma;
  MP::pr_t previous;previous.w_roll_c=.6108652353;previous.attack_c=-.0372508;previous.T_c=.894687;previous.t_way=3;p.remember_command(previous);
  std::vector<MP::pr_t> library;
  for(float dh:{-4.f,-3.f,-2.f,-1.f,-.5f,0.f,.5f,1.f,2.f,3.f,4.f}) for(int i=0;i<29;++i){
   MP::wp_t wp;wp.d=46.5;wp.V=15.5;wp.h=s.h+dh;wp.theata=-M_PI/4+i*(M_PI/2)/28;
   auto command=p.generate_unlimited_agility_cmd(wp,s);command=p.compensate_cmd(command,s);library.push_back(command);
  }
  int id=0;
  for(const auto& pr:library){
   const auto trajectory=p.rollout_motion_primitive(s,pr);const bool feasible=p.check_primary_feasibility(s,trajectory);
   const auto e=trajectory.states.back();const auto c=p.evaluate_motion_primitive(s,pr,trajectory);
   bool verified=feasible&&p.check_terminal_recovery(s,trajectory,pr.t_way).verified;
   double ex=350-e.pos[0],ey=-e.pos[1],ez=24-e.h;
   double hc=3*p.abs_err_angle(std::atan2(ey,ex),e.sigma),ac=.15*std::abs(ez),dc=.02*std::sqrt(ex*ex+ey*ey+ez*ez);
   double cc=.1*std::abs(pr.attack_c-previous.attack_c)+std::abs(pr.w_roll_c-previous.w_roll_c)+.05*std::abs(pr.T_c-previous.T_c);
   out<<sample<<','<<id++<<','<<pr.wp.h<<','<<pr.wp.theata<<','<<feasible<<','<<trajectory.valid<<','<<trajectory.collision<<','<<c<<','<<e.h<<','<<e.pos[0]<<','<<e.pos[1]<<','<<e.sigma<<','<<e.gamma<<','<<pr.w_roll_c<<','<<pr.attack_c<<','<<pr.T_c<<','<<verified<<','<<hc<<','<<ac<<','<<dc<<','<<cc<<'\n';
  }
 }
 }
 ros::param::del(ns);
}
