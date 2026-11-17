#include <ros/ros.h>
#include <ros/serialization.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <sstream>
#include <map>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include "runtime_motion_primitives.h"
using Planner=RuntimeMotionPrimitives;
std::vector<std::vector<double>> csv(const std::string&p){std::ifstream in(p);if(!in)throw std::runtime_error("missing "+p);std::string l;std::vector<std::vector<double>> out;while(std::getline(in,l)){std::replace(l.begin(),l.end(),',',' ');std::istringstream s(l);std::vector<double> a;double v;while(s>>v)a.push_back(v);out.push_back(a);}return out;}
nav_msgs::Odometry odom(const std::vector<double>&a){nav_msgs::Odometry m;m.header.stamp.fromSec(a[0]);m.header.frame_id="world";m.pose.pose.position.x=a[1];m.pose.pose.position.y=a[2];m.pose.pose.position.z=a[3];m.pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(-a[7],-a[8]-a[5],a[6]);m.twist.twist.linear.x=a[4];m.twist.twist.linear.y=a[5];m.twist.twist.linear.z=a[6];m.twist.twist.angular.x=a[9];m.twist.twist.angular.y=a[10];return m;}
Planner::s_t state(const std::vector<double>&a){Planner::s_t s;s.pos[0]=a[1];s.pos[1]=a[2];s.h=a[3];s.V=a[4];s.gamma=a[5];s.sigma=a[6];s.roll=a[7];s.attack=a[8];s.T=a[9];s.d_sigma=a[10];const auto q=odom(a).pose.pose.orientation;s.q[0]=q.w;s.q[1]=q.x;s.q[2]=q.y;s.q[3]=q.z;return s;}

std::string witness_hash(const Planner::selection_t&r) {
 std::ostringstream bytes;bytes<<std::hexfloat<<r.status<<' '<<r.recovery_verified<<' '<<r.primary_feasible<<' '<<r.cost<<' '<<r.selected_cost_rank;
 auto command=[&](const Planner::pr_t&c){bytes<<' '<<c.w_roll_c<<' '<<c.attack_c<<' '<<c.T_c<<' '<<c.t_way;};
 auto states=[&](const Planner::rollout_t&trace){for(const auto&s:trace.states)bytes<<' '<<s.pos[0]<<' '<<s.pos[1]<<' '<<s.h<<' '<<s.V<<' '<<s.gamma<<' '<<s.sigma<<' '<<s.roll<<' '<<s.attack<<' '<<s.T<<' '<<s.d_sigma;};
 command(r.command);states(r.rollout);if(r.recovery_verified){command(r.backup_command);states(r.backup_rollout);for(const auto&c:r.execution_plan)command(c);}
 std::uint64_t h=14695981039346656037ull;for(unsigned char b:bytes.str()){h^=b;h*=1099511628211ull;}std::ostringstream hex;hex<<std::hex<<h;return hex.str();
}
struct Method {std::string name,order;bool compact,deferred,pruning;int k;std::string extra_model;float alpha;};
int main(int argc,char**argv){
 try {
  ros::init(argc,argv,"unknown_query_replay",ros::init_options::AnonymousName);if(argc<6)throw std::runtime_error("usage replay inputdir outputdir namespace compactmodel suffix [maxqueries]");const std::string dir=argv[1],outdir=argv[2],suffix=argv[5];ros::NodeHandle nh(argv[3]);Planner p(nh);p.planning_timer.stop();p.reuse_verified_plan=false;p.cached_execution_plan.clear();
  if(!p.learned_network.ready()||p.recovery_order_mode!="learned"||!p.sensor_map_enabled||!p.static_obs_vec.empty()||!p.box_obstacles_.empty())throw std::runtime_error("invalid replay config");
  auto v1=p.learned_network;LearnedRecoveryNetwork v2;if(!v2.load(argv[4]))throw std::runtime_error("compact load failed");
#ifdef FROZEN_BASELINE
  std::vector<Method> methods={{"V1_Eager","learned",false,false,false,12},{"Diverse","diverse",false,false,false,12},{"Cost","cost",false,false,false,12},{"Full","cost",false,false,false,10000}};
  int repeats=1;
#else
  std::vector<Method> methods={{"V1_Eager","learned",false,false,false,12},{"V2_Eager","learned",true,false,false,12},{"V2_Deferred","learned",true,true,false,12},{"V2_DeferredPruned","learned",true,true,true,12},{"Diverse","diverse",false,false,false,12},{"DiversePruned","diverse",false,true,true,12},{"Cost","cost",false,true,true,12},{"Full","cost",false,true,true,10000}};
#ifdef REPLAY_INSTRUMENTED
  int repeats=1;
#else
  int repeats=3;
#endif
#endif
#ifndef FROZEN_BASELINE
  if(const char* value=std::getenv("REPLAY_REPEATS")) {repeats=std::stoi(value);if(repeats<1||repeats>5)throw std::runtime_error("invalid repeat count");}
  std::map<std::string,LearnedRecoveryNetwork> extra_networks;
  if(const char* file=std::getenv("REPLAY_METHODS_TSV")) {
   std::ifstream spec(file);if(!spec)throw std::runtime_error("missing extra methods file");std::string line;
   while(std::getline(spec,line)) {
    if(line.empty()||line[0]=='#')continue;std::istringstream row(line);std::string name,order,path;float alpha;
    if(!(row>>name>>order>>path>>alpha)||alpha<0||alpha>1)throw std::runtime_error("invalid extra method row");
    int budget=12;row>>std::ws;
    if(!row.eof() && (!(row>>budget)||budget<1||budget>10000))throw std::runtime_error("invalid optional candidate budget");
    std::string extra;if(row>>extra)throw std::runtime_error("extra method fields");
    Method m{name,order,true,true,true,budget};m.alpha=alpha;
    if(path!="-") {auto&net=extra_networks[path];if(!net.ready()&&!net.load(path))throw std::runtime_error("extra model load failed");m.extra_model=path;}
    methods.push_back(m);
   }
  }
#endif
  const auto poses=csv(dir+"/poses.csv"), scans=csv(dir+"/accepted_scans.csv"),queries=csv(dir+"/queries.csv");
  std::map<long long,sensor_msgs::PointCloud2Ptr> raw;
  {std::ifstream in(dir+"/rays.bin",std::ios::binary);while(true){std::uint32_t n=0;in.read(reinterpret_cast<char*>(&n),4);if(!in)break;std::vector<std::uint8_t> data(n);in.read(reinterpret_cast<char*>(data.data()),n);if(!in)throw std::runtime_error("truncated rays");sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);ros::serialization::IStream stream(data.data(),n);ros::serialization::deserialize(stream,*msg);raw[std::llround(msg->header.stamp.toSec()*1e6)]=msg;}}
  std::ofstream audit(outdir+"/map_audit_"+suffix+".csv"), out(outdir+"/query_results_"+suffix+".csv");audit<<"scan_stamp,receipt,free_expected,free_replay,occupied_expected,occupied_replay,scan_count,integrated\n";
  out<<"query_index,target_time,state_stamp,map_stamp,scan_count,free_cells,occupied_cells,method,repeat,order,ms,primary_evaluated,primary_feasible,witness,queries,backup_steps,primary_steps,cost_rank,cost,roll,attack,thrust,status,signature,inference_batches,inference_candidates,feature_candidates,observed_feature_candidates,pruned_candidates,scheduled_count,primary_ms,feature_ms,inference_ms,schedule_other_ms,continuation_ms,residual_ms\n";out<<std::setprecision(17);audit<<std::setprecision(17);
  size_t pi=0,si=0;const int maxq=argc>6?std::stoi(argv[6]):queries.size();
  for(size_t qi=0;qi<queries.size()&&qi<maxq;++qi){const auto&q=queries[qi];const double qt=q[1];
   while(si<scans.size()&&scans[si][1]<=qt+1e-9&&scans[si][0]<=qt+1e-9){const auto&sc=scans[si];const double st=sc[0];
    while(pi<poses.size()&&(pi==0||poses[pi-1][0]<st)){if(poses[pi][0]>qt+1e-9)throw std::runtime_error("future pose");p.sensor_map->add_pose(odom(poses[pi]));++pi;}
    auto it=raw.find(std::llround(st*1e6));if(it==raw.end())throw std::runtime_error("missing scan");p.sensor_map->rays(it->second);const auto ms=p.sensor_map->map().stats();
    const bool integrated=std::abs(ms.last_stamp-st)<=1e-6;
    audit<<st<<','<<sc[1]<<','<<sc[2]<<','<<ms.free_cells<<','<<sc[3]<<','<<ms.occupied_cells<<','<<ms.scans<<','<<integrated<<'\n';++si;
   }
   auto it=std::upper_bound(poses.begin(),poses.end(),qt,[](double t,const std::vector<double>&a){return t<a[0];});if(it==poses.begin())throw std::runtime_error("state missing");--it;const auto s=state(*it);if(std::abs((*it)[0]-qt)>1e-7)throw std::runtime_error("state time mismatch");
   if(!p.sensor_map->ready(qt))throw std::runtime_error("stale map");p.global_target.pos[0]=q[2];p.global_target.pos[1]=q[3];p.global_target.h=q[4];
   for(int repeat=0;repeat<repeats;++repeat)for(size_t j=0;j<methods.size();++j){const int method=(qi+repeat+j)%methods.size();const auto& m=methods[method];p.recovery_order_mode=m.order;p.recovery_max_candidates=m.k;
#ifndef FROZEN_BASELINE
    auto& method_network=m.extra_model.empty()?(m.compact?v2:v1):extra_networks.at(m.extra_model);
    // Switch model storage without copying or artificially warming its weights.
    std::swap(p.learned_network,method_network);p.deferred_inference=m.deferred;p.prune_dominated=m.pruning;p.coverage_weight=m.alpha;
#endif
#ifdef REPLAY_INSTRUMENTED
    p.replay_primary=p.replay_schedule=p.replay_features=p.replay_inference=p.replay_continuation=0;
#endif
    p.sensor_map->map_->clear_query_cache();p.cached_execution_plan.clear();if(p.last_pr_ptr)throw std::runtime_error("unexpected previous control");
    const auto t0=std::chrono::steady_clock::now();const auto r=p.select_planning_proposal(s);const auto t1=std::chrono::steady_clock::now();const double elapsed=std::chrono::duration<double,std::milli>(t1-t0).count();const auto ms=p.sensor_map->map().stats();
#ifndef FROZEN_BASELINE
    std::swap(p.learned_network,method_network);
#endif
    out<<qi<<','<<q[0]<<','<<qt<<','<<p.sensor_map->stamp()<<','<<ms.scans<<','<<ms.free_cells<<','<<ms.occupied_cells<<','<<m.name<<','<<repeat<<','<<j<<','<<elapsed<<','<<r.primary_evaluated<<','<<r.primary_feasible_count<<','<<r.recovery_verified<<','<<r.recovery_checked<<','<<r.backup_integration_steps<<','<<r.primary_integration_steps<<','<<r.selected_cost_rank<<','<<r.cost<<','<<r.command.w_roll_c<<','<<r.command.attack_c<<','<<r.command.T_c<<','<<r.status<<','<<witness_hash(r)
#ifdef FROZEN_BASELINE
    <<",0,0,0,0,0,"<<r.scheduled_count
#else
    <<','<<r.inference_batches<<','<<r.inference_candidates<<','<<r.feature_candidates<<','<<r.observed_feature_candidates<<','<<r.pruned_candidates<<','<<r.scheduled_count
#endif
#ifdef REPLAY_INSTRUMENTED
    <<','<<p.replay_primary<<','<<p.replay_features<<','<<p.replay_inference<<','<<(p.replay_schedule-p.replay_features-p.replay_inference)<<','<<p.replay_continuation<<','<<(elapsed-p.replay_primary-p.replay_schedule-p.replay_continuation)
#else
    <<",0,0,0,0,0,0"
#endif
    <<'\n';
   }
   out.flush();audit.flush();std::cout<<dir<<" query "<<qi+1<<'/'<<maxq<<" t="<<qt<<" scans="<<si<<" free="<<p.sensor_map->map().stats().free_cells<<std::endl;
  }
  ros::shutdown();
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
