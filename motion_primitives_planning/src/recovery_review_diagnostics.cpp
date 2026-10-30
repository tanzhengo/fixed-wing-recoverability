// Reviewer diagnostics reuse the exact frozen scene generator and planner.
// This translation unit introduces no changes to archived experiment sources.
#define main frozen_learning_benchmark_main
#include "recovery_learning_benchmark.cpp"
#undef main
#include <set>

struct DiagnosticRecord { MP::s_t state; MP::pr_t previous; MP::selection_t decision; int step; };
struct DiagnosticMethod { std::string name; Method method; };

double diagnostic_distance(const MP::s_t& a,const MP::s_t& b) {
 return std::sqrt(std::pow(a.pos[0]-b.pos[0],2)+std::pow(a.pos[1]-b.pos[1],2)+std::pow(a.h-b.h,2));
}
template<class States> double diagnostic_clearance(const MP& p,const States& states) {
 double d=30.;for(const auto& s:states)d=std::min(d,double(p.obstacle_signed_distance(s.pos[0],s.pos[1],s.h)));return d;
}
EvaluationPlant::Result diagnostic_execution(const MP::s_t& s,const MP::pr_t& previous,
 const MP::pr_t& chosen,const Condition& c) {
 EvaluationPlant plant(c.parameters);
 if(c.delay<=0)return plant.advance(s,chosen,.2);
 auto a=plant.advance(s,previous,c.delay);
 if(!a.valid)return a;
 auto b=plant.advance(a.states.back(),chosen,.2-c.delay);
 a.states.insert(a.states.end(),b.states.begin()+1,b.states.end());a.valid=b.valid;return a;
}

int main(int argc,char** argv) {
 ros::init(argc,argv,"recovery_review_diagnostics");ros::NodeHandle args("~");Options o;
 o.split="test";o.phase="all";o.work_power=0;
 args.param<std::string>("output_prefix",o.prefix,"/tmp/review_diagnostics");
 args.param<std::string>("model_path",o.model,"");args.param<std::string>("state_only_model_path",o.state_only_model,"");
 args.param<int>("maps_per_family",o.maps_per_family,8);
 std::string yaw_mask;args.param<std::string>("yaw_mask_model_path",yaw_mask,"");
 const std::vector<DiagnosticMethod> methods={
  {"geometry",{"learned12","learned",12}},
  {"no_geometry",{"state_only12","learned",12}},
  {"diverse",{"diverse12","diverse",12}},
  {"full",{"full203","cost",203}},
  {"geometry_yaw_mask",{"maskedlearned12","learned",12}}};
 std::vector<Condition> conditions(4);conditions[0].name="nominal";conditions[1].name="model_mismatch";
 conditions[1].parameters.actuator_scale=1.25;conditions[1].parameters.lift_scale=.97;conditions[1].parameters.drag_scale=1.10;
 conditions[2].name="crosswind";conditions[2].parameters.wind_y=1.;conditions[3].name="delay100ms";conditions[3].delay=.1;
 try {
  auto ef=output(o.prefix+"_episodes.csv"),pairs=output(o.prefix+"_paired.csv"),features=output(o.prefix+"_features.csv");
  ef<<"scene,family,condition,method,outcome,steps,sim_seconds,goal_distance\n";
  pairs<<"state_id,source_method,source_outcome,source_role,scene,family,condition,step,method,primary_feasible,verified,cost,rank,primary_clearance,selected_backup_clearance,feasible_backups,backup_count,best_backup_clearance,min_gamma_slack_deg,min_altitude_slack,euler02_rk4_error,euler005_rk4_error,euler02_actual_error,rk4_actual_error,euler02_short_clearance,euler005_short_clearance,rk4_short_clearance,actual_short_clearance,euler02_short_collision,euler005_short_collision,rk4_short_collision,actual_short_collision,"<<state_fields<<"\n";
  features<<"scene,family,condition,method,step,verified";for(int j=0;j<103;++j)features<<",f"<<j;features<<'\n';
  int episode_id=0,state_id=0;
  for(const auto& scene:scenes(o)) {
   std::vector<std::unique_ptr<MP>> planners;std::vector<std::string> namespaces;
   for(const auto& m:methods){std::string ns="/review_diagnostic/"+scene.name+"_"+m.name;namespaces.push_back(ns);ros::NodeHandle nh(ns);Options mo=o;if(m.name=="geometry_yaw_mask")mo.model=yaw_mask;configure(nh,scene,m.method,mo);planners.emplace_back(new MP(nh));}
   std::string fine_ns="/review_diagnostic/"+scene.name+"_fine";ros::NodeHandle fine_nh(fine_ns);configure(fine_nh,scene,{"fine","cost",203},o);fine_nh.setParam("motion_primitive/integration_dt",.005);MP fine(fine_nh);
   for(const auto& condition:conditions)for(size_t method_id=0;method_id<4;++method_id) {
    ros::NodeHandle replay_nh("/review_diagnostic/replay");configure(replay_nh,scene,methods[method_id].method,o);
    MP p(replay_nh);auto s=trim(p);auto previous=history_control(s);
    std::string outcome="timeout";int steps=0;std::vector<DiagnosticRecord> records;
    auto inspect=[&](const EvaluationPlant::Result& segment){
     if(!segment.valid){outcome="invalid_dynamics";return false;}
     for(const auto& q:segment.states){if(!p.state_within_recovery_envelope(q)){outcome="envelope_violation";return false;}if(p.collision_check(q.pos[0],q.pos[1],q.h)){outcome="collision";return false;}}
     return true;
    };
    for(int step=0;step<200;++step) {
     if(goal(s)){outcome="goal";break;}
     auto d=p.select_motion_primitive(s);records.push_back({s,previous,d,step});
     if(!d.primary_feasible){outcome="no_feasible_primary";break;}
     MP::recovery_candidate_t candidate;candidate.terminal=d.rollout.states.back();candidate.cost=d.cost;auto f=p.recovery_features(s,candidate);
     features<<scene.name<<','<<scene.family<<','<<condition.name<<','<<methods[method_id].name<<','<<step<<','<<d.recovery_verified;for(float v:f)features<<','<<v;features<<'\n';
     p.accept_selection(d);bool ok=true;EvaluationPlant plant(condition.parameters);
     if(condition.delay>0){auto r=plant.advance(s,previous,condition.delay);s=r.states.back();ok=inspect(r);}
     if(ok){auto r=plant.advance(s,d.command,.2-condition.delay);s=r.states.back();ok=inspect(r);}
     previous=d.command;p.advance_execution(.2);++steps;if(!ok)break;if(goal(s)){outcome="goal";break;}
    }
    ef<<scene.name<<','<<scene.family<<','<<condition.name<<','<<methods[method_id].name<<','<<outcome<<','<<steps<<','<<steps*.2<<','<<std::hypot(250-s.pos[0],s.pos[1])<<'\n';
    if(outcome!="goal"&&!records.empty()) {
     std::set<size_t> chosen;chosen.insert(records.size()-1);
     for(size_t j=records.size();j>0;--j)if(records[j-1].decision.primary_feasible){chosen.insert(j-1);break;}
     for(size_t index:chosen) {
      const auto& r=records[index];++state_id;
      for(size_t m=0;m<methods.size();++m) {
       MP& alt=*planners[m];alt.remember_command(r.previous);auto d=alt.select_motion_primitive(r.state);
       const double nan=std::numeric_limits<double>::quiet_NaN();
       double primary_c=nan,selected_c=nan,best_c=nan,gamma_slack=nan,altitude_slack=nan;
       double err02=nan,err005=nan,erract=nan,errrk4=nan,c02=nan,c005=nan,crk4=nan,cact=nan;
       int feasible=0,count=0,coll02=-1,coll005=-1,collrk4=-1,collact=-1;
       if(d.primary_feasible){
        primary_c=diagnostic_clearance(alt,d.rollout.states);gamma_slack=20.;altitude_slack=1e9;
        for(const auto& q:d.rollout.states){gamma_slack=std::min(gamma_slack,20.-std::abs(q.gamma)*180/M_PI);altitude_slack=std::min(altitude_slack,std::min(double(q.h)-scene.low,scene.high-double(q.h)));}
        if(d.recovery_verified)selected_c=diagnostic_clearance(alt,d.backup_rollout.states);
        const auto& terminal=d.rollout.states.back();auto bank=alt.generate_recovery_library(terminal);count=bank.size();
        for(const auto& b:bank){auto rr=alt.rollout_motion_primitive(terminal,b,d.command.t_way);if(alt.check_primary_feasibility(r.state,rr)){++feasible;double cc=diagnostic_clearance(alt,rr.states);if(!std::isfinite(best_c)||cc>best_c)best_c=cc;}}
        auto short_command=d.command;short_command.t_way=.2f;
        auto e02=alt.rollout_motion_primitive(r.state,short_command);auto e005=fine.rollout_motion_primitive(r.state,short_command);
        auto rk4=EvaluationPlant().advance(r.state,d.command,.2);auto actual=diagnostic_execution(r.state,r.previous,d.command,condition);
        if(e02.valid&&e005.valid&&rk4.valid&&actual.valid){err02=diagnostic_distance(e02.states.back(),rk4.states.back());err005=diagnostic_distance(e005.states.back(),rk4.states.back());erract=diagnostic_distance(e02.states.back(),actual.states.back());errrk4=diagnostic_distance(rk4.states.back(),actual.states.back());c02=diagnostic_clearance(alt,e02.states);c005=diagnostic_clearance(alt,e005.states);crk4=diagnostic_clearance(alt,rk4.states);cact=diagnostic_clearance(alt,actual.states);coll02=c02<=0;coll005=c005<=0;collrk4=crk4<=0;collact=cact<=0;}
       }
       pairs<<state_id<<','<<methods[method_id].name<<','<<outcome<<','<<(index==records.size()-1?"final_attempt":"last_executed")<<','<<scene.name<<','<<scene.family<<','<<condition.name<<','<<r.step<<','<<methods[m].name<<','<<d.primary_feasible<<','<<d.recovery_verified<<','<<d.cost<<','<<d.selected_cost_rank<<','<<primary_c<<','<<selected_c<<','<<feasible<<','<<count<<','<<best_c<<','<<gamma_slack<<','<<altitude_slack<<','<<err02<<','<<err005<<','<<erract<<','<<errrk4<<','<<c02<<','<<c005<<','<<crk4<<','<<cact<<','<<coll02<<','<<coll005<<','<<collrk4<<','<<collact<<',';state_record(pairs,r.state);pairs<<'\n';
      }
     }
    }
    ++episode_id;std::cout<<episode_id<<"/512 "<<scene.name<<' '<<condition.name<<' '<<methods[method_id].name<<' '<<outcome<<" states="<<state_id<<std::endl;
    ef.flush();pairs.flush();features.flush();
   }
   planners.clear();for(const auto& ns:namespaces)ros::param::del(ns);ros::param::del(fine_ns);
  }
 } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
 return 0;
}
