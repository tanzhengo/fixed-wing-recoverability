// Exactly shared-prefix counterfactual for the frozen geometry-input ablation.
#define main frozen_learning_benchmark_main
#include "recovery_learning_benchmark.cpp"
#undef main
struct BranchMetrics {double cost=0,primary=30,backup=30,gamma=20,altitude=1e9;int feasible=0,total=0,rank=0;};
BranchMetrics branch_metrics(MP& p,const MP::s_t& s,const MP::selection_t& d,const Scene& scene) {
 BranchMetrics m;m.cost=d.cost;m.rank=d.selected_cost_rank;
 for(const auto& q:d.rollout.states){m.primary=std::min(m.primary,double(p.obstacle_signed_distance(q.pos[0],q.pos[1],q.h)));m.gamma=std::min(m.gamma,20.-std::abs(q.gamma)*180/M_PI);m.altitude=std::min(m.altitude,std::min(double(q.h)-scene.low,scene.high-double(q.h)));}
 if(d.recovery_verified)for(const auto& q:d.backup_rollout.states)m.backup=std::min(m.backup,double(p.obstacle_signed_distance(q.pos[0],q.pos[1],q.h)));else m.backup=std::numeric_limits<double>::quiet_NaN();
 const auto& t=d.rollout.states.back();
 for(const auto& c:p.generate_recovery_library(t)){++m.total;auto b=p.rollout_motion_primitive(t,c,d.command.t_way);m.feasible+=p.check_primary_feasibility(s,b);}
 return m;
}
void branch_values(std::ostream& f,const BranchMetrics& m){f<<m.cost<<','<<m.primary<<','<<m.backup<<','<<m.feasible<<','<<m.total<<','<<m.gamma<<','<<m.altitude<<','<<m.rank;}
int main(int argc,char** argv) {
 ros::init(argc,argv,"recovery_review_branchpoints");ros::NodeHandle args("~");Options o;o.split="test";o.work_power=0;o.maps_per_family=8;
 args.param<std::string>("output_prefix",o.prefix,"/tmp/review_branch");args.param<std::string>("model_path",o.model,"");args.param<std::string>("state_only_model_path",o.state_only_model,"");
 std::vector<Condition> cs(4);cs[0].name="nominal";cs[1].name="model_mismatch";cs[1].parameters.actuator_scale=1.25;cs[1].parameters.lift_scale=.97;cs[1].parameters.drag_scale=1.10;cs[2].name="crosswind";cs[2].parameters.wind_y=1;cs[3].name="delay100ms";cs[3].delay=.1;
 try {
  auto f=output(o.prefix+"_branchpoints.csv");
  f<<"scene,family,condition,branch_found,step,common_prefix_outcome,geometry_verified,no_geometry_verified,g_cost,g_primary_clearance,g_backup_clearance,g_feasible,g_total,g_gamma_slack,g_altitude_slack,g_rank,n_cost,n_primary_clearance,n_backup_clearance,n_feasible,n_total,n_gamma_slack,n_altitude_slack,n_rank,"<<state_fields<<'\n';
  int done=0;
  for(const auto& scene:scenes(o))for(const auto& c:cs){
   ros::NodeHandle ng("/review_branch/geometry"),nn("/review_branch/no_geometry");configure(ng,scene,{"learned12","learned",12},o);configure(nn,scene,{"state_only12","learned",12},o);MP g(ng),n(nn);auto s=trim(g);auto prev=history_control(s);EvaluationPlant plant(c.parameters);bool branched=false;std::string outcome="timeout";int step=0;MP::selection_t gd,nd;
   for(;step<200;++step){
    if(goal(s)){outcome="goal";break;}
    gd=g.select_motion_primitive(s);nd=n.select_motion_primitive(s);
    if(!gd.primary_feasible||!nd.primary_feasible){outcome="no_feasible_primary";break;}
    if(gd.command.w_roll_c!=nd.command.w_roll_c||gd.command.attack_c!=nd.command.attack_c||gd.command.T_c!=nd.command.T_c){branched=true;outcome="branch";break;}
    g.accept_selection(gd);n.accept_selection(nd);bool ok=true;
    auto inspect=[&](const EvaluationPlant::Result& r){if(!r.valid){outcome="invalid_dynamics";return false;}for(const auto& q:r.states){if(!g.state_within_recovery_envelope(q)){outcome="envelope_violation";return false;}if(g.collision_check(q.pos[0],q.pos[1],q.h)){outcome="collision";return false;}}return true;};
    if(c.delay>0){auto r=plant.advance(s,prev,c.delay);s=r.states.back();ok=inspect(r);}if(ok){auto r=plant.advance(s,gd.command,.2-c.delay);s=r.states.back();ok=inspect(r);}prev=gd.command;g.advance_execution(.2);n.advance_execution(.2);if(!ok)break;
   }
   f<<scene.name<<','<<scene.family<<','<<c.name<<','<<branched<<','<<step<<','<<outcome<<','<<gd.recovery_verified<<','<<nd.recovery_verified<<',';
   if(branched){branch_values(f,branch_metrics(g,s,gd,scene));f<<',';branch_values(f,branch_metrics(n,s,nd,scene));}else f<<"nan,nan,nan,0,0,nan,nan,0,nan,nan,nan,0,0,nan,nan,0";
   f<<',';state_record(f,s);f<<'\n';f.flush();std::cout<<++done<<"/128 "<<scene.name<<' '<<c.name<<' '<<outcome<<" step="<<step<<std::endl;
  }
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
 return 0;
}
