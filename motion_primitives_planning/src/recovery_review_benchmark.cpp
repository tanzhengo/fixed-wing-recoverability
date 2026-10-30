#include "review_motion_primitives.h"
#include "evaluation_plant.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <cstdint>
#include <stdexcept>

// Review experiments: untouched seed ranges are gated externally by a model freeze.
// No dataset generation or training is performed by this executable.
using MP = MotionPrimitives;
struct Obstacle { std::string type; double x,y,z,r,height; };
struct Scene { std::string name,family; uint32_t seed; double low=16,high=24; std::vector<Obstacle> obstacles; };
struct Method { std::string name,order; int candidates=12,work=0; double horizon=3; bool terminal=true; };
struct Condition { std::string name; EvaluationPlant::Parameters parameters; double delay=0; };
struct Generator {
    uint32_t state;
    double range(double a,double b) { state=1664525u*state+1013904223u; return a+(b-a)*(double(state)/4294967296.); }
};
struct Options { std::string split,phase,prefix,model,state_only_model,compact_legacy_model,selected_model,geometry_bce,geometry_aux,state_bce,state_aux; int maps_per_family=0,repeats=3; bool profile=false,selection_only=false,prefix_check=false; double work_power=0; };
std::vector<Scene> scenes(const Options& options) {
    const uint32_t base=options.split=="train"?301000u:options.split=="val"?401000u:options.split=="review"?801000u:501000u;
    const int count=options.maps_per_family>0?options.maps_per_family:(options.split=="train"?40:8);
    std::vector<Scene> out;
    for(int family=0;family<4;++family) for(int seed=0;seed<count;++seed) {
        const std::string names[]={"staggered","wall","culdesac","spatial"};
        Scene s;s.family=names[family];s.name=options.split+"_"+s.family+"_"+std::to_string(seed);
        s.seed=base+family*1000u+seed; Generator g{s.seed};
        auto cylinder=[&](double x,double y,double r,double z=20,double h=100){s.obstacles.push_back({"cylinder",x,y,z,r,h});};
        if(family==0) for(int i=0;i<4;++i){const double x=65+i*43+g.range(-8,8);const double y=(i%2?-1:1)*g.range(4,22);const double r=g.range(8,13);cylinder(x,y,r);}
        if(family==1) {double x=g.range(75,110),cy=g.range(-10,10);for(int i=-2;i<=2;++i)cylinder(x,cy+i*18,g.range(6.5,9));cylinder(175,g.range(-30,30),12);}
        if(family==2) {double x=g.range(65,90),w=g.range(27,36);for(int i=0;i<4;++i){cylinder(x+i*20,-w,8);cylinder(x+i*20,w,8);}for(int i=-1;i<=1;++i)cylinder(x+70,i*17,8);}
        if(family==3) {s.low=5;s.high=45;for(int i=0;i<3;++i){const double h=g.range(18,33);const double y=g.range(-15,15);const double r=g.range(8,13);cylinder(65+i*63,y,r,h/2,h);}for(int i=0;i<2;++i){const double y=g.range(-15,15);const double z=g.range(20,34);const double r=g.range(8,12);s.obstacles.push_back({"sphere",105.+i*75,y,z,r,0});}}
        out.push_back(s);
    }
    return out;
}
void configure(ros::NodeHandle& nh,const Scene& scene,const Method& method,const Options& options) {
    ros::param::del(nh.getNamespace());
    nh.setParam("motion_primitive/horizon_time",method.horizon);nh.setParam("motion_primitive/V",15.5);
    nh.setParam("motion_primitive/num",29);nh.setParam("motion_primitive/vertical_num",7);
    nh.setParam("motion_primitive/min_altitude",scene.low);nh.setParam("motion_primitive/max_altitude",scene.high);
    nh.setParam("recovery/stop_on_rejection",true);nh.setParam("recovery/enabled",true);nh.setParam("recovery/require_terminal",method.terminal);
    nh.setParam("recovery/horizon_time",6.);nh.setParam("recovery/max_candidates",method.candidates);
    nh.setParam("recovery/candidate_order",method.order);nh.setParam("recovery/max_backup_integration_steps",method.work);
    nh.setParam("recovery/known_space_radius",0.);nh.setParam("recovery/reuse_verified_plan",false);nh.setParam("recovery/min_execution_time",.2);
    std::string model=options.model;
    if(method.name.find("state_legacy")==0)model=options.state_only_model;
    if(method.name.find("state_compact")==0)model=options.compact_legacy_model;
    if(method.name.find("selected")==0)model=options.selected_model;
    if(method.name.find("geometry_bce")==0)model=options.geometry_bce;
    if(method.name.find("geometry_aux")==0)model=options.geometry_aux;
    if(method.name.find("state_bce")==0)model=options.state_bce;
    if(method.name.find("state_aux")==0)model=options.state_aux;
    nh.setParam("recovery/model_path",model);nh.setParam("recovery/learned_work_power",options.work_power);
    nh.setParam("review/profile",options.profile);
    nh.setParam("review/lazy_inference",method.name.find("lazy")!=std::string::npos);
    nh.setParam("waypoints/num",1);nh.setParam("waypoints/wp1/x",250.);nh.setParam("waypoints/wp1/y",0.);nh.setParam("waypoints/wp1/h",20.);
    int cylinders=0,spheres=0;
    for(const auto& o:scene.obstacles) {
        const std::string key=o.type=="cylinder"?"static_obstacles/obs"+std::to_string(++cylinders):"sphere_obstacles/sphere"+std::to_string(++spheres);
        nh.setParam(key+"/x",o.x);nh.setParam(key+"/y",o.y);nh.setParam(key+"/z",o.z);nh.setParam(key+"/r",o.r);
        if(o.type=="cylinder")nh.setParam(key+"/height",o.height);
    }
    nh.setParam("static_obstacles/num",cylinders);nh.setParam("sphere_obstacles/num",spheres);
}
void bind_prefix_check(MP& planner,const Options& options) {
    if(!options.prefix_check)return;
    MP* p=&planner;
    planner.review_prefix_validator=[p](const MP::s_t& s,const MP::pr_t& command) {
        EvaluationPlant::Parameters nominal;nominal.dt=.005;
        EvaluationPlant plant(nominal);
        const auto executed=plant.advance(s,command,.2);
        if(!executed.valid)return false;
        for(const auto& q:executed.states)
            if(!p->state_within_recovery_envelope(q)||p->collision_check(q.pos[0],q.pos[1],q.h))return false;
        return true;
    };
}
MP::s_t trim(const MP& p,double speed=15.5,double x=0,double y=0,double heading=0) {
    double lo=-20*M_PI/180,hi=25*M_PI/180;
    for(int i=0;i<60;++i){double a=(lo+hi)/2;double r=1.86*speed*speed*(p.C_D(a)*std::tan(a)+p.C_L(a))-9.81;if(r>0)hi=a;else lo=a;}
    MP::s_t s;s.pos[0]=x;s.pos[1]=y;s.h=20;s.V=speed;s.sigma=heading;s.attack=(lo+hi)/2;s.T=1.86*speed*speed*p.C_D(s.attack)/std::cos(s.attack);return s;
}
MP::pr_t history_control(const MP::s_t& s) { MP::pr_t c;c.attack_c=s.attack;c.T_c=s.T;c.w_roll_c=s.roll;c.t_way=3;return c; }
bool goal(const MP::s_t& s) { return std::hypot(250-s.pos[0],s.pos[1])<15&&std::abs(s.h-20)<3; }
const char* state_fields="x,y,z,V,gamma,sigma,roll,attack,T,d_sigma";
void state_record(std::ostream& out,const MP::s_t& s) { out<<s.pos[0]<<','<<s.pos[1]<<','<<s.h<<','<<s.V<<','<<s.gamma<<','<<s.sigma<<','<<s.roll<<','<<s.attack<<','<<s.T<<','<<s.d_sigma; }
const char* decision_fields="planning_ms,status,primary_feasible,verified,reused,primary_count,feasible_count,recovery_checked,backup_evaluated,primary_steps,backup_steps,cached_steps,rank,cost,reject_collision,reject_envelope,reject_known,reject_invalid,truncated,work_exhausted";
void decision_record(std::ostream& o,const MP::selection_t& d,double ms) { o<<ms<<','<<d.status<<','<<d.primary_feasible<<','<<d.recovery_verified<<','<<d.reused_verified_plan<<','<<d.primary_evaluated<<','<<d.primary_feasible_count<<','<<d.recovery_checked<<','<<d.backup_evaluated<<','<<d.primary_integration_steps<<','<<d.backup_integration_steps<<','<<d.cached_integration_steps<<','<<d.selected_cost_rank<<','<<d.cost<<','<<d.backup_rejected_collision<<','<<d.backup_rejected_envelope<<','<<d.backup_rejected_known_space<<','<<d.backup_rejected_invalid<<','<<d.backup_truncated<<','<<d.backup_work_budget_exhausted; }
MP::selection_t select(MP& p,const MP::s_t& s,double& ms) { const auto t=std::chrono::steady_clock::now();auto d=p.select_motion_primitive(s);ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count();return d; }
std::ofstream output(const std::string& path) {std::ofstream f(path);if(!f)throw std::runtime_error("Cannot open "+path);f<<std::setprecision(10);return f;}
void geometry(const Options& options,const std::vector<Scene>& maps) {
    auto f=output(options.prefix+"_scenarios.csv");f<<"scene,family,seed,low,high,type,x,y,z,r,height\n";
    for(const auto& s:maps)for(const auto& o:s.obstacles)f<<s.name<<','<<s.family<<','<<s.seed<<','<<s.low<<','<<s.high<<','<<o.type<<','<<o.x<<','<<o.y<<','<<o.z<<','<<o.r<<','<<o.height<<'\n';
}
void dataset(const Options& options,const std::vector<Scene>& maps) {
    auto rows=output(options.prefix+"_candidates.csv"),states=output(options.prefix+"_states.csv");
    states<<"state_id,scene,family,"<<state_fields<<",primary_count,feasible_count,verified_count\n";
    bool header=false;int state_id=0;size_t row_count=0;
    for(const auto& scene:maps) {
        const std::string ns="/learning/data_"+scene.name;ros::NodeHandle nh(ns);configure(nh,scene,{"oracle","cost",203},options);
        {
            MP p(nh);Generator g{scene.seed^0x9e3779b9u};std::vector<MP::s_t> initial_states;
            if(options.split=="test") {
                for(double x:{0.,25.,50.})for(double y:{-5.,0.,5.})for(double heading:{-.15,.15})
                    initial_states.push_back(trim(p,15.5,x,y,heading));
            } else for(int j=0;j<24;++j) {
                // Use separate LCG draws, preserving a language-independent order.
                const double speed=g.range(14.8,16.2),x=g.range(0,65),y=g.range(-12,12),heading=g.range(-.3,.3);
                auto s=trim(p,speed,x,y,heading);s.gamma=g.range(-.04,.04);s.roll=g.range(-.18,.18);s.attack+=g.range(-.015,.015);s.T*=g.range(.9,1.1);
                initial_states.push_back(s);
            }
            for(const auto& s:initial_states) {
                p.remember_command(history_control(s));++state_id;
                auto library=p.generate_library(s);int feasible=0,verified=0;
                for(size_t i=0;i<library.size();++i) {
                    const auto primary=p.rollout_motion_primitive(s,library[i]);
                    const float cost=p.evaluate_motion_primitive(s,library[i],primary);
                    if(!p.check_primary_feasibility(s,primary)||!std::isfinite(cost))continue;
                    MP::recovery_candidate_t candidate;candidate.terminal=primary.states.back();candidate.cost=cost;
                    const auto features=p.recovery_features(s,candidate);
                    if(features.size()!=103)throw std::runtime_error("Unexpected feature dimension");
                    const auto result=p.check_terminal_recovery(s,primary,library[i].t_way);
                    if(result.work_budget_exhausted)throw std::runtime_error("Unlimited oracle exhausted budget");
                    if(!header) {rows<<"state_id,scene,family,candidate_id,cost,verified,backup_steps,backup_evaluated";for(size_t k=0;k<features.size();++k)rows<<",f"<<k;rows<<'\n';header=true;}
                    rows<<state_id<<','<<scene.name<<','<<scene.family<<','<<i<<','<<cost<<','<<result.verified<<','<<result.integration_steps<<','<<result.checked;
                    for(float feature:features){if(!std::isfinite(feature))throw std::runtime_error("Nonfinite feature");rows<<','<<feature;}rows<<'\n';++feasible;++row_count;verified+=result.verified;
                }
                states<<state_id<<','<<scene.name<<','<<scene.family<<',';state_record(states,s);states<<','<<library.size()<<','<<feasible<<','<<verified<<'\n';
            }
        }
        ros::param::del(ns);rows.flush();states.flush();std::cout<<"dataset "<<scene.name<<" states="<<state_id<<" candidates="<<row_count<<std::endl;
    }
}
std::vector<Method> matched_methods(const Options& options) {
    std::vector<Method> methods;
    for(bool work:{false,true}) {
        const int cap=work?203:12,steps=work?3600:0;
        const std::string suffix=work?"_w3600":"_k12";
        methods.push_back({"cost"+suffix,"cost",cap,steps});
        methods.push_back({"diverse"+suffix,"diverse",cap,steps});
        methods.push_back({"geometry_eager"+suffix,"learned",cap,steps});
        methods.push_back({"geometry_lazy"+suffix,"learned",cap,steps});
        methods.push_back({"state_legacy_eager"+suffix,"learned",cap,steps});
        methods.push_back({"state_compact_eager"+suffix,"learned",cap,steps});
        methods.push_back({"state_compact_lazy"+suffix,"learned",cap,steps});
        if(!options.selected_model.empty()) {
            methods.push_back({"selected_eager"+suffix,"learned",cap,steps});
            methods.push_back({"selected_lazy"+suffix,"learned",cap,steps});
        }
        if(!options.geometry_bce.empty()) {
            for(const std::string name:{"geometry_bce_lazy","geometry_aux_lazy","state_bce_lazy","state_aux_lazy"})
                methods.push_back({name+suffix,"learned",cap,steps});
        }
    }
    methods.push_back({"full203","cost",203});
    return methods;
}
const char* profile_fields="primary_ms,primary_integration_ms,primary_other_ms,prefix_ms,prefix_checked,prefix_rejected,scalar_feature_ms,geometry_feature_ms,inference_ms,schedule_ms,continuation_ms,other_ms,profile_total_ms,feature_candidates";
void profile_record(std::ostream& o,const MP::review_profile_t& p) {
    const double feature=p.scalar_feature_ms+p.geometry_feature_ms+p.inference_ms;
    const double schedule=p.schedule_inclusive_ms-feature;
    const double other=p.total_ms-p.primary_ms-p.schedule_inclusive_ms-p.continuation_ms;
    o<<p.primary_ms<<','<<p.primary_integration_ms<<','<<(p.primary_ms-p.primary_integration_ms-p.prefix_ms)<<','<<p.prefix_ms<<','<<p.prefix_checked<<','<<p.prefix_rejected<<','<<p.scalar_feature_ms<<','<<p.geometry_feature_ms<<','<<p.inference_ms<<','<<schedule<<','<<p.continuation_ms<<','<<other<<','<<p.total_ms<<','<<p.feature_candidates;
}

void matched(const Options& options,const std::vector<Scene>& maps) {
    const auto methods=matched_methods(options);auto f=output(options.prefix+"_matched.csv");f<<"state_id,scene,family,method,repeat,"<<state_fields<<','<<decision_fields<<','<<profile_fields<<'\n';int id=0;
    for(const auto& scene:maps) {
        std::vector<std::unique_ptr<MP>> planners;std::vector<std::string> namespaces;
        for(const auto& method:methods){const std::string ns="/learning/matched_"+scene.name+method.name;namespaces.push_back(ns);ros::NodeHandle nh(ns);configure(nh,scene,method,options);planners.emplace_back(new MP(nh));bind_prefix_check(*planners.back(),options);}
        for(double x:{0.,25.,50.})for(double y:{-5.,0.,5.})for(double heading:{-.15,.15}) {
            ++id;auto s=trim(*planners.front(),15.5,x,y,heading);auto history=history_control(s);
            for(int repeat=0;repeat<options.repeats;++repeat)for(size_t j=0;j<methods.size();++j) {
                const size_t i=(j+repeat+id)%methods.size();auto& p=*planners[i];p.remember_command(history);double ms;auto d=select(p,s,ms);
                f<<id<<','<<scene.name<<','<<scene.family<<','<<methods[i].name<<','<<repeat<<',';state_record(f,s);f<<',';decision_record(f,d,ms);f<<',';profile_record(f,p.review_profile);f<<'\n';
            }
        }
        planners.clear();for(const auto& ns:namespaces)ros::param::del(ns);f.flush();std::cout<<"matched "<<scene.name<<" states="<<id<<std::endl;
    }
}
void episodes(const Options& options,const std::vector<Scene>& maps) {
    std::vector<Method> methods={{"cost12","cost",12},{"diverse12","diverse",12},{"geometry_eager12","learned",12},{"state_legacy_eager12","learned",12},{"geometry_bce_lazy12","learned",12},{"geometry_aux_lazy12","learned",12},{"state_bce_lazy12","learned",12},{"state_aux_lazy12","learned",12},{"full203","cost",203}};
    if(options.selection_only)methods.erase(std::remove_if(methods.begin(),methods.end(),[](const Method& m){return m.name!="geometry_bce_lazy12"&&m.name!="geometry_aux_lazy12"&&m.name!="state_bce_lazy12"&&m.name!="state_aux_lazy12";}),methods.end());
    std::vector<Condition> conditions(4);conditions[0].name="nominal";conditions[1].name="model_mismatch";
    conditions[1].parameters.actuator_scale=1.25;conditions[1].parameters.lift_scale=.97;conditions[1].parameters.drag_scale=1.10;
    conditions[2].name="crosswind";conditions[2].parameters.wind_y=1.;conditions[3].name="delay100ms";conditions[3].delay=.1;
    auto ef=output(options.prefix+"_episodes.csv"),plans=output(options.prefix+"_plans.csv"),paths=output(options.prefix+"_paths.csv");
    ef<<"case_id,scene,family,condition,method,outcome,steps,sim_seconds,goal_distance,min_obstacle_clearance\n";
    plans<<"case_id,step,"<<decision_fields<<'\n';paths<<"case_id,step,time,"<<state_fields<<'\n';int id=0;
    for(const auto& scene:maps)for(const auto& condition:conditions)for(const auto& method:methods) {
        const std::string case_id=options.split+"_"+std::to_string(++id),ns="/learning/episode_"+case_id;ros::NodeHandle nh(ns);configure(nh,scene,method,options);
        {
            MP p(nh);bind_prefix_check(p,options);auto s=trim(p);auto previous=history_control(s);EvaluationPlant plant(condition.parameters);
            std::string outcome="timeout";int steps=0;double clearance=1e9;
            auto record=[&](){paths<<case_id<<','<<steps<<','<<steps*.2<<',';state_record(paths,s);paths<<'\n';};
            auto inspect=[&](const EvaluationPlant::Result& segment) {
                if(!segment.valid){outcome="invalid_dynamics";return false;}
                for(const auto& q:segment.states) {
                    for(const auto& o:scene.obstacles) {
                        double c;if(o.type=="sphere")c=std::sqrt(std::pow(q.pos[0]-o.x,2)+std::pow(q.pos[1]-o.y,2)+std::pow(q.h-o.z,2))-o.r;
                        else {double radial=std::hypot(q.pos[0]-o.x,q.pos[1]-o.y)-o.r;double axial=std::abs(q.h-o.z)-o.height/2;c=std::hypot(std::max(radial,0.),std::max(axial,0.))+std::min(std::max(radial,axial),0.);}
                        clearance=std::min(clearance,c);
                    }
                    if(!p.state_within_recovery_envelope(q)){outcome="envelope_violation";return false;}
                    if(p.collision_check(q.pos[0],q.pos[1],q.h)){outcome="collision";return false;}
                }
                return true;
            };
            for(int step=0;step<200;++step) {
                record();if(goal(s)){outcome="goal";break;}double ms;auto d=select(p,s,ms);plans<<case_id<<','<<step<<',';decision_record(plans,d,ms);plans<<'\n';
                if(!d.primary_feasible){outcome="no_feasible_primary";break;}p.accept_selection(d);bool ok=true;
                if(condition.delay>0){auto r=plant.advance(s,previous,condition.delay);s=r.states.back();ok=inspect(r);}
                if(ok){auto r=plant.advance(s,d.command,.2-condition.delay);s=r.states.back();ok=inspect(r);}
                previous=d.command;p.advance_execution(.2);++steps;if(!ok)break;if(goal(s)){outcome="goal";break;}
            }
            record();ef<<case_id<<','<<scene.name<<','<<scene.family<<','<<condition.name<<','<<method.name<<','<<outcome<<','<<steps<<','<<steps*.2<<','<<std::hypot(250-s.pos[0],s.pos[1])<<','<<clearance<<'\n';
            std::cout<<id<<'/'<<maps.size()*conditions.size()*methods.size()<<' '<<scene.name<<' '<<condition.name<<' '<<method.name<<' '<<outcome<<std::endl;
        }
        ros::param::del(ns);ef.flush();plans.flush();paths.flush();
    }
}
int main(int argc,char** argv) {
    ros::init(argc,argv,"recovery_review_benchmark");ros::NodeHandle args("~");Options o;
    args.param<std::string>("split",o.split,"train");args.param<std::string>("phase",o.phase,"dataset");args.param<std::string>("output_prefix",o.prefix,"/tmp/recovery_learning");args.param<std::string>("model_path",o.model,"");
    args.param<std::string>("state_only_model_path",o.state_only_model,"");
    args.param<std::string>("compact_legacy_model_path",o.compact_legacy_model,"");
    args.param<std::string>("selected_model_path",o.selected_model,"");
    args.param<std::string>("geometry_bce_model_path",o.geometry_bce,"");
    args.param<std::string>("geometry_aux_model_path",o.geometry_aux,"");
    args.param<std::string>("state_bce_model_path",o.state_bce,"");
    args.param<std::string>("state_aux_model_path",o.state_aux,"");
    args.param<bool>("selection_only",o.selection_only,false);
    args.param<bool>("prefix_check",o.prefix_check,false);
    args.param<int>("repeats",o.repeats,3);args.param<bool>("profile",o.profile,false);
    args.param<int>("maps_per_family",o.maps_per_family,0);args.param<double>("work_power",o.work_power,1.);
    if(o.split!="train"&&o.split!="val"&&o.split!="test"&&o.split!="review")return 2;
    if(o.phase!="matched"&&o.phase!="episodes"&&o.phase!="all")return 2;
    if(o.repeats<1||o.repeats>10)return 2;
    if(o.selection_only&&(o.split!="val"||o.phase!="episodes"||o.prefix_check))return 2;
    if((o.phase=="episodes"||o.phase=="all")&&o.selected_model.empty())return 2;
    if(o.maps_per_family<0||o.maps_per_family>1000)return 2;
    if(o.phase!="dataset") {
        LearnedRecoveryNetwork model;
        if(!model.load(o.model)){std::cerr<<"Evaluation requires a valid exported model_path\n";return 2;}
        if(!o.state_only_model.empty()&&!model.load(o.state_only_model)){std::cerr<<"Invalid state_only_model_path\n";return 2;}
        if(o.compact_legacy_model.empty()||!model.load(o.compact_legacy_model)){std::cerr<<"Invalid compact_legacy_model_path\n";return 2;}
        if(!o.selected_model.empty()&&!model.load(o.selected_model)){std::cerr<<"Invalid selected_model_path\n";return 2;}
        if(!o.geometry_bce.empty())for(const auto& path:{o.geometry_bce,o.geometry_aux,o.state_bce,o.state_aux})
            if(!model.load(path)){std::cerr<<"Invalid factorial model path\n";return 2;}
        if((o.phase=="episodes"||o.phase=="all")&&o.geometry_bce.empty())return 2;
        if(!std::isfinite(o.work_power)||o.work_power<0||o.work_power>4){std::cerr<<"work_power must lie in [0,4]\n";return 2;}
    }
    try {
        auto maps=scenes(o);geometry(o,maps);
        if(o.phase=="dataset")dataset(o,maps);
        if(o.phase=="matched"||o.phase=="all")matched(o,maps);
        if(o.phase=="episodes"||o.phase=="all")episodes(o,maps);
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
    return 0;
}
