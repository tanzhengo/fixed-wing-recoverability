#include "motion_primitives.h"
#include "evaluation_plant.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <cstdint>
using MP=MotionPrimitives;
struct Obstacle {std::string type;double x,y,z,r,height;};
struct Scene {std::string name,family;double low=16,high=24;std::vector<Obstacle> obstacles;};
struct Method {std::string name,order;int candidates=12,work=0;double horizon=3;bool terminal=true,cache=false;};
struct Condition {std::string name;EvaluationPlant::Parameters parameters;double delay=0;};
struct Generator {
    uint32_t state;
    double range(double a,double b){state=1664525u*state+1013904223u;return a+(b-a)*(double(state)/4294967296.);}
};
std::vector<Scene> scenes(){
    std::vector<Scene> out;
    for(int family=0;family<4;++family)for(int seed=0;seed<6;++seed){
        const std::string names[]={"staggered","wall","culdesac","spatial"};
        Scene s;s.family=names[family];s.name=s.family+"_"+std::to_string(seed);
        Generator g{uint32_t(20261030+family*100+seed)};
        auto cylinder=[&](double x,double y,double r,double z=20,double h=100){s.obstacles.push_back({"cylinder",x,y,z,r,h});};
        if(family==0){for(int i=0;i<4;++i)cylinder(65+i*43+g.range(-8,8),(i%2? -1:1)*g.range(4,22),g.range(8,13));}
        if(family==1){double x=g.range(75,110),cy=g.range(-10,10);for(int i=-2;i<=2;++i)cylinder(x,cy+i*18,g.range(6.5,9));cylinder(175,g.range(-30,30),12);}
        if(family==2){double x=g.range(65,90),w=g.range(27,36);for(int i=0;i<4;++i){cylinder(x+i*20,-w,8);cylinder(x+i*20,w,8);}for(int i=-1;i<=1;++i)cylinder(x+70,i*17,8);}
        if(family==3){s.low=5;s.high=45;for(int i=0;i<3;++i){double h=g.range(18,33);cylinder(65+i*63,g.range(-15,15),g.range(8,13),h/2,h);}for(int i=0;i<2;++i)s.obstacles.push_back({"sphere",105.+i*75,g.range(-15,15),g.range(20,34),g.range(8,12),0});}
        out.push_back(s);
    }return out;
}
void configure(ros::NodeHandle& nh,const Scene& scene,const Method& method,double speed=15.5,double radius=0){
    ros::param::del(nh.getNamespace());
    nh.setParam("motion_primitive/horizon_time",method.horizon);nh.setParam("motion_primitive/V",speed);
    nh.setParam("motion_primitive/num",29);nh.setParam("motion_primitive/vertical_num",7);
    nh.setParam("motion_primitive/min_altitude",scene.low);nh.setParam("motion_primitive/max_altitude",scene.high);
    nh.setParam("recovery/enabled",true);nh.setParam("recovery/require_terminal",method.terminal);
    nh.setParam("recovery/horizon_time",6.);nh.setParam("recovery/max_candidates",method.candidates);
    nh.setParam("recovery/candidate_order",method.order);nh.setParam("recovery/max_backup_integration_steps",method.work);
    nh.setParam("recovery/known_space_radius",radius);nh.setParam("recovery/reuse_verified_plan",method.cache);
    nh.setParam("recovery/min_execution_time",.2);
    nh.setParam("waypoints/num",1);nh.setParam("waypoints/wp1/x",250.);nh.setParam("waypoints/wp1/y",0.);nh.setParam("waypoints/wp1/h",20.);
    int cylinders=0,spheres=0;
    for(const auto& o:scene.obstacles){std::string key;
        if(o.type=="cylinder")key="static_obstacles/obs"+std::to_string(++cylinders);
        else key="sphere_obstacles/sphere"+std::to_string(++spheres);
        nh.setParam(key+"/x",o.x);nh.setParam(key+"/y",o.y);nh.setParam(key+"/z",o.z);nh.setParam(key+"/r",o.r);
        if(o.type=="cylinder")nh.setParam(key+"/height",o.height);
    }
    nh.setParam("static_obstacles/num",cylinders);nh.setParam("sphere_obstacles/num",spheres);
}
MP::s_t trim(const MP& p,double speed,double x=0,double y=0,double heading=0){
    double lo=-20*M_PI/180,hi=25*M_PI/180;
    for(int i=0;i<60;++i){double a=(lo+hi)/2;double r=1.86*speed*speed*(p.C_D(a)*std::tan(a)+p.C_L(a))-9.81;if(r>0)hi=a;else lo=a;}
    MP::s_t s;s.pos[0]=x;s.pos[1]=y;s.h=20;s.V=speed;s.sigma=heading;s.attack=(lo+hi)/2;s.T=1.86*speed*speed*p.C_D(s.attack)/std::cos(s.attack);return s;
}
bool goal(const MP::s_t& s){return std::hypot(250-s.pos[0],s.pos[1])<15&&std::abs(s.h-20)<3;}
MP::pr_t trim_control(const MP::s_t& s){MP::pr_t c;c.attack_c=s.attack;c.T_c=s.T;c.t_way=3;return c;}
const char* state_fields="x,y,z,V,gamma,sigma,roll,attack,T,d_sigma";
void state_record(std::ostream& out,const MP::s_t& s){out<<s.pos[0]<<','<<s.pos[1]<<','<<s.h<<','<<s.V<<','<<s.gamma<<','<<s.sigma<<','<<s.roll<<','<<s.attack<<','<<s.T<<','<<s.d_sigma;}
const char* decision_fields="planning_ms,status,primary_feasible,verified,reused,primary_count,feasible_count,recovery_checked,backup_evaluated,primary_steps,backup_steps,cached_steps,rank,cost,reject_collision,reject_envelope,reject_known,reject_invalid,truncated,work_exhausted";
void decision_record(std::ostream& o,const MP::selection_t& d,double ms){o<<ms<<','<<d.status<<','<<d.primary_feasible<<','<<d.recovery_verified<<','<<d.reused_verified_plan<<','<<d.primary_evaluated<<','<<d.primary_feasible_count<<','<<d.recovery_checked<<','<<d.backup_evaluated<<','<<d.primary_integration_steps<<','<<d.backup_integration_steps<<','<<d.cached_integration_steps<<','<<d.selected_cost_rank<<','<<d.cost<<','<<d.backup_rejected_collision<<','<<d.backup_rejected_envelope<<','<<d.backup_rejected_known_space<<','<<d.backup_rejected_invalid<<','<<d.backup_truncated<<','<<d.backup_work_budget_exhausted;}
MP::selection_t select(MP& p,const MP::s_t& s,double& ms){const auto t=std::chrono::steady_clock::now();auto d=p.select_motion_primitive(s);ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count();return d;}
int main(int argc,char** argv){
    ros::init(argc,argv,"recovery_benchmark_v3");ros::NodeHandle args("~");std::string prefix,phase;
    args.param<std::string>("output_prefix",prefix,"/tmp/recovery_v3");args.param<std::string>("phase",phase,"all");
    if(phase!="all"&&phase!="episodes"&&phase!="matched"&&phase!="diagnostic")return 2;
    auto maps=scenes();std::ofstream geometry(prefix+"_scenarios.csv");geometry<<std::setprecision(10)<<"scene,family,low,high,type,x,y,z,r,height\n";
    for(const auto& s:maps)for(const auto& o:s.obstacles)geometry<<s.name<<','<<s.family<<','<<s.low<<','<<s.high<<','<<o.type<<','<<o.x<<','<<o.y<<','<<o.z<<','<<o.r<<','<<o.height<<'\n';
    const std::vector<Method> methods={{"primary3","cost",12,0,3,false},{"primary6","cost",12,0,6,false},{"cost12","cost",12},{"diverse12","diverse",12},{"full203","cost",203}};
    if(phase=="all"||phase=="episodes"){
        std::vector<Condition> conditions(4);conditions[0].name="nominal";conditions[1].name="model_mismatch";
        conditions[1].parameters.actuator_scale=1.25;conditions[1].parameters.lift_scale=.97;conditions[1].parameters.drag_scale=1.10;
        conditions[2].name="crosswind";conditions[2].parameters.wind_y=1.;conditions[3].name="delay100ms";conditions[3].delay=.1;
        std::ofstream episodes(prefix+"_episodes.csv"),plans(prefix+"_plans.csv"),paths(prefix+"_paths.csv");
        episodes<<std::setprecision(10)<<"case_id,scene,family,condition,method,outcome,steps,sim_seconds,goal_distance,min_obstacle_clearance\n";
        plans<<std::setprecision(10)<<"case_id,step,"<<decision_fields<<'\n';paths<<std::setprecision(10)<<"case_id,step,time,"<<state_fields<<'\n';int id=0;
        for(const auto& scene:maps)for(const auto& condition:conditions)for(const auto& method:methods){
            std::string caseid="v3_"+std::to_string(++id),ns="/v3/"+caseid;ros::NodeHandle nh(ns);configure(nh,scene,method);
            {MP p(nh);auto s=trim(p,15.5);auto previous=trim_control(s);EvaluationPlant plant(condition.parameters);
                std::string outcome="timeout";int steps=0;double clearance=1e9;
                auto record=[&](){paths<<caseid<<','<<steps<<','<<steps*.2<<',';state_record(paths,s);paths<<'\n';};
                auto inspect=[&](const EvaluationPlant::Result& segment){
                    if(!segment.valid){outcome="invalid_dynamics";return false;}
                    for(const auto& q:segment.states){
                        for(const auto& o:scene.obstacles){double c;
                            if(o.type=="sphere")c=std::sqrt(std::pow(q.pos[0]-o.x,2)+std::pow(q.pos[1]-o.y,2)+std::pow(q.h-o.z,2))-o.r;
                            else {double radial=std::hypot(q.pos[0]-o.x,q.pos[1]-o.y)-o.r;double axial=std::abs(q.h-o.z)-o.height/2;c=std::hypot(std::max(radial,0.),std::max(axial,0.))+std::min(std::max(radial,axial),0.);}
                            clearance=std::min(clearance,c);
                        }
                        if(!p.state_within_recovery_envelope(q)){outcome="envelope_violation";return false;}
                        if(p.collision_check(q.pos[0],q.pos[1],q.h)){outcome="collision";return false;}
                    }return true;
                };
                for(int step=0;step<200;++step){record();if(goal(s)){outcome="goal";break;}double ms;auto d=select(p,s,ms);plans<<caseid<<','<<step<<',';decision_record(plans,d,ms);plans<<'\n';
                    if(!d.primary_feasible){outcome="no_feasible_primary";break;}p.accept_selection(d);
                    bool ok=true;
                    if(condition.delay>0){auto r=plant.advance(s,previous,condition.delay);s=r.states.back();ok=inspect(r);}
                    if(ok){auto r=plant.advance(s,d.command,.2-condition.delay);s=r.states.back();ok=inspect(r);}
                    previous=d.command;p.advance_execution(.2);++steps;if(!ok)break;if(goal(s)){outcome="goal";break;}
                }
                record();episodes<<caseid<<','<<scene.name<<','<<scene.family<<','<<condition.name<<','<<method.name<<','<<outcome<<','<<steps<<','<<steps*.2<<','<<std::hypot(250-s.pos[0],s.pos[1])<<','<<clearance<<'\n';
                std::cout<<id<<"/480 "<<scene.name<<' '<<condition.name<<' '<<method.name<<' '<<outcome<<std::endl;
            }ros::param::del(ns);
        }
    }
    if(phase=="all"||phase=="matched"){
        std::vector<Method> mm={{"cost12","cost",12},{"diverse12","diverse",12},{"full203","cost",203},{"cost_w3600","cost",203,3600},{"diverse_w3600","diverse",203,3600},{"cost_w10800","cost",203,10800},{"diverse_w10800","diverse",203,10800}};
        std::ofstream matched(prefix+"_matched.csv");matched<<std::setprecision(10)<<"state_id,scene,family,method,repeat,"<<state_fields<<','<<decision_fields<<'\n';int id=0;
        for(const auto& scene:maps){std::vector<std::unique_ptr<MP>> planners;std::vector<std::string> namespaces;
            for(const auto& m:mm){auto ns="/v3/matched_"+scene.name+m.name;namespaces.push_back(ns);ros::NodeHandle nh(ns);configure(nh,scene,m);planners.emplace_back(new MP(nh));}
            for(double x:{0.,25.,50.})for(double y:{-5.,0.,5.})for(double heading:{-.15,.15}){++id;auto s=trim(*planners.front(),15.5,x,y,heading);auto history=trim_control(s);
                for(int repeat=0;repeat<2;++repeat)for(int j=0;j<int(mm.size());++j){int i=(j+repeat+id)%mm.size();auto& p=*planners[i];p.remember_command(history);double ms;auto d=select(p,s,ms);matched<<id<<','<<scene.name<<','<<scene.family<<','<<mm[i].name<<','<<repeat<<',';state_record(matched,s);matched<<',';decision_record(matched,d,ms);matched<<'\n';}
            }planners.clear();for(const auto& ns:namespaces)ros::param::del(ns);std::cout<<"matched "<<scene.name<<' '<<id<<"/432"<<std::endl;
        }
    }
    if(phase=="all"||phase=="diagnostic"){
        Scene scene;scene.name="v2_mirrored";scene.family="diagnostic";scene.obstacles={{"cylinder",95,-8,20,14,100},{"cylinder",150,20,20,10,100},{"cylinder",195,-18,20,10,100}};
        std::vector<Method> dm={{"cache120","diverse",12,0,3,true,true},{"diverse120","diverse",12},{"full120","cost",203},{"diverse_unlimited","diverse",12},{"full_unlimited","cost",203}};
        std::vector<std::unique_ptr<MP>> planners;std::vector<std::string> namespaces;
        for(size_t i=0;i<dm.size();++i){auto ns="/v3/diagnostic_"+dm[i].name;namespaces.push_back(ns);ros::NodeHandle nh(ns);configure(nh,scene,dm[i],17,i<3?120:0);planners.emplace_back(new MP(nh));}
        auto s=trim(*planners[0],17);auto previous=trim_control(s);
        std::ofstream diagnostic(prefix+"_diagnostic.csv");diagnostic<<std::setprecision(10)<<"step,time,method,"<<state_fields<<','<<decision_fields<<'\n';
        int diagnostic_steps=0;std::string diagnostic_outcome="timeout";
        for(int step=0;step<250&&!goal(s);++step){MP::selection_t chosen;for(size_t i=0;i<planners.size();++i){if(i&&step>0)planners[i]->remember_command(previous);double ms;auto d=select(*planners[i],s,ms);diagnostic<<step<<','<<step*.2<<','<<dm[i].name<<',';state_record(diagnostic,s);diagnostic<<',';decision_record(diagnostic,d,ms);diagnostic<<'\n';if(i==0)chosen=d;}
            if(!chosen.primary_feasible){diagnostic_outcome="no_feasible_primary";break;}planners[0]->accept_selection(chosen);previous=chosen.command;auto c=chosen.command;c.t_way=.2;auto plant=planners[0]->rollout_motion_primitive(s,c);s=plant.states.back();planners[0]->advance_execution(.2);++diagnostic_steps;if(!plant.valid){diagnostic_outcome="invalid_dynamics";break;}if(plant.collision){diagnostic_outcome="collision";break;}
        }
        if(goal(s))diagnostic_outcome="goal";
        std::ofstream summary(prefix+"_diagnostic_summary.csv");summary<<std::setprecision(10)<<"outcome,steps,time,goal_distance,"<<state_fields<<"\n"<<diagnostic_outcome<<','<<diagnostic_steps<<','<<diagnostic_steps*.2<<','<<std::hypot(250-s.pos[0],s.pos[1])<<',';state_record(summary,s);summary<<'\n';
        std::cout<<"diagnostic final distance "<<std::hypot(250-s.pos[0],s.pos[1])<<" goal "<<goal(s)<<std::endl;
    }
    return 0;
}
