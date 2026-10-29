// Deterministic same-model diagnostics. No claim of independent dynamics,
// sensor realism, recursive safety, or statistical generalization.
#include "motion_primitives.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

using MP = MotionPrimitives;
struct Cylinder { double x, y, r; };
struct Scene { std::string name, split; std::vector<Cylinder> obstacles; };
struct Variant { std::string name, order; bool enabled; int budget; double primary, backup; };

std::vector<Scene> scenes() {
    std::vector<Cylinder> dead = {{45,-25,8},{65,-25,8},{85,-25,8},{105,-25,8},
        {45,25,8},{65,25,8},{85,25,8},{105,25,8},{115,-16,8},{115,0,8},{115,16,8}};
    auto wider = dead;
    for (auto& o : wider) o.x += 30;
    return {{"open","development",{}},
        {"offset_pillars","development",{{75,5,12},{125,-18,12},{175,12,12}}},
        {"late_wall","development",{{80,-24,12},{80,0,12},{80,24,12}}},
        {"cul_de_sac","development",dead},
        {"wide_entry_cul_de_sac","validation",wider},
        {"mirrored_pillars","validation",{{95,-8,14},{150,20,10},{195,-18,10}}}};
}

void configure(ros::NodeHandle& nh, const Scene& scene, const Variant& v,
               double speed, double radius) {
    // A stopped/restarted experiment must not inherit a previous case's params.
    ros::param::del(nh.getNamespace());
    nh.setParam("motion_primitive/horizon_time",v.primary);
    nh.setParam("motion_primitive/V",speed);
    nh.setParam("motion_primitive/num",29);
    nh.setParam("motion_primitive/vertical_num",7);
    nh.setParam("motion_primitive/min_altitude",16.0);
    nh.setParam("motion_primitive/max_altitude",24.0);
    nh.setParam("recovery/enabled",v.enabled);
    nh.setParam("recovery/horizon_time",v.backup);
    nh.setParam("recovery/max_candidates",v.budget);
    nh.setParam("recovery/candidate_order",v.order);
    nh.setParam("recovery/known_space_radius",radius);
    nh.setParam("recovery/reuse_verified_plan",v.name=="diverse12_h6_cache");
    nh.setParam("recovery/min_execution_time",0.2);
    nh.setParam("waypoints/num",1);
    nh.setParam("waypoints/wp1/x",250.0);
    nh.setParam("waypoints/wp1/y",0.0);
    nh.setParam("waypoints/wp1/h",20.0);
    nh.setParam("static_obstacles/num",static_cast<int>(scene.obstacles.size()));
    for (size_t i=0;i<scene.obstacles.size();++i) {
        const auto& o=scene.obstacles[i];
        const auto key="static_obstacles/obs"+std::to_string(i+1);
        nh.setParam(key+"/x",o.x); nh.setParam(key+"/y",o.y);
        nh.setParam(key+"/r",o.r); nh.setParam(key+"/z",20.0);
        nh.setParam(key+"/height",100.0);
    }
}

// Solve the exact level-flight equilibrium for the benchmark's default model.
// Changing airspeed must change attack and thrust, not just the velocity field.
MP::s_t trimmed_state(const MP& planner,double speed,double x,double y) {
    double lo=-20*M_PI/180.0,hi=25*M_PI/180.0;
    auto residual=[&](double a) { return 1.86*speed*speed*
        (planner.C_D(a)*std::tan(a)+planner.C_L(a))-9.81; };
    if (residual(lo)>0 || residual(hi)<0) throw std::runtime_error("No trim in attack limits");
    for (int i=0;i<50;++i) {
        const double mid=(lo+hi)/2;
        if (residual(mid)>0) hi=mid; else lo=mid;
    }
    MP::s_t s; s.V=speed; s.h=20; s.pos[0]=x; s.pos[1]=y;
    s.attack=(lo+hi)/2;
    s.T=1.86*speed*speed*planner.C_D(s.attack)/std::cos(s.attack);
    return s;
}

bool goal(const MP::s_t& s) {
    return std::hypot(250.0-s.pos[0],s.pos[1])<15 && std::abs(s.h-20)<3;
}

int main(int argc,char** argv) {
    ros::init(argc,argv,"recovery_benchmark_v2");
    ros::NodeHandle args("~");
    std::string prefix,phase;
    args.param<std::string>("output_prefix",prefix,"/tmp/recovery_v2");
    args.param<std::string>("phase",phase,"all");
    if (phase!="all" && phase!="episodes" && phase!="matched") {
        std::cerr<<"phase must be all, episodes, or matched\n";return 2;
    }
    const auto maps=scenes();
    std::ofstream geometry(prefix+"_scenarios.csv");
    geometry<<"scenario,split,x,y,r,inflated_r,z,height\n";
    for (const auto& scene:maps) for(const auto& o:scene.obstacles)
        geometry<<scene.name<<','<<scene.split<<','<<o.x<<','<<o.y<<','<<o.r<<','<<o.r*1.2<<",20,100\n";
    if(!geometry) return 2;
    std::vector<Variant> variants={{"baseline","cost",false,12,3,3},{"long6","cost",false,12,6,3}};
    for(double h:{3.,6.}) {
        const auto suffix= h==3 ? "_h3" : "_h6";
        variants.push_back({std::string("cost12")+suffix,"cost",true,12,3,h});
        variants.push_back({std::string("cost203")+suffix,"cost",true,203,3,h});
        variants.push_back({std::string("diverse12")+suffix,"diverse",true,12,3,h});
    }
    variants.push_back({"diverse12_h6_cache","diverse",true,12,3,6});
    int run=0;
    if(phase!="matched") {
        std::ofstream episodes(prefix+"_episodes.csv"),plans(prefix+"_plans.csv"),paths(prefix+"_paths.csv");
        if(!episodes||!plans||!paths) return 2;
        episodes<<"case_id,split,scenario,variant,primary_horizon,backup_horizon,max_candidates,speed,known_space_radius,initial_y,outcome,steps,sim_seconds,goal_distance\n";
        plans<<"case_id,step,planning_ms,status,primary_evaluated,primary_feasible_count,recovery_checked,backup_evaluated,verified,budget_exhausted,primary_integration_steps,backup_integration_steps,selected_cost_rank,reused_verified_plan,cached_integration_steps\n";
        paths<<"case_id,step,time,x,y,z\n";
        auto episode=[&](const Scene& scene,const Variant& v,double speed,double radius,double y,const std::string& split) {
            const auto case_id="episode_"+std::to_string(++run);
            const auto ns="/recovery_v2/"+case_id;
            ros::NodeHandle nh(ns); configure(nh,scene,v,speed,radius);
            {
                MP planner(nh); auto s=trimmed_state(planner,speed,0,y);
                // Maintain original initial condition for direct v1 comparability.
                if(speed==15.5) {s.attack=-0.20;s.T=0.93;}
                std::string outcome="timeout"; int executed=0;
                auto record=[&](){paths<<case_id<<','<<executed<<','<<executed*.2<<','<<s.pos[0]<<','<<s.pos[1]<<','<<s.h<<'\n';};
                for(int step=0;step<150;++step) {
                    record(); if(goal(s)){outcome="goal";break;}
                    const auto begin=std::chrono::steady_clock::now();
                    auto d=planner.select_motion_primitive(s);
                    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
                    plans<<case_id<<','<<step<<','<<ms<<','<<d.status<<','<<d.primary_evaluated<<','<<d.primary_feasible_count
                         <<','<<d.recovery_checked<<','<<d.backup_evaluated<<','<<d.recovery_verified<<','<<d.budget_exhausted
                         <<','<<d.primary_integration_steps<<','<<d.backup_integration_steps<<','<<d.selected_cost_rank
                         <<','<<d.reused_verified_plan<<','<<d.cached_integration_steps<<'\n';
                    if(!d.primary_feasible){outcome="no_feasible_primary";break;}
                    auto command=d.command; planner.accept_selection(d);command.t_way=.2;
                    const auto plant=planner.rollout_motion_primitive(s,command);
                    s=plant.states.back();++executed;planner.advance_execution(.2);
                    if(!plant.valid){outcome="invalid_dynamics";break;}
                    if(plant.collision){outcome="collision";break;}
                    if(goal(s)){outcome="goal";break;}
                }
                record();
                episodes<<case_id<<','<<split<<','<<scene.name<<','<<v.name<<','<<v.primary<<','<<v.backup<<','<<v.budget
                        <<','<<speed<<','<<radius<<','<<y<<','<<outcome<<','<<executed<<','<<executed*.2<<','
                        <<std::hypot(250.0-s.pos[0],s.pos[1])<<'\n';
                std::cout<<run<<"/194 "<<split<<' '<<scene.name<<' '<<v.name<<" V="<<speed<<" R="<<radius<<" -> "<<outcome<<std::endl;
            }
            ros::param::del(ns);
        };
        for(const auto& scene:maps) for(const auto& v:variants) for(double y:{-2.,0.,2.})
            episode(scene,v,15.5,0,y,scene.split);
        for(const auto& scene:maps) if(scene.split=="validation")
            for(double speed:{14.,17.}) for(double radius:{0.,120.})
                for(const auto& v:variants) if(v.enabled && v.backup==6)
                    episode(scene,v,speed,radius,0,"robustness");
    }
    if(phase!="episodes") {
        std::ofstream matched(prefix+"_matched.csv");
        if(!matched) return 2;
        matched<<"state_id,scenario,split,variant,backup_horizon,speed,known_space_radius,repeat,planning_ms,verified,status,recovery_checked,backup_evaluated,cost,selected_cost_rank,primary_integration_steps,backup_integration_steps\n";
        int state_id=0;
        for(const auto& scene:maps) for(double h:{3.,6.}) {
            const std::vector<Variant> methods={{"cost12","cost",true,12,3,h},{"cost203","cost",true,203,3,h},{"diverse12","diverse",true,12,3,h}};
            std::vector<std::unique_ptr<MP>> planners;std::vector<std::string> namespaces;
            for(const auto& v:methods){
                const auto ns="/recovery_v2/matched_"+scene.name+v.name+std::to_string(int(h));
                namespaces.push_back(ns);ros::NodeHandle nh(ns);configure(nh,scene,v,15.5,0);
                planners.emplace_back(new MP(nh));
            }
            for(double x:{0.,20.,40.}) for(double y:{-2.,0.,2.}) {
                auto s=trimmed_state(*planners.front(),15.5,x,y);
                MP::pr_t history;history.attack_c=s.attack;history.T_c=s.T;history.t_way=3;
                const auto id="state_"+std::to_string(++state_id);
                // Same full state, goal, map and previous command for all methods.
                // Rotate execution order to avoid a fixed first-method timing bias.
                for(int repeat=0;repeat<3;++repeat) for(int j=0;j<3;++j) {
                    const int i=(j+repeat+state_id)%3;auto& p=*planners[i];p.remember_command(history);
                    const auto begin=std::chrono::steady_clock::now();auto d=p.select_motion_primitive(s);
                    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
                    matched<<id<<','<<scene.name<<','<<scene.split<<','<<methods[i].name<<','<<h<<",15.5,0,"<<repeat
                           <<','<<ms<<','<<d.recovery_verified<<','<<d.status<<','<<d.recovery_checked<<','<<d.backup_evaluated
                           <<','<<d.cost<<','<<d.selected_cost_rank<<','<<d.primary_integration_steps<<','<<d.backup_integration_steps<<'\n';
                }
            }
            planners.clear();for(const auto& ns:namespaces)ros::param::del(ns);
        }
        std::cout<<"Matched-state samples: "<<state_id<<" inputs x 3 methods x 3 repeats\n";
    }
    return 0;
}
