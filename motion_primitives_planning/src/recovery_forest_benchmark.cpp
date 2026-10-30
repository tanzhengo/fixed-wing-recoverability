// Supplementary forest-density visualization experiment. The original learning
// benchmark remains byte-for-byte frozen; its state, parameter, logging and
// prediction helpers are reused here without running its main entry point.
#define main frozen_learning_benchmark_main
#include "recovery_learning_benchmark.cpp"
#undef main

std::vector<Scene> forest_maps(int count) {
    std::vector<Scene> maps;
    for(int index=0;index<count;++index) {
        const uint32_t seed=601000u+index;
        Generator trees{seed},bushes{seed^0x51ed270bu};
        std::vector<Obstacle> tree_bank,bush_bank;
        for(int j=0;j<80;++j) {
            const double x=trees.range(40,240),y=trees.range(-80,80);
            const double radius=trees.range(.8,1.4),height=trees.range(24,36),crown=trees.range(4,6);
            tree_bank.push_back({"cylinder",x,y,height/2,radius,height});
            tree_bank.push_back({"sphere",x,y,height,crown,0});
        }
        for(int j=0;j<12;++j) {
            const double x=bushes.range(40,240),y=bushes.range(-80,80),radius=bushes.range(3,5);
            bush_bank.push_back({"sphere",x,y,radius,radius,0});
        }
        for(int density=0;density<2;++density) {
            Scene scene;scene.family=density?"forest_dense":"forest_sparse";
            scene.name=scene.family+"_"+std::to_string(index);scene.seed=seed;scene.low=5;scene.high=45;
            scene.obstacles.insert(scene.obstacles.end(),tree_bank.begin(),tree_bank.begin()+(density?160:80));
            scene.obstacles.insert(scene.obstacles.end(),bush_bank.begin(),bush_bank.end());
            maps.push_back(scene);
        }
    }
    return maps;
}
void forest_metadata(const Options& options,const std::vector<Scene>& maps) {
    auto f=output(options.prefix+"_map_metadata.csv");
    f<<"scene,seed,tree_count,bush_count,area_m2,tree_density_per_m2,display_seed\n";
    for(const auto& scene:maps) {
        const int trees=scene.family=="forest_sparse"?40:80;
        f<<scene.name<<','<<scene.seed<<','<<trees<<",12,32000,"<<trees/32000.<<','<<(scene.seed==601000u)<<'\n';
    }
}
void forest_episodes(const Options& options,const std::vector<Scene>& maps) {
    const std::vector<Method> methods={{"cost12","cost",12},{"diverse12","diverse",12},{"learned12","learned",12},{"state_only12","learned",12},{"full203","cost",203}};
    auto ef=output(options.prefix+"_episodes.csv"),plans=output(options.prefix+"_plans.csv"),paths=output(options.prefix+"_paths.csv");
    ef<<"case_id,scene,family,condition,method,outcome,steps,sim_seconds,goal_distance,min_obstacle_clearance\n";
    plans<<"case_id,step,"<<decision_fields<<'\n';paths<<"case_id,step,time,"<<state_fields<<'\n';int id=0;
    for(const auto& scene:maps)for(const auto& method:methods) {
        const std::string case_id="forest_"+std::to_string(++id),ns="/forest/episode_"+case_id;
        ros::NodeHandle nh(ns);configure(nh,scene,method,options);
        {
            MP p(nh);auto state=trim(p);EvaluationPlant plant;
            std::string outcome="timeout";int steps=0;double clearance=1e9;
            const auto record=[&](){paths<<case_id<<','<<steps<<','<<steps*.2<<',';state_record(paths,state);paths<<'\n';};
            const auto inspect=[&](const EvaluationPlant::Result& segment) {
                if(!segment.valid){outcome="invalid_dynamics";return false;}
                for(const auto& q:segment.states) {
                    for(const auto& obstacle:scene.obstacles) {
                        double d;
                        if(obstacle.type=="sphere")d=std::sqrt(std::pow(q.pos[0]-obstacle.x,2)+std::pow(q.pos[1]-obstacle.y,2)+std::pow(q.h-obstacle.z,2))-obstacle.r;
                        else {
                            const double radial=std::hypot(q.pos[0]-obstacle.x,q.pos[1]-obstacle.y)-obstacle.r;
                            const double axial=std::abs(q.h-obstacle.z)-obstacle.height/2;
                            d=std::hypot(std::max(radial,0.),std::max(axial,0.))+std::min(std::max(radial,axial),0.);
                        }
                        clearance=std::min(clearance,d);
                    }
                    if(!p.state_within_recovery_envelope(q)){outcome="envelope_violation";return false;}
                    if(p.collision_check(q.pos[0],q.pos[1],q.h)){outcome="collision";return false;}
                }
                return true;
            };
            for(int step=0;step<200;++step) {
                record();if(goal(state)){outcome="goal";break;}
                double ms;const auto decision=select(p,state,ms);
                plans<<case_id<<','<<step<<',';decision_record(plans,decision,ms);plans<<'\n';
                if(!decision.primary_feasible){outcome="no_feasible_primary";break;}
                p.accept_selection(decision);
                const auto segment=plant.advance(state,decision.command,.2);
                state=segment.states.back();const bool valid=inspect(segment);
                p.advance_execution(.2);++steps;
                if(!valid)break;if(goal(state)){outcome="goal";break;}
            }
            record();ef<<case_id<<','<<scene.name<<','<<scene.family<<",nominal,"<<method.name<<','<<outcome<<','<<steps<<','<<steps*.2<<','<<std::hypot(250-state.pos[0],state.pos[1])<<','<<clearance<<'\n';
            std::cout<<id<<'/'<<maps.size()*methods.size()<<' '<<scene.name<<' '<<method.name<<' '<<outcome<<std::endl;
        }
        ros::param::del(ns);ef.flush();plans.flush();paths.flush();
    }
}
int main(int argc,char** argv) {
    ros::init(argc,argv,"recovery_forest_benchmark");ros::NodeHandle args("~");Options options;int count;
    args.param<std::string>("output_prefix",options.prefix,"/tmp/recovery_forest");
    args.param<std::string>("phase",options.phase,"all");
    args.param<std::string>("model_path",options.model,"");
    args.param<std::string>("state_only_model_path",options.state_only_model,"");
    args.param<int>("map_count",count,8);options.work_power=0;
    if(count<1||count>8||(options.phase!="all"&&options.phase!="geometry"))return 2;
    LearnedRecoveryNetwork network;
    if(!network.load(options.model)||!network.load(options.state_only_model)){std::cerr<<"Both frozen models are required\n";return 2;}
    try {
        const auto maps=forest_maps(count);geometry(options,maps);forest_metadata(options,maps);
        if(options.phase=="all")forest_episodes(options,maps);
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
