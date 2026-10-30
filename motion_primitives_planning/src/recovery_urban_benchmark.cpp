// Supplementary paired city-block experiment. The generated extension changes
// only box geometry; learning benchmark helpers and original files remain frozen.
#include "urban_motion_primitives.h"
#define main frozen_learning_benchmark_main
#include "recovery_learning_benchmark.cpp"
#undef main

struct UrbanComponent {
    int building_id;
    std::string component;
    double x,y,z,size_x,size_y,size_z;
};
struct UrbanScene {
    Scene configuration;
    std::vector<UrbanComponent> boxes;
    int buildings;
};
std::vector<UrbanScene> urban_maps(int count) {
    std::vector<UrbanScene> maps;
    for(int index=0;index<count;++index) {
        const uint32_t seed=701000u+index;
        Generator blocks{seed},infill{seed^0x51ed270bu};
        std::vector<UrbanComponent> base,extra;
        int id=0;
        for(int col=0;col<4;++col)for(int row=0;row<5;++row) {
            ++id;
            const double x=60+45*col+blocks.range(-3,3),y=-68+34*row+blocks.range(-3,3);
            const double sx=blocks.range(16,24),sy=blocks.range(16,24);
            const double hp=blocks.range(8,14),ht=blocks.range(24,58);
            const double sign=(col+row)%2?1.:-1.;
            base.push_back({id,"podium",x,y,hp/2,sx,sy,hp});
            base.push_back({id,"tower",x+sign*.2*sx,y+sign*.2*sy,ht/2,.6*sx,.6*sy,ht});
        }
        for(int col=0;col<4;++col)for(int row=0;row<2;++row) {
            ++id;
            const double x=82.5+45*col+infill.range(-2,2),y=(row?17.:-17.)+infill.range(-2,2);
            const double sx=infill.range(10,16),sy=infill.range(10,16),h=infill.range(18,42);
            extra.push_back({id,"infill",x,y,h/2,sx,sy,h});
        }
        for(int dense=0;dense<2;++dense) {
            UrbanScene city;city.configuration.family=dense?"urban_dense":"urban_blocks";
            city.configuration.name=city.configuration.family+"_"+std::to_string(index);
            city.configuration.seed=seed;city.configuration.low=5;city.configuration.high=45;
            city.boxes=base;city.buildings=dense?28:20;
            if(dense)city.boxes.insert(city.boxes.end(),extra.begin(),extra.end());
            maps.push_back(city);
        }
    }
    return maps;
}
void export_urban_geometry(const Options& options,const std::vector<UrbanScene>& maps) {
    auto f=output(options.prefix+"_scenarios.csv");
    f<<"scene,family,seed,low,high,building_id,component,type,x,y,z,size_x,size_y,size_z\n";
    auto m=output(options.prefix+"_map_metadata.csv");
    m<<"scene,seed,building_count,box_count,view_area_m2,display_seed\n";
    for(const auto& city:maps) {
        const auto& s=city.configuration;
        m<<s.name<<','<<s.seed<<','<<city.buildings<<','<<city.boxes.size()<<",45000,"<<(s.seed==701000u)<<'\n';
        for(const auto& b:city.boxes)f<<s.name<<','<<s.family<<','<<s.seed<<','<<s.low<<','<<s.high<<','<<b.building_id<<','<<b.component<<",box,"<<b.x<<','<<b.y<<','<<b.z<<','<<b.size_x<<','<<b.size_y<<','<<b.size_z<<'\n';
    }
}
std::vector<MP::urban_box_t> urban_boxes(const UrbanScene& city) {
    std::vector<MP::urban_box_t> out;
    for(const auto& b:city.boxes) {
        MP::urban_box_t box;box.x=b.x;box.y=b.y;box.z=b.z;
        box.size_x=b.size_x;box.size_y=b.size_y;box.size_z=b.size_z;out.push_back(box);
    }
    return out;
}
double physical_box_distance(const UrbanComponent& b,const MP::s_t& s) {
    const double dx=std::abs(s.pos[0]-b.x)-b.size_x/2;
    const double dy=std::abs(s.pos[1]-b.y)-b.size_y/2;
    const double dz=std::abs(s.h-b.z)-b.size_z/2;
    return std::sqrt(std::pow(std::max(dx,0.),2)+std::pow(std::max(dy,0.),2)+std::pow(std::max(dz,0.),2))+
        std::min(std::max(dx,std::max(dy,dz)),0.);
}
void urban_episodes(const Options& options,const std::vector<UrbanScene>& maps) {
    const std::vector<Method> methods={{"cost12","cost",12},{"diverse12","diverse",12},{"learned12","learned",12},{"state_only12","learned",12},{"full203","cost",203}};
    auto ef=output(options.prefix+"_episodes.csv"),plans=output(options.prefix+"_plans.csv"),paths=output(options.prefix+"_paths.csv");
    ef<<"case_id,scene,family,condition,method,outcome,steps,sim_seconds,goal_distance,min_obstacle_clearance\n";
    plans<<"case_id,step,"<<decision_fields<<'\n';paths<<"case_id,step,time,"<<state_fields<<'\n';int id=0;
    for(const auto& city:maps)for(const auto& method:methods) {
        const auto& scene=city.configuration;
        const std::string case_id="urban_"+std::to_string(++id),ns="/urban/episode_"+case_id;
        ros::NodeHandle nh(ns);configure(nh,scene,method,options);
        {
            MP p(nh);p.set_urban_boxes(urban_boxes(city));auto state=trim(p);EvaluationPlant plant;
            // A point on each box's interior must be visible to both geometry APIs.
            for(const auto& b:city.boxes) {
                const float height=std::max(6.,std::min(44.,b.z));
                if(height<b.size_z && (!p.collision_check(b.x,b.y,height)||p.obstacle_signed_distance(b.x,b.y,height)>0))
                    throw std::runtime_error("Urban geometry did not enter planner queries");
            }
            std::string outcome="timeout";int steps=0;double clearance=1e9;
            const auto record=[&](){paths<<case_id<<','<<steps<<','<<steps*.2<<',';state_record(paths,state);paths<<'\n';};
            const auto inspect=[&](const EvaluationPlant::Result& segment) {
                if(!segment.valid){outcome="invalid_dynamics";return false;}
                for(const auto& q:segment.states) {
                    for(const auto& obstacle:city.boxes)clearance=std::min(clearance,physical_box_distance(obstacle,q));
                    if(!p.state_within_recovery_envelope(q)){outcome="envelope_violation";return false;}
                    if(p.collision_check(q.pos[0],q.pos[1],q.h)){outcome="margin_incursion";return false;}
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
                if(segment.states.empty()){outcome="invalid_dynamics";break;}
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
    ros::init(argc,argv,"recovery_urban_benchmark");ros::NodeHandle args("~");Options options;int count;
    args.param<std::string>("output_prefix",options.prefix,"/tmp/recovery_urban");
    args.param<std::string>("phase",options.phase,"all");
    args.param<std::string>("model_path",options.model,"");
    args.param<std::string>("state_only_model_path",options.state_only_model,"");
    args.param<int>("map_count",count,8);options.work_power=0;
    if(count<1||count>8||(options.phase!="all"&&options.phase!="geometry"))return 2;
    LearnedRecoveryNetwork network;
    if(!network.load(options.model)||!network.load(options.state_only_model)){std::cerr<<"Both frozen models are required\n";return 2;}
    try {
        const auto maps=urban_maps(count);export_urban_geometry(options,maps);
        if(options.phase=="all")urban_episodes(options,maps);
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
