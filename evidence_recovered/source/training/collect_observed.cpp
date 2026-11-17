#include "observed_scene.h"
#include "observed_recovery_features.h"
#include <cstdint>

struct Candidate { P::pr_t command; P::rollout_t rollout; float cost; int library_index; };
// 12 metadata + 58 features + 15 triples (validity, integration steps, rejection).
// Rejection: 0=verified,1=envelope,2=collision/unknown boundary,3=known-space,4=invalid.
void capture(P& p, const P::s_t& s, int query, double t, bool active,
             std::ofstream& binary, std::ofstream& states) {
    P::FlagScope guard(p.planning_guard_active, true);
    const auto commands=p.generate_library(s); std::vector<Candidate> candidates;
    auto append=[&](const P::pr_t& c,int index){
        auto r=p.rollout_motion_primitive(s,c);float cost=p.evaluate_motion_primitive(s,c,r);
        if(p.check_primary_feasibility(s,r)&&std::isfinite(cost)) candidates.push_back({c,std::move(r),cost,index});
    };
    for(size_t i=0;i<commands.size();++i)append(commands[i],int(i));
    if(candidates.empty()&&p.last_pr_ptr){auto c=*p.last_pr_ptr;c.t_way=p.horizon_time;append(c,int(commands.size()));}
    std::stable_sort(candidates.begin(),candidates.end(),[](const Candidate&a,const Candidate&b){return a.cost<b.cost;});
    int positives=0;double oracle_ms=0;
    auto begin=std::chrono::steady_clock::now();
    for(size_t rank=0;rank<candidates.size();++rank){
        const auto& c=candidates[rank];P::recovery_candidate_t summary;summary.terminal=c.rollout.states.back();summary.cost=c.cost;
        auto features=p.recovery_features(s,summary);features.resize(28);
        const auto& z=summary.terminal;auto obs=observed_recovery::encode(p.sensor_map->map(),z.pos[0],z.pos[1],z.h,z.sigma,z.V,p.recovery_horizon);
        features.insert(features.end(),obs.begin(),obs.end());
        std::array<float,115> row{};row[0]=query;row[1]=rank+1;row[2]=c.cost;row[11]=c.library_index;
        std::copy(features.begin(),features.end(),row.begin()+12);
        const auto successors=p.generate_recovery_library(z);assert(successors.size()==15);bool found=false;
        for(size_t j=0;j<successors.size();++j){
            const auto r=p.rollout_motion_primitive(z,successors[j],c.command.t_way,-1,&s);
            const int work=int(r.states.size())-1;int reason=0;
            if(r.early_rejected){
                if(r.early_rejection_reason==P::rejection_reason_t::ENVELOPE)reason=1;
                else if(r.early_rejection_reason==P::rejection_reason_t::COLLISION)reason=2;
                else if(r.early_rejection_reason==P::rejection_reason_t::KNOWN_SPACE)reason=3;
            }else if(!r.valid)reason=4;else if(r.collision)reason=2;
            else if(!p.within_recovery_envelope(r))reason=1;else if(!p.within_known_space(s,r))reason=3;
            row[70+3*j]=reason==0;row[71+3*j]=work;row[72+3*j]=reason;
            if(!found){row[4]+=work;row[5]+=1;if(reason==2)row[6]+=1;if(reason==1)row[7]+=1;if(reason==3)row[8]+=1;if(reason==4)row[9]+=1;}
            if(reason==0)found=true;
        }
        row[3]=found;row[10]=successors.size();positives+=found;
        if(rank==0){const auto reference=p.check_terminal_recovery(s,c.rollout,c.command.t_way);assert(reference.verified==found&&reference.integration_steps==int(row[4])&&reference.checked==int(row[5]));}
        binary.write(reinterpret_cast<const char*>(row.data()),sizeof(row));
    }
    oracle_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    const auto stats=p.sensor_map->map().stats();
    states<<query<<','<<t<<','<<active<<','<<commands.size()<<','<<candidates.size()<<','<<positives<<','<<s.pos[0]<<','<<s.pos[1]<<','<<s.h<<','<<s.V<<','<<s.gamma<<','<<s.sigma<<','<<s.roll<<','<<s.attack<<','<<s.T<<','<<stats.scans<<','<<stats.free_cells<<','<<stats.occupied_cells<<','<<oracle_ms<<'\n';states.flush();
}
int main(int argc,char**argv){try{
 if(argc<8)throw std::runtime_error("usage: collector output split family index sampler base max_steps");
 ros::init(argc,argv,"observed_training_collector",ros::init_options::AnonymousName);
 const std::string outdir=argv[1],split=argv[2];int family=std::stoi(argv[3]),index=std::stoi(argv[4]),method=std::stoi(argv[5]);unsigned base=std::stoul(argv[6]);int maxsteps=std::stoi(argv[7]);
 Scene sc=scene(family,index,base);std::string key=split+"_"+sc.name+"_m"+std::to_string(method);mkdir(outdir.c_str(),0755);
 ros::NodeHandle nh("/observed_training/"+key);cfg(nh,sc,"");nh.setParam("recovery/candidate_order",method==1?"diverse":"cost");P p(nh);p.planning_timer.stop();p.recovery_order_mode=method==1?"diverse":"cost";p.recovery_max_candidates=method==2?10000:12;p.global_target.pos[0]=250;p.global_target.pos[1]=0;p.global_target.h=20;
 assert(p.sensor_map->map().size()==0&&p.static_obs_vec.empty()&&p.sphere_obs_vec.empty()&&p.box_obstacles_.empty()&&p.horizontal_obs_vec.empty()&&p.dyn_obs_vec.empty());
 EvaluationPlant plant;int prefixchecks=0,prefixrejects=0;

 auto s=trim(p);P::pr_t prev;prev.w_roll_c=s.roll;prev.attack_c=s.attack;prev.T_c=s.T;prev.t_way=3;auto dirs=directions();std::string outcome="timeout";bool active=false;int query=0,lastcapture=-1;double simtime=0,minclear=1e9;
 std::ofstream binary(outdir+"/"+key+"_candidates.f32",std::ios::binary),states(outdir+"/"+key+"_states.csv"),path(outdir+"/"+key+"_path.csv"),geom(outdir+"/"+key+"_geometry.csv");
 states<<"query,t,active,library_size,feasible,positive,x,y,h,V,gamma,heading,roll,attack,thrust,scans,free_cells,occupied_cells,oracle_ms\n";states<<std::setprecision(17);path<<"t,x,y,h,V,gamma,heading,roll,attack,thrust,command_roll,command_attack,command_thrust\n";path<<std::setprecision(17);geom<<"type,x,y,z,r,height\n";geom<<std::setprecision(17);for(const auto&o:sc.obs)geom<<o.type<<','<<o.x<<','<<o.y<<','<<o.z<<','<<o.r<<','<<o.height<<'\n';
 const auto begin=std::chrono::steady_clock::now();
 for(int step=0;step<maxsteps;++step){double t=.1*step;simtime=t;
  if(std::hypot(250-s.pos[0],s.pos[1])<15&&std::abs(s.h-20)<3){outcome="goal";break;}
  if(step%2==0)scan(p,sc,s,t,dirs);
  if((step<=20&&step%2==0)||(step>20&&step%20==0)){capture(p,s,query++,t,active,binary,states);lastcapture=step;}
  auto d=p.select_planning_proposal(s);bool executable=p.prepare_execution(s,d);P::pr_t applied=prev;
  if(executable){active=true;p.accept_selection(d);applied=d.command;}
  else if(!active&&t<2.){}
  else{if(lastcapture!=step)capture(p,s,query++,t,active,binary,states);outcome=active?(d.primary_feasible_count==0?"no_feasible_primary":"no_verified_sequence"):"initialization_failed";break;}
  path<<t<<','<<s.pos[0]<<','<<s.pos[1]<<','<<s.h<<','<<s.V<<','<<s.gamma<<','<<s.sigma<<','<<s.roll<<','<<s.attack<<','<<s.T<<','<<applied.w_roll_c<<','<<applied.attack_c<<','<<applied.T_c<<'\n';
  auto r=plant.advance(s,applied,.1);if(!r.valid){outcome="invalid_dynamics";break;}bool safe=true;size_t terminal=r.states.size()-1;
  for(size_t j=1;j<r.states.size()&&safe;++j){auto&q=r.states[j];auto&last=r.states[j-1];Pt pt{q.pos[0],q.pos[1],q.h},pt0{last.pos[0],last.pos[1],last.h};double length=std::sqrt(std::pow(pt.x-pt0.x,2)+std::pow(pt.y-pt0.y,2)+std::pow(pt.z-pt0.z,2));for(const auto&o:sc.obs){double endmin=std::min(clearance(pt,o),clearance(pt0,o));if(endmin-length>std::max(minclear,2.))continue;double c=chord_clearance(pt0,pt,o);minclear=std::min(minclear,c);if(c<2){outcome=c<0?"collision":"margin_incursion";safe=false;break;}}if(safe&&!p.state_within_recovery_envelope(q)){outcome="envelope_violation";safe=false;}if(!safe)terminal=j;}
  s=r.states[terminal];if(executable)prev=d.command;p.advance_execution(.1);if(!safe)break;
 }
 const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();std::ofstream summary(outdir+"/"+key+"_summary.csv");summary<<"case,split,family,seed,index,sampler,outcome,queries,sim_seconds,wall_seconds,prefix_checks,prefix_rejects,min_clearance\n";summary<<key<<','<<split<<','<<sc.family<<','<<sc.seed<<','<<index<<','<<method<<','<<outcome<<','<<query<<','<<simtime<<','<<elapsed<<','<<prefixchecks<<','<<prefixrejects<<','<<minclear<<'\n';std::cout<<key<<" outcome="<<outcome<<" queries="<<query<<" wall="<<elapsed<<std::endl;ros::param::del(nh.getNamespace());ros::shutdown();return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}}
