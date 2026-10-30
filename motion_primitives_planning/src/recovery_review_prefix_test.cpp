#define main review_benchmark_entry
#include "recovery_review_benchmark.cpp"
#undef main
#include <cassert>
int main(int argc,char**argv){
 ros::init(argc,argv,"review_prefix_test");Options options;options.prefix_check=false;
 Scene scene;scene.name="prefix_unit";scene.family="empty";Method method{"cost12","cost",12};
 ros::NodeHandle nh("/review_prefix_unit");configure(nh,scene,method,options);MP planner(nh);auto s=trim(planner);
 auto before=planner.select_motion_primitive(s);assert(before.primary_feasible);
 planner.review_prefix_validator=[](const MP::s_t&,const MP::pr_t&){return true;};
 auto pass=planner.select_motion_primitive(s);assert(pass.primary_feasible&&pass.cost==before.cost&&pass.selected_cost_rank==before.selected_cost_rank&&pass.command.w_roll_c==before.command.w_roll_c&&pass.command.attack_c==before.command.attack_c&&pass.command.T_c==before.command.T_c);
 assert(planner.review_profile.prefix_checked>0&&planner.review_profile.prefix_rejected==0);
 planner.remember_command(before.command);
 planner.review_prefix_validator=[](const MP::s_t&,const MP::pr_t&){return false;};
 auto blocked=planner.select_motion_primitive(s);assert(!blocked.primary_feasible&&planner.review_profile.prefix_rejected==planner.review_profile.prefix_checked&&planner.review_profile.prefix_checked>0);
 assert(blocked.primary_evaluated==before.primary_evaluated+1); // Previous-control fallback is also rejected by the hook.
 options.prefix_check=true;bind_prefix_check(planner,options);auto checked=planner.select_motion_primitive(s);assert(checked.primary_feasible&&planner.review_prefix_validator(s,checked.command));
 auto outside=s;outside.roll=1.0f;assert(!planner.review_prefix_validator(outside,checked.command));
 Scene blocked_scene=scene;blocked_scene.obstacles.push_back({"cylinder",0,0,20,10,100});ros::NodeHandle blocked_nh("/review_prefix_collision");configure(blocked_nh,blocked_scene,method,options);MP obstacle(blocked_nh);bind_prefix_check(obstacle,options);assert(!obstacle.review_prefix_validator(s,checked.command));
 ros::param::del(nh.getNamespace());ros::param::del(blocked_nh.getNamespace());
 std::cout<<"{\"checks\":7,\"passed\":7,\"checks_description\":[\"accept-all preserves decision\",\"prefix accounting\",\"reject-all removes primaries\",\"previous-control fallback uses same prefix filter\",\"nominal prefix accepts open-flight control\",\"prefix rejects envelope violation\",\"prefix rejects collision\"]}\n";
}
