#include <gtest/gtest.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include "../src/evaluation_plant.h"
#include "../src/runtime_motion_primitives.h"

namespace {
using Runtime = RuntimeMotionPrimitives;
using Frozen = MotionPrimitives;

template <typename Destination, typename Source>
Destination state_cast(const Source& source) {
    Destination result;
    result.V=source.V;result.gamma=source.gamma;result.sigma=source.sigma;
    result.d_sigma=source.d_sigma;result.T=source.T;result.h=source.h;
    result.pos[0]=source.pos[0];result.pos[1]=source.pos[1];
    for(int i=0;i<4;++i)result.q[i]=source.q[i];
    result.roll=source.roll;result.attack=source.attack;
    return result;
}
Frozen::pr_t plant_command(const Runtime::pr_t& source) {
    Frozen::pr_t result;
    result.w_roll_c=source.w_roll_c;result.attack_c=source.attack_c;
    result.T_c=source.T_c;result.t_way=source.t_way;
    return result;
}

class RuntimeVertical : public ::testing::Test {
protected:
    std::unique_ptr<ros::NodeHandle> nh;
    std::string ns;
    void SetUp() override {
        static unsigned serial=0;
        ns="/runtime_vertical_test/case_"+std::to_string(++serial);
        nh.reset(new ros::NodeHandle(ns));
        nh->setParam("motion_primitive/horizon_time",3.0);
        nh->setParam("motion_primitive/V",15.5);
        nh->setParam("motion_primitive/num",29);
        nh->setParam("motion_primitive/vertical_num",7);
        nh->setParam("motion_primitive/min_altitude",1.0);
        nh->setParam("motion_primitive/max_altitude",80.0);
        nh->setParam("motion_primitive/max_climb_angle",20.0);
        nh->setParam("motion_primitive/max_descent_angle",20.0);
        nh->setParam("motion_primitive/compensate_tau",0.8);
        nh->setParam("motion_primitive/waypoint_altitude_tolerance",3.0);
        nh->setParam("waypoints/num",1);
        nh->setParam("waypoints/wp1/x",350.0);
        nh->setParam("waypoints/wp1/y",0.0);
        nh->setParam("waypoints/wp1/h",24.0);
        nh->setParam("recovery/enabled",true);
        nh->setParam("recovery/require_terminal",true);
        nh->setParam("recovery/horizon_time",6.0);
        nh->setParam("recovery/max_candidates",12);
        nh->setParam("recovery/stop_on_rejection",true);
        nh->setParam("recovery/reuse_verified_plan",false);
        nh->setParam("recovery/candidate_order",std::string("learned"));
        nh->setParam("recovery/learned_work_power",0.0);
        std::string model;
        ASSERT_TRUE(ros::param::get("/runtime_vertical_model_path",model));
        nh->setParam("recovery/model_path",model);
    }
    void TearDown() override {nh.reset();ros::param::del(ns);}
    Runtime::s_t orbit(unsigned phase=0) const {
        // Three phases recorded from the stalled Gazebo orbit. The actuator
        // values are the measured steady command; gamma is effectively zero.
        const float positions[3][3]={{386.765537f,33.585469f,1.8451461f},
            {325.322497f,45.438944f,-2.2218675f},{352.810957f,-10.776147f,-.00569567f}};
        Runtime::s_t s;s.pos[0]=positions[phase][0];s.pos[1]=positions[phase][1];
        s.sigma=positions[phase][2];s.h=8.6838457f;s.V=15.4975267f;
        s.roll=35*Rad2Deg;s.attack=-.03725088f;s.T=.894687f;s.d_sigma=.443235f;
        return s;
    }
    template <typename Planner>
    void remember_orbit(Planner& planner) const {
        typename Planner::pr_t command;
        command.w_roll_c=35*Rad2Deg;command.attack_c=-.0372508f;
        command.T_c=.894687f;command.t_way=3;
        planner.remember_command(command);
    }
};

TEST_F(RuntimeVertical, CapturedOrbitSelectsVerifiedClimbingTurn) {
    Frozen old(*nh);Runtime runtime(*nh);
    remember_orbit(old);remember_orbit(runtime);
    const auto initial=orbit();
    const auto before=old.select_motion_primitive(state_cast<Frozen::s_t>(initial));
    ASSERT_TRUE(before.recovery_verified);
    EXPECT_NEAR(before.rollout.states.back().h,initial.h,.05);
    const auto after=runtime.select_motion_primitive(initial);
    ASSERT_TRUE(after.primary_feasible);ASSERT_TRUE(after.recovery_verified);
    EXPECT_EQ(after.candidate_order,"learned");
    EXPECT_GT(after.rollout.states.back().h,initial.h+1);
    EXPECT_GT(after.command.w_roll_c,10*Rad2Deg);
    EXPECT_GT(after.rollout.states.back().gamma,0);
    EXPECT_LT(after.cost,before.cost);
    for(const auto& s:after.rollout.states)
        EXPECT_LE(s.gamma,20*Rad2Deg+1e-5);
}

TEST_F(RuntimeVertical, AggressiveCoarseClimbStillFailsOriginalEnvelope) {
    Runtime runtime(*nh);const auto initial=orbit();
    unsigned coarse_climbs=0,coarse_feasible=0;
    for(const auto& command:runtime.generate_library(initial)) {
        if(std::abs(command.wp.h-16.333333f)>.001)continue;
        ++coarse_climbs;const auto prediction=runtime.rollout_motion_primitive(initial,command);
        coarse_feasible+=runtime.check_primary_feasibility(initial,prediction);
        EXPECT_TRUE(std::any_of(prediction.states.begin(),prediction.states.end(),
            [](const Runtime::s_t& s){return s.gamma>20*Rad2Deg;}));
    }
    EXPECT_EQ(coarse_climbs,29u);EXPECT_EQ(coarse_feasible,0u);
}

TEST_F(RuntimeVertical, DistantAltitudeTargetsRetainGentleVerifiedClimbs) {
    const auto initial=orbit();
    for(double target:{24.,40.,65.}) {
        nh->setParam("waypoints/wp1/h",target);Runtime runtime(*nh);
        bool found=false;
        for(const auto& command:runtime.generate_library(initial)) {
            if(command.wp.h<=initial.h+.25f||command.wp.h>initial.h+5)continue;
            const auto prediction=runtime.rollout_motion_primitive(initial,command);
            if(!runtime.check_primary_feasibility(initial,prediction))continue;
            if(prediction.states.back().h<=initial.h+1)continue;
            if(runtime.check_terminal_recovery(initial,prediction,command.t_way).verified){found=true;break;}
        }
        EXPECT_TRUE(found)<<"target altitude="<<target;
    }
}

TEST_F(RuntimeVertical, AltitudeBoundaryClippingKeepsFiniteUniqueTargets) {
    for(float altitude:{1.001f,79.999f}) {
        nh->setParam("waypoints/wp1/h",altitude<40?1.:80.);
        Runtime runtime(*nh);auto initial=orbit();initial.h=altitude;
        const auto library=runtime.generate_library(initial);
        ASSERT_FALSE(library.empty());std::vector<float> heading_zero_heights;
        for(const auto& command:library) {
            EXPECT_TRUE(std::isfinite(command.w_roll_c));EXPECT_TRUE(std::isfinite(command.attack_c));
            EXPECT_TRUE(std::isfinite(command.T_c));EXPECT_GE(command.wp.h,1);EXPECT_LE(command.wp.h,80);
            if(std::abs(command.wp.theata)<1e-5)heading_zero_heights.push_back(command.wp.h);
        }
        for(size_t a=0;a<heading_zero_heights.size();++a)
            for(size_t b=a+1;b<heading_zero_heights.size();++b)
                EXPECT_GT(std::abs(heading_zero_heights[a]-heading_zero_heights[b]),1e-4);
    }
}

TEST_F(RuntimeVertical, HeightToleranceIsNotRelaxedToSkipWaypoint) {
    nh->setParam("waypoints/num",2);nh->setParam("waypoints/wp2/x",600.);
    nh->setParam("waypoints/wp2/y",0.);nh->setParam("waypoints/wp2/h",24.);
    Runtime runtime(*nh);auto initial=orbit();initial.pos[0]=350;initial.pos[1]=0;
    initial.h=20.;runtime.check_target(initial);
    EXPECT_FLOAT_EQ(runtime.goal_visual().pose.position.x,350.);
    initial.h=21.1f;runtime.check_target(initial);
    EXPECT_FLOAT_EQ(runtime.goal_visual().pose.position.x,600.);
}

TEST_F(RuntimeVertical, IndependentPlantEscapesThreeCapturedOrbitPhases) {
    for(unsigned phase=0;phase<3;++phase) {
        Runtime runtime(*nh);remember_orbit(runtime);auto state=orbit(phase);
        EvaluationPlant plant;bool reached=false;double elapsed=0;
        for(int step=0;step<450;++step) {
            if(std::hypot(state.pos[0]-350,state.pos[1])<=48.825f&&std::abs(state.h-24)<=3){reached=true;break;}
            const auto choice=runtime.select_motion_primitive(state);
            ASSERT_TRUE(choice.primary_feasible)<<"phase="<<phase<<" t="<<elapsed;
            ASSERT_TRUE(choice.recovery_verified)<<"phase="<<phase<<" t="<<elapsed;
            runtime.accept_selection(choice);
            const auto segment=plant.advance(state_cast<Frozen::s_t>(state),plant_command(choice.command),.1);
            ASSERT_TRUE(segment.valid);
            for(const auto& sample:segment.states) {
                const auto observed=state_cast<Runtime::s_t>(sample);
                ASSERT_FALSE(runtime.collision_check(observed.pos[0],observed.pos[1],observed.h));
                ASSERT_TRUE(runtime.state_within_recovery_envelope(observed));
            }
            state=state_cast<Runtime::s_t>(segment.states.back());runtime.advance_execution(.1);elapsed+=.1;
        }
        EXPECT_TRUE(reached)<<"phase="<<phase<<" h="<<state.h<<" t="<<elapsed;
    }
}
}  // namespace

int main(int argc,char** argv) {
    ros::init(argc,argv,"runtime_vertical_test");
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
