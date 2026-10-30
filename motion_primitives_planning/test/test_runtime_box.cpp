#include <gtest/gtest.h>
#include <ros/ros.h>
#include <limits>
#include <memory>
#include "../src/runtime_motion_primitives.h"

namespace {
using Planner = RuntimeMotionPrimitives;
using Box = Planner::box_obstacle_t;

Box box(double x, double y, double z, double sx, double sy, double sz,
        double clearance = 1.0) {
    Box result;
    result.geometry = {x, y, z, sx, sy, sz};
    result.clearance = clearance;
    return result;
}

class RuntimeBoxes : public ::testing::Test {
protected:
    std::unique_ptr<ros::NodeHandle> nh;
    std::string ns;
    void SetUp() override {
        static unsigned serial = 0;
        ns = "/runtime_box_test/case_" + std::to_string(++serial);
        nh.reset(new ros::NodeHandle(ns));
        nh->setParam("motion_primitive/min_altitude", 1.0);
        nh->setParam("motion_primitive/max_altitude", 80.0);
        nh->setParam("recovery/enabled", true);
        nh->setParam("recovery/stop_on_rejection", true);
        nh->setParam("waypoints/wp1/x", 250.0);
        nh->setParam("waypoints/wp1/y", 0.0);
        nh->setParam("waypoints/wp1/h", 20.0);
    }
    void TearDown() override { nh.reset(); ros::param::del(ns); }
    void install_parameter_box() {
        nh->deleteParam("box_obstacles");
        nh->setParam("box_obstacles/num", 1);
        nh->setParam("box_obstacles/box1/x", 10);  // Mixed XML-RPC int/double.
        nh->setParam("box_obstacles/box1/y", 0.0);
        nh->setParam("box_obstacles/box1/z", 20);
        nh->setParam("box_obstacles/box1/size_x", 4.0);
        nh->setParam("box_obstacles/box1/size_y", 6);
        nh->setParam("box_obstacles/box1/size_z", 8.0);
    }
    Planner::s_t trim(const Planner& planner) const {
        Planner::s_t state;
        state.V = 15.5; state.h = 20;
        double lo = -20 * Rad2Deg, hi = 25 * Rad2Deg;
        for (int i = 0; i < 60; ++i) {
            const double a = (lo + hi) / 2;
            const double residual = 1.86 * state.V * state.V *
                (planner.C_D(a) * std::tan(a) + planner.C_L(a)) - 9.81;
            if (residual > 0) hi = a; else lo = a;
        }
        state.attack = (lo + hi) / 2;
        state.T = 1.86 * state.V * state.V * planner.C_D(state.attack) / std::cos(state.attack);
        return state;
    }
};

TEST_F(RuntimeBoxes, LoadsNumericFieldsAndPerBoxClearance) {
    install_parameter_box();
    Planner defaults(*nh);
    ASSERT_EQ(defaults.box_obstacles().size(), 1u);
    EXPECT_DOUBLE_EQ(defaults.box_obstacles()[0].clearance, 1.0);
    EXPECT_TRUE(defaults.collision_check(7, 0, 20));
    EXPECT_FALSE(defaults.collision_check(6.9, 0, 20));
    nh->setParam("box_obstacles/box1/clearance", .25);
    Planner custom(*nh);
    EXPECT_DOUBLE_EQ(custom.box_obstacles()[0].clearance, .25);
    EXPECT_FALSE(custom.collision_check(7, 0, 20));
    EXPECT_TRUE(custom.collision_check(7.75, 0, 20));
    EXPECT_FLOAT_EQ(custom.obstacle_signed_distance(7, 0, 20), .75);
}

TEST_F(RuntimeBoxes, MalformedParametersCannotSilentlyOmitObstacles) {
    for (const std::string field : {"num", "box1", "box1/size_x"}) {
        install_parameter_box(); nh->deleteParam("box_obstacles/" + field);
        EXPECT_THROW({Planner planner(*nh);}, std::invalid_argument) << field;
    }
    install_parameter_box(); nh->setParam("box_obstacles/num", -1);
    EXPECT_THROW({Planner planner(*nh);}, std::invalid_argument);
    nh->setParam("box_obstacles/num", 1.0);
    EXPECT_THROW({Planner planner(*nh);}, std::invalid_argument);
    nh->setParam("box_obstacles/num", true);
    EXPECT_THROW({Planner planner(*nh);}, std::invalid_argument);
    install_parameter_box(); nh->setParam("box_obstacles/box1/size_x", std::string("4"));
    EXPECT_THROW({Planner planner(*nh);}, std::invalid_argument);
    install_parameter_box(); nh->setParam("box_obstacles/box1/size_x", 0.0);
    EXPECT_THROW({Planner planner(*nh);}, std::invalid_argument);
    install_parameter_box(); nh->setParam("box_obstacles/box1/clearance", -.1);
    EXPECT_THROW({Planner planner(*nh);}, std::invalid_argument);
}

TEST_F(RuntimeBoxes, ExplicitZeroCountClearsPreviousSceneParameters) {
    install_parameter_box();
    Planner first_scene(*nh);
    ASSERT_TRUE(first_scene.collision_check(10, 0, 20));
    // Launch files reset only the count; old box keys can remain on the master.
    nh->setParam("box_obstacles/num", 0);
    Planner next_scene(*nh);
    EXPECT_TRUE(next_scene.box_obstacles().empty());
    EXPECT_FALSE(next_scene.collision_check(10, 0, 20));
    EXPECT_FLOAT_EQ(next_scene.obstacle_signed_distance(10, 0, 20), 30);
    EXPECT_TRUE(next_scene.collision_visual().markers.empty());
    nh->deleteParam("box_obstacles");
    Planner no_map(*nh);
    EXPECT_TRUE(no_map.box_obstacles().empty());
}

TEST_F(RuntimeBoxes, InvalidMapReplacementPreservesExistingCollisionMap) {
    Planner planner(*nh);
    const Box good = box(10, 0, 20, 4, 6, 8);
    planner.set_box_obstacles({good});
    EXPECT_THROW(planner.set_box_obstacles({good, box(0, 0, 20, 0, 4, 8)}), std::invalid_argument);
    EXPECT_THROW(planner.set_box_obstacles({box(0, 0, 20, 4, 4, 8, -1)}), std::invalid_argument);
    EXPECT_THROW(planner.set_box_obstacles({box(0, 0, 20, 4, 4, 8, std::numeric_limits<double>::quiet_NaN())}), std::invalid_argument);
    ASSERT_EQ(planner.box_obstacles().size(), 1u);
    EXPECT_TRUE(planner.collision_check(10, 0, 20));
    planner.set_box_obstacles({});
    EXPECT_FALSE(planner.collision_check(10, 0, 20));
}

TEST_F(RuntimeBoxes, CollisionAndNetworkFeaturesUseSameExpandedGeometry) {
    Planner planner(*nh);
    const auto state = trim(planner);
    Planner::recovery_candidate_t candidate; candidate.terminal = state; candidate.cost = 2;
    const auto before = planner.recovery_features(state, candidate);
    planner.set_box_obstacles({box(10, 0, 20, 10, 10, 20, 2),
                              box(25, 3, 12, 6, 8, 24, .5)});
    const auto after = planner.recovery_features(state, candidate);
    ASSERT_EQ(after.size(), 103u);
    for (int i = 0; i < 28; ++i) EXPECT_FLOAT_EQ(before[i], after[i]);
    EXPECT_NEAR(after[28 + 2*3 + 1], 3.0/30, 1e-6);
    for (float x = 0; x <= 34; x += .7f)
        for (float y = -12; y <= 12; y += 1.3f)
            for (float z : {1.f, 10.f, 20.f, 25.f, 32.f, 40.f}) {
                bool inside = false;
                for (const auto& entry : planner.box_obstacles()) {
                    const auto& b = entry.geometry; const double c = entry.clearance;
                    inside = inside || (std::abs(x-b.x) <= b.size_x/2+c &&
                        std::abs(y-b.y) <= b.size_y/2+c && std::abs(z-b.z) <= b.size_z/2+c);
                }
                EXPECT_EQ(planner.collision_check(x,y,z), inside);
                EXPECT_EQ(planner.obstacle_signed_distance(x,y,z) <= 0, inside);
            }
}

TEST_F(RuntimeBoxes, PhysicalMarkersAndPredictedWallEncounter) {
    Planner planner(*nh); const auto state = trim(planner);
    Planner::pr_t command; command.attack_c=state.attack; command.T_c=state.T; command.t_way=3;
    ASSERT_FALSE(planner.rollout_motion_primitive(state,command).collision);
    planner.set_box_obstacles({box(25,0,20,.2,100,40,2)});
    const auto blocked=planner.rollout_motion_primitive(state,command);
    EXPECT_TRUE(blocked.collision);
    EXPECT_FALSE(planner.check_primary_feasibility(state,blocked));
    const auto markers=planner.collision_visual();
    ASSERT_EQ(markers.markers.size(),1u); const auto& marker=markers.markers.front();
    EXPECT_EQ(marker.type,visualization_msgs::Marker::CUBE);
    EXPECT_EQ(marker.header.frame_id,"world");
    EXPECT_DOUBLE_EQ(marker.pose.position.x,25);
    EXPECT_DOUBLE_EQ(marker.pose.position.z,20);
    EXPECT_DOUBLE_EQ(marker.pose.orientation.w,1);
    EXPECT_DOUBLE_EQ(marker.scale.x,.2);  // Physical size, without the 2 m margin.
    EXPECT_DOUBLE_EQ(marker.scale.y,100); EXPECT_DOUBLE_EQ(marker.scale.z,40);
    planner.set_box_obstacles({box(25,0,4,10,100,8,2)});
    EXPECT_FALSE(planner.rollout_motion_primitive(state,command).collision);
}
}  // namespace

int main(int argc,char** argv) {
    ros::init(argc,argv,"runtime_box_test");
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
