#include <gtest/gtest.h>
#include <ros/ros.h>
#include <limits>
#include <memory>
#include "../src/runtime_motion_primitives.h"

namespace {
using Planner = RuntimeMotionPrimitives;
using runtime_swept_geometry::Point;

TEST(SweptGeometry, BoxSegmentsIncludeFacesCornersAndZeroLength) {
    const urban_geometry::Box box{0, 0, 10, 2, 2, 2};
    EXPECT_TRUE(runtime_swept_geometry::box({-2, 0, 10}, {2, 0, 10}, box, 0));
    EXPECT_TRUE(runtime_swept_geometry::box({-2, 1, 10}, {2, 1, 10}, box, 0));
    EXPECT_TRUE(runtime_swept_geometry::box({-2, 0, 10}, {0, 2, 10}, box, 0));
    EXPECT_FALSE(runtime_swept_geometry::box({-2, 1.01, 10}, {2, 1.01, 10}, box, 0));
    EXPECT_FALSE(runtime_swept_geometry::box({-2, 0, 12}, {2, 0, 12}, box, 0));
    EXPECT_TRUE(runtime_swept_geometry::box({-2, 0, 12}, {2, 0, 12}, box, 1));
    EXPECT_TRUE(runtime_swept_geometry::box({1, 1, 11}, {1, 1, 11}, box, 0));
    EXPECT_FALSE(runtime_swept_geometry::box({1.01, 1, 11}, {1.01, 1, 11}, box, 0));
}

TEST(SweptGeometry, SphereAndFiniteCylinderRespectCapsAndTangencies) {
    EXPECT_TRUE(runtime_swept_geometry::sphere({-2, 1, 0}, {2, 1, 0}, 1));
    EXPECT_FALSE(runtime_swept_geometry::sphere({-2, 1.01, 0}, {2, 1.01, 0}, 1));
    EXPECT_TRUE(runtime_swept_geometry::sphere({0, 0, 0}, {0, 0, 0}, 1));
    EXPECT_TRUE(runtime_swept_geometry::cylinder({0, 0, -2}, {0, 0, 2}, 1, 1));
    EXPECT_TRUE(runtime_swept_geometry::cylinder({-2, 1, 1}, {2, 1, 1}, 1, 1));
    EXPECT_FALSE(runtime_swept_geometry::cylinder({-2, 0, 1.01}, {2, 0, 1.01}, 1, 1));
    // The XY projection crosses the disk only after leaving the finite caps.
    EXPECT_FALSE(runtime_swept_geometry::cylinder({2, 0, 0}, {0, 0, 4}, 1, 1));
}

class RuntimeSwept : public ::testing::Test {
protected:
    std::unique_ptr<ros::NodeHandle> nh;
    std::string ns;
    void SetUp() override {
        static unsigned serial = 0;
        ns = "/runtime_swept_test/case_" + std::to_string(++serial);
        nh.reset(new ros::NodeHandle(ns));
        nh->setParam("motion_primitive/max_altitude", 200.0);
        nh->setParam("motion_primitive/integration_dt", .02);
        nh->setParam("recovery/enabled", true);
        nh->setParam("waypoints/wp1/x", 20.0);
        nh->setParam("waypoints/wp1/y", -230.0);
        nh->setParam("waypoints/wp1/h", 55.0);
    }
    void TearDown() override { nh.reset(); ros::param::del(ns); }
    Planner::s_t point(float x, float y, float z) const {
        Planner::s_t s; s.pos[0] = x; s.pos[1] = y; s.h = z; return s;
    }
    Planner::box_obstacle_t garage() const {
        Planner::box_obstacle_t box;
        box.geometry = {(52.88045 + 95.81302) / 2, (-163.78175 - 102.35183) / 2,
                        137.711761475 / 2, 95.81302 - 52.88045,
                        163.78175 - 102.35183, 137.711761475};
        box.clearance = 2;
        return box;
    }
};

TEST_F(RuntimeSwept, RecordedGarageCornerCrossesBetweenFreeOdometrySamples) {
    Planner planner(*nh); planner.set_box_obstacles({garage()});
    const auto a = point(51.0290536f, -100.3508989f, 54.8755904f);
    const auto b = point(50.8099174f, -100.466652f, 54.881485f);
    EXPECT_FALSE(planner.collision_check(a.pos[0], a.pos[1], a.h));
    EXPECT_FALSE(planner.collision_check(b.pos[0], b.pos[1], b.h));
    EXPECT_TRUE(planner.collision_check_segment(a, b, 0, .016));
    EXPECT_TRUE(planner.collision_check_segment(b, a, 0, .016));
}

TEST_F(RuntimeSwept, RecordedPrimaryLibraryRejectsEveryMissedCornerCrossing) {
    Planner planner(*nh); planner.set_box_obstacles({garage()});
    auto state = point(51.0290536f, -100.3508989f, 54.8755904f);
    state.V = 15.4958386f; state.gamma = .02346417125f; state.sigma = -2.655540677f;
    state.roll = -.0263210889f; state.attack = -.1321418044f;
    state.T = .7873395446f; state.d_sigma = -.02159051983f;
    const auto library = planner.generate_library(state);
    ASSERT_EQ(library.size(), 406u);
    unsigned missed_by_points = 0;
    for (const auto& command : library) {
        const auto prediction = planner.rollout_motion_primitive(state, command);
        EXPECT_TRUE(prediction.collision);
        EXPECT_FALSE(planner.check_primary_feasibility(state, prediction));
        bool point_collision = false;
        for (const auto& s : prediction.states)
            point_collision = point_collision || planner.collision_check(s.pos[0], s.pos[1], s.h);
        if (!point_collision) ++missed_by_points;
    }
    EXPECT_GT(missed_by_points, 250u);
}

TEST_F(RuntimeSwept, SegmentChecksKeepThinObjectsAndAltitudeLimits) {
    Planner planner(*nh);
    Planner::box_obstacle_t thin;
    thin.geometry = {.15, 0, 20, .01, 4, 4}; thin.clearance = 0;
    planner.set_box_obstacles({thin});
    const auto a = point(0, 0, 20), b = point(.31, 0, 20);
    EXPECT_FALSE(planner.collision_check(a.pos[0], a.pos[1], a.h));
    EXPECT_FALSE(planner.collision_check(b.pos[0], b.pos[1], b.h));
    EXPECT_TRUE(planner.collision_check_segment(a, b, 0, .02));
    EXPECT_FALSE(planner.collision_check_segment(point(0, 3, 20), point(.31, 3, 20)));
    EXPECT_TRUE(planner.collision_check_segment(point(0, 3, 20), point(.31, 3, 201)));
    EXPECT_TRUE(planner.collision_check_segment(a, point(std::numeric_limits<float>::quiet_NaN(), 0, 20)));
}

TEST_F(RuntimeSwept, StaticShapeSegmentsMatchExistingMargins) {
    for (const std::string type : {"static_obstacles", "sphere_obstacles", "horizontal_obstacles"}) {
        nh->setParam(type + "/num", 1);
        const std::string prefix = type + (type == "static_obstacles" ? "/obs1" :
                                          type == "sphere_obstacles" ? "/sphere1" : "/cylinder1");
        nh->setParam(prefix + "/x", 0.0); nh->setParam(prefix + "/y", 0.0);
        nh->setParam(prefix + "/z", 20.0); nh->setParam(prefix + "/r", .2);
        nh->setParam(prefix + "/height", 2.0); nh->setParam(prefix + "/length", 2.0);
        nh->setParam(prefix + "/axis_yaw", 45.0);
        Planner planner(*nh);
        const auto a = point(-4, 0, 20), b = point(4, 0, 20);
        EXPECT_FALSE(planner.collision_check(a.pos[0], a.pos[1], a.h));
        EXPECT_FALSE(planner.collision_check(b.pos[0], b.pos[1], b.h));
        EXPECT_TRUE(planner.collision_check_segment(a, b, 0, .02)) << type;
        for (float x : {-3.f, -.5f, 0.f, .5f, 3.f})
            for (float y : {-3.f, 0.f, 3.f})
                for (float z : {17.f, 20.f, 23.f}) {
                    const auto s = point(x, y, z);
                    EXPECT_EQ(planner.collision_check_segment(s, s), planner.collision_check(x, y, z)) << type;
                }
        nh->deleteParam(type);
    }
}

TEST_F(RuntimeSwept, MovingCylinderCrossingUsesRelativeSegment) {
    nh->setParam("dynamic_obstacles/num", 1);
    const std::string prefix = "dynamic_obstacles/dyn_obs1";
    nh->setParam(prefix + "/x", -2.0); nh->setParam(prefix + "/y", 0.0);
    nh->setParam(prefix + "/z", 20.0); nh->setParam(prefix + "/r", .1);
    nh->setParam(prefix + "/height", 2.0); nh->setParam(prefix + "/velocity", 4.0);
    Planner planner(*nh); const auto s = point(0, 0, 20);
    EXPECT_FALSE(planner.collision_check(0, 0, 20, 0));
    EXPECT_FALSE(planner.collision_check(0, 0, 20, 1));
    EXPECT_TRUE(planner.collision_check_segment(s, s, 0, 1));
    EXPECT_FALSE(planner.collision_check_segment(point(0, 2, 20), point(0, 2, 20), 0, 1));
}

TEST_F(RuntimeSwept, CircularObstacleChordUsesArcDeviationBound) {
    nh->setParam("dynamic_obstacles/num", 1);
    const std::string prefix = "dynamic_obstacles/dyn_obs1";
    nh->setParam(prefix + "/type", std::string("circular"));
    nh->setParam(prefix + "/x", 10.0); nh->setParam(prefix + "/y", 0.0);
    nh->setParam(prefix + "/z", 20.0); nh->setParam(prefix + "/r", .01);
    nh->setParam(prefix + "/height", 2.0); nh->setParam(prefix + "/radius", 10.0);
    nh->setParam(prefix + "/angular_velocity", 1.0);
    Planner planner(*nh);
    const auto s = point(10 * std::cos(.25), 10 * std::sin(.25), 20);
    EXPECT_FALSE(planner.collision_check(s.pos[0], s.pos[1], s.h, 0));
    EXPECT_FALSE(planner.collision_check(s.pos[0], s.pos[1], s.h, .5));
    EXPECT_TRUE(planner.collision_check_segment(s, s, 0, .5));
}
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "runtime_swept_test");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
