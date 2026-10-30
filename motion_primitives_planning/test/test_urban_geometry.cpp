#include <gtest/gtest.h>
#include <ros/ros.h>
#include <memory>
#include "../src/urban_motion_primitives.h"

namespace {
using Box = urban_geometry::Box;

TEST(UrbanBoxDistance, PhysicalFacesRoofEdgesAndCorner) {
    const Box box{10, 20, 15, 8, 12, 30};
    EXPECT_DOUBLE_EQ(urban_geometry::signed_distance(box, 10, 20, 15, 0), -4);
    EXPECT_DOUBLE_EQ(urban_geometry::signed_distance(box, 14, 20, 15, 0), 0);
    EXPECT_DOUBLE_EQ(urban_geometry::signed_distance(box, 10, 20, 33, 0), 3);
    EXPECT_DOUBLE_EQ(urban_geometry::signed_distance(box, 17, 30, 15, 0), 5);
    EXPECT_DOUBLE_EQ(urban_geometry::signed_distance(box, 17, 30, 42, 0), 13);
}

TEST(UrbanBoxDistance, ThinWallAndPerFaceMargin) {
    const Box wall{10, 0, 20, .2, 30, 40};
    EXPECT_NEAR(urban_geometry::signed_distance(wall, 8.9, 0, 20), 0, 1e-12);
    EXPECT_GT(urban_geometry::signed_distance(wall, 8.89, 0, 20), 0);
    EXPECT_LT(urban_geometry::signed_distance(wall, 8.91, 0, 20), 0);
    EXPECT_DOUBLE_EQ(urban_geometry::signed_distance(wall, 10, 0, 41), 0);
    EXPECT_DOUBLE_EQ(urban_geometry::signed_distance(wall, 10, 0, 42), 1);
}

class UrbanPlannerGeometry : public ::testing::Test {
protected:
    using MP = MotionPrimitives;
    std::unique_ptr<ros::NodeHandle> nh;
    std::string ns;
    void SetUp() override {
        static int serial = 0;
        ns = "/urban_geometry_test/case_" + std::to_string(++serial);
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
    MP::s_t trim(const MP& p) {
        MP::s_t s; s.V = 15.5; s.h = 20;
        double lo = -20 * Rad2Deg, hi = 25 * Rad2Deg;
        for (int i = 0; i < 60; ++i) {
            const double a = (lo + hi) / 2;
            const double residual = 1.86 * s.V * s.V *
                (p.C_D(a) * std::tan(a) + p.C_L(a)) - 9.81;
            if (residual > 0) hi = a; else lo = a;
        }
        s.attack = (lo + hi) / 2;
        s.T = 1.86 * s.V * s.V * p.C_D(s.attack) / std::cos(s.attack);
        return s;
    }
};

TEST_F(UrbanPlannerGeometry, InvalidInstallationPreservesPreviousMap) {
    MP p(*nh); const Box good{10, 0, 20, 8, 12, 40};
    p.set_urban_boxes({good});
    EXPECT_THROW(p.set_urban_boxes({good, {0, 0, 20, 0, 5, 10}}), std::invalid_argument);
    ASSERT_EQ(p.urban_boxes().size(), 1u);
    EXPECT_TRUE(p.collision_check(10, 0, 20));
    p.set_urban_boxes({});
    EXPECT_FALSE(p.collision_check(10, 0, 20));
    EXPECT_FLOAT_EQ(p.obstacle_signed_distance(10, 0, 20), 30);
}

TEST_F(UrbanPlannerGeometry, UnionCollisionsMatchIndependentExpandedBounds) {
    MP p(*nh);
    const std::vector<Box> boxes{{10, 0, 20, .2, 18, 40}, {25, 3, 12, 10, 6, 24},
                               {25, -6, 5, 10, 12, 10}};
    p.set_urban_boxes(boxes);
    for (float x = 0; x <= 36; x += .7f)
        for (float y = -18; y <= 18; y += 1.3f)
            for (float z : {1.f, 10.f, 11.f, 20.f, 25.f, 40.f, 41.f, 45.f}) {
                bool inside = false;
                for (const auto& b : boxes)
                    inside = inside || (x >= b.x - b.size_x / 2 - 1 && x <= b.x + b.size_x / 2 + 1 &&
                        y >= b.y - b.size_y / 2 - 1 && y <= b.y + b.size_y / 2 + 1 &&
                        z >= b.z - b.size_z / 2 - 1 && z <= b.z + b.size_z / 2 + 1);
                EXPECT_EQ(p.collision_check(x, y, z), inside);
                EXPECT_EQ(p.obstacle_signed_distance(x, y, z) <= 0, inside);
            }
}

TEST_F(UrbanPlannerGeometry, FeatureProbeSeesBoxesAndUnchangedStateChannels) {
    MP p(*nh); const auto s = trim(p);
    MP::recovery_candidate_t candidate; candidate.terminal = s; candidate.cost = 2;
    const auto before = p.recovery_features(s, candidate);
    // Central probe (0,0,20) is 4 m from the inflated near face x=4.
    p.set_urban_boxes({{10, 0, 20, 10, 10, 20}});
    const auto after = p.recovery_features(s, candidate);
    ASSERT_EQ(after.size(), 103u);
    for (int i = 0; i < 28; ++i) EXPECT_FLOAT_EQ(before[i], after[i]);
    EXPECT_NEAR(after[28 + 2 * 3 + 1], 4.0 / 30.0, 1e-6);
    EXPECT_GT(before[28 + 2 * 3 + 1], after[28 + 2 * 3 + 1]);
}

TEST_F(UrbanPlannerGeometry, PredictionRejectsWallEncounter) {
    MP p(*nh); const auto s = trim(p);
    MP::pr_t command; command.attack_c = s.attack; command.T_c = s.T; command.t_way = 3;
    const auto free = p.rollout_motion_primitive(s, command);
    ASSERT_TRUE(free.valid); ASSERT_FALSE(free.collision);
    p.set_urban_boxes({{25, 0, 20, .2, 100, 40}});
    const auto blocked = p.rollout_motion_primitive(s, command);
    EXPECT_TRUE(blocked.collision);
    EXPECT_FALSE(p.check_primary_feasibility(s, blocked));
    // A trajectory entirely above a short roof remains feasible geometrically.
    p.set_urban_boxes({{25, 0, 4, 10, 100, 8}});
    EXPECT_FALSE(p.rollout_motion_primitive(s, command).collision);
}
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "urban_geometry_test");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
