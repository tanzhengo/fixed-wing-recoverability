#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <unistd.h>
#include "../src/motion_primitives.h"

namespace {
// A known linear map implemented by signed ReLU channels: output0 = feature13,
// output1 = 0. This independently exercises matrix orientation and activations.
std::string linear_model(float bias = 0.0f) {
    std::ostringstream out;
    out << "RECOVERY_MLP_V1 103 2 2 2\n";
    for (int row = 0; row < 2; ++row) for (int col = 0; col < 103; ++col)
        out << (col == 13 ? (row == 0 ? 1 : -1) : 0) << ' ';
    out << "\n0 0\n1 0 0 1\n0 0\n1 -1 0 0\n" << bias << " 0\n";
    return out.str();
}
TEST(LearnedNetwork, DenseForwardKnownSignedLinearMap) {
    std::istringstream stream(linear_model());
    LearnedRecoveryNetwork network;
    ASSERT_TRUE(network.load(stream));
    std::vector<float> x(103, 0.0f);
    std::array<float, 2> y;
    for (float value : {-3.5f, 0.0f, 2.75f}) {
        x[13] = value;
        ASSERT_TRUE(network.predict(x, y));
        EXPECT_FLOAT_EQ(y[0], value);
        EXPECT_FLOAT_EQ(y[1], 0.0f);
    }
    x[0] = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(network.predict(x, y));
    EXPECT_FALSE(network.predict(std::vector<float>(102), y));
}
TEST(LearnedNetwork, RejectsMalformedAndPartialModels) {
    LearnedRecoveryNetwork network;
    for (const auto& text : {std::string("RECOVERY_MLP_V1 102 2 2 2\n"),
                            std::string("RECOVERY_MLP_V1 103 100000 2 2\n"),
                            linear_model().substr(0, 80), linear_model() + "extra"}) {
        std::istringstream stream(text);
        EXPECT_FALSE(network.load(stream));
        EXPECT_FALSE(network.ready());
    }
}
TEST(LearnedNetwork, ProbabilityStableAtExtremeLogits) {
    EXPECT_FLOAT_EQ(LearnedRecoveryNetwork::probability(1000), 1.0f);
    EXPECT_FLOAT_EQ(LearnedRecoveryNetwork::probability(-1000), 0.0f);
    EXPECT_FLOAT_EQ(LearnedRecoveryNetwork::probability(0), .5f);
    EXPECT_FLOAT_EQ(LearnedRecoveryNetwork::predicted_steps(-2), 0.0f);
    EXPECT_NEAR(LearnedRecoveryNetwork::predicted_steps(std::log(1001.0f)), 1000, .001);
}

class LearnedRecovery : public ::testing::Test {
protected:
    using MP = MotionPrimitives;
    std::unique_ptr<ros::NodeHandle> nh;
    std::string ns, model_path;
    void SetUp() override {
        static unsigned count = 0;
        ns = "/learned_recovery_test/case_" + std::to_string(++count);
        nh.reset(new ros::NodeHandle(ns));
        model_path = "/tmp/learned_recovery_" + std::to_string(getpid()) + "_" + std::to_string(count) + ".txt";
        std::ofstream model(model_path); model << linear_model(); model.close();
        nh->setParam("recovery/enabled", true);
        nh->setParam("recovery/candidate_order", "learned");
        nh->setParam("recovery/model_path", model_path);
        nh->setParam("recovery/horizon_time", 6.0);
        nh->setParam("recovery/max_candidates", 3);
        nh->setParam("recovery/stop_on_rejection", true);
        nh->setParam("motion_primitive/min_altitude", 1.0);
        nh->setParam("motion_primitive/max_altitude", 80.0);
        nh->setParam("waypoints/wp1/x", 250.0);
        nh->setParam("waypoints/wp1/y", 0.0);
        nh->setParam("waypoints/wp1/h", 20.0);
    }
    void TearDown() override { nh.reset(); ros::param::del(ns); std::remove(model_path.c_str()); }
    MP::s_t state(const MP& p) {
        MP::s_t s; s.V = 15.5; s.h = 20;
        double lo=-20*M_PI/180,hi=25*M_PI/180;
        for(int i=0;i<60;++i){double a=(lo+hi)/2;
            double r=1.86*s.V*s.V*(p.C_D(a)*std::tan(a)+p.C_L(a))-9.81;
            if(r>0)hi=a;else lo=a;}
        s.attack=(lo+hi)/2;s.T=1.86*s.V*s.V*p.C_D(s.attack)/std::cos(s.attack);return s;
    }
};
TEST_F(LearnedRecovery, LearnedRankingRetainsCheapestAndBudget) {
    MP p(*nh); auto s=state(p);
    std::vector<MP::recovery_candidate_t> candidates(4);
    for (int i=0;i<4;++i) { candidates[i].terminal=s; candidates[i].cost=i; }
    candidates[1].terminal.pos[1]=10;
    candidates[2].terminal.pos[1]=30;
    candidates[3].terminal.pos[1]=20;
    const auto order=p.recovery_candidate_order(s,candidates);
    ASSERT_EQ(order.size(),3u);
    EXPECT_EQ(order[0],0u); EXPECT_EQ(order[1],2u); EXPECT_EQ(order[2],3u);
    EXPECT_EQ(std::set<size_t>(order.begin(),order.end()).size(),order.size());
}
TEST_F(LearnedRecovery, NetworkCannotBypassExplicitWorkBudget) {
    std::ofstream model(model_path); model<<linear_model(100); model.close();
    nh->setParam("recovery/max_backup_integration_steps",1);
    MP p(*nh); auto s=state(p); auto d=p.select_motion_primitive(s);
    EXPECT_TRUE(d.primary_feasible);
    EXPECT_FALSE(d.recovery_verified);
    EXPECT_TRUE(d.backup_work_budget_exhausted);
    EXPECT_LE(d.backup_integration_steps,1);
    EXPECT_EQ(d.candidate_order,"learned");
}
TEST_F(LearnedRecovery, MissingModelFallsBackExplicitly) {
    nh->setParam("recovery/model_path",model_path+".missing");
    MP p(*nh); auto s=state(p); auto d=p.select_motion_primitive(s);
    EXPECT_EQ(d.candidate_order,"diverse");
}
TEST_F(LearnedRecovery, FeatureGeometryUsesInflatedFiniteVolumes) {
    nh->setParam("static_obstacles/num",1);
    nh->setParam("static_obstacles/obs1/x",10.0);
    nh->setParam("static_obstacles/obs1/y",0.0);
    nh->setParam("static_obstacles/obs1/z",20.0);
    nh->setParam("static_obstacles/obs1/r",5.0);
    nh->setParam("static_obstacles/obs1/height",10.0);
    MP p(*nh); auto s=state(p);
    EXPECT_NEAR(p.obstacle_signed_distance(10,0,20),-6,.0001);
    EXPECT_NEAR(p.obstacle_signed_distance(17,0,20),1,.0001);
    EXPECT_NEAR(p.obstacle_signed_distance(10,0,27),1,.0001);
    EXPECT_FALSE(p.collision_check(10,0,27));
    MP::recovery_candidate_t candidate;candidate.terminal=s;candidate.cost=1;
    auto feature=p.recovery_features(s,candidate);
    ASSERT_EQ(feature.size(),103u);
    for(float f:feature) EXPECT_TRUE(std::isfinite(f));
    // center-level probe at forward=0, lateral=0: x=0, y=0, z=20.
    EXPECT_NEAR(feature[28+2*3+1],4.0/30.0,1e-6);
}
TEST_F(LearnedRecovery, FeatureEncodingIsRigidTransformInvariant) {
    nh->setParam("static_obstacles/num",1);
    nh->setParam("static_obstacles/obs1/x",70.0);
    nh->setParam("static_obstacles/obs1/y",10.0);
    nh->setParam("static_obstacles/obs1/z",20.0);
    nh->setParam("static_obstacles/obs1/r",5.0);
    nh->setParam("static_obstacles/obs1/height",10.0);
    MP first(*nh);auto a=state(first);a.pos[0]=5;a.pos[1]=-3;a.sigma=.2;
    MP::recovery_candidate_t c;c.terminal=a;c.terminal.pos[0]=40;c.terminal.pos[1]=7;
    c.terminal.sigma=.4;c.cost=2;
    const auto original=first.recovery_features(a,c);
    // Rotate pi/2, then translate (100,-20), including map and goal.
    nh->setParam("static_obstacles/obs1/x",90.0);
    nh->setParam("static_obstacles/obs1/y",50.0);
    nh->setParam("waypoints/wp1/x",100.0);
    nh->setParam("waypoints/wp1/y",230.0);
    MP second(*nh);
    const auto transform=[](MP::s_t& t){float x=t.pos[0],y=t.pos[1];
        t.pos[0]=100-y;t.pos[1]=-20+x;t.sigma+=M_PI/2;};
    transform(a);transform(c.terminal);
    const auto transformed=second.recovery_features(a,c);
    ASSERT_EQ(original.size(),transformed.size());
    for(size_t i=0;i<original.size();++i) EXPECT_NEAR(original[i],transformed[i],2e-6) << i;
}
TEST_F(LearnedRecovery, DistanceSignsMatchAllObstacleTypesAndPrediction) {
    nh->setParam("sphere_obstacles/num",1);
    nh->setParam("sphere_obstacles/sphere1/x",40.0);
    nh->setParam("sphere_obstacles/sphere1/y",0.0);
    nh->setParam("sphere_obstacles/sphere1/z",20.0);
    nh->setParam("sphere_obstacles/sphere1/r",3.0);
    nh->setParam("horizontal_obstacles/num",1);
    nh->setParam("horizontal_obstacles/cylinder1/x",80.0);
    nh->setParam("horizontal_obstacles/cylinder1/y",0.0);
    nh->setParam("horizontal_obstacles/cylinder1/z",20.0);
    nh->setParam("horizontal_obstacles/cylinder1/r",3.0);
    nh->setParam("horizontal_obstacles/cylinder1/length",10.0);
    nh->setParam("horizontal_obstacles/cylinder1/yaw",30.0);
    nh->setParam("dynamic_obstacles/num",1);
    nh->setParam("dynamic_obstacles/dyn_obs1/x",120.0);
    nh->setParam("dynamic_obstacles/dyn_obs1/y",0.0);
    nh->setParam("dynamic_obstacles/dyn_obs1/z",20.0);
    nh->setParam("dynamic_obstacles/dyn_obs1/r",3.0);
    nh->setParam("dynamic_obstacles/dyn_obs1/height",10.0);
    nh->setParam("dynamic_obstacles/dyn_obs1/type","linear");
    nh->setParam("dynamic_obstacles/dyn_obs1/velocity",2.0);
    nh->setParam("dynamic_obstacles/dyn_obs1/direction",0.0);
    MP p(*nh);
    for(float t:{0.0f,3.0f})for(float x=30.5f;x<139;x+=2.0f)
        for(float y=-9.5f;y<10;y+=2.0f)for(float z:{12.3f,20.3f,28.3f})
            EXPECT_EQ(p.obstacle_signed_distance(x,y,z,t)<=0,p.collision_check(x,y,z,t));
    EXPECT_LT(p.obstacle_signed_distance(126,0,20,3),0);
    EXPECT_GT(p.obstacle_signed_distance(126,0,20,0),0);
}
TEST_F(LearnedRecovery, HybridExploresBetweenLearnedChoices) {
    nh->setParam("recovery/candidate_order","learned_diverse");
    nh->setParam("recovery/max_candidates",4);
    MP p(*nh);auto s=state(p);
    std::vector<MP::recovery_candidate_t> c(4);
    for(int i=0;i<4;++i){c[i].terminal=s;c[i].cost=i;}
    c[1].terminal.pos[1]=10;c[1].terminal.sigma=-.7;
    c[2].terminal.pos[1]=30;c[2].terminal.sigma=.05;
    c[3].terminal.pos[1]=20;c[3].terminal.sigma=.1;
    auto order=p.recovery_candidate_order(s,c);
    ASSERT_EQ(order.size(),4u);
    EXPECT_EQ(order[0],0u);EXPECT_EQ(order[1],2u);EXPECT_EQ(order[2],1u);EXPECT_EQ(order[3],3u);
}
}
int main(int argc,char** argv){ros::init(argc,argv,"learned_recovery_test");::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
