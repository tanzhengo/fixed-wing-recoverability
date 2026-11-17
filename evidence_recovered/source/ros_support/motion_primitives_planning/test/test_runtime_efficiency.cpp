#include <gtest/gtest.h>
#include <ros/ros.h>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <unistd.h>
#include "../src/runtime_motion_primitives.h"

namespace {
using P = RuntimeMotionPrimitives;
std::string constant_model(size_t inputs, size_t outputs=1) {
    std::ostringstream out;
    out << "RECOVERY_MLP_V2 " << inputs << " 1 1 " << outputs << '\n';
    for(size_t i=0;i<inputs;++i)out<<"0 ";
    out<<"\n0\n0\n0\n";
    for(size_t i=0;i<outputs;++i)out<<"0 ";
    out<<'\n';for(size_t i=0;i<outputs;++i)out<<"0 ";
    return out.str();
}
TEST(RecoveryCompactLoader, SingleOutputSchemasAndInvalidInput) {
    for(size_t size:{28u,58u,103u}) {
        std::istringstream in(constant_model(size));LearnedRecoveryNetwork net;
        ASSERT_TRUE(net.load(in));EXPECT_EQ(net.feature_size(),size);EXPECT_EQ(net.output_size(),1u);
        std::array<float,2> out{{99,99}};
        EXPECT_TRUE(net.predict(std::vector<float>(size,0),out));EXPECT_EQ(out[0],0);EXPECT_EQ(out[1],0);
        EXPECT_FALSE(net.predict(std::vector<float>(size-1,0),out));
        std::vector<float> bad(size,0);bad[0]=std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(net.predict(bad,out));
    }
    for(const std::string& bad:{"RECOVERY_MLP_V1 28 1 1 2", "RECOVERY_MLP_V2 58 1 1 3", "RECOVERY_MLP_V2 58 0 1 1", "RECOVERY_MLP_V2 999999999 1 1 1"}) {
        std::istringstream in(bad);LearnedRecoveryNetwork net;EXPECT_FALSE(net.load(in));EXPECT_FALSE(net.ready());
    }
}
class RuntimeEfficiency : public ::testing::Test {
protected:
    std::unique_ptr<ros::NodeHandle> nh;std::string ns,model,compact;
    void SetUp() override {
        static unsigned serial=0;ns="/runtime_efficiency/case_"+std::to_string(++serial);
        nh.reset(new ros::NodeHandle(ns));
        ASSERT_TRUE(ros::param::get("/runtime_efficiency_model_path",model));
        ASSERT_TRUE(ros::param::get("/runtime_efficiency_compact_path",compact));
        nh->setParam("motion_primitive/horizon_time",3.0);nh->setParam("motion_primitive/V",15.5);
        nh->setParam("motion_primitive/num",29);nh->setParam("motion_primitive/vertical_num",7);
        nh->setParam("motion_primitive/min_altitude",16.0);nh->setParam("motion_primitive/max_altitude",24.0);
        nh->setParam("motion_primitive/state_relative_altitudes",true);
        nh->setParam("recovery/enabled",true);nh->setParam("recovery/require_terminal",true);
        nh->setParam("recovery/horizon_time",3.0);nh->setParam("recovery/max_candidates",12);
        nh->setParam("recovery/reuse_verified_plan",false);nh->setParam("recovery/stop_on_rejection",true);
        nh->setParam("recovery/candidate_order",std::string("learned"));nh->setParam("recovery/model_path",model);
        nh->setParam("recovery/state_only_features",true);nh->setParam("recovery/learned_work_power",0.0);
        nh->setParam("waypoints/num",1);nh->setParam("waypoints/wp1/x",250.0);
        nh->setParam("waypoints/wp1/y",0.0);nh->setParam("waypoints/wp1/h",20.0);
    }
    void TearDown() override {nh.reset();ros::param::del(ns);}
    std::unique_ptr<P> make(bool deferred,bool pruning) {
        nh->setParam("recovery/deferred_inference",deferred);nh->setParam("recovery/prune_dominated",pruning);
        return std::unique_ptr<P>(new P(*nh));
    }
    P::s_t trim(const P&p) const {
        P::s_t s;s.h=20;s.V=15.5;double lo=-20*Rad2Deg,hi=25*Rad2Deg;
        for(int i=0;i<60;++i){double a=(lo+hi)/2;
            if(1.86*s.V*s.V*(p.C_D(a)*std::tan(a)+p.C_L(a))-9.81>0)hi=a;else lo=a;}
        s.attack=(lo+hi)/2;s.T=1.86*s.V*s.V*p.C_D(s.attack)/std::cos(s.attack);return s;
    }
    static void command_equal(const P::pr_t&a,const P::pr_t&b) {
        EXPECT_EQ(a.w_roll_c,b.w_roll_c);EXPECT_EQ(a.attack_c,b.attack_c);EXPECT_EQ(a.T_c,b.T_c);EXPECT_EQ(a.t_way,b.t_way);
    }
    static void decision_equal(const P::selection_t&a,const P::selection_t&b) {
        EXPECT_EQ(a.status,b.status);EXPECT_EQ(a.recovery_verified,b.recovery_verified);
        EXPECT_EQ(a.primary_feasible,b.primary_feasible);EXPECT_EQ(a.selected_cost_rank,b.selected_cost_rank);
        EXPECT_EQ(a.cost,b.cost);command_equal(a.command,b.command);
        EXPECT_EQ(a.scheduled_count,b.scheduled_count);ASSERT_EQ(a.execution_plan.size(),b.execution_plan.size());
        for(size_t i=0;i<a.execution_plan.size();++i)command_equal(a.execution_plan[i],b.execution_plan[i]);
        if(a.recovery_verified) {
            command_equal(a.backup_command,b.backup_command);
            ASSERT_EQ(a.backup_rollout.states.size(),b.backup_rollout.states.size());
            for(size_t i=0;i<a.backup_rollout.states.size();++i) {
                const auto&x=a.backup_rollout.states[i];const auto&y=b.backup_rollout.states[i];
                EXPECT_EQ(x.pos[0],y.pos[0]);EXPECT_EQ(x.pos[1],y.pos[1]);EXPECT_EQ(x.h,y.h);
                EXPECT_EQ(x.V,y.V);EXPECT_EQ(x.sigma,y.sigma);EXPECT_EQ(x.gamma,y.gamma);
            }
        }
    }
    void obstacle(double x,double y,double r) {
        nh->setParam("static_obstacles/num",1);nh->setParam("static_obstacles/obs1/x",x);
        nh->setParam("static_obstacles/obs1/y",y);nh->setParam("static_obstacles/obs1/r",r);
        nh->setParam("static_obstacles/obs1/z",20.);nh->setParam("static_obstacles/obs1/height",100.);
    }
};
TEST_F(RuntimeEfficiency, RealCompactExportMatchesPaddedNetwork) {
    LearnedRecoveryNetwork a,b;ASSERT_TRUE(a.load(model));ASSERT_TRUE(b.load(compact));
    ASSERT_EQ(a.feature_size(),103u);ASSERT_EQ(b.feature_size(),28u);ASSERT_EQ(b.output_size(),1u);
    for(int seed=0;seed<128;++seed) {
        std::vector<float> full(103,0),shorter(28);for(int i=0;i<28;++i)full[i]=shorter[i]=std::sin(.31*(seed+1)*(i+1));
        std::array<float,2> x,y;ASSERT_TRUE(a.predict(full,x));ASSERT_TRUE(b.predict(shorter,y));
        EXPECT_EQ(x[0],y[0]);
    }
}
TEST_F(RuntimeEfficiency, CheapestWitnessSkipsNeuralInference) {
    auto eager=make(false,false),deferred=make(true,true);auto s=trim(*eager);
    const auto a=eager->select_planning_proposal(s),b=deferred->select_planning_proposal(s);
    ASSERT_TRUE(a.recovery_verified);ASSERT_EQ(a.selected_cost_rank,1);decision_equal(a,b);
    EXPECT_EQ(a.inference_batches,1);EXPECT_GT(a.inference_candidates,0);
    EXPECT_EQ(b.inference_batches,0);EXPECT_EQ(b.feature_candidates,0);EXPECT_EQ(b.recovery_checked,1);
    EXPECT_EQ(a.backup_integration_steps,b.backup_integration_steps);
    nh->setParam("recovery/model_path",compact);auto compact_deferred=make(true,true);
    decision_equal(a,compact_deferred->select_planning_proposal(s));
}
TEST_F(RuntimeEfficiency, NoFeasiblePrimaryDoesNotInfer) {
    obstacle(0,0,100);auto p=make(true,true);const auto a=p->select_planning_proposal(trim(*p));
    EXPECT_EQ(a.primary_feasible_count,0);EXPECT_EQ(a.inference_batches,0);EXPECT_EQ(a.recovery_checked,0);
    EXPECT_EQ(a.status,"NO_FEASIBLE_PRIMARY");
}
TEST_F(RuntimeEfficiency, ExactChoicesAndWitnessesAcrossMatchedInputs) {
    int pruned=0,inferred=0,verified=0;
    for(const std::string mode:{"learned","diverse"}) {
        nh->setParam("recovery/candidate_order",mode);
        for(int scene=0;scene<9;++scene) {
            obstacle(60.+5*scene,(scene%3-1)*9.,10.+(scene%4)*3.);
            auto eager=make(false,false),deferred=make(true,false),optimized=make(true,true);
            for(int state=0;state<5;++state) {
                auto s=trim(*eager);s.sigma=(state-2)*.10f;s.roll=(state%3-1)*.12f;s.h=18+state;
                const auto a=eager->select_planning_proposal(s),b=deferred->select_planning_proposal(s),c=optimized->select_planning_proposal(s);
                SCOPED_TRACE(mode+" scene="+std::to_string(scene)+" state="+std::to_string(state));
                decision_equal(a,b);decision_equal(a,c);
                EXPECT_EQ(a.backup_integration_steps,b.backup_integration_steps);
                EXPECT_LE(c.backup_integration_steps,a.backup_integration_steps);
                EXPECT_LE(c.recovery_checked+c.pruned_candidates,c.scheduled_count);
                if(!c.recovery_verified || c.selected_cost_rank != 1)
                    EXPECT_EQ(c.recovery_checked+c.pruned_candidates,c.scheduled_count);
                EXPECT_EQ(a.recovery_checked,b.recovery_checked);
                EXPECT_LE(c.inference_batches,a.inference_batches);
                pruned+=c.pruned_candidates;inferred+=c.inference_batches;verified+=c.recovery_verified;
            }
        }
    }
    EXPECT_GT(pruned,0);EXPECT_GT(inferred,0);EXPECT_GT(verified,0);
    RecordProperty("pruned_candidates",pruned);RecordProperty("inference_batches",inferred);
}
TEST_F(RuntimeEfficiency, OnlyVerifiedBoundsCanPruneAndEqualCostsKeepStableRank) {
    P::selection_t bound;bound.cost=1;bound.selected_cost_rank=1;
    EXPECT_FALSE(P::candidate_cannot_improve(100,12,bound));
    bound.recovery_verified=true;bound.selected_cost_rank=5;
    EXPECT_FALSE(P::candidate_cannot_improve(1,4,bound));
    EXPECT_TRUE(P::candidate_cannot_improve(1,5,bound));
    EXPECT_TRUE(P::candidate_cannot_improve(1,6,bound));
    EXPECT_TRUE(P::candidate_cannot_improve(2,1,bound));
    EXPECT_FALSE(P::candidate_cannot_improve(.5,12,bound));
}
TEST_F(RuntimeEfficiency, BindingWorkBudgetCountsSeedOnceAndSkipsInferenceOnExhaustion) {
    obstacle(65,0,18);nh->setParam("recovery/max_backup_integration_steps",1);
    auto eager=make(false,false),deferred=make(true,true);auto s=trim(*eager);
    const auto a=eager->select_planning_proposal(s),b=deferred->select_planning_proposal(s);
    EXPECT_FALSE(b.recovery_verified);EXPECT_TRUE(b.backup_work_budget_exhausted);
    EXPECT_LE(b.backup_integration_steps,1);EXPECT_EQ(b.recovery_checked,1);
    EXPECT_EQ(b.inference_batches,0);EXPECT_EQ(a.backup_integration_steps,b.backup_integration_steps);
    EXPECT_EQ(a.selected_cost_rank,b.selected_cost_rank);
}
TEST_F(RuntimeEfficiency, MissingObservedMapFallsBackAndSingleHeadRejectsWorkUtility) {
    nh->setParam("recovery/candidate_order",std::string("observed_coverage"));auto p=make(true,true);
    EXPECT_EQ(p->select_planning_proposal(trim(*p)).candidate_order,"diverse");
    nh->setParam("recovery/candidate_order",std::string("learned"));nh->setParam("recovery/model_path",compact);
    nh->setParam("recovery/learned_work_power",1.0);EXPECT_THROW({P q(*nh);},std::invalid_argument);
}
TEST_F(RuntimeEfficiency, ObservedHeuristicAndLearnedCoverageAreDeterministic) {
    nh->setParam("sensor_map/enabled",true);nh->setParam("recovery/strict_execution",true);
    nh->setParam("recovery/candidate_order",std::string("observed_coverage"));
    nh->setParam("recovery/coverage_weight",0.0);auto observed=make(true,true);
    nh->setParam("recovery/candidate_order",std::string("diverse"));auto diverse=make(true,true);
    auto s=trim(*observed);std::vector<P::recovery_candidate_t> c(20);
    for(size_t i=0;i<c.size();++i){c[i].terminal=s;c[i].terminal.sigma=.12*(i%7);c[i].terminal.h=18+i%5;c[i].cost=i/2;}
    P::selection_t diagnostics;const auto a=observed->recovery_candidate_order(s,c,&diagnostics);
    EXPECT_EQ(a,diverse->recovery_candidate_order(s,c));EXPECT_EQ(diagnostics.inference_batches,0);
    EXPECT_EQ(diagnostics.observed_feature_candidates,20);EXPECT_EQ(a.front(),0u);EXPECT_EQ(a.size(),12u);
    nh->setParam("recovery/candidate_order",std::string("learned_coverage"));nh->setParam("recovery/coverage_weight",1.0);
    auto coverage=make(true,true);nh->setParam("recovery/candidate_order",std::string("learned"));auto learned=make(true,true);
    const auto cov=coverage->recovery_candidate_order(s,c);EXPECT_EQ(cov,learned->recovery_candidate_order(s,c));
    EXPECT_EQ(cov,coverage->recovery_candidate_order(s,c));
}
TEST_F(RuntimeEfficiency, Observed58SchemaUsesRealTriStateMapAndRejectsMissingMap) {
    char path[]="/tmp/runtime-observed58-XXXXXX";const int fd=mkstemp(path);ASSERT_GE(fd,0);
    const auto text=constant_model(58);ASSERT_EQ(write(fd,text.data(),text.size()),static_cast<ssize_t>(text.size()));close(fd);
    nh->setParam("recovery/model_path",std::string(path));auto unavailable=make(true,true);
    auto s=trim(*unavailable);P::recovery_candidate_t c;c.terminal=s;c.cost=1;
    EXPECT_EQ(unavailable->select_planning_proposal(s).candidate_order,"diverse");
    EXPECT_THROW(unavailable->recovery_features(s,c,58),std::invalid_argument);
    nh->setParam("sensor_map/enabled",true);nh->setParam("recovery/strict_execution",true);
    auto observed=make(true,true);unlink(path);
    const auto feature=observed->recovery_features(s,c);ASSERT_EQ(feature.size(),58u);
    const auto base=observed->recovery_features(s,c,28);
    for(size_t i=0;i<28;++i)EXPECT_EQ(feature[i],base[i]);
    for(size_t i=28;i<58;i+=2){EXPECT_EQ(feature[i],0);EXPECT_EQ(feature[i+1],1);}
    P::selection_t counts;std::vector<P::recovery_candidate_t> candidates(3,c);
    const auto schedule=observed->recovery_candidate_order(s,candidates,&counts);
    EXPECT_EQ(schedule,(std::vector<size_t>{0,1,2}));
    EXPECT_EQ(counts.inference_batches,1);EXPECT_EQ(counts.inference_candidates,3);
    EXPECT_EQ(counts.feature_candidates,3);EXPECT_EQ(counts.observed_feature_candidates,3);
}
} // namespace
int main(int argc,char**argv) {
    ros::init(argc,argv,"runtime_efficiency_test");::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
