#include <gtest/gtest.h>
#include <ros/ros.h>
#include <memory>
#include "../src/runtime_motion_primitives.h"

namespace {
using Planner = RuntimeMotionPrimitives;

class RuntimeExecution : public ::testing::Test {
protected:
    std::unique_ptr<ros::NodeHandle> nh;
    std::string ns;
    void SetUp() override {
        static unsigned serial = 0;
        ns = "/runtime_execution/case_" + std::to_string(++serial);
        nh.reset(new ros::NodeHandle(ns));
        nh->setParam("recovery/enabled", true);
        nh->setParam("recovery/require_terminal", true);
        nh->setParam("recovery/strict_execution", true);
        nh->setParam("recovery/reuse_verified_plan", false); // Strict execution enables it.
        nh->setParam("recovery/max_candidates", 29);
        nh->setParam("motion_primitive/vertical_num", 1);
        nh->setParam("waypoints/wp1/x", 250.0);
        nh->setParam("waypoints/wp1/y", 0.0);
        nh->setParam("waypoints/wp1/h", 20.0);
    }
    void TearDown() override { nh.reset(); ros::param::del(ns); }
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
    Planner::pr_t hold(const Planner::s_t& state, float duration) const {
        Planner::pr_t command;
        command.w_roll_c = state.roll; command.attack_c = state.attack;
        command.T_c = state.T; command.t_way = duration;
        return command;
    }
    void expect_same(const Planner::s_t& a, const Planner::s_t& b) const {
        EXPECT_NEAR(a.pos[0], b.pos[0], 1e-5);
        EXPECT_NEAR(a.pos[1], b.pos[1], 1e-5);
        EXPECT_NEAR(a.h, b.h, 1e-5);
        EXPECT_NEAR(a.V, b.V, 1e-5);
        EXPECT_NEAR(a.roll, b.roll, 1e-6);
        EXPECT_NEAR(a.attack, b.attack, 1e-6);
        EXPECT_NEAR(a.T, b.T, 1e-6);
    }
};

TEST_F(RuntimeExecution, StrictModeRejectsConflictingAblation) {
    nh->setParam("recovery/require_terminal", false);
    EXPECT_THROW({ Planner planner(*nh); }, std::invalid_argument);
}

TEST_F(RuntimeExecution, UnverifiedProposalCannotEraseRunningWitness) {
    Planner planner(*nh); const auto initial = trim(planner);
    const auto verified = planner.select_motion_primitive(initial);
    ASSERT_TRUE(verified.recovery_verified);
    ASSERT_TRUE(planner.selection_is_executable(verified));
    planner.accept_selection(verified);
    const float duration = planner.remaining_execution_time();
    ASSERT_GT(duration, 0);
    auto rejected = verified;
    rejected.status = "BUDGET_EXHAUSTED";
    rejected.recovery_verified = false;
    rejected.execution_plan.clear();
    EXPECT_FALSE(planner.selection_is_executable(rejected));
    planner.accept_selection(rejected);
    EXPECT_FLOAT_EQ(duration, planner.remaining_execution_time());
    ASSERT_TRUE(planner.prepare_execution(initial, rejected));
    EXPECT_EQ("REVALIDATED_PLAN", rejected.status);
    EXPECT_TRUE(rejected.reused_verified_plan);
    EXPECT_FALSE(rejected.recovery_verified);
}

TEST_F(RuntimeExecution, NoWitnessIsExplicitlyRejected) {
    Planner planner(*nh); const auto initial = trim(planner);
    auto selection = planner.select_motion_primitive(initial);
    ASSERT_TRUE(selection.recovery_verified);
    selection.status = "BUDGET_EXHAUSTED";
    selection.recovery_verified = false;
    selection.execution_plan.clear();
    EXPECT_FALSE(planner.prepare_execution(initial, selection));
    EXPECT_EQ("EXECUTION_REJECTED", selection.status);
    EXPECT_FALSE(selection.primary_feasible);
    EXPECT_FALSE(planner.selection_is_executable(selection));
}

TEST_F(RuntimeExecution, DispatchRecheckRejectsChangedWorldAndCachedPlan) {
    Planner planner(*nh); const auto initial = trim(planner);
    auto selection = planner.select_motion_primitive(initial);
    ASSERT_TRUE(selection.recovery_verified);
    planner.accept_selection(selection);
    Planner::box_obstacle_t obstacle;
    obstacle.geometry = {initial.pos[0], initial.pos[1], initial.h, 1, 1, 1};
    obstacle.clearance = 0;
    planner.set_box_obstacles({obstacle});
    EXPECT_FALSE(planner.prepare_execution(initial, selection));
    EXPECT_EQ("EXECUTION_REJECTED", selection.status);
    EXPECT_TRUE(planner.remaining_execution_plan().empty());
}

TEST_F(RuntimeExecution, RevalidatedPlanDoesNotRegainElapsedDuration) {
    Planner planner(*nh); auto state = trim(planner);
    auto selection = planner.select_motion_primitive(state);
    ASSERT_TRUE(selection.recovery_verified);
    planner.accept_selection(selection);
    const float original = planner.remaining_execution_time();
    auto command = selection.command; command.t_way = .2;
    state = planner.rollout_motion_primitive(state, command).states.back();
    planner.advance_execution(.2);
    selection.status = "BUDGET_EXHAUSTED";
    selection.recovery_verified = false;
    selection.execution_plan.clear();
    ASSERT_TRUE(planner.prepare_execution(state, selection));
    planner.accept_selection(selection);
    EXPECT_NEAR(original - .2, planner.remaining_execution_time(), 1e-5);
}

TEST_F(RuntimeExecution, DelayPredictionUsesEveryAppliedControlInterval) {
    Planner planner(*nh); const auto initial = trim(planner);
    auto a = hold(initial, .04); auto b = a; b.w_roll_c = .2; b.t_way = .08;
    planner.record_applied_command(a, ros::Time(.9));
    planner.record_applied_command(b, ros::Time(1.04));
    const auto at_switch = planner.rollout_motion_primitive(initial, a).states.back();
    const auto expected = planner.rollout_motion_primitive(at_switch, b).states.back();
    Planner::s_t predicted;
    ASSERT_TRUE(planner.predict_state_at(initial, ros::Time(1.0), ros::Time(1.12), predicted));
    expect_same(expected, predicted);
}

TEST_F(RuntimeExecution, ShortPrimaryTailCanHandOffToRecheckedBackup) {
    Planner planner(*nh); const auto initial = trim(planner);
    const auto selection = planner.select_motion_primitive(initial);
    ASSERT_TRUE(selection.recovery_verified);
    ASSERT_EQ(selection.execution_plan.size(), 2u);
    planner.accept_selection(selection);
    auto applied = selection.command; applied.t_way -= .05;
    const auto actual = planner.rollout_motion_primitive(initial, applied).states.back();
    planner.advance_execution(applied.t_way);
    ASSERT_LT(planner.remaining_execution_plan().front().t_way, .1);
    const auto reused = planner.revalidate_cached_plan(actual);
    ASSERT_TRUE(reused.reused_verified_plan);
    ASSERT_EQ(reused.execution_plan.size(), 1u);
    EXPECT_FLOAT_EQ(reused.command.w_roll_c, selection.backup_command.w_roll_c);
    EXPECT_FLOAT_EQ(reused.command.attack_c, selection.backup_command.attack_c);
    EXPECT_NEAR(reused.plan_remaining_time, selection.backup_command.t_way, 1e-5);
    EXPECT_TRUE(planner.check_primary_feasibility(actual, reused.rollout));
}

TEST_F(RuntimeExecution, StartupBeforeFirstCommandUsesPluginZeroControl) {
    Planner planner(*nh); const auto initial = trim(planner);
    auto first = hold(initial, .04);
    planner.record_applied_command(first, ros::Time(1.04));
    Planner::pr_t startup; startup.t_way = .04;
    const auto before_command = planner.rollout_motion_primitive(initial, startup).states.back();
    const auto expected = planner.rollout_motion_primitive(before_command, first).states.back();
    Planner::s_t predicted;
    ASSERT_TRUE(planner.predict_state_at(initial, ros::Time(1.0), ros::Time(1.08), predicted));
    expect_same(expected, predicted);
}

TEST_F(RuntimeExecution, MissingFutureOrStaleTimestampsAreRejected) {
    Planner planner(*nh); const auto initial = trim(planner);
    Planner::s_t predicted;
    EXPECT_FALSE(planner.predict_state_at(initial, ros::Time(0), ros::Time(.1), predicted));
    EXPECT_FALSE(planner.predict_state_at(initial, ros::Time(1.1), ros::Time(1.0), predicted));
    EXPECT_FALSE(planner.predict_state_at(initial, ros::Time(1.0), ros::Time(1.501), predicted));
    ASSERT_TRUE(planner.predict_state_at(initial, ros::Time(1.0), ros::Time(1.0), predicted));
    expect_same(initial, predicted);
}

TEST_F(RuntimeExecution, ExplicitAblationStillAcceptsPrimaryOnlySelection) {
    nh->setParam("recovery/strict_execution", false);
    nh->setParam("recovery/require_terminal", false);
    Planner planner(*nh); const auto initial = trim(planner);
    const auto selection = planner.select_motion_primitive(initial);
    EXPECT_EQ("PRIMARY_ONLY_FEASIBLE", selection.status);
    EXPECT_TRUE(planner.selection_is_executable(selection));
}
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "runtime_execution_test");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
