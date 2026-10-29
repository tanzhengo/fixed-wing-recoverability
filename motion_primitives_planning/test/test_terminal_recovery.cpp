#include <gtest/gtest.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "../src/motion_primitives.h"

namespace {

class TerminalRecovery : public ::testing::Test {
 protected:
  using Planner = MotionPrimitives;

  void SetUp() override {
    static unsigned int instance = 0;
    ns_ = "/terminal_recovery_test/case_" + std::to_string(++instance);
    nh_.reset(new ros::NodeHandle(ns_));
    nh_->setParam("motion_primitive/horizon_time", 3.0);
    nh_->setParam("motion_primitive/V", 15.5);
    nh_->setParam("motion_primitive/num", 29);
    nh_->setParam("motion_primitive/vertical_num", 1);
    nh_->setParam("motion_primitive/theata", 45.0);
    nh_->setParam("motion_primitive/min_altitude", 1.0);
    nh_->setParam("motion_primitive/max_altitude", 80.0);
    nh_->setParam("motion_primitive/integration_dt", 0.02);
    nh_->setParam("motion_primitive/min_speed", 3.0);
    nh_->setParam("motion_primitive/max_speed", 45.0);
    nh_->setParam("motion_primitive/max_roll_angle", 35.0);
    nh_->setParam("motion_primitive/max_attack_angle", 25.0);
    nh_->setParam("motion_primitive/min_attack_angle", -20.0);
    nh_->setParam("motion_primitive/max_thrust", 20.0);
    nh_->setParam("motion_primitive/compensate_tau", 0.8);
    nh_->setParam("static_obstacles/num", 0);
    nh_->setParam("sphere_obstacles/num", 0);
    nh_->setParam("horizontal_obstacles/num", 0);
    nh_->setParam("dynamic_obstacles/num", 0);
    nh_->setParam("waypoints/num", 1);
    nh_->setParam("waypoints/wp1/x", 500.0);
    nh_->setParam("waypoints/wp1/y", 0.0);
    nh_->setParam("waypoints/wp1/h", 20.0);
    nh_->setParam("recovery/enabled", true);
    nh_->setParam("recovery/horizon_time", 3.0);
    nh_->setParam("recovery/max_candidates", 29);
    nh_->setParam("recovery/known_space_radius", 0.0);
  }

  void TearDown() override {
    nh_.reset();
    ros::param::del(ns_);
  }

  std::unique_ptr<Planner> planner() const {
    return std::unique_ptr<Planner>(new Planner(*nh_));
  }

  Planner::s_t state() const {
    Planner::s_t s;
    s.V = 15.5;
    s.h = 20.0;
    s.T = 0.93;
    s.attack = -0.20;
    s.q[0] = std::cos(0.10);
    s.q[2] = std::sin(0.10);
    return s;
  }

  Planner::pr_t straight(Planner& mp, const Planner::s_t& s) const {
    const auto library = mp.generate_library(s);
    const auto found = std::min_element(library.begin(), library.end(),
        [](const Planner::pr_t& a, const Planner::pr_t& b) {
          return std::abs(a.wp.theata) < std::abs(b.wp.theata);
        });
    return *found;
  }

  void blockingCylinder(double x = 80.0, double radius = 25.0) {
    nh_->setParam("static_obstacles/num", 1);
    nh_->setParam("static_obstacles/obs1/x", x);
    nh_->setParam("static_obstacles/obs1/y", 0.0);
    nh_->setParam("static_obstacles/obs1/z", 40.0);
    nh_->setParam("static_obstacles/obs1/r", radius);
    nh_->setParam("static_obstacles/obs1/height", 80.0);
  }

  static void expectSameState(const Planner::s_t& a, const Planner::s_t& b) {
    EXPECT_FLOAT_EQ(a.pos[0], b.pos[0]);
    EXPECT_FLOAT_EQ(a.pos[1], b.pos[1]);
    EXPECT_FLOAT_EQ(a.h, b.h);
    EXPECT_FLOAT_EQ(a.V, b.V);
    EXPECT_FLOAT_EQ(a.gamma, b.gamma);
    EXPECT_FLOAT_EQ(a.sigma, b.sigma);
    EXPECT_FLOAT_EQ(a.d_sigma, b.d_sigma);
    EXPECT_FLOAT_EQ(a.roll, b.roll);
    EXPECT_FLOAT_EQ(a.attack, b.attack);
    EXPECT_FLOAT_EQ(a.T, b.T);
  }

  std::string ns_;
  std::unique_ptr<ros::NodeHandle> nh_;
};

TEST_F(TerminalRecovery, OpenSpaceHasVerifiedContinuationWithFullStateContinuity) {
  auto mp = planner();
  auto initial = state();
  // A nonzero actuator state catches restarting the backup from a trim state.
  initial.roll = 0.10;
  std::vector<Planner::rollout_t> predictions;
  const auto result = mp->select_motion_primitive(initial, &predictions);
  ASSERT_TRUE(result.primary_feasible);
  ASSERT_TRUE(result.recovery_verified);
  EXPECT_EQ("RECOVERY_VERIFIED", result.status);
  EXPECT_FALSE(result.budget_exhausted);
  ASSERT_FALSE(result.rollout.states.empty());
  ASSERT_FALSE(result.backup_rollout.states.empty());
  EXPECT_FALSE(result.rollout.collision);
  EXPECT_FALSE(result.backup_rollout.collision);
  EXPECT_TRUE(result.backup_rollout.valid);
  expectSameState(result.rollout.states.back(), result.backup_rollout.states.front());
  const auto replay = mp->rollout_motion_primitive(result.rollout.states.back(),
      result.backup_command, result.command.t_way);
  ASSERT_EQ(replay.states.size(), result.backup_rollout.states.size());
  expectSameState(replay.states.back(), result.backup_rollout.states.back());
  int primary_steps = 0;
  for (const auto& prediction : predictions)
    primary_steps += static_cast<int>(prediction.states.size()) - 1;
  EXPECT_EQ(primary_steps, result.primary_integration_steps);
  EXPECT_GE(result.backup_integration_steps, static_cast<int>(replay.states.size()) - 1);
  EXPECT_GT(result.backup_integration_steps, 0);
}

TEST_F(TerminalRecovery, SafePrimaryCanHaveNoVerifiedContinuation) {
  blockingCylinder();
  auto mp = planner();
  const auto initial = state();
  const auto command = straight(*mp, initial);
  const auto primary = mp->rollout_motion_primitive(initial, command);
  ASSERT_TRUE(primary.valid);
  ASSERT_FALSE(primary.collision)
      << "The obstacle must lie beyond the complete first segment";
  const auto recovery = mp->check_terminal_recovery(initial, primary, command.t_way);
  EXPECT_FALSE(recovery.verified);
  EXPECT_GT(recovery.checked, 0);
}

TEST_F(TerminalRecovery, SelectionRejectsShortSightedWinnerForVerifiedTurn) {
  blockingCylinder();
  nh_->setParam("recovery/enabled", false);
  auto baseline = planner();
  const auto initial = state();
  const auto short_sighted = baseline->select_motion_primitive(initial);
  ASSERT_TRUE(short_sighted.primary_feasible);
  nh_->setParam("recovery/enabled", true);
  auto guarded = planner();
  ASSERT_FALSE(guarded->check_terminal_recovery(initial, short_sighted.rollout,
      short_sighted.command.t_way).verified);
  const auto selected = guarded->select_motion_primitive(initial);
  ASSERT_TRUE(selected.recovery_verified);
  EXPECT_EQ("RECOVERY_VERIFIED", selected.status);
  EXPECT_GT(selected.cost, short_sighted.cost);
  EXPECT_GT(std::abs(selected.command.w_roll_c),
            std::abs(short_sighted.command.w_roll_c) + 1e-3);
  EXPECT_GT(selected.recovery_checked, 1);
  EXPECT_FALSE(selected.rollout.collision);
  EXPECT_FALSE(selected.backup_rollout.collision);
}

TEST_F(TerminalRecovery, InvalidStatesAndCommandsCannotBecomeEvidence) {
  auto mp = planner();
  const auto initial = state();
  const auto command = straight(*mp, initial);
  auto invalid_state = initial;
  invalid_state.V = 0.5;
  EXPECT_FALSE(mp->rollout_motion_primitive(invalid_state, command).valid);
  EXPECT_EQ("INVALID_STATE", mp->select_motion_primitive(invalid_state).status);
  invalid_state = initial;
  invalid_state.pos[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(mp->rollout_motion_primitive(invalid_state, command).valid);
  invalid_state = initial;
  invalid_state.roll = 1.0;
  EXPECT_FALSE(mp->rollout_motion_primitive(invalid_state, command).valid);
  auto invalid_command = command;
  invalid_command.T_c = 21.0;
  EXPECT_FALSE(mp->rollout_motion_primitive(initial, invalid_command).valid);
  invalid_command = command;
  invalid_command.w_roll_c = 1.0;
  EXPECT_FALSE(mp->rollout_motion_primitive(initial, invalid_command).valid);
  invalid_command = command;
  invalid_command.attack_c = 1.0;
  EXPECT_FALSE(mp->rollout_motion_primitive(initial, invalid_command).valid);
  invalid_command = command;
  invalid_command.t_way = -1.0;
  EXPECT_FALSE(mp->rollout_motion_primitive(initial, invalid_command).valid);
  Planner::rollout_t empty;
  EXPECT_FALSE(mp->check_terminal_recovery(initial, empty, 3.0).verified);
}

TEST_F(TerminalRecovery, RecoveryRejectsInteriorFlightPathAngleViolations) {
  nh_->setParam("motion_primitive/max_climb_angle", 20.0);
  nh_->setParam("motion_primitive/max_descent_angle", 20.0);
  auto mp = planner();
  const auto initial = state();
  const auto command = straight(*mp, initial);
  const auto primary = mp->rollout_motion_primitive(initial, command);
  ASSERT_TRUE(primary.valid);
  ASSERT_FALSE(primary.collision);
  ASSERT_GT(primary.states.size(), 2u);
  ASSERT_TRUE(mp->check_terminal_recovery(initial, primary, command.t_way).verified);
  for (float angle : {25.0f, -25.0f}) {
    auto violated = primary;
    // Both endpoints remain unchanged. A check of only the terminal state
    // would miss this interior climb/descent violation of the 20 degree limit.
    violated.states[violated.states.size() / 2].gamma = angle * Rad2Deg;
    const auto rejected = mp->check_terminal_recovery(initial, violated, command.t_way);
    EXPECT_FALSE(rejected.verified) << "Interior flight-path angle: " << angle;
    EXPECT_EQ(0, rejected.checked);
  }
}

TEST_F(TerminalRecovery, KnownSpaceRemainsCenteredAtObservationState) {
  nh_->setParam("recovery/known_space_radius", 55.0);
  auto mp = planner();
  const auto initial = state();
  const auto command = straight(*mp, initial);
  const auto primary = mp->rollout_motion_primitive(initial, command);
  ASSERT_TRUE(primary.valid);
  ASSERT_FALSE(primary.collision);
  ASSERT_FALSE(primary.states.empty());
  for (const auto& s : primary.states) {
    EXPECT_LT(std::sqrt(s.pos[0] * s.pos[0] + s.pos[1] * s.pos[1] +
                       (s.h - initial.h) * (s.h - initial.h)), 55.0);
  }
  EXPECT_FALSE(mp->check_terminal_recovery(initial, primary, command.t_way).verified)
      << "A valid backup must remain inside the original observed sphere";
  // Deliberately supplying a different observation origin demonstrates that
  // recentering at the endpoint would incorrectly certify this same maneuver.
  EXPECT_TRUE(mp->check_terminal_recovery(primary.states.back(), primary,
      command.t_way).verified);
}

TEST_F(TerminalRecovery, DynamicObstacleUsesAbsoluteContinuationStartTime) {
  nh_->setParam("dynamic_obstacles/num", 1);
  nh_->setParam("dynamic_obstacles/dyn_obs1/x", 0.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/y", 0.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/z", 20.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/r", 2.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/height", 40.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/type", "linear");
  nh_->setParam("dynamic_obstacles/dyn_obs1/velocity", 10.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/direction", 0.0);
  auto mp = planner();
  auto initial = state();
  initial.pos[0] = 100.0;
  auto command = straight(*mp, initial);
  command.t_way = 1.0;
  const auto wrong_time = mp->rollout_motion_primitive(initial, command, 0.0);
  const auto future_time = mp->rollout_motion_primitive(initial, command, 10.0);
  ASSERT_TRUE(wrong_time.valid);
  ASSERT_TRUE(future_time.valid);
  EXPECT_FALSE(wrong_time.collision);
  EXPECT_TRUE(future_time.collision)
      << "At t=10 the obstacle occupies the continuation's initial position";
}

TEST_F(TerminalRecovery, BudgetExhaustionIsDifferentFromNoFeasiblePrimary) {
  blockingCylinder();
  nh_->setParam("recovery/max_candidates", 1);
  auto limited = planner();
  const auto budget = limited->select_motion_primitive(state());
  ASSERT_TRUE(budget.primary_feasible);
  EXPECT_FALSE(budget.recovery_verified);
  EXPECT_TRUE(budget.budget_exhausted);
  EXPECT_EQ("BUDGET_EXHAUSTED", budget.status);
  EXPECT_EQ(1, budget.recovery_checked);
  EXPECT_GT(budget.primary_feasible_count, budget.recovery_checked);

  // Place the initial state inside a tall obstacle: this is a distinct failure
  // before recovery testing, even with the same tiny recovery budget.
  blockingCylinder(0.0, 2.0);
  auto blocked = planner();
  const auto impossible = blocked->select_motion_primitive(state());
  EXPECT_FALSE(impossible.primary_feasible);
  EXPECT_FALSE(impossible.recovery_verified);
  EXPECT_FALSE(impossible.budget_exhausted);
  EXPECT_EQ("NO_FEASIBLE_PRIMARY", impossible.status);
  EXPECT_EQ(0, impossible.recovery_checked);
}

TEST_F(TerminalRecovery, DisabledModeMatchesOriginalGreedySelection) {
  nh_->setParam("recovery/enabled", false);
  // These recovery-only limits must have no effect on the disabled baseline.
  nh_->setParam("recovery/known_space_radius", 1.0);
  nh_->setParam("recovery/max_candidates", 1);
  auto mp = planner();
  const auto initial = state();
  const auto library = mp->generate_library(initial);
  float best_cost = std::numeric_limits<float>::infinity();
  Planner::pr_t best;
  for (const auto& command : library) {
    const auto predicted = mp->rollout_motion_primitive(initial, command);
    const float cost = mp->evaluate_motion_primitive(initial, command, predicted);
    if (cost < best_cost) {
      best = command;
      best_cost = cost;
    }
  }
  const auto actual = mp->select_motion_primitive(initial);
  ASSERT_TRUE(actual.primary_feasible);
  EXPECT_EQ("BASELINE_FEASIBLE", actual.status);
  EXPECT_FLOAT_EQ(best_cost, actual.cost);
  EXPECT_FLOAT_EQ(best.w_roll_c, actual.command.w_roll_c);
  EXPECT_FLOAT_EQ(best.attack_c, actual.command.attack_c);
  EXPECT_FLOAT_EQ(best.T_c, actual.command.T_c);
  EXPECT_EQ(0, actual.recovery_checked);
  EXPECT_EQ(0, actual.backup_evaluated);
  EXPECT_FALSE(actual.recovery_verified);
}

TEST_F(TerminalRecovery, DiverseOrderIsDeterministicAndCoversDistinctManeuversWithinBudget) {
  nh_->setParam("recovery/candidate_order", "diverse");
  nh_->setParam("recovery/max_candidates", 5);
  auto mp = planner();
  const auto initial = state();
  std::vector<Planner::recovery_candidate_t> candidates;
  auto add = [&](float heading, float height, float cost) {
    Planner::recovery_candidate_t candidate;
    candidate.terminal = initial;
    candidate.terminal.sigma = heading;
    candidate.terminal.h = height;
    candidate.cost = cost;
    candidates.push_back(candidate);
  };
  add(0.0, 20.0, 0.0);    // Lowest-cost center must be tested first.
  add(0.01, 20.01, 1.0);  // Cheap near-duplicate must not consume coverage.
  add(-0.6, 20.0, 2.0);   // Distinct left terminal heading.
  add(0.6, 20.0, 3.0);    // Distinct right terminal heading.
  add(0.0, 3.0, 4.0);     // Distinct descending terminal state.
  add(0.0, 37.0, 5.0);    // Distinct climbing terminal state.
  add(-0.59, 20.01, 6.0); // Near-duplicate of left.
  add(0.59, 20.01, 7.0);  // Near-duplicate of right.
  const auto order = mp->recovery_candidate_order(initial, candidates);
  ASSERT_EQ(5u, order.size());
  EXPECT_EQ(0u, order.front());
  EXPECT_EQ((std::set<size_t>{0, 2, 3, 4, 5}),
            std::set<size_t>(order.begin(), order.end()));
  for (int repeat = 0; repeat < 5; ++repeat)
    EXPECT_EQ(order, mp->recovery_candidate_order(initial, candidates));

  nh_->setParam("recovery/max_candidates", 1);
  auto one = planner();
  EXPECT_EQ((std::vector<size_t>{0}), one->recovery_candidate_order(initial, candidates));
  nh_->setParam("recovery/max_candidates", 100);
  auto unlimited = planner();
  const auto all = unlimited->recovery_candidate_order(initial, candidates);
  EXPECT_EQ(candidates.size(), all.size());
  EXPECT_EQ(candidates.size(), std::set<size_t>(all.begin(), all.end()).size());
}

TEST_F(TerminalRecovery, CostOrderKeepsStableRankingAndBudget) {
  nh_->setParam("recovery/candidate_order", "cost");
  nh_->setParam("recovery/max_candidates", 3);
  auto mp = planner();
  const auto initial = state();
  std::vector<Planner::recovery_candidate_t> candidates;
  for (float cost : {3.0f, 1.0f, 1.0f, 2.0f}) {
    Planner::recovery_candidate_t candidate;
    candidate.terminal = initial;
    candidate.cost = cost;
    candidates.push_back(candidate);
  }
  EXPECT_EQ((std::vector<size_t>{1, 2, 3}),
            mp->recovery_candidate_order(initial, candidates));
  EXPECT_TRUE(mp->recovery_candidate_order(initial, {}).empty());
}

TEST_F(TerminalRecovery, DefaultOrderRetainsCostModeSelection) {
  blockingCylinder();
  nh_->setParam("recovery/max_candidates", 12);
  auto default_mode = planner();
  const auto expected = default_mode->select_motion_primitive(state());
  nh_->setParam("recovery/candidate_order", "cost");
  auto explicit_mode = planner();
  const auto actual = explicit_mode->select_motion_primitive(state());
  EXPECT_EQ(expected.status, actual.status);
  EXPECT_EQ(expected.recovery_verified, actual.recovery_verified);
  EXPECT_EQ(expected.budget_exhausted, actual.budget_exhausted);
  EXPECT_EQ(expected.recovery_checked, actual.recovery_checked);
  EXPECT_EQ(expected.backup_evaluated, actual.backup_evaluated);
  EXPECT_FLOAT_EQ(expected.cost, actual.cost);
  EXPECT_FLOAT_EQ(expected.command.w_roll_c, actual.command.w_roll_c);
  EXPECT_FLOAT_EQ(expected.command.attack_c, actual.command.attack_c);
  EXPECT_FLOAT_EQ(expected.command.T_c, actual.command.T_c);
}

TEST_F(TerminalRecovery, DiverseSelectionUsesLowestCostVerifiedScheduledCandidate) {
  blockingCylinder();
  nh_->setParam("recovery/candidate_order", "diverse");
  nh_->setParam("recovery/max_candidates", 12);
  auto mp = planner();
  const auto initial = state();
  struct Candidate {
    Planner::pr_t command;
    Planner::rollout_t rollout;
    float cost;
  };
  std::vector<Candidate> feasible;
  std::vector<Planner::recovery_candidate_t> descriptors;
  for (const auto& command : mp->generate_library(initial)) {
    const auto prediction = mp->rollout_motion_primitive(initial, command);
    const float cost = mp->evaluate_motion_primitive(initial, command, prediction);
    if (!prediction.valid || prediction.collision || !std::isfinite(cost)) continue;
    feasible.push_back({command, prediction, cost});
    Planner::recovery_candidate_t descriptor;
    descriptor.terminal = prediction.states.back();
    descriptor.cost = cost;
    descriptors.push_back(descriptor);
  }
  const auto schedule = mp->recovery_candidate_order(initial, descriptors);
  ASSERT_GT(schedule.size(), 1u);
  const auto& cheapest = feasible[schedule.front()];
  ASSERT_FALSE(mp->check_terminal_recovery(initial, cheapest.rollout,
      cheapest.command.t_way).verified)
      << "Fixture must exercise the branch after the first candidate fails";
  int verified_count = 0;
  float first_verified_cost = std::numeric_limits<float>::infinity();
  float best_verified_cost = std::numeric_limits<float>::infinity();
  for (size_t index : schedule) {
    const auto& candidate = feasible[index];
    if (!mp->check_terminal_recovery(initial, candidate.rollout,
                                    candidate.command.t_way).verified) continue;
    if (verified_count++ == 0) first_verified_cost = candidate.cost;
    best_verified_cost = std::min(best_verified_cost, candidate.cost);
  }
  ASSERT_GE(verified_count, 2);
  ASSERT_GT(first_verified_cost, best_verified_cost + 1e-4)
      << "Fixture must catch incorrectly returning the first diverse success";
  const auto actual = mp->select_motion_primitive(initial);
  // Also check that the manually replayed set matches the selector's filters;
  // otherwise this fixture cannot be used to assert the chosen minimum.
  ASSERT_EQ(static_cast<int>(feasible.size()), actual.primary_feasible_count);
  ASSERT_TRUE(actual.recovery_verified);
  EXPECT_EQ("RECOVERY_VERIFIED", actual.status);
  EXPECT_NEAR(best_verified_cost, actual.cost, 1e-5);
  EXPECT_EQ(static_cast<int>(schedule.size()), actual.scheduled_count);
  EXPECT_LE(actual.recovery_checked, 12);
  EXPECT_EQ("diverse", actual.candidate_order);
}

TEST_F(TerminalRecovery, CachedPlanConsumesElapsedTimeWithoutRenewingItsHorizon) {
  nh_->setParam("recovery/reuse_verified_plan", true);
  auto mp = planner();
  const auto initial = state();
  const auto selected = mp->select_motion_primitive(initial);
  ASSERT_TRUE(selected.recovery_verified);
  ASSERT_EQ(2u, selected.execution_plan.size());
  const float full_duration = selected.execution_plan[0].t_way + selected.execution_plan[1].t_way;
  mp->accept_selection(selected);
  EXPECT_NEAR(full_duration, mp->remaining_execution_time(), 1e-5);
  auto applied = selected.execution_plan.front();
  applied.t_way = 0.2;
  const auto advanced = mp->rollout_motion_primitive(initial, applied);
  ASSERT_TRUE(advanced.valid);
  ASSERT_FALSE(advanced.collision);
  mp->advance_execution(applied.t_way);
  ASSERT_EQ(2u, mp->remaining_execution_plan().size());
  EXPECT_NEAR(selected.command.t_way - applied.t_way,
              mp->remaining_execution_plan().front().t_way, 1e-5);
  const auto reused = mp->revalidate_cached_plan(advanced.states.back());
  ASSERT_TRUE(reused.primary_feasible);
  EXPECT_TRUE(reused.reused_verified_plan);
  EXPECT_FALSE(reused.recovery_verified)
      << "Replaying old evidence must not count as newly verified continuation";
  EXPECT_EQ("REVALIDATED_PLAN", reused.status);
  EXPECT_EQ(2, reused.cached_segments_checked);
  EXPECT_GT(reused.cached_integration_steps, 0);
  EXPECT_NEAR(full_duration - applied.t_way, reused.plan_remaining_time, 1e-5);
  mp->accept_selection(reused);
  EXPECT_NEAR(full_duration - applied.t_way, mp->remaining_execution_time(), 1e-5);
  // Crossing the segment boundary consumes exactly the old first segment,
  // leaving the old backup controls with their remaining duration.
  mp->advance_execution(selected.command.t_way - applied.t_way + 0.2f);
  ASSERT_EQ(1u, mp->remaining_execution_plan().size());
  const auto remaining = mp->remaining_execution_plan().front();
  EXPECT_FLOAT_EQ(selected.backup_command.w_roll_c, remaining.w_roll_c);
  EXPECT_FLOAT_EQ(selected.backup_command.attack_c, remaining.attack_c);
  EXPECT_FLOAT_EQ(selected.backup_command.T_c, remaining.T_c);
  EXPECT_NEAR(selected.backup_command.t_way - 0.2f, remaining.t_way, 1e-5);
  mp->advance_execution(remaining.t_way);
  EXPECT_TRUE(mp->remaining_execution_plan().empty());
  EXPECT_NEAR(0.0, mp->remaining_execution_time(), 1e-6);
  EXPECT_EQ("NO_CACHED_PLAN", mp->revalidate_cached_plan(initial).status);
}

TEST_F(TerminalRecovery, CachedPlanRejectsUnusableTailAndInvalidElapsedTime) {
  nh_->setParam("recovery/reuse_verified_plan", true);
  nh_->setParam("recovery/min_execution_time", 0.1);
  auto mp = planner();
  const auto initial = state();
  const auto selected = mp->select_motion_primitive(initial);
  ASSERT_TRUE(selected.recovery_verified);
  mp->accept_selection(selected);
  auto applied = selected.command;
  applied.t_way -= 0.05f;
  const auto advanced = mp->rollout_motion_primitive(initial, applied);
  ASSERT_TRUE(advanced.valid);
  mp->advance_execution(applied.t_way);
  const auto rejected = mp->revalidate_cached_plan(advanced.states.back());
  EXPECT_EQ("CACHED_PLAN_REJECTED", rejected.status);
  EXPECT_FALSE(rejected.primary_feasible);
  EXPECT_FALSE(rejected.reused_verified_plan);
  EXPECT_TRUE(mp->remaining_execution_plan().empty());
  for (float invalid_dt : {-0.1f, std::numeric_limits<float>::quiet_NaN(),
                           std::numeric_limits<float>::infinity()}) {
    mp->accept_selection(selected);
    ASSERT_GT(mp->remaining_execution_time(), 0.0);
    mp->advance_execution(invalid_dt);
    EXPECT_TRUE(mp->remaining_execution_plan().empty());
  }
}

TEST_F(TerminalRecovery, PlanReuseIsOptInAndExternalControlClearsCachedEvidence) {
  auto disabled = planner();
  nh_->setParam("recovery/reuse_verified_plan", true);
  auto enabled = planner();
  const auto selected = enabled->select_motion_primitive(state());
  ASSERT_TRUE(selected.recovery_verified);
  disabled->accept_selection(selected);
  EXPECT_TRUE(disabled->remaining_execution_plan().empty());
  enabled->accept_selection(selected);
  ASSERT_GT(enabled->remaining_execution_time(), 0.0);
  enabled->remember_command(selected.command);
  EXPECT_TRUE(enabled->remaining_execution_plan().empty());
  EXPECT_EQ("NO_CACHED_PLAN", enabled->revalidate_cached_plan(state()).status);
}

TEST_F(TerminalRecovery, CachedPlanIsRecheckedFromActualStateAgainstCurrentObstacles) {
  nh_->setParam("recovery/reuse_verified_plan", true);
  auto open = planner();
  const auto initial = state();
  const auto selected = open->select_motion_primitive(initial);
  ASSERT_TRUE(selected.recovery_verified);
  // A new obstacle in the second segment invalidates an otherwise legitimate
  // stored plan. Loading a previously accepted selection does not certify it.
  blockingCylinder(70.0, 2.0);
  auto changed_world = planner();
  changed_world->accept_selection(selected);
  ASSERT_FALSE(changed_world->rollout_motion_primitive(initial, selected.command).collision);
  const auto obstacle_rejected = changed_world->revalidate_cached_plan(initial);
  EXPECT_EQ("CACHED_PLAN_REJECTED", obstacle_rejected.status);
  EXPECT_FALSE(obstacle_rejected.primary_feasible);
  EXPECT_FALSE(obstacle_rejected.recovery_verified);
  EXPECT_TRUE(changed_world->remaining_execution_plan().empty());

  // Here the original state remains clear but a physically finite lateral
  // disturbance places the aircraft inside an obstacle beside its old path.
  blockingCylinder(0.0, 2.0);
  nh_->setParam("static_obstacles/obs1/y", 10.0);
  auto disturbed = planner();
  disturbed->accept_selection(selected);
  ASSERT_EQ("REVALIDATED_PLAN", disturbed->revalidate_cached_plan(initial).status);
  auto actual = initial;
  actual.pos[1] = 10.0;
  const auto state_rejected = disturbed->revalidate_cached_plan(actual);
  EXPECT_EQ("CACHED_PLAN_REJECTED", state_rejected.status);
  EXPECT_FALSE(state_rejected.reused_verified_plan);
  EXPECT_TRUE(disturbed->remaining_execution_plan().empty());
}

TEST_F(TerminalRecovery, CachedPlanMustFitTheCurrentKnownRegion) {
  nh_->setParam("recovery/reuse_verified_plan", true);
  auto unlimited = planner();
  const auto initial = state();
  const auto selected = unlimited->select_motion_primitive(initial);
  ASSERT_TRUE(selected.recovery_verified);
  nh_->setParam("recovery/known_space_radius", 55.0);
  auto limited = planner();
  limited->accept_selection(selected);
  const auto rejected = limited->revalidate_cached_plan(initial);
  EXPECT_EQ("CACHED_PLAN_REJECTED", rejected.status);
  EXPECT_FALSE(rejected.primary_feasible);
  EXPECT_FALSE(rejected.reused_verified_plan);
  EXPECT_TRUE(limited->remaining_execution_plan().empty());
}

TEST_F(TerminalRecovery, SelectorRevalidatesRemainingPlanWhenNewHorizonDoesNotFit) {
  nh_->setParam("recovery/reuse_verified_plan", true);
  nh_->setParam("motion_primitive/num", 1);
  nh_->setParam("motion_primitive/min_altitude", 16.0);
  nh_->setParam("motion_primitive/max_altitude", 24.0);
  auto open = planner();
  const auto initial = state();
  const auto selected = open->select_motion_primitive(initial);
  ASSERT_TRUE(selected.recovery_verified);
  auto applied = selected.command;
  applied.t_way = 1.0;
  const auto advanced = open->rollout_motion_primitive(initial, applied);
  ASSERT_TRUE(advanced.valid);
  ASSERT_FALSE(advanced.collision);
  const auto actual = advanced.states.back();
  // A fresh 3+3 second continuation reaches beyond this observation. The
  // already accepted plan has only 5 seconds left and still fits inside it.
  nh_->setParam("recovery/known_space_radius", 80.0);
  auto no_cache = planner();
  const auto without = no_cache->select_motion_primitive(actual);
  ASSERT_TRUE(without.primary_feasible);
  ASSERT_FALSE(without.recovery_verified)
      << "Fixture requires all new six-second plans to exceed known space";
  ASSERT_EQ("DEGRADED_NO_CONTINUATION", without.status);
  auto with_cache = planner();
  with_cache->accept_selection(selected);
  with_cache->advance_execution(applied.t_way);
  const auto reused = with_cache->select_motion_primitive(actual);
  EXPECT_EQ("REVALIDATED_PLAN", reused.status);
  EXPECT_TRUE(reused.primary_feasible);
  EXPECT_TRUE(reused.reused_verified_plan);
  EXPECT_FALSE(reused.recovery_verified);
  EXPECT_NEAR(5.0, reused.plan_remaining_time, 1e-4);
  EXPECT_NEAR(2.0, reused.command.t_way, 1e-4);
}

TEST_F(TerminalRecovery, CachedSegmentsAccumulateDynamicObstaclePredictionTime) {
  nh_->setParam("recovery/reuse_verified_plan", true);
  auto open = planner();
  const auto initial = state();
  const auto selected = open->select_motion_primitive(initial);
  ASSERT_TRUE(selected.recovery_verified);
  ASSERT_GT(selected.backup_rollout.states.size(), 100u);
  const float crossing_time_in_backup = 2.0f;
  // With integration_dt=0.02, state 100 is two seconds into the backup.
  const auto crossing = selected.backup_rollout.states[100];
  const float obstacle_velocity = 10.0f;
  nh_->setParam("dynamic_obstacles/num", 1);
  nh_->setParam("dynamic_obstacles/dyn_obs1/x", crossing.pos[0]);
  nh_->setParam("dynamic_obstacles/dyn_obs1/y", crossing.pos[1] - obstacle_velocity *
      (selected.command.t_way + crossing_time_in_backup));
  nh_->setParam("dynamic_obstacles/dyn_obs1/z", 20.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/r", 2.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/height", 40.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/type", "linear");
  nh_->setParam("dynamic_obstacles/dyn_obs1/velocity", obstacle_velocity);
  nh_->setParam("dynamic_obstacles/dyn_obs1/direction", 90.0);
  auto moving_world = planner();
  const auto first = moving_world->rollout_motion_primitive(initial, selected.command);
  ASSERT_TRUE(first.valid);
  ASSERT_FALSE(first.collision);
  const auto wrong_time = moving_world->rollout_motion_primitive(first.states.back(),
      selected.backup_command, 0.0);
  const auto correct_time = moving_world->rollout_motion_primitive(first.states.back(),
      selected.backup_command, selected.command.t_way);
  ASSERT_FALSE(wrong_time.collision);
  ASSERT_TRUE(correct_time.collision)
      << "Fixture must distinguish cumulative prediction time from resetting to zero";
  moving_world->accept_selection(selected);
  const auto rejected = moving_world->revalidate_cached_plan(initial);
  EXPECT_EQ("CACHED_PLAN_REJECTED", rejected.status);
  EXPECT_FALSE(rejected.primary_feasible);
  EXPECT_FALSE(rejected.reused_verified_plan);
  EXPECT_EQ(2, rejected.cached_segments_checked);
}

TEST_F(TerminalRecovery, PreviousShortControlCannotMasqueradeAsFreshPrimary) {
  nh_->setParam("motion_primitive/num", 1);
  nh_->setParam("motion_primitive/vertical_num", 1);
  nh_->setParam("motion_primitive/horizon_time", 3.0);
  nh_->setParam("recovery/reuse_verified_plan", false);
  blockingCylinder(25.0, 2.0);
  auto mp = planner();
  const auto initial = state();
  auto full_command = straight(*mp, initial);
  ASSERT_FLOAT_EQ(3.0f, full_command.t_way);
  const auto full_rollout = mp->rollout_motion_primitive(initial, full_command);
  ASSERT_TRUE(full_rollout.valid);
  ASSERT_TRUE(full_rollout.collision)
      << "The entire configured primary horizon must be blocked";
  auto short_command = full_command;
  short_command.t_way = 0.2f;
  const auto short_rollout = mp->rollout_motion_primitive(initial, short_command);
  ASSERT_TRUE(short_rollout.valid);
  ASSERT_FALSE(short_rollout.collision)
      << "Only the previously executed short control fragment should be clear";
  mp->remember_command(short_command);
  ASSERT_TRUE(mp->remaining_execution_plan().empty());
  const auto selected = mp->select_motion_primitive(initial);
  EXPECT_FALSE(selected.primary_feasible);
  EXPECT_FALSE(selected.recovery_verified);
  EXPECT_FALSE(selected.reused_verified_plan);
  EXPECT_EQ("NO_FEASIBLE_PRIMARY", selected.status)
      << "Previous controls must be rechecked for the full primary horizon, "
         "not reclassified using their short executed duration";
}

TEST_F(TerminalRecovery, PrimaryOnlyAblationUsesTheSameHardFiltersWithoutBackupWork) {
  blockingCylinder();
  nh_->setParam("recovery/require_terminal", false);
  nh_->setParam("recovery/max_backup_integration_steps", 1);
  nh_->setParam("recovery/reuse_verified_plan", true);
  auto control = planner();
  const auto initial = state();
  const auto selected = control->select_motion_primitive(initial);
  ASSERT_TRUE(selected.primary_feasible);
  EXPECT_EQ("PRIMARY_ONLY_FEASIBLE", selected.status);
  EXPECT_FALSE(selected.recovery_verified);
  EXPECT_FALSE(selected.reused_verified_plan);
  EXPECT_TRUE(selected.execution_plan.empty());
  EXPECT_EQ(0, selected.backup_evaluated);
  EXPECT_EQ(0, selected.backup_integration_steps);
  EXPECT_EQ(0, selected.recovery_checked);
  int feasible_count = 0;
  float best_cost = std::numeric_limits<float>::infinity();
  for (const auto& command : control->generate_library(initial)) {
    const auto rollout = control->rollout_motion_primitive(initial, command);
    if (!control->check_primary_feasibility(initial, rollout)) continue;
    ++feasible_count;
    best_cost = std::min(best_cost, control->evaluate_motion_primitive(initial, command, rollout));
  }
  EXPECT_EQ(feasible_count, selected.primary_feasible_count);
  EXPECT_FLOAT_EQ(best_cost, selected.cost);
  control->accept_selection(selected);
  EXPECT_TRUE(control->remaining_execution_plan().empty());
  nh_->setParam("recovery/known_space_radius", 1.0);
  auto unknown = planner();
  EXPECT_EQ("NO_FEASIBLE_PRIMARY", unknown->select_motion_primitive(initial).status)
      << "Primary-only must retain the same known-space hard gate";
  auto over_limit = initial;
  over_limit.gamma = 25.0f * Rad2Deg;
  EXPECT_FALSE(control->state_within_recovery_envelope(over_limit));
}

TEST_F(TerminalRecovery, IntegrationWorkCapNeverTurnsAPrefixIntoACompleteTrajectory) {
  auto mp = planner();
  const auto initial = state();
  const auto command = straight(*mp, initial);
  const auto zero = mp->rollout_motion_primitive(initial, command, 0.0f, 0);
  EXPECT_FALSE(zero.valid);
  EXPECT_TRUE(zero.work_exhausted);
  EXPECT_EQ(1u, zero.states.size());
  const auto partial = mp->rollout_motion_primitive(initial, command, 0.0f, 7);
  EXPECT_FALSE(partial.valid);
  EXPECT_TRUE(partial.work_exhausted);
  EXPECT_EQ(8u, partial.states.size());
  EXPECT_FALSE(mp->check_primary_feasibility(initial, partial));
  const auto full = mp->rollout_motion_primitive(initial, command);
  ASSERT_TRUE(full.valid);
  const auto exact = mp->rollout_motion_primitive(initial, command, 0.0f,
      static_cast<int>(full.states.size()) - 1);
  EXPECT_TRUE(exact.valid);
  EXPECT_FALSE(exact.work_exhausted);
  EXPECT_EQ(full.states.size(), exact.states.size());
}

TEST_F(TerminalRecovery, TerminalBudgetDistinguishesZeroExactAndUnlimitedWork) {
  auto mp = planner();
  const auto initial = state();
  const auto command = straight(*mp, initial);
  const auto primary = mp->rollout_motion_primitive(initial, command);
  const auto zero = mp->check_terminal_recovery(initial, primary, command.t_way, 0);
  EXPECT_FALSE(zero.verified);
  EXPECT_TRUE(zero.work_budget_exhausted);
  EXPECT_EQ(0, zero.integration_steps);
  EXPECT_EQ(0, zero.checked);
  const auto limited = mp->check_terminal_recovery(initial, primary, command.t_way, 7);
  EXPECT_FALSE(limited.verified);
  EXPECT_TRUE(limited.work_budget_exhausted);
  EXPECT_EQ(7, limited.integration_steps);
  EXPECT_EQ(1, limited.truncated);
  EXPECT_EQ(0, limited.rejected_invalid) << "Budget truncation is not a numerical failure";
  const auto unlimited = mp->check_terminal_recovery(initial, primary, command.t_way);
  ASSERT_TRUE(unlimited.verified);
  ASSERT_GT(unlimited.integration_steps, 7);
  const auto exact = mp->check_terminal_recovery(initial, primary, command.t_way,
      unlimited.integration_steps);
  EXPECT_TRUE(exact.verified);
  EXPECT_FALSE(exact.work_budget_exhausted);
  EXPECT_EQ(unlimited.integration_steps, exact.integration_steps);
}

TEST_F(TerminalRecovery, SelectSharesOneExactWorkBudgetAcrossDifferentPrimaries) {
  blockingCylinder();
  nh_->setParam("recovery/require_terminal", false);
  auto reference = planner();
  const auto initial = state();
  const auto cheapest = reference->select_motion_primitive(initial);
  ASSERT_TRUE(cheapest.primary_feasible);
  const auto exhaustive = reference->check_terminal_recovery(initial, cheapest.rollout,
      cheapest.command.t_way);
  ASSERT_FALSE(exhaustive.verified);
  ASSERT_FALSE(exhaustive.work_budget_exhausted);
  ASSERT_GT(exhaustive.integration_steps, 0);
  const int budget = exhaustive.integration_steps + 7;
  nh_->setParam("recovery/require_terminal", true);
  nh_->setParam("recovery/max_backup_integration_steps", budget);
  nh_->setParam("recovery/max_candidates", 29);
  auto limited = planner();
  const auto selected = limited->select_motion_primitive(initial);
  EXPECT_TRUE(selected.primary_feasible);
  EXPECT_FALSE(selected.recovery_verified);
  EXPECT_TRUE(selected.backup_work_budget_exhausted);
  EXPECT_TRUE(selected.budget_exhausted);
  EXPECT_EQ("BACKUP_WORK_BUDGET_EXHAUSTED", selected.status);
  EXPECT_EQ(budget, selected.backup_integration_steps);
  EXPECT_EQ(2, selected.recovery_checked);
  EXPECT_EQ(1, selected.backup_truncated);
  nh_->setParam("recovery/max_backup_integration_steps", 0); // Config 0 is unlimited.
  nh_->setParam("recovery/max_candidates", 1);
  auto count_limited = planner();
  const auto different_limit = count_limited->select_motion_primitive(initial);
  EXPECT_EQ("BUDGET_EXHAUSTED", different_limit.status);
  EXPECT_FALSE(different_limit.backup_work_budget_exhausted);
  EXPECT_EQ(exhaustive.integration_steps, different_limit.backup_integration_steps);
}

TEST_F(TerminalRecovery, BackupRejectionCountersSeparateCollisionFromUnknownSpace) {
  blockingCylinder();
  auto blocked = planner();
  const auto initial = state();
  const auto command = straight(*blocked, initial);
  const auto primary = blocked->rollout_motion_primitive(initial, command);
  const auto collision = blocked->check_terminal_recovery(initial, primary, command.t_way);
  EXPECT_FALSE(collision.verified);
  EXPECT_GT(collision.rejected_collision, 0);
  EXPECT_EQ(collision.checked, collision.rejected_collision + collision.rejected_envelope +
      collision.rejected_known_space + collision.rejected_invalid);
  EXPECT_EQ(0, collision.truncated);
  nh_->setParam("static_obstacles/num", 0);
  nh_->setParam("recovery/known_space_radius", 55.0);
  auto limited = planner();
  const auto clear_primary = limited->rollout_motion_primitive(initial, command);
  const auto unknown = limited->check_terminal_recovery(initial, clear_primary, command.t_way);
  EXPECT_FALSE(unknown.verified);
  EXPECT_EQ(0, unknown.rejected_collision);
  EXPECT_GT(unknown.rejected_known_space, 0);
  EXPECT_EQ(unknown.checked, unknown.rejected_collision + unknown.rejected_envelope +
      unknown.rejected_known_space + unknown.rejected_invalid);
}

TEST_F(TerminalRecovery, BackupCountersDistinguishModelInvalidityAndRecoveryEnvelope) {
  auto mp = planner();
  auto near_angle_limit = state();
  near_angle_limit.gamma = 20.0f * Rad2Deg;
  Planner::rollout_t primary;
  primary.states = {near_angle_limit, near_angle_limit};
  const auto angle = mp->check_terminal_recovery(near_angle_limit, primary, 3.0f);
  EXPECT_FALSE(angle.verified);
  EXPECT_GT(angle.rejected_envelope, 0);
  auto accelerating = state();
  accelerating.V = 44.9f;
  accelerating.T = 20.0f;
  accelerating.attack = 0.3f;
  ASSERT_TRUE(mp->state_within_recovery_envelope(accelerating));
  primary.states = {accelerating, accelerating};
  const auto invalid = mp->check_terminal_recovery(accelerating, primary, 3.0f);
  EXPECT_FALSE(invalid.verified);
  EXPECT_EQ(invalid.checked, invalid.rejected_invalid);
  EXPECT_GT(invalid.rejected_invalid, 0);
  EXPECT_EQ(0, invalid.truncated);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "terminal_recovery_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
