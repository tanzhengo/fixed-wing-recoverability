#include <gtest/gtest.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

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
  const auto result = mp->select_motion_primitive(initial);
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

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "terminal_recovery_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
