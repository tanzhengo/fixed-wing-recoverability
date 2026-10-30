#!/usr/bin/env python3
"""Budget and ordering regressions for validation-only schedule selection."""
import unittest
import numpy as np
from select_recovery_schedule import candidate_schedule, replay, utility


def features(angles):
    x = np.zeros((len(angles), 103), dtype=np.float32)
    x[:, 6] = 15.5 / 20
    x[:, 22] = .3
    x[:, 15] = np.sin(angles); x[:, 16] = np.cos(angles)
    return x


class ScheduleReplayTests(unittest.TestCase):
    def test_cost_seed_and_neural_order(self):
        x = features([0, .1, .2, .3])
        order = list(candidate_schedule(x, [0, 1, 2, 3], [.1, .4, .8, .9], 'learned', 3))
        self.assertEqual(order, [0, 3, 2])

    def test_hybrid_updates_coverage_with_neural_selection(self):
        x = features([0, .1, -.5, .8])
        order = list(candidate_schedule(x, [0, 1, 2, 3], [0, .2, .1, .9], 'learned_diverse', 4))
        self.assertEqual(order, [0, 3, 2, 1])

    def test_cost_ties_preserve_input_order(self):
        x = features([0, 0, 0])
        self.assertEqual(list(candidate_schedule(x, [1, 1, 1], [1, 1, 1], 'learned', 3)), [0, 1, 2])
        self.assertEqual(list(candidate_schedule(x, [1, 1, 1], [1, 1, 1], 'diverse', 3)), [0, 1, 2])

    def test_circular_heading_distance(self):
        x = features([np.pi-.01, -np.pi+.01, 0])
        self.assertEqual(list(candidate_schedule(x, [0, 1, 2], [0, 0, 0], 'diverse', 2)), [0, 2])

    def test_partial_positive_query_is_not_verified(self):
        result = replay(features([0, .2]), np.array([0, 1]), [0, 1], [100, 400], [0, 1], 'learned', 2, 300)
        self.assertFalse(result['verified']); self.assertTrue(result['truncated'])
        self.assertEqual(result['consumed_steps'], 300)

    def test_exact_budget_completion_can_verify(self):
        result = replay(features([0, .2]), np.array([0, 1]), [0, 1], [100, 400], [0, 1], 'learned', 2, 500)
        self.assertTrue(result['verified']); self.assertFalse(result['truncated'])
        self.assertEqual(result['consumed_steps'], 500)

    def test_success_before_truncation_is_retained(self):
        result = replay(features([0, .2, .3]), np.array([0, 1, 2]), [0, 1, 0], [100, 200, 300], [0, 1, .5], 'learned', 3, 400)
        self.assertTrue(result['verified']); self.assertTrue(result['truncated'])
        self.assertEqual(result['selected_index'], 1)
        self.assertEqual(result['consumed_steps'], 400)

    def test_global_cheapest_success_stops(self):
        result = replay(features([0, .2]), np.array([0, 1]), [1, 1], [100, 300], [0, 1], 'learned', 2, 0)
        self.assertEqual(result['checked'], 1)
        self.assertEqual(result['consumed_steps'], 100)
        self.assertEqual(result['regret'], 0)

    def test_work_head_changes_priority_when_enabled(self):
        predictions = np.array([[2, np.log1p(3000)], [1.5, np.log1p(300)]], dtype=np.float32)
        self.assertGreater(utility(predictions, 0)[0], utility(predictions, 0)[1])
        self.assertLess(utility(predictions, 1)[0], utility(predictions, 1)[1])

    def test_empty_state(self):
        result = replay(features([]), [], [], [], [], 'learned', 12, 3600)
        self.assertFalse(result['verified']); self.assertEqual(result['checked'], 0)


if __name__ == '__main__':
    unittest.main()
