"""Dispatch contracts using real dynamics/maps and deterministic test clocks.

These are numerical unit/integration checks, not flight experiments or timing
benchmarks. A rejected result must contain no sequence available for publication.
"""
from dataclasses import replace
import itertools
import math
import unittest
from unittest.mock import patch

from fwcontinuation.types import Config, Control, Candidate, Witness, SearchResult
from fwcontinuation.dynamics import trim_state, rollout
from fwcontinuation.observed_map import ObservedMap
from fwcontinuation.execution import (TimedState, AppliedControl, Sequence,
    remaining_controls, align_state, validate_sequence, dispatch, ForecastEstimator)


class Clock:
    def __init__(self, *values):
        self.values = iter(values)
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return next(self.values)


class ExecutionTests(unittest.TestCase):
    def setUp(self):
        self.config = Config(H=.2, B=.2)
        self.state = trim_state(x=1, y=1, h=21)
        self.control = Control(0, self.state.alpha, self.state.T, .2)
        self.map = ObservedMap()
        # Explicit free-map fixture, not reconstructed sensor flight evidence.
        for index in itertools.product(range(-3, 15), range(-4, 5), range(6, 16)):
            self.map.set_cell(index, 'free')
        self.map.timestamp = 10.0
        primary = rollout(self.state, self.control, self.config, self.map)
        successor = rollout(primary.states[-1], self.control, self.config, self.map,
                            start_time=self.config.H)
        self.assertTrue(primary.valid and successor.valid)
        candidate = Candidate(0, self.control, primary, 1.0)
        self.proposal = SearchResult('verified', Witness(candidate, self.control, successor, 0))
        self.observation = TimedState(self.state, 10.0)

    def call_dispatch(self, *, observation=None, proposal='default', cache=None,
                      history=(), clock=None, map_provider=None):
        return dispatch(observation or self.observation,
                        self.proposal if proposal == 'default' else proposal,
                        cache, history, map_provider or (lambda: self.map), self.config,
                        clock=clock or Clock(10.0, 10.001))

    def assertRejected(self, result, reason=None):
        self.assertFalse(result.accepted)
        self.assertIsNone(result.sequence)
        self.assertEqual(result.source, '')
        if reason:
            self.assertEqual(result.reason, reason)

    def test_real_pair_is_reintegrated_before_dispatch(self):
        result = self.call_dispatch(clock=Clock(10.0, 10.005))
        self.assertTrue(result.accepted)
        self.assertEqual(result.source, 'fresh')
        self.assertEqual(result.steps, 20)
        self.assertEqual(result.attempts, 1)
        self.assertEqual(result.map_version, self.map.version)
        self.assertAlmostEqual(result.sequence.start, 10.005)
        self.assertAlmostEqual(sum(c.duration for c in result.sequence.controls), .4)
        self.assertEqual(result.validation_started, 10.0)

    def test_map_change_invalidates_previously_verified_pair(self):
        self.map.set_cell((2, 0, 10), 'occupied')
        result = self.call_dispatch()
        self.assertRejected(result, 'observed_space')

    def test_unverified_primary_is_never_a_dispatch_fallback(self):
        proposal = SearchResult('budget_exhausted', candidates=self.proposal.candidates)
        self.assertRejected(self.call_dispatch(proposal=proposal), 'no_verified_sequence')
        self.assertRejected(self.call_dispatch(proposal=None), 'no_verified_sequence')

    def test_real_failed_proposal_can_reuse_revalidated_cache(self):
        bad_command = replace(self.control, phi=2.0)
        bad_candidate = replace(self.proposal.witness.candidate, control=bad_command)
        failed = SearchResult('verified', replace(self.proposal.witness, candidate=bad_candidate))
        cache = Sequence((self.control, self.control), 9.9)
        result = self.call_dispatch(proposal=failed, cache=cache, clock=Clock(10.0, 10.005))
        self.assertTrue(result.accepted)
        self.assertEqual(result.source, 'cached')
        self.assertEqual(result.sequence.start, 10.005)
        self.assertAlmostEqual(result.sequence.controls[0].duration, .095)
        self.assertAlmostEqual(result.sequence.start + sum(c.duration for c in result.sequence.controls), 10.3)

    def test_cache_trim_and_exhaustion(self):
        cache = Sequence((self.control, self.control), 10)
        controls = remaining_controls(cache, 10.25)
        self.assertEqual(len(controls), 1)
        self.assertAlmostEqual(controls[0].duration, .15)
        self.assertEqual(remaining_controls(cache, 10.4), ())
        with self.assertRaises(ValueError):
            remaining_controls(cache, 9.9)
        with self.assertRaises(ValueError):
            remaining_controls(Sequence((replace(self.control, duration=0),), 10), 10)
        old_cache = Sequence((self.control,), 9.7)
        self.assertRejected(self.call_dispatch(proposal=None, cache=old_cache), 'empty_sequence')

    def test_cache_expiring_during_validation_is_not_extended(self):
        cache = Sequence((replace(self.control, duration=.01),), 9.995)
        result = self.call_dispatch(proposal=None, cache=cache, clock=Clock(10.0, 10.01))
        self.assertRejected(result, 'cached_sequence_expired')

    def test_future_stale_and_nonfinite_state_timestamps(self):
        for stamp, reason in ((10.1, 'invalid_timestamp'), (9.49, 'stale_state'),
                              (math.nan, 'invalid_timestamp'), (math.inf, 'invalid_timestamp')):
            with self.subTest(timestamp=stamp):
                self.assertRejected(self.call_dispatch(observation=TimedState(self.state, stamp),
                    clock=Clock(10.0)), reason)

    def test_future_stale_and_nonfinite_map_timestamps(self):
        for stamp, reason in ((10.1, 'invalid_timestamp'), (8.99, 'stale_map'),
                              (math.nan, 'invalid_timestamp'), (-math.inf, 'invalid_timestamp')):
            with self.subTest(timestamp=stamp):
                self.map.timestamp = stamp
                self.assertRejected(self.call_dispatch(clock=Clock(10.0)), reason)

    def test_history_alignment_integrates_actual_intervals(self):
        observation = TimedState(self.state, 9.9)
        history = (AppliedControl(9.9, 9.95, self.control),
                   AppliedControl(9.95, 10.1, self.control))
        aligned = align_state(observation, 10.0, history, self.config)
        self.assertAlmostEqual(aligned.x, self.state.x+self.state.V*.1, places=10)
        self.assertAlmostEqual(aligned.h, self.state.h, places=10)
        result = self.call_dispatch(observation=observation, history=history)
        self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.aligned_state.x, aligned.x)

    def test_history_gaps_overlaps_and_invalid_intervals_reject(self):
        observation = TimedState(self.state, 9.9)
        cases = [
            ((), 'Missing control history'),
            ((AppliedControl(9.91, 10.1, self.control),), 'Missing control history'),
            ((AppliedControl(9.9, 9.94, self.control), AppliedControl(9.96, 10.1, self.control)),
             'Missing control history'),
            ((AppliedControl(9.9, 9.98, self.control), AppliedControl(9.95, 10.1, self.control)),
             'Overlapping control history'),
            ((AppliedControl(10, 9, self.control),), 'Invalid control-history interval'),
        ]
        for history, reason in cases:
            with self.subTest(reason=reason, history=history):
                self.assertRejected(self.call_dispatch(observation=observation, history=history,
                    clock=Clock(10.0)), reason)

    def test_validation_overrun_retries_once_with_updated_alignment(self):
        history = (AppliedControl(10.0, 10.2, self.control),)
        clock = Clock(10.0, 10.03, 10.031, 10.04)
        result = self.call_dispatch(history=history, clock=clock)
        self.assertTrue(result.accepted)
        self.assertEqual(result.attempts, 2)
        self.assertEqual(clock.calls, 4)
        self.assertEqual(result.steps, 40)
        self.assertAlmostEqual(result.aligned_state.x, self.state.x+self.state.V*.031)
        self.assertEqual(result.sequence.start, 10.04)

    def test_second_validation_overrun_rejects_without_sequence(self):
        clock = Clock(10.0, 10.03, 10.031, 10.06)
        result = self.call_dispatch(history=(AppliedControl(10, 10.2, self.control),), clock=clock)
        self.assertRejected(result, 'validation_overrun')
        self.assertEqual(result.attempts, 2)
        self.assertEqual(clock.calls, 4)

    def test_map_mutation_retries_then_rejects_if_repeated(self):
        original = validate_sequence
        calls = 0
        def mutating_validation(*args):
            nonlocal calls
            calls += 1
            outcome = original(*args)
            self.map.set_cell((100+calls, 100, 100), 'free')
            return outcome
        with patch('fwcontinuation.execution.validate_sequence', side_effect=mutating_validation):
            result = self.call_dispatch(history=(AppliedControl(10, 10.2, self.control),),
                clock=Clock(10, 10.001, 10.002, 10.003))
        self.assertRejected(result, 'map_changed_during_validation')
        self.assertEqual(result.attempts, 2)
        self.assertEqual(calls, 2)

    def test_backward_or_nonfinite_validation_clock_rejects(self):
        for end in (9.99, math.nan):
            with self.subTest(end=end):
                self.assertRejected(self.call_dispatch(clock=Clock(10, end)), 'invalid_clock')

    def test_forecast_is_bounded_and_does_not_cross_unobserved_space(self):
        forecast = ForecastEstimator()
        self.assertEqual(forecast.estimate, .05)
        forecast.update(.5)
        self.assertEqual(forecast.estimate, .15)
        predicted = forecast.predict(self.state, self.control, self.map, self.config)
        self.assertGreater(predicted.x, self.state.x)
        self.assertEqual(forecast.predict(self.state, self.control, ObservedMap(), self.config), self.state)
        self.assertEqual(forecast.predict(self.state, None, self.map, self.config), self.state)


if __name__ == '__main__':
    unittest.main()
