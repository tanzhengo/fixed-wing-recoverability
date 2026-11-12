import math
import unittest
from dataclasses import replace
from unittest.mock import patch

from fwcontinuation.types import State, Control, Config
from fwcontinuation.dynamics import (derivative, step, trim_state,
                                     generate_primary, generate_successors, rollout)


class RecordingMap:
    def __init__(self, limit=float('inf')):
        self.calls = []
        self.limit = limit

    def segment_free(self, a, b, margin):
        self.calls.append((a, b, margin))
        return margin <= self.limit


class DynamicsTests(unittest.TestCase):
    def setUp(self):
        self.state = trim_state()
        self.control = Control(self.state.phi, self.state.alpha, self.state.T)
        self.config = Config()

    def test_trim_is_ode_equilibrium(self):
        d = derivative(self.state, self.control)
        self.assertAlmostEqual(d.x, 15.5, places=12)
        for name in ('y', 'h', 'V', 'gamma', 'psi', 'phi', 'alpha', 'T'):
            self.assertAlmostEqual(getattr(d, name), 0.0, places=11)
        r = rollout(self.state, self.control, self.config)
        self.assertTrue(r.valid)
        self.assertEqual(r.steps, 150)
        self.assertAlmostEqual(r.states[-1].x, 46.5, places=10)
        self.assertAlmostEqual(r.times[-1], 3.0)

    def test_flight_axis_and_thrust_signs(self):
        climb = replace(self.state, gamma=0.1)
        turn = replace(self.state, phi=0.1, psi=math.pi / 2)
        self.assertGreater(derivative(climb, self.control).h, 0)
        self.assertLess(derivative(climb, self.control).V, 0)
        self.assertGreater(derivative(turn, self.control).psi, 0)
        self.assertAlmostEqual(derivative(turn, self.control).x, 0, places=10)
        self.assertGreater(derivative(turn, self.control).y, 0)
        self.assertGreater(derivative(replace(self.state, T=self.state.T + 1), self.control).V, 0)

    def test_actuator_states_continue_across_command_switch(self):
        command = Control(0.2, self.state.alpha + 0.1, self.state.T + 1)
        endpoint = step(self.state, command, 0.02)
        self.assertAlmostEqual(endpoint.phi, 0.008)
        self.assertAlmostEqual(endpoint.alpha, self.state.alpha + 0.002)
        self.assertAlmostEqual(endpoint.T, self.state.T + 0.2)
        switched = step(endpoint, self.control, 0.02)
        self.assertAlmostEqual(switched.phi, endpoint.phi * 0.96)
        self.assertNotEqual(switched.phi, self.control.phi)
        self.assertAlmostEqual(endpoint.yaw_rate, derivative(self.state, command).psi)

    def test_rk4_matches_analytic_actuator_response(self):
        command = Control(0.2, self.state.alpha + 0.1, self.state.T + 1, 0.2)
        r = rollout(self.state, command, self.config, method='rk4')
        self.assertTrue(r.valid)
        self.assertEqual(r.steps, 40)
        self.assertAlmostEqual(r.states[-1].phi, 0.2 * (1 - math.exp(-0.4)), places=9)
        self.assertAlmostEqual(r.states[-1].alpha, self.state.alpha + 0.1 * (1 - math.exp(-0.2)), places=10)
        self.assertAlmostEqual(r.states[-1].T, self.state.T + 1 - math.exp(-2), places=7)

    def test_primary_library_target_order_and_deduplication(self):
        captures = []
        def record(state, distance, heading, height, duration, config):
            captures.append((heading, height, duration))
            return Control(0, state.alpha, state.T, duration)
        with patch('fwcontinuation.dynamics._target_command', side_effect=record):
            bank = generate_primary(self.state, (250, 0, 23.4), self.config)
        self.assertEqual(len(bank), 406)
        heights = [captures[i][1] for i in range(0, len(captures), 29)]
        self.assertEqual(len(heights), 14)
        self.assertEqual(heights[:7], sorted(heights[:7]))
        self.assertAlmostEqual(heights[3], 23.4)
        self.assertAlmostEqual(heights[7], self.state.h)
        self.assertGreater(heights[8], self.state.h)
        self.assertLess(heights[9], self.state.h)
        self.assertAlmostEqual(captures[0][0], -self.config.heading_span)
        self.assertAlmostEqual(captures[28][0], self.config.heading_span)
        self.assertEqual(generate_primary(self.state, (250, 0, 20), self.config),
                         generate_primary(self.state, (250, 0, 20), self.config))

    def test_successor_bank_order_and_command_bounds(self):
        config = replace(self.config, compensate_tau=0)
        bank = generate_successors(self.state, config)
        self.assertEqual(len(bank), 15)
        self.assertEqual([math.copysign(1, c.phi) if abs(c.phi)>1e-10 else 0 for c in bank[:5]],
                         [0, -1, 1, -1, 1])
        self.assertGreater(bank[5].alpha, bank[0].alpha)
        self.assertLess(bank[10].alpha, bank[0].alpha)
        for c in bank + generate_primary(self.state, (250, 0, 24), self.config):
            self.assertLessEqual(abs(c.phi), config.max_roll)
            self.assertTrue(config.min_alpha <= c.alpha <= config.max_alpha)
            self.assertTrue(0 <= c.T <= config.max_thrust)
        self.assertEqual(len(generate_successors(replace(self.state,h=config.max_altitude),config)),15)

    def test_every_nonfinite_state_field_is_rejected_before_work(self):
        for name in State.__dataclass_fields__:
            with self.subTest(name=name):
                r = rollout(replace(self.state, **{name:float('nan')}), self.control, self.config)
                self.assertFalse(r.valid)
                self.assertEqual(r.reason, 'invalid_dynamics')
                self.assertEqual(r.steps, 0)
        r = rollout(replace(self.state,phi=self.config.max_roll+0.001), self.control, self.config)
        self.assertEqual((r.valid,r.reason,r.steps),(False,'envelope',0))

    def test_successor_reserve_uses_global_prediction_time(self):
        recorded = RecordingMap()
        r = rollout(self.state, replace(self.control,duration=0.04), self.config,
                    recorded,start_time=self.config.H)
        self.assertTrue(r.valid)
        self.assertEqual(r.times,[3.0,3.02,3.04])
        self.assertTrue(all(call[2]==3.0 for call in recorded.calls))
        blocked = rollout(self.state,self.control,self.config,RecordingMap(2.5),start_time=3)
        self.assertEqual((blocked.valid,blocked.steps),(False,0))
        dispatch = rollout(self.state,self.control,self.config,RecordingMap(2.5),start_time=3,proposal=False)
        self.assertTrue(dispatch.valid)

    def test_segment_uses_larger_endpoint_reserve(self):
        recorded=RecordingMap()
        r=rollout(self.state,replace(self.control,duration=0.04),self.config,recorded,start_time=.19)
        self.assertTrue(r.valid)
        self.assertAlmostEqual(recorded.calls[0][2],2)
        self.assertAlmostEqual(recorded.calls[1][2],2+(.21-.2)/.3)
        self.assertAlmostEqual(recorded.calls[2][2],2+(.23-.2)/.3)

    def test_partial_final_step_and_initial_map_rejection(self):
        r=rollout(self.state,replace(self.control,duration=.051),self.config)
        self.assertTrue(r.valid)
        self.assertEqual(r.steps,3)
        self.assertEqual(r.times,[0,.02,.04,.051])
        self.assertAlmostEqual(r.states[-1].x,15.5*.051)
        blocked=rollout(self.state,self.control,self.config,RecordingMap(1))
        self.assertEqual((blocked.reason,blocked.steps),('observed_space',0))


if __name__=='__main__':
    unittest.main()
