#!/usr/bin/env python3
"""Pure-data checks for the ROS observer; no ROS master is contacted."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'scripts/verify_end_world.py'
spec = importlib.util.spec_from_file_location('observer', MODULE)
observer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(observer)


class ObserverTests(unittest.TestCase):
    def test_box_face_corner_and_interior(self):
        boxes = [{'x':0, 'y':0, 'z':2, 'size_x':2, 'size_y':2, 'size_z':4, 'clearance':1}]
        distances = observer.BoxDistances(boxes)
        self.assertEqual(distances.query((1,0,2)), (0,-1))
        self.assertEqual(distances.query((0,0,2)), (-1,-2))
        physical, inflated = distances.query((3,3,2))
        self.assertAlmostEqual(physical, 8**.5)
        self.assertAlmostEqual(inflated, 2**.5)
        self.assertEqual(observer.BoxDistances([]).query((0,0,0)), (None,None))

    def test_cycle_requires_full_order(self):
        goals = [(1,2,3),(4,5,6),(7,8,9)]
        transitions = [dict(x=x,y=y,z=z,t=i) for i,(x,y,z) in enumerate(goals+[goals[0]])]
        self.assertTrue(observer.complete_cycle(transitions, goals))
        self.assertFalse(observer.complete_cycle(transitions[:-1], goals))
        self.assertFalse(observer.complete_cycle([transitions[i] for i in (0,2,1,3)], goals))

    def test_gate_uses_preceding_odometry_without_relaxation(self):
        transitions = [dict(x=0,y=0,z=20,t=0),dict(x=100,y=0,z=20,t=2)]
        odometry = [dict(x=0,y=0,z=16.99,t=1.99),dict(x=0,y=0,z=17.01,t=2.01)]
        result = observer.associate_transitions(transitions,odometry,48.825,3.0)
        self.assertFalse(result[1]['previous_goal_gate_passed'])
        self.assertEqual(result[1]['aircraft']['t'],1.99)
        odometry[0]['z']=17.0
        self.assertTrue(observer.associate_transitions(transitions,odometry,48.825,3.0)[1]['previous_goal_gate_passed'])

    def test_task_load_and_invalid_dimensions(self):
        text = 'box_obstacles:\n  num: 1\n  box1: {x: 0, y: 0, z: 2, size_x: 2, size_y: 3, size_z: 4}\nwaypoints:\n  num: 1\n  wp1: {x: 10, y: 20, h: 30}\n'
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory)/'task.yaml'; file.write_text(text)
            _, boxes, goals = observer.load_task(file)
            self.assertEqual(boxes[0]['clearance'],1.0)
            self.assertEqual(goals,[(10.,20.,30.)])
            file.write_text(text.replace('size_x: 2','size_x: -2'))
            with self.assertRaises(ValueError): observer.load_task(file)
            file.write_text(text.replace('num: 1','num: true',1))
            with self.assertRaises(ValueError): observer.load_task(file)


if __name__ == '__main__':
    unittest.main()
