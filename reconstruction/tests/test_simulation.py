"""Small numerical fixtures; not flight trials or manuscript performance data."""
from dataclasses import asdict,replace
import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np

from fwcontinuation.types import State,Control,Config
from fwcontinuation.dynamics import trim_state
from fwcontinuation.observed_map import ObservedMap
from fwcontinuation.records import parse_frame
from fwcontinuation.sensor import Sphere,Cylinder,rotation_matrix
from fwcontinuation.simulation import (SimulationConfig,plant_step,pose_from_state,
    sampled_clearance,segment_contacts_geometry,run_episode,parse_scene)


class SimulationTests(unittest.TestCase):
    def test_independent_plant_level_trim_and_ground_wind(self):
        state=trim_state();control=Control(state.phi,state.alpha,state.T)
        nominal=state;wind=state
        for _ in range(200):
            nominal=plant_step(nominal,control,.005)
            wind=plant_step(wind,control,.005,'crosswind')
        self.assertAlmostEqual(nominal.x,15.5,places=9)
        self.assertAlmostEqual(nominal.h,state.h,places=9)
        self.assertAlmostEqual(nominal.V,state.V,places=9)
        self.assertAlmostEqual(wind.y-nominal.y,1.0,places=10)
        self.assertAlmostEqual(wind.x,nominal.x,places=10)

    def test_mismatch_changes_plant_and_invalid_condition_rejected(self):
        state=replace(trim_state(),phi=.1)
        control=Control(0,state.alpha,state.T)
        a=plant_step(state,control,.1,'nominal')
        b=plant_step(state,control,.1,'model_mismatch')
        self.assertGreater(b.phi,a.phi) # increased actuator time constant
        self.assertNotEqual(a.V,b.V)
        with self.assertRaises(ValueError):plant_step(state,control,.1,'invented')
        with self.assertRaises(ValueError):SimulationConfig(condition='invented')

    def test_exact_physical_chord_contacts(self):
        sphere=Sphere((0,0,0),1)
        self.assertTrue(segment_contacts_geometry((-2,0,0),(2,0,0),[sphere]))
        self.assertFalse(segment_contacts_geometry((-2,2,0),(2,2,0),[sphere]))
        cylinder=Cylinder((0,0,0),1,2)
        self.assertTrue(segment_contacts_geometry((-2,0,0),(2,0,0),[cylinder]))
        self.assertTrue(segment_contacts_geometry((0,0,-2),(0,0,2),[cylinder]))
        self.assertFalse(segment_contacts_geometry((-2,0,2),(2,0,2),[cylinder]))
        self.assertTrue(segment_contacts_geometry((0,0,1),(0,0,-1),[],0))
        self.assertAlmostEqual(sampled_clearance((2,0,0),[cylinder]),1)
        self.assertAlmostEqual(sampled_clearance((0,0,0),[sphere]),-1)

    def test_sensor_pose_sign_convention(self):
        state=State(phi=.2,gamma=.1,alpha=.05,psi=.4)
        pose=pose_from_state(state,1.)
        forward=rotation_matrix(pose.quaternion_xyzw)@np.array([1.,0.,0.])
        np.testing.assert_allclose(forward,[math.cos(.15)*math.cos(.4),
                                           math.cos(.15)*math.sin(.4),math.sin(.15)],atol=1e-14)

    @staticmethod
    def scene():
        return {'scene_id':'unit_fixture_not_experiment','start':asdict(trim_state()),
                'goal':[250,0,20],'obstacles':[],'ground_height':0.}

    def test_one_second_recorded_smoke_keeps_failed_queries(self):
        with tempfile.TemporaryDirectory() as directory:
            config=Config(headings=1,vertical_levels=1,K=0)
            simulation=SimulationConfig(duration=1.,azimuth_count=8,elevation_count=3,max_range=8.)
            summary=run_episode(self.scene(),directory,config,simulation,mode='cost')
            self.assertEqual(summary['outcome'],'time_limit')
            self.assertEqual(summary['clock_policy'],'SIMULATION_CLOCK_ZERO_COMPUTATIONAL_LATENCY')
            self.assertFalse(summary['historical_result_reproduced'])
            self.assertEqual(summary['fresh_sequences'],0)
            self.assertEqual(summary['query_count'],5)
            self.assertGreaterEqual(summary['scan_count'],5)
            out=Path(directory)
            initial=json.loads((out/'initial_map.json').read_text())
            self.assertEqual(initial['cells'],[])
            frames=[json.loads(v) for v in (out/'frames.jsonl').read_text().splitlines()]
            queries=[json.loads(v) for v in (out/'queries.jsonl').read_text().splitlines()]
            self.assertEqual(len(queries),len(frames))
            for frame,query in zip(frames,queries):
                state,goal,observed,cfg,previous=parse_frame(frame)
                self.assertIsInstance(observed,ObservedMap)
                self.assertEqual(cfg.K,0)
                self.assertFalse(query['dispatch']['accepted'])
                self.assertEqual(query['execution_action'],'unverified_initialization_hold')
                self.assertEqual(query['query_id'],frame['query_id'])
                self.assertNotIn('obstacles',frame) # truth is isolated from planner frames
            with self.assertRaises(FileExistsError):run_episode(self.scene(),directory,config,simulation)

    def test_failure_after_declared_initialization_is_recorded(self):
        with tempfile.TemporaryDirectory() as directory:
            simulation=SimulationConfig(duration=.5,initialization_seconds=.2,
                azimuth_count=4,elevation_count=2,max_range=4.)
            summary=run_episode(self.scene(),directory,Config(headings=1,vertical_levels=1,K=0),simulation)
            self.assertEqual(summary['outcome'],'initialization_failure')
            self.assertAlmostEqual(summary['elapsed_simulation_seconds'],.2)
            queries=[json.loads(v) for v in (Path(directory)/'queries.jsonl').read_text().splitlines()]
            self.assertEqual(queries[-1]['execution_action'],'terminate_recording_no_stop_maneuver')

    def test_scene_and_model_requirements(self):
        scene=self.scene();scene['start']={'x':0,'y':0,'h':20}
        with self.assertRaises(ValueError):parse_scene(scene)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):run_episode(self.scene(),directory,mode='learned')


if __name__=='__main__':unittest.main()
