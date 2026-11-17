#!/usr/bin/env python3
"""Protocol regressions that prevent truth geometry leaking into unknown trials."""
import copy
import importlib.util
import gzip
import json
import struct
import sys
import types
from pathlib import Path
import tempfile
import unittest
import yaml

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/run_unknown_city.py'
SPEC = importlib.util.spec_from_file_location('unknown_city', SCRIPT)
u = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(u)


class UnknownProtocol(unittest.TestCase):
    def setUp(self):
        self.source = {'motion_primitive': {'min_altitude': 5., 'max_altitude': 160.},
                       'waypoints': {'num': 1, 'wp1': {'x': -30., 'y': 80., 'h': 50.}},
                       'box_obstacles': {'num': 1, 'box1': {'x': 99., 'source': 'secret_geometry'}},
                       'model': {'map_file': 'full_truth.yaml'}}
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name)/'task.yaml'
        self.path.write_text(yaml.safe_dump(self.source))
    def tearDown(self):
        self.directory.cleanup()
    def parameters(self):
        return dict(u.sanitized_task(self.path), sensor_map={'enabled': True})
    def test_snapshot_keeps_task_values_and_discards_truth(self):
        before = self.path.read_bytes()
        task = u.sanitized_task(self.path)
        self.assertEqual(task['waypoints'], self.source['waypoints'])
        self.assertEqual(task['motion_primitive'], self.source['motion_primitive'])
        self.assertNotIn('model', task)
        self.assertNotIn('secret_geometry', yaml.safe_dump(task))
        self.assertEqual(before, self.path.read_bytes())
        u.assert_geometry_free(self.parameters())
    def test_each_truth_obstacle_family_is_rejected(self):
        for name in u.OBSTACLES:
            for value in [{'num': 1}, {'num': 0, 'old_obstacle': {}}, {'num': False}]:
                parameters = self.parameters(); parameters[name] = value
                with self.assertRaises(ValueError):
                    u.assert_geometry_free(parameters)
    def test_sensor_map_must_be_explicitly_enabled(self):
        for value in [False, 1, 'true', None]:
            parameters = self.parameters();parameters['sensor_map']['enabled'] = value
            with self.assertRaises(ValueError):u.assert_geometry_free(parameters)
    def test_invalid_waypoint_and_hidden_primitive_string_rejected(self):
        for config in [dict(self.source, motion_primitive={'map_file': 'truth.yaml'}),
                       dict(self.source, waypoints={'num': 1, 'wp1': {'x': True, 'y': 0, 'h': 40}})]:
            self.path.write_text(yaml.safe_dump(config))
            with self.assertRaises(ValueError):u.sanitized_task(self.path)
    def test_only_leading_map_waiting_is_excluded(self):
        waiting = {'status': 'MAP_WAITING', 'command_published': False}
        verified = {'status': 'RECOVERY_VERIFIED', 'command_published': True}
        self.assertEqual(u.active_statuses([waiting, waiting, verified, waiting]), [verified, waiting])
        bad_wait = dict(waiting, command_published=True)
        self.assertEqual(u.active_statuses([bad_wait]), [bad_wait])

    def test_binary_map_snapshot_preserves_free_occupied_split(self):
        import numpy as np
        path = Path(self.directory.name)
        xyz = np.array([[1.,2.,3.],[-2.,0.,4.],[8.,9.,10.]],dtype='<f4')
        raw = struct.pack('<8sddQQ',b'SENMAP01',2.,4.2,2,1)+xyz.tobytes()
        (path/'sensor_map.bin').write_bytes(raw)
        result = u.convert_map_snapshot(path)
        self.assertEqual(result['free_cells'],2)
        self.assertEqual(result['occupied_cells'],1)
        with np.load(path/'map_snapshot.npz') as data:
            np.testing.assert_array_equal(data['free'],xyz[:2])
            np.testing.assert_array_equal(data['occupied'],xyz[2:])
        (path/'sensor_map.bin').write_bytes(raw[:-1])
        with self.assertRaises(ValueError):u.convert_map_snapshot(path)

    def test_actual_mesh_criterion_is_explicit_and_raw_aabb_result_is_retained(self):
        path = Path(self.directory.name)
        (path/'flight').mkdir()
        task = {'box_obstacles':{'num':1,'box1':dict(x=0.,y=0.,z=10.,size_x=4.,size_y=4.,size_z=4.,clearance=2.)}}
        (path/'flight/task_config.yaml').write_text(yaml.safe_dump(task))
        waiting = {'status':'MAP_WAITING','primary_feasible':False,'command_published':False}
        valid = {'status':'RECOVERY_VERIFIED','primary_feasible':True,'command_published':True,
                 'recovery_verified':True,'reused_verified_plan':False,'sensor_map_enabled':True,
                 'truth_obstacle_count':0,'map_ready':True,'sensor_map_current_state_free':True,
                 'map_age':.1,'observation_stamp':1.,'command_stamp':1.02,'dispatch_validation_age':.004,
                 'candidate_order':'learned'}
        flight = {'passed':False,'stop_reason':'completed_first_waypoint_cycle','odom_count':2,'error':None,
                  'waypoint_gate_checks_passed':True,'physical_box_intrusion_samples':2,
                  'inflated_box_intrusion_samples':2,'last_sim_time':2.}
        trace = {'status':[waiting,valid],'odom':[{'t':1.,'x':0.,'y':0.,'z':10.},{'t':2.,'x':1.,'y':0.,'z':10.}],
                 'parse_errors':[]}
        (path/'flight/result.json').write_text(json.dumps(flight))
        (path/'run.json').write_text(json.dumps({'order':'learned'}))
        (path/'flight/observations.json.gz').write_bytes(gzip.compress(json.dumps(trace).encode()))
        telemetry = {'status':[],'sensor_map':[],'errors':[]}
        (path/'telemetry.json.gz').write_bytes(gzip.compress(json.dumps(telemetry).encode()))
        for name in ['planner_parameters_initial.yaml','planner_parameters_final.yaml']:
            (path/name).write_text(yaml.safe_dump(self.parameters()))
        mesh = path/'triangles.npz';mesh.write_bytes(b'synthetic mesh fixture')
        class FakeTruth:
            clearance = 4.8
            def __init__(self,*_):pass
            def segment_clearance(self,*_,**__):return self.clearance
        old = sys.modules.get('collision_mesh_truth')
        sys.modules['collision_mesh_truth'] = types.SimpleNamespace(MeshTruth=FakeTruth)
        try:
            result = u.summarize(path,'unknown','flight_observer_complete',mesh)
            self.assertTrue(result['passed'])
            self.assertFalse(result['raw_flight_observer_passed'])
            self.assertEqual(result['clearance_criterion'],'physical_collision_mesh_isotropic_margin')
            self.assertGreater(result['segment_audit']['expanded']['count'],0)
            self.assertEqual(result['mesh_audit']['margin_violation_count'],0)
            self.assertEqual(result['startup_wait_updates'],1)
            FakeTruth.clearance = 1.9
            self.assertFalse(u.summarize(path,'unknown','flight_observer_complete',mesh)['passed'])
            FakeTruth.clearance = 4.8
            trace['status'][1] = dict(valid,candidate_order='diverse')
            (path/'flight/observations.json.gz').write_bytes(gzip.compress(json.dumps(trace).encode()))
            mismatch = u.summarize(path,'unknown','flight_observer_complete',mesh)
            self.assertFalse(mismatch['passed'])
            self.assertEqual(mismatch['candidate_order_mismatch_count'],1)
            trace['status'][1] = valid
            trace['status'].append(waiting)
            (path/'flight/observations.json.gz').write_bytes(gzip.compress(json.dumps(trace).encode()))
            self.assertFalse(u.summarize(path,'unknown','flight_observer_complete',mesh)['passed'])
        finally:
            if old is None:sys.modules.pop('collision_mesh_truth',None)
            else:sys.modules['collision_mesh_truth'] = old

    def test_json_writer_normalizes_nested_numpy_scalars(self):
        import numpy as np
        path = Path(self.directory.name)/'numpy.json'
        u.write_json(path,{'outer':[np.bool_(True),np.int64(7),{'value':np.float32(1.5)}]})
        self.assertEqual(json.loads(path.read_text()),{'outer':[True,7,{'value':1.5}]})
        with self.assertRaises(ValueError):u.write_json(path,{'invalid':np.float64(float('nan'))})
        with self.assertRaises(TypeError):u.write_json(path,{'invalid':object()})

    def test_real_mesh_summary_and_snapshot_are_plain_json_and_numpy(self):
        import numpy as np
        path = Path(self.directory.name);(path/'flight').mkdir()
        mesh_spec = importlib.util.spec_from_file_location('collision_mesh_truth',SCRIPT.parent/'collision_mesh_truth.py')
        helper = importlib.util.module_from_spec(mesh_spec);mesh_spec.loader.exec_module(helper)
        triangles = helper._box_triangles([2.,2.,2.]) + np.array([0.,4.,10.])
        mesh = path/'real_mesh.npz'
        np.savez_compressed(mesh,triangles=triangles,object_offsets=np.array([0,len(triangles)],np.int64),
                            object_names=np.array(['fixture_box']),closed=np.array([True]),approximation_error=np.array([0.]))
        truth = {'box_obstacles':{'num':1,'box1':dict(x=100.,y=100.,z=100.,size_x=1.,size_y=1.,size_z=1.,clearance=2.)}}
        (path/'flight/task_config.yaml').write_text(yaml.safe_dump(truth))
        good = {'status':'RECOVERY_VERIFIED','primary_feasible':True,'command_published':True,
                'recovery_verified':True,'reused_verified_plan':False,'sensor_map_enabled':True,
                'truth_obstacle_count':0,'map_ready':True,'sensor_map_current_state_free':True,
                'candidate_order':'learned','map_age':.1,'observation_stamp':1.,'command_stamp':1.02,'dispatch_validation_age':.004}
        flight = {'passed':True,'stop_reason':'completed_first_waypoint_cycle','odom_count':2,'error':None,
                  'waypoint_gate_checks_passed':True,'physical_box_intrusion_samples':0,'inflated_box_intrusion_samples':0,'last_sim_time':2.}
        trace = {'status':[good],'odom':[{'t':1.,'x':0.,'y':0.,'z':10.},{'t':2.,'x':1.,'y':0.,'z':10.}],'parse_errors':[]}
        (path/'flight/result.json').write_text(json.dumps(flight))
        (path/'flight/observations.json.gz').write_bytes(gzip.compress(json.dumps(trace).encode()))
        (path/'telemetry.json.gz').write_bytes(gzip.compress(json.dumps({'status':[],'sensor_map':[],'errors':[]}).encode()))
        (path/'run.json').write_text(json.dumps({'order':'learned'}))
        for name in ['planner_parameters_initial.yaml','planner_parameters_final.yaml']:
            (path/name).write_text(yaml.safe_dump(self.parameters()))
        old = sys.modules.get('collision_mesh_truth');sys.modules['collision_mesh_truth'] = helper
        try:
            result = u.summarize(path,'unknown','flight_observer_complete',mesh,2.,5.)
            saved = json.loads((path/'summary.json').read_text())
            self.assertTrue(saved['passed'])
            self.assertIs(type(saved['passed']),bool)
            self.assertIs(type(saved['mesh_audit']['minimum_is_lower_bound']),bool)
            self.assertAlmostEqual(saved['mesh_audit']['minimum_surface_clearance_m'],3.)
            self.assertFalse(saved['mesh_audit']['minimum_is_lower_bound'])
            xyz = np.array([[0.,0.,10.],[0.,4.,10.]],dtype='<f4')
            (path/'sensor_map.bin').write_bytes(struct.pack('<8sddQQ',b'SENMAP01',2.,2.,1,1)+xyz.tobytes())
            snapshot = u.convert_map_snapshot(path)
            self.assertEqual(json.loads((path/'map_snapshot.json').read_text())['occupied_cells'],1)
            with np.load(path/'map_snapshot.npz') as data:
                np.testing.assert_array_equal(data['free'],xyz[:1])
                np.testing.assert_array_equal(data['occupied'],xyz[1:])
        finally:
            if old is None:sys.modules.pop('collision_mesh_truth',None)
            else:sys.modules['collision_mesh_truth'] = old


if __name__ == '__main__':
    unittest.main()
