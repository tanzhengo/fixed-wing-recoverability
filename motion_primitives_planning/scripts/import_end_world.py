#!/usr/bin/env python3
"""Prepare the supplied end.world and a conservative, matching planning map.

Gazebo's loaded collision bounds account for saved state, mesh units, model
scales and all link/collision poses. The city ground mesh is decomposed to avoid
blocking its streets with one bounding box. No source assets are overwritten.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def mesh_components(mesh, scale, matrix, ground_height=.35):
    triangles = []
    for sub in mesh['submeshes']:
        points = np.asarray(sub['vertices']) * scale
        points = points @ matrix[:3, :3].T + matrix[:3, 3]
        triangles.extend(points[np.asarray(sub['indices']).reshape(-1, 3)])
    triangles = np.asarray(triangles)
    elevated = triangles[triangles[:, :, 2].max(axis=1) > ground_height]
    ground = triangles[triangles[:, :, 2].max(axis=1) <= ground_height]
    parent = list(range(len(elevated)))
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    vertices = {}
    for i, triangle in enumerate(elevated):
        for vertex in triangle:
            key = tuple(np.round(vertex, 4))
            if key in vertices:
                parent[root(i)] = root(vertices[key])
            else:
                vertices[key] = i
    groups = {}
    for i in range(len(elevated)):
        groups.setdefault(root(i), []).append(i)
    parts = [elevated[indices] for indices in groups.values()]
    if len(ground):
        parts.append(ground)
    boxes = []
    for part in parts:
        points = part.reshape(-1, 3)
        low, high = points.min(0), points.max(0)
        # Pad degenerate surfaces symmetrically; all input triangles remain inside.
        pad = np.maximum((.02 - (high - low)) / 2, 0)
        boxes.append((low-pad, high+pad, len(part)))
    assert sum(item[2] for item in boxes) == len(triangles)
    return boxes, len(triangles)


def pose_matrix(pose):
    x, y, z, roll, pitch, yaw = pose
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                       [-sp, cp*sr, cp*cr]]
    matrix[:3, 3] = [x, y, z]
    return matrix


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--loaded-geometry', type=Path, required=True)
    p.add_argument('--mcity-mesh', type=Path, required=True)
    p.add_argument('--airplane-world', type=Path, required=True)
    p.add_argument('--output-world', type=Path, required=True)
    p.add_argument('--output-task', type=Path, required=True)
    p.add_argument('--manifest', type=Path, required=True)
    args = p.parse_args()
    inputs = {path.resolve() for path in (args.source, args.loaded_geometry,
              args.mcity_mesh, args.airplane_world)}
    outputs = [path.resolve() for path in (args.output_world, args.output_task, args.manifest)]
    if len(set(outputs)) != len(outputs) or inputs.intersection(outputs):
        p.error('Output files must be distinct from each other and from every input')
    tree = ET.parse(args.source)
    world = tree.getroot().find('world')
    loaded = json.loads(args.loaded_geometry.read_text())['models']
    models = {model.get('name'): model for model in world.findall('model')}
    if set(models) != {model['name'] for model in loaded}:
        raise ValueError('Loaded model inventory differs from source world')
    boxes = []
    def add(low, high, name, triangles=None):
        low, high = np.asarray(low), np.asarray(high)
        if not np.all(np.isfinite([low, high])) or np.any(high < low):
            raise ValueError('Invalid Gazebo collision bounds: ' + name)
        center, size = (low+high)/2, np.maximum(high-low, .02)
        record = dict(zip(('x','y','z','size_x','size_y','size_z'),
                          map(float, np.r_[center, size])))
        record.update(clearance=2.0, source=name)
        if triangles is not None:
            record['triangles'] = triangles
        boxes.append(record)
    covered_triangles = 0
    for model in loaded:
        for collision in model['collisions']:
            if model['name'] == 'cafe':
                source_mesh = models['cafe'].find('link/collision/geometry/mesh')
                if source_mesh.findtext('uri') != 'model://mcity/meshes/mcity.obj':
                    raise ValueError('Unexpected ground mesh')
                scale = np.asarray([float(v) for v in source_mesh.findtext('scale', '1 1 1').split()])
                components, covered_triangles = mesh_components(
                    json.loads(args.mcity_mesh.read_text()), scale, pose_matrix(collision['pose']))
                for i, (low, high, count) in enumerate(components):
                    add(low, high, collision['name'] + '/component' + str(i), count)
            else:
                add(collision['min'], collision['max'], collision['name'])
    # Retain saved model poses/scales: Gazebo applied this state when inspected.
    # Freeze the ten movable street props for this static-map planning test.
    frozen = []
    for name, model in models.items():
        static = model.find('static')
        if static is None:
            static = ET.SubElement(model, 'static')
        if static.text not in ('1', 'true'):
            frozen.append(name)
        static.text = '1'
    state = world.find('state')
    if state is not None:
        for name in ('sim_time', 'real_time', 'wall_time'):
            child = state.find(name)
            if child is not None:
                child.text = '0 0'
        iterations = state.find('iterations')
        if iterations is not None:
            iterations.text = '0'
    airplane = copy.deepcopy(ET.parse(args.airplane_world).getroot().find("world/model[@name='suav1']"))
    if airplane is None:
        raise ValueError('suav1 model missing')
    airplane.find('pose').text = '-90 -270 40 0 0 1.5707963267948966'
    for link in airplane.findall('link'):
        for gravity in link.findall('gravity'):
            link.remove(gravity)
        ET.SubElement(link, 'gravity').text = '0'
    for uri in airplane.findall('.//uri'):
        if uri.text.startswith('package://simple_airplane_description/'):
            uri.text = uri.text.replace('package://', 'model://', 1)
    world.append(airplane)
    gui = world.find('gui')
    if gui is not None:
        # Use a useful city overview; omit the unrelated keyboard GUI plugin.
        for plugin in list(gui.findall('plugin')):
            gui.remove(plugin)
        gui.find('camera/pose').text = '580 -650 480 0 0.56 2.25'
        camera = gui.find('camera')
        for old_track in list(camera.findall('track_visual')):
            camera.remove(old_track)
        track = ET.SubElement(camera, 'track_visual')
        for name, value in [('name','suav1'), ('static','true'),
                            ('use_model_frame','true'), ('xyz','-7 -4 3'),
                            ('inherit_yaw','true')]:
            ET.SubElement(track, name).text = value
    task = {'motion_primitive': {'min_altitude': 5.0, 'max_altitude': 160.0},
            'static_obstacles': {'num': 0}, 'dynamic_obstacles': {'num': 0},
            'sphere_obstacles': {'num': 0}, 'horizontal_obstacles': {'num': 0},
            'box_obstacles': {'num': len(boxes)},
            'waypoints': {'num': 5}}
    task['box_obstacles'].update({'box'+str(i+1): value for i, value in enumerate(boxes)})
    goals = [(-85,-100,40), (-85,180,50), (120,225,60), (125,-265,55), (-90,-270,40)]
    for i, (x,y,h) in enumerate(goals):
        task['waypoints']['wp'+str(i+1)] = {'x': float(x), 'y': float(y), 'h': float(h)}
    for output in (args.output_world, args.output_task, args.manifest):
        output.parent.mkdir(parents=True, exist_ok=True)
    # The input already has indentation; preserve Python 3.8 / ROS Noetic support.
    tree.write(args.output_world, encoding='utf-8', xml_declaration=True)
    args.output_world.write_text('\n'.join(line.rstrip() for line in
                                args.output_world.read_text().splitlines())+'\n')
    args.output_task.write_text(yaml.safe_dump(task, sort_keys=False))
    manifest = {'source_world_sha256': hashlib.sha256(args.source.read_bytes()).hexdigest(),
                'model_count': len(models), 'planner_box_count': len(boxes),
                'mcity_triangles_covered': covered_triangles, 'frozen_street_props': frozen,
                'original_state_geometry_retained': True,
                'airplane': 'suav1', 'initial_pose': [-90,-270,40,0,0,float(np.pi/2)],
                'waypoints': goals, 'box_clearance_m': 2.0,
                'geometry_method': 'Gazebo loaded collision AABBs; mcity ground/elevated connected components',
                'files': {str(path.name): hashlib.sha256(path.read_bytes()).hexdigest() for path in
                          (args.loaded_geometry, args.mcity_mesh, args.output_world, args.output_task)}}
    args.manifest.write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
