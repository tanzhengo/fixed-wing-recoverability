#!/usr/bin/env python3
"""Append static high-rise buildings and matching planning boxes to a city.

All source world elements, including its saved state and aircraft, are retained.
New buildings use their SDF model poses; no saved-state override is introduced.
The facades use built-in colored geometry, so no external assets are required.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


def add(parent, tag, value):
    element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def numbers(values):
    return ' '.join(format(float(value), '.12g') for value in values)


def box_geometry(parent, dimensions):
    box = ET.SubElement(ET.SubElement(parent, 'geometry'), 'box')
    add(box, 'size', numbers(dimensions))


def visual(link, name, dimensions, position, color):
    item = ET.SubElement(link, 'visual', name=name)
    add(item, 'pose', numbers(tuple(position) + (0, 0, 0)))
    box_geometry(item, dimensions)
    material = ET.SubElement(item, 'material')
    add(material, 'ambient', color)
    add(material, 'diffuse', color)
    add(material, 'specular', '0.35 0.4 0.45 1')
    return item


def high_rise(building, index):
    x, y, z = (building[k] for k in ('x', 'y', 'z'))
    sx, sy, height = (building[k] for k in ('size_x', 'size_y', 'height'))
    model = ET.Element('model', name=building['name'])
    add(model, 'static', 'true')
    add(model, 'pose', numbers((x, y, z, 0, 0, 0)))
    link = ET.SubElement(model, 'link', name='building')
    collision = ET.SubElement(link, 'collision', name='collision')
    box_geometry(collision, (sx, sy, height))
    # One collision box per tower. Facade details remain inside that box and
    # therefore agree with the geometry measured by the CPU ray lidar.
    palette = ('0.30 0.42 0.50 1', '0.38 0.46 0.51 1', '0.27 0.36 0.45 1')
    inset = min(0.02, sx / 20, sy / 20, height / 20)
    visual(link, 'glass_facade', (sx - 2 * inset, sy - 2 * inset, height - 2 * inset),
           (0, 0, 0), palette[index % len(palette)])
    band_color = '0.65 0.68 0.70 1'
    # Sparse floor groups keep rendering light while making the boxes read as
    # high-rise facades. The full-width slabs show on each of the four sides.
    floor_groups = min(14, max(2, int(math.ceil(height / 10.0))))
    thickness = min(0.45, height / (4.0 * floor_groups))
    for level in range(1, floor_groups):
        local_z = -height / 2 + height * level / floor_groups
        visual(link, 'floor_band_%02d' % level, (sx, sy, thickness),
               (0, 0, local_z), band_color)
    cap = min(1.2, height / 10)
    visual(link, 'roof_cap', (sx, sy, cap), (0, 0, height / 2 - cap / 2), band_color)
    visual(link, 'ground_plinth', (sx, sy, cap), (0, 0, -height / 2 + cap / 2),
           '0.45 0.47 0.49 1')
    return model


def read_buildings(path, occupied_names):
    document = json.loads(path.read_text())
    rows = document.get('buildings') if isinstance(document, dict) else document
    if not isinstance(rows, list) or not rows:
        raise ValueError('The buildings file must contain a nonempty buildings array')
    result = []
    names = set(occupied_names)
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError('Each building must be an object')
        name = row.get('name')
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
            raise ValueError('Building names must use letters, digits or underscores')
        if name in names:
            raise ValueError('Duplicate or existing model name: ' + name)
        names.add(name)
        record = {'name': name}
        for key in ('x', 'y', 'size_x', 'size_y', 'height'):
            value = row.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('%s: %s must be a finite number' % (name, key))
            record[key] = float(value)
        if min(record[k] for k in ('size_x', 'size_y', 'height')) <= 0:
            raise ValueError(name + ': dimensions must be positive')
        z = row.get('z', record['height'] / 2)
        if isinstance(z, bool) or not isinstance(z, (int, float)) or not math.isfinite(z):
            raise ValueError(name + ': z must be finite')
        record['z'] = float(z)
        result.append(record)
    return result


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for option in ('base-world', 'base-task', 'buildings', 'output-world', 'output-task', 'manifest'):
        parser.add_argument('--' + option, type=Path, required=True)
    args = parser.parse_args()
    inputs = [args.base_world, args.base_task, args.buildings]
    outputs = [args.output_world, args.output_task, args.manifest]
    if len({p.resolve() for p in outputs}) != len(outputs):
        parser.error('Output paths must be distinct')
    if {p.resolve() for p in inputs} & {p.resolve() for p in outputs}:
        parser.error('Outputs must not overwrite any input')
    try:
        tree = ET.parse(args.base_world)
        world = tree.getroot().find('world')
        if world is None:
            raise ValueError('The source has no SDF world')
        source_world = copy.deepcopy(world)
        names = [node.get('name') for node in world.findall('model')]
        names.extend(node.findtext('name') for node in world.findall('include'))
        names.extend(node.get('name') for node in world.findall('state/model'))
        buildings = read_buildings(args.buildings, names)
        task = yaml.safe_load(args.base_task.read_text())
        if not isinstance(task, dict):
            raise ValueError('The source task must be a mapping')
        original_task = copy.deepcopy(task)
        boxes = task.setdefault('box_obstacles', {'num': 0})
        if not isinstance(boxes, dict):
            raise ValueError('box_obstacles must be a mapping')
        count = boxes.get('num')
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise ValueError('box_obstacles.num must be a nonnegative integer')
        if any('box%d' % i not in boxes for i in range(1, count + 1)):
            raise ValueError('The source box list is incomplete')
        for index, building in enumerate(buildings):
            key = 'box%d' % (count + index + 1)
            if key in boxes:
                raise ValueError('Unexpected existing planning box: ' + key)
            world.append(high_rise(building, index))
            boxes[key] = {'x': building['x'], 'y': building['y'], 'z': building['z'],
                          'size_x': building['size_x'], 'size_y': building['size_y'],
                          'size_z': building['height'], 'clearance': 2.0,
                          'source': building['name'] + '::building::collision'}
        boxes['num'] = count + len(buildings)
    except (ValueError, OSError, ET.ParseError, yaml.YAMLError) as error:
        parser.error(str(error))
    # These invariants also cover the saved world state, aircraft, goals and
    # planner settings; only the appended models and boxes are different.
    retained = copy.deepcopy(world)
    for model in list(retained)[len(source_world):]:
        retained.remove(model)
    assert ET.tostring(retained) == ET.tostring(source_world)
    unchanged_task = copy.deepcopy(task)
    for index in range(count + 1, boxes['num'] + 1):
        del unchanged_task['box_obstacles']['box%d' % index]
    unchanged_task['box_obstacles']['num'] = count
    if 'box_obstacles' not in original_task:
        del unchanged_task['box_obstacles']
    assert unchanged_task == original_task
    for path in outputs:
        path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output_world, encoding='utf-8', xml_declaration=True)
    args.output_world.write_text('\n'.join(line.rstrip() for line in
                                          args.output_world.read_text().splitlines()) + '\n')
    args.output_task.write_text(yaml.safe_dump(task, sort_keys=False))
    manifest = {
        'inputs_sha256': {key: sha256(value) for key, value in
                          (('base_world', args.base_world), ('base_task', args.base_task),
                           ('buildings', args.buildings))},
        'outputs_sha256': {'world': sha256(args.output_world), 'task': sha256(args.output_task)},
        'original_planning_boxes': count,
        'added_planning_boxes': len(buildings),
        'total_planning_boxes': boxes['num'],
        'buildings': buildings,
        'all_original_world_elements_unchanged': True,
        'original_task_except_added_boxes_unchanged': True,
        'new_collision_geometry': 'One exact axis-aligned box per static building',
        'planning_clearance_m': 2.0,
        'saved_state_policy': 'Original state retained; new buildings load from their model poses',
        'external_assets_required_for_new_buildings': False,
    }
    args.manifest.write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
