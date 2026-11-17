#!/usr/bin/env python3
"""Create the unknown-map sensor variant; retain the world outside three leaves."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


SENSOR_PATH = "world/model[@name='suav1']/link[@name='base_link']/sensor[@name='lidar_3d']"
VERTICAL_PATH = SENSOR_PATH + '/ray/scan/vertical'


def digest(value):
    return hashlib.sha256(value).hexdigest()


def unique(root, path):
    nodes = root.findall(path)
    if len(nodes) != 1:
        raise ValueError('Expected exactly one node at ' + path)
    return nodes[0]


def parameters(root):
    sensor = unique(root, SENSOR_PATH)
    get = lambda path: float(unique(sensor, path).text)
    return {
        'vertical_samples': int(get('ray/scan/vertical/samples')),
        'vertical_min_angle_deg': math.degrees(get('ray/scan/vertical/min_angle')),
        'vertical_max_angle_deg': math.degrees(get('ray/scan/vertical/max_angle')),
        'horizontal_samples': int(get('ray/scan/horizontal/samples')),
        'horizontal_min_angle_rad': get('ray/scan/horizontal/min_angle'),
        'horizontal_max_angle_rad': get('ray/scan/horizontal/max_angle'),
        'range_min_m': get('ray/range/min'),
        'range_max_m': get('ray/range/max'),
        'update_rate_hz': get('update_rate'),
    }


def signature(element):
    # Ignore serialization-only whitespace, retain every XML attribute and leaf.
    return [element.tag, sorted(element.attrib.items()),
            ' '.join((element.text or '').split()),
            [signature(child) for child in element]]


def normalized_world(root):
    # Only these three sensor settings are allowed to differ.
    for field in ('samples', 'min_angle', 'max_angle'):
        unique(root, VERTICAL_PATH + '/' + field).text = '__VERTICAL_SETTING__'
    return json.dumps(signature(root), separators=(',', ':')).encode()


def collision_geometry(root):
    # Include ancestor model/link poses in addition to each complete collision.
    rows = []
    for model in root.findall('world/model'):
        for link in model.findall('link'):
            rows.append([model.get('name'), model.findtext('pose'),
                         link.get('name'), link.findtext('pose'),
                         [signature(c) for c in link.findall('collision')]])
    return json.dumps(rows, separators=(',', ':')).encode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--manifest', type=Path)
    args = parser.parse_args()
    source, target = args.input.resolve(), args.output.resolve()
    if source == target:
        parser.error('Output must differ from the input world')
    manifest_path = (args.manifest.resolve() if args.manifest else
                     target.with_suffix('.manifest.json'))
    if manifest_path in (source, target):
        parser.error('Manifest must differ from both world paths')
    original = source.read_bytes()
    before_root = ET.fromstring(original)
    before = parameters(before_root)
    matches = list(re.finditer(rb'<sensor\b[^>]*\bname=[\"\']lidar_3d[\"\'][^>]*>.*?</sensor>',
                               original, re.S))
    if len(matches) != 1:
        parser.error('Expected one serialized lidar_3d sensor in the input world')
    match = matches[0]
    sensor_bytes = match.group()
    vertical = list(re.finditer(rb'<vertical\b[^>]*>.*?</vertical>', sensor_bytes, re.S))
    if len(vertical) != 1:
        parser.error('Expected one vertical scan block in lidar_3d')
    setting_match = vertical[0]
    settings = setting_match.group()
    updates = {'samples': '64', 'min_angle': repr(math.radians(-60)),
               'max_angle': repr(math.radians(60))}
    for name, value in updates.items():
        pattern = rb'(<'+name.encode()+rb'>)(.*?)(</'+name.encode()+rb'>)'
        settings, count = re.subn(pattern, lambda m: m[1] + value.encode() + m[3], settings, flags=re.S)
        if count != 1:
            parser.error('Expected exactly one vertical/' + name)
    patched_sensor = (sensor_bytes[:setting_match.start()] + settings +
                      sensor_bytes[setting_match.end():])
    patched = original[:match.start()] + patched_sensor + original[match.end():]
    after_root = ET.fromstring(patched)
    after = parameters(after_root)
    physical_before = digest(collision_geometry(before_root))
    physical_after = digest(collision_geometry(after_root))
    unchanged_before = digest(normalized_world(before_root))
    unchanged_after = digest(normalized_world(after_root))
    if physical_before != physical_after or unchanged_before != unchanged_after:
        raise RuntimeError('World changed outside the three permitted vertical sensor settings')
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(patched)
    manifest = {
        'input': str(source), 'output': str(target),
        'input_sha256': digest(original), 'output_sha256': digest(patched),
        'sensor_xpath': SENSOR_PATH,
        'changed_leaf_paths': [VERTICAL_PATH + '/' + name for name in updates],
        'sensor_before': before, 'sensor_after': after,
        'collision_geometry_and_ancestor_poses_sha256': physical_after,
        'world_except_three_vertical_settings_sha256': unchanged_after,
        'collision_geometry_and_model_poses_unchanged': True,
        'all_other_xml_content_unchanged': True,
        'input_world_unchanged': digest(source.read_bytes()) == digest(original),
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
