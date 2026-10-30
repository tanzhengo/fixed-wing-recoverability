#!/usr/bin/env python3
"""Add the aircraft's 3D ray lidar without changing the city or flight dynamics."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def add(parent, name, text):
    child = ET.SubElement(parent, name)
    child.text = str(text)
    return child


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-world', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--horizontal-samples', type=int, default=512)
    parser.add_argument('--vertical-samples', type=int, default=32)
    parser.add_argument('--update-rate', type=float, default=5.0)
    parser.add_argument('--max-range', type=float, default=120.0)
    args = parser.parse_args()
    if args.base_world.resolve() == args.output.resolve():
        parser.error('The output must differ from the source world')
    if min(args.horizontal_samples, args.vertical_samples) < 2:
        parser.error('Both scan dimensions must contain at least two rays')
    if not all(math.isfinite(value) and value > 0 for value in (args.update_rate, args.max_range)):
        parser.error('Rate and range must be positive and finite')
    if args.max_range <= 1.0:
        parser.error('Maximum range must exceed the 1.0 m minimum range')
    tree = ET.parse(args.base_world)
    link = tree.getroot().find("world/model[@name='suav1']/link[@name='base_link']")
    if link is None:
        parser.error('The source world does not contain suav1/base_link')
    for node in list(link):
        if node.get('name') in ('lidar_3d', 'lidar_housing', 'lidar_band'):
            link.remove(node)
    for name, z, length, radius, color in (
            ('lidar_housing', .15, .06, .055, '.1 .12 .15 1'),
            ('lidar_band', .16, .015, .057, '0 .8 .9 1')):
        visual = ET.SubElement(link, 'visual', name=name)
        add(visual, 'pose', '0 0 %s 0 0 0' % z)
        cylinder = ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder')
        add(cylinder, 'radius', radius); add(cylinder, 'length', length)
        material = ET.SubElement(visual, 'material')
        add(material, 'ambient', color); add(material, 'diffuse', color)
    sensor = ET.SubElement(link, 'sensor', name='lidar_3d', type='ray')
    add(sensor, 'pose', '0 0 0.15 0 0 0')
    add(sensor, 'always_on', 'true'); add(sensor, 'visualize', 'false')
    add(sensor, 'update_rate', args.update_rate)
    ray = ET.SubElement(sensor, 'ray'); scan = ET.SubElement(ray, 'scan')
    horizontal = ET.SubElement(scan, 'horizontal')
    add(horizontal, 'samples', args.horizontal_samples); add(horizontal, 'resolution', 1)
    add(horizontal, 'min_angle', -math.pi)
    # One ray per azimuth: do not repeat -pi at +pi.
    add(horizontal, 'max_angle', math.pi-2*math.pi/args.horizontal_samples)
    vertical = ET.SubElement(scan, 'vertical')
    add(vertical, 'samples', args.vertical_samples); add(vertical, 'resolution', 1)
    add(vertical, 'min_angle', math.radians(-40)); add(vertical, 'max_angle', math.radians(20))
    ranges = ET.SubElement(ray, 'range')
    add(ranges, 'min', 1.0); add(ranges, 'max', args.max_range); add(ranges, 'resolution', .01)
    plugin = ET.SubElement(sensor, 'plugin', name='lidar_pointcloud', filename='libgazebo_ros_3d_lidar.so')
    add(plugin, 'robotNamespace', '/suav1'); add(plugin, 'topicName', 'lidar/points_raw')
    add(plugin, 'frameName', 'suav1/lidar')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding='utf-8', xml_declaration=True)
    args.output.write_text('\n'.join(line.rstrip() for line in args.output.read_text().splitlines())+'\n')
    print(json.dumps({'source_sha256': hashlib.sha256(args.base_world.read_bytes()).hexdigest(),
                      'world_sha256': hashlib.sha256(args.output.read_bytes()).hexdigest(),
                      'horizontal_samples': args.horizontal_samples, 'vertical_samples': args.vertical_samples,
                      'update_rate_hz': args.update_rate, 'max_range_m': args.max_range,
                      'frame': 'suav1/lidar', 'topic': '/suav1/lidar/points_raw',
                      'extrinsic_xyz_rpy': [0,0,.15,0,0,0]}, indent=2))


if __name__ == '__main__':
    main()
