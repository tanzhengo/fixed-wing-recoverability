#!/usr/bin/env python3
"""Plot a recorded dense-city flight and its commanded target altitude."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Patch
from matplotlib.lines import Line2D
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--task-config', type=Path, required=True)
    parser.add_argument('--buildings', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    raw = args.trace.read_bytes()
    trace = json.loads(gzip.decompress(raw) if args.trace.suffix == '.gz' else raw)
    task = yaml.safe_load(args.task_config.read_text())
    additions = json.loads(args.buildings.read_text())
    additions = additions['buildings'] if isinstance(additions, dict) else additions
    odometry = trace['odom']
    if not odometry:
        raise ValueError('The supplied flight trace has no odometry samples')
    xyz = np.asarray([[row[k] for k in ('x', 'y', 'z')] for row in odometry], dtype=float)
    time = np.asarray([row['t'] for row in odometry], dtype=float)
    boxes_cfg = task['box_obstacles']
    boxes = [boxes_cfg['box' + str(i)] for i in range(1, boxes_cfg['num'] + 1)]
    new_names = {b['name'] for b in additions}
    old_boxes = [b for b in boxes if b.get('source', '').split('::')[0] not in new_names
                 and b['z'] + b['size_z'] / 2 > 1.0]
    targets_cfg = task['waypoints']
    targets = np.asarray([[targets_cfg['wp' + str(i)][k] for k in ('x', 'y', 'h')]
                          for i in range(1, targets_cfg['num'] + 1)])
    # Require every supplied tower to be present in this run's task map.
    for building in additions:
        matches = [b for b in boxes if b.get('source', '').split('::')[0] == building['name']]
        if len(matches) != 1:
            raise ValueError('Expected one planning box for ' + building['name'])
        box = matches[0]
        for key in ('x', 'y', 'size_x', 'size_y'):
            if not np.isclose(box[key], building[key]):
                raise ValueError('Building geometry differs from task: ' + building['name'])
        if not np.isclose(box['size_z'], building['height']):
            raise ValueError('Building height differs from task: ' + building['name'])

    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False})
    fig, (map_ax, height_ax) = plt.subplots(1, 2, figsize=(13, 7.1),
        gridspec_kw={'width_ratios': [1.1, 1]}, constrained_layout=True)
    for box in old_boxes:
        map_ax.add_patch(Rectangle((box['x'] - box['size_x']/2, box['y'] - box['size_y']/2),
            box['size_x'], box['size_y'], facecolor='#dddddd', edgecolor='#9b9b9b', lw=.4))
    for index, building in enumerate(additions, 1):
        map_ax.add_patch(Rectangle((building['x'] - building['size_x']/2,
                                   building['y'] - building['size_y']/2),
            building['size_x'], building['size_y'], facecolor='#72afd2',
            edgecolor='#23658d', lw=.9, zorder=3))
        map_ax.text(building['x'], building['y'], 'N%d\n%g m' % (index, building['height']),
            ha='center', va='center', fontsize=7, color='#103d58', zorder=6)
    map_ax.plot(xyz[:, 0], xyz[:, 1], color='#dc6b26', lw=1.8, zorder=4)
    map_ax.scatter(targets[:, 0], targets[:, 1], marker='*', s=85,
                   color='#202f48', zorder=5)
    for index, point in enumerate(targets, 1):
        map_ax.annotate(str(index), point[:2], xytext=(5, 5),
            textcoords='offset points', color='#202f48', fontsize=9)
    map_ax.scatter(xyz[0, 0], xyz[0, 1], s=30, color='#289363', zorder=7)
    map_ax.scatter(xyz[-1, 0], xyz[-1, 1], s=28, color='#222222', zorder=7)
    # Set map extent from the recorded route, requested targets and additions.
    # Existing peripheral buildings outside this extent do not set its scale.
    points = np.vstack([xyz[:, :2], targets[:, :2],
        np.asarray([[b['x'] + sx*b['size_x']/2, b['y'] + sy*b['size_y']/2]
                    for b in additions for sx in (-1, 1) for sy in (-1, 1)])])
    lo, hi = points.min(axis=0)-55, points.max(axis=0)+40
    map_ax.set_xlim(lo[0], hi[0]); map_ax.set_ylim(lo[1], hi[1])
    map_ax.set_aspect('equal', adjustable='box')
    map_ax.set(xlabel='X (m)', ylabel='Y (m)', title='(a) Dense city and recorded flight')
    map_ax.grid(alpha=.15)
    map_ax.legend(handles=[Patch(facecolor='#dddddd', edgecolor='#9b9b9b', label='Existing buildings'),
        Patch(facecolor='#72afd2', edgecolor='#23658d', label='Added high-rises'),
        Line2D([], [], color='#dc6b26', lw=1.8, label='Recorded flight'),
        Line2D([], [], color='#202f48', marker='*', markersize=9, linestyle='', label='Eight waypoints')],
        loc='upper left', bbox_to_anchor=(1.02, 1.0), borderaxespad=0, fontsize=8, framealpha=.94)
    height_ax.plot(time, xyz[:, 2], color='#176e91', lw=2, label='Recorded altitude')
    goals = trace.get('goal', [])
    if goals:
        target_times = [row['t'] for row in goals]
        target_heights = [row['z'] for row in goals]
        if target_times[-1] < time[-1]:
            target_times.append(float(time[-1])); target_heights.append(target_heights[-1])
        height_ax.step(target_times, target_heights, where='post', color='#dc6b26',
                       ls='--', lw=1.5, label='Active target altitude')
    height_ax.set(xlabel='Simulation time (s)', ylabel='Altitude (m)',
                  title='(b) Flight altitude and target')
    height_ax.set_xlim(time[0], max(time[-1], time[0]+.01))
    height_ax.grid(alpha=.2); height_ax.legend(loc='lower right', fontsize=9)
    fig.suptitle('Gazebo flight with eight additional high-rises', fontsize=14)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180); plt.close(fig)
    print(json.dumps({'trace': str(args.trace), 'trace_sha256': hashlib.sha256(raw).hexdigest(),
        'output': str(args.output), 'odom_samples': len(odometry),
        'new_buildings': len(additions), 'waypoints': len(targets),
        'simulation_time_start_s': float(time[0]), 'simulation_time_end_s': float(time[-1])}))


if __name__ == '__main__':
    main()
