#!/usr/bin/env python3
"""Publication figure from the archived dense-city Gazebo flight, not UI captures."""
import gzip
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import colors, cm
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle, Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import yaml


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    repo = Path(__file__).resolve().parents[1]
    evidence = repo / 'docs/recovery/dense_city/evidence/flight1'
    trace_path = evidence / 'flight/observations.json.gz'
    task_path = evidence / 'flight/task_config.yaml'
    snapshot_path = evidence / 'mapping/snapshots.npz'
    building_path = repo / 'motion_primitives_planning/launch/config/end_city_dense_buildings.json'
    trace = json.loads(gzip.decompress(trace_path.read_bytes()))
    task = yaml.safe_load(task_path.read_text())
    buildings = json.loads(building_path.read_text())['buildings']
    snapshot = np.load(snapshot_path)
    odom = trace['odom']
    t = np.asarray([v['t'] for v in odom])
    xyz = np.asarray([[v[k] for k in ('x', 'y', 'z')] for v in odom])
    roll = np.degrees([v['roll'] for v in odom])
    physical = np.asarray([v['physical_box_clearance'] for v in odom])
    inflated = np.asarray([v['inflated_box_clearance'] for v in odom])
    box_config = task['box_obstacles']
    boxes = [box_config['box%d' % i] for i in range(1, box_config['num'] + 1)]
    goals = task['waypoints']
    waypoints = np.asarray([[goals['wp%d' % i][k] for k in ('x', 'y', 'h')]
                           for i in range(1, goals['num'] + 1)])
    new_names = {b['name'] for b in buildings}
    voxels = np.asarray(snapshot['map'], dtype=float).reshape((-1, 3))
    resolution = float(snapshot['resolution'][0])
    assert np.isfinite(voxels).all()
    ground_threshold = 2.0
    elevated = voxels[voxels[:, 2] > ground_threshold]
    max_cubes = 18000
    stride = max(1, int(np.ceil(len(elevated) / max_cubes)))
    shown = elevated[::stride]

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 8,
                         'axes.titlesize': 8.5, 'axes.labelsize': 8,
                         'xtick.labelsize': 7, 'ytick.labelsize': 7,
                         'legend.fontsize': 6.7, 'axes.linewidth': .65,
                         'lines.linewidth': 1.0, 'pdf.fonttype': 42,
                         'ps.fonttype': 42, 'axes.spines.top': False,
                         'axes.spines.right': False})
    fig = plt.figure(figsize=(7.1, 5.35))
    grid = fig.add_gridspec(2, 2, left=.09, right=.95, bottom=.085, top=.95,
                           hspace=.48, wspace=.34, height_ratios=[1.12, 1])
    route_ax = fig.add_subplot(grid[0, 0])
    voxel_ax = fig.add_subplot(grid[0, 1], projection='3d')
    envelope = grid[1, 0].subgridspec(2, 1, hspace=.15)
    altitude_ax = fig.add_subplot(envelope[0])
    roll_ax = fig.add_subplot(envelope[1], sharex=altitude_ax)
    clearance_ax = fig.add_subplot(grid[1, 1])

    # The plan view uses Y horizontally to show the long city corridor at
    # undistorted metric scale within a landscape panel. Axes state this order.
    for box in boxes:
        if box['z'] + box['size_z'] / 2 <= 1:
            continue
        added = box.get('source', '').split('::')[0] in new_names
        route_ax.add_patch(Rectangle((box['y'] - box['size_y']/2,
                                     box['x'] - box['size_x']/2),
                          box['size_y'], box['size_x'],
                          facecolor='#65a7cb' if added else '#dedede',
                          edgecolor='#286181' if added else '#a5a5a5',
                          linewidth=.45 if added else .3, zorder=2 if added else 1))
    route_ax.plot(xyz[:, 1], xyz[:, 0], color='#d86323', linewidth=1.2, zorder=4)
    route_ax.scatter(waypoints[:, 1], waypoints[:, 0], s=19, marker='*',
                     color='#263954', zorder=5)
    for index, goal in enumerate(waypoints, 1):
        route_ax.annotate(str(index), (goal[1], goal[0]), xytext=(3, 3),
                          textcoords='offset points', fontsize=6.5, zorder=6)
    route_ax.scatter(xyz[0, 1], xyz[0, 0], s=15, color='#1a8559', zorder=7)
    route_ax.set(xlim=(-355, 285), ylim=(-148, 182), xlabel='Y (m)', ylabel='X (m)',
                 title='(a) Urban map and executed route')
    route_ax.set_aspect('equal', adjustable='box')
    route_ax.set_xticks([-300, -150, 0, 150])
    route_ax.set_yticks([-100, 0, 100])
    route_ax.grid(alpha=.16, linewidth=.4)
    handles = [Patch(facecolor='#dedede', edgecolor='#a5a5a5', label='Existing'),
               Patch(facecolor='#65a7cb', edgecolor='#286181', label='Added towers'),
               Line2D([], [], color='#d86323', label='Flight')]
    route_ax.legend(handles=handles, ncol=3, loc='upper center',
                    bbox_to_anchor=(.5, -.28), frameon=False, borderpad=.3,
                    columnspacing=.8, handlelength=1.1)

    half = resolution/2
    corners = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                        [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]]) * half
    vertices = shown[:, None, :] + corners[None, :, :]
    faces = np.array([[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]])
    polygons = vertices[:, faces, :].reshape((-1, 4, 3))
    norm = colors.Normalize(0, 140)
    cmap = cm.get_cmap('viridis')
    poly = Poly3DCollection(polygons, facecolors=np.repeat(cmap(norm(shown[:, 2])), 6, axis=0),
                           edgecolors='none', linewidths=0)
    poly.set_rasterized(True)
    voxel_ax.add_collection3d(poly)
    voxel_ax.set(xlim=(-200, 220), ylim=(-420, 340), zlim=(0, 145),
                 xlabel='X (m)', ylabel='Y (m)', zlabel='',
                 title='(b) Observed LiDAR voxels')
    voxel_ax.set_xticks([-150, 0, 150]); voxel_ax.set_yticks([-300, 0, 300])
    voxel_ax.set_zticks([0, 60, 120])
    voxel_ax.tick_params(axis='both', which='major', pad=-1, labelsize=6)
    voxel_ax.xaxis.labelpad=0; voxel_ax.yaxis.labelpad=0; voxel_ax.zaxis.labelpad=-1
    voxel_ax.view_init(elev=24, azim=-57)
    if hasattr(voxel_ax, 'set_box_aspect'):
        voxel_ax.set_box_aspect([420, 760, 350])
    voxel_ax.dist=9
    voxel_ax.text2D(1.04, .65, 'Z (m)', transform=voxel_ax.transAxes,
                    rotation=90, fontsize=8, ha='left', va='center')
    # Color is height in metres, using the same measured Z shown on this axis.
    color_ax = fig.add_axes([.64, .515, .26, .012])
    scalar = cm.ScalarMappable(norm=norm, cmap=cmap)
    scalar.set_array([])
    colorbar = fig.colorbar(scalar, cax=color_ax, orientation='horizontal',
                           ticks=[0, 70, 140])
    colorbar.set_label('Height (m)', fontsize=7, labelpad=1)
    colorbar.ax.tick_params(labelsize=6.5, pad=1, length=2)
    colorbar.outline.set_linewidth(.4)

    target_times = [v['t'] for v in trace['goal']]
    target_heights = [v['z'] for v in trace['goal']]
    if target_times[-1] < t[-1]:
        target_times.append(float(t[-1])); target_heights.append(target_heights[-1])
    altitude_ax.plot(t, xyz[:, 2], color='#16658a', label='Altitude')
    altitude_ax.step(target_times, target_heights, where='post',
                     color='#d86323', linestyle='--', label='Target')
    altitude_ax.set(title='(c) Altitude and bank angle', ylabel='Altitude (m)', ylim=(30, 65))
    altitude_ax.set_yticks([35, 50, 65]); altitude_ax.tick_params(labelbottom=False)
    altitude_ax.legend(loc='lower right', ncol=2, frameon=False, columnspacing=.8,
                       handlelength=1.5, borderaxespad=.15)
    roll_ax.plot(t, roll, color='#6a5199')
    for boundary in [-35, 35]:
        roll_ax.axhline(boundary, color='#6b6b6b', linestyle=':', linewidth=.8)
    roll_ax.set(ylabel='Roll (deg)', xlabel='Simulation time (s)', ylim=(-42, 42))
    roll_ax.set_yticks([-35, 0, 35])
    roll_ax.text(.02, .74, r'Limits: $\pm35^\circ$', transform=roll_ax.transAxes,
                 fontsize=6.5, color='#555555')
    for ax in [altitude_ax, roll_ax]:
        ax.set_xlim(0, 92); ax.set_xticks([0, 30, 60, 90]); ax.grid(alpha=.18, linewidth=.4)

    clearance_ax.plot(t, physical, color='#16658a', label='Physical boxes')
    clearance_ax.plot(t, inflated, color='#9b5c34', linestyle='--', label='Inflated boxes')
    clearance_ax.axhline(0, color='#555555', linewidth=.7, linestyle=':')
    clearance_ax.set(title='(d) Executed geometric clearance', ylabel='Clearance (m)',
                     xlabel='Simulation time (s)', xlim=(0, 92), ylim=(-2, 65))
    clearance_ax.set_xticks([0, 30, 60, 90]); clearance_ax.grid(alpha=.18, linewidth=.4)
    clearance_ax.legend(loc='upper right', frameon=False, handlelength=1.7)

    output = repo / 'paper/figures/gazebo_city'
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output)+'.pdf', dpi=400)
    fig.savefig(str(output)+'.png', dpi=250)
    plt.close(fig)
    summary = {
        'figure_kind': 'Replotted recorded Gazebo simulation data; not Gazebo/RViz screenshots',
        'input_sha256': {str(p.relative_to(repo)): sha(p) for p in
                         [trace_path, task_path, snapshot_path, building_path]},
        'odometry_samples': len(odom), 'simulation_time_range_s': [float(t[0]), float(t[-1])],
        'waypoints': len(waypoints), 'planning_boxes': len(boxes), 'added_towers': len(buildings),
        'map_projection': 'Y horizontal, X vertical, equal scale; view limits Y [-355,285], X [-148,182] m',
        'minimum_physical_clearance_m': float(physical.min()),
        'minimum_inflated_clearance_m': float(inflated.min()),
        'maximum_abs_roll_deg': float(np.abs(roll).max()),
        'voxel_resolution_m': resolution, 'total_observed_voxels': len(voxels),
        'ground_display_threshold_m': ground_threshold, 'eligible_above_ground_voxels': len(elevated),
        'voxel_display_stride': stride, 'rendered_voxels': len(shown),
        'voxel_display_method': 'Filter voxel centers Z > 2 m, then select every fourth cell in stored order; render true 2 m cubes',
        'voxel_color_range_m': [0, 140],
        'voxel_colorbar_ticks_m': [0, 70, 140],
        'rendering': '3D voxel faces rasterized at 400 dpi in PDF; map, curves and labels retained as vectors',
        'output_sha256': {str(output.with_suffix(ext).relative_to(repo)): sha(output.with_suffix(ext))
                          for ext in ['.pdf', '.png']},
    }
    summary_path = repo/'docs/recovery/dense_city/paper_figure_data.json'
    summary_path.write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
