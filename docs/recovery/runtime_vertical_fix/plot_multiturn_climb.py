#!/usr/bin/env python3
"""Plot actual Gazebo odometry from an observations.json[.gz] file.

Example:
  python3 plot_multiturn_climb.py --input observations.json.gz \
      --output multiturn_climb.png --altitude-tolerance 3

Flight positions, time, heading turns and target coordinates come exclusively
from the supplied raw trace. The altitude tolerance is a plotting parameter
matching the planner's existing goal acceptance rule.
"""
import argparse
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.ticker import MaxNLocator
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--altitude-tolerance', type=float, default=3.0)
args = parser.parse_args()
data = args.input.read_bytes()
if args.input.suffix == '.gz':
    data = gzip.decompress(data)
trace = json.loads(data)
odom = trace['odom']
assert len(odom) >= 2 and trace['goal'], 'Trace needs odometry and a goal.'
t = np.array([row['t'] for row in odom])
xyz = np.array([[row[k] for k in ('x', 'y', 'z')] for row in odom])
heading = np.array([row['sigma'] for row in odom])
heading_difference = np.diff(heading)
net_turns = np.sum(np.arctan2(np.sin(heading_difference),
                            np.cos(heading_difference))) / (2 * np.pi)
goal = np.array([trace['goal'][-1][k] for k in ('x', 'y', 'z')])
threshold = goal[2] - args.altitude_tolerance
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                     'axes.labelsize': 10, 'axes.titlesize': 11,
                     'xtick.labelsize': 9, 'ytick.labelsize': 9,
                     'axes.spines.top': False, 'axes.spines.right': False})
fig = plt.figure(figsize=(10.2, 4.35))
ax = fig.add_subplot(121, projection='3d')
height_norm = Normalize(vmin=float(xyz[:, 2].min()), vmax=float(goal[2]))
segments = np.stack((xyz[:-1], xyz[1:]), axis=1)
line = Line3DCollection(segments, cmap='viridis', norm=height_norm,
                        linewidth=2.2)
line.set_array((xyz[:-1, 2] + xyz[1:, 2]) / 2)
ax.add_collection3d(line)
ax.scatter(*xyz[0], color='#176f43', s=42, edgecolors='white', linewidths=.6,
           label='Start', depthshade=False)
ax.scatter(*goal, color='#bd3c32', marker='*', s=115, edgecolors='white',
           linewidths=.6, label='Goal', depthshade=False)
ax.scatter(*xyz[-1], color='#20252b', s=23, label='Final sample',
           depthshade=False)
for dim, setter in enumerate([ax.set_xlim, ax.set_ylim, ax.set_zlim]):
    lo, hi = min(xyz[:, dim].min(), goal[dim]), max(xyz[:, dim].max(), goal[dim])
    pad = max((hi - lo) * .08, 2.0)
    setter(lo - pad, hi + pad)
ax.set_xlabel('X (m)', labelpad=6)
ax.set_ylabel('Y (m)', labelpad=6)
ax.set_zlabel('Altitude (m)', labelpad=7)
ax.view_init(elev=24, azim=-58)
ax.xaxis.set_major_locator(MaxNLocator(4))
ax.yaxis.set_major_locator(MaxNLocator(4))
ax.zaxis.set_major_locator(MaxNLocator(5))
ax.set_title('(a) Actual 3D trajectory', pad=11)
ax.legend(loc='upper left', frameon=False, fontsize=9,
          bbox_to_anchor=(-.05, 1.0), handletextpad=.4)
cbar_ax = fig.add_axes([.452, .20, .011, .60])
cbar = fig.colorbar(line, cax=cbar_ax)
cbar.set_label('Altitude (m)', labelpad=5)
cbar.ax.tick_params(labelsize=8)
cbar.set_ticks([10, 40, 70, 100])

ah = fig.add_axes([.60, .17, .365, .70])
ah.plot(t, xyz[:, 2], color='#236a8c', linewidth=2, label='Actual altitude')
ah.axhline(goal[2], color='#bd3c32', linestyle='--', linewidth=1.3,
           label='Goal: {:g} m'.format(goal[2]))
ah.axhline(threshold, color='#8b6a27', linestyle=':', linewidth=1.5,
           label='Acceptance: {:g} m'.format(threshold))
ah.scatter(t[-1], xyz[-1, 2], s=23, color='#20252b', zorder=4)
ah.set_xlabel('Simulation time (s)')
ah.set_ylabel('Altitude (m)')
ah.set_xlim(0, t[-1] * 1.025)
ah.set_ylim(0, goal[2] * 1.09)
ah.grid(True, linewidth=.6, alpha=.22)
ah.set_title('(b) Climb history', pad=11)
ah.legend(loc='upper left', bbox_to_anchor=(.0, .90), frameon=False,
          fontsize=9, handlelength=2.0)
ah.text(.035, .65, 'Net heading turns: {:.3f}'.format(net_turns),
        transform=ah.transAxes, va='top', fontsize=10,
        bbox=dict(facecolor='white', edgecolor='none', alpha=.9, pad=2))
ah.text(.97, .10, 'Final: {:.2f} s, {:.2f} m'.format(t[-1], xyz[-1, 2]),
        transform=ah.transAxes, fontsize=9, ha='right', color='#20252b')
fig.subplots_adjust(left=.015, right=.48, bottom=.09, top=.91, wspace=.0)
# Give the first subplot its intended share; the history uses an explicit box.
ax.set_position([.025, .08, .405, .82])
args.output.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(str(args.output), dpi=220, facecolor='white')
plt.close(fig)
print(json.dumps({'output': str(args.output), 'samples': len(odom),
                  'net_heading_turns': float(net_turns),
                  'final_sim_time': float(t[-1]),
                  'final_height': float(xyz[-1, 2]),
                  'target_from_trace': goal.tolist()}))
