#!/usr/bin/env python3
"""Plot recorded flight over the imported physical planning boxes."""
import argparse
import gzip
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from matplotlib import colors
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--task-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--minimum-roof-height', type=float, default=1.0,
                        help='Hide ground-level box footprints below this height in the plot only')
    args = parser.parse_args()
    raw = args.trace.read_bytes()
    if args.trace.suffix == '.gz': raw = gzip.decompress(raw)
    trace = json.loads(raw); task = yaml.safe_load(args.task_config.read_text())
    od = trace['odom']
    if not od: raise ValueError('No recorded odometry to plot')
    xyz = np.array([[row[k] for k in ('x','y','z')] for row in od])
    t = np.array([row['t'] for row in od])
    box_config = task.get('box_obstacles', {})
    boxes = [box_config['box'+str(i)] for i in range(1,box_config.get('num',0)+1)]
    wp = task['waypoints']
    goals = np.array([[wp['wp'+str(i)][k] for k in ('x','y','h')] for i in range(1,wp['num']+1)])
    shown = [b for b in boxes if b['z'] + b['size_z']/2 > args.minimum_roof_height]
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig, (map_ax, altitude_ax) = plt.subplots(1,2,figsize=(12.4,5.8),gridspec_kw={'width_ratios':[1.25,1]},constrained_layout=True)
    if shown:
        rectangles = [Rectangle((b['x']-b['size_x']/2,b['y']-b['size_y']/2),b['size_x'],b['size_y']) for b in shown]
        heights = np.array([b['z']+b['size_z']/2 for b in shown])
        collection = PatchCollection(rectangles,cmap='Greys',norm=colors.Normalize(0,max(heights)),edgecolor='#777777',linewidth=.35,alpha=.75)
        collection.set_array(heights); map_ax.add_collection(collection)
        bar = fig.colorbar(collection,ax=map_ax,label='Planning box roof height (m)',fraction=.035,pad=.02)
        bar.solids.set_edgecolor('face')
        map_ax.update_datalim(np.array([[b['x']+sx*b['size_x']/2,b['y']+sy*b['size_y']/2] for b in shown for sx in (-1,1) for sy in (-1,1)]))
    map_ax.plot(xyz[:,0],xyz[:,1],color='#d63e28',lw=1.8,label='Recorded flight',zorder=5)
    map_ax.scatter(xyz[0,0],xyz[0,1],color='#22885b',s=45,label='Start',zorder=6)
    map_ax.scatter(xyz[-1,0],xyz[-1,1],color='#222222',s=35,label='Final sample',zorder=6)
    map_ax.scatter(goals[:,0],goals[:,1],marker='*',s=95,color='#18699c',label='Waypoints',zorder=6)
    for index,goal in enumerate(goals,1):
        map_ax.annotate(str(index),(goal[0],goal[1]),xytext=(5,5),textcoords='offset points',color='#18699c')
    map_ax.autoscale_view(); map_ax.margins(.05); map_ax.set_aspect('equal',adjustable='box')
    map_ax.set(xlabel='X (m)',ylabel='Y (m)',title='(a) Imported map and actual flight')
    map_ax.legend(loc='upper left',fontsize=8,framealpha=.9); map_ax.grid(alpha=.15)
    altitude_ax.plot(t,xyz[:,2],color='#176d91',lw=2,label='Actual altitude')
    target_trace = trace.get('goal',[])
    if target_trace:
        altitude_ax.step([row['t'] for row in target_trace],[row['z'] for row in target_trace],where='post',color='#c94d2e',ls='--',lw=1.4,label='Active target altitude')
    altitude_ax.set(xlabel='Simulation time (s)',ylabel='Altitude (m)',title='(b) Recorded altitude and active target')
    altitude_ax.grid(alpha=.2); altitude_ax.legend(loc='best')
    altitude_ax.set_xlim(t[0],max(t[-1],t[0]+.01))
    fig.suptitle('Gazebo flight in end.world',fontsize=14)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=180)
    plt.close(fig)
    print(json.dumps({'output':str(args.output),'odom_samples':len(od),'displayed_boxes':len(shown),
                      'total_boxes':len(boxes),'ground_display_threshold_m':args.minimum_roof_height}))


if __name__ == '__main__':
    main()
