#!/usr/bin/env python3
"""Plot recorded lidar voxel snapshots; never reads prior world geometry."""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import colors, cm
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-cubes',type=int,default=12000)
    parser.add_argument('--ground-threshold',type=float,default=2.)
    args=parser.parse_args()
    if args.max_cubes<1:parser.error('--max-cubes must be positive')
    data=np.load(args.input)
    points=np.asarray(data['map'],dtype=float).reshape((-1,3))
    points=points[np.isfinite(points).all(1)]
    if not len(points):raise ValueError('Snapshot contains no occupied voxels')
    resolution=float(data['resolution'][0]);odom=data['odometry']
    elevated=points[points[:,2]>args.ground_threshold]
    if not len(elevated):raise ValueError('No elevated returns available for 3D plot')
    # Deterministic thinning only for the rendered 3D view; do not interpolate
    # cells or change the saved map. The plan view retains every recorded cell.
    stride=max(1,int(np.ceil(len(elevated)/args.max_cubes)))
    shown=elevated[::stride]
    norm=colors.Normalize(vmin=min(0.,points[:,2].min()),vmax=max(1.,points[:,2].max()))
    cmap=cm.get_cmap('viridis')
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig=plt.figure(figsize=(13,6.3))
    top=fig.add_subplot(1,2,1)
    top.scatter(points[:,0],points[:,1],c=points[:,2],norm=norm,cmap=cmap,s=2,marker='s',linewidths=0,rasterized=True)
    if len(odom):
        top.plot(odom[:,1],odom[:,2],color='#e2482e',lw=1.4,label='Recorded flight')
        top.scatter(odom[0,1],odom[0,2],color='#22a25a',s=28,label='Start',zorder=4)
        top.scatter(odom[-1,1],odom[-1,2],color='#222222',s=25,label='Latest aircraft position',zorder=4)
        top.legend(loc='upper left',fontsize=8,framealpha=.9)
    top.set(xlabel='X (m)',ylabel='Y (m)',title='(a) Accumulated observed voxels — plan view')
    top.set_aspect('equal',adjustable='box');top.grid(alpha=.12)
    ax=fig.add_subplot(1,2,2,projection='3d')
    half=resolution/2
    corners=np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])*half
    vertices=shown[:,None,:]+corners[None,:,:]
    faces=np.array([[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]])
    polygons=vertices[:,faces,:].reshape((-1,4,3))
    rgba=cmap(norm(shown[:,2]));face_colors=np.repeat(rgba,6,axis=0)
    collection=Poly3DCollection(polygons,facecolors=face_colors,edgecolors='none',linewidths=0)
    ax.add_collection3d(collection)
    low=shown.min(0)-resolution;high=shown.max(0)+resolution
    ax.set_xlim(low[0],high[0]);ax.set_ylim(low[1],high[1]);ax.set_zlim(min(0.,low[2]),high[2])
    ax.set(xlabel='X (m)',ylabel='Y (m)',zlabel='Height (m)',title='(b) Occupied voxel surfaces')
    if hasattr(ax,'set_box_aspect'):ax.set_box_aspect([high[0]-low[0],high[1]-low[1],max(60,high[2]-low[2])])
    ax.view_init(elev=26,azim=-64)
    mapper=cm.ScalarMappable(norm=norm,cmap=cmap);mapper.set_array([])
    cax=fig.add_axes([.92,.21,.015,.57]);fig.colorbar(mapper,cax=cax,label='Observed height (m)')
    fig.suptitle('3D lidar map from Gazebo scans — %.1f m voxels'%resolution,fontsize=14)
    fig.text(.5,.035,'All %s observed cells in (a); %s / %s cells above %.1f m rendered in (b).'%
             (format(len(points),','),format(len(shown),','),format(len(elevated),','),args.ground_threshold),ha='center',fontsize=9)
    fig.subplots_adjust(left=.07,right=.90,bottom=.13,top=.87,wspace=.18)
    args.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(args.output,dpi=170);plt.close(fig)
    print(json.dumps({'input':str(args.input),'output':str(args.output),'voxel_count':len(points),'rendered_3d_cells':len(shown),
                      'eligible_above_ground_cells':len(elevated),'resolution':resolution,'render_stride':stride}))


if __name__=='__main__':main()
