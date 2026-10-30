#!/usr/bin/env python3
"""Voxel-map visualization of logged forest geometry and execution.

Occupied cells have positive-volume overlap with the analytic collision volumes.
Voxelization is for display; the experiment retains its analytic collision model.
Orthographic projection changes only the viewing coordinates. Stored
trajectory vertices are neither smoothed, resampled, displaced nor extended.
"""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.collections import PolyCollection, LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
import numpy as np

ROOT = Path(__file__).resolve().parent
DATA = ROOT.parent / 'docs/recovery/forest_figure'
FIGURES = ROOT / 'figures'
METHODS = ['cost12', 'diverse12', 'learned12', 'state_only12', 'full203']
LABELS = {'cost12':'Cost', 'diverse12':'Diverse', 'learned12':'Learned',
          'state_only12':'No geometry', 'full203':'Full'}
COLORS = {'cost12':'#9467bd', 'diverse12':'#d98b34', 'learned12':'#167c76',
          'state_only12':'#bf4b4b', 'full203':'#42699a'}
STYLES = {'cost12':(0,(1.4,1.5)), 'diverse12':(0,(5,2)),
          'learned12':'-', 'state_only12':(2,(4,2,1,2)), 'full203':(0,(9,3))}
WIDTHS = {'full203':2.8,'cost12':2.3,'diverse12':2.0,'learned12':1.6,'state_only12':1.5}
AZIMUTH, ELEVATION = np.deg2rad([-80., 34.])
RIGHT = np.array([-np.sin(AZIMUTH),np.cos(AZIMUTH),0.])
UP = np.array([-np.sin(ELEVATION)*np.cos(AZIMUTH),-np.sin(ELEVATION)*np.sin(AZIMUTH),np.cos(ELEVATION)])
DEPTH = np.cross(RIGHT,UP)
VOXEL_SIZE = 1.5
GRID_ORIGIN = np.zeros(3)


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def project(points):
    return np.asarray(points) @ np.array([RIGHT,UP]).T


def voxelize(objects):
    """Union of globally aligned cells intersecting obstacle interiors."""
    occupied=set()
    for row in objects:
        center=np.array([float(row[k]) for k in ['x','y','z']])
        r,h=float(row['r']),float(row['height'])
        assert row['type'] in ['sphere','cylinder']
        extent=np.array([r,r,r if row['type']=='sphere' else h/2])
        lo=np.floor((center-extent-GRID_ORIGIN)/VOXEL_SIZE).astype(int)
        hi=np.ceil((center+extent-GRID_ORIGIN)/VOXEL_SIZE).astype(int)
        cells=np.stack(np.meshgrid(*[np.arange(lo[a],hi[a]) for a in range(3)],indexing='ij'),axis=-1).reshape(-1,3)
        lower=GRID_ORIGIN+cells*VOXEL_SIZE;upper=lower+VOXEL_SIZE
        gap=np.maximum(np.maximum(lower-center,center-upper),0)
        if row['type']=='sphere':
            intersects=np.sum(gap*gap,axis=1)<r*r
        else:
            intersects=(np.sum(gap[:,:2]**2,axis=1)<r*r)&(lower[:,2]<center[2]+h/2)&(upper[:,2]>center[2]-h/2)
        occupied.update(map(tuple,cells[intersects]))
    return np.array(sorted(occupied),dtype=np.int64)


def visible_voxel_faces(cells):
    """Exterior cube faces toward the camera, sorted from far to near."""
    occupied=set(map(tuple,cells));faces=[];shades=[]
    for axis in range(3):
        sign=1 if DEPTH[axis]>0 else -1
        normal=np.zeros(3,dtype=int);normal[axis]=sign
        corners=np.zeros((4,3));corners[:,axis]=(sign+1)/2
        a,b=[i for i in range(3) if i!=axis]
        corners[:,a]=[0,1,1,0];corners[:,b]=[0,0,1,1]
        for cell in cells:
            if tuple(cell+normal) in occupied:continue
            faces.append(GRID_ORIGIN+(cell+corners)*VOXEL_SIZE)
            shades.append([.85,.72,1.][axis])
    faces=np.array(faces);order=np.argsort(faces.mean(axis=1)@DEPTH,kind='stable')
    return faces[order],faces[order,:,2].mean(axis=1),np.array(shades)[order]


def main():
    manifest=json.loads((DATA/'run_manifest.json').read_text())
    assert manifest['status']=='complete', 'Only complete experiments may become paper figures'
    geometry=read_csv(DATA/'forest_scenarios.csv')
    metadata=read_csv(DATA/'forest_map_metadata.csv')
    episodes=read_csv(DATA/'forest_episodes.csv')
    paths=read_csv(DATA/'forest_paths.csv')
    assert len(episodes)==80
    chosen=[r for r in episodes if r['scene'] in ['forest_sparse_0','forest_dense_0']]
    assert len(chosen)==10 and {r['method'] for r in chosen}==set(METHODS)
    points={r['case_id']:[] for r in chosen}
    for row in paths:
        if row['case_id'] in points:
            points[row['case_id']].append([float(row[k]) for k in ['x','y','z']])
    plt.rcParams.update({'font.size':9,'pdf.fonttype':42,'savefig.facecolor':'white'})
    cmap=LinearSegmentedColormap.from_list('forest_height',['#d4e9e4','#a4c9d1','#87afca','#9d9bc9','#b19aca'])
    norm=Normalize(0,45)
    display_geometry={scene:voxelize([row for row in geometry if row['scene']==scene])
                      for scene in ['forest_sparse_0','forest_dense_0']}
    meshes={scene:visible_voxel_faces(cells) for scene,cells in display_geometry.items()}
    shared_bounds=project(np.vstack([mesh[0].reshape(-1,3) for mesh in meshes.values()] + list(points.values()) +
                                   [[[0,-88,0],[265,-88,0],[265,88,0],[0,88,0]]]))
    limits=(shared_bounds.min(axis=0)-[12,14],shared_bounds.max(axis=0)+[18,7])
    fig=plt.figure(figsize=(7.2,6.0))
    axes=[fig.add_axes([.025,.535,.88,.385]),fig.add_axes([.025,.08,.88,.385])]
    evidence=[]
    for index,(ax,density,trees) in enumerate(zip(axes,['sparse','dense'],[40,80])):
        scene='forest_'+density+'_0'
        info=next(r for r in metadata if r['scene']==scene)
        assert int(info['seed'])==601000 and int(info['tree_count'])==trees
        objects=[r for r in geometry if r['scene']==scene]
        assert len(objects)==2*trees+12
        ground=np.array([[0,-88,0],[265,-88,0],[265,88,0],[0,88,0]])
        ax.add_patch(Polygon(project(ground),fc='#f0f5f5',ec='#cbd9dc',lw=.65,zorder=0))
        ground_lines=[project([[x,-88,0],[x,88,0]]) for x in range(0,266,20)]
        ground_lines += [project([[0,y,0],[265,y,0]]) for y in range(-80,81,20)]
        ax.add_collection(LineCollection(ground_lines,colors='#d9e3e6',linewidths=.22,zorder=.5))
        # The density denominator covers this explicit tree-placement rectangle.
        placement=project([[40,-80,0],[240,-80,0],[240,80,0],[40,80,0],[40,-80,0]])
        ax.plot(placement[:,0],placement[:,1],color='#9eaeb4',ls=(0,(3,3)),lw=.55,zorder=1)
        faces,heights,shades=meshes[scene]
        facecolors=cmap(norm(heights));facecolors[:,:3]*=shades[:,None]
        ax.add_collection(PolyCollection(project(faces.reshape(-1,3)).reshape(-1,4,2),
                          facecolors=facecolors,edgecolors=(.27,.36,.43,.48),
                          linewidths=.12,rasterized=True,zorder=2))
        selected={row['method']:row for row in chosen if row['scene']==scene}
        draw_order=['full203','cost12','diverse12','learned12','state_only12']
        # All white halos are drawn first, so later halos cannot erase another curve.
        for method in draw_order:
            xyz=np.array(points[selected[method]['case_id']]); assert len(xyz)>1
            curve=project(xyz)
            ax.plot(curve[:,0],curve[:,1],color='white',lw=4.0,alpha=.85,zorder=3)
        for method in draw_order:
            row=selected[method]; xyz=np.array(points[row['case_id']]); curve=project(xyz)
            ax.plot(curve[:,0],curve[:,1],color=COLORS[method],ls=STYLES[method],
                    lw=WIDTHS[method],zorder=4)
            success=row['outcome']=='goal'
            ax.plot(*curve[-1],marker='o' if success else 'x',color=COLORS[method],
                    markersize=4.4 if success else 6.0,mew=1.2,mec=COLORS[method],
                    mfc='white',zorder=5)
            evidence.append({'scene':scene,'method':method,'case_id':row['case_id'],
                             'outcome':row['outcome'],'stored_vertices':len(xyz),
                             'endpoint_xyz':xyz[-1].tolist()})
        start,goal=project([[0,0,20],[250,0,20]])
        ax.plot(*start,'ks',ms=4,zorder=6);ax.plot(*goal,'k*',ms=8,zorder=6)
        ax.annotate('Start',start,xytext=(-4,9),textcoords='offset points',ha='center',fontsize=8,fontweight='bold')
        ax.annotate('Goal',goal,xytext=(8,-10),textcoords='offset points',ha='left',fontsize=8,fontweight='bold')
        bar=project([[5,-90,0],[55,-90,0]])
        ax.plot(bar[:,0],bar[:,1],color='#384a51',lw=1.8,zorder=7)
        ax.text(*(bar.mean(axis=0)+[0,-5]),'50 m',ha='center',va='top',fontsize=7.5,color='#384a51')
        ax.set_xlim(limits[0][0],limits[1][0]);ax.set_ylim(limits[0][1],limits[1][1])
        ax.set_aspect('equal',adjustable='box');ax.axis('off')
        numerator=800 if density=='sparse' else 400
        ax.text(.5,-.035,'(%s) %s forest: %d trees, 1/%d tree/m²'%('ab'[index],density.capitalize(),trees,numerator),
                transform=ax.transAxes,ha='center',va='top',fontsize=9)
    handles=[Line2D([0],[0],color=COLORS[m],ls=STYLES[m],lw=2,label=LABELS[m]) for m in METHODS]
    fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.48,.995),ncol=5,
               frameon=False,handlelength=2.4,columnspacing=1.25,fontsize=8.5)
    fig.text(.48,.945,'Voxel size: 1.5 m',ha='center',fontsize=7.5,color='#52616d')
    cax=fig.add_axes([.925,.245,.016,.49]);mappable=plt.cm.ScalarMappable(norm=norm,cmap=cmap)
    mappable.set_array(np.array([0.,45.]));cb=fig.colorbar(mappable,cax=cax)
    cb.set_label('Height (m)',fontsize=8);cb.set_ticks([0,15,30,45]);cb.ax.tick_params(labelsize=7.5,length=2)
    fig.text(.475,.018,'○ Goal attained     × Other termination     ★ Target',ha='center',fontsize=8,color='#384a51')
    FIGURES.mkdir(exist_ok=True)
    fig.savefig(FIGURES/'forest_comparison.pdf',dpi=400,bbox_inches='tight',pad_inches=.025)
    fig.savefig(FIGURES/'forest_comparison.png',dpi=300,bbox_inches='tight',pad_inches=.025)
    files=[DATA/('forest_'+suffix+'.csv') for suffix in ['scenarios','map_metadata','episodes','paths']]
    report={'figure_seed':601000,'selection_rule':'first prespecified seed for both densities',
            'projection':'orthographic, azimuth -80 degrees, elevation 34 degrees',
            'geometry':'voxel display of uninflated analytic cylinders and spheres',
            'voxel_size_m':VOXEL_SIZE,'grid_origin_m':GRID_ORIGIN.tolist(),
            'occupancy_rule':'positive-volume cell/obstacle intersection; union over all obstacles',
            'collision_model':'original analytic geometry; voxelization used for visualization only',
            'voxel_counts':{scene:len(cells) for scene,cells in display_geometry.items()},
            'visible_face_counts':{scene:len(mesh[0]) for scene,mesh in meshes.items()},
            'occupied_indices_sha256':{scene:hashlib.sha256(cells.astype('<i8').tobytes()).hexdigest() for scene,cells in display_geometry.items()},
            'trajectory_processing':'projection of recorded vertices only; no smoothing, displacement or extension',
            'displayed_cases':evidence,'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
            'renderer_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (DATA/'figure_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Rendered 10 original recorded trajectories; outputs: forest_comparison.pdf/png')


if __name__=='__main__':main()
