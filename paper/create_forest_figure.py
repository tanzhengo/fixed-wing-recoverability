#!/usr/bin/env python3
"""Original Fig-8-style forest rendering from logged geometry and execution.

Surface points are deterministically sampled from the actual analytic collision
volumes. Orthographic projection changes only the viewing coordinates. Stored
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


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def project(points):
    return np.asarray(points) @ np.array([RIGHT,UP]).T


def sample_surface(row):
    x,y,z,r,h = [float(row[k]) for k in ['x','y','z','r','height']]
    if row['type'] == 'cylinder':
        theta,height = np.meshgrid(np.linspace(0,2*np.pi,32,endpoint=False),np.linspace(-h/2,h/2,28))
        return np.c_[x+r*np.cos(theta.ravel()), y+r*np.sin(theta.ravel()), z+height.ravel()]
    assert row['type'] == 'sphere'
    # Equal-area Fibonacci sphere: fixed samples, independent of method/outcome.
    n=650; k=np.arange(n); q=1-2*(k+.5)/n; angle=k*np.pi*(3-np.sqrt(5)); radius=np.sqrt(1-q*q)
    return np.c_[x+r*radius*np.cos(angle),y+r*radius*np.sin(angle),z+r*q]


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
    cmap=LinearSegmentedColormap.from_list('forest_height',['#d5e5e6','#b3cfd0','#8baebd','#889bb7','#a997b9'])
    norm=Normalize(0,45)
    display_geometry={scene:np.concatenate([sample_surface(row) for row in geometry if row['scene']==scene])
                      for scene in ['forest_sparse_0','forest_dense_0']}
    shared_bounds=project(np.vstack(list(display_geometry.values()) + list(points.values()) +
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
        # The density denominator covers this explicit tree-placement rectangle.
        placement=project([[40,-80,0],[240,-80,0],[240,80,0],[40,80,0],[40,-80,0]])
        ax.plot(placement[:,0],placement[:,1],color='#9eaeb4',ls=(0,(3,3)),lw=.55,zorder=1)
        surface=display_geometry[scene]
        order=np.argsort(surface@DEPTH)
        surface=surface[order]; xy=project(surface)
        ax.scatter(xy[:,0],xy[:,1],c=surface[:,2],cmap=cmap,norm=norm,s=.65,
                   alpha=.8,linewidths=0,rasterized=True,zorder=2)
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
            'geometry':'deterministic points on actual uninflated cylinders and spheres',
            'trajectory_processing':'projection of recorded vertices only; no smoothing, displacement or extension',
            'displayed_cases':evidence,'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
            'renderer_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (DATA/'figure_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Rendered 10 original recorded trajectories; outputs: forest_comparison.pdf/png')


if __name__=='__main__':main()
