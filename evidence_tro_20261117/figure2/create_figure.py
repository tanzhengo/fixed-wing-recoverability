#!/usr/bin/env python3
"""Plot verified original-validation pairs; no synthetic trajectories or labels."""
from pathlib import Path
import argparse,csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch,Rectangle
from matplotlib.lines import Line2D
OUT=Path(__file__).resolve().parent
BLUE='#246887';GREEN='#26774f';RED='#b45a51';FREE='#f4f8f4';UNKNOWN='#d9dce0';OCCUPIED='#576572'
FAMILIES={'culdesac':'Cul-de-sac','staggered':'Staggered','wall':'Wall','spatial':'Spatial','forest_sparse':'Sparse forest','forest_dense':'Dense forest'}
def read(p):
 with p.open() as f:return list(csv.DictReader(f))
def main():
 global OUT
 ap=argparse.ArgumentParser();ap.add_argument('--records-dir',type=Path,default=OUT);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--font-dir',type=Path);args=ap.parse_args();HERE=OUT;OUT=args.records_dir.resolve();DEST=args.output_dir.resolve();DEST.mkdir(parents=True,exist_ok=True)
 import sys
 sys.path.insert(0,str(HERE.parent))
 from portable_support import register_times
 register_times(args.font_dir)
 plt.rcParams.update({'font.family':'Times New Roman','font.size':7.6,'axes.labelsize':7.6,'axes.titlesize':8.0,'xtick.labelsize':7.3,'ytick.labelsize':7.3,'legend.fontsize':7.4,'pdf.fonttype':42,'ps.fonttype':42,'axes.linewidth':.55})
 report=json.loads((OUT/'pair_evidence.json').read_text());paths=read(OUT/'raw/paths.csv');checks=read(OUT/'raw/candidate_checks.csv')
 def plotmap(ax,member,limits,detail=False):
  xlo,xhi,ylo,yhi=limits
  # Pixel centers are odd metres; cells are exactly two metres wide.
  xedges=np.arange(-10,148,2);yedges=np.arange(-86,88,2);grid=np.zeros((len(yedges)-1,len(xedges)-1))
  for c in read(OUT/'raw'/(member['case']+'_slice.csv')):
   x,y,z=float(c['x']),float(c['y']),float(c['z'])
   if z!=21:continue
   ix=int(round((x-xedges[0]-1)/2));iy=int(round((y-yedges[0]-1)/2))
   if 0<=iy<grid.shape[0] and 0<=ix<grid.shape[1]:grid[iy,ix]=int(c['occupancy'])
  ax.pcolormesh(xedges,yedges,grid,cmap=ListedColormap([UNKNOWN,FREE,OCCUPIED]),vmin=0,vmax=2,shading='flat',linewidth=0,rasterized=False,zorder=0)
  sc=member['scene_index'];cid=member['candidate_id'];primary=[p for p in paths if int(p['scene'])==sc and int(p['candidate'])==cid and int(p['successor'])==-1]
  for sid in range(15):
   q=[p for p in paths if int(p['scene'])==sc and int(p['candidate'])==cid and int(p['successor'])==sid];valid=member['successor_outcomes'][sid]['verified'];color=GREEN if valid else RED
   ax.plot([float(p['x']) for p in q],[float(p['y']) for p in q],color=color,lw=1.15 if valid else .58,ls='--',alpha=1 if valid else .73,zorder=4 if valid else 3)
   if not valid:ax.plot(float(q[-1]['x']),float(q[-1]['y']),marker='x',color=RED,ms=2.8,mew=.65,zorder=5)
  ax.plot([float(p['x']) for p in primary],[float(p['y']) for p in primary],color=BLUE,lw=1.45,zorder=6)
  terminal=primary[-1];ax.plot(float(terminal['x']),float(terminal['y']),marker='o',ms=3.6,mfc='white',mec=BLUE,mew=.85,zorder=7)
  ax.plot(float(primary[0]['x']),float(primary[0]['y']),marker='s',ms=3.3,color='#222a30',zorder=7)
  if cid==246 and member['label']==1:
   valid_sid=member['verified_controls_1based'][0]-1
   endpoint=next(p for p in reversed(paths) if int(p['scene'])==sc and int(p['candidate'])==cid and int(p['successor'])==valid_sid)
   ax.annotate('End altitude: %.1f m'%float(endpoint['h']),(float(endpoint['x']),float(endpoint['y'])),xytext=(54,-22),textcoords='data',fontsize=7.2 if not detail else 9,color=GREEN,arrowprops={'arrowstyle':'-','color':GREEN,'lw':.5},zorder=9)
  ax.annotate('P'+str(cid),(float(terminal['x']),float(terminal['y'])),xytext=(-12,9 if cid==69 else 10),textcoords='offset points',color=BLUE,fontsize=7.5 if not detail else 9.5,fontweight='bold',zorder=9)
  ax.text(.025,.96,'seed '+str(member['seed']),transform=ax.transAxes,va='top',fontsize=7.2 if not detail else 9,color='#333333',bbox={'facecolor':'white','alpha':.9,'edgecolor':'none','pad':1})
  ax.set_xlim(xlo,xhi);ax.set_ylim(ylo,yhi);ax.set_aspect('equal',adjustable='box');ax.tick_params(axis='both',length=2,pad=1.5)
  ax.set_xlabel('X (m)',labelpad=1.2);ax.set_ylabel('Y (m)',labelpad=1.2)
  return ax
 handles=[Patch(facecolor=FREE,edgecolor='#afb7b0',label='Observed free'),Patch(facecolor=UNKNOWN,label='Unknown'),Patch(facecolor=OCCUPIED,label='Occupied'),Line2D([0],[0],color=BLUE,lw=1.45,label='Primary'),Line2D([0],[0],color=GREEN,ls='--',lw=1.1,label='Valid successor'),Line2D([0],[0],color=RED,ls='--',lw=.8,marker='x',ms=3,label='Rejected successor')]
 fig=plt.figure(figsize=(7.2,5.25));gs=fig.add_gridspec(3,3,left=.065,right=.99,bottom=.095,top=.895,wspace=.26,hspace=.48,width_ratios=[1,1,.65])
 for row,g in enumerate(report['groups']):
  map_axes=[]
  limits=(0,124,-55,12) if row<2 else (0,124,-34,33)
  for col,m in enumerate(g['members']):
   ax=fig.add_subplot(gs[row,col]);plotmap(ax,m,limits);ax.set_xticks([0,40,80,120]);ax.set_yticks([-50,-25,0] if row<2 else [-25,0,25]);letter=chr(97+row*3+col);side='A' if col==0 else 'B'
   ax.set_title('('+letter+') '+side+': '+FAMILIES[m['family']],pad=4,loc='left');map_axes.append(ax)
  ax=fig.add_subplot(gs[row,2]);pmap=map_axes[0].get_position();pquant=ax.get_position();ax.set_position([pquant.x0,pmap.y0,pquant.width,pmap.height]);ax.set_xlim(0,1);ax.set_ylim(0,1);ax.axis('off');letter=chr(99+row*3);cid=g['members'][0]['candidate_id'];ax.text(0,1.05,'('+letter+') P'+str(cid)+': 15 controls',fontsize=8,va='bottom')
  x0=.08;dx=.92/15;ybase=[.73,.54];dy=.16
  for j in [0,4,9,14]:ax.text(x0+(j+.5)*dx,.96,str(j+1),ha='center',va='center',fontsize=7.2)
  for side,m in enumerate(g['members']):
   ax.text(.01,ybase[side]+dy/2,'AB'[side],ha='left',va='center',fontsize=7.4)
   for j,q in enumerate(m['successor_outcomes']):
    valid=q['verified'];ax.add_patch(Rectangle((x0+j*dx,ybase[side]),dx,dy,facecolor='#4c926a' if valid else '#f0d4cf',edgecolor='white',linewidth=.45));ax.text(x0+(j+.5)*dx,ybase[side]+dy/2,str(valid),ha='center',va='center',fontsize=7.2,color='white' if valid else '#8a403b')
  a,b=g['members'];ax.text(0,.405,'Witness A / B:  '+str(a['label'])+' / '+str(b['label']),fontsize=7.5,va='center')
  ax.text(0,.245,'Work A / B:  '+str(a['work_until_first_witness'])+' / '+str(b['work_until_first_witness']),fontsize=7.5,va='center')
  ax.text(0,.085,'Obs. changed:  '+str(g['observation_changed_components'])+' / 30',fontsize=7.5,va='center')
 fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.51,1.005),ncol=3,frameon=False,columnspacing=1.9,handlelength=2.0,labelspacing=.35)
 fig.text(.5,.015,'Observed slices: z in [20, 22) m. Curves are horizontal projections; all outcomes use 3D validation.',ha='center',fontsize=7.2)
 for ext in ['pdf','png']:fig.savefig(DEST/('observed_continuation_mechanism.'+ext),dpi=320)
 plt.close(fig)
 # Full-map companion retains the complete exported spatial window at a larger readable scale.
 fig,axes=plt.subplots(3,2,figsize=(8.2,11.0));fig.subplots_adjust(left=.075,right=.985,bottom=.05,top=.92,hspace=.30,wspace=.18)
 for row,g in enumerate(report['groups']):
  for col,m in enumerate(g['members']):
   ax=axes[row,col];plotmap(ax,m,(0,144,-84,84),True);ax.set_xticks([0,40,80,120]);ax.set_yticks([-80,-40,0,40,80]);ax.tick_params(labelsize=9);ax.set_title('Pair '+str(row+1)+' / '+'AB'[col]+': '+FAMILIES[m['family']]+' (P'+str(m['candidate_id'])+')',fontsize=10,pad=6)
   ax.text(.025,.035,'Valid controls: '+str(len(m['verified_controls_1based']))+'/15; witness work: '+str(m['work_until_first_witness']),transform=ax.transAxes,fontsize=9,bbox={'facecolor':'white','alpha':.88,'edgecolor':'none','pad':2})
 fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.99),ncol=3,fontsize=9,frameon=False)
 for ext in ['pdf','png']:fig.savefig(DEST/('observed_continuation_full_maps.'+ext),dpi=260)
 plt.close(fig)
 (DEST/'figure_layout.json').write_text(json.dumps({'main_inches':[7.2,5.25],'font':'Times New Roman','minimum_font_pt':7.2,'map_geometry':'equal aspect, 2m vector cells at z in [20,22)','curves':'unmodified 3D C++ rollouts projected into xy','main_crop_windows':[[0,124,-55,12],[0,124,-55,12],[0,124,-34,33]],'full_map_companion':True},indent=2)+'\n')
 print('Saved main and full-map PDF/PNG figures in',DEST)
if __name__=='__main__':main()
