from pathlib import Path
import argparse,sys
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description='Render the original paired-map and urban-query mechanisms from small records.');p.add_argument('--records-dir',type=Path,default=HERE);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--font-dir',type=Path);args=p.parse_args();OUT=args.records_dir.resolve();sys.path.insert(0,str(HERE.parent))
from pathlib import Path
import csv,json,hashlib,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch,Rectangle
from matplotlib.lines import Line2D
from portable_support import register_times
register_times(args.font_dir);plt.rcParams.update({'font.size':8,'axes.labelsize':8,'axes.titlesize':9,'xtick.labelsize':7,'ytick.labelsize':7,'pdf.fonttype':42,'axes.linewidth':.65})
FIG=args.output_dir.resolve();FIG.mkdir(parents=True,exist_ok=True)
BLUE='#246887';GREEN='#3a815d';RED='#b95d51';GRAY='#737c85'
def read(p):return list(csv.DictReader(p.open()))
def save(fig,name):
 for ext in ['pdf','png']:fig.savefig(FIG/(name+'.'+ext),dpi=260,bbox_inches='tight',pad_inches=.035)
 plt.close(fig)
def pair_figure():
 w=OUT/'validation_pair';paths=read(w/'paths.csv');checks=read(w/'candidate_checks.csv');features=json.loads((OUT/'parity.json').read_text())
 fig=plt.figure(figsize=(7.2,3.65));gs=fig.add_gridspec(2,2,height_ratios=[2.3,1],left=.085,right=.98,top=.87,bottom=.145,hspace=.67,wspace=.34)
 for scene,title in [(0,'(a) Map A: continuation outcomes'),(1,'(b) Map B: same state and primary')]:
  ax=fig.add_subplot(gs[0,scene]);name=['val_culdesac_0_m0','val_staggered_0_m0'][scene]
  cells=read(w/(name+'_slice.csv'));grid=np.zeros((66,59))
  # A true z=20m occupancy slice; FREE=1, OCCUPIED=2, absent=UNKNOWN.
  for c in cells:
   x,y,z=float(c['x']),float(c['y']),float(c['z']);ix=int((x+1)/2);iy=int((y+65)/2)
   if z==21 and 0<=ix<59 and 0<=iy<66:grid[iy,ix]=int(c['occupancy'])
  ax.imshow(grid,origin='lower',extent=(-2,116,-66,66),cmap=ListedColormap(['#d9dce0','#f6faf8','#606c79']),vmin=0,vmax=2,interpolation='nearest',aspect='equal')
  candidates=[69,107] if scene==0 else [69]
  for cid in candidates:
   col=BLUE if cid==69 else GREEN
   q=[p for p in paths if int(p['scene'])==scene and int(p['candidate'])==cid and int(p['successor'])==-1]
   ax.plot([float(p['x']) for p in q],[float(p['y']) for p in q],c=col,lw=1.6,zorder=5)
   ax.scatter(float(q[-1]['x']),float(q[-1]['y']),s=20,facecolor='white',edgecolor=col,zorder=8,lw=1)
   sc=[a for a in checks if int(a['scene'])==scene and int(a['candidate'])==cid]
   first_success=next((int(a['successor']) for a in sc if int(a['verified'])),None)
   for sid in range(15):
    if cid==107 and sid!=first_success:continue
    q=[p for p in paths if int(p['scene'])==scene and int(p['candidate'])==cid and int(p['successor'])==sid]
    valid=int(sc[sid]['verified']);c=GREEN if valid else RED
    ax.plot([float(p['x']) for p in q],[float(p['y']) for p in q],c=c,lw=1.25 if valid else .65,ls='--',alpha=1 if valid else .62,zorder=4)
    if not valid:ax.plot(float(q[-1]['x']),float(q[-1]['y']),marker='x',c=RED,ms=3,mew=.65,zorder=6)
   primary=[p for p in paths if int(p['scene'])==scene and int(p['candidate'])==cid and int(p['successor'])==-1]
   endpoint=primary[-1];xx,yy=float(endpoint['x']),float(endpoint['y'])
   ax.annotate('P'+str(cid),(xx,yy),xytext=(-9,-15 if cid==69 else 9),textcoords='offset points',fontsize=7.5,color=col,fontweight='bold')
  start=next(p for p in paths if int(p['scene'])==scene and int(p['candidate'])==69 and int(p['successor'])==-1)
  ax.plot(float(start['x']),float(start['y']),'s',ms=4,color='#252a30',zorder=9)
  ax.set_xlim(0,112);ax.set_ylim(-58,66);ax.set_xticks([0,25,50,75,100]);ax.set_yticks([-50,-25,0,25,50]);ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_title(title,pad=7)
  ax.text(.02,.97,'t = 0.8 s;  five lidar scans',transform=ax.transAxes,va='top',fontsize=7,color='#333333',bbox={'facecolor':'white','alpha':.85,'edgecolor':'none','pad':2})
 ax=fig.add_subplot(gs[1,0]);pairs=[(0,69,'Map A · P69'),(0,107,'Map A · P107'),(1,69,'Map B · P69')]
 bank=np.array([[int(a['verified']) for a in checks if int(a['scene'])==scene and int(a['candidate'])==cid] for scene,cid,_ in pairs])
 ax.imshow(bank,origin='upper',cmap=ListedColormap(['#f0d4cf','#5d9c79']),vmin=0,vmax=1,aspect='auto',extent=(-.5,14.5,2.5,-.5))
 for i in range(3):
  for j in range(15):ax.text(j,i,'1' if bank[i,j] else '0',ha='center',va='center',fontsize=8,color='white' if bank[i,j] else '#843f38')
 ax.set_yticks(range(3));ax.set_yticklabels([x[2] for x in pairs]);ax.set_xticks(range(15));ax.set_xticklabels(range(1,16),fontsize=6.7);ax.set_xlabel('Successor control index');ax.set_title('(c) Complete 15-control validation',pad=8)
 ax=fig.add_subplot(gs[1,1]);obs=np.array([features[name]['observation30'] for name in ['val_culdesac_0_m0','val_staggered_0_m0']]).reshape(2,15,2)
 for i,col,mark in [(0,BLUE,'o'),(1,GREEN,'s')]:ax.plot(np.arange(1,16),obs[i,:,0],color=col,marker=mark,ms=3,lw=1,label='Map '+('A' if i==0 else 'B'))
 for x in [3.5,6.5,9.5,12.5]:ax.axvline(x,color='#ddd',lw=.5,zorder=0)
 ax.set_xticks([2,5,8,11,14]);ax.set_xticklabels(['−45','−22.5','0','22.5','45']);ax.set_ylim(-.05,1.05);ax.set_yticks([0,.5,1]);ax.set_ylabel('Free fraction');ax.set_xlabel('Probe azimuth (deg); elevation −20 / 0 / 20');ax.set_title('(d) Observation features of P69',pad=8);ax.legend(loc='lower left',frameon=False,fontsize=7,ncol=2)
 handles=[Patch(facecolor='#f6faf8',edgecolor='#b5c4ba',label='Observed free'),Patch(facecolor='#d9dce0',label='Unknown'),Patch(facecolor='#606c79',label='Occupied'),Line2D([0],[0],c=BLUE,lw=1.6,label='Primary P69'),Line2D([0],[0],c=GREEN,ls='--',lw=1.3,label='Valid successor'),Line2D([0],[0],c=RED,ls='--',lw=1,label='Rejected successor')]
 fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.52,1.01),ncol=3,fontsize=7,frameon=False,columnspacing=1.8)
 save(fig,'observed_continuation_mechanism')

def blocks():
 out={};cur=None
 for line in (OUT/'urban_trace/trace.log').read_text().splitlines():
  z=line.split()
  if not z:continue
  if z[0]=='TRACE_BEGIN':cur={'name':z[2],'mode':None,'schedule':None,'candidates':{},'queries':{},'prunes':{},'events':[]};out[z[2]]=cur
  elif cur is not None:
   if z[0]=='TRACE_MODE':cur['mode']=z[1:]
   elif z[0]=='TRACE_CAND':cur['candidates'][int(z[1])]=list(map(float,z[2:]))
   elif z[0]=='TRACE_SCHEDULE':cur['schedule']=list(map(int,z[1:]))
   elif z[0]=='TRACE_QUERY':cur['queries'][int(z[1])]={'cost':float(z[2]),'valid':int(z[3]),'steps':int(z[4])};cur['events'].append(['query',int(z[1])])
   elif z[0]=='TRACE_PRUNE':cur['prunes'][int(z[1])]={'cost':float(z[2]),'incumbent':float(z[3])};cur['events'].append(['prune',int(z[1])])
   elif z[0]=='TRACE_END':cur['signature']=z[3];cur=None
 return out

def query_figure():
 b=blocks();assert b['Selected']['schedule']==b['Diverse']['schedule'];assert b['Selected']['queries']==b['Diverse']['queries'];assert b['Selected']['prunes']==b['Diverse']['prunes'];assert b['Cost']['queries']==b['Full']['queries']
 rows=read(OUT/'urban_trace/query_results_mechanism.csv');results={r['method']:r for r in rows};parsed={}
 for name in ['Selected','Diverse','ObsAux','Cost','Full']:
  a=b[name];schedule=a['schedule'] if a['schedule'] is not None else list(range(12));cumulative=0;series=[0];events=[]
  for k,cid in enumerate(schedule[:12],1):
   if cid in a['queries']:q=a['queries'][cid];state='witness' if q['valid'] else 'rejected';work=q['steps'];cumulative+=work;cost=q['cost']
   elif cid in a['prunes']:state='pruned';work=0;cost=a['prunes'][cid]['cost']
   else:state='unvisited';work=0;cost=None
   series.append(cumulative);events.append({'slot':k,'cost_rank':cid+1,'status':state,'steps':work,'cumulative_steps':cumulative,'cost':cost})
  assert cumulative==int(results[name]['backup_steps'])
  parsed[name]={'events':events,'cumulative_steps':series,'result':results[name]}
 (FIG/'parsed_trace.json').write_text(json.dumps(parsed,indent=2)+'\n')
 fig=plt.figure(figsize=(7.2,3.65));gs=fig.add_gridspec(2,1,height_ratios=[1,1.15],left=.155,right=.945,top=.84,bottom=.14,hspace=.62)
 ax=fig.add_subplot(gs[0]);spec=[('Selected','Selected / Diverse',BLUE),('ObsAux','Obs+Aux',GREEN),('Cost','Cost / Full',GRAY)];statuscolors={'witness':'#75a48b','rejected':'#e2b3aa','pruned':'#dbe2e8','unvisited':'#fafafa'}
 for i,(name,label,col) in enumerate(spec):
  for event in parsed[name]['events']:
   slot=event['slot'];status=event['status'];rect=Rectangle((slot-.45,i-.36),.9,.72,facecolor=statuscolors[status],edgecolor='#969da4' if status=='unvisited' else 'white',lw=.5,hatch='///' if status=='pruned' else None);ax.add_patch(rect)
   ax.text(slot,i,str(event['cost_rank']) if status!='unvisited' else '–',ha='center',va='center',fontsize=7.5,color='#333')
 ax.set_xlim(.45,12.55);ax.set_ylim(2.55,-.55);ax.set_yticks(range(3));ax.set_yticklabels([r[1] for r in spec]);ax.set_xticks(range(1,13));ax.set_xlabel('Position in the fixed candidate schedule');ax.set_title('(a) Actual query order and incumbent-cost pruning',pad=8)
 for spine in ax.spines.values():spine.set_visible(False)
 ax.tick_params(length=0)
 ax=fig.add_subplot(gs[1]);
 for name,label,col in spec:
  y=parsed[name]['cumulative_steps'];ax.step(range(13),y,where='post',label=label,color=col,lw=1.4);ax.plot(range(13),y,'o',color=col,ms=2.7)
  ax.text(12.12,y[-1],format(y[-1],','),color=col,va='center',fontsize=8)
 ax.set_xlim(0,13);ax.set_xticks(range(13));ax.set_ylim(0,8000);ax.set_yticks([0,2000,4000,6000,8000]);ax.set_yticklabels(['0','2','4','6','8']);ax.set_ylabel('Cumulative successor\nsteps (10³)');ax.set_xlabel('Position in the fixed candidate schedule');ax.set_title('(b) Measured validation work on the same urban input',pad=8);ax.grid(axis='y',lw=.4,color='#ddd');ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
 handles=[Patch(facecolor=statuscolors['rejected'],label='Rejected bank'),Patch(facecolor=statuscolors['witness'],label='Witness'),Patch(facecolor=statuscolors['pruned'],hatch='///',edgecolor='#969da4',label='Cost-pruned'),Patch(facecolor=statuscolors['unvisited'],edgecolor='#969da4',label='Not visited')]
 fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.53,.99),ncol=4,fontsize=7.5,frameon=False,columnspacing=1.8)
 save(fig,'urban_query_trace')
 return parsed
pair_figure();query_figure()
print(FIG)
