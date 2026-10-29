#!/usr/bin/env python3
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch,FancyArrowPatch
plt.rcParams.update({'font.size':9,'pdf.fonttype':42})
f,ax=plt.subplots(figsize=(7,2.5));ax.set(xlim=(0,7),ylim=(0,2.5));ax.axis('off')
boxes=[(.1,1.55,'State + map + target\n203 primary rollouts','#e4e7eb'),(2.45,1.55,'Common feasibility checks\nTask cost + terminal state','#e4e7eb'),(4.8,1.55,'Cost-seeded diversity\nHeading / altitude coverage','#fae4d4'),(4.8,.15,'15 continuation controls\nIncremental rejection','#d9eee8'),(2.45,.15,'Cheapest verified primary\nExplicit search status','#d9eee8'),(.1,.15,'Execute one update\nMeasure actual state','#e4e7eb')]
for x,y,t,c in boxes:
 ax.add_patch(FancyBboxPatch((x,y),2.05,.65,boxstyle='round,pad=.04',lw=.8,fc=c,ec='#4d5966'));ax.text(x+1.025,y+.325,t,ha='center',va='center')
for a,b in [((2.2,1.88),(2.4,1.88)),((4.55,1.88),(4.75,1.88)),((5.82,1.5),(5.82,.85)),((4.75,.48),(4.55,.48)),((2.4,.48),(2.2,.48)),((1.13,.85),(1.13,1.5))]:
 ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=12,lw=1,color='#4d5966'))
ax.text(6.05,1.16,'K or W\nbudget',ha='left',va='center',fontsize=8)
f.tight_layout(pad=.1);d=Path(__file__).parent/'figures';d.mkdir(exist_ok=True);f.savefig(d/'architecture.pdf',bbox_inches='tight');f.savefig(d/'architecture.png',dpi=200,bbox_inches='tight')
