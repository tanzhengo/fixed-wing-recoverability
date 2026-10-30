#!/usr/bin/env python3
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
plt.rcParams.update({'font.size':9,'pdf.fonttype':42})
f,ax=plt.subplots(figsize=(7.2,3.05));ax.set(xlim=(0,7.2),ylim=(0,3.05));ax.axis('off')
boxes=[(.1,2.12,'State + map + target\n203 primary rollouts','#e4e7eb'),(2.52,2.12,'Common feasibility checks\n103 terminal-frame features','#e4e7eb'),(4.94,2.12,'MLP: 103 → 64 → 32 → 2\nFeasibility + validation work','#e0e8f7'),(4.94,.93,'Learned / hybrid priority\nCandidate or work budget','#fae4d4'),(2.52,.93,'15 continuation controls\nIncremental validation','#d9eee8'),(.1,.93,'Cheapest verified primary\nExecute and observe','#d9eee8')]
for x,y,t,c in boxes:
 ax.add_patch(FancyBboxPatch((x,y),2.12,.65,boxstyle='round,pad=.04',lw=.8,fc=c,ec='#4d5966'));ax.text(x+1.06,y+.325,t,ha='center',va='center')
for a,b in [((2.27,2.445),(2.47,2.445)),((4.69,2.445),(4.89,2.445)),((6,2.07),(6,1.63)),((4.89,1.255),(4.69,1.255)),((2.47,1.255),(2.27,1.255)),((1.16,1.63),(1.16,2.07))]:
 ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=12,lw=1,color='#4d5966'))
ax.plot([.15,7.05],[.65,.65],lw=.7,color='#aab1b8')
ax.text(.18,.34,'Offline training',fontweight='bold',va='center',fontsize=8.5)
ax.text(1.65,.34,'Full continuation checks → labels (feasibility, work) → BCE + Huber loss',va='center',fontsize=8.5)
f.tight_layout(pad=.1);d=Path(__file__).parent/'figures';d.mkdir(exist_ok=True)
f.savefig(d/'architecture.pdf',bbox_inches='tight');f.savefig(d/'architecture.png',dpi=200,bbox_inches='tight')
