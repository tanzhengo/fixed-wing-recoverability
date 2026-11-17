#!/usr/bin/env python3
"""Render the paper's illustrated architecture from bundled screenshot crops.

The figure is schematic; this script does not run or modify planning experiments.
Times New Roman font files are supplied by the user and are not redistributed.
"""
from pathlib import Path
import argparse,base64,hashlib,json,math,re,sys

parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('--font-dir',required=True,type=Path,help='Directory containing Times New Roman regular, bold and italic TTF files (times.ttf, timesbd.ttf, timesi.ttf; case-insensitive).')
parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'generated',help='Destination for architecture_revised.pdf/.svg and figure_design.json (default: ./generated beside this script).')
args=parser.parse_args()
font_dir=args.font_dir.expanduser().resolve()
if not font_dir.is_dir():parser.error('Times New Roman font directory does not exist: '+str(font_dir))
font_files={p.name.casefold():p for p in font_dir.iterdir() if p.is_file()}
required={'TNR':'times.ttf','TNRB':'timesbd.ttf','TNRI':'timesi.ttf'}
missing=[name for name in required.values() if name not in font_files]
if missing:parser.error('Missing Times New Roman font files in '+str(font_dir)+': '+', '.join(missing)+'. Supply the actual Times New Roman fonts; no substitute font is used.')
FONT_PATHS={name:font_files[file] for name,file in required.items()}
ASSETS=Path(__file__).resolve().parent/'assets'
for name in ['gazebo_suav1.png','rviz_voxels.png']:
 if not (ASSETS/name).is_file():parser.error('Bundled figure asset is missing: '+str(ASSETS/name))
try:
 from PIL import Image
 from reportlab.pdfbase import pdfmetrics
 from reportlab.pdfbase.ttfonts import TTFont
 from reportlab.graphics.shapes import Drawing,Group,Rect,Line,PolyLine,Polygon,Circle,Ellipse,String,Path as RPath,Image as RImage
 from reportlab.graphics import renderPDF,renderSVG
 from reportlab.lib.colors import HexColor,Color
 import fitz
except ImportError as exc:
 parser.error(str(exc)+'. Install the dependencies listed in requirements.txt.')
# ReportLab 3.5 needs this Pillow adapter for SVG image export.
if not hasattr(renderSVG,'_getImage'):renderSVG._getImage=lambda:Image
for name,path in FONT_PATHS.items():
 font=TTFont(name,str(path));family=font.face.familyName
 if isinstance(family,bytes):family=family.decode('utf-8','replace')
 if 'times new roman' not in family.casefold():parser.error(str(path)+' is not a Times New Roman font (family: '+family+').')
 pdfmetrics.registerFont(font)
OUT=args.output.expanduser().resolve();OUT.mkdir(parents=True,exist_ok=True)
W,H=1560,690;D=Drawing(W,H);labels=[]
C={'white':'#ffffff','ink':'#233642','blue':'#237dab','orange':'#d98524','green':'#23836c','purple':'#8953ae','line':'#a8b7bf','unknown':'#d7dce1','occupied':'#3a819c','free':'#ffffff','red':'#bd655d'}
def col(v):return HexColor(C.get(v,v))
def rect(x,y,w,h,fill='white',stroke='line',r=5,lw=1.2,dash=None):
 s=Rect(x,H-y-h,w,h,rx=r,ry=r,fillColor=col(fill) if fill else None,strokeColor=col(stroke) if stroke else None,strokeWidth=lw)
 if dash:s.strokeDashArray=dash
 D.add(s)
def text(x,y,t,size=23,font='TNR',color='ink',anchor='middle'):
 D.add(String(x,H-y,t,fontName=font,fontSize=size,fillColor=col(color),textAnchor=anchor));labels.append({'text':t,'x':x,'y':y,'font':font,'size':size})
def arrow(points,color='ink',lw=1.7,dash=None,head=7):
 xy=[(x,H-y) for x,y in points];s=PolyLine([v for p in xy for v in p],strokeColor=col(color),strokeWidth=lw,fillColor=None)
 if dash:s.strokeDashArray=dash
 D.add(s);a,b=xy[-2:];angle=math.atan2(b[1]-a[1],b[0]-a[0]);ang=.43
 tip=[b[0],b[1],b[0]-head*math.cos(angle-ang),b[1]-head*math.sin(angle-ang),b[0]-head*math.cos(angle+ang),b[1]-head*math.sin(angle+ang)]
 D.add(Polygon(tip,fillColor=col(color),strokeColor=None))
def line(points,color='ink',lw=1.3,dash=None):
 s=PolyLine([v for x,y in points for v in (x,H-y)],strokeColor=col(color),strokeWidth=lw,fillColor=None)
 if dash:s.strokeDashArray=dash
 D.add(s)
def plane(x,y,s=1,color='ink'):
 pts=[(18,0),(6,-3),(-2,-19),(-6,-19),(-3,-3),(-13,-3),(-17,-8),(-20,-8),(-17,0),(-20,8),(-17,8),(-13,3),(-3,3),(-6,19),(-2,19),(6,3)]
 D.add(Polygon([v for a,b in pts for v in (x+s*a,H-y-s*b)],fillColor=col(color),strokeColor=None))
def mathtext(x,y,parts,size=21.5,color='green'):
 widths=[pdfmetrics.stringWidth(t,'TNRI',size*(.72 if sub else 1)) for t,sub in parts];xx=x-sum(widths)/2
 for (t,sub),ww in zip(parts,widths):text(xx,y+(5 if sub else 0),t,size*(.72 if sub else 1),'TNRI',color,'start');xx+=ww
def fan(x,y,w,h,color='orange',n=7,obstacles=True):
 if obstacles:
  D.add(Polygon([x,H-y+3,x+w-10,H-y-12,x+w,H-y+h*.50,x+10,H-y+h*.75],fillColor=HexColor('#ecf4f7'),strokeColor=HexColor('#d8e4e9'),strokeWidth=1))
  for fx,fy in [(.3,-.28),(.58,-.4),(.81,-.18)]:
   xx=x+w*fx;yy=y+h*fy;D.add(Rect(xx-5,H-yy-7,10,21,fillColor=HexColor('#dbe4e8'),strokeColor=HexColor('#a9b8c0'),strokeWidth=1));D.add(Ellipse(xx,H-yy+14,5,2.2,fillColor=HexColor('#edf3f6'),strokeColor=HexColor('#a9b8c0'),strokeWidth=1))
 for i in range(n):
  v=(i-(n-1)/2)/(n-1);p=RPath();p.moveTo(x,H-y);p.curveTo(x+w*.35,H-y-v*h*.15,x+w*.7,H-y-v*h*.9,x+w,H-y-v*h)
  chosen=i==n//2;p.strokeColor=col('green' if chosen else color);p.strokeWidth=2.1 if chosen else 1.5;p.fillColor=None;D.add(p);D.add(Circle(x+w,H-y-v*h,2.3,fillColor=p.strokeColor,strokeColor=None))
 plane(x-4,y,.43)
def bank(x,y,w,h):
 for i in range(7):
  v=(i-3)/6;p=RPath();p.moveTo(x,H-y);p.curveTo(x+w*.3,H-y,x+w*.7,H-y-v*h*.7,x+w,H-y-v*h);p.strokeColor=col('green' if i in [2,3] else ('red' if i in [1,5] else '#b4c1ca'));p.strokeWidth=2 if i in [2,3] else 1.3;p.fillColor=None;D.add(p)
  if i in [1,5]:xx=x+w*.8;yy=y+v*h*.8;line([(xx-3,yy-3),(xx+3,yy+3)],'red',1.3);line([(xx-3,yy+3),(xx+3,yy-3)],'red',1.3)
 D.add(Circle(x,H-y,4.2,fillColor=col('blue'),strokeColor=None))
def network(x,y,w,h,training=False,dim_y=None):
 ns=[4,5,4,2 if training else 1];xx=[x+w*i/3 for i in range(4)];ys=[]
 for n in ns:ys.append([y+(j-(n-1)/2)*h/max(ns) for j in range(n)])
 for i in range(3):
  for a in ys[i]:
   for b in ys[i+1]:line([(xx[i],a),(xx[i+1],b)],'#d8c8e8',.7)
 for i in range(4):
  for yy in ys[i]:D.add(Circle(xx[i],H-yy,4.5,fillColor=col('#f0e6f8' if i in [0,3] else '#dbc6ee'),strokeColor=col('purple'),strokeWidth=1.2))
 for xxv,t in zip(xx,['58','64','32','2' if training else '1']):text(xxv,dim_y if dim_y is not None else y+h*.7+8,t,22,'TNRB','purple')
def grid(x,y,cell=10,rows=7,cols=11):
 for j in range(rows):
  for i in range(cols):
   free=(1<i<cols-2 and 0<j<rows-1);occ=(i in [5,6] and j in [2,3]) or (i==8 and j>3)
   fill='occupied' if occ else 'free' if free else 'unknown';rect(x+i*cell,y+j*cell,cell,cell,fill,'#eef2f4',r=0,lw=.5)
 plane(x+cell*3.1,y+cell*4.5,.26)
def imagebox(path,x,y,w,h):
 D.add(RImage(x,H-y-h,w,h,str(path)));rect(x,y,w,h,None,'#9badae',r=0,lw=1)
# Main lanes.
rect(10,10,165,632,'#f5f7f8','#98a8b0',r=9,dash=[5,3])
rect(192,10,1020,214,'#eef7fb','#98a8b0',r=9,dash=[5,3])
rect(192,242,1020,400,'#fff9ef','#98a8b0',r=9,dash=[5,3])
rect(1230,10,318,632,'#eff7f2','#98a8b0',r=9,dash=[5,3])
text(92.5,43,'Inputs',29,'TNRB')
text(210,42,'Offline training',29,'TNRB','blue','start')
text(210,274,'Online planning',29,'TNRB','orange','start')
text(1389,43,'Execution & simulation',27,'TNRB','green')
# Inputs.
rect(23,69,139,126,'white','#a1afb7');text(92.5,96,'Vehicle state',24,'TNRB');text(92.5,127,'Position p',22);text(92.5,153,'Airspeed V',22);text(92.5,179,'γ, ψ, φ, α, T',22,'TNRI')
rect(23,283,139,166,'white','#a1afb7');text(92.5,311,'Three-state',24,'TNRB','blue');text(92.5,337,'voxel map',24,'TNRB','blue');grid(38,352,10,7,11);text(92.5,440,'Lidar-built',22,'TNRI','blue')
rect(23,506,139,108,'white','#a1afb7');text(92.5,533,'Task',24,'TNRB');text(92.5,565,'Waypoint',22);text(92.5,595,'Budget K',23,'TNRI')
# Offline modules.
rect(210,64,180,139,'#e4f2fa','#4c9abf');text(300,91,'Model rollouts',24,'TNRB','blue');fan(239,144,120,54);text(300,191,'H = B = 3 s',23,'TNRI','blue')
rect(420,64,200,139,'white','#4c9abf');text(520,91,'Continuation labels',23,'TNRB','blue')
for x,c in [(470,'green'),(570,'red')]:D.add(Circle(x,H-119,13,fillColor=col('#eff8f3' if c=='green' else '#fbefed'),strokeColor=col(c),strokeWidth=1.5))
line([(463,119),(469,125),(479,112)],'green',2);line([(564,113),(576,125)],'red',2);line([(576,113),(564,125)],'red',2)
text(520,151,'C, log(1+w)',22,'TNRI','blue');text(520,175,'15-control bank',22);text(520,199,'Same map + checks',21.5)
rect(650,64,280,139,'#f5edfb','purple');text(790,91,'Supervised MLP',24,'TNRB','purple');network(690,116,197,31,True,151);text(790,174,'Selected: BCE',22,'TNRB','purple');text(790,199,'Aux work loss: ablation',21.5,'TNR','purple')
rect(960,64,230,139,'#f5edfb','purple');text(1075,91,'Compact export',24,'TNRB','purple')
for i in range(3):
 rect(1042+i*10,113+i*7,57,31,'#e5d5f1','purple',r=0,lw=1.2)
 for row in range(2):
  for cl in range(4):D.add(Circle(1054+i*10+cl*9,H-(123+i*7+row*10),1.2,fillColor=col('purple'),strokeColor=None))
text(1075,187,'58 → 64 → 32 → 1',23,'TNRI','purple')
for a,b in [(390,420),(620,650),(930,960)]:arrow([(a,134),(b,134)],'blue')
# Online top branch.
text(292.5,316,'Observed space',23,'TNRB','blue');grid(222,334,12,7,11)
for xx,fill,t in [(218,'white','Free'),(271,'occupied','Occ.'),(324,'unknown','Unk.')]:rect(xx,428,10,10,fill,'#91a1ad',r=0,lw=.7);text(xx+14,439,t,21.5,'TNR','ink','start')
text(495,316,'Primary primitives',23,'TNRB','orange');fan(426,385,138,65);text(495,436,'H = 3 s',23,'TNRI','orange')
rect(615,295,230,150,'white','orange');text(730,322,'Cheapest-first query',23,'TNRB','orange');bank(643,368,176,46);text(730,412,'15 successors; B = 3 s',21.5);text(730,438,'Counts toward K',21.5,'TNRI','orange')
arrow([(162,368),(210,368)],'blue');arrow([(356,380),(402,380)],'blue');arrow([(585,380),(615,380)],'ink')
arrow([(162,135),(183,135),(183,287),(495,287),(495,300)],'ink',1.4)
arrow([(162,560),(183,560),(183,287)],'ink',1.4)
# Lower learned path: first failure is charged to K.
arrow([(730,445),(730,469),(290,469),(290,489)],'purple');text(514,463,'No witness; slots remain',21.5,'TNRI','purple')
rect(200,489,184,132,'white','#c9a5df');text(292,515,'Terminal encoding',21.5,'TNRB','purple')
for j in range(4):
 for i in range(6):rect(247+i*14,533+j*12,10,9,['#e4d3f0','#c9a4e1','#aa78cd'][(i+2*j)%3],None,r=0)
text(290,609,'58 = 28 + 30',22,'TNRI','purple')
rect(400,489,200,132,'white','#c9a5df');text(500,515,'Compact MLP',23,'TNRB','purple');network(421,555,157,58,False)
arrow([(384,554),(400,554)],'purple');arrow([(600,554),(645,554)],'purple');mathtext(623,545,[('p',False),('i',True)],22,'purple');line([(617,527),(621,523),(625,527)],'purple',1.2)
arrow([(1075,203),(1075,233),(602,233),(602,477),(500,477),(500,489)],'purple',1.3,[5,4]);text(838,229,'Frozen feasibility weights',21.5,'TNRI','purple')
# Fixed scheduled set with cost gate before the expensive bank.
rect(645,489,365,132,'#f3faf6','green');text(827.5,515,'Fixed-schedule validation',23,'TNRB','green');text(827.5,539,'Learning + coverage · Budget K',21.5,'TNR','green')
rect(660,547,335,47,'#e7f1eb','#70a28c',r=2,lw=1)
mathtext(827.5,567,[('P exists: κ',False),('i',True),(' ≥ κ',False),('P',True),(' → skip',False)]);text(827.5,589,'Fixed slots; no refill',21.5,'TNR','green');text(827.5,614,'15 controls; stop at first witness',21.5)
# Incumbent and accepted proposal.
rect(1040,295,155,326,'#eaf5ed','green');text(1117.5,322,'Selection',24,'TNRB','green')
p=RPath();p.moveTo(1071,H-387);p.curveTo(1090,H-369,1096,H-340,1119,H-359);p.curveTo(1145,H-381,1151,H-395,1176,H-382);p.fillColor=None;p.strokeColor=col('green');p.strokeWidth=2.5;D.add(p);plane(1069,389,.37,'green')
text(1117.5,425,'Best verified',22,'TNR','green');text(1117.5,451,'pair P',24,'TNRI','green');mathtext(1117.5,488,[('min lex κ',False),('i',True)],22);mathtext(1117.5,532,[('Incumbent κ',False),('P',True)]);text(1117.5,588,'P or empty',22,'TNRI','green')
arrow([(845,380),(1040,380)],'green',1.8);text(936,366,'Witness',22,'TNR','green');arrow([(845,430),(1040,430)],'#7b8b96',1.3,[5,4]);text(941,420,'No witness; no slots',21.5,'TNRI','#7b8b96');arrow([(1010,610),(1040,610)],'green',1.8)
arrow([(1040,532),(1024,532),(1024,568),(995,568)],'green',1.3,[4,3])
# Actual simulator images; no real-world flight claim.
imagebox(ASSETS/'gazebo_suav1.png',1247,64,284,160);text(1389,249,'Gazebo: suav1 + 3-D lidar',22,'TNRB','green')
imagebox(ASSETS/'rviz_voxels.png',1247,269,284,165);text(1389,458,'RViz: online voxels + primitives',21.5,'TNRB','green')
rect(1247,481,284,102,'#e2f1e7','green');text(1389,506,'Timestamp-aware dispatch',22,'TNRB','green');text(1389,531,'Revalidate proposal P',21.5);text(1389,554,'Else check remaining sequence',21.5);text(1389,577,'Publish passing control / reject',21.5)
arrow([(1195,568),(1220,568),(1220,526),(1247,526)],'green',1.8)
text(1389,613,'Aircraft + actuator dynamics',21.5,'TNR','green');arrow([(1389,583),(1389,592)],'green');arrow([(1531,532),(1540,532),(1540,145),(1531,145)],'green',1.4)
# Shared observation feedback.
arrow([(1531,352),(1554,352),(1554,664),(92.5,664),(92.5,642)],'ink',1.4)
rect(579,650,400,29,'white',None,r=0);text(779,671,'State and lidar updates',24,'TNRI')
# Save at publication size; all text and geometric objects remain vectors.
SCALE=518.4/W;D.scale(SCALE,SCALE);D.width=W*SCALE;D.height=H*SCALE
renderPDF.drawToFile(D,str(OUT/'architecture_revised.pdf'))
# Drop the renderer's unused default-font setup, keeping visible labels entirely TNR.
pdoc=fitz.open(OUT/'architecture_revised.pdf');page=pdoc[0]
stream=page.read_contents()
empty_setup=b'BT /F1 12 Tf 14.4 TL ET'
assert empty_setup in stream[:200]
stream=stream.replace(empty_setup,b'',1)
pdoc.update_stream(page.get_contents()[0],stream)
for ref in page.get_contents()[1:]:pdoc.update_stream(ref,b'')
page.clean_contents(sanitize=True)
assert b'/F1 ' not in page.read_contents()
tnrref=next(f[0] for f in page.get_fonts() if f[3].endswith('TimesNewRomanPSMT'))
font_alias=str(tnrref)+' 0 R'
rtype,rvalue=pdoc.xref_get_key(page.xref,'Resources')
if rtype=='xref':
 rx=int(rvalue.split()[0]);ftype,fvalue=pdoc.xref_get_key(rx,'Font')
 if ftype=='xref':pdoc.xref_set_key(int(fvalue.split()[0]),'F1',font_alias)
 else:pdoc.xref_set_key(rx,'Font/F1',font_alias)
else:pdoc.xref_set_key(page.xref,'Resources/Font/F1',font_alias)
finished=pdoc.tobytes(garbage=4,deflate=True);pdoc.close();(OUT/'architecture_revised.pdf').write_bytes(finished)
renderSVG.drawToFile(D,str(OUT/'architecture_revised.svg'))
svg=(OUT/'architecture_revised.svg').read_text()
css=[]
for name,font_file in FONT_PATHS.items():
 css.append("@font-face{font-family:'"+name+"';src:url(data:font/truetype;base64,"+base64.b64encode(font_file.read_bytes()).decode()+") format('truetype');}")
svg=svg.replace('</desc>','</desc><defs><style type="text/css"><![CDATA['+''.join(css)+']]></style></defs>',1)
svg=svg.replace('viewBox="0 0 518 229"',f'viewBox="0 0 {D.width} {D.height}"').replace('translate(0,-229)',f'translate(0,-{D.height})')
for image in [ASSETS/'gazebo_suav1.png',ASSETS/'rviz_voxels.png']:
 svg=svg.replace(str(image),'data:image/png;base64,'+base64.b64encode(image.read_bytes()).decode())
(OUT/'architecture_revised.svg').write_text(svg)
report={'reference_style':'User-supplied structure1.pdf','figure_size_inches':[D.width/72,D.height/72],'min_body_font_pt':21.5*SCALE,'asset_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [ASSETS/'gazebo_suav1.png',ASSETS/'rviz_voxels.png']},'gazebo_crop_from_original':[330,310,790,570],'rviz_crop_from_original':[180,300,950,750],'screenshots':'Bundled original project Gazebo and RViz acquisition crops; no real-world photographs','layout':'Inputs at left, offline training above online planning, execution at right','labels':labels}
(OUT/'figure_design.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='labels'},indent=2))
