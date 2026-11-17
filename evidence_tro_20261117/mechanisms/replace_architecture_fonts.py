from pathlib import Path
import argparse,json,hashlib,collections,statistics
p=argparse.ArgumentParser(description='Replace architecture labels with Times New Roman, preserving vector drawings.');p.add_argument('--input-pdf',type=Path,required=True);p.add_argument('--output-pdf',type=Path,required=True);p.add_argument('--font-dir',type=Path,required=True);a=p.parse_args();SRC=a.input_pdf;DST=a.output_pdf;FONTS=a.font_dir;OUT=DST.parent;OUT.mkdir(parents=True,exist_ok=True)
import fitz
fontfiles={'regular':FONTS/'Times.TTF','bold':FONTS/'Timesbd.TTF'}
fonts={k:fitz.Font(fontfile=str(v)) for k,v in fontfiles.items()}
doc=fitz.open(SRC);page=doc[0];source_drawings=page.get_drawings()
raw=page.get_text('rawdict');targets=[];math=[]
def normchar(c,font):return (font,c['c'],tuple(c['origin']))
def glyphs_match(a,b):
 ga=collections.defaultdict(list);gb=collections.defaultdict(list)
 for font,c,xy in a:ga[font,c].append(xy)
 for font,c,xy in b:gb[font,c].append(xy)
 if ga.keys()!=gb.keys():return False
 for key in ga:
  if len(ga[key])!=len(gb[key]):return False
  if any(abs(x-y)>.002 for pa,pb in zip(sorted(ga[key]),sorted(gb[key])) for x,y in zip(pa,pb)):return False
 return True
for b in raw['blocks']:
 for line in b.get('lines',[]):
  spans=line['spans']
  for index,s in enumerate(spans):
   if s['font'].startswith('LMSans'):
    text=''.join(c['c'] for c in s['chars']);nonwhite=[c for c in s['chars'] if c['c'].strip()]
    assert nonwhite
    angle=90 if line['dir']==(0.,-1.) else 0
    assert line['dir'] in [(1.,0.),(0.,-1.)]
    weight='bold' if 'Bold' in s['font'] else 'regular'
    # Keep each label's anchor region. Adjacent math remains at its original positions.
    targets.append({'span':s,'text':text,'angle':angle,'weight':weight,'mixed':any(not q['font'].startswith('LMSans') for q in spans),'index':index,'count':len(spans)})
    for c in nonwhite:
     r=fitz.Rect(c['bbox']);r.x0+=.03;r.x1-=.03;r.y0+=.08;r.y1-=.08
     assert not r.is_empty
     page.add_redact_annot(r,fill=None,cross_out=False)
   else:
    math.extend(normchar(c,s['font']) for c in s['chars'] if c['c'].strip())
page.apply_redactions(images=0,graphics=0,text=0)
remaining=[]
for b in page.get_text('rawdict')['blocks']:
 for line in b.get('lines',[]):
  for s in line['spans']:
   remaining.extend(normchar(c,s['font']) for c in s['chars'] if c['c'].strip())
assert glyphs_match(math,remaining),{'removed_math':list((collections.Counter(math)-collections.Counter(remaining)).elements()),'remaining_original_text':list((collections.Counter(remaining)-collections.Counter(math)).elements())}
for weight,path in fontfiles.items():page.insert_font(fontname='TNR'+weight,fontfile=str(path),set_simple=True)
records=[]
for t in targets:
 s=t['span'];text=t['text'];font=fonts[t['weight']];size=s['size'];bbox=fitz.Rect(s['bbox']);color=fitz.sRGB_to_pdf(s['color']);width=font.text_length(text,fontsize=size)
 # PyMuPDF can place a synthetic space on a preceding subscript baseline;
 # the visible letters define the actual source baseline.
 glyphs=[c for c in s['chars'] if c['c'].strip()]
 if t['angle']==0:
  baseline=statistics.median(c['origin'][1] for c in glyphs)
  if t['mixed'] and t['index']==0:x=bbox.x1-width
  elif t['mixed'] and t['index']>0:
   lead=text[:len(text)-len(text.lstrip())]
   x=glyphs[0]['origin'][0]-font.text_length(lead,fontsize=size)
  else:x=(bbox.x0+bbox.x1-width)/2
  # A short preceding "If" is 0.4pt wider in Times; allow that expansion leftward.
  point=fitz.Point(x,baseline)
 else:
  point=fitz.Point(statistics.median(c['origin'][0] for c in glyphs),(bbox.y0+bbox.y1+width)/2)
 page.insert_text(point,text,fontname='TNR'+t['weight'],fontsize=size,color=color,rotate=t['angle'])
 records.append({'text':text,'source_font':s['font'],'new_font':font.name,'point':[point.x,point.y],'size':size,'rotate':t['angle']})
page.clean_contents(sanitize=True)
doc.subset_fonts();doc.save(DST,garbage=4,deflate=True)
check=fitz.open(DST);newpage=check[0]
assert [{k:v for k,v in d.items() if k!='seqno'} for d in newpage.get_drawings()]==[{k:v for k,v in d.items() if k!='seqno'} for d in source_drawings],'Vector drawings changed'
newmath=[];newlabels=[]
for b in newpage.get_text('rawdict')['blocks']:
 for line in b.get('lines',[]):
  for s in line['spans']:
   assert not s['font'].startswith('LMSans')
   if s['font'].startswith('TimesNewRoman'):
    newlabels.extend(c['c'] for c in s['chars'] if c['c'].strip())
   else:newmath.extend(normchar(c,s['font']) for c in s['chars'] if c['c'].strip())
assert glyphs_match(math,newmath),'Math glyphs or positions changed'
assert collections.Counter(c for t in targets for c in t['text'] if c.strip())==collections.Counter(newlabels),'Label characters changed'
newpage.get_pixmap(matrix=fitz.Matrix(3,3),alpha=False).save(DST.with_suffix('.png'))
report={'source':str(SRC),'source_sha256':hashlib.sha256(SRC.read_bytes()).hexdigest(),'output':str(DST),'output_sha256':hashlib.sha256(DST.read_bytes()).hexdigest(),'replaced_spans':len(targets),'preserved_math_characters':len(math),'preserved_vector_drawings':len(source_drawings),'page_size':list(newpage.rect),'verification':{'labels_preserved':True,'math_glyphs_and_positions_unchanged':True,'all_vector_drawings_unchanged':True,'no_sans_text_remains':True},'replacements':records}
(OUT/'architecture_font_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='replacements'},indent=2))
