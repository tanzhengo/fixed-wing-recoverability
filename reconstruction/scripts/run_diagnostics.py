#!/usr/bin/env python3
"""Run new, deterministic numerical fixtures; never reconstruct historical data.

Only the supplied manuscript-based implementation is exercised. Map cells are
constructed openly, with unobserved space outside the declared free boxes.
No plant is flown and no lidar-history or cul-de-sac trajectory is invented.
"""
import argparse
import csv
from dataclasses import asdict, replace
import gzip
import hashlib
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Patch
from fwcontinuation.types import Config
from fwcontinuation.dynamics import trim_state, generate_primary, generate_successors, rollout
from fwcontinuation.observed_map import ObservedMap
from fwcontinuation.features import encode_features, observation_features
from fwcontinuation.planner import feasible_candidates, continuation_query, search_candidates

STATUS = 'NEW NUMERICAL FIXTURES - NOT HISTORICAL FLIGHTS'
BLUE, ORANGE, GREEN, RED = '#3777a8', '#b5712d', '#21816c', '#b64a4e'


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def state_bytes(state):
    return struct.pack('<10d', *asdict(state).values())


def control_bytes(control):
    return struct.pack('<4d', *asdict(control).values())


def save_json(path, value):
    path.write_bytes(json.dumps(value, indent=2, sort_keys=True, allow_nan=False).encode())


def save_csv(path, rows, fieldnames=None):
    rows = list(rows)
    if not rows and not fieldnames:
        return
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames or list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def box_map(spec, config):
    result = ObservedMap(config.voxel_size)
    # Bounds are voxel-aligned; no implicit rounding or interpolated map states.
    ranges = []
    for low, high in spec['free_bounds']:
        assert low / config.voxel_size == int(low / config.voxel_size)
        assert high / config.voxel_size == int(high / config.voxel_size)
        ranges.append(range(int(low / config.voxel_size), int(high / config.voxel_size)))
    for ix in ranges[0]:
        for iy in ranges[1]:
            for iz in ranges[2]:
                result.set_cell((ix, iy, iz), 'free')
    return result


def export_rollout(out, name, path):
    save_csv(out / (name + '.csv'), [dict(time=t, **asdict(s)) for t, s in zip(path.times, path.states)])
    return {'path': name + '.csv', 'valid': path.valid, 'reason': path.reason,
            'steps': path.steps, 'start_time': path.times[0], 'end_time': path.times[-1]}


def complete_fan(candidate, observed_map, config, out, prefix):
    """An extra diagnostic fan, explicitly excluded from the online query work."""
    paths = []
    records = []
    for index, command in enumerate(generate_successors(candidate.terminal, config)):
        path = rollout(candidate.terminal, command, config, observed_map, start_time=config.H)
        record = {'successor_id': index, 'control': asdict(command),
                  **export_rollout(out, prefix + '_successor_%02d' % index, path)}
        records.append(record)
        paths.append(path)
    save_json(out / (prefix + '_full_fan.json'), records)
    return paths, records


def save_figure(fig, out, name):
    for extension in ('pdf', 'svg', 'png'):
        fig.savefig(out / (name + '.' + extension), dpi=300, bbox_inches='tight', pad_inches=.05)
    plt.close(fig)


def scene_background(ax, spec, title):
    bounds = spec['free_bounds']
    ax.set_facecolor('#e2e5e8')
    ax.add_patch(Rectangle((bounds[0][0], bounds[1][0]), bounds[0][1]-bounds[0][0],
                           bounds[1][1]-bounds[1][0], color='white', zorder=0))
    ax.axvline(bounds[0][1], color='#6c7884', ls=':', lw=.9)
    ax.plot(0, 0, 'ks', ms=3)
    ax.set(xlim=(-8, 145), ylim=(-62, 62), xlabel='x (m)', ylabel='y (m)', title=title)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(alpha=.12)


def plot_path(ax, candidate, fan, color, label, only_first_valid=False):
    primary = candidate.rollout.states
    ax.plot([s.x for s in primary], [s.y for s in primary], color=color, lw=1.7, label=label)
    ax.plot(candidate.terminal.x, candidate.terminal.y, 'o', color=color, ms=3)
    for path in fan:
        if only_first_valid and not path.valid:
            continue
        states = path.states
        ax.plot([s.x for s in states], [s.y for s in states], color=GREEN if path.valid else color,
                ls='--', lw=1.0 if path.valid else .5, alpha=.95 if path.valid else .55)
        if not path.valid:
            ax.plot(states[-1].x, states[-1].y, marker='x', color=color, ms=2.7, mew=.6)
        if only_first_valid and path.valid:
            break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol', type=Path, default=ROOT/'configs/diagnostic_protocol.json')
    parser.add_argument('--output', type=Path, default=ROOT.parent/'reconstruction_validation')
    parser.add_argument('--model', type=Path, help='Optional genuinely trained reconstructed checkpoint; never an old or random fallback')
    parser.add_argument('--model-label', default='Learned (new smoke model)', help='Figure label; must not imply the historical Selected configuration')
    args = parser.parse_args()
    protocol = json.loads(args.protocol.read_text(encoding='utf-8'))
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    config = Config(**protocol['config_overrides'])
    initial_spec = dict(protocol['initial'])
    initial = trim_state(**initial_spec)
    goal = tuple(protocol['goal'])
    source_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for path in sorted((ROOT/'fwcontinuation').glob('*.py'))}
    source_hashes[str(Path(__file__).relative_to(ROOT))] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    report = {'evidence_status': STATUS, 'protocol': protocol, 'config': asdict(config),
              'initial': asdict(initial), 'goal': goal, 'source_sha256': source_hashes,
              'protocol_sha256': hashlib.sha256(args.protocol.read_bytes()).hexdigest(),
              'learned_status': protocol['learned_schedule'], 'culdesac_status': protocol['culdesac']}
    model = None
    if args.model is not None:
        from fwcontinuation.learning import ContinuationMLP
        if 'selected' in args.model_label.lower():
            raise ValueError('A newly supplied checkpoint must not be labeled as the historical Selected configuration')
        model = ContinuationMLP.load(args.model)
        report['learned_status'] = 'Supplied trained reconstruction checkpoint; pipeline diagnostic only, no historical-model or validation-selection claim'
        report['model'] = {'path': str(args.model.resolve()),
                           'sha256': hashlib.sha256(args.model.read_bytes()).hexdigest(),
                           'label': args.model_label, 'provenance': model.provenance}
    maps = {}
    for key in ('map1', 'map2'):
        maps[key] = box_map(protocol[key], config)
        snapshot = maps[key].snapshot()
        data = canonical(snapshot)
        with gzip.open(out/(key+'_snapshot.json.gz'), 'wb') as stream:
            stream.write(data)
        report[key+'_snapshot_sha256'] = hashlib.sha256(data).hexdigest()
    # Independent smoke-training bounds, not the two illustration maps. These
    # are unlabelled inputs only; label_dataset performs all actual oracle work.
    smoke_info = []
    for split in ('train', 'validation'):
        frames = []
        for spec in protocol['training_smoke_frames'][split]:
            smoke_map = box_map(spec, config)
            smoke_state = trim_state(h=spec['h'])
            frame = {'scene_id': spec['scene_id'], 'query_id': '0', 'state': asdict(smoke_state),
                     'goal': [250.0, 0.0, spec['h']], 'map': smoke_map.snapshot(),
                     'config': asdict(config), 'previous': None,
                     'provenance': {'evidence_status': STATUS, 'use': 'training-pipeline smoke only',
                                    'constructed_free_bounds': spec['free_bounds']}}
            frames.append(frame)
            smoke_info.append({'split': split, 'scene_id': spec['scene_id'], 'frame_sha256': digest(frame),
                               'free_bounds': spec['free_bounds']})
        with (out/(split+'_frames.jsonl')).open('wb') as stream:
            for frame in frames:
                stream.write(canonical(frame)+b'\n')
    with (out/'diagnostic_test_frames.jsonl').open('wb') as stream:
        for key in ('map1', 'map2'):
            frame = {'scene_id': 'diagnostic_test_'+key, 'query_id': '0', 'state': asdict(initial),
                     'goal': list(goal), 'map': maps[key].snapshot(), 'config': asdict(config), 'previous': None,
                     'provenance': {'evidence_status': STATUS, 'use': 'illustrative controlled-map diagnostic',
                                    'constructed_free_bounds': protocol[key]['free_bounds']}}
            stream.write(canonical(frame)+b'\n')
    report['training_smoke_frames'] = smoke_info
    print('Constructed the two declared map fixtures.', flush=True)
    candidates, primary_steps = feasible_candidates(initial, goal, maps['map1'], config)
    report['primary_bank_count'] = len(generate_primary(initial, goal, config))
    report['feasible_primary_count'] = len(candidates)
    report['primary_generation_steps'] = primary_steps
    if not candidates:
        report['status'] = 'No feasible primaries in declared fixture; no comparison manufactured.'
        save_json(out/'diagnostic_manifest.json', report)
        print(report['status'])
        return

    # Establish both labels using a predeclared cost-order rule, not guessed geometry.
    labels, oracle_records = {}, []
    chosen = {}
    for candidate in candidates:
        witness, work, successors = continuation_query(candidate, maps['map1'], config)
        label = int(witness is not None)
        labels[candidate.index] = label
        oracle_records.append({'candidate_id': candidate.index, 'label': label, 'steps': work,
                               'successors': successors})
        chosen.setdefault(label, candidate)
        if len(chosen) == 2:
            break
    save_json(out/'label_discovery_calls.json', oracle_records)
    save_csv(out/'feasible_candidates.csv', [dict(candidate_id=c.index, rank=c.rank, cost=c.cost,
        A=1, C=labels.get(c.index, ''), control_phi=c.control.phi, control_alpha=c.control.alpha,
        control_T=c.control.T, duration=c.control.duration, **{'terminal_'+k:v for k,v in asdict(c.terminal).items()}) for c in candidates])
    report['label_discovery_steps'] = sum(r['steps'] for r in oracle_records)
    report['label_discovery_examined_candidates'] = len(oracle_records)
    report['label_discovery_selection'] = {str(label): c.index for label,c in chosen.items()}
    print('Feasible pool: %d; inspected %d candidates; found labels %s.' %
          (len(candidates), len(oracle_records), sorted(chosen)), flush=True)

    # Query traces are actual, direct validation calls. No precomputed oracle is
    # substituted and no randomized or archived model is used as a learner.
    search_records = []
    all_trace_rows = []
    schedule_entries = list(protocol['schedules'])
    if model is not None:
        schedule_entries.append({'mode': 'learned', 'label': args.model_label})
    for entry in schedule_entries:
        setting = replace(config, coverage_weight=entry.get('coverage_weight', config.coverage_weight))
        result = search_candidates(initial, candidates, goal, maps['map1'], setting, mode=entry['mode'], model=model)
        record = {'mode': entry['mode'], 'label': entry['label'], 'status': result.status,
                  'steps': result.steps, 'schedule': result.schedule,
                  'selected_candidate_id': result.witness.candidate.index if result.witness else None,
                  'selected_cost': result.witness.candidate.cost if result.witness else None,
                  'queries': result.queries, 'scores': result.recorded_scores,
                  'coverage_weight': setting.coverage_weight}
        search_records.append(record)
        for query in result.queries:
            all_trace_rows.append({k:v for k,v in dict(method=entry['label'], **query).items() if k!='successors'})
    report['searches'] = search_records
    save_json(out/'query_traces.json', search_records)
    save_csv(out/'query_traces.csv', all_trace_rows)

    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.titlesize':8.5,
        'axes.labelsize':8,'xtick.labelsize':7,'ytick.labelsize':7,
        'pdf.fonttype':42,'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False})
    fig,(top,bottom)=plt.subplots(2,1,figsize=(7.2,3.5),gridspec_kw={'height_ratios':[1,1.1]})
    for row,record in enumerate(search_records):
        lookup={q['position']:q for q in record['queries']}
        for slot in range(config.K):
            q=lookup.get(slot)
            color='#f0f1f2' if q is None else '#c7ccd1' if q['action']=='pruned' else GREEN if q['label'] else '#d98282'
            top.add_patch(Rectangle((slot+.05,row-.36),.9,.72,facecolor=color,edgecolor='white',lw=.5,
                                    hatch='///' if q and q['action']=='pruned' else None))
            if q is not None:
                top.text(slot+.5,row,str(q['candidate_id']),ha='center',va='center',fontsize=6.6,
                         color='white' if q.get('label') else '#1c2b39')
        qs=record['queries']; positions=[0]+[q['position']+1 for q in qs]
        steps=np.cumsum([0]+[q['steps'] for q in qs])
        bottom.step(positions,steps,where='post',lw=1.7 if record['mode']=='diverse' else 1.3,
                    ls='--' if record['mode']=='diverse' else '-',zorder=4 if record['mode']=='diverse' else 3,
                    label='%s: %d steps; %s'%(record['label'],record['steps'],record['status']))
    top.set(xlim=(0,config.K),ylim=(len(search_records)-.5,-.5),
            xticks=np.arange(config.K)+.5,xticklabels=np.arange(1,config.K+1),
            yticks=np.arange(len(search_records)),yticklabels=[r['label'] for r in search_records],
            xlabel='Scheduled position (including common cheapest primary)',title='(a) Actual candidate IDs and validation outcomes')
    top.tick_params(length=0)
    bottom.set(xlim=(0,config.K),xlabel='Scheduled positions processed',ylabel='Successor integration steps',
               title='(b) Work actually performed by each search')
    bottom.grid(alpha=.18);bottom.legend(fontsize=6.5,frameon=False)
    fig.suptitle(STATUS,fontsize=9,color='#9b3737')
    model_note = 'Learned model is newly trained; not historical Selected.' if model is not None else 'Learned schedule pending a trained checkpoint.'
    fig.text(.5,.013,'Green: witness; red: all 15 fail; hatch: pruned; light gray: unreached. '+model_note,ha='center',fontsize=6.3)
    fig.subplots_adjust(top=.83,bottom=.16,left=.17,right=.99,hspace=.72)
    save_figure(fig,out,'query_work_diagnostic')

    if 0 in chosen:
        a=chosen[0]
        primary2=rollout(initial,a.control,config,maps['map2'])
        identical_path = ([state_bytes(s) for s in a.rollout.states] == [state_bytes(s) for s in primary2.states])
        assert primary2.valid and identical_path
        bank1=generate_successors(a.terminal,config)
        bank2=generate_successors(primary2.states[-1],config)
        assert [control_bytes(c) for c in bank1]==[control_bytes(c) for c in bank2]
        witness2,work2,records2=continuation_query(a,maps['map2'],config)
        descriptor1=observation_features(a.terminal,maps['map1'],config)
        descriptor2=observation_features(a.terminal,maps['map2'],config)
        f1=encode_features(initial,a,goal,maps['map1'],config)
        f2=encode_features(initial,a,goal,maps['map2'],config)
        assert np.array_equal(f1[:28],f2[:28])
        map_pair={'candidate_id':a.index,'A_map1':1,'A_map2':int(primary2.valid),'C_map1':0,
                  'C_map2':int(witness2 is not None),'map2_query_steps':work2,'map2_query':records2,
                  'raw_initial_state_hex':state_bytes(initial).hex(),'raw_goal_hex':struct.pack('<3d',*goal).hex(),
                  'primary_control_hex':control_bytes(a.control).hex(),
                  'terminal_state_hex':state_bytes(a.terminal).hex(),
                  'successor_bank_hex':[control_bytes(c).hex() for c in bank1],
                  'same_primary_state_path_bytes':identical_path,'same_state_feature_bytes':f1[:28].tobytes()==f2[:28].tobytes(),
                  'features_map1':f1.tolist(),'features_map2':f2.tolist()}
        report['same_raw_state_map_pair']=map_pair
        save_json(out/'same_raw_state_map_pair.json',map_pair)
        export_rollout(out,'primary_A',a.rollout)
        fan_a,fan_a_records=complete_fan(a,maps['map1'],config,out,'map1_A')
        fan_a2,fan_a2_records=complete_fan(a,maps['map2'],config,out,'map2_A')
        assert not any(p.valid for p in fan_a)
        report['additional_full_fan_steps']=sum(p.steps for p in fan_a+fan_a2)
        if 1 in chosen:
            b=chosen[1]
            export_rollout(out,'primary_B',b.rollout)
            fan_b,fan_b_records=complete_fan(b,maps['map1'],config,out,'map1_B')
            report['additional_full_fan_steps']+=sum(p.steps for p in fan_b)
            fig,axs=plt.subplots(2,2,figsize=(7.2,4.8))
            scene_background(axs[0,0],protocol['map1'],'(a) Same map: A=1 for both primaries')
            plot_path(axs[0,0],a,fan_a,BLUE,'Primary A: C=0')
            plot_path(axs[0,0],b,fan_b,ORANGE,'Primary B: C=1',only_first_valid=True)
            axs[0,0].legend(frameon=False,fontsize=6.5,loc='lower left')
            axs[0,1].axis('off')
            text=(f'(b) Explicit finite-bank checks\n\n'
                  f'Candidate A: id {a.index}; J = {a.cost:.5f}\n'
                  f'All 15 successors fail in map 1.\n\n'
                  f'Candidate B: id {b.index}; J = {b.cost:.5f}\n'
                  f'At least one complete successor passes.\n\n'
                  f'H = B = 3 s; K = 12; dt = 0.02 s.\n'
                  f'Full 3-D states and segments are checked.\n'
                  f'The drawing shows XY projections only.')
            axs[0,1].text(.02,.98,text,va='top',fontsize=8,linespacing=1.45)
            scene_background(axs[1,0],protocol['map2'],f'(c) Same candidate A, extended map: C={int(witness2 is not None)}')
            axs[1,0].axvline(protocol['map1']['free_bounds'][0][1],color='#9b3737',ls=':',lw=.9)
            plot_path(axs[1,0],a,fan_a2,BLUE,'Identical primary A',only_first_valid=True)
            axs[1,0].legend(frameon=False,fontsize=6.5,loc='lower left')
            values=np.stack([descriptor1[::2],descriptor1[1::2],descriptor2[::2],descriptor2[1::2]])
            im=axs[1,1].imshow(values,vmin=0,vmax=1,cmap='Blues',aspect='auto')
            axs[1,1].set(yticks=range(4),yticklabels=['Map 1 free','Map 1 unknown','Map 2 free','Map 2 unknown'],
                          xticks=[1,4,7,10,13],xticklabels=['-45','-22.5','0','22.5','45'],
                          xlabel='Azimuth group (each: elevations -20, 0, 20 deg)',title='(d) Measured descriptor fractions')
            for cut in [2.5,5.5,8.5,11.5]:axs[1,1].axvline(cut,color='white',lw=1)
            fig.colorbar(im,ax=axs[1,1],fraction=.036,pad=.02,ticks=[0,.5,1])
            fig.suptitle(STATUS,fontsize=9,color='#9b3737')
            fig.text(.5,.013,'Constructed free boxes (white); outside unknown (gray). Dashed paths: successor checks. No flight, lidar replay, or long-term safety claim.',ha='center',fontsize=6.2)
            fig.subplots_adjust(top=.88,bottom=.12,left=.075,right=.98,hspace=.49,wspace=.43)
            save_figure(fig,out,'continuation_and_map_diagnostic')
    report['status']='completed; empirical manuscript figures remain pending historical-compatible data'
    report['outputs_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.iterdir())
                              if p.is_file() and p.name!='diagnostic_manifest.json'}
    # Guard against source mutation while producing a claimed reproducible set.
    report['source_unchanged_during_run']=all(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==sha
                                             for name,sha in source_hashes.items())
    save_json(out/'diagnostic_manifest.json',report)
    print(json.dumps({'status':report['status'],'source_unchanged':report['source_unchanged_during_run'],
                      'output':str(out),'map_pair':report.get('same_raw_state_map_pair',{}).get('C_map2'),
                      'schedule_steps':{r['label']:r['steps'] for r in search_records}},indent=2))


if __name__=='__main__':
    main()
