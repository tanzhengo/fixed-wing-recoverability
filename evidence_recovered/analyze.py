#!/usr/bin/env python3
"""Recompute manuscript summaries from recovered tables. No ROS or archive needed."""
import argparse
import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np

PACKAGE = Path(__file__).resolve().parent
RUNS = ['learned3', 'cost1', 'diverse1']
METHOD_ALIASES = {'DiversePruned': 'Diverse', 'V2_DeferredPruned': 'Legacy compact deferred + pruning'}
FAMILIES = ['staggered', 'wall', 'culdesac', 'spatial']

def read_csv(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))

def read_json(path):
    return json.loads(path.read_text())

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

def mean(values):
    return float(np.mean(list(values)))

def cumulative_work(row):
    return float(row['mean_backup_steps']) * int(row['planning_calls'])

def check_copies():
    records = read_json(PACKAGE / 'COPIED_FILES.json')['files']
    for r in records:
        p = PACKAGE / r['path']
        assert p.stat().st_size == r['bytes'], r['path']
        assert sha(p) == r['sha256'], r['path']
    return {'copied_original_files': len(records), 'all_sha256_match': True}

def validation():
    rows = read_csv(PACKAGE / 'tables/validation_factorial.csv')
    cells = {}
    for cell in ['state_aux', 'observed', 'observed_aux']:
        a = [r for r in rows if r['cell'] == cell and r['mode'] == 'learned']
        assert len(a) == 3 and all(int(r['full_positive']) == 517 for r in a)
        cells[cell] = {'seeds': [int(r['seed']) for r in a],
            'recall_pct_mean': mean(100 * float(r['recall']) for r in a),
            'recall_pct_sample_sd': float(np.std([100 * float(r['recall']) for r in a], ddof=1)),
            'bce_mean': mean(float(r['bce']) for r in a),
            'full_positive_denominator': 517, 'inputs': 767}
    rows = read_csv(PACKAGE / 'tables/selection_episodes.csv')
    selection = {}
    for label in sorted({r['method'] for r in rows}):
        a = [r for r in rows if r['method'] == label]
        selection[label] = {'episodes': len(a), 'completed': sum(r['outcome'] == 'goal' for r in a),
            'mean_backup_steps': mean(float(r['mean_backup_steps']) for r in a)}
    frozen = read_json(PACKAGE / 'metadata/original/evaluation/validation_selection.json')
    for r in frozen['scores']:
        v = selection[r['label']]
        assert v['completed'] == r['completed'] and v['episodes'] == r['episodes']
        assert np.isclose(v['mean_backup_steps'], r['mean_backup_steps'], rtol=1e-10)
    return {'factorial': cells, 'execution_selection': selection,
        'final_selected': {'model': 'models/selected_obs_bce.txt', 'order': 'learned_coverage',
            'coverage_weight_lambda_c': 0.75, 'training_seed': 20261103, 'checkpoint_epoch': 23}}

def episode_summary(rows):
    result = {}
    for method in sorted({r['method'] for r in rows}):
        a = [r for r in rows if r['method'] == method]
        result[method] = {'tasks': len(a), 'outcomes': dict(Counter(r['outcome'] for r in a)),
            'completed': sum(r['outcome'] == 'goal' for r in a),
            'mean_cumulative_successor_steps': mean(cumulative_work(r) for r in a),
            'mean_calls_per_task': mean(int(r['planning_calls']) for r in a),
            'mean_episode_mean_successor_steps_per_call': mean(float(r['mean_backup_steps']) for r in a),
            'mean_sim_seconds': mean(float(r['sim_seconds']) for r in a)}
    return result

def matched_queries():
    raw = []
    instrumented = []
    budget = []
    for run in RUNS:
        for name, target in [('query_results_quiet.csv', raw),
                             ('query_results_quiet_instrumented.csv', instrumented),
                             ('query_results_budget_grid.csv', budget)]:
            target.extend(dict(r, run=run) for r in read_csv(PACKAGE / 'tables/urban_raw' / run / name))
    grouped = defaultdict(list)
    for r in raw:
        grouped[(r['run'], int(r['query_index']), r['method'])].append(r)
    medians = {}
    for key, a in grouped.items():
        assert len(a) == 3
        # Repeats are timing replicates, not additional independent inputs.
        for field in ['signature', 'witness', 'backup_steps', 'queries', 'cost', 'inference_batches', 'pruned_candidates']:
            assert len({r[field] for r in a}) == 1, (key, field)
        medians[key] = dict(a[0], ms=str(np.median([float(r['ms']) for r in a])))
    archived = read_csv(PACKAGE / 'tables/urban_matched_medians.csv')
    for r in archived:
        key = (r['run'], int(r['query_index']), r['method'])
        assert np.isclose(float(r['ms']), float(medians[key]['ms']), rtol=1e-12)
        assert r['signature'] == medians[key]['signature']
    full = {(run, q): r for (run, q, method), r in medians.items() if method == 'Full'}
    assert len(full) == 123 and all(r['witness'] == '1' for r in full.values())
    result = {}
    for method in sorted({key[2] for key in medians}):
        a = [r for key, r in medians.items() if key[2] == method]
        found = [r for r in a if r['witness'] == '1']
        ms = np.array([float(r['ms']) for r in a])
        result[method] = {'paper_label': METHOD_ALIASES.get(method, method),
            'inputs': len(a), 'full_positive': len(full), 'witnesses': len(found),
            'recall_pct': 100 * len(found) / len(full),
            'mean_backup_steps': mean(float(r['backup_steps']) for r in a),
            'mean_queries': mean(float(r['queries']) for r in a),
            'mean_cost_regret_on_witnesses': mean(float(r['cost']) - float(full[(r['run'], int(r['query_index']))]['cost']) for r in found),
            'time_ms': {'median': float(np.median(ms)), 'p95': float(np.percentile(ms, 95)),
                        'p99': float(np.percentile(ms, 99)), 'max': float(ms.max())},
            'inference_batches': sum(int(r['inference_batches']) for r in a),
            'mean_pruned_candidates': mean(float(r['pruned_candidates']) for r in a)}
    components = {}
    fields = ['ms', 'primary_ms', 'feature_ms', 'inference_ms', 'schedule_other_ms', 'continuation_ms', 'residual_ms']
    for method in sorted(result):
        a = [r for r in instrumented if r['method'] == method]
        assert len(a) == 123
        components[method] = {field: mean(float(r[field]) for r in a) for field in fields}
    full_budget = {(r['run'], r['query_index']): r for r in budget if r['method'] == 'Full'}
    budgets = []
    for method in ['Cost', 'Diverse', 'Selected', 'ObservedCoverage']:
        for k in [1, 2, 4, 8, 12, 24]:
            a = [r for r in budget if r['method'] == '{}_K{}'.format(method, k)]
            assert len(a) == 123
            found = [r for r in a if r['witness'] == '1']
            budgets.append({'method': method, 'K': k, 'inputs': 123, 'witnesses': len(found),
                'mean_backup_steps': mean(float(r['backup_steps']) for r in a),
                'mean_cost_regret_witness': mean(float(r['cost']) - float(full_budget[(r['run'], r['query_index'])]['cost']) for r in found)})
    for r in read_csv(PACKAGE / 'tables/urban_budget_grid_summary.csv'):
        actual = next(v for v in budgets if v['method'] == r['method'] and v['K'] == int(r['K']))
        for field in ['witnesses', 'mean_backup_steps', 'mean_cost_regret_witness']:
            assert np.isclose(actual[field], float(r[field]), rtol=1e-10, atol=1e-12)
    return {'uninstrumented_within_input_medians': result, 'instrumented_component_means_ms': components,
        'budget_grid': budgets, 'repeat_decisions_verified_identical': True,
        'obs_aux_work_reduction_vs_diverse_pct': 100 * (1 - result['ObsAux']['mean_backup_steps'] / result['DiversePruned']['mean_backup_steps']),
        'obs_aux_work_reduction_vs_full_pct': 100 * (1 - result['ObsAux']['mean_backup_steps'] / result['Full']['mean_backup_steps'])}

def paired_bootstrap(procedural, forest):
    protocol = read_json(PACKAGE / 'metadata/paired_analysis_protocol.json')
    rng = np.random.default_rng(protocol['bootstrap']['seed'])
    replicates = protocol['bootstrap']['replicates']
    draws = {}
    def stats(rows, common=False):
        chosen = {m: {(r['family'], int(r['seed']), int(r['condition'])): r for r in rows if r['method'] == m}
                  for m in ['Selected', 'Diverse']}
        assert chosen['Selected'].keys() == chosen['Diverse'].keys()
        keys = sorted(chosen['Selected'])
        if common:
            keys = [k for k in keys if chosen['Selected'][k]['outcome'] == chosen['Diverse'][k]['outcome'] == 'goal']
        clusters = defaultdict(list)
        for k in keys:
            clusters[k[:2]].append(k)
        units = []
        for _, ks in sorted(clusters.items()):
            s = [chosen['Selected'][k] for k in ks]
            d = [chosen['Diverse'][k] for k in ks]
            units.append([sum(cumulative_work(r) for r in s), sum(cumulative_work(r) for r in d),
                sum(r['outcome'] == 'goal' for r in s), sum(r['outcome'] == 'goal' for r in d), len(ks)])
        a = np.array(units, dtype=float)
        # Shared layout IDs use identical draws across forest densities.
        cachekey = tuple(k[1] for k in sorted(clusters))
        if cachekey not in draws:
            draws[cachekey] = rng.integers(0, len(a), size=(replicates, len(a)))
        total = a[draws[cachekey]].sum(axis=1)
        p = a.sum(axis=0)
        reduction = 100 * (1 - total[:, 0] / total[:, 1])
        difference = 100 * (total[:, 2] - total[:, 3]) / total[:, 4]
        return {'paired_tasks': len(keys), 'map_clusters': len(a),
            'conditions_per_cluster': sorted({len(v) for v in clusters.values()}),
            'Selected_mean_work': float(p[0] / p[4]), 'Diverse_mean_work': float(p[1] / p[4]),
            'work_reduction_pct': float(100 * (1 - p[0] / p[1])),
            'work_reduction_pct_ci95': np.percentile(reduction, [2.5, 97.5]).tolist(),
            'Selected_completed': int(p[2]), 'Diverse_completed': int(p[3]),
            'completion_difference_pp': float(100 * (p[2] - p[3]) / p[4]),
            'completion_difference_pp_ci95': np.percentile(difference, [2.5, 97.5]).tolist(),
            'map_ids': [[k[0], k[1]] for k in sorted(clusters)]}
    report = {'protocol': protocol, 'procedural': {'all_tasks': stats(procedural), 'common_completions': stats(procedural, True)}, 'forest': {}}
    for density in protocol['forest']['density_levels']:
        a = [r for r in forest if int(r['tree_count']) == density]
        report['forest'][str(density)] = {'all_tasks': stats(a), 'common_completions': stats(a, True)}
    return report

def fixed_failure():
    result = {}
    for method in ['Selected', 'Diverse', 'Full']:
        prefix = PACKAGE / 'tables/failure_case' / ('culdesac_0_m{}_c0'.format(method))
        p = read_csv(Path(str(prefix) + '_plans.csv'))
        t = read_csv(Path(str(prefix) + '_path.csv'))
        s = read_csv(Path(str(prefix) + '_summary.csv'))[0]
        assert int(s['seed']) == 1321000 and int(s['condition']) == 0
        first = next(float(r['t']) for r in p if r['executable'] == '1')
        fresh = [r for r in p if r['executable'] == r['witness'] == '1']
        total = sum(int(r['backup_steps']) for r in p)
        assert np.isclose(total, cumulative_work(s), atol=1e-5)
        result[method] = {'outcome': s['outcome'], 'planning_calls': len(p),
            'termination_sim_s': float(t[-1]['t']), 'active_execution_duration_s': float(t[-1]['t']) - first,
            'last_fresh_pair_s': float(fresh[-1]['t']), 'terminal_primary_feasible': int(p[-1]['primary_feasible']),
            'successor_integration_steps': total, 'mean_successor_steps_per_call': total / len(p),
            'min_chord_clearance_m': float(s['min_chord_clearance']), 'terminal_status': p[-1]['status']}
    return result

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True, help='New analysis output directory (outside this package recommended)')
    args = parser.parse_args()
    checks = check_copies()
    all_original = read_csv(PACKAGE / 'tables/procedural_and_legacy_forest_episodes.csv')
    procedural = [r for r in all_original if r['family'] in FAMILIES]
    forest = read_csv(PACKAGE / 'tables/forest600_episodes.csv')
    assert len(procedural) == 1152 and len(forest) == 120
    results = {'integrity': checks, 'validation': validation(), 'procedural': episode_summary(procedural),
        'procedural_by_family': {f: episode_summary([r for r in procedural if r['family'] == f]) for f in FAMILIES},
        'forest600': {str(d): episode_summary([r for r in forest if int(r['tree_count']) == d]) for d in [100, 200, 300]},
        'urban_replay': matched_queries(), 'city_flights': read_csv(PACKAGE / 'tables/city_flights.csv'),
        'prescribed_failure': fixed_failure(),
        'scope': {'procedural_tasks_per_method': 128, 'procedural_maps': 32,
                  'legacy_40_80_tree_rows_excluded_from_final_forest_results': len(all_original) - len(procedural),
                  'city_acquisitions_per_method': 1,
                  'timing': 'query CPU milliseconds use within-input medians; flight clocks are ROS simulation time'}}
    paired = paired_bootstrap(procedural, forest)
    assert results['procedural']['Selected']['completed'] == 96
    assert results['urban_replay']['uninstrumented_within_input_medians']['Selected']['witnesses'] == 123
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'main_results.json').write_text(json.dumps(results, indent=2) + '\n')
    (args.out / 'paired_uncertainty.json').write_text(json.dumps(paired, indent=2) + '\n')
    print('Verified {} original files; recomputed all summaries and {} cluster bootstrap replicates.'.format(checks['copied_original_files'], paired['protocol']['bootstrap']['replicates']))
    print('Selected/Diverse common procedural work reduction: {:.8f}%'.format(paired['procedural']['common_completions']['work_reduction_pct']))
    print('Outputs:', args.out)

if __name__ == '__main__':
    main()
