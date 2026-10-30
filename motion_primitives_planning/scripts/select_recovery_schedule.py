#!/usr/bin/env python3
"""Select the shared neural scheduling exponent using validation maps only.

Replay applies the planner's candidate ordering, early success, and exact
integration-budget accounting to complete-query labels. Geometry is recovered
from the version-1 deployment features. It never loads a held-out test split.
"""
import argparse
import csv
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


EXPONENTS = (0.0, 0.5, 1.0)
BUDGETS = (("K12", 12, 0), ("W3600", 203, 3600))


def sha(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def load_validation(path):
    path = Path(path)
    digest = sha(path)
    cache = path.with_suffix('.npz')
    if cache.exists():
        data = dict(np.load(cache, allow_pickle=False))
        if str(data.get('source_sha256', '')) == digest:
            return data
    features, labels, work, state, scene, family, cost, candidate = ([] for _ in range(8))
    with path.open() as stream:
        reader = csv.DictReader(stream)
        assert [k for k in reader.fieldnames if k.startswith('f') and k[1:].isdigit()] == ['f%d' % i for i in range(103)]
        for row in reader:
            features.append([float(row['f%d' % i]) for i in range(103)])
            labels.append(int(row['verified'])); work.append(int(row['backup_steps']))
            state.append(int(row['state_id'])); scene.append(row['scene']); family.append(row['family'])
            cost.append(float(row['cost'])); candidate.append(int(row['candidate_id']))
    data = dict(x=np.asarray(features, dtype=np.float32), y=np.asarray(labels, dtype=np.float32),
                work=np.asarray(work, dtype=np.int32), state=np.asarray(state, dtype=np.int32),
                scene=np.asarray(scene), family=np.asarray(family), cost=np.asarray(cost),
                candidate=np.asarray(candidate), source_sha256=np.asarray(digest))
    np.savez_compressed(cache, **data)
    return data


def utility(output, exponent):
    output = np.asarray(output, dtype=np.float32)
    # Stable sigmoid, with the same positive/negative branches as C++.
    p = np.empty(len(output), dtype=np.float32)
    positive = output[:, 0] >= 0
    p[positive] = 1 / (1 + np.exp(-output[positive, 0]))
    e = np.exp(output[~positive, 0]); p[~positive] = e / (1 + e)
    steps = np.expm1(np.clip(output[:, 1], np.float32(0), np.float32(12)))
    return p / np.power(np.float32(1) + steps / np.float32(1000), np.float32(exponent))


def candidate_schedule(features, costs, scores, mode, count):
    """Yield original row indices, reproducing the deployed schedule semantics."""
    n = len(costs)
    if not n or count <= 0:
        return
    cost_order = np.argsort(costs, kind='stable')
    limit = min(n, count)
    if mode == 'cost':
        yield from cost_order[:limit]
        return
    if mode not in ('diverse', 'learned', 'learned_diverse'):
        raise ValueError('Unknown scheduling mode: ' + mode)
    features = np.asarray(features, dtype=np.float32)
    # Heading differences and altitude differences are invariant to the common
    # initial heading/altitude removed by the feature encoder.
    headings = np.arctan2(features[:, 15], features[:, 16])
    heights = features[:, 14] * np.float32(30)
    speed = np.maximum(np.float32(3), features[0, 6] * np.float32(20))
    horizon = features[0, 22] * np.float32(10)
    heading_span = np.minimum(np.float32(np.pi),
                             np.float32(9.81) * np.abs(np.tan(np.float32(np.deg2rad(35)))) * horizon / speed)
    height_span = speed * horizon * np.tan(np.float32(np.deg2rad(20)))
    learned_order = cost_order[np.argsort(-np.asarray(scores)[cost_order], kind='stable')]
    used = np.zeros(n, dtype=bool)
    closest = np.full(n, np.inf, dtype=np.float32)
    current = int(cost_order[0]); used[current] = True
    yield current
    for selected_count in range(1, limit):
        difference = headings - headings[current]
        angle = np.abs(np.arctan2(np.sin(difference), np.cos(difference)))
        hd = angle / heading_span if heading_span > 1e-5 else np.zeros(n, dtype=np.float32)
        zd = (heights - heights[current]) / height_span if height_span > 1e-5 else np.zeros(n, dtype=np.float32)
        closest = np.minimum(closest, hd * hd + zd * zd)
        if mode == 'learned' or (mode == 'learned_diverse' and selected_count % 2 == 1):
            current = int(next(i for i in learned_order if not used[i]))
        else:
            # C++ walks stable cost order and only replaces the incumbent when
            # the distance exceeds its value by 1e-7, rather than sorting ties.
            farthest = np.float32(-1)
            for i in cost_order:
                if not used[i] and closest[i] > np.float32(farthest + np.float32(1e-7)):
                    farthest = closest[i]; current = int(i)
        used[current] = True
        yield current


def replay(features, costs, labels, work, scores, mode, count, work_cap):
    result = dict(verified=False, checked=0, consumed_steps=0, truncated=False,
                  selected_index=-1, selected_cost=None, regret=None)
    if not len(costs):
        return result
    cheapest = int(np.argsort(costs, kind='stable')[0])
    positive = np.flatnonzero(labels)
    oracle_cost = float(np.min(costs[positive])) if len(positive) else None
    for i in candidate_schedule(features, costs, scores, mode, count):
        remaining = work_cap - result['consumed_steps'] if work_cap else None
        if remaining == 0:
            break
        result['checked'] += 1
        needed = int(work[i])
        if remaining is not None and needed > remaining:
            result['consumed_steps'] += remaining
            result['truncated'] = True
            break
        result['consumed_steps'] += needed
        if not labels[i]:
            continue
        if result['selected_cost'] is None or costs[i] < result['selected_cost']:
            result['selected_cost'] = float(costs[i]); result['selected_index'] = int(i)
        result['verified'] = True
        if mode == 'cost' or i == cheapest:
            break
    if result['verified']:
        result['regret'] = max(0.0, result['selected_cost'] - oracle_cost)
    return result


def prepare_states(data, path):
    with open(path) as stream:
        rows = list(csv.DictReader(stream))
    assert rows and all(row['scene'].startswith('val_') for row in rows), 'Only validation maps may select schedules'
    identifiers = [int(row['state_id']) for row in rows]
    assert len(set(identifiers)) == len(rows), 'Duplicate state identifiers'
    assert set(np.unique(data['state'])).issubset(identifiers), 'Candidate state missing from summaries'
    assert np.isfinite(data['x']).all() and np.isfinite(data['cost']).all()
    assert set(np.unique(data['y'])).issubset({0, 1}) and (data['work'] >= 0).all()
    groups = []
    for row in rows:
        indices = np.flatnonzero(data['state'] == int(row['state_id']))
        assert len(indices) == int(row['feasible_count']), 'Candidate count mismatch'
        assert int(data['y'][indices].sum()) == int(row['verified_count']), 'Oracle count mismatch'
        if len(indices):
            assert set(data['scene'][indices]) == {row['scene']}
            assert set(data['family'][indices]) == {row['family']}
            assert len(set(data['candidate'][indices])) == len(indices)
        groups.append((row, indices))
    return groups


def summarize(rows):
    positive = [row for row in rows if row['oracle_positive']]
    verified = [row for row in positive if row['verified']]
    regrets = [row['regret'] for row in verified]
    return dict(states=len(rows), oracle_positive=len(positive), verified=len(verified),
                recall=float(len(verified) / len(positive)) if positive else None,
                mean_work=float(np.mean([row['consumed_steps'] for row in rows])),
                mean_checks=float(np.mean([row['checked'] for row in rows])),
                mean_regret=float(np.mean(regrets)) if regrets else None,
                p95_regret=float(np.percentile(regrets, 95)) if regrets else None,
                truncations=sum(row['truncated'] for row in rows))


def evaluate_setting(data, groups, output, model, mode, exponent, budget, count, work_cap):
    scores = utility(output, exponent) if output is not None else np.zeros(len(data['y']), dtype=np.float32)
    rows = []
    for metadata, indices in groups:
        result = replay(data['x'][indices], data['cost'][indices], data['y'][indices],
                        data['work'][indices], scores[indices], mode, count, work_cap)
        row = dict(model=model, mode=mode, exponent=exponent, budget=budget,
                   state_id=int(metadata['state_id']), scene=metadata['scene'], family=metadata['family'],
                   oracle_positive=int(metadata['verified_count']) > 0, **result)
        rows.append(row)
    return rows


def run(args):
    data = load_validation(args.val)
    groups = prepare_states(data, args.states)
    report = dict(protocol='validation_schedule_v1', generated_utc=datetime.now(timezone.utc).isoformat(),
                  selection_rule='Shared exponent maximizes mean validation recall across geometry learned/hybrid x K12/W3600; exact ties choose smaller exponent.',
                  exponents=list(EXPONENTS), candidate_data_sha256=str(data['source_sha256']),
                  state_data_sha256=sha(args.states), script_sha256=sha(__file__),
                  validation_maps=len({row['scene'] for row, _ in groups}),
                  validation_states=len(groups), models={}, settings=[])
    all_rows = []
    for model, directory in [('geometry', args.geometry), ('state_only', args.state_only)]:
        if directory is None:
            continue
        directory = Path(directory)
        training = json.loads((directory / 'training.json').read_text())
        model_digest = sha(directory / 'network.txt')
        assert training['val_sha256'] == str(data['source_sha256']), 'Prediction/data provenance mismatch'
        assert training['model_sha256'] == model_digest, 'Export changed after training report'
        output = np.load(directory / 'validation_predictions.npz', allow_pickle=False)['output']
        assert output.shape == (len(data['y']), 2) and np.isfinite(output).all()
        report['models'][model] = dict(model_sha256=model_digest,
                                      predictions_sha256=sha(directory / 'validation_predictions.npz'),
                                      training_sha256=sha(directory / 'training.json'))
        for exponent in EXPONENTS:
            for mode in ['learned', 'learned_diverse']:
                for budget, count, cap in BUDGETS:
                    rows = evaluate_setting(data, groups, output, model, mode, exponent, budget, count, cap)
                    all_rows.extend(rows)
                    report['settings'].append(dict(model=model, mode=mode, exponent=exponent, budget=budget,
                                                   **summarize(rows)))
    for mode in ['cost', 'diverse']:
        for budget, count, cap in BUDGETS:
            rows = evaluate_setting(data, groups, None, 'none', mode, None, budget, count, cap)
            all_rows.extend(rows)
            report['settings'].append(dict(model='none', mode=mode, exponent=None, budget=budget,
                                           **summarize(rows)))
    objectives = []
    for exponent in EXPONENTS:
        cells = [s for s in report['settings'] if s['model'] == 'geometry' and s['exponent'] == exponent]
        assert len(cells) == 4 and all(s['recall'] is not None for s in cells)
        objectives.append(dict(exponent=exponent, mean_recall=float(np.mean([s['recall'] for s in cells]))))
    chosen = max(objectives, key=lambda record: (record['mean_recall'], -record['exponent']))
    report['selection_objectives'] = objectives
    report['chosen_shared_work_power'] = chosen['exponent']
    report['chosen_mean_recall'] = chosen['mean_recall']
    report['replay_scope'] = 'Complete-query feasibility and integration counts; feature-based float32 reconstruction of maneuver distances. No test labels or closed-loop outcomes used.'
    output_path = Path(args.output); output_path.parent.mkdir(parents=True, exist_ok=True)
    replay_path = output_path.with_name(output_path.stem + '_states.csv')
    with replay_path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_rows[0])); writer.writeheader(); writer.writerows(all_rows)
    report['state_replay_sha256'] = sha(replay_path)
    output_path.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(chosen_shared_work_power=chosen['exponent'], objectives=objectives,
                          states=len(groups), maps=report['validation_maps'], report=str(output_path)), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--val', required=True, help='Validation candidates CSV')
    parser.add_argument('--states', required=True, help='Validation state summaries CSV')
    parser.add_argument('--geometry', required=True, help='Trained geometry model directory')
    parser.add_argument('--state-only', help='Trained geometry-masked model directory')
    parser.add_argument('--output', required=True, help='JSON selection report')
    run(parser.parse_args())
