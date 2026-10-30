#!/usr/bin/env python3
"""Controlled geometry x auxiliary-loss ablation on archived train/val maps.

All four cells retain the same 103-64-32-2 training architecture, initialization
and optimizer. Every checkpoint is selected by validation BCE. Export also
removes inactive geometry columns and the unused regression output for actual
online inference. This script never accepts or reads test data.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import torch
from torch import nn

import train_recovery_network as frozen_train
import select_recovery_schedule as frozen_schedule


SEEDS = (20261030, 20261031, 20261032)
CELLS = (('geometry_bce', False, 0.0), ('geometry_joint', False, 0.1),
         ('state_bce', True, 0.0), ('state_joint', True, 0.1))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def validate_npz(path, split):
    with np.load(path, allow_pickle=False) as stream:
        data = dict(stream)
    assert data['x'].ndim == 2 and data['x'].shape[1] == 103
    assert np.isfinite(data['x']).all() and np.isfinite(data['cost']).all()
    assert set(np.unique(data['y'])) == {0, 1}
    assert np.min(data['work']) >= 0
    assert all(str(s).startswith(split + '_') for s in set(data['scene']))
    return data


def metrics(output, data, auxiliary_weight):
    result = frozen_train.metrics(output, data)
    result['loss'] = result['bce'] + auxiliary_weight * result['huber']
    result['work_head_trained'] = auxiliary_weight > 0
    return result


def compact_export(model, path):
    inputs = 28 if model.state_only else 103
    arrays = []
    for index in (0, 2, 4):
        weight = model.layers[index].weight.detach().cpu().numpy().copy()
        bias = model.layers[index].bias.detach().cpu().numpy().copy()
        if index == 0:
            weight = weight[:, :inputs]
        if index == 4:
            weight, bias = weight[:1], bias[:1]
        arrays.extend((weight.reshape(-1), bias))
    with Path(path).open('w') as stream:
        stream.write('RECOVERY_MLP_V2 %d 64 32 1\n' % inputs)
        for array in arrays:
            stream.write(' '.join('%.9g' % value for value in array) + '\n')
    return inputs, int(sum(len(array) for array in arrays))


def export_parity(model, validation, output, directory):
    inputs, parameters = compact_export(model, directory / 'network_compact.txt')
    frozen_train.export(model, directory / 'network_v1.txt')
    indices = np.linspace(0, len(validation['x']) - 1, 64, dtype=int)
    with (directory / 'compact_parity.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['f%d' % i for i in range(inputs)] + ['logit'])
        writer.writerows(np.c_[validation['x'][indices, :inputs], output[indices, 0]])
    with (directory / 'v1_parity.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['f%d' % i for i in range(103)] + ['logit', 'log_work'])
        writer.writerows(np.c_[validation['x'][indices], output[indices]])
    return inputs, parameters


def train_cell(name, state_only, auxiliary_weight, seed, train, validation,
               groups, args, protocol_digest):
    directory = args.output / name / str(seed)
    directory.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(seed)
    np.random.seed(seed)
    model = frozen_train.Network(state_only)
    optimizer = torch.optim.Adam(model.parameters(), lr=.001, weight_decay=1e-5)
    x, y = torch.from_numpy(train['x']), torch.from_numpy(train['y'])
    logwork = torch.from_numpy(np.log1p(train['work']).astype(np.float32))
    history, best, best_epoch = [], float('inf'), -1
    start = time.monotonic()
    for epoch in range(args.epochs + 1):
        if epoch:
            model.train()
            permutation = torch.randperm(len(x))
            totals = np.zeros(3)
            for indices in permutation.split(args.batch_size):
                optimizer.zero_grad()
                output = model(x[indices])
                bce = nn.functional.binary_cross_entropy_with_logits(output[:, 0], y[indices])
                huber = nn.functional.smooth_l1_loss(output[:, 1], logwork[indices])
                loss = bce + auxiliary_weight * huber
                loss.backward()
                optimizer.step()
                totals += np.array([loss.item(), bce.item(), huber.item()]) * len(indices)
            tm = dict(zip(('loss', 'bce', 'huber'), (totals / len(x)).tolist()))
        else:
            tm = metrics(frozen_train.predict(model, train['x']), train, auxiliary_weight)
        output = frozen_train.predict(model, validation['x'])
        vm = metrics(output, validation, auxiliary_weight)
        row = dict(epoch=epoch, train=tm, validation=vm,
                   elapsed_s=time.monotonic() - start)
        history.append(row)
        if vm['bce'] < best:
            best, best_epoch = vm['bce'], epoch
            torch.save(model.state_dict(), directory / 'checkpoint.pt')
        if epoch % 10 == 0 or epoch == args.epochs:
            print(json.dumps(dict(cell=name, seed=seed, epoch=epoch,
                                  validation_bce=vm['bce'], elapsed_s=row['elapsed_s'])), flush=True)
    model.load_state_dict(torch.load(directory / 'checkpoint.pt', weights_only=True))
    output = frozen_train.predict(model, validation['x'])
    np.savez_compressed(directory / 'validation_predictions.npz', output=output)
    inputs, parameters = export_parity(model, validation, output, directory)
    settings, replay_rows = [], []
    for budget, count, cap in frozen_schedule.BUDGETS:
        rows = frozen_schedule.evaluate_setting(validation, groups, output,
                name, 'learned', 0., budget, count, cap)
        replay_rows.extend(rows)
        settings.append(dict(budget=budget, **frozen_schedule.summarize(rows)))
    with (directory / 'validation_schedule_states.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(replay_rows[0]))
        writer.writeheader()
        writer.writerows(replay_rows)
    report = dict(cell=name, seed=seed, state_only=state_only,
                  auxiliary_weight=auxiliary_weight, architecture_training=[103, 64, 32, 2],
                  architecture_inference=[inputs, 64, 32, 1],
                  parameters_training=sum(p.numel() for p in model.parameters()),
                  parameters_inference=parameters, checkpoint_criterion='validation BCE',
                  best_epoch=best_epoch, epochs=args.epochs, batch_size=args.batch_size,
                  training_seconds=time.monotonic() - start,
                  validation_metrics=metrics(output, validation, auxiliary_weight),
                  validation_schedules=settings,
                  selection_score=float(np.mean([setting['recall'] for setting in settings])),
                  training_samples=len(x), validation_samples=len(validation['y']),
                  training_maps=len(set(train['scene'])), validation_maps=len(set(validation['scene'])),
                  protocol_sha256=protocol_digest, script_sha256=sha(__file__),
                  torch_version=torch.__version__, numpy_version=np.__version__,
                  model_v1_sha256=sha(directory / 'network_v1.txt'),
                  model_compact_sha256=sha(directory / 'network_compact.txt'),
                  history=history)
    write_json(directory / 'training.json', report)
    print(json.dumps(dict(completed=name, seed=seed, best_epoch=best_epoch,
                          selection_score=report['selection_score'],
                          training_seconds=report['training_seconds'])), flush=True)
    return report


def run(args):
    torch.set_num_threads(args.threads)
    torch.use_deterministic_algorithms(True)
    args.output.mkdir(parents=True, exist_ok=True)
    train = validate_npz(args.train, 'train')
    validation = validate_npz(args.val, 'val')
    assert not set(train['scene']) & set(validation['scene'])
    groups = frozen_schedule.prepare_states(validation, args.states)
    protocol = dict(protocol='review_loss_ablation_v1',
                    created_utc=datetime.now(timezone.utc).isoformat(),
                    cells=[dict(name=n, state_only=s, auxiliary_weight=w) for n,s,w in CELLS],
                    seeds=list(SEEDS), epochs=args.epochs, batch_size=args.batch_size,
                    optimizer='Adam lr=0.001 weight_decay=0.00001',
                    checkpoint_rule='Minimum validation BCE for every cell; exact ties retain earliest epoch.',
                    priority='Feasibility only (beta=0); common cheapest feasible primary is queried first.',
                    configuration_rule='Maximize three-seed mean of validation K12 and W3600 recall; exact ties minimize three-seed mean validation BCE, then lexical cell name.',
                    deployment_seed=SEEDS[0],
                    heldout_policy='No test arrays, maps, closed-loop outcomes or test-derived parameters are read by this training and selection script.',
                    inference_export='Remove work head; for state-only also remove 75 inactive geometry columns. No retraining after compact export.',
                    train_archive_sha256=sha(args.train), val_archive_sha256=sha(args.val),
                    validation_states_sha256=sha(args.states), script_sha256=sha(__file__),
                    helper_sha256={Path(module.__file__).name:sha(module.__file__)
                                   for module in [frozen_train, frozen_schedule]})
    protocol_path = args.output / 'PROTOCOL.json'
    if protocol_path.exists():
        existing = json.loads(protocol_path.read_text())
        protocol['created_utc'] = existing['created_utc']
        assert protocol == existing, 'Protocol changed after experiment began'
    else:
        write_json(protocol_path, protocol)
    reports = []
    for name, state_only, weight in CELLS:
        for seed in SEEDS:
            target = args.output / name / str(seed) / 'training.json'
            if target.exists():
                report = json.loads(target.read_text())
                assert report['protocol_sha256'] == sha(protocol_path)
                assert report['model_compact_sha256'] == sha(target.parent/'network_compact.txt')
            else:
                report = train_cell(name, state_only, weight, seed, train, validation,
                                    groups, args, sha(protocol_path))
            reports.append(report)
    summary = []
    for name, state_only, weight in CELLS:
        rows = [r for r in reports if r['cell'] == name]
        record = dict(cell=name, state_only=state_only, auxiliary_weight=weight,
                      mean_validation_recall=float(np.mean([r['selection_score'] for r in rows])),
                      mean_validation_bce=float(np.mean([r['validation_metrics']['bce'] for r in rows])),
                      mean_validation_auroc=float(np.mean([r['validation_metrics']['auroc'] for r in rows])))
        for budget in ('K12', 'W3600'):
            values = [next(s['recall'] for s in r['validation_schedules'] if s['budget']==budget) for r in rows]
            record[budget] = dict(values=values, mean=float(np.mean(values)), std=float(np.std(values, ddof=1)))
        summary.append(record)
    selected = sorted(summary, key=lambda r:(-r['mean_validation_recall'], r['mean_validation_bce'],r['cell']))[0]
    chosen = next(r for r in reports if r['cell']==selected['cell'] and r['seed']==SEEDS[0])
    directory = args.output / chosen['cell'] / str(chosen['seed'])
    freeze = dict(frozen_utc=datetime.now(timezone.utc).isoformat(),
                  protocol_sha256=sha(protocol_path), configuration_summaries=summary,
                  selected_cell=chosen['cell'], selected_seed=chosen['seed'],
                  selection_score_across_seeds=selected['mean_validation_recall'],
                  network_v1=str(directory/'network_v1.txt'),
                  network_compact=str(directory/'network_compact.txt'),
                  model_v1_sha256=chosen['model_v1_sha256'],
                  model_compact_sha256=chosen['model_compact_sha256'],
                  inference_architecture=chosen['architecture_inference'],
                  checkpoint_epoch=chosen['best_epoch'],
                  test_results_used=False)
    write_json(args.output / 'SELECTION_FREEZE.json', freeze)
    write_json(args.output / 'summary.json', dict(protocol_sha256=sha(protocol_path), cells=summary,
               runs=[{key:r[key] for key in ['cell','seed','best_epoch','validation_metrics',
                     'validation_schedules','selection_score','training_seconds','model_compact_sha256']}
                     for r in reports]))
    print(json.dumps(freeze, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train', type=Path, required=True)
    parser.add_argument('--val', type=Path, required=True)
    parser.add_argument('--states', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--batch-size', type=int, default=2048)
    parser.add_argument('--threads', type=int, default=2)
    run(parser.parse_args())
