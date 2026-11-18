#!/usr/bin/env python3
"""Fit the existing 31D get-up policy to successful CPU teacher replays."""

import argparse
import json
from pathlib import Path

import numpy as np
import torch

from realdata_train import SamplePolicy31, digest, save_checkpoint, verified_artifact


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replays', type=Path, nargs='+', required=True)
    parser.add_argument('--init', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=810600)
    parser.add_argument('--steps', type=int, default=2000)
    parser.add_argument('--device', default='cuda')
    parser.add_argument('--lr', type=float, default=.001)
    parser.add_argument('--decay', action='store_true')
    parser.add_argument('--minimum-balance-ticks', type=int, default=1000,
                        help='include partial teacher rollouts for an explicitly tagged curriculum warm start')
    parser.add_argument('--candidate-index', type=int, action='append',
                        help='fit one motion family when multiple successful teachers create conflicting labels')
    args = parser.parse_args()
    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    sources = [directory/'report.json' for directory in args.replays]
    reports = [json.loads(source.read_text()) for source in sources]
    cfg = reports[0]['configuration']
    if any(report['configuration'] != cfg for report in reports[1:]):
        raise ValueError('demonstration configurations differ')
    low, high = cfg['seed_ranges']['train']
    traces = [(directory, row) for directory, report in zip(args.replays, reports)
              for row in report['rows'] if row['balance_ticks'] >= args.minimum_balance_ticks]
    if args.candidate_index:
        traces = [(directory, row) for directory, row in traces if row['candidate'] in args.candidate_index]
    if not traces or not all(low <= row['seed'] <= high for _, row in traces):
        raise ValueError('eligible teacher replays must come from the training seed split')
    obs = []
    actions = []
    for directory, row in traces:
        with np.load(directory/row['trace']) as trace:
            obs.append(torch.from_numpy(trace['observation']))
            actions.append(torch.from_numpy(trace['action']).float())
    x = torch.cat(obs).to(args.device)
    y = torch.cat(actions).to(args.device)
    bound = y.new_tensor([1, 1, 2, 1, 1, 2])
    y = y.clamp(-bound, bound)
    policy = SamplePolicy31(verified_artifact('model_3050.pt'), args.device)
    initial = torch.load(args.init, map_location=args.device, weights_only=True)
    policy.actor.load_state_dict({key.removeprefix('actor.'): value for key, value in initial['model'].items()
                                  if key.startswith('actor.')})
    optimizer = torch.optim.Adam(policy.actor.parameters(), lr=args.lr)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, args.steps, eta_min=.05*args.lr) if args.decay else None
    args.out.mkdir(parents=True, exist_ok=False)
    report_rows = []
    for step in range(args.steps):
        index = torch.randint(len(x), (512,), device=args.device)
        prediction = policy.actor(x[index])
        loss = ((prediction-y[index])/bound).square().mean()
        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(policy.actor.parameters(), 1., error_if_nonfinite=True)
        optimizer.step()
        if scheduler:
            scheduler.step()
        if (step+1)%250 == 0:
            with torch.no_grad():
                rmse = ((policy.actor(x)-y).square().mean(0).sqrt()).cpu().tolist()
            row = {'step': step+1, 'loss': float(loss.detach()), 'train_rmse': rmse}
            report_rows.append(row)
            print(json.dumps(row), flush=True)
            save_checkpoint(args.out/f'fit_{step+1:05d}.pt',
                            {'schema': 'seated_getup_cpu_demonstration_fit_v1', 'configuration': cfg,
                             'teacher_report_sha256': [digest(source) for source in sources],
                             'training_seeds': [row['seed'] for _, row in traces],
                             'minimum_teacher_balance_ticks': args.minimum_balance_ticks, 'model': policy.state_dict()})
    (args.out/'report.json').write_text(json.dumps({'training_samples': len(x), 'traces': len(traces),
                                                        'minimum_teacher_balance_ticks': args.minimum_balance_ticks,
                                                        'progress': report_rows}, indent=2)+'\n')


if __name__ == '__main__':
    main()
