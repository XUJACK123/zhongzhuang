#!/usr/bin/env python3
"""Evaluate the learned lift and tracking gate in closed-loop CPU MuJoCo."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import itertools
import json
from pathlib import Path

import torch

from realdata_parallel_train import getup_evaluate
from realdata_seated_state_teacher import GatedActor, load_policy
from realdata_train import SamplePolicy31, digest, verified_artifact


def evaluate(task):
    lift, tracking, balance, cfg, gates, seeds, seed_base, randomize = task
    torch.set_num_threads(1)
    policy = SamplePolicy31(verified_artifact('model_3050.pt'), 'cpu')
    policy.actor = GatedActor(load_policy(lift, 'cpu'), load_policy(tracking, 'cpu'), gates)
    settings = {'task': 'getup', 'balance_checkpoint': str(balance), 'config': cfg,
                'reset_range': [0., 0.], 'settle_seconds': 1., 'domain': randomize}
    result = getup_evaluate(policy, settings, count=seeds, seed_base=seed_base)
    return {'gates': gates, 'lift': str(lift), 'trials': seeds, 'passed': result['passed'],
            'max_balance_ticks': max(x['balance_ticks'] for x in result['rows']),
            'findings': result['rows']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lift', type=Path, nargs='+', required=True)
    parser.add_argument('--tracking', type=Path, required=True)
    parser.add_argument('--balance', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seeds', type=int, default=1)
    parser.add_argument('--seed-base', type=int, default=910720)
    parser.add_argument('--workers', type=int, default=12)
    parser.add_argument('--domain', action='store_true')
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    grid = list(itertools.product([.18, .26, .34, .42, .50], [-.40, -.20, .0, .20], [.28, .48]))
    tasks = [(lift, args.tracking, args.balance, cfg, gate, args.seeds, args.seed_base, args.domain)
             for lift in args.lift for gate in grid]
    report = []
    args.out.mkdir(parents=True, exist_ok=False)
    with ProcessPoolExecutor(max_workers=args.workers) as pool, (args.out/'rows.jsonl').open('w') as stream:
        for result in pool.map(evaluate, tasks):
            report.append(result)
            stream.write(json.dumps(result)+'\n')
            stream.flush()
            print(json.dumps({k: v for k, v in result.items() if k != 'findings'}), flush=True)
    report.sort(key=lambda row: (row['passed'], row['max_balance_ticks']), reverse=True)
    metadata = {'rows': report, 'configuration': cfg, 'tracking_sha256': digest(args.tracking),
                'balance_sha256': digest(args.balance), 'lift_sha256': {str(p): digest(p) for p in args.lift}}
    (args.out/'report.json').write_text(json.dumps(metadata, indent=2)+'\n')


if __name__ == '__main__':
    main()
