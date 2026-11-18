#!/usr/bin/env python3
"""Search feedback event thresholds for the seated maneuver on CPU."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import itertools
import json
from pathlib import Path

import torch

from realdata_parallel_train import getup_evaluate
from realdata_seated_state_teacher import ContactSequenceActor, load_policy
from realdata_train import SamplePolicy31, digest, verified_artifact


def evaluate(job):
    thresholds, cfg, motions, tracking, balance, seed, count, domain = job
    torch.set_num_threads(1)
    policy = SamplePolicy31(verified_artifact('model_3050.pt'), 'cpu')
    policy.actor = ContactSequenceActor(load_policy(tracking, 'cpu'), motions, thresholds)
    settings = {'task': 'getup', 'balance_checkpoint': str(balance), 'config': cfg,
                'reset_range': [0., 0.], 'settle_seconds': 1., 'domain': domain}
    result = getup_evaluate(policy, settings, count=count, seed_base=seed)
    return {'thresholds': thresholds, 'passed': result['passed'], 'trials': result['trials'],
            'max_balance_ticks': max(row['balance_ticks'] for row in result['rows']),
            'rows': result['rows']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--motion', type=Path, required=True)
    parser.add_argument('--tracking', type=Path, required=True)
    parser.add_argument('--balance', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=910810)
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--workers', type=int, default=12)
    parser.add_argument('--domain', action='store_true')
    parser.add_argument('--prior', type=Path)
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    motion_source = json.loads(args.motion.read_text())
    motions = [[h, k, 2*w, -h, -k, 2*w] for h, k, w in motion_source['candidates'][0]]
    if len(motions) != 5:
        parser.error('the controller requires five motion stages')
    grid = list(itertools.product([-.003, -.007, -.012], [.8, 1.25, 1.7], [.46, .50, .54],
                                  [.37, .43, .48], [.53, .56], [.01, .035]))
    if args.prior:
        grid = [row['thresholds'] for row in json.loads(args.prior.read_text())['rows'][:12]]
    jobs = [(p, cfg, motions, args.tracking, args.balance, args.seed, args.count, args.domain) for p in grid]
    rows = []
    args.out.mkdir(parents=True, exist_ok=False)
    with ProcessPoolExecutor(max_workers=args.workers) as pool, (args.out/'rows.jsonl').open('w') as log:
        for row in pool.map(evaluate, jobs):
            rows.append(row)
            log.write(json.dumps(row)+'\n')
            log.flush()
            print(json.dumps({k: v for k, v in row.items() if k != 'rows'}), flush=True)
    rows.sort(key=lambda row: (row['passed'], row['max_balance_ticks']), reverse=True)
    (args.out/'report.json').write_text(json.dumps({'motion': motions, 'rows': rows, 'configuration': cfg,
          'motion_sha256': digest(args.motion), 'tracking_sha256': digest(args.tracking),
          'balance_sha256': digest(args.balance)}, indent=2)+'\n')


if __name__ == '__main__':
    main()
