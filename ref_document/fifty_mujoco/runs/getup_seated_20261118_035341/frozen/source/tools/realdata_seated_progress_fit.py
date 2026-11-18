#!/usr/bin/env python3
"""Learn when to advance the rise from 31D observations of completed teacher runs."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import itertools
import json
from pathlib import Path

import numpy as np
import torch

from realdata_parallel_train import getup_evaluate
from realdata_seated_state_teacher import load_policy
from realdata_train import SamplePolicy31, digest, save_checkpoint, verified_artifact

# Leave previous actions out of the regressions. They identify the current stage.
FEATURES = [0, 1, 2, 3, 4, 5, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 25, 26, 29, 30]


class ProgressActor(torch.nn.Module):
    def __init__(self, tracking, motion, center, scale, coefficients, thresholds):
        super().__init__()
        self.tracking = tracking
        for name, value in zip(('motion', 'center', 'scale', 'coefficients', 'thresholds'),
                               (motion, center, scale, coefficients, thresholds)):
            self.register_buffer(name, torch.as_tensor(value, dtype=torch.float32))
        self.register_buffer('features', torch.tensor(FEATURES))

    def forward(self, observation):
        last = observation[:, 19:25]
        error = (last[:, None]-self.motion).abs().amax(-1)
        distance, phase = error.min(dim=-1)
        cold = last.abs().amax(-1) < 1e-6
        state = observation.index_select(1, self.features)
        value = ((state-self.center[phase])/self.scale[phase]*self.coefficients[phase, :-1]).sum(-1)
        value = value+self.coefficients[phase, -1]
        advance = value >= self.thresholds[phase]
        next_phase = phase+advance.to(dtype=torch.long)
        next_phase = torch.where(cold, 0, next_phase)
        complete = ((distance > .015) | (next_phase >= len(self.motion))) & ~cold
        proposed = self.motion[next_phase.clamp(max=len(self.motion)-1)]
        return torch.where(complete[:, None], self.tracking(observation), proposed)


def fit(trajectories, stages, ticks, penalty):
    centers, scales, weights = [], [], []
    for i in range(stages):
        feature = np.concatenate([t[i*ticks:(i+1)*ticks, FEATURES] for t in trajectories])
        y = np.tile((np.arange(ticks)+1)/ticks, len(trajectories))
        center = feature.mean(0)
        scale = np.maximum(feature.std(0), .0005)
        z = np.column_stack(((feature-center)/scale, np.ones(len(feature))))
        regularizer = np.eye(z.shape[1])*penalty*len(feature)
        regularizer[-1, -1] = 0.
        weights.append(np.linalg.solve(z.T@z+regularizer, z.T@y).tolist())
        centers.append(center.tolist())
        scales.append(scale.tolist())
    return centers, scales, weights


def evaluate(job):
    profile, tracking, balance, model, thresholds, seed, count, domain = job
    torch.set_num_threads(1)
    actor = SamplePolicy31(verified_artifact('model_3050.pt'), 'cpu')
    actor.actor = ProgressActor(load_policy(tracking, 'cpu'), model['motion'], model['center'],
                                model['scale'], model['coefficients'], thresholds)
    settings = {'task': 'getup', 'balance_checkpoint': str(balance), 'config': profile,
                'reset_range': [0., 0.], 'settle_seconds': 1., 'domain': domain}
    result = getup_evaluate(actor, settings, count=count, seed_base=seed)
    return {'thresholds': thresholds, 'penalty': model['penalty'], 'passed': result['passed'], 'trials': count,
            'balance_ticks': [r['balance_ticks'] for r in result['rows']],
            'results': result['rows']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replays', type=Path, required=True)
    parser.add_argument('--motion', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--tracking', type=Path, required=True)
    parser.add_argument('--balance', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--candidate-index', type=int, default=2)
    parser.add_argument('--workers', type=int, default=12)
    parser.add_argument('--seed-base', type=int, default=910850)
    parser.add_argument('--count', type=int, default=1)
    args = parser.parse_args()
    profile = json.loads(args.config.read_text())
    motion_spec = json.loads(args.motion.read_text())
    motion = [[h,k,2*w,-h,-k,2*w] for h,k,w in motion_spec['candidates'][args.candidate_index]]
    ticks = round(motion_spec['segment_seconds']/.01)
    report = json.loads((args.replays/'report.json').read_text())
    low, high = profile['seed_ranges']['train']
    sources = [r for r in report['rows'] if r['candidate'] == args.candidate_index and r['success']]
    if not sources or not all(low <= r['seed'] <= high for r in sources):
        raise ValueError('expected successful trajectories in training split')
    data = []
    for row in sources:
        with np.load(args.replays/row['trace']) as replay:
            data.append(replay['observation'][:ticks*len(motion)])
    models = {}
    for penalty in (.001, .01, .05):
        center, scale, coefficients = fit(data, len(motion), ticks, penalty)
        models[penalty] = {'motion': motion, 'center': center, 'scale': scale,
                           'coefficients': coefficients, 'penalty': penalty}
    grids = list(itertools.product([.55, .68, .78], [.76, .9, 1.04], [.85, .98], [.85, .98], [.83, .97]))
    args.out.mkdir(parents=True, exist_ok=False)
    tasks = [(profile, args.tracking, args.balance, model, t, args.seed_base, args.count, False)
             for model in models.values() for t in grids]
    rows = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool, (args.out/'rows.jsonl').open('w') as stream:
        for row in pool.map(evaluate, tasks):
            rows.append(row)
            stream.write(json.dumps(row)+'\n')
            stream.flush()
            print(json.dumps({key: value for key, value in row.items() if key != 'results'}), flush=True)
    rows.sort(key=lambda r: (r['passed'], sum(r['balance_ticks'])), reverse=True)
    (args.out/'report.json').write_text(json.dumps({'rows': rows, 'models': models, 'training_seeds':
             [row['seed'] for row in sources], 'configuration': profile}, indent=2)+'\n')
    for i, row in enumerate(rows[:12]):
        model = models[row['penalty']]
        policy = ProgressActor(load_policy(args.tracking, 'cpu'), model['motion'], model['center'], model['scale'],
                               model['coefficients'], row['thresholds'])
        save_checkpoint(args.out/f'candidate_{i:02d}.pt', {'schema': 'seated_observation_progress_v1',
            'model':policy.state_dict(),'architecture':model, 'thresholds':row['thresholds'], 'configuration':profile,
            'balance_sha256':digest(args.balance), 'tracking_sha256':digest(args.tracking),
            'development':{key:value for key,value in row.items() if key != 'results'}})


if __name__ == '__main__':
    main()
