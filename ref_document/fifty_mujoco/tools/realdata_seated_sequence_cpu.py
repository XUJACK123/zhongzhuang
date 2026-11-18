#!/usr/bin/env python3
"""Fine-tune the state-triggered rise on paired CPU physics trials."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import time

import numpy as np
import torch

from realdata_parallel_train import getup_evaluate
from realdata_train import digest, evaluation_policy, save_checkpoint


def unpack(vector):
    compact = np.asarray(vector[:15]).reshape(5, 3)
    h, k, w = compact.T
    return np.column_stack([h,k,w,-h,-k,w]).tolist(), list(vector[15:])


def candidate(state, vector, profile):
    motion, thresholds = unpack(vector)
    model = dict(state['model'])
    model['motions'] = torch.tensor(motion,dtype=torch.float32)
    model['thresholds'] = torch.tensor(thresholds,dtype=torch.float32)
    return dict(state, model=model, motion=motion, thresholds=thresholds, configuration=profile)


def evaluate(job):
    vector, parent, balance, profile, seed, count = job
    torch.set_num_threads(1)
    state = candidate(torch.load(parent,map_location='cpu',weights_only=True), vector, profile)
    policy = evaluation_policy(state)
    settings = {'task':'getup','balance_checkpoint':str(balance),'config':profile,'reset_range':[0.,0.],
                'settle_seconds':1.,'domain':True}
    data = getup_evaluate(policy,settings,count=count,seed_base=seed)
    rows = data['rows']
    perturbed = [r for r in rows if r['episode_parameters']['motor_gain'] != [1.]*6]
    perturbed_wins = sum(r['success'] for r in perturbed)
    score = 100*data['passed']+120*perturbed_wins+sum(r['balance_ticks']*.005 for r in rows)/count
    return {'parameters':vector,'score':score,'successes':data['passed'],'trials':count,
            'perturbed_successes':perturbed_wins,'perturbed_trials':len(perturbed),'rows':rows}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('init','balance','config','out'):
        p.add_argument(f'--{name}',type=Path,required=True)
    p.add_argument('--seed',type=int,default=812600)
    p.add_argument('--workers',type=int,default=12)
    p.add_argument('--population',type=int,default=20)
    p.add_argument('--replicates',type=int,default=6)
    p.add_argument('--iterations',type=int,default=10)
    p.add_argument('--seconds',type=float,default=1800)
    args = p.parse_args()
    torch.set_num_threads(1)
    profile = json.loads(args.config.read_text())
    parent = torch.load(args.init,map_location='cpu',weights_only=True)
    if parent['schema'] != 'seated_contact_sequence_v1':
        p.error('expected ContactSequence checkpoint')
    low_seed, high_seed = profile['seed_ranges']['train']
    if args.seed < low_seed or args.seed + args.iterations*args.replicates - 1 > high_seed:
        p.error('training seed range')
    rng = np.random.default_rng(args.seed)
    center = np.array([v for motion in parent['motion'] for v in motion[:3]] + parent['thresholds'])
    spread = np.array([.025,.035,.07]*5 + [.002,.08,.014,.017,.017,.012])
    minimum_spread = spread*.21
    lower = np.array([-1.,-1.,-2.]*5 + [-.04,.25,.32,.22,.40,-.10])
    upper = np.array([1.,1.,2.]*5 + [-.0005,1.95,.61,.56,.64,.20])
    elite = center[None].copy()
    unmodified = center.copy()
    start = time.monotonic()
    args.out.mkdir(exist_ok=False,parents=True)
    with ProcessPoolExecutor(max_workers=args.workers) as pool, (args.out/'rows.jsonl').open('w') as stream:
        for iteration in range(1,args.iterations+1):
            if time.monotonic()-start >= args.seconds:
                break
            group = np.clip(center+spread*rng.normal(size=(args.population,len(center))), lower, upper)
            group[:len(elite)] = elite
            group[-1] = unmodified
            seed = args.seed+(iteration-1)*args.replicates
            jobs = [(v.tolist(),args.init,args.balance,profile,seed,args.replicates) for v in group]
            results = list(pool.map(evaluate,jobs))
            ranked = sorted(results,key=lambda item:item['score'],reverse=True)
            elite = np.array([row['parameters'] for row in ranked[:max(3,args.population//5)]])
            center = .5*center+.5*elite.mean(0)
            spread = np.maximum(minimum_spread,.6*spread+.4*elite.std(0))
            best = ranked[0]
            row = {'iteration':iteration,'trials':args.replicates,'best':best['successes'],
                    'perturbed_successes':best['perturbed_successes'],'perturbed_trials':best['perturbed_trials'],
                    'baseline':results[-1]['successes'],'baseline_perturbed':results[-1]['perturbed_successes'],
                    'seed_base':seed,'wall_s':time.monotonic()-start}
            stream.write(json.dumps(row)+'\n');stream.flush()
            print(json.dumps(row),flush=True)
            (args.out/f'iteration_{iteration:03d}.json').write_text(json.dumps(ranked,indent=2)+'\n')
            for rank,result in enumerate(ranked[:3]):
                state = candidate(parent,result['parameters'],profile)
                state['cpu_training'] = row
                state['source'] = {'initial_sha256':digest(args.init),'balance_sha256':digest(args.balance)}
                save_checkpoint(args.out/f'iteration_{iteration:03d}_rank{rank}.pt',state)


if __name__ == '__main__':
    main()
