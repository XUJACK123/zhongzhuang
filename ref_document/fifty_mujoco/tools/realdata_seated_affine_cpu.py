#!/usr/bin/env python3
"""Fit small output-layer corrections on authoritative CPU closed-loop trials."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import copy
import json
from pathlib import Path
import time

import numpy as np
import torch

from realdata_parallel_train import getup_evaluate
from realdata_train import digest, evaluation_policy, save_checkpoint


def adjusted(saved, params):
    hip, knee, wheel, hip_bias, knee_bias, wheel_bias = params
    scale = torch.tensor([hip, knee, wheel, hip, knee, wheel], dtype=torch.float32)
    bias = torch.tensor([hip_bias, knee_bias, wheel_bias, -hip_bias, -knee_bias, wheel_bias], dtype=torch.float32)
    model = copy.deepcopy(saved['model'])
    final_weight = next(k for k in reversed(model) if k.startswith('actor.') and k.endswith('.weight'))
    final_bias = final_weight.removesuffix('weight')+'bias'
    model[final_weight] = model[final_weight]*scale[:, None]
    model[final_bias] = model[final_bias]*scale+bias
    return model


def evaluate(job):
    params, parent, balance, cfg, seed, count, domain = job
    torch.set_num_threads(1)
    weights = torch.load(parent, map_location='cpu', weights_only=True)
    policy = evaluation_policy({'model':adjusted(weights, params)})
    settings = {'task':'getup','balance_checkpoint':str(balance),'config':cfg, 'reset_range':[0.,0.],
                'settle_seconds':1., 'domain':domain}
    result = getup_evaluate(policy, settings, count=count, seed_base=seed)
    rows = result['rows']
    score = 100*result['passed']+sum((y['balance_ticks'] or 0)*.006
                 + float(y.get('final_clearance_m') or 0.) for y in rows)/count
    return {'parameters':params, 'passed':result['passed'],'trials':count,'score':score,
            'balance_ticks':[y['balance_ticks'] for y in rows], 'rows':rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('init','balance','config','out'):
        parser.add_argument(f'--{name}',type=Path,required=True)
    parser.add_argument('--workers',type=int,default=12)
    parser.add_argument('--population',type=int,default=20)
    parser.add_argument('--replicates',type=int,default=6)
    parser.add_argument('--iterations',type=int,default=18)
    parser.add_argument('--seconds',type=float,default=2200.)
    parser.add_argument('--seed',type=int,default=811700)
    parser.add_argument('--domain',action='store_true')
    args=parser.parse_args()
    torch.set_num_threads(1)
    cfg=json.loads(args.config.read_text())
    low,high=cfg['seed_ranges']['train']
    if args.seed < low or args.seed+args.iterations*args.replicates > high:
        parser.error('search seed range outside the training split')
    args.out.mkdir(parents=True,exist_ok=False)
    rng=np.random.default_rng(args.seed)
    start=time.monotonic()
    parent=torch.load(args.init,map_location='cpu',weights_only=True)
    base=np.array([1.,1.,1.,0.,0.,0.])
    mean=base.copy()
    spread=np.array([.010,.012,.016,.007,.01,.015])
    minimum_spread=spread*.20
    elites=base[None].copy()
    summary=[]
    with ProcessPoolExecutor(max_workers=args.workers) as pool, (args.out/'rows.jsonl').open('w') as stream:
        for iteration in range(1,args.iterations+1):
            if time.monotonic()-start>=args.seconds:
                break
            parameters=np.clip(mean+rng.normal(size=(args.population,6))*spread,
                               [.85,.85,.85,-.10,-.12,-.15],[1.15,1.15,1.15,.10,.12,.15])
            parameters[:len(elites)]=elites
            parameters[-1]=base
            seed=args.seed+(iteration-1)*args.replicates
            jobs=[(p.tolist(),args.init,args.balance,cfg,seed,args.replicates,args.domain) for p in parameters]
            results=list(pool.map(evaluate,jobs))
            ranked=sorted(results,key=lambda x:x['score'],reverse=True)
            winners=ranked[:max(3,args.population//5)]
            elites=np.array([x['parameters'] for x in winners])
            mean=.5*mean+.5*elites.mean(0)
            spread=np.maximum(minimum_spread,.6*spread+.4*elites.std(0))
            row={'iteration':iteration,'best':winners[0]['passed'],'baseline':results[-1]['passed'],
                 'replicates':args.replicates,'seed_base':seed,'wall_s':time.monotonic()-start}
            summary.append(row)
            stream.write(json.dumps(row)+'\n');stream.flush()
            print(json.dumps(row),flush=True)
            (args.out/f'iteration_{iteration:03d}.json').write_text(json.dumps(ranked,indent=2)+'\n')
            for rank,result in enumerate(winners[:3]):
                params=result['parameters']
                save_checkpoint(args.out/f'iteration_{iteration:03d}_rank{rank}.pt', {
                    'schema':'seated_cpu_affine_refinement_v1','model':adjusted(parent,params),'configuration':cfg,
                    'initial_sha256':digest(args.init),'balance_sha256':digest(args.balance),'training':row,'parameters':params})
    (args.out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')


if __name__=='__main__':
    main()
