#!/usr/bin/env python3
"""CPU task authority for GPU/CPU candidate checkpoints; explicit seed split."""
import argparse
import json
from pathlib import Path

import torch

from realdata_train import RealDataHardwareEnv31,evaluate,digest,evaluation_policy
from realdata_parallel_train import getup_evaluate


def main():
    p=argparse.ArgumentParser();p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--config',type=Path)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--task',choices=['balance','getup'],default='balance')
    p.add_argument('--balance-checkpoint',type=Path);p.add_argument('--reset-range',nargs=2,type=float,default=[0,0])
    p.add_argument('--settle-seconds',type=float,default=0.);p.add_argument('--domain',action='store_true')
    p.add_argument('--seeds',type=int,default=4);p.add_argument('--seed-base',type=int,default=613000)
    p.add_argument('--seconds',type=float,default=20.);p.add_argument('--split',choices=['development','heldout'],default='development')
    args=p.parse_args();torch.set_num_threads(1)
    saved=torch.load(args.checkpoint,map_location='cpu',weights_only=True)
    config=json.loads(args.config.read_text()) if args.config else saved.get('configuration',saved.get('contract',{}).get('configuration'))
    if not config:raise ValueError('candidate configuration required')
    defaults = {'development': [610000, 619999], 'heldout': [710000, 719999]}
    low, high = config.get('seed_ranges', defaults)[args.split]
    if args.seeds < 1 or not low <= args.seed_base <= high - args.seeds + 1:
        p.error(f'{args.split} seed range is {low}..{high}')
    policy=evaluation_policy(saved)
    if args.task=='balance':
        import realdata_train
        original=realdata_train.RealDataHardwareEnv31
        realdata_train.RealDataHardwareEnv31=lambda **kwargs:original(config=config,**kwargs)
        result=evaluate(policy,count=args.seeds,seconds=args.seconds,seed_base=args.seed_base)
    else:
        if not args.balance_checkpoint:p.error('getup requires frozen balance')
        settings={'task':'getup','balance_checkpoint':str(args.balance_checkpoint),'reset_range':args.reset_range,
                  'settle_seconds':args.settle_seconds,'domain':args.domain,'config':config}
        result=getup_evaluate(policy,settings,count=args.seeds,seed_base=args.seed_base)
    result.update(checkpoint=str(args.checkpoint),checkpoint_sha256=digest(args.checkpoint),split=args.split,
                  configuration=config,backend='CPU MuJoCo + actual C qualified odometry',evaluator_sha256=digest(__file__))
    args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
