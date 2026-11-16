#!/usr/bin/env python3
"""Collect observed teacher trajectories for distillation, never acceptance."""
import argparse
import json
from pathlib import Path
import torch

from realdata_train import SamplePolicy31,verified_artifact,digest
from realdata_allocator import AllocatorActor
from fifty_mujoco.envs.realdata_warp_getup import WarpSupportedGetup


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--balance',type=Path,required=True);ap.add_argument('--config',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--envs',type=int,default=256)
    ap.add_argument('--reset-range',nargs=2,type=float,default=[0,1]);args=ap.parse_args()
    torch.set_num_threads(1);torch.manual_seed(558000)
    cfg=json.loads(args.config.read_text());policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cuda')
    policy.load_state_dict(torch.load(args.balance,map_location='cuda',weights_only=True)['model'])
    actor=AllocatorActor(policy.actor,cfg)
    env=WarpSupportedGetup(policy.actor,args.envs,config=cfg,reset_range=args.reset_range,settle_seconds=1.)
    obs=[];targets=[];masks=[]
    with torch.no_grad():
        for i in range(2500):
            observation=env.observation.clone();action=actor(observation)
            if i%2==0:
                obs.append(observation);targets.append(action.clone());masks.append(((env.phase==1)|(env.phase==2)).clone())
            env.step(action)
            if i%25==0 and bool((env.phase>=4).all()):break
    mask=torch.stack(masks).flatten();observation=torch.stack(obs).reshape(-1,31)[mask]
    target=torch.stack(targets).reshape(-1,6)[mask]
    metadata={'schema':'observed_teacher_distillation_data_v1','seed':558000,'envs':args.envs,
              'reset_range':args.reset_range,'balance_sha256':digest(args.balance),'configuration':cfg,
              'valid_samples':observation.shape[0],'gpu_successes':int((env.phase==5).sum()),
              'scope':'training observations, no extra actor inputs; STOP/BALANCE excluded'}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    torch.save({'observation':observation.cpu(),'target':target.cpu(),'metadata':metadata},args.out)
    args.out.with_suffix('.json').write_text(json.dumps(metadata,indent=2));print(json.dumps(metadata),flush=True)


if __name__=='__main__':main()
