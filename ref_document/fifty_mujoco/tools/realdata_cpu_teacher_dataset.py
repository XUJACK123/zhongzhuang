#!/usr/bin/env python3
"""Development CPU demonstrations for fitting the observed motor allocator."""
import argparse
import json
from pathlib import Path
import numpy as np
import torch
from realdata_train import SamplePolicy31,verified_artifact,digest
from realdata_allocator import AllocatorActor
from fifty_mujoco.envs.sample_env31_supported import SupportedGetupEnv31
from fifty_mujoco.contracts.getup31 import Phase


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--balance',type=Path,required=True);ap.add_argument('--config',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--seeds',type=int,default=4)
    ap.add_argument('--seed-base',type=int,default=566800)
    args=ap.parse_args();torch.set_num_threads(1);cfg=json.loads(args.config.read_text())
    policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cpu')
    policy.load_state_dict(torch.load(args.balance,map_location='cpu',weights_only=True)['model'])
    teacher=AllocatorActor(policy.actor,cfg)
    env=SupportedGetupEnv31(policy.actor,reset_range=[1,1],settle_seconds=1.,config=cfg)
    observations=[];targets=[];rows=[];advance=env._advance
    def record(action,scale):
        if env.controller.phase in (Phase.GETUP,Phase.HANDOFF):
            observation=env.env.observation().copy()
            with torch.no_grad():target=teacher(torch.tensor(observation)[None])[0].numpy()
            observations.append(observation);targets.append(target)
        return advance(action,scale)
    env._advance=record
    for seed in range(args.seed_base,args.seed_base+args.seeds):
        obs=env.reset(seed)
        for _ in range(env.max_steps):
            with torch.no_grad():action=teacher(torch.tensor(obs)[None])[0].numpy()
            obs,_,done,truncated,info=env.step(action)
            if done or truncated:break
        row={k:info[k] for k in ['success','failure','phase_counts','balance_ticks']};row['seed']=seed
        rows.append(row);print(json.dumps(row),flush=True)
    metadata={'schema':'cpu_teacher_development_demonstrations_v1','configuration':cfg,
              'balance_sha256':digest(args.balance),'seeds':list(range(args.seed_base,args.seed_base+args.seeds)),
              'valid_samples':len(observations),'rows':rows,'scope':'Training demonstrations; never held-out acceptance.'}
    torch.save({'observation':torch.tensor(np.asarray(observations)),'target':torch.tensor(np.asarray(targets)),
                'metadata':metadata},args.out)
    args.out.with_suffix('.json').write_text(json.dumps(metadata,indent=2))


if __name__=='__main__':main()
