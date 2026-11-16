#!/usr/bin/env python3
"""Development-only GPU feedback grid; all promising rows need CPU replay.

Search uses the existing 31 observations, original action/motor limits, exact
coupled-target teacher and frozen balance controller. It is controller design,
not a PPO training result or an independent acceptance set.
"""
import argparse
import copy
import itertools
import json
import math
from pathlib import Path
import time

import torch
from torch import nn

from realdata_train import SamplePolicy31,verified_artifact,digest
from realdata_allocator import AllocatorActor
from fifty_mujoco.envs.realdata_warp_getup import WarpSupportedGetup,FAILURES


class Variants(nn.Module):
    def __init__(self,actor,values):
        super().__init__();self.actor=actor
        self.register_buffer('values',torch.tensor(values,device='cuda'))

    def forward(self,obs):
        action=self.actor(obs)
        kp,kd,kv,kx=self.values.unbind(-1)
        delta=(kp-10)*obs[:,3]+4*(kd-1)*obs[:,1]+(kv-1.5)*obs[:,29]/2-(kx-3)*obs[:,25]/5
        action=action.clone();action[:,2]+=delta;action[:,5]+=delta
        return action


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--balance',type=Path,required=True);ap.add_argument('--config',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--reset-range',nargs=2,type=float,default=[0,.1])
    ap.add_argument('--kp',nargs='+',type=float,default=[8.,10.,14.,18.])
    ap.add_argument('--kd',nargs='+',type=float,default=[.5,1.,1.5,2.])
    ap.add_argument('--kv',nargs='+',type=float,default=[.8,1.2,1.6,2.,2.5,3.,4.,6.])
    ap.add_argument('--kx',nargs='+',type=float,default=[3.])
    args=ap.parse_args();torch.set_num_threads(1);torch.manual_seed(559000)
    cfg=json.loads(args.config.read_text());args.out.mkdir(parents=True,exist_ok=False)
    grid=list(itertools.product(args.kp,args.kd,args.kv,args.kx))
    policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cuda')
    saved=torch.load(args.balance,map_location='cuda',weights_only=True);policy.load_state_dict(saved['model'])
    balance=Variants(policy.actor,grid)
    actor=AllocatorActor(balance,cfg)
    env=WarpSupportedGetup(balance,len(grid),config=cfg,reset_range=args.reset_range,settle_seconds=1.)
    start=time.monotonic();steps=0
    with torch.no_grad():
        max_ticks=math.ceil((1+cfg['success']['getup_timeout_s']+cfg['success']['blend_s']+cfg['success']['balance_s']+1)/.01)
        for i in range(max_ticks):
            env.step(actor(env.observation));steps=i+1
            if i%25==0 and bool((env.phase>=4).all()):break
    p=env.plant
    rows=[]
    for i,gains in enumerate(grid):
        terminal=env.terminal_metrics[i]
        rows.append({'index':i,'gains':gains,'phase':int(env.phase[i]),
                     'failure':FAILURES[int(env.first_failure[i])],
                     'balance_s':float(env.balance_time[i]),'stable_s':float(env.stable[i]),
                     'height_error_m':float((terminal[0]-cfg['standing_clearance_m']).abs()),
                     'speed_m_s':float(terminal[1]),'displacement_m':float(terminal[2])})
    rows.sort(key=lambda r:(r['phase']==5,r['balance_s'],r['stable_s'],
                            -r['height_error_m']-abs(r['speed_m_s'])),reverse=True)
    report={'schema':'development_feedback_grid_v1','balance_sha256':digest(args.balance),
            'configuration':cfg,'reset_range':args.reset_range,'seed':559000,
            'wall_s':time.monotonic()-start,'steps':steps,'rows':rows}
    (args.out/'search.json').write_text(json.dumps(report,indent=2))
    for rank,row in enumerate(rows[:12]):
        state=copy.deepcopy(saved);kp,kd,kv,kx=row['gains']
        for index in (2,5):
            state['model']['actor.0.weight'][index,3]+=kp-10
            state['model']['actor.0.weight'][index,1]+=4*(kd-1)
            state['model']['actor.0.weight'][index,29]+=(kv-1.5)/2
            state['model']['actor.0.weight'][index,25]-=(kx-3)/5
        config=copy.deepcopy(cfg);config['teacher_feedback'].update(pitch_kp=kp,pitch_kd=kd,velocity=kv)
        config['velocity_feedback_derivation']['selected_action_velocity_gain']=kv
        state['position_gain']=kx
        state['configuration']=config;state['development_search_row']=row
        torch.save(state,args.out/f'balance_rank{rank:02d}.pt')
    print(json.dumps({'wall_s':report['wall_s'],'gpu_successes':sum(r['phase']==5 for r in rows),'top':rows[:12]}),flush=True)


if __name__=='__main__':main()
