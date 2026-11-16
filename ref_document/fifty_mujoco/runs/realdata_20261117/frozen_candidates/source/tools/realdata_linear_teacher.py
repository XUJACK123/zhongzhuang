#!/usr/bin/env python3
"""Encode a calculated linear state-feedback teacher exactly in the 31D MLP.

Positive ELU paths carry the linear controller; unused paths remain available
for subsequent PPO. This avoids approximation error from short distillation.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import torch

from realdata_train import (RealDataHardwareEnv31,SamplePolicy31,verified_artifact,
                            equilibrium_action,evaluate,PROJECT)


def encode(policy,base,gains,position_gain=0.,leg_feedforward=None):
    device=next(policy.parameters()).device
    matrix=torch.zeros((6,31),device=device)
    matrix[0,3]=-2;matrix[3,3]=2
    matrix[1,4]=matrix[4,4]=-gains['roll_kp']
    matrix[1,0]=matrix[4,0]=4*gains['roll_kd']
    for i in (2,5):
        matrix[i,3]=gains['pitch_kp'];matrix[i,1]=4*gains['pitch_kd']
        matrix[i,29]=gains['velocity']/2
        matrix[i,25]=-position_gain/5
    matrix[2,2]=-.6;matrix[5,2]=.6
    matrix[2,27]=.15;matrix[5,27]=-.15
    if leg_feedforward is not None:
        default,radius=leg_feedforward
        a,b=default[0],default[0]+default[1]+np.pi/2
        # For zero root vx: wheel_qd = foot_x_dot / R - shin_qd.
        # Observation qd scale .05 and action wheel scale 20 cancel.
        hip=(-.21*np.sin(a)-.25*np.sin(b))/radius-1
        knee=-.25*np.sin(b)/radius-1
        matrix[2,13]+=hip;matrix[2,14]+=knee
        matrix[5,16]-=hip;matrix[5,17]-=knee
    layers=[v for v in policy.actor if isinstance(v,torch.nn.Linear)]
    with torch.no_grad():
        for layer in layers:
            layer.weight.normal_(std=1e-3);layer.bias.zero_()
        layers[0].weight[:6]=matrix;layers[0].bias[:6]=4.
        for layer in layers[1:-1]:
            layer.weight[:6]=0;layer.weight[:6,:6]=torch.eye(6,device=device)
        layers[-1].weight.zero_();layers[-1].weight[:,:6]=torch.eye(6,device=device)
        layers[-1].bias.copy_(torch.tensor(base,device=device)-4.)
        policy.log_std.copy_(torch.tensor([.025,.025,.015,.025,.025,.015],device=device).log())


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--position-gain',type=float,default=0.);p.add_argument('--eval-seconds',type=float,default=20.)
    p.add_argument('--leg-feedforward',action='store_true');p.add_argument('--skip-evaluation',action='store_true')
    args=p.parse_args();torch.set_num_threads(1);torch.manual_seed(510000)
    cfg=json.loads(args.config.read_text());e=RealDataHardwareEnv31(config=cfg)
    base=equilibrium_action(e);policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cpu')
    encode(policy,base,cfg['teacher_feedback'],args.position_gain,
           (cfg['standing_default_rad'],cfg['wheel_radius_m']) if args.leg_feedforward else None)
    args.out.mkdir(parents=True,exist_ok=False)
    torch.save({'schema':'realdata_exact_linear_initialization_v1','model':policy.state_dict(),
                'configuration':cfg,'equilibrium_action':base.tolist(),'position_gain':args.position_gain,
                'leg_velocity_feedforward':args.leg_feedforward},args.out/'actor.pt')
    if args.skip_evaluation:return
    import realdata_train
    old=realdata_train.RealDataHardwareEnv31
    realdata_train.RealDataHardwareEnv31=lambda **kwargs:old(config=cfg,**kwargs)
    result=evaluate(policy,count=4,seconds=args.eval_seconds,seed_base=612500)
    (args.out/'evaluation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)


if __name__=='__main__':main()
