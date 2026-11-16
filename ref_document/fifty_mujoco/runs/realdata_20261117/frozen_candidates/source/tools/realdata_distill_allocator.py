#!/usr/bin/env python3
"""Distil motor-feasible knee targets into the standard 31D MLP."""
import argparse
import copy
import json
from pathlib import Path

import torch

from realdata_train import SamplePolicy31,verified_artifact,digest
from realdata_allocator import AllocatorActor


def observations(n,cfg,device):
    obs=torch.zeros((n,31),device=device)
    roll=torch.empty(n,device=device).uniform_(-.10,.10);pitch=torch.empty(n,device=device).uniform_(-.30,.30)
    obs[:,:3].uniform_(-.25,.25)
    obs[:,3]=torch.sin(pitch);obs[:,4]=-torch.sin(roll)*torch.cos(pitch);obs[:,5]=-torch.cos(roll)*torch.cos(pitch)
    obs[:,8]=5*cfg['standing_clearance_m']
    obs[:,9].uniform_(-1.6,.8);obs[:,11].uniform_(-.8,1.6)
    obs[:n//2,11]=-obs[:n//2,9]+torch.empty(n//2,device=device).uniform_(-.02,.02)
    obs[:,10].uniform_(-.4,.67);obs[:,12].uniform_(-.67,.4)
    obs[:n//2,12]=-obs[:n//2,10]
    obs[:,13:19].uniform_(-.20,.20);obs[:,25:27].uniform_(-1.,1.)
    obs[:,27].uniform_(-.1,.1);obs[:,28]=torch.sqrt(1-obs[:,27].square())-1
    obs[:,29:31].uniform_(-.5,.5)
    # Include dense near-standing examples so static feedforward is preserved.
    m=n//4;obs[:m,:3].uniform_(-.01,.01);obs[:m,3:5].uniform_(-.01,.01);obs[:m,5]=-1
    obs[:m,9:13].uniform_(-.02,.02);obs[:m,13:19].uniform_(-.005,.005);obs[:m,25:31].uniform_(-.05,.05)
    return obs


def main():
    p=argparse.ArgumentParser();p.add_argument('--balance',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--steps',type=int,default=5000)
    p.add_argument('--dataset',type=Path);p.add_argument('--resume',type=Path)
    p.add_argument('--trajectory-fraction',type=float,default=.5)
    p.add_argument('--no-origin-correction',action='store_true');args=p.parse_args()
    torch.set_num_threads(1);torch.manual_seed(553000);cfg=json.loads(args.config.read_text())
    args.out.mkdir(exist_ok=False,parents=True);policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cuda')
    policy.load_state_dict(torch.load(args.balance,map_location='cuda',weights_only=True)['model'])
    base=copy.deepcopy(policy.actor).requires_grad_(False)
    teacher=AllocatorActor(copy.deepcopy(policy.actor),cfg)
    layers=[v for v in policy.actor if isinstance(v,torch.nn.Linear)];hooks=[]
    used=list(range(6))+list(range(9,19))+list(range(25,31))
    if args.resume:
        policy.load_state_dict(torch.load(args.resume,map_location='cuda',weights_only=True)['model'])
    dataset=torch.load(args.dataset,map_location='cuda',weights_only=True) if args.dataset else None
    with torch.no_grad():
      if not args.resume:
        layers[0].weight[6:]=0
        layers[0].weight[6:,used]=torch.randn((layers[0].out_features-6,len(used)),device='cuda')*.2
        layers[0].bias[6:]=0
        for layer in layers[1:-1]:
            layer.weight[6:,:6]=0;torch.nn.init.xavier_uniform_(layer.weight[6:,6:]);layer.bias[6:]=0
        layers[-1].weight[:,6:]=0
    for i,layer in enumerate(layers):
        w=torch.zeros_like(layer.weight);b=torch.zeros_like(layer.bias)
        if i==0:w[6:,used]=1;b[6:]=1
        elif i<len(layers)-1:w[6:,6:]=1;b[6:]=1
        else:w[1,6:]=1;w[4,6:]=1
        hooks.append(layer.weight.register_hook(lambda grad,mask=w:grad*mask))
        hooks.append(layer.bias.register_hook(lambda grad,mask=b:grad*mask))
    optimizer=torch.optim.Adam(policy.actor.parameters(),lr=.001);losses=[]
    for step in range(args.steps):
        obs=observations(2048,cfg,'cuda')
        if dataset is not None:
            size=int(2048*args.trajectory_fraction)
            chosen=torch.randint(dataset['observation'].shape[0],(size,),device='cuda')
            obs[:size]=dataset['observation'][chosen]
        with torch.no_grad():target=teacher(obs)
        output=policy.actor(obs)
        loss=(output[:,[1,4]]-target[:,[1,4]]).square().mean()
        optimizer.zero_grad();loss.backward();optimizer.step()
        if step%500==0:
            losses.append({'step':step,'knee_mse':float(loss.detach())});print(json.dumps(losses[-1]),flush=True)
    for hook in hooks:hook.remove()
    with torch.no_grad():
        origin=torch.zeros((1,31),device='cuda');origin[:,5]=-1;origin[:,8]=5*cfg['standing_clearance_m']
        residual=policy.actor(origin)[0]-base(origin)[0]
        if not args.no_origin_correction:
            layers[-1].bias[1]-=residual[1];layers[-1].bias[4]-=residual[4]
        policy.log_std.copy_(torch.tensor([.01,.01,.008,.01,.01,.008],device='cuda').log())
        obs=observations(8192,cfg,'cuda');prediction=policy.actor(obs);target=teacher(obs)
        report={'synthetic_validation_samples':8192,'knee_rmse':float((prediction[:,[1,4]]-target[:,[1,4]]).square().mean().sqrt()),
                'preserved_hip_wheel_max_error':float((prediction[:,[0,2,3,5]]-base(obs)[:,[0,2,3,5]]).abs().max()),
                'losses':losses,'balance_sha256':digest(args.balance),
                'trajectory_dataset_sha256':digest(args.dataset) if args.dataset else None,
                'resume_sha256':digest(args.resume) if args.resume else None,
                'trajectory_fraction':args.trajectory_fraction,'origin_correction':not args.no_origin_correction,
                'physical_validation':'pending CPU and GPU task rollouts'}
        if dataset is not None:
            sample=dataset['observation'][:8192];target=teacher(sample);prediction=policy.actor(sample)
            report['trajectory_training_knee_rmse']=float((prediction[:,[1,4]]-target[:,[1,4]]).square().mean().sqrt())
    torch.save({'schema':'realdata_motor_allocation_distillation_v1','configuration':cfg,'model':policy.state_dict(),
                'initialization_report':report},args.out/'actor.pt')
    (args.out/'distillation_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)


if __name__=='__main__':main()
