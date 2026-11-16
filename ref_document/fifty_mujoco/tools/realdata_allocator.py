#!/usr/bin/env python3
"""Observation-only teacher: anticipate coupled motor clipping in knee targets."""
import argparse
import json
import math
from pathlib import Path

import torch
from torch import nn

from realdata_train import RealDataHardwareEnv31,SamplePolicy31,verified_artifact
from realdata_parallel_train import getup_evaluate


class AllocatorActor(nn.Module):
    def __init__(self,balance_actor,configuration):
        super().__init__();self.balance=balance_actor.requires_grad_(False)
        env=RealDataHardwareEnv31(config=configuration)
        device=next(balance_actor.parameters()).device
        self.register_buffer('default',torch.tensor(configuration['standing_default_rad'],device=device,dtype=torch.float32))
        self.register_buffer('limits',torch.tensor(env.target_limits,device=device,dtype=torch.float32))
        self.register_buffer('bound',torch.tensor([1,1,2,1,1,2],device=device,dtype=torch.float32))
        self.ratio=env.fast_actuators.ratio

    def forward(self,obs):
        action=torch.maximum(torch.minimum(self.balance(obs),self.bound),-self.bound)
        qh=torch.stack((self.default[0]+obs[:,9],self.default[3]+obs[:,11]),-1)
        qk=torch.stack((self.default[1]+obs[:,10],self.default[4]+obs[:,12]),-1)
        qdh=torch.stack((20*obs[:,13],20*obs[:,16]),-1)
        qdk=torch.stack((20*obs[:,14],20*obs[:,17]),-1)
        ah=torch.stack((action[:,0],action[:,3]),-1);ak=torch.stack((action[:,1],action[:,4]),-1)
        default_h=torch.stack((self.default[0],self.default[3]));default_k=torch.stack((self.default[1],self.default[4]))
        error=default_h+.5*ah-qh;error=torch.atan2(torch.sin(error),torch.cos(error))
        th=(40*error-2*qdh).clamp(-40,40)
        target_k=torch.maximum(torch.minimum(default_k+.5*ak,self.limits[:,1]),self.limits[:,0])
        tk=(60*(target_k-qk)-2*qdk).clamp(-54.82787762486449,54.82787762486449)
        sign=qh.new_tensor([1.,-1.])
        beta=sign*qk+math.pi/2
        alpha=2*torch.atan2(torch.sin(beta),self.ratio+torch.cos(beta))
        ratio=.5+self.ratio*torch.cos(alpha/2)/(2*torch.sqrt(1-self.ratio**2*torch.sin(alpha/2)**2))
        vh=sign*qdh;vk=sign*(qdh+qdk/ratio)
        rated,free=100*2*math.pi/60,160*2*math.pi/60
        def bounds(velocity):
            positive=20*((free-velocity)/(free-rated)).clamp(0,1)
            negative=-20*((free+velocity)/(free-rated)).clamp(0,1)
            return torch.where(sign>0,negative,-positive),torch.where(sign>0,positive,-negative)
        hmin,hmax=bounds(vh);kmin,kmax=bounds(vk)
        feasible_h=torch.maximum(torch.minimum(th,hmax+kmax),hmin+kmin)
        low=torch.maximum(kmin,feasible_h-hmax);high=torch.minimum(kmax,feasible_h-hmin)
        motor_k=torch.maximum(torch.minimum(ratio*tk,high),low)
        desired_joint_k=motor_k/ratio
        position=qk+(desired_joint_k+2*qdk)/60
        adjusted=(2*(position-default_k)).clamp(-1,1)
        return torch.stack((action[:,0],adjusted[:,0],action[:,2],action[:,3],adjusted[:,1],action[:,5]),-1)


def main():
    p=argparse.ArgumentParser();p.add_argument('--balance',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--reset-range',type=float,nargs=2,default=[0,0.1])
    p.add_argument('--settle-seconds',type=float,default=1.);p.add_argument('--seeds',type=int,default=2)
    args=p.parse_args();torch.set_num_threads(1);config=json.loads(args.config.read_text())
    policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cpu')
    policy.load_state_dict(torch.load(args.balance,map_location='cpu',weights_only=True)['model'])
    policy.actor=AllocatorActor(policy.actor,config)
    settings={'task':'getup','balance_checkpoint':str(args.balance),'reset_range':args.reset_range,
              'settle_seconds':args.settle_seconds,'domain':False,'config':config}
    report=getup_evaluate(policy,settings,count=args.seeds,seed_base=618500)
    args.out.write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)


if __name__=='__main__':main()
