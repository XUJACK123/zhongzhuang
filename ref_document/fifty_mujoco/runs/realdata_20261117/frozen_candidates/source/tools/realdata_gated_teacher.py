#!/usr/bin/env python3
"""Train a hip-first knee gate while preserving the exact balance feedback paths."""
import argparse
import json
from pathlib import Path

import numpy as np
import torch

from realdata_train import SamplePolicy31,verified_artifact,RealDataHardwareEnv31,equilibrium_action


def main():
    p=argparse.ArgumentParser();p.add_argument('--balance',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--steps',type=int,default=3000);args=p.parse_args()
    torch.set_num_threads(1);torch.manual_seed(552000);args.out.mkdir(exist_ok=False,parents=True)
    cfg=json.loads(args.config.read_text());policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cuda')
    policy.load_state_dict(torch.load(args.balance,map_location='cuda',weights_only=True)['model'])
    frozen=__import__('copy').deepcopy(policy.actor).requires_grad_(False)
    layers=[v for v in policy.actor if isinstance(v,torch.nn.Linear)];hooks=[]
    with torch.no_grad():
        layers[0].weight[6:]=0;layers[0].weight[6:,9].normal_(std=.5);layers[0].weight[6:,11].normal_(std=.5)
        layers[0].bias[6:]=0
        for layer in layers[1:-1]:
            layer.weight[6:,:6]=0;torch.nn.init.xavier_uniform_(layer.weight[6:,6:]);layer.bias[6:]=0
        layers[-1].weight[:,6:]=0
    for i,layer in enumerate(layers):
        wmask=torch.zeros_like(layer.weight);bmask=torch.zeros_like(layer.bias)
        if i==0:
            wmask[6:,9]=1;wmask[6:,11]=1;bmask[6:]=1
        elif i<len(layers)-1:
            wmask[6:,6:]=1;bmask[6:]=1
        else:wmask[1,6:]=1;wmask[4,6:]=1
        hooks.append(layer.weight.register_hook(lambda gradient,mask=wmask:gradient*mask))
        hooks.append(layer.bias.register_hook(lambda gradient,mask=bmask:gradient*mask))
    optimizer=torch.optim.Adam(policy.actor.parameters(),lr=.001)
    losses=[]
    for step in range(args.steps):
        observation=torch.zeros((2048,31),device='cuda');observation[:,5]=-1;observation[:,8]=5*cfg['standing_clearance_m']
        hip=torch.empty((2048,2),device='cuda').uniform_(-1.7,.8)
        hip[:512].uniform_(-.025,.025)
        observation[:,9]=hip[:,0];observation[:,11]=hip[:,1]
        with torch.no_grad():
            target=frozen(observation)
            gate=((hip.abs().amax(-1)-.05)/.20).clamp(0,1);gate=gate*gate*(3-2*gate)
            target[:,1]=(1-gate)*target[:,1]+gate
            target[:,4]=(1-gate)*target[:,4]-gate
        loss=(policy.actor(observation)-target).square().mean()
        optimizer.zero_grad();loss.backward();optimizer.step()
        if step%500==0:losses.append({'step':step,'mse':float(loss)});print(json.dumps(losses[-1]),flush=True)
    for hook in hooks:hook.remove()
    with torch.no_grad():
        origin=torch.zeros((1,31),device='cuda');origin[:,5]=-1;origin[:,8]=5*cfg['standing_clearance_m']
        residual=policy.actor(origin)[0]-frozen(origin)[0]
        layers[-1].bias[1]-=residual[1];layers[-1].bias[4]-=residual[4]
        policy.log_std.copy_(torch.tensor([.01,.01,.008,.01,.01,.008],device='cuda').log())
    torch.save({'schema':'realdata_hip_first_gate_initialization_v1','configuration':cfg,'model':policy.state_dict(),
                'gate':'smoothstep(clamp((max(abs(hip_offset))-.05)/.20,0,1)); blend knee actions toward +1/-1',
                'balance_feedback_preserved':True,'losses':losses},args.out/'actor.pt')
    # Quantify why independent joint PD plus coupled motor clipping stalls hips.
    e=RealDataHardwareEnv31(config=cfg);pose=np.array([3.417,-2.630,0,-3.407,2.609,0.])
    base=equilibrium_action(e);observation=e.reset();observation[9]=pose[0]-e.source.defaults[0];observation[11]=pose[3]-e.source.defaults[3]
    with torch.no_grad():a=policy.actor(torch.tensor(observation,device='cuda')[None])[0].cpu().numpy()
    from fifty_mujoco.contracts.sample_actor31 import action_targets
    comparison=[]
    for name,action in [('immediate_knee_compression',base),('hip_first_gate',a)]:
        target=action_targets(action,e.target_limits,e.source.defaults)
        request=np.array([40,60,0,40,60,0])*(np.asarray(target['position'])-pose)
        delivered=e.fast_actuators.apply(request,pose,np.zeros(6))
        comparison.append({'controller':name,'actions':action.tolist(),'requested_joint_nm':request.tolist(),
                           'motor_request_nm':e.fast_actuators.requested.tolist(),
                           'motor_limited_nm':e.fast_actuators.command.tolist(),'delivered_joint_nm':delivered.tolist()})
    (args.out/'gate_and_torque_audit.json').write_text(json.dumps(comparison,indent=2));print(json.dumps(comparison),flush=True)


if __name__=='__main__':main()
