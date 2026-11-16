#!/usr/bin/env python3
"""Check observer vs actual C and closed-loop CUDA physics vs CPU candidate."""
import argparse
import ctypes as c
import json
from pathlib import Path
import sys
import time

import mujoco
import numpy as np
import torch
import warp as wp

from realdata_train import PROJECT,RealDataHardwareEnv31,equilibrium_action
from fifty_mujoco.envs.realdata_warp import RealDataWarpEnv,QualifiedObserver


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=PROJECT/'configs/realdata_350_20261117.json')
    p.add_argument('--worlds',type=int,default=256);p.add_argument('--out',type=Path,required=True);args=p.parse_args()
    torch.set_num_threads(1);config=json.loads(args.config.read_text());rng=np.random.default_rng(612000)
    e=RealDataHardwareEnv31(config=config);e.reset(612000);e.data.qvel[:]=0
    observer=QualifiedObserver(1,'cuda',config['wheel_radius_m']);observer.last_q[:]=torch.tensor(e.source.defaults,device='cuda')
    maximum={'qd':0.,'xy':0.,'velocity':0.,'confidence':0.}
    for i in range(128):
        e.data.time=(i+1)*.005
        e.data.qpos[e.source.qpos_ids]=e.source.defaults+.003*np.sin(.03*i+np.arange(6))
        e.data.qvel[e.source.qvel_ids]=.018*np.cos(.03*i+np.arange(6))
        e.data.qvel[3:6]=rng.uniform(-.05,.05,3)
        mujoco.mj_forward(e.model,e.data)
        acceleration=rng.uniform(-.3,.3,3);e.data.qacc[:3]=acceleration
        valid=e._update_estimator()
        def gpu(x):return torch.tensor(np.asarray(x),device='cuda',dtype=torch.float32)[None]
        observer.update(gpu(e.data.qpos[e.source.qpos_ids]),gpu(e.data.qvel[e.source.qvel_ids]),
                        gpu(e.data.qpos[3:7]),gpu(e._velocity6()[:3]),gpu(acceleration))
        pairs={'qd':(observer.qd[0].cpu().numpy(),e.joint_velocity),
               'xy':(observer.xy[0].cpu().numpy(),e.estimated_xy),
               'velocity':(observer.velocity[0,:2].cpu().numpy(),e.estimated_velocity[:2]),
               'confidence':(observer.confidence[0].item(),e.c_quality.confidence)}
        for name,(a,b) in pairs.items():maximum[name]=max(maximum[name],float(np.max(np.abs(np.asarray(a)-b))))
        if not valid:raise RuntimeError('C rejected validation trajectory')
    observer_pass=maximum['qd']<.002 and maximum['xy']<1e-4 and maximum['velocity']<1e-4 and maximum['confidence']<.001
    print(json.dumps({'observer_errors':maximum,'observer_passed':observer_pass}),flush=True)
    args.out.write_text(json.dumps({'observer_errors':maximum,'observer_passed':observer_pass,'physics_status':'running'},indent=2))
    gpuenv=RealDataWarpEnv(args.worlds,config=config,capture=True)
    base=equilibrium_action(e);e.reset(612000);e.data.qvel[:]=0;mujoco.mj_forward(e.model,e.data);gpuenv.reset()
    worst_q=worst_obs=worst_height=0.;flag_mismatch=0
    trace=[]
    for i in range(200):
        from fifty_mujoco.envs.sample_env31 import rpy_from_wxyz
        roll,pitch,_=rpy_from_wxyz(e.data.qpos[3:7]);omega=e._velocity6()[:3]
        action=base.copy();action[[0,3]]+=[-2*pitch,2*pitch];action[[1,4]]+=6*roll+.4*omega[0]
        action[[2,5]]=10*pitch+omega[1]+.5*e.estimated_velocity[0]
        obs,_,done,timeout,info=e.step(action)
        grpy=__import__('fifty_mujoco.envs.realdata_warp',fromlist=['angles']).angles(gpuenv.qpos[:,3:7])
        ga=torch.tensor(base,device='cuda',dtype=torch.float32)[None].expand(args.worlds,-1).clone()
        ga[:,0]-=2*grpy[:,1];ga[:,3]+=2*grpy[:,1]
        ga[:,1]+=6*grpy[:,0]+.4*gpuenv.qvel[:,3];ga[:,4]+=6*grpy[:,0]+.4*gpuenv.qvel[:,3]
        ga[:,2]=10*grpy[:,1]+gpuenv.qvel[:,4]+.5*gpuenv.observer.velocity[:,0];ga[:,5]=ga[:,2]
        go,_,gd,gt=gpuenv.step(ga)
        qerr=float(np.abs(gpuenv.qpos[0].cpu().numpy()-e.data.qpos).max())
        oerr=float(np.abs(go[0].cpu().numpy()-obs).max())
        herr=abs(float(gpuenv.clearance[0])-info['clearance_m'])
        worst_q=max(worst_q,qerr);worst_obs=max(worst_obs,oerr);worst_height=max(worst_height,herr)
        flag_mismatch+=int(bool(gd[0])!=done)
        if i%20==0:trace.append({'step':i,'qpos_error':qerr,'observation_error':oerr,'height_error':herr})
        if done or bool(gd[0]):break
    gpuenv.reset();zeros=torch.zeros((args.worlds,6),device='cuda');start=time.monotonic()
    for i in range(100):gpuenv.step(zeros)
    torch.cuda.synchronize();seconds=time.monotonic()-start
    report={'observer_errors':maximum,'observer_passed':observer_pass,'physics_steps_compared':i+1,
            'qpos_max_error':worst_q,'observation_max_error':worst_obs,'clearance_max_error_m':worst_height,
            'termination_mismatches':flag_mismatch,'trace':trace,'worlds':args.worlds,
            'captured_full_policy_steps_per_s':args.worlds*100/seconds,
            'physics_passed':worst_q<.02 and worst_obs<.1 and worst_height<.005 and flag_mismatch==0,
            'candidate_scope':'independent teacher feedback on each backend; open-loop divergence recorded separately; CPU task heldout acceptance still required'}
    args.out.write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)


if __name__=='__main__':main()
