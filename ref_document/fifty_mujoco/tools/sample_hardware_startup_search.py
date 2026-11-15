#!/usr/bin/env python3
"""Offline search for a short symmetric action pulse before actor handoff.

This is a simulated controller experiment, never a hardware motion tool. Every
trial starts from the externally supported surveyed pose, removes support, and
stops on first physical failure. A pulse is not part of the exported actor.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
import itertools,json,multiprocessing,sys,time,hashlib
from pathlib import Path
import numpy as np
import torch
PROJECT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(PROJECT/'src'))
from fifty_mujoco.envs.sample_env31_hardware_fast import FastHardwareEnv31
from fifty_mujoco.learning.sample_ppo import SamplePolicy31
from fifty_mujoco.transfer.sample_source import verified_artifact
ACTOR=None


def initialize(checkpoint):
    global ACTOR
    torch.set_num_threads(1);p=SamplePolicy31(verified_artifact('model_3050.pt'))
    p.load_state_dict(torch.load(checkpoint,map_location='cpu',weights_only=True)['model']);ACTOR=p.actor.eval()


def trial(job):
    h,k,w=job['pulse'];e=FastHardwareEnv31(10,reset_mode='natural',wheel_kd=.1)
    obs=e.reset(job['seed']);rows=[];saturation=np.zeros(6);controls=0
    pulse=np.array([h,k,w,-h,-k,w])
    for tick in range(e.max_steps):
        with torch.no_grad():action=ACTOR(torch.tensor(obs)[None])[0].numpy()
        if tick*e.source.policy_dt<job['pulse_seconds']:action=pulse
        obs,_,d,t,info=e.step(action)
        saturation+=info['saturation_counts'];controls+=info['controls']
        if info['time_s']>=2:rows.append([info['point_velocity_xy'][0],info['yaw_rate'],info['clearance_m']-.1])
        if d or t:break
    a=np.asarray(rows)
    return {**job,'completed':bool(t and not d),'elapsed_s':info['time_s'],'first_failure':info['failure'],
        'steady_vx_rmse':float(np.sqrt(np.mean(a[:,0]**2))) if len(a) else None,
        'steady_yaw_rmse':float(np.sqrt(np.mean(a[:,1]**2))) if len(a) else None,
        'steady_clearance_mae':float(np.abs(a[:,2]).mean()) if len(a) else None,
        'motor_saturation_fractions':(saturation/max(controls,1)).tolist()}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--workers',type=int,default=4)
    p.add_argument('--deadline-epoch',type=float,required=True);a=p.parse_args()
    jobs=[{'pulse':list(v),'pulse_seconds':duration,'seed':85000} for v in itertools.product([-1.,0.,1.],repeat=3) for duration in (.08,.16)]
    started=time.monotonic();rows=[];a.out.parent.mkdir(parents=True,exist_ok=True)
    if a.out.exists():raise ValueError('use a new output path')
    with a.out.with_suffix('.rows.jsonl').open('w') as f,ProcessPoolExecutor(max_workers=a.workers,
        mp_context=multiprocessing.get_context('spawn'),initializer=initialize,
        initargs=(str(a.checkpoint.resolve()),)) as pool:
        futures=[pool.submit(trial,j) for j in jobs] if time.time()<a.deadline_epoch-300 else []
        for future in as_completed(futures):
            r=future.result();rows.append(r);f.write(json.dumps(r)+'\n');f.flush()
            if len(rows)%10==0:print(json.dumps({'done':len(rows),'total':len(jobs),'completed':sum(x['completed'] for x in rows)}),flush=True)
    report={'tool':'hardware-startup-action-search','status':'offline_development_not_deployment',
        'checkpoint_sha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),'trials':len(rows),
        'completed':sum(r['completed'] for r in rows),'wall_seconds':time.monotonic()-started,
        'rows':sorted(rows,key=lambda r:r['elapsed_s'],reverse=True),'real_robot_accepted':False,
        'pulse_in_exported_actor':False,'limitations':['One development seed only; any successful pulse needs independent validation and a separate deployment controller.']}
    a.out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'trials':len(rows),'completed':report['completed'],'best':report['rows'][:3]}))


if __name__=='__main__':main()
