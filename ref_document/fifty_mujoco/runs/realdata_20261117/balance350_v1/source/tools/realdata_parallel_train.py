#!/usr/bin/env python3
"""Parallel CPU sampling + CUDA PPO for the balance/get-up candidate tasks."""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import signal
import time

import numpy as np
import torch

from realdata_train import (PROJECT, RealDataHardwareEnv31, SamplePolicy31, SamplePPO,
                            verified_artifact, save_checkpoint, initialize, evaluate, digest)
from realdata_sampler import ProcessSampler, make_env


def getup_evaluate(policy, settings, count=8, seed_base=611000):
    balanced=SamplePolicy31(verified_artifact('model_3050.pt'),'cpu')
    balanced.load_state_dict(torch.load(settings['balance_checkpoint'],map_location='cpu',weights_only=True)['model'])
    env=make_env(settings,balanced.actor)
    rows=[]
    for j in range(count):
        obs=env.reset(seed_base+j)
        for _ in range(2100):
            with torch.no_grad():action=policy.actor(torch.tensor(obs,device=next(policy.parameters()).device)[None])[0].cpu().numpy()
            obs,_,terminated,truncated,info=env.step(action)
            if terminated or truncated:break
        rows.append({k:info[k] for k in ('success','failure','phase','phase_counts','balance_ticks','support_peak_n','support_peak_nm','support_wrench','wheel_load_n','episode_parameters')})
    wins=sum(r['success'] for r in rows)
    return {'trials':count,'passed':wins,'rows':rows,'score':100*(1-wins/count)+sum(0 if r['phase_counts'].get('HANDOFF',0) else 1 for r in rows)/count}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--init',type=Path)
    p.add_argument('--resume',type=Path);p.add_argument('--teacher-init',action='store_true')
    p.add_argument('--config',type=Path);p.add_argument('--task',choices=['balance','getup'],default='balance')
    p.add_argument('--balance-checkpoint',type=Path);p.add_argument('--reset-range',nargs=2,type=float,default=[0.,0.])
    p.add_argument('--settle-seconds',type=float,default=0.);p.add_argument('--domain',action='store_true')
    p.add_argument('--device',default='cuda');p.add_argument('--seconds',type=float,default=1800)
    p.add_argument('--updates',type=int,default=100000);p.add_argument('--envs',type=int,default=64)
    p.add_argument('--workers',type=int,default=8);p.add_argument('--steps',type=int,default=64)
    p.add_argument('--actor-lr',type=float,default=3e-5);p.add_argument('--eval-every',type=int,default=100)
    p.add_argument('--save-every',type=int,default=50);p.add_argument('--episode-seconds',type=float,default=12.)
    p.add_argument('--seed',type=int,default=510000);p.add_argument('--no-initial-eval',action='store_true')
    p.add_argument('--initial-std',type=float);args=p.parse_args()
    if args.task=='getup' and not args.balance_checkpoint:p.error('getup requires frozen balance checkpoint')
    torch.set_num_threads(1);torch.manual_seed(args.seed)
    if args.device.startswith('cuda') and not torch.cuda.is_available():raise RuntimeError('CUDA required')
    args.out.mkdir(parents=True,exist_ok=False)
    config=json.loads(args.config.read_text()) if args.config else None
    example=RealDataHardwareEnv31(config=config,episode_seconds=args.episode_seconds)
    policy,base=initialize(args,example)
    if args.initial_std:
        with torch.no_grad():policy.log_std.fill_(math.log(args.initial_std))
    optimizer=SamplePPO(policy,actor_lr=args.actor_lr,critic_lr=3e-4)
    settings={'task':args.task,'episode_seconds':args.episode_seconds,'seed':args.seed,
              'balance_checkpoint':str(args.balance_checkpoint.resolve()) if args.balance_checkpoint else None,
              'reset_range':list(args.reset_range),'settle_seconds':args.settle_seconds,'domain':args.domain,'config':config}
    sampler=ProcessSampler(args.envs,args.workers,settings)
    observations=sampler.reset()
    files=['tools/realdata_parallel_train.py','tools/realdata_sampler.py','tools/realdata_train.py',
           'src/fifty_mujoco/envs/sample_env31_realdata.py','src/fifty_mujoco/envs/sample_env31_supported.py',
           'src/fifty_mujoco/learning/sample_ppo.py','src/fifty_mujoco/contracts/getup31.py',
           'src/fifty_mujoco/contracts/sample_actor31.py','src/fifty_mujoco/transfer/firmware_runtime.py']
    contract={'schema':'realdata_parallel_ppo_v1','task':args.task,'configuration':example.candidate,
              'source_sha256':{name:digest(PROJECT/name) for name in files},'model':example.snapshot(),
              'sampling':settings,'device':args.device,'gpu_name':torch.cuda.get_device_name() if args.device.startswith('cuda') else None,
              'balance_checkpoint_sha256':digest(args.balance_checkpoint) if args.balance_checkpoint else None,
              'initial_checkpoint_sha256':digest(args.init) if args.init else None,
              'teacher_initialization':args.teacher_init,'static_teacher_action':base.tolist(),
              'hyperparameters':{k:getattr(args,k) for k in ('envs','workers','steps','actor_lr')}}
    for name in files:
        target=args.out/'source'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PROJECT/name,target)
    (args.out/'contract.json').write_text(json.dumps(contract,indent=2))
    update=0;transitions=0;best=math.inf;start=time.monotonic();stopped=False
    if args.resume:
        saved=torch.load(args.resume,map_location=args.device,weights_only=True)
        if saved['contract']!=contract:raise ValueError('resume contract mismatch; use --init for explicit new experiment')
        policy.load_state_dict(saved['model']);optimizer.actor_optimizer.load_state_dict(saved['actor_optimizer'])
        optimizer.critic_optimizer.load_state_dict(saved['critic_optimizer']);torch.set_rng_state(saved['torch_rng'].cpu())
        if args.device.startswith('cuda'):torch.cuda.set_rng_state_all([s.cpu() for s in saved['cuda_rng']])
        update=saved['update'];transitions=saved['transitions']
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def save(name):
        save_checkpoint(args.out/name,{'schema':'realdata_parallel_ppo_v1','contract':contract,'model':policy.state_dict(),
                     'actor_optimizer':optimizer.actor_optimizer.state_dict(),'critic_optimizer':optimizer.critic_optimizer.state_dict(),
                     'torch_rng':torch.get_rng_state(),'cuda_rng':torch.cuda.get_rng_state_all() if args.device.startswith('cuda') else [],
                     'update':update,'transitions':transitions,'sampler_resume_scope':'fresh plants and disjoint future seed block required'})
    def run_eval():
        if args.task=='getup':return getup_evaluate(policy,settings,count=8)
        if config:
            # Temporarily parameterise the evaluator's factory; source stays frozen.
            import realdata_train
            old=realdata_train.RealDataHardwareEnv31
            realdata_train.RealDataHardwareEnv31=lambda **kwargs:old(config=config,**kwargs)
            try:return evaluate(policy,count=4,seconds=20.,seed_base=611000)
            finally:realdata_train.RealDataHardwareEnv31=old
        return evaluate(policy,count=4,seconds=20.,seed_base=611000)
    names=('observation','action','log_prob','value','reward','next_value','terminated','truncated','mean','std')
    save('update_000000.pt')
    try:
        if not args.no_initial_eval:
            initial=run_eval();(args.out/'initial_evaluation.json').write_text(json.dumps(initial,indent=2))
            best=initial['score'];save('best.pt')
            print(json.dumps({'initial_evaluation':initial}),flush=True)
        with (args.out/'rows.jsonl').open('a') as log:
            for _ in range(args.updates):
                if stopped or time.monotonic()-start>=args.seconds:break
                rollout={n:[] for n in names};outcomes=[];sample_start=time.monotonic()
                for _ in range(args.steps):
                    obs=torch.tensor(np.stack(observations),device=args.device)
                    with torch.no_grad():
                        dist=policy.distribution(obs);action=dist.sample();value=policy.value(obs);lp=dist.log_prob(action).sum(-1)
                    results=sampler.step(action.cpu().numpy())
                    terminal=torch.tensor(np.stack([r[0] for r in results]),device=args.device)
                    with torch.no_grad():next_value=policy.value(terminal)
                    values=(obs,action,lp,value,torch.tensor([r[1] for r in results],device=args.device),next_value,
                            torch.tensor([r[2] for r in results],device=args.device),torch.tensor([r[3] for r in results],device=args.device),dist.mean,dist.stddev)
                    for name,item in zip(names,values):rollout[name].append(item.detach())
                    observations=[r[5] for r in results]
                    for r in results:
                        if r[2] or r[3]:
                            outcomes.append({k:r[4].get(k) for k in ('time_s','failure','success','phase','phase_counts','balance_ticks','episode_parameters')})
                sample_s=time.monotonic()-sample_start;optimization_start=time.monotonic()
                metrics=optimizer.update({k:torch.stack(v) for k,v in rollout.items()},warmup=update<5,epochs=4,batch_size=1024)
                optimization_s=time.monotonic()-optimization_start
                update+=1;transitions+=args.envs*args.steps
                row={'update':update,'transitions':transitions,'wall_s':time.monotonic()-start,
                     'sample_s':sample_s,'optimization_s':optimization_s,'samples_per_s':args.envs*args.steps/sample_s,
                     'reward_mean':float(torch.stack(rollout['reward']).mean()),'outcomes':outcomes,**metrics}
                if update%args.eval_every==0:
                    result=run_eval();row['evaluation']=result
                    (args.out/f'evaluation_{update:06d}.json').write_text(json.dumps(result,indent=2))
                    if result['score']<best:best=result['score'];save('best.pt')
                log.write(json.dumps(row,allow_nan=False)+'\n');log.flush()
                print(json.dumps({k:v for k,v in row.items() if k!='outcomes'},allow_nan=False),flush=True)
                if update%args.save_every==0:save(f'update_{update:06d}.pt')
    finally:
        save('last.pt');sampler.close()
        (args.out/'training_report.json').write_text(json.dumps({'update':update,'transitions':transitions,
                  'wall_s':time.monotonic()-start,'best_development_score':best if math.isfinite(best) else None,
                  'stopped_by_signal':stopped,'device':args.device,'workers':args.workers},indent=2))


if __name__=='__main__':main()
