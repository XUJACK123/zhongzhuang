#!/usr/bin/env python3
"""CUDA cohort PPO for supported rise, support release and frozen handoff."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import torch

from realdata_train import PROJECT,SamplePolicy31,verified_artifact,save_checkpoint,digest
from fifty_mujoco.envs.realdata_warp_getup import WarpSupportedGetup,FAILURES
from fifty_mujoco.learning.realdata_ppo import RealDataPPO


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--init',type=Path,required=True);p.add_argument('--balance',type=Path,required=True)
    p.add_argument('--envs',type=int,default=256);p.add_argument('--reset-range',nargs=2,type=float,default=[0,0])
    p.add_argument('--settle-seconds',type=float,default=1.);p.add_argument('--domain',action='store_true')
    p.add_argument('--seconds',type=float,default=1800);p.add_argument('--updates',type=int,default=100000)
    p.add_argument('--actor-lr',type=float,default=1e-6);p.add_argument('--warmup',type=int,default=3)
    p.add_argument('--save-every',type=int,default=5);p.add_argument('--eval-every',type=int,default=5)
    p.add_argument('--seed',type=int,default=550000);p.add_argument('--initial-std',type=float)
    p.add_argument('--eval-seed-base',type=int);p.add_argument('--eval-seeds',type=int,default=8)
    args=p.parse_args();torch.set_num_threads(1);torch.manual_seed(args.seed);torch.cuda.manual_seed_all(args.seed)
    args.out.mkdir(parents=True,exist_ok=False);cfg=json.loads(args.config.read_text())
    if args.eval_seed_base is None:
        args.eval_seed_base = cfg.get('seed_ranges', {}).get('development', [615000])[0]
    policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cuda')
    policy.load_state_dict(torch.load(args.init,map_location='cuda',weights_only=True)['model'])
    if args.initial_std:
        with torch.no_grad():policy.log_std.fill_(torch.log(torch.tensor(args.initial_std,device='cuda')))
    frozen=SamplePolicy31(verified_artifact('model_3050.pt'),'cuda')
    frozen.load_state_dict(torch.load(args.balance,map_location='cuda',weights_only=True)['model'])
    optimizer=RealDataPPO(policy,actor_lr=args.actor_lr,critic_lr=3e-4)
    env=WarpSupportedGetup(frozen.actor,args.envs,config=cfg,reset_range=args.reset_range,
                          settle_seconds=args.settle_seconds,domain=args.domain)
    files=['tools/realdata_gpu_getup_train.py','src/fifty_mujoco/envs/realdata_warp_getup.py',
           'src/fifty_mujoco/envs/realdata_warp.py','src/fifty_mujoco/learning/realdata_ppo.py',
           'src/fifty_mujoco/envs/sample_env31_supported.py','src/fifty_mujoco/envs/sample_env31_realdata.py',
           'tools/realdata_evaluate.py','src/fifty_mujoco/contracts/getup31.py']
    contract={'schema':'realdata_gpu_supported_getup31_v1','configuration':cfg,
              'balance_checkpoint':str(args.balance.resolve()),'balance_sha256':digest(args.balance),
              'initial_checkpoint_sha256':digest(args.init),'reset_range':args.reset_range,
              'settle_seconds':args.settle_seconds,'domain':args.domain,'source_sha256':{n:digest(PROJECT/n) for n in files},
              'seed':args.seed,'envs':args.envs,'actor_lr':args.actor_lr,
              'learner_phases':['GETUP','HANDOFF'],'excluded_phases':['STOP','BALANCE'],
              'balance_transition':'coalesce frozen 10 s outcome onto final learner transition',
              'physics_backend':'GPU MuJoCo Warp; CPU C-observer evaluation required',
              'critic':'31D task-specific critic, no privileged actor input'}
    for name in files:
        target=args.out/'source'/name;target.parent.mkdir(exist_ok=True,parents=True);shutil.copy2(PROJECT/name,target)
    (args.out/'contract.json').write_text(json.dumps(contract,indent=2))
    update=transitions=physical_ticks=0;stopped=False;start=time.monotonic()
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def save(name):
        path=args.out/name
        save_checkpoint(path,{'schema':'realdata_gpu_supported_getup31_v1','contract':contract,'model':policy.state_dict(),
                'actor_optimizer':optimizer.actor_optimizer.state_dict(),'critic_optimizer':optimizer.critic_optimizer.state_dict(),
                'torch_rng':torch.get_rng_state(),'cuda_rng':torch.cuda.get_rng_state_all(),
                'update':update,'transitions':transitions,'physical_policy_ticks':physical_ticks})
        return path
    evaluator=None;eval_log=None
    def start_evaluation(path):
        nonlocal evaluator,eval_log
        if evaluator and evaluator.poll() is None:return
        if eval_log:eval_log.close()
        eval_log=(args.out/f'cpu_development_{update:06d}.log').open('w')
        command=[sys.executable,'-u',str(PROJECT/'tools/realdata_evaluate.py'),'--checkpoint',str(path.resolve()),
                 '--out',str((args.out/f'cpu_development_{update:06d}.json').resolve()),'--task','getup',
                 '--balance-checkpoint',str(args.balance.resolve()),'--reset-range',*map(str,args.reset_range),
                 '--settle-seconds',str(args.settle_seconds),'--seeds',str(args.eval_seeds),'--seed-base',str(args.eval_seed_base)]
        if args.domain:command.append('--domain')
        evaluator=subprocess.Popen(command,stdout=eval_log,stderr=subprocess.STDOUT)
    start_evaluation(save('update_000000.pt'))
    T=math.ceil((cfg['success']['getup_timeout_s']+cfg['success']['blend_s'])/.01)+20
    max_ticks=math.ceil((args.settle_seconds+cfg['success']['getup_timeout_s']
                         +cfg['success']['blend_s']+cfg['success']['balance_s']+1.)/.01)
    N=args.envs;device='cuda';indices=torch.arange(N,device=device)
    try:
        with (args.out/'rows.jsonl').open('a') as log:
            for _ in range(args.updates):
                if stopped or time.monotonic()-start>=args.seconds:break
                env.reset();count=torch.zeros(N,device=device,dtype=torch.long)
                shapes={'observation':(31,),'action':(6,),'mean':(6,),'std':(6,),
                        'log_prob':(),'value':(),'reward':(),'next_value':()}
                rollout={k:torch.zeros((T,N,*shape),device=device) for k,shape in shapes.items()}
                rollout['terminated']=torch.ones((T,N),device=device,dtype=torch.bool)
                rollout['truncated']=torch.zeros((T,N),device=device,dtype=torch.bool)
                sample_start=time.monotonic();steps=0
                with torch.no_grad():
                    for step in range(max_ticks):
                        before=env.phase.clone();active=(before==1)|(before==2)
                        observation=env.observation.clone();distribution=policy.distribution(observation)
                        action=distribution.sample();value=policy.value(observation);lp=distribution.log_prob(action).sum(-1)
                        env.step(action)
                        next_value=policy.value(env.observation)
                        # All index operations have fixed N shape; no per-world host calls.
                        index=count.clamp(max=T-1)
                        values={'observation':observation,'action':action,'mean':distribution.mean,'std':distribution.stddev,
                                'log_prob':lp,'value':value,'reward':env.reward,'next_value':next_value}
                        for name,item in values.items():
                            mask=active[:,None] if item.ndim==2 else active
                            old=rollout[name][index,indices]
                            rollout[name][index,indices]=torch.where(mask,item,old)
                        rollout['terminated'][index,indices]=torch.where(active,False,rollout['terminated'][index,indices])
                        count+=active.long();steps=step+1
                        if step%25==0 and bool((env.phase>=4).all()):break
                unfinished=env.phase<4
                env.first_failure[unfinished]=14
                env.phase[unfinished]=4
                # Coalesce frozen validation reward and terminal flag onto the
                # last learner action, never onto a STOP or BALANCE sample.
                last=(count-1).clamp(min=0,max=T-1);valid_world=count>0
                bonus=torch.where(env.phase==5,10.,-5.)
                bonus+=cfg.get('balance_duration_reward_per_s',0.)*env.balance_time
                rollout['reward'][last,indices]+=torch.where(valid_world,bonus,0)
                rollout['terminated'][last,indices]=True
                rollout['next_value'][last,indices]=0
                rollout['valid']=torch.arange(T,device=device)[:,None]<count[None]
                if int(count.max())>=T:raise RuntimeError('getup transition buffer exceeded')
                if not bool(torch.isfinite(rollout['observation']).all()):raise FloatingPointError('nonfinite physics')
                valid_count=int(rollout['valid'].sum())
                if not valid_count:raise RuntimeError('all initial supported resets invalid; no learning data')
                sample_s=time.monotonic()-sample_start;optimization_start=time.monotonic()
                metrics=optimizer.update(rollout,warmup=update<args.warmup,epochs=4,batch_size=8192)
                torch.cuda.synchronize();optimization_s=time.monotonic()-optimization_start
                update+=1;transitions+=valid_count;physical_ticks+=N*steps
                failure_counts=Counter(FAILURES.get(int(v),str(int(v))) for v in env.first_failure.cpu().tolist())
                row={'update':update,'transitions':transitions,'physical_policy_ticks':physical_ticks,'wall_s':time.monotonic()-start,
                     'sampling_s':sample_s,'optimization_s':optimization_s,'learner_steps_per_s':valid_count/sample_s,
                     'cohort_steps':steps,'cohort_successes':int((env.phase==5).sum()),'cohort_trials':N,
                     'failures':dict(failure_counts),'minimum_balance_ticks':float(env.balance_time.min()/.01),
                     'support_peak_force_n':float(env.support_peak[:,0].max()),'support_peak_moment_nm':float(env.support_peak[:,1].max()),**metrics}
                log.write(json.dumps(row,allow_nan=False)+'\n');log.flush();print(json.dumps(row,allow_nan=False),flush=True)
                if update%args.save_every==0:
                    path=save(f'update_{update:06d}.pt')
                    if update%args.eval_every==0:start_evaluation(path)
    finally:
        save('last.pt')
        (args.out/'training_report.json').write_text(json.dumps({'update':update,'transitions':transitions,
                  'physical_policy_ticks':physical_ticks,'wall_s':time.monotonic()-start,'stopped_by_signal':stopped,
                  'pending_cpu_evaluator_pid':evaluator.pid if evaluator and evaluator.poll() is None else None},indent=2))
        if eval_log:eval_log.close()


if __name__=='__main__':main()
