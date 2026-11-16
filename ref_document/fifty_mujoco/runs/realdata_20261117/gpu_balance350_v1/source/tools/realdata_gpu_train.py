#!/usr/bin/env python3
"""Captured CUDA MuJoCo/controller/observer + CUDA PPO. CPU evaluates candidates."""
import argparse
import json
import math
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import torch

from realdata_train import PROJECT,SamplePolicy31,SamplePPO,verified_artifact,save_checkpoint,digest
from fifty_mujoco.envs.realdata_warp import RealDataWarpEnv


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--init',type=Path,required=True)
    p.add_argument('--envs',type=int,default=512);p.add_argument('--steps',type=int,default=32)
    p.add_argument('--updates',type=int,default=100000);p.add_argument('--seconds',type=float,default=1200)
    p.add_argument('--actor-lr',type=float,default=3e-6);p.add_argument('--save-every',type=int,default=50)
    p.add_argument('--eval-every',type=int,default=100);p.add_argument('--warmup',type=int,default=10)
    p.add_argument('--seed',type=int,default=515000);args=p.parse_args()
    torch.set_num_threads(1);torch.manual_seed(args.seed);torch.cuda.manual_seed_all(args.seed)
    args.out.mkdir(parents=True,exist_ok=False);cfg=json.loads(args.config.read_text())
    policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cuda')
    policy.load_state_dict(torch.load(args.init,map_location='cuda',weights_only=True)['model'])
    optimizer=SamplePPO(policy,actor_lr=args.actor_lr,critic_lr=3e-4)
    env=RealDataWarpEnv(args.envs,config=cfg,capture=True,auto_reset=True)
    files=['tools/realdata_gpu_train.py','src/fifty_mujoco/envs/realdata_warp.py',
           'src/fifty_mujoco/envs/sample_env31_realdata.py','src/fifty_mujoco/learning/sample_ppo.py',
           'tools/realdata_evaluate.py','tools/realdata_train.py']
    contract={'schema':'realdata_captured_gpu31_ppo_v1','configuration':cfg,'device':'cuda',
              'gpu_name':torch.cuda.get_device_name(),'initial_checkpoint':str(args.init),
              'initial_sha256':digest(args.init),'source_sha256':{n:digest(PROJECT/n) for n in files},
              'hyperparameters':vars(args)|{'out':str(args.out),'config':str(args.config),'init':str(args.init)},
              'limitations':['GPU/CPU contact trajectories differ; independent CPU evaluation is required.',
                             'Ideal fresh sensors; observer equations checked vs actual C on 128 samples.',
                             'Nominal physical model only in this first GPU balance stage.']}
    for name in files:
        target=args.out/'source'/name;target.parent.mkdir(exist_ok=True,parents=True);shutil.copy2(PROJECT/name,target)
    (args.out/'contract.json').write_text(json.dumps(contract,indent=2));update=transitions=0;stopped=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def save(name):
        target=args.out/name
        save_checkpoint(target,{'schema':'realdata_captured_gpu31_ppo_v1','model':policy.state_dict(),
                       'actor_optimizer':optimizer.actor_optimizer.state_dict(),'critic_optimizer':optimizer.critic_optimizer.state_dict(),
                       'torch_rng':torch.get_rng_state(),'cuda_rng':torch.cuda.get_rng_state_all(),
                       'contract':contract,'update':update,'transitions':transitions})
        return target
    names=('observation','action','log_prob','value','reward','next_value','terminated','truncated','mean','std')
    evaluation_process=None;evaluation_log=None
    def start_evaluation(checkpoint):
        nonlocal evaluation_process,evaluation_log
        if evaluation_process and evaluation_process.poll() is None:return
        if evaluation_log:evaluation_log.close()
        output=args.out/f'cpu_development_{update:06d}.json'
        evaluation_log=(args.out/f'cpu_development_{update:06d}.log').open('w')
        evaluation_process=subprocess.Popen([sys.executable,'-u',str(PROJECT/'tools/realdata_evaluate.py'),
              '--checkpoint',str(checkpoint.resolve()),'--out',str(output.resolve()),'--seeds','4',
              '--seed-base','613000','--seconds','20'],stdout=evaluation_log,stderr=subprocess.STDOUT)
    checkpoint=save('update_000000.pt');start_evaluation(checkpoint)
    start=time.monotonic()
    try:
        with (args.out/'rows.jsonl').open('a') as log:
            for _ in range(args.updates):
                if stopped or time.monotonic()-start>=args.seconds:break
                rollout={n:[] for n in names};sample_start=time.monotonic();ends=0;failures=0
                env.reset_velocity[:,:2].uniform_(-.02,.02);env.reset_velocity[:,3:6].uniform_(-.02,.02)
                with torch.no_grad():
                    for _ in range(args.steps):
                        obs=env.obs.clone();dist=policy.distribution(obs);action=dist.sample();value=policy.value(obs)
                        lp=dist.log_prob(action).sum(-1)
                        env.step(action)
                        terminal=env.terminal_obs.clone();nv=policy.value(terminal)
                        values=(obs,action,lp,value,env.reward.clone(),nv,env.terminated.clone(),env.truncated.clone(),dist.mean,dist.stddev)
                        for n,v in zip(names,values):rollout[n].append(v.detach())
                        ends+=env.truncated.sum();failures+=env.terminated.sum()
                torch.cuda.synchronize();sample_s=time.monotonic()-sample_start;optimization_start=time.monotonic()
                metrics=optimizer.update({k:torch.stack(v) for k,v in rollout.items()},warmup=update<args.warmup,epochs=4,batch_size=4096)
                torch.cuda.synchronize();optimization_s=time.monotonic()-optimization_start
                update+=1;transitions+=args.envs*args.steps
                row={'update':update,'transitions':transitions,'wall_s':time.monotonic()-start,
                     'sampling_s':sample_s,'optimization_s':optimization_s,'sampling_steps_per_s':args.envs*args.steps/sample_s,
                     'reward_mean':float(torch.stack(rollout['reward']).mean()),'completed_episodes':int(ends),'physical_failures':int(failures),**metrics}
                log.write(json.dumps(row,allow_nan=False)+'\n');log.flush();print(json.dumps(row,allow_nan=False),flush=True)
                if update%args.save_every==0:
                    checkpoint=save(f'update_{update:06d}.pt')
                    if update%args.eval_every==0:start_evaluation(checkpoint)
    finally:
        save('last.pt')
        (args.out/'training_report.json').write_text(json.dumps({'update':update,'transitions':transitions,
               'wall_s':time.monotonic()-start,'stopped_by_signal':stopped,'pending_cpu_evaluation_pid':evaluation_process.pid if evaluation_process and evaluation_process.poll() is None else None},indent=2))
        if evaluation_log:evaluation_log.close()


if __name__=='__main__':main()
