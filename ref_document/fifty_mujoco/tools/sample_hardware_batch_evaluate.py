#!/usr/bin/env python3
"""Parallel CPU evaluation with disjoint development/heldout seed scopes.

Uses the same episode metrics and first-failure scoring as the serial evaluator.
DM gain randomization is an uncertainty experiment, not an identified physical
probability distribution. Nothing here accepts a real robot or changes firmware.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime,timezone
import hashlib
import json
import multiprocessing
from pathlib import Path
import sys
import time
import numpy as np
import torch
PROJECT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT/'src'))
from sample_hardware_evaluate import episode
from fifty_mujoco.learning.sample_ppo import SamplePolicy31
from fifty_mujoco.transfer.sample_source import verified_artifact
from fifty_mujoco.envs.sample_env31 import wilson_interval
from fifty_mujoco.envs.sample_env31_hardware import hardware_profile

ACTOR=None
CLASSES=('stand','forward','backward','left','right')


def initialize(checkpoint, backend, knee_margin):
    if backend == "scalar":
        import sample_hardware_evaluate as serial
        from fifty_mujoco.envs.sample_env31_hardware_fast import FastHardwareEnv31
        serial.SampleTargetEnv31 = FastHardwareEnv31
    if backend == "guarded":
        import functools
        import sample_hardware_evaluate as serial
        from fifty_mujoco.envs.sample_env31_hardware_guarded import GuardedHardwareEnv31
        serial.SampleTargetEnv31 = functools.partial(GuardedHardwareEnv31, knee_margin_rad=knee_margin)
    global ACTOR
    torch.set_num_threads(1)
    policy=SamplePolicy31(verified_artifact('model_3050.pt'))
    policy.load_state_dict(torch.load(checkpoint,map_location='cpu',weights_only=True)['model'])
    ACTOR=policy.actor.eval()


def run(job):
    if time.time()>job['deadline_epoch']-30:
        return {'scenario':job['name'],'seed':job['seed'],'condition':job['condition'],
                'thresholds_met':False,'completed':False,'first_failure':'budget_skipped','metrics':{}}
    row=episode(ACTOR,job['name'],job['seed'],job['duration'],wheel_kd=job['wheel_kd'],
                reset_mode='default',**job['params'])
    return {**row,'condition':job['condition'],'sampled_parameters':job['params']}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--suite',choices=['nominal','sensitivity','dm-random'],default='nominal')
    p.add_argument('--scope',choices=['development','heldout'],default='development')
    p.add_argument('--seeds-per-class',type=int,default=40);p.add_argument('--duration',type=float,default=40)
    p.add_argument('--backend',choices=['reference','scalar','guarded'],default='scalar')
    p.add_argument('--knee-margin',type=float,default=.03)
    p.add_argument('--workers',type=int,default=4);p.add_argument('--wheel-kd',type=float,default=.1)
    p.add_argument('--deadline-epoch',type=float,required=True);p.add_argument('--extra-protocols',action='store_true')
    a=p.parse_args()
    if not 1<=a.workers<=8 or not 1<=a.seeds_per_class<=100 or not 2<a.duration<=120:
        p.error('workers1..8,seeds1..100,duration(2,120] required')
    if a.suite=='nominal' and a.seeds_per_class>40:p.error('nominal seed scope contains 40 seeds/class')
    cases={'nominal':{}}
    if a.suite=='sensitivity':
        cases.update({'dm_gain_080':{'motor_gain':[.8,.8,1.,.8,.8,1.]},
            'dm_gain_090':{'motor_gain':[.9,.9,1.,.9,.9,1.]},
            'friction_060':{'friction':.6},'friction_090':{'friction':.9},
            'spring_090':{'spring_scale':.9},'spring_110':{'spring_scale':1.1}})
    if a.suite=='dm-random':cases={'dm_random_080_100':{}}
    base=(42000 if a.scope=='development' else 52000) if a.suite=='nominal' else (86000 if a.scope=='development' else 62000)
    stride=40 if a.suite=='nominal' else 100
    jobs=[]
    for condition,params in cases.items():
        for index,name in enumerate(CLASSES):
            for j in range(a.seeds_per_class):
                seed=base+index*stride+j;sample=dict(params)
                if a.suite=='dm-random':
                    rng=np.random.default_rng(seed+7000000)
                    gains=np.ones(6)
                    if j%5!=0:gains[[0,1,3,4]]=rng.uniform(.8,1.,4)
                    sample['motor_gain']=gains.tolist()
                jobs.append({'condition':condition,'params':sample,'name':name,'seed':seed,
                    'duration':a.duration,'wheel_kd':a.wheel_kd,'deadline_epoch':a.deadline_epoch})
    if a.extra_protocols:
        for i,name in enumerate(('backward_fast_ood','reversal','turning')):
            jobs.append({'condition':'extra_protocol','params':{},'name':name,'seed':base+1000+i,
                'duration':a.duration,'wheel_kd':a.wheel_kd,'deadline_epoch':a.deadline_epoch})
    inputs=[Path(__file__),PROJECT/'tools/sample_hardware_evaluate.py',
            PROJECT/'src/fifty_mujoco/envs/sample_env31_hardware.py',PROJECT/'configs/sample_hardware_profile.json',
            PROJECT/'src/fifty_mujoco/envs/sample_env31_hardware_fast.py',
            PROJECT/'src/fifty_mujoco/envs/sample_env31_hardware_guarded.py']
    implementation={str(q.relative_to(PROJECT)):hashlib.sha256(q.read_bytes()).hexdigest() for q in inputs}
    a.out.parent.mkdir(parents=True,exist_ok=True)
    partial=a.out.with_suffix('.rows.jsonl')
    if a.out.exists() or partial.exists():raise ValueError('use a new evaluation output path')
    started=time.monotonic();rows=[]
    with partial.open('w') as stream,ProcessPoolExecutor(max_workers=a.workers,
            mp_context=multiprocessing.get_context('spawn'),initializer=initialize,
            initargs=(str(a.checkpoint.resolve()),a.backend,a.knee_margin)) as pool:
        futures=[pool.submit(run,job) for job in jobs]
        for f in as_completed(futures):
            row=f.result();rows.append(row);stream.write(json.dumps(row)+'\n');stream.flush()
            if len(rows)%10==0 or len(rows)==len(jobs):
                print(json.dumps({'done':len(rows),'total':len(jobs),'passed':sum(r['thresholds_met'] for r in rows),
                    'wall_s':time.monotonic()-started}),flush=True)
    summaries={}
    for condition in cases:
        groups={}
        for name in CLASSES:
            group=[r for r in rows if r['condition']==condition and r['scenario']==name]
            wins=sum(r['thresholds_met'] for r in group)
            groups[name]={'trials':len(group),'completed':sum(r['completed'] for r in group),
                'passed':wins,'rate':wins/len(group),'wilson_95':wilson_interval(wins,len(group))}
        current=[r for r in rows if r['condition']==condition]
        wins=sum(r['thresholds_met'] for r in current)
        summaries[condition]={'classes':groups,'total':len(current),'passed':wins,
            'rate':wins/len(current),'wilson_95':wilson_interval(wins,len(current)),
            'point_rate_thresholds_met':wins/len(current)>=.95 and all(g['rate']>=.9 for g in groups.values())}
    report={'tool':'hardware-parallel-command-evaluation','status':'reconstruction_evaluation',
        'generated_at':datetime.now(timezone.utc).isoformat(),'checkpoint':str(a.checkpoint.resolve()),
        'checkpoint_sha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
        'suite':a.suite,'backend':a.backend,'knee_margin_rad':a.knee_margin if a.backend=='guarded' else None,'seed_scope':a.scope,'seed_base':base,'duration_s':a.duration,
        'seeds_per_class':a.seeds_per_class,'workers':a.workers,'wheel_kd':a.wheel_kd,
        'wall_seconds':time.monotonic()-started,'hardware_profile':hardware_profile(),
        'implementation_sha256':implementation,'summaries':summaries,
        'rows':sorted(rows,key=lambda r:(r['condition'],r['scenario'],r['seed'])),
        'adequate_nominal_protocol':a.suite=='nominal' and a.seeds_per_class==40 and a.duration==40,
        'adequate_random_protocol':a.suite=='dm-random' and a.seeds_per_class==100 and a.duration==40,
        'all_conditions_point_rate_thresholds_met':all(s['point_rate_thresholds_met'] for s in summaries.values()),
        'real_robot_accepted':False,'original_PhysX_parity':False,
        'randomization_scope':'DM delivered motor gains only; fixed throughout each episode, 20% nominal by index' if a.suite=='dm-random' else None}
    a.out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'out':str(a.out),'summaries':summaries,'wall_seconds':report['wall_seconds']}),flush=True)


if __name__=='__main__':main()
