#!/usr/bin/env python3
"""Development-only command boundary scan, including the firmware request caps."""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
import hashlib,json,multiprocessing,sys,time
from pathlib import Path
import numpy as np
import torch
PROJECT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(PROJECT/'src'))
import sample_hardware_evaluate as serial
from fifty_mujoco.envs.sample_env31_hardware_guarded import GuardedHardwareEnv31
from fifty_mujoco.learning.sample_ppo import SamplePolicy31
from fifty_mujoco.transfer.sample_source import verified_artifact
ACTOR=None
COMMAND=[0.,0.,.1]


class BoundaryEnv(GuardedHardwareEnv31):
    def request(self,command):return super().request(COMMAND)


def initialize(checkpoint):
    global ACTOR
    torch.set_num_threads(1)
    p=SamplePolicy31(verified_artifact('model_3050.pt'))
    p.load_state_dict(torch.load(checkpoint,map_location='cpu',weights_only=True)['model'])
    ACTOR=p.actor.eval();serial.SampleTargetEnv31=BoundaryEnv


def run(job):
    global COMMAND
    COMMAND=[job['vx'],job['wz'],.1]
    if time.time()>job['deadline']-30:
        return {'command':COMMAND,'seed':job['seed'],'thresholds_met':False,'first_failure':'budget_skipped'}
    row=serial.episode(ACTOR,'stand',job['seed'],job['duration'],wheel_kd=job['wheel_kd'])
    return {**row,'scenario':'command_boundary','requested_command':COMMAND}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--workers',type=int,default=8)
    p.add_argument('--seeds',type=int,default=3);p.add_argument('--duration',type=float,default=40)
    p.add_argument('--wheel-kd',type=float,default=.14)
    p.add_argument('--deadline-epoch',type=float,required=True);a=p.parse_args()
    if not 1<=a.workers<=8 or not 1<=a.seeds<=5:p.error('workers1..8,seeds1..5 required')
    commands=[(0,0),(.15,0),(.3,0),(-.15,0),(-.3,0)]
    commands += [(0,w) for w in (.3,-.3,.6,-.6,1.2,-1.2,2.5,-2.5)]
    commands += [(v,w) for v in (.15,.3) for w in (1.2,-1.2,2.5,-2.5)]
    jobs=[{'vx':v,'wz':w,'seed':88000+i*10+j,'duration':a.duration,'deadline':a.deadline_epoch,'wheel_kd':a.wheel_kd}
        for i,(v,w) in enumerate(commands) for j in range(a.seeds)]
    a.out.parent.mkdir(parents=True,exist_ok=True)
    if a.out.exists():raise ValueError('use a new output file')
    rows=[];started=time.monotonic()
    with a.out.with_suffix('.rows.jsonl').open('w') as stream,ProcessPoolExecutor(max_workers=a.workers,
        mp_context=multiprocessing.get_context('spawn'),initializer=initialize,
        initargs=(str(a.checkpoint.resolve()),)) as pool:
        for f in as_completed([pool.submit(run,j) for j in jobs]):
            row=f.result();rows.append(row);stream.write(json.dumps(row)+'\n');stream.flush()
            if len(rows)%10==0:print(json.dumps({'done':len(rows),'total':len(jobs),'passed':sum(r['thresholds_met'] for r in rows)}),flush=True)
    summaries=[]
    for v,w in commands:
        current=[r for r in rows if r.get('requested_command',r.get('command'))==[v,w,.1]]
        summaries.append({'vx':v,'wz':w,'trials':len(current),'passed':sum(r['thresholds_met'] for r in current),
            'failures':[r['first_failure'] for r in current if r['first_failure']],
            'max_saturation':max(max(r.get('motor_saturation_fractions',[0])) for r in current)})
    report={'tool':'hardware-command-boundary','status':'development_scan_not_accepted_envelope',
        'checkpoint':str(a.checkpoint.resolve()),'checkpoint_sha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
        'duration_s':a.duration,'seed_scope':'development_88000','wheel_kd':a.wheel_kd,'target_knee_margin_rad':.03,'backend':'guarded',
        'wall_seconds':time.monotonic()-started,'summaries':summaries,'rows':rows,'real_robot_accepted':False}
    a.out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'out':str(a.out),'summaries':summaries}),flush=True)


if __name__=='__main__':main()
