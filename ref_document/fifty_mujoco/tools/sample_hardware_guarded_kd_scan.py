#!/usr/bin/env python3
"""Development regression of target-angle guards; never relax physical stops."""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
import json,multiprocessing,sys,time,hashlib
from pathlib import Path
import torch
PROJECT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(PROJECT/'src'))
import sample_hardware_evaluate as serial
from fifty_mujoco.envs.sample_env31_hardware_guarded import GuardedHardwareEnv31
from fifty_mujoco.learning.sample_ppo import SamplePolicy31
from fifty_mujoco.transfer.sample_source import verified_artifact
ACTOR=None;MARGIN=.03


class TrialEnv(GuardedHardwareEnv31):
    def __init__(self,*args,**kwargs):super().__init__(*args,knee_margin_rad=MARGIN,**kwargs)


def initialize(checkpoint):
    global ACTOR
    torch.set_num_threads(1);p=SamplePolicy31(verified_artifact('model_3050.pt'))
    p.load_state_dict(torch.load(checkpoint,map_location='cpu',weights_only=True)['model']);ACTOR=p.actor.eval()
    serial.SampleTargetEnv31=TrialEnv


def run(job):
    global MARGIN
    MARGIN=job['margin']
    r=serial.episode(ACTOR,job['scenario'],job['seed'],40,wheel_kd=job['kd'])
    return {**r,'target_knee_margin_rad':MARGIN}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    scenarios=[('stand',42000),('forward',42040),('backward',42080),('left',42120),
        ('right',42160),('right',42172),('backward_fast_ood',43000),('reversal',43001),('turning',43002)]
    jobs=[{'margin':.03,'kd':kd,'scenario':name,'seed':seed} for kd in [.07,.08,.09,.10,.11,.12,.14] for name,seed in scenarios]
    started=time.monotonic();rows=[];a.out.parent.mkdir(parents=True,exist_ok=True)
    with a.out.with_suffix('.rows.jsonl').open('w') as stream,ProcessPoolExecutor(max_workers=8,
        mp_context=multiprocessing.get_context('spawn'),initializer=initialize,
        initargs=(str(a.checkpoint.resolve()),)) as pool:
        for f in as_completed([pool.submit(run,j) for j in jobs]):
            r=f.result();rows.append(r);stream.write(json.dumps(r)+'\n');stream.flush()
            if len(rows)%9==0:print(json.dumps({'done':len(rows),'total':len(jobs),'passed':sum(r['thresholds_met'] for r in rows)}),flush=True)
    summary=[{'kd':kd,'margin_rad':.03,'passed':sum(r['thresholds_met'] for r in rows if r['wheel_kd']==kd),
        'trials':9,'hard_failures':sum(r['first_failure'] is not None for r in rows if r['wheel_kd']==kd)} for kd in [.07,.08,.09,.10,.11,.12,.14]]
    report={'tool':'hardware-guarded-kd-development','checkpoint_sha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
        'wall_seconds':time.monotonic()-started,'summaries':summary,'rows':rows,'physical_limits_unchanged':True,
        'firmware_modified':False,'real_robot_accepted':False}
    a.out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'summaries':summary}))


if __name__=='__main__':main()
